"""FastAPI app: SMS webhook + shared budget dashboard API.

Run with:  uvicorn app.main:app --host 0.0.0.0 --port 8000
"""

from __future__ import annotations

import os
from datetime import datetime
from pathlib import Path

from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel

from .config import load_config, current_period
from .db import Database
from .parser import SmsParser

BASE_DIR = Path(__file__).resolve().parent.parent
STATIC_DIR = BASE_DIR / "static"

config = load_config()
db = Database(os.environ.get("BUDGET_DB", str(BASE_DIR / "budget.db")))
parser = SmsParser(config)

app = FastAPI(title="Household Budget Tracker", version="1.0.0")


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
def _period(period: str | None = None) -> str:
    return period or current_period(config.get("cycle_start_day", 1))


def _check_secret(provided: str | None):
    secret = config.get("webhook_secret") or ""
    if secret and provided != secret:
        raise HTTPException(status_code=401, detail="invalid or missing secret")


def _summary(period: str) -> dict:
    net_spent, by_person = db.period_totals(period)
    budget = db.get_budget(period) or 0.0
    remaining = budget - net_spent
    pct = (net_spent / budget * 100.0) if budget > 0 else 0.0
    return {
        "period": period,
        "currency": config.get("currency", "SAR"),
        "budget": round(budget, 2),
        "spent": round(net_spent, 2),
        "remaining": round(remaining, 2),
        "percent_used": round(pct, 1),
        "by_person": {k: round(v, 2) for k, v in sorted(by_person.items())},
        "cycle_start_day": config.get("cycle_start_day", 1),
    }


# ---------------------------------------------------------------------------
# Models
# ---------------------------------------------------------------------------
class SmsIn(BaseModel):
    text: str
    sender: str | None = None
    secret: str | None = None
    received_at: str | None = None


class BudgetIn(BaseModel):
    amount: float
    period: str | None = None
    make_default: bool = False
    secret: str | None = None


class ManualTxIn(BaseModel):
    amount: float
    person: str | None = None
    merchant: str | None = None
    note: str | None = None
    kind: str = "manual"  # manual | refund
    period: str | None = None
    secret: str | None = None


# ---------------------------------------------------------------------------
# SMS webhook — this is what the phone forwarder app posts to
# ---------------------------------------------------------------------------
@app.post("/api/sms")
async def receive_sms(request: Request):
    """Accept a forwarded SMS.

    Tolerant of how forwarder apps send data: JSON body, form fields, or query
    params, with flexible field names (text/message/body/msg, from/sender)."""
    text = sender = secret = received_at = None

    ctype = request.headers.get("content-type", "")
    if "application/json" in ctype:
        data = await request.json()
    else:
        form = await request.form()
        data = dict(form) if form else {}
    # merge query params as a fallback
    for k, v in request.query_params.items():
        data.setdefault(k, v)

    def pick(*names):
        for n in names:
            if data.get(n) not in (None, ""):
                return data.get(n)
        return None

    text = pick("text", "message", "body", "msg", "content")
    sender = pick("sender", "from", "address", "originator")
    secret = pick("secret", "token", "key") or request.headers.get("x-secret")
    received_at = pick("received_at", "timestamp", "date")

    _check_secret(secret)

    if not text:
        raise HTTPException(status_code=400, detail="no message text found")

    result = parser.parse(text)
    if not result.matched:
        # We still return 200 so the forwarder app doesn't keep retrying; the
        # message simply wasn't a household spend.
        return JSONResponse({"recorded": False, "parsed": result.to_dict()})

    period = _period()
    ts = received_at or datetime.utcnow().isoformat()
    tx_id = db.add_transaction(
        period=period,
        person=result.person,
        card_last4=result.card_last4,
        amount=result.amount,
        currency=result.currency,
        kind=result.kind,
        merchant=result.merchant,
        raw_text=text,
        source="sms",
        ts=ts,
    )
    return {"recorded": True, "id": tx_id, "parsed": result.to_dict(),
            "summary": _summary(period)}


# ---------------------------------------------------------------------------
# Dashboard API
# ---------------------------------------------------------------------------
@app.get("/api/summary")
def get_summary(period: str | None = None):
    return _summary(_period(period))


@app.get("/api/transactions")
def get_transactions(period: str | None = None, limit: int = 100):
    return {"transactions": db.list_transactions(_period(period), limit)}


@app.post("/api/budget")
def set_budget(body: BudgetIn):
    _check_secret(body.secret)
    period = _period(body.period)
    db.set_budget(period, body.amount)
    if body.make_default:
        db.set_setting("default_budget", body.amount)
    return _summary(period)


@app.post("/api/transactions")
def add_manual(body: ManualTxIn):
    _check_secret(body.secret)
    period = _period(body.period)
    kind = body.kind if body.kind in ("manual", "refund") else "manual"
    tx_id = db.add_transaction(
        period=period,
        person=body.person,
        card_last4=None,
        amount=abs(body.amount),
        currency=config.get("currency", "SAR"),
        kind=kind,
        merchant=body.merchant,
        note=body.note,
        source="manual",
    )
    return {"id": tx_id, "summary": _summary(period)}


@app.delete("/api/transactions/{tx_id}")
def delete_tx(tx_id: int, secret: str | None = None):
    _check_secret(secret)
    if not db.delete_transaction(tx_id):
        raise HTTPException(status_code=404, detail="transaction not found")
    return {"deleted": tx_id, "summary": _summary(_period())}


@app.get("/api/health")
def health():
    return {"ok": True, "period": _period(),
            "cards_configured": len(config.get("cards", []))}


# ---------------------------------------------------------------------------
# Static dashboard
# ---------------------------------------------------------------------------
@app.get("/")
def index():
    return FileResponse(STATIC_DIR / "index.html")


if STATIC_DIR.exists():
    app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")
