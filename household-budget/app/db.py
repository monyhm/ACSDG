"""Tiny SQLite persistence layer (no ORM, stdlib only).

Stores transactions and per-period budgets. Money is stored in the smallest
useful unit as a float of currency units; for a household tracker the
precision of float is more than enough.
"""

from __future__ import annotations

import sqlite3
import threading
from datetime import datetime
from typing import Optional

_LOCK = threading.Lock()


SCHEMA = """
CREATE TABLE IF NOT EXISTS transactions (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    ts          TEXT NOT NULL,          -- ISO timestamp the spend happened/recorded
    period      TEXT NOT NULL,          -- budget period label, e.g. 2026-06
    person      TEXT,                   -- who spent (resolved from card)
    card_last4  TEXT,
    amount      REAL NOT NULL,          -- always positive
    currency    TEXT,
    kind        TEXT NOT NULL,          -- purchase | refund | manual
    merchant    TEXT,
    note        TEXT,
    raw_text    TEXT,                   -- original SMS, for auditing
    source      TEXT,                   -- sms | manual
    created_at  TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS budgets (
    period      TEXT PRIMARY KEY,       -- e.g. 2026-06
    amount      REAL NOT NULL,
    updated_at  TEXT NOT NULL
);

CREATE TABLE IF NOT EXISTS settings (
    key         TEXT PRIMARY KEY,
    value       TEXT
);
"""


class Database:
    def __init__(self, path: str):
        self.path = path
        self._conn = sqlite3.connect(path, check_same_thread=False)
        self._conn.row_factory = sqlite3.Row
        self._conn.executescript(SCHEMA)
        self._conn.commit()

    # ---- transactions -------------------------------------------------
    def add_transaction(self, *, period, person, card_last4, amount, currency,
                        kind, merchant=None, note=None, raw_text=None,
                        source="sms", ts=None) -> int:
        now = datetime.utcnow().isoformat()
        ts = ts or now
        with _LOCK:
            cur = self._conn.execute(
                """INSERT INTO transactions
                   (ts, period, person, card_last4, amount, currency, kind,
                    merchant, note, raw_text, source, created_at)
                   VALUES (?,?,?,?,?,?,?,?,?,?,?,?)""",
                (ts, period, person, card_last4, amount, currency, kind,
                 merchant, note, raw_text, source, now),
            )
            self._conn.commit()
            return cur.lastrowid

    def list_transactions(self, period: Optional[str] = None, limit: int = 200):
        q = "SELECT * FROM transactions"
        args = []
        if period:
            q += " WHERE period = ?"
            args.append(period)
        q += " ORDER BY ts DESC, id DESC LIMIT ?"
        args.append(limit)
        return [dict(r) for r in self._conn.execute(q, args).fetchall()]

    def delete_transaction(self, tx_id: int) -> bool:
        with _LOCK:
            cur = self._conn.execute("DELETE FROM transactions WHERE id = ?", (tx_id,))
            self._conn.commit()
            return cur.rowcount > 0

    def period_totals(self, period: str):
        """Return net spent and per-person breakdown for a period.

        Purchases add, refunds subtract. Returns (net_spent, by_person dict)."""
        rows = self._conn.execute(
            "SELECT person, kind, amount FROM transactions WHERE period = ?",
            (period,),
        ).fetchall()
        net = 0.0
        by_person: dict[str, float] = {}
        for r in rows:
            signed = -r["amount"] if r["kind"] == "refund" else r["amount"]
            net += signed
            name = r["person"] or "Unknown"
            by_person[name] = by_person.get(name, 0.0) + signed
        return net, by_person

    # ---- budgets ------------------------------------------------------
    def set_budget(self, period: str, amount: float):
        now = datetime.utcnow().isoformat()
        with _LOCK:
            self._conn.execute(
                """INSERT INTO budgets (period, amount, updated_at) VALUES (?,?,?)
                   ON CONFLICT(period) DO UPDATE SET amount=excluded.amount,
                   updated_at=excluded.updated_at""",
                (period, amount, now),
            )
            self._conn.commit()

    def get_budget(self, period: str) -> Optional[float]:
        row = self._conn.execute(
            "SELECT amount FROM budgets WHERE period = ?", (period,)
        ).fetchone()
        if row:
            return row["amount"]
        # fall back to a saved default budget that applies to every period
        return self.get_setting_float("default_budget")

    # ---- settings -----------------------------------------------------
    def set_setting(self, key: str, value: str):
        with _LOCK:
            self._conn.execute(
                """INSERT INTO settings (key, value) VALUES (?,?)
                   ON CONFLICT(key) DO UPDATE SET value=excluded.value""",
                (key, str(value)),
            )
            self._conn.commit()

    def get_setting(self, key: str) -> Optional[str]:
        row = self._conn.execute(
            "SELECT value FROM settings WHERE key = ?", (key,)
        ).fetchone()
        return row["value"] if row else None

    def get_setting_float(self, key: str) -> Optional[float]:
        v = self.get_setting(key)
        try:
            return float(v) if v is not None else None
        except ValueError:
            return None
