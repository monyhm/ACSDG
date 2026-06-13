"""Configuration loading and the budget-period helper.

Config is read from a JSON file (path via the ``BUDGET_CONFIG`` env var,
defaulting to ``config.json`` next to the project). Missing keys fall back to
sensible defaults so the app still boots on first run.
"""

from __future__ import annotations

import json
import os
from datetime import date
from pathlib import Path

DEFAULTS = {
    "currency": "SAR",
    # Day of month the budget cycle resets. 1 = calendar month. Set to your
    # salary day (e.g. 27) to budget pay-cheque to pay-cheque.
    "cycle_start_day": 1,
    # Default monthly budget used until one is set in the dashboard.
    "default_budget": 3000,
    # Shared secret the SMS-forwarder app must send. Empty = no auth (only do
    # this on a trusted local network).
    "webhook_secret": "",
    "cards": [
        # {"name": "Abdulrahman", "last4": "1234", "currency": "SAR"},
        # {"name": "Partner",     "last4": "5678", "currency": "SAR"},
    ],
    "require_known_card": True,
}


def load_config(path: str | None = None) -> dict:
    path = path or os.environ.get("BUDGET_CONFIG", "config.json")
    cfg = dict(DEFAULTS)
    p = Path(path)
    if p.exists():
        with open(p, "r", encoding="utf-8") as f:
            user_cfg = json.load(f)
        cfg.update(user_cfg)
    _apply_env_overrides(cfg)
    return cfg


def _apply_env_overrides(cfg: dict) -> None:
    """Let environment variables override config — needed for cloud hosting,
    where there is no config.json and secrets must not be committed."""
    env = os.environ
    if env.get("WEBHOOK_SECRET"):
        cfg["webhook_secret"] = env["WEBHOOK_SECRET"]
    if env.get("BUDGET_CURRENCY"):
        cfg["currency"] = env["BUDGET_CURRENCY"]
    if env.get("BUDGET_CYCLE_START_DAY"):
        try:
            cfg["cycle_start_day"] = int(env["BUDGET_CYCLE_START_DAY"])
        except ValueError:
            pass
    if env.get("BUDGET_DEFAULT_BUDGET"):
        try:
            cfg["default_budget"] = float(env["BUDGET_DEFAULT_BUDGET"])
        except ValueError:
            pass
    if env.get("BUDGET_REQUIRE_KNOWN_CARD"):
        cfg["require_known_card"] = env["BUDGET_REQUIRE_KNOWN_CARD"].lower() in (
            "1", "true", "yes", "on")
    # BUDGET_CARDS is a JSON array, e.g.
    #   [{"name":"Abdulrahman","last4":"1234"},{"name":"Partner","last4":"5678"}]
    if env.get("BUDGET_CARDS"):
        try:
            cards = json.loads(env["BUDGET_CARDS"])
            if isinstance(cards, list):
                cfg["cards"] = cards
        except json.JSONDecodeError:
            pass


def current_period(cycle_start_day: int = 1, today: date | None = None) -> str:
    """Return the budget-period label (YYYY-MM) for a date.

    With ``cycle_start_day`` > 1, a spend before that day belongs to the cycle
    that started in the *previous* month. The label is the month the cycle
    started in.
    """
    today = today or date.today()
    cycle_start_day = max(1, min(28, int(cycle_start_day)))
    if today.day >= cycle_start_day:
        y, m = today.year, today.month
    else:
        if today.month == 1:
            y, m = today.year - 1, 12
        else:
            y, m = today.year, today.month - 1
    return f"{y:04d}-{m:02d}"
