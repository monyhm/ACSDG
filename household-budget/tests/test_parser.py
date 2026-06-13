"""Tests for the SMS parser using realistic bank-SMS wording."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from app.parser import SmsParser  # noqa: E402
from app.config import current_period  # noqa: E402
from datetime import date  # noqa: E402

CONFIG = {
    "cards": [
        {"name": "Abdulrahman", "last4": "1234", "currency": "SAR"},
        {"name": "Partner", "last4": "5678", "currency": "SAR"},
    ],
    "require_known_card": True,
}


def p():
    return SmsParser(CONFIG)


def test_english_purchase():
    r = p().parse("Purchase of SAR 150.00 at LULU MARKET with card ending 1234")
    assert r.matched and r.kind == "purchase"
    assert r.amount == 150.0
    assert r.currency == "SAR"
    assert r.person == "Abdulrahman"
    assert r.card_last4 == "1234"


def test_amount_after_currency_with_thousands():
    r = p().parse("POS purchase 1,250.50 SAR card *5678 at IKEA")
    assert r.matched and r.amount == 1250.5 and r.person == "Partner"


def test_arabic_purchase():
    sms = "شراء بمبلغ 75.50 ريال لدى تميمي بالبطاقة المنتهية 1234"
    r = p().parse(sms)
    assert r.matched and r.kind == "purchase"
    assert r.amount == 75.5
    assert r.person == "Abdulrahman"


def test_refund_is_negative_kind():
    r = p().parse("Refund of SAR 40.00 to card ending 5678 from NOON")
    assert r.matched and r.kind == "refund" and r.amount == 40.0


def test_salary_ignored():
    r = p().parse("Salary credited SAR 12,000.00 to account 1234")
    assert not r.matched and r.kind == "ignored"


def test_unknown_card_ignored():
    r = p().parse("Purchase of SAR 99.00 with card ending 9999")
    assert not r.matched and r.kind == "ignored"
    assert r.card_last4 == "9999"


def test_no_amount():
    r = p().parse("Your card ending 1234 was used today")
    assert not r.matched and r.kind == "unknown"


def test_merchant_extracted():
    r = p().parse("Purchase of SAR 20.00 at STARBUCKS on card ending 1234")
    assert r.merchant and "STARBUCKS" in r.merchant.upper()


def test_known_card_no_keyword_defaults_purchase():
    r = p().parse("SAR 33.00 card ending 1234")
    assert r.matched and r.kind == "purchase" and r.amount == 33.0


def test_allow_unknown_card_when_disabled():
    parser = SmsParser({"cards": [], "require_known_card": False})
    r = parser.parse("Purchase of SAR 10.00 at SHOP card ending 0001")
    assert r.matched and r.amount == 10.0 and r.person is None


# --- period helper ---
def test_calendar_period():
    assert current_period(1, date(2026, 6, 13)) == "2026-06"


def test_cycle_start_before_day_uses_prev_month():
    assert current_period(27, date(2026, 6, 13)) == "2026-05"


def test_cycle_start_on_or_after_day():
    assert current_period(27, date(2026, 6, 27)) == "2026-06"


def test_cycle_start_january_wraps_year():
    assert current_period(27, date(2026, 1, 5)) == "2025-12"
