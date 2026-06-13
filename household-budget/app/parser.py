"""SMS parsing for bank spending alerts.

The goal is to turn a raw bank SMS (Arabic or English) into a structured
transaction: how much was spent, on which card, and whether it was a
purchase (money out) or a refund (money back in).

Everything that varies between banks lives in ``config.json`` so a household
can adapt the system to their bank without touching code.
"""

from __future__ import annotations

import re
from dataclasses import dataclass, asdict
from typing import Optional


# Words that tell us money LEFT the account. Arabic + English defaults that
# cover the most common Gulf / international bank wording.
DEFAULT_DEBIT_KEYWORDS = [
    "purchase", "payment", "paid", "pos", "withdrawal", "withdraw", "debit",
    "spent", "deducted",
    "شراء", "نقاط بيع", "مدفوعات", "سحب", "خصم", "دفع", "حسم",
]

# Words that tell us money CAME BACK (refund / reversal). We record these as
# negative spend so the budget self-corrects when a purchase is reversed.
DEFAULT_REFUND_KEYWORDS = [
    "refund", "reversal", "reversed", "returned", "credit",
    "استرجاع", "استرداد", "عكس", "إرجاع", "اعادة",
]

# Words that mean money came IN but is NOT household spending (salary, transfer
# in, deposit). These are ignored entirely so a salary SMS never touches the
# budget even if it lands on the same card.
DEFAULT_IGNORE_KEYWORDS = [
    "salary", "deposit", "transfer received", "received from",
    "راتب", "إيداع", "ايداع", "حوالة واردة", "اضافة مبلغ",
]

# Amount patterns. We try each in order and take the first hit. They are kept
# deliberately broad so they survive small wording changes between banks.
DEFAULT_AMOUNT_PATTERNS = [
    # "SAR 150.00", "AED 1,250.50", "USD 12.30"
    r"(?:SAR|AED|USD|EGP|KWD|BHD|QAR|OMR|JOD|ر\.?س|﷼|درهم|ريال|جنيه)\s*([0-9][0-9,]*\.?[0-9]*)",
    # "150.00 SAR", "1,250.50 ر.س", "12.30 ريال"
    r"([0-9][0-9,]*\.?[0-9]*)\s*(?:SAR|AED|USD|EGP|KWD|BHD|QAR|OMR|JOD|ر\.?س|﷼|درهم|ريال|جنيه)",
    # generic "amount: 150.00" / "بمبلغ 150.00"
    r"(?:amount|amt|بمبلغ|مبلغ)\s*:?\s*([0-9][0-9,]*\.?[0-9]*)",
]

# Card-number patterns: capture the last digits the bank prints.
DEFAULT_CARD_PATTERNS = [
    r"(?:ending(?:\s+in)?|ends with)\s*[:#]?\s*([0-9]{3,4})\b",
    r"(?:المنتهية|تنتهي|منتهية)\s*(?:بـ|ب|في)?\s*([0-9]{3,4})\b",
    r"(?:card|بطاقة|البطاقة)\s*(?:no\.?|number|رقم)?\s*[:#]?\s*[*x×\-]*\s*([0-9]{3,4})\b",
    r"[*x×]{2,}\s*([0-9]{3,4})\b",
]

# Merchant patterns (best-effort, optional).
DEFAULT_MERCHANT_PATTERNS = [
    r"(?:at|from|@|لدى|في|من)\s+([A-Za-z0-9؀-ۿ&'\-\. ]{2,40}?)(?:\s+on\b|\s+بتاريخ|\s*[,،\.]|$)",
]


@dataclass
class ParseResult:
    matched: bool                      # did we recognise this as a spend/refund?
    kind: str                          # "purchase" | "refund" | "ignored" | "unknown"
    amount: Optional[float] = None     # positive number, currency units
    currency: Optional[str] = None
    card_last4: Optional[str] = None
    person: Optional[str] = None       # resolved from the card map
    merchant: Optional[str] = None
    reason: Optional[str] = None       # why it was ignored / not matched

    def to_dict(self) -> dict:
        return asdict(self)


def _clean_amount(raw: str) -> Optional[float]:
    """Turn '1,250.50' or '1.250,50' into a float, tolerating separators."""
    if raw is None:
        return None
    s = raw.strip().rstrip(".")
    if not s:
        return None
    # If both separators present, assume the last one is the decimal point.
    if "," in s and "." in s:
        if s.rfind(",") > s.rfind("."):
            s = s.replace(".", "").replace(",", ".")
        else:
            s = s.replace(",", "")
    else:
        # Only commas: treat as thousands separators.
        s = s.replace(",", "")
    try:
        val = float(s)
    except ValueError:
        return None
    return val if val > 0 else None


def _first_match(patterns, text, flags=re.IGNORECASE):
    for pat in patterns:
        m = re.search(pat, text, flags)
        if m:
            return m
    return None


def _contains_any(text_lower: str, words) -> bool:
    return any(w.lower() in text_lower for w in words)


class SmsParser:
    """Parses bank SMS into transactions using a household config."""

    def __init__(self, config: dict):
        self.cards = config.get("cards", [])  # [{name, last4, currency?}]
        self.amount_patterns = config.get("amount_patterns") or DEFAULT_AMOUNT_PATTERNS
        self.card_patterns = config.get("card_patterns") or DEFAULT_CARD_PATTERNS
        self.merchant_patterns = config.get("merchant_patterns") or DEFAULT_MERCHANT_PATTERNS
        self.debit_keywords = config.get("debit_keywords") or DEFAULT_DEBIT_KEYWORDS
        self.refund_keywords = config.get("refund_keywords") or DEFAULT_REFUND_KEYWORDS
        self.ignore_keywords = config.get("ignore_keywords") or DEFAULT_IGNORE_KEYWORDS
        self.default_currency = config.get("default_currency", "SAR")
        # If true, only SMS whose card matches a configured card are counted.
        self.require_known_card = config.get("require_known_card", True)
        self._card_index = {
            str(c["last4"]): c for c in self.cards if c.get("last4")
        }

    def _resolve_card(self, text: str):
        """Return (last4, card_config_or_None) found in the text, if any."""
        m = _first_match(self.card_patterns, text)
        last4 = m.group(1) if m else None
        if last4 and last4 in self._card_index:
            return last4, self._card_index[last4]
        return last4, None

    def parse(self, text: str) -> ParseResult:
        if not text or not text.strip():
            return ParseResult(matched=False, kind="unknown", reason="empty message")

        text_lower = text.lower()

        # 1) Drop pure money-in / informational messages early.
        if _contains_any(text_lower, self.ignore_keywords):
            return ParseResult(matched=False, kind="ignored",
                               reason="matched ignore keyword (e.g. salary/deposit)")

        # 2) Resolve which card this SMS is about.
        last4, card = self._resolve_card(text)
        if self.require_known_card and card is None:
            return ParseResult(
                matched=False, kind="ignored", card_last4=last4,
                reason="card not in the household card list",
            )

        # 3) Extract the amount.
        amount_match = _first_match(self.amount_patterns, text)
        amount = _clean_amount(amount_match.group(1)) if amount_match else None
        if amount is None:
            return ParseResult(matched=False, kind="unknown", card_last4=last4,
                               person=card.get("name") if card else None,
                               reason="no amount found")

        # 4) Currency: from the card config, else detected, else default.
        currency = (card or {}).get("currency") or self._detect_currency(text) \
            or self.default_currency

        # 5) Debit vs refund.
        is_refund = _contains_any(text_lower, self.refund_keywords)
        is_debit = _contains_any(text_lower, self.debit_keywords)
        if is_refund and not is_debit:
            kind = "refund"
        elif is_debit:
            kind = "purchase"
        else:
            # No explicit keyword: a known-card amount SMS is almost always a
            # purchase, so default to that rather than dropping a real spend.
            kind = "purchase"

        merchant = self._detect_merchant(text)

        return ParseResult(
            matched=True,
            kind=kind,
            amount=amount,
            currency=currency,
            card_last4=last4,
            person=(card or {}).get("name"),
            merchant=merchant,
        )

    def _detect_currency(self, text: str) -> Optional[str]:
        mapping = {
            "SAR": ["sar", "ر.س", "ريال", "﷼"],
            "AED": ["aed", "درهم"],
            "USD": ["usd", "$"],
            "EGP": ["egp", "جنيه"],
            "KWD": ["kwd"], "BHD": ["bhd"], "QAR": ["qar"],
            "OMR": ["omr"], "JOD": ["jod"],
        }
        tl = text.lower()
        for code, tokens in mapping.items():
            if any(t in tl for t in tokens):
                return code
        return None

    def _detect_merchant(self, text: str) -> Optional[str]:
        m = _first_match(self.merchant_patterns, text)
        if not m:
            return None
        merchant = m.group(1).strip(" .,،-")
        return merchant or None
