# 🏠 Household Budget Tracker (from bank SMS)

A dead-simple, automated way for two people to share one household budget.

**The problem this solves:** two family members each have a card used *only*
for household spending (food, etc.). Every time they pay, the bank sends them
an SMS. But neither of them knows the running total, so they keep
double-spending and arguing about "how much is left."

**The solution:** each phone automatically forwards those bank SMS to a tiny
server. The server reads the amount and the card, adds it up, and shows a
single live page that *both* people can open any time:

> **Remaining: 2,774.50 SAR** — Budget 3,000 · Spent 225.50 (Abdulrahman 150, Partner 75.50)

No manual entry. No spreadsheets. They just spend as usual and the number
updates by itself.

```
[Person A phone] ──bank SMS──▶ ┐
                                ├─▶ SMS-forwarder app ──POST /api/sms──▶ [Server] ──▶ shared dashboard 📱
[Person B phone] ──bank SMS──▶ ┘                                          (parses amount,    (both open the
                                                                           tracks budget)     same web link)
```

---

## How it works

1. A **free phone app** (no coding) forwards the bank's spending SMS to the
   server's `/api/sms` URL.
2. The server **parses** the SMS — pulls out the amount, the card's last 4
   digits, and whether it was a purchase or a refund. Salary/deposit messages
   are ignored automatically.
3. It maps the card to the right person and **adds it to this month's budget**.
4. Both people open the **dashboard web page** and see Budget / Spent /
   Remaining update live (auto-refreshes every 15s).

Because each card is only used for household spending, *every* purchase SMS
from that card is household spending — so the tracking is automatic and
accurate, with nothing to log by hand.

---

## 📘 Ready-to-use guides

- **[DEPLOY.md](DEPLOY.md)** — one-time server setup (Render / Fly / your own
  machine). Do this first.
- **[IPHONE-SETUP.md](IPHONE-SETUP.md)** — the sheet to hand directly to each
  family member. Both have iPhones → this is the forwarding method to use.

> **Both on iPhone?** iOS can't run a background SMS-forwarder app like Android,
> but the built-in **Shortcuts → Automation** feature can auto-send each bank
> SMS for you. `IPHONE-SETUP.md` walks through it tap-by-tap.

---

## Quick start (local / development)

### 1. Run the server

```bash
cd household-budget
pip install -r requirements.txt
cp config.example.json config.json     # then edit config.json (see below)
./run.sh                               # serves on http://0.0.0.0:8000
```

Open `http://<server-ip>:8000` on any phone or laptop on the network — that's
the shared dashboard.

> Want it reachable from anywhere (not just home Wi-Fi)? Run it on a small
> cloud box, or expose your local server with a tunnel like `cloudflared` /
> `ngrok`. Then both phones (and the forwarder app) use that public URL.

### 2. Configure `config.json`

```json
{
  "currency": "SAR",
  "cycle_start_day": 1,            // 1 = calendar month; set to salary day e.g. 27
  "default_budget": 3000,          // can also be changed from the dashboard
  "webhook_secret": "a-long-random-string",
  "require_known_card": true,      // only count SMS from the cards below
  "cards": [
    { "name": "Abdulrahman", "last4": "1234", "currency": "SAR" },
    { "name": "Partner",     "last4": "5678", "currency": "SAR" }
  ]
}
```

- **`last4`** — the last 4 digits the bank prints in its SMS (e.g. *"card
  ending 1234"*). This is how each spend is attributed to the right person.
- **`webhook_secret`** — a password the forwarder app must send so randoms
  can't post fake spends. Leave it `""` only on a trusted local network.
- **`cycle_start_day`** — budget pay-cheque to pay-cheque by setting this to
  your salary day instead of the 1st.

### 3. Set up the SMS forwarder on each phone (one-time, no coding)

**Android** (recommended — full SMS access):

- Install a forwarder app such as **"SMS Forwarder"** (by Hilmy/others) or
  **"SMS to URL Forwarder"** / use **Tasker** or **MacroDroid**.
- Add a rule:
  - **Trigger:** SMS received from your bank's sender ID (e.g. your bank's
    name/number). You can also forward all SMS — non-bank messages are simply
    ignored by the server.
  - **Action:** HTTP **POST** to `http://<server>:8000/api/sms`
  - **Body (JSON):**
    ```json
    { "text": "{{message}}", "sender": "{{from}}", "secret": "a-long-random-string" }
    ```
    (Use whatever placeholder the app provides for the message text — most use
    `%text%`, `{{message}}`, or `%sms_body%`.)

**iPhone:** use **Shortcuts → Automation** (a "Message" trigger that runs a
"Get Contents of URL" POST). Full tap-by-tap steps are in **[IPHONE-SETUP.md](IPHONE-SETUP.md)**
— hand that file to each family member.

The server accepts JSON, form-encoded, or query-string posts, and recognises
common field names (`text`/`message`/`body`/`msg`, `from`/`sender`,
`secret`/`token`/`key`), so almost any forwarder app works out of the box.

---

## Using the dashboard

- **Big number = what's left** this period. The bar turns amber at 80% and red
  when over budget.
- **By person** shows each spender's running total.
- **Recent** lists each transaction; tap **×** to remove a mistaken one.
- **Quick actions** (tap to expand): change the monthly budget, or add a
  **cash spend** that didn't come through a card. If your server uses a secret,
  paste it once here to enable edits.

---

## Adapting to your bank

The parser already understands common Arabic + English bank wording (SAR, AED,
USD, EGP, and more). If your bank's SMS format is unusual, add custom regex to
`config.json` — `amount_patterns`, `card_patterns`, `debit_keywords`,
`refund_keywords`, `ignore_keywords` all override the defaults. Send a sample
SMS to `/api/sms` and check the `parsed` field in the JSON response to see
exactly what was extracted.

---

## API reference

| Method | Path                      | Purpose                                  |
|--------|---------------------------|------------------------------------------|
| POST   | `/api/sms`                | Receive a forwarded SMS (the webhook)    |
| GET    | `/api/summary`            | Budget / spent / remaining / by person   |
| GET    | `/api/transactions`       | Recent transactions                      |
| POST   | `/api/budget`             | Set the budget for the current period    |
| POST   | `/api/transactions`       | Add a manual cash spend or refund        |
| DELETE | `/api/transactions/{id}`  | Remove a transaction                     |
| GET    | `/api/health`             | Liveness + how many cards are configured  |

Write endpoints require the `secret` (body field, `?secret=`, or `X-Secret`
header) when one is configured.

---

## Privacy & safety

- Everything runs on **your own** server; data lives in a local SQLite file
  (`budget.db`). Nothing is sent to any third party.
- `config.json` and `budget.db` are git-ignored so your secret and card digits
  are never committed.
- Only the **last 4 digits** of a card are ever used or stored — never a full
  card number.
- Always set a `webhook_secret` if the server is reachable from the internet.

---

## Tests

```bash
pip install pytest
python -m pytest tests/ -q
```
