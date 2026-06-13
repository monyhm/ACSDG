# 🚀 Deploy the server (one-time, ~10 minutes)

You only need to do this **once**. It puts the budget tracker on the internet so
both iPhones can reach it from anywhere (home Wi-Fi or mobile data). At the end
you'll have two things to hand to your family members:

- a **Budget link** (the dashboard), and
- a **Webhook URL** (Budget link + `/api/sms`) + the **Secret**.

Pick **one** option below. Render is the easiest (all in a web browser).

---

## Option 1 — Render (recommended, easiest)

1. Push this repo to GitHub (it already is, if you're reading this there).
2. Go to **https://render.com**, sign up, and connect your GitHub.
3. Click **New → Blueprint**, choose this repo. Render reads
   `household-budget/render.yaml` automatically.
4. Before the first deploy, set these **environment variables** (Render will
   prompt for the ones marked "sync: false"):
   - `WEBHOOK_SECRET` → a long random string. Generate one with:
     ```bash
     python3 -c "import secrets; print(secrets.token_urlsafe(24))"
     ```
   - `BUDGET_CARDS` → the cards, as JSON (use **the last 4 digits your bank
     prints in its SMS**):
     ```json
     [{"name":"Abdulrahman","last4":"1234"},{"name":"Partner","last4":"5678"}]
     ```
   - (Optional) `BUDGET_CURRENCY` (default `SAR`), `BUDGET_DEFAULT_BUDGET`
     (e.g. `3000`), `BUDGET_CYCLE_START_DAY` (set to your salary day, e.g. `27`).
5. Click **Apply / Deploy**. After a couple of minutes you'll get a URL like
   `https://household-budget-xxxx.onrender.com`.

> The blueprint uses the **Starter** plan (~a few $/month) so the app stays
> always-on and keeps your history on a persistent disk. To just try it for
> free, change `plan: starter` → `plan: free` and remove the `disk:` block in
> `render.yaml` — but note the free plan **sleeps when idle and wipes data on
> redeploy**, so a spending SMS that arrives while it's asleep can be missed.
> For real daily use, keep Starter.

**Budget link:** `https://household-budget-xxxx.onrender.com`
**Webhook URL:** `https://household-budget-xxxx.onrender.com/api/sms`

---

## Option 2 — Fly.io (always-on, low cost)

Install the CLI from https://fly.io/docs/flyctl/install, then from the
`household-budget/` folder:

```bash
fly launch --no-deploy                  # creates the app; keep the included fly.toml
fly volumes create data --size 1        # persistent storage for the budget DB
fly secrets set WEBHOOK_SECRET="$(python3 -c 'import secrets;print(secrets.token_urlsafe(24))')"
fly secrets set BUDGET_CARDS='[{"name":"Abdulrahman","last4":"1234"},{"name":"Partner","last4":"5678"}]'
fly deploy
```

Your URL will be `https://<app-name>.fly.dev`.

---

## Option 3 — Any always-on computer / VPS (full control)

```bash
git clone <this-repo> && cd household-budget
pip install -r requirements.txt
cp config.example.json config.json      # edit cards + webhook_secret
./run.sh                                 # http://0.0.0.0:8000
```

To reach it from the iPhones over the internet, either open a port on your
router, or run a tunnel (no router changes):

```bash
# Cloudflare Tunnel (free): gives you a public https URL
cloudflared tunnel --url http://localhost:8000
```

Use the printed `https://….trycloudflare.com` URL as the Budget link.

---

## After deploying — set the budget

Open the **Budget link**, expand **Quick actions**, paste the **Secret** once,
enter your monthly amount, and tap **Save**. (Or it uses
`BUDGET_DEFAULT_BUDGET` automatically.)

## Then hand off to the family

Give both family members the **`IPHONE-SETUP.md`** sheet along with:
1. the **Budget link**, 2. the **Webhook URL**, 3. the **Secret**.

That's everything — once their iPhone Shortcut is set up, spending updates the
shared budget automatically.
