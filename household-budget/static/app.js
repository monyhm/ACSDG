// Dashboard logic: poll the API, render the budget, and wire quick actions.
const $ = (id) => document.getElementById(id);
const REFRESH_MS = 15000;

function fmt(n) {
  return Number(n).toLocaleString(undefined, { minimumFractionDigits: 2, maximumFractionDigits: 2 });
}
function secret() { return $("secret").value.trim(); }

async function api(path, opts) {
  const res = await fetch(path, opts);
  if (!res.ok) {
    let msg = res.statusText;
    try { msg = (await res.json()).detail || msg; } catch (e) {}
    throw new Error(msg);
  }
  return res.json();
}

let lastCurrency = "";

function renderSummary(s) {
  lastCurrency = s.currency || "";
  $("period").textContent = s.period;
  $("cur1").textContent = s.currency;
  $("remaining").textContent = fmt(s.remaining);
  $("budget").textContent = fmt(s.budget) + " " + s.currency;
  $("spent").textContent = fmt(s.spent) + " " + s.currency;
  $("pct").textContent = s.percent_used + "%";

  const fill = $("barfill");
  fill.style.width = Math.min(100, Math.max(0, s.percent_used)) + "%";
  const hero = document.querySelector(".hero");
  hero.classList.toggle("over", s.percent_used >= 100);
  hero.classList.toggle("warn", s.percent_used >= 80 && s.percent_used < 100);

  // people
  const people = $("people");
  const entries = Object.entries(s.by_person || {});
  if (!entries.length) {
    people.innerHTML = '<p class="muted">No spending yet.</p>';
  } else {
    people.innerHTML = entries.map(([name, amt]) => `
      <div class="person">
        <span class="who"><span class="dot"></span>${escapeHtml(name)}</span>
        <span class="amt">${fmt(amt)} ${s.currency}</span>
      </div>`).join("");
  }
  if (!$("budgetInput").value) $("budgetInput").placeholder = "current: " + fmt(s.budget);
}

function renderTx(list) {
  const ul = $("tx");
  if (!list.length) { ul.innerHTML = '<li class="muted">No transactions yet.</li>'; return; }
  ul.innerHTML = list.map((t) => {
    const when = (t.ts || "").replace("T", " ").slice(5, 16);
    const title = t.merchant || (t.source === "manual" ? "Cash spend" : "Purchase");
    const sub = [t.person, t.card_last4 ? "••" + t.card_last4 : null, when]
      .filter(Boolean).join(" · ");
    const isRefund = t.kind === "refund";
    const sign = isRefund ? "+" : "−";
    return `<li>
      <span class="meta">
        <span class="merchant">${escapeHtml(title)}</span>
        <span class="sub">${escapeHtml(sub)}</span>
      </span>
      <span class="val ${isRefund ? "refund" : ""}">${sign}${fmt(t.amount)} ${t.currency || lastCurrency}</span>
      <button class="del" title="Remove" data-id="${t.id}">×</button>
    </li>`;
  }).join("");

  ul.querySelectorAll(".del").forEach((b) => b.onclick = () => removeTx(b.dataset.id));
}

function escapeHtml(s) {
  return String(s == null ? "" : s).replace(/[&<>"']/g, (c) =>
    ({ "&": "&amp;", "<": "&lt;", ">": "&gt;", '"': "&quot;", "'": "&#39;" }[c]));
}

async function refresh() {
  try {
    const [s, tx] = await Promise.all([
      api("/api/summary"),
      api("/api/transactions?limit=40"),
    ]);
    renderSummary(s);
    renderTx(tx.transactions);
    $("updated").textContent = "updated " + new Date().toLocaleTimeString();
  } catch (e) {
    $("updated").textContent = "offline — retrying…";
  }
}

async function removeTx(id) {
  if (!confirm("Remove this transaction?")) return;
  try {
    const q = secret() ? "?secret=" + encodeURIComponent(secret()) : "";
    await api(`/api/transactions/${id}${q}`, { method: "DELETE" });
    refresh();
  } catch (e) { msg(e.message); }
}

function msg(t) { $("actionMsg").textContent = t || ""; if (t) setTimeout(() => msg(""), 4000); }

$("saveBudget").onclick = async () => {
  const amount = parseFloat($("budgetInput").value);
  if (isNaN(amount)) return msg("Enter a number");
  try {
    await api("/api/budget", {
      method: "POST", headers: { "content-type": "application/json" },
      body: JSON.stringify({ amount, make_default: true, secret: secret() }),
    });
    $("budgetInput").value = "";
    msg("Budget saved ✓"); refresh();
  } catch (e) { msg(e.message); }
};

$("addCash").onclick = async () => {
  const amount = parseFloat($("cashAmount").value);
  if (isNaN(amount)) return msg("Enter an amount");
  try {
    await api("/api/transactions", {
      method: "POST", headers: { "content-type": "application/json" },
      body: JSON.stringify({ amount, person: $("cashWho").value || null, secret: secret() }),
    });
    $("cashAmount").value = ""; $("cashWho").value = "";
    msg("Added ✓"); refresh();
  } catch (e) { msg(e.message); }
};

refresh();
setInterval(refresh, REFRESH_MS);
document.addEventListener("visibilitychange", () => { if (!document.hidden) refresh(); });
