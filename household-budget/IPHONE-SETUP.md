# 📱 iPhone setup — auto-send your bank SMS to the household budget

**You only do this once. It takes ~5 minutes.** After this, every time you pay
with the household card, your iPhone quietly sends the bank's text to our shared
budget — you don't have to do anything. Then you just open the budget link to
see how much is left.

> Before you start, the person who set this up will give you **3 things**:
> 1. **Budget link** (looks like `https://something.onrender.com`)
> 2. **Webhook URL** = the budget link + `/api/sms`
>    (e.g. `https://something.onrender.com/api/sms`)
> 3. **Secret** (a long password-like string)
>
> Fill them in where you see 👉 below.

---

## Part A — Build the Shortcut (the action that sends the SMS)

1. Open the **Shortcuts** app (it's built into iPhone).
2. Tap the **+** (top-right) to create a new shortcut.
3. Tap **Add Action**, search for **Get Contents of URL**, and tap it.
4. In the **URL** field, paste your **Webhook URL**:
   👉 `https://YOUR-BUDGET-LINK/api/sms`
5. Tap the small arrow **▸** next to "Get Contents of URL" to **Show More**.
   - **Method:** change to **POST**
   - **Request Body:** change to **JSON**
   - Tap **Add new field** → choose **Text**:
     - **Key:** `text`
     - **Value:** tap the value box, then tap **Shortcut Input** (a blue chip
       called "Shortcut Input" — it represents the incoming message).
   - Tap **Add new field** again → choose **Text**:
     - **Key:** `secret`
     - **Value:** type your **Secret** 👉 `YOUR-SECRET-HERE`
6. Tap the shortcut's name at the top, rename it to **Send Bank SMS**, and tap **Done**.

> 💡 If the budget later shows blank amounts, redo step 5's `text` value and make
> sure the blue **Shortcut Input** chip is selected (not typed text).

---

## Part B — Make it run automatically on every bank SMS

1. In the **Shortcuts** app, tap the **Automation** tab (bottom).
2. Tap **+** (top-right) → **Create Personal Automation**.
3. Scroll down and tap **Message**.
4. Under **Message**, tap **Message Contains** and type a word that appears in
   **every** spending text from your bank — the safest choice is your currency
   or bank name, e.g. **`SAR`** (or in Arabic, **`شراء`** or **`ريال`**).
   - Leave "Sender" as Any, or set it to your bank if you prefer.
5. Tap **Next**.
6. Tap **Run Immediately** (so it works without asking you each time), then turn
   **off** "Notify When Run" if the toggle appears.
7. Tap **Next**, then tap **Add Action** is *not* needed here — instead:
   - Search and add the action **Run Shortcut**, set it to **Send Bank SMS**
     (the shortcut you made in Part A), and pass **Shortcut Input** to it.
   - *(On newer iOS you can instead build the "Get Contents of URL" step
     directly inside the automation — either way works.)*
8. Tap **Done**.

---

## Part C — Test it

1. Make a small purchase with the household card (or wait for the next one).
2. When the bank SMS arrives, open the **Budget link** 👉 `https://YOUR-BUDGET-LINK`.
3. You should see the amount appear under **Recent** and the **Remaining**
   number go down. 🎉

If nothing shows up:
- Make sure the trigger word in Part B step 4 really appears in your bank's SMS.
- Make sure **Run Immediately** is on (Part B step 6).
- Open the **Budget link**, expand **Quick actions**, paste the **Secret** once —
  this also confirms the secret is correct.

That's it. From now on it's fully automatic — spend as normal, and both of you
can open the link any time to see exactly how much of the budget is left.
