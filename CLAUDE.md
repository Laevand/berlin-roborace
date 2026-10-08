# Notes for agents working on this repo

The user drives a Cutebot robot from an iPhone (the Bluefy browser, using Web Bluetooth) and has **no laptop**. Every push to `main` is live on their phone within about a minute, so a broken push can cost them practice time at the track.

## Rules
- **Run `node tools/smoke.mjs` before every push.** It must print no FAIL lines. Add a check when you add behavior.
- No build step and no dependencies in the app. It is plain ES modules served as static files.
- Don't edit `index.html`. It is the cache-busting loader. All changes go in `ui.html`, `style.css`, `app.js`, `sim.js` and `autopilot/`.
- Prefer exposing a value as a slider in `PARAMS` (app.js) over hard-coding it. The user tunes live on the phone, and that is faster than any push.
- Autopilot logic belongs in `autopilot/*.js`, registered in `autopilot/index.json`. The user loads it with "Pull from repo" without losing the BLE connection. Each script is a function body `(s, p, mem, ctx) => [left, right]`. See the help text in `ui.html` and the existing scripts.
- Keep safety behavior: lifting the thumb stops the car, STOP clears the queue and sends `S` first, the car stops on background/pagehide and on stale sensor data in auto.

## Robot protocol (firmware: droidconHQ/CuteBotDriver `microbitapi.js`)
- Nordic UART. Write commands to `6e400003-…` and receive indications on `6e400002-…`. The micro:bit has the two swapped compared to Nordic's naming.
- Commands end with `#`. Replies end with `#\n`. Writes are at most 20 bytes; `Link`/`BleTransport.write` already chunks them.
- `MS,l,r` (-100..100) sets the motors. Motors below ~25 don't turn. `S` stops.
- Queries: `?LINE` → `LINE:0-3` (bit 2 = left sensor black, bit 1 = right sensor black), `?DIST`, `?ACCEL`, `?LIGHT`, `?TEMP`, `PING` → `PONG`.
- `HORN`/`BEEP`/`TONE`/`ICON`/`DISP` block the firmware's command handler, and `?DIST` blocks for up to ~30 ms. Never put them in the driving loop.
- Don't flood the link. `Link` has three queues: urgent, then the latest motor command, then a FIFO. The control loop sends at most one motor update, one query and one light command per tick.

## Deploy
GitHub Pages serves `main` at https://laevand.github.io/berlin-roborace/. The user has approved pushing straight to `main`.
Run the smoke test first. A push goes live about a minute later. On the phone, the user taps "New build" or "Pull from repo".
