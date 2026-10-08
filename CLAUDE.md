# Notes for agents working on this repo

The user drives a Cutebot robot from an iPhone (the Bluefy browser, using Web Bluetooth) and has **no laptop**. Every push to `main` is live on their phone within about a minute, so a broken push can cost them practice time at the track.

## Goal
Win the Next App Robot Rally ([CONTEST.md](https://github.com/droidconHQ/CuteBotDriver/blob/main/CONTEST.md)). The lowest **adjusted lap time** wins: raw lap time minus up to 30 s of bonuses the judges award.
- **Finals: Friday 14:30 (Fri 9 Oct 2026), with ONE timed attempt.** Wednesday to Friday morning is practice on shared robots at the booth. Robots never leave the booth, so testing only happens there. Away from it, use the `?demo` simulator.
- Priorities, in this order:
  1. **The one attempt must not fail.** It must connect reliably, never lose control and never brown out. From Friday morning on, make only small, well-tested changes. When in doubt, don't push.
  2. **Raw lap time.** Use whichever is faster on the real track: manual driving with a good feel, Assist (the user holds the throttle while the line sensors steer), or full Auto. Decide from measurements, not guesses.
  3. **Bonuses** (judge's discretion):
     - Telemetry/autonomous assist, up to −10 s. Covered by the dashboard, Assist and Auto.
     - Lights, up to −5 s. Covered by turn signals, the brake light and underglow.
     - Audio/visual, up to −5 s. Covered by the horn and team name on the LED matrix, which can only be used while stopped.
     - App innovation/UX, up to −10 s. Covered by floating joysticks, tilt steering, gamepad support, dark UI and the live tuning loop.
     - Make sure each of these is easy to *demonstrate* to the judges.
- The track is a wide pink → purple → blue lane with white edge lines on a black mat. It has an S-bend and a long loop.

## Open questions (update this list as answers come in)
- What do the line sensors read on each surface? The user runs the "Sensor probe" script and reports from the Log tab.
- What line-loop rate (Line Hz) and ping do we get on real BLE through Bluefy? This decides how fast the autopilot can go.
- Do the organizers accept a web app in Bluefy as the entry? If not, the fallback is a native WKWebView shell with a CoreBluetooth bridge, built in the cloud and installed via TestFlight with the user's Apple signing assets.

## Working with the user
They are at the booth with only a phone. Keep replies short. For each change, say exactly what to tap: "Pull from repo" for `autopilot/` changes (keeps the connection), or "⬆ New build" then Connect for app changes. Ask for Log tab or dashboard screenshots as data, and prefer adding a Tune slider over another push.

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
