# Roborace Pit 🏁

An iPhone controller for the [Next App Robot Rally](https://github.com/droidconHQ/CuteBotDriver/blob/main/CONTEST.md) Cutebot. It needs no laptop, no Xcode and no build step.

It is a static web app using **Web Bluetooth**. Safari on iOS doesn't support Web Bluetooth, so you open the page in the free **Bluefy** browser from the App Store. Any push to the repo is live on your phone within about a minute.

## One-time setup (all from the iPhone)

1. **Install "Bluefy – Web BLE Browser"** from the App Store. "WebBLE" also works as a backup.
2. **Host the repo.** Pick one:
   - **GitHub Pages.** It needs a public repo or a paid GitHub plan. First make the repo public: github.com → repo → Settings → General → Danger Zone → Change visibility. Then go to Settings → Pages → *Deploy from a branch*, choose `main` and `/ (root)`, and save. The URL is `https://laevand.github.io/berlin-roborace/`.
   - **Netlify** (free, and the repo can stay private). Go to app.netlify.com → Add new site → Import from Git → GitHub and pick this repo and branch. Leave the build command empty, set the publish directory to `.`, and deploy.
3. Open the URL **in Bluefy**, tap **Connect** and pick `BBC micro:bit [xxxxx]`. Put the robot's 5-letter ID in **Tune → Robot ID** so the picker only shows your car. This matters at a busy booth.

You can try it without a robot first: open `…/?demo` in any browser to get a simulated car on a line track.

## The adjust loops, fastest first

| Loop | How | Time |
| --- | --- | --- |
| Tuning | **Tune** tab sliders: speed, deadband, trim, steering feel, autopilot speeds. Changes apply live while you drive and are saved on the phone. | 0 s |
| Autopilot logic | **Pilot** tab: edit the JS in place and tap **Apply**. It hot-swaps without disconnecting. | ~10 s |
| Agent-written autopilot | Ask Claude Code in the phone app to change `autopilot/*.js`. It pushes, then you tap **Pull from repo** in the Pilot tab. The connection stays up. | ~1 min |
| Agent-written app change | Claude pushes and a **⬆ New build** button appears in the header. Tap it, then tap **Connect** again. | ~1–2 min |

## Driving

- **Manual:** left thumb steers and right thumb is the throttle. Both are floating joysticks, so put your thumb down anywhere. Lifting your thumb stops the car. You can switch to **Tilt** steering (zeroed at the angle where you turn it on) or the single-stick layout in Tune. A Bluetooth gamepad also works: left stick steers, RT/LT for throttle and reverse, A = horn, B = stop, Start = GO.
- **Assist:** you hold the throttle and the autopilot steers from the line sensors. Steering hard yourself overrides it.
- **Auto:** tap **GO** and the active Pilot script drives. The car stops automatically if sensor data goes stale. **STOP** always wins.
- **FX** (contest bonus): amber turn signals that blink on the headlights, a red underglow brake light when slowing or reversing, and underglow that shifts blue → purple → pink with speed, matching the track. Horn, team name on the LED matrix, a lap timer and a live telemetry dashboard (line sensors, distance, ping, accelerometer, light and temperature) are also included.

## Race-day checklist

1. **Ask the organizers** whether a web app running in Bluefy counts as your "mobile app". The rules say any framework, but confirm before Friday.
2. **Fit fresh AAA batteries.** If the robot reboots (sad face, then happy face) when you floor it, the batteries are low. **Tune → Acceleration limit** of 40–80 softens the current spike.
3. **Set trim on a straight.** If the car drifts right, lower *Trim*. If it drifts left, raise it.
4. **Find out what the track looks like to the sensors.** In Auto mode, select **Sensor probe**, press GO (the motors stay off) and push the car across the pink, purple and blue lane, the white edges and the black mat by hand. The Log tab shows what each surface reads as. Then choose *Line follower*, *Edge follower* or *Invert sensors* to match.
5. **Back up your settings:** Tune → Copy settings, then paste them into a note.

## Firmware gotchas (from `microbitapi.js`)

- `HORN`, `BEEP`, `TONE`, `ICON` (~0.6 s) and `DISP` (several seconds) **block the robot's command loop**. While they run, motor commands queue up. Use them only while stopped.
- `?DIST` can block for up to ~30 ms. The autopilot doesn't read distance unless you enable it.
- Each BLE write is at most 20 bytes. The app splits longer commands automatically.
- A disconnect makes the firmware stop the motors. The app tries to reconnect on its own.

## Files

| File | What |
| --- | --- |
| `index.html` | Stable loader shell that cache-busts everything else. Don't edit it. |
| `ui.html`, `style.css` | Markup and styling. |
| `app.js` | BLE link, command queue, control loop, inputs, lights, telemetry, autopilot engine, Tune form. |
| `sim.js` | Demo simulator, a fake robot speaking the same protocol. |
| `autopilot/*.js` | Autopilot scripts, listed in `autopilot/index.json`. |
| `tools/smoke.mjs` | Headless test. Run `node tools/smoke.mjs` before pushing. |
