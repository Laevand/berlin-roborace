# Vision feed → control loop

The app takes the car's pose from a camera pipeline and uses it three ways:

1. **Self-calibration.** It compares what the car actually did with the motor commands it was sent, then fits trim, speed at motor 100, motor deadband, effective wheelbase and command → motion delay. The results are *learned values*, stored in `rrLearn.v1` and applied on top of your sliders, which never move (Tune → Vision).
2. **`s.vis` for autopilot scripts.** This is the pose predicted to the moment the next command lands, plus lane error, the bend ahead and a look-ahead point on the lane center. The **Vision pilot** (`autopilot/vision-pilot.js`) steers with it.
3. **Laps.** The camera taps **Lap** when the car crosses the start of the track outline.

The code is in `adapt.js` (pure logic, no DOM) and in the "vision feedback" section of `app.js`. It is layer L3 (adapter) of [docs/AUTONOMY-ARCHITECTURE.md](docs/AUTONOMY-ARCHITECTURE.md). The camera itself (`vision-core.js`, `vision.html`) is a separate piece.

## Coordinates

- Use one frame for the poses and the track, in **centimetres**: x to the right, y up, heading `h` in radians counter-clockwise from +x.
- Image-style coordinates are fine too. Add `"yDown": true` to flip y and the heading, `"hdeg"` instead of `"h"` for degrees, and `"scale"` (e.g. cm per pixel) to multiply x, y and the track width.
- The heading may be 180° off (a symmetric outline). The app fixes it from the direction the car moves. If there is no heading at all, it uses the direction of motion while driving. Calibration samples need a heading, so send one if you have it.

## The in-page camera: `window.__rrVision`

When the camera runs inside the app page, it publishes the v1 measurement from docs/AUTONOMY-ARCHITECTURE.md §3.2 as `window.__rrVision`, with a new object per frame. The control tick picks it up: `t` is the capture time on `performance.now()`, `robot` is in RALLY track cm, and `robot: null` means "not seen". A `robot.sigma` above 15 cm moves the pose but is not used for calibration, and an `hSigma` above 0.5 rad counts as no heading. The track outline is `RALLY` from `sim.js`, loaded with the first frame. Scripts also get the raw frame as `s.vision` (null when older than 1 s).

## Messages from elsewhere

For a camera that doesn't run in the app page, use messages. Each message is a JSON object, or JSON text. An array of messages also works.

```json
{"type": "pose", "x": 12.3, "y": -40.1, "h": 1.57, "t": 1791543801074, "conf": 0.9, "id": "tupaz"}
```

- `t`: the capture time of the frame, in ms since the epoch (`Date.now()` on the device that took it). Alternatively, `age` is the ms between capture and sending. If a frame has neither, the app assumes Tune → Vision → *Camera delay when the feed has no timestamps*. Late frames are fine: the app predicts forward with the commands it sent since.
- `conf` (0–1, default 1): frames below 0.5 move the pose but are not used for calibration.
- `id` (optional): the robot ID, as in Tune → Robot ID.

When the camera sees several robots, send them in one message:

```json
{"type": "poses", "t": 1791543801074, "robots": [{"id": "tupaz", "x": 12, "y": -40, "h": 1.5}, {"id": "other", "x": 80, "y": 10, "h": 0}]}
```

The app picks ours by `id` (Tune → Robot ID). If no ID matches, it takes the only robot, or else the one nearest the last pose.

**Track outline.** Send it once, and again whenever it changes:

```json
{"type": "track", "center": [[x, y], [x, y], ...], "width": 20, "closed": true}
```

`center` is the lane center line, a point every few cm, in driving order. If the car drives the other way, the app notices from its heading. Without a track, calibration and the pose still work, but there's no lane error, no laps and no Vision pilot (it falls back to the line sensors).

Send 15–60 frames per second. Calibration uses 250 ms windows of motion, so below ~10 fps it gets few samples.

## How to send it

| From | How |
| --- | --- |
| The same page | `window.rr.vision.ingest(msg)` |
| Another page or tab on the same site (a vision debug page, an iframe) | `new BroadcastChannel('rr-vision').postMessage(msg)`, or `postMessage({ rrVision: msg }, origin)` to the controller's window |
| Another device | Tune → Vision → *Feed URL*. Use `wss://…` (one message per WebSocket frame) or an `https://…` Server-Sent Events stream (one message per `data:` line). The app is served over https, so the browser blocks plain `ws://` and `http://`. |

## Self-calibration

- **Samples:** one every ~80 ms while the camera sees the car and the motors are on. Each one is the speed and turn rate over the last 250 ms, or up to 1 s when the camera reports a large `sigma`. Samples are skipped when the motors are off (the car is parked or being carried) and on jumps faster than 3 m/s (tracking glitches). The app keeps about 40 s of driving. Manual driving with varied speeds and turns teaches it the most.
- **Fit:** every 2 s, about 3 ms. It searches delay × deadband, then fits each wheel's gain and the wheelbase by least squares, drops outliers and fits again. A value counts as *measured* only when the data pins it down. Otherwise the panel shows "(assumed)" and the slider stays put. Trim must agree between the older and newer half of the data.
- **Learned values** go in `localStorage['rrLearn.v1']`: `trimLearned` (±8, added to the Trim slider), `vmax` (cm/s at motor 100), `deadband`, `wheelbase` (cm), `delay` (ms), `turnGain` (= 9 / wheelbase, for dead reckoning in scripts) and `at`. Other keys in it belong to other scripts and are kept. Before anything is learned the defaults are trim 0, 50 cm/s, deadband 22, wheelbase 9 cm, delay 150 ms. The sliders are never written.
- **Tune → Vision → Self-calibrate:**
  - `off`: learned values are ignored, so driving uses only the sliders, exactly as before.
  - `suggest` (default): the Log says what to change, and **Apply fit** stores it.
  - `auto`: every 3 s it moves the learned values toward the fit, trim by at most *Auto: largest learned-trim change per step*.
  - **Reset learning** clears the learned values. **Forget samples** starts over, e.g. after a battery swap or on a different robot.
- The Vision panel shows the feed, the latest fit and the learned values next to the slider.

## `s.vis` in autopilot scripts

`s.vis` is `null` until the first pose arrives. After that it has these fields:

| Field | Meaning |
| --- | --- |
| `fresh` | The newest frame is younger than *Camera counts as lost after* |
| `age` | ms since the newest frame was captured |
| `x`, `y`, `h` | Pose predicted to now + *Command → motion delay*, i.e. when the command you return lands |
| `raw` | The newest pose as the camera saw it |
| `v`, `w` | Speed (cm/s) and turn rate (rad/s, + = left) from the last ~250 ms of frames |
| `e` | cm left of the lane center (+ = left), relative to the driving direction |
| `he` | Heading error against the lane direction (rad, + = pointing left of it) |
| `ahead` | `[forward, left]` cm to the lane center *Vision pilot: look-ahead* cm ahead |
| `k`, `kAhead` | Lane curvature here, and the sharpest within 2× the look-ahead (1/cm, + = bends left) |
| `s`, `len`, `half` | Distance along the outline, its length, half the lane width (cm) |
| `laps`, `hz`, `n` | Lap times (s), frames in the last second, frames so far |
| `model` | The learned car model: `vmax`, `deadband`, `wheelbase`, `delay` |

## Testing without a camera

- `?demo` sends the simulated car's pose as a camera feed would: 30 fps, 80 ms late, with a little noise (Tune → Demo → *Demo camera feed*). Set *Demo right wheel stronger by* to 5 % and Self-calibrate to `auto`, then watch the learned trim go up in the Vision panel.
- `node tools/simrun.mjs autopilot/vision-pilot.js --cam=80 --apBase=60,80 --skew=4 --cal` runs the same thing headless and prints the fit of each run.
