# Autonomy architecture: mapping + vision + self-correcting control as one system

This is the shared plan for the three agents now working in parallel:

| Agent | Session | Works on today |
|---|---|---|
| **A — Map** | "Mapping run launch" | `autopilot/explore.js`: dead reckoning, lane snapping, 📍 placement, learned speed scale |
| **B — Vision** | "Optical recognition pipeline" | `vision-core.js`, `tools/vision-synth.js`, `vision.js`: finds the track and the robot in phone-camera frames |
| **C — Control** | "Control loop video feedback integration" (branch `claude/vision-control-loop`) | `app.js`: feeding vision into the control loop so the car corrects speed, turning and trim by itself |

All three are building pieces of one loop: **see → estimate → drive → learn**. This document says who owns which piece, the interfaces between them, and the order to merge them. Where it conflicts with `CLAUDE.md`, `CLAUDE.md` wins.

---

## 0. The clock comes first

**The final is today, Fri 9 Oct, 14:30, with one attempt.** CLAUDE.md priority #1 is that this attempt must not fail. So:

- **13:55: freeze.** No push to `main` after this unless the user asks for it on purpose. Anything not merged and tested on the robot by then stays on a branch.
- **Vision ships OFF by default.** With vision off, the app must behave exactly like it does now, which is today's lane keeper. Vision may only *add* corrections and must never be needed for the car to drive.
- The configuration for the final is the one that **measured** fastest at the booth (manual, Assist, `lane.js` or explore + vision), not the most advanced one.
- Everything below that doesn't fit before the freeze is the plan for after the final (demo for the judges, or the next event). Write it so it can be finished later, but don't let it touch the attempt.

---

## 1. Hard constraints that shape the design

1. **One page, one phone.** The BLE connection lives in the app page (`ui.html` + `app.js`), and the camera page is now its Vision tab, so vision and driving already share the page and the connection. Vision still isn't in the control loop.
2. **Camera in Bluefy is unverified.** Before more integration work, someone has to confirm on the user's iPhone that `getUserMedia` works in Bluefy *while BLE is connected*, and that the camera permission prompt doesn't fire `visibilitychange`/`pagehide`. Those events stop the car, by design. If either check fails, vision is a post-event project and A + C go back to the tap-based loop.
3. **The phone's CPU is shared with the control tick.** `tick()` runs on `setInterval(p.tickMs)` on the main thread. A 40 ms vision frame on the main thread delays motor commands. Vision must run in a **Worker** (frames sent as `ImageBitmap` or downscaled `ImageData`), or at most process one ≤160 px-wide frame per tick and skip frames when it falls behind. Gate: tick jitter p95 must not rise by more than 10 ms with vision on. Log it.
4. **Vision adds no Bluetooth traffic.** It only adds information on the phone. The BLE budget (one motor update, one query and one light command per tick) stays unchanged.
5. **The camera is hand-held and moving.** The user walks around the track. Views are oblique, the robot is often occluded or out of frame, and the track is seen partially. Vision output is **intermittent and noisy by nature**. Every consumer must treat it as an occasional, possibly wrong measurement, never as a continuous signal.
6. **No build step, no dependencies** (CLAUDE.md). Plain ES modules, Node-testable cores.

---

## 2. Layers

```
              fast (every tick, ~15 Hz)                      slow (seconds)
 robot ──?LINE/?DIST──►  L1 REFLEX  ──MS,l,r──► robot
                          lane keeper
                             ▲ target speed / steer bias
                             │
 camera ──frames──► L0 VISION ──measurements──► L2 ESTIMATOR ──pose, progress──► L3 ADAPTER
  (in Worker)        vision-core.js            pose + map             learns scale, trim,
                                                ▲                     turn gain, speed
                    📍 tap ─────────────────────┘                          │
                                                                          ▼
                                               L4 SUPERVISOR ◄── health of every layer
                                               RUN / RECOVER / HALT, watchdogs
```

| Layer | Rate | Owner | Lives in | May command motors? |
|---|---|---|---|---|
| L0 Vision | 5–15 fps, irregular | **B** | `vision-core.js` (pure), `vision.js` (camera, Worker glue) | **Never** |
| L1 Reflex | every tick | **A** (driving code), unchanged from `lane.js` | `autopilot/explore.js` → `laneKeeper()` | Yes, the only layer that does |
| L2 Estimator | every tick + on each measurement | **A** | `autopilot/explore.js` (later `autopilot/lib/estimator.js` if shared) | No |
| L3 Adapter | ~1 Hz / per lap | **C** | `autopilot/` script code, plus app.js only for plumbing | No, it only writes bounded *learned* values |
| L4 Supervisor | every tick | **C** for app-level gates (stale data, background, STOP), **A** for in-script states | app.js + script | Only to stop |

**The core rule: the line sensors keep the car on the lane, and vision makes it smarter.** L1 must keep working if L0, L2 and L3 all fail. That is what makes the system self-healing: each slower layer can drop out without taking down the faster ones.

---

## 3. Interfaces (the contracts to agree on now)

### 3.1 Coordinate frame (everyone)
- The **track frame** is `RALLY` from `sim.js`: centimetres, x to the right, y up, heading in radians, 0 = +x, counter-clockwise positive. `explore.js` already uses it, and the 📍 placement already converts screen → track cm.
- **Time** is `performance.now()` milliseconds, the same clock as `s.t` in scripts (`now()` in app.js). Vision stamps each measurement with the **frame capture time** (`requestVideoFrameCallback` metadata `expectedDisplayTime`/`captureTime` when available, otherwise the time the frame was grabbed), *not* the time processing finished.

### 3.2 Vision measurement (B produces, A consumes)

B publishes the latest measurement on the page, and C passes it into scripts:

```js
// B: vision.js sets this (a plain object, replaced, never mutated in place)
window.__rrVision = {
  v: 1,                    // contract version
  seq: 123,                // increments with every published measurement
  t: 81234.5,              // performance.now() at frame capture
  latency: 85,             // ms from capture to publish (for logs)
  robot: {                 // null when the robot was not found in this frame
    x: 412.0, y: -37.5,    // track cm
    h: 1.57,               // rad, or null if heading is unknown
    sigma: 8,              // 1-σ position error, cm (be honest; 2× too big beats 2× too small)
    hSigma: 0.3,           // rad, or null
  },
  track: {                 // null when the track couldn't be registered in this frame
    fit: 0.93,             // 0..1, how well the RALLY lane matches what's seen (lane-overlap score)
    visibleFrac: 0.4,      // share of the lap visible in this frame
  },
  conf: 0.85,              // 0..1, B's overall confidence for this frame
  note: 'ok',              // short reason when robot/track is null: 'no-robot', 'blur', 'no-track', ...
};
```

Rules for B:
- Publish *nothing* rather than a guess. A missing robot is `robot: null`, not the last known position.
- Without track registration (`track === null` or `fit < 0.6`), image coordinates can't be turned into track cm. Publish `robot: null`.
- Publish no more than 15 Hz. More doesn't help the estimator, and it costs CPU (constraint 3).
- `vision-synth.js` must be able to generate **wrong** frames on purpose: a mis-registered track (shifted by one strand of the meander), a robot look-alike, motion blur, dropouts. A uses these to test the estimator.

C adds one field to the script state in `runAutopilot()` (`app.js`):

```js
s.vision = window.__rrVision && window.__rrVision.robot && t - window.__rrVision.t < 1000 ? window.__rrVision : null;
```

Scripts must also run where `s.vision` is undefined (an old build, simrun, the sim). Treat `undefined` the same as `null`.

### 3.3 Estimator input (A)

Taps and vision should feed one function, so the 📍 tap and the camera are just two sources of the same thing:

```js
fix({ x, y, h /* or null */, sigma, hSigma, t, source: 'tap' | 'vision' })
```

- **Tap** = `sigma ≈ 25` cm, `h = null` (this matches today's "coarse hint" behavior in `cc79bac`).
- **Vision** = the sigma B reports.
- Apply it as a **weighted blend**, not a teleport: `k = σ_est² / (σ_est² + σ_meas²)`. Keep a scalar `σ_est` that grows with odometry, and with turning faster, and shrinks on each accepted fix. This is a 1-D Kalman filter per axis. That's enough here; don't build anything bigger.
- **Latency compensation:** keep a ~1 s ring buffer of `(t, x, y, h)`. When a measurement for time `t_m` arrives, compute the correction against the pose at `t_m` and apply the same delta to the current pose.
- **Gate (essential for self-healing):** reject a vision fix when the innovation is more than `3·sqrt(σ_est² + σ_meas²)`, *unless* the estimator is already `lost`. In that case accept a fix with `conf > 0.7` that sits on the lane. Log every rejection, rate-limited. This keeps one bad frame (a look-alike, or the wrong strand of the meander) from throwing the pose onto a neighbouring strand.
- Lane snapping (`SNAP_POS`, `SNAP_HEAD`) stays. It is the third source and works without a camera.

The estimator gives the rest of the script one object:

```js
mem.world = { x, y, h, sigma, lost, ti /* index into RALLY */, s /* cm along the lap */, curv /* 1/cm ahead */, lap, lapT }
```

### 3.4 Adapter outputs (C)

Self-correction means learning a small, **named, bounded** set of values from measured motion. It never means rewriting the user's sliders.

| Learned value | Learned from | Bounds | Replaces / multiplies |
|---|---|---|---|
| `speedScale` | odometry vs vision/tap distance along the track (already in A, `rrExploreScale`) | 0.3 – 3 | `VMAX` in dead reckoning |
| `trimLearned` | heading drift on straights vs commanded (vision heading or lane-snap corrections) | ±8 | added to the `trim` slider |
| `turnGain` | yaw rate measured by vision vs `(vr−vl)/WHEELBASE` | 0.5 – 2 | effective `WHEELBASE` in dead reckoning |
| `speedBySection[k]` | lane exits / edge hits per section of the lap (section = slice of `RALLY` by curvature) | `p.minSpeed+5` … **`p.apBase`** | the target speed handed to L1 |

Rules:
- **The sliders stay the user's limits.** The learned speed never goes above `p.apBase`. A learned trim is reported as "slider + learned".
- **Adjust slowly.** Change a value by at most 10 % per lap, or per 10 s on the first run. Back off after a lane exit (speed −15 % for that section) and creep up after a clean pass (+3 %). This is AIMD (additive increase, multiplicative decrease), which recovers on its own when the link slows down (CLAUDE.md: speed vs. delay is what decides it).
- **Persist** under one versioned key `rrLearn.v1` = `{ speedScale, trimLearned, turnGain, speedBySection, at }`. **Fold in the existing key**: read `rrExploreScale` once, then stop writing it. Add a "Reset learning" button and log every change: `learn: section 3 speed 52 → 44 (exit)`.
- **Speed control goes through L1.** L1 reads `target = speedBySection[section(mem.world)] ?? base` and uses it as `base`. If the estimator is `lost` or `sigma > 30`, L1 uses plain `base`. In other words, without a trusted pose it falls back to today's behavior.
- **Keep it demoable** (judges' telemetry bonus): show the learned values and the current section on the explore map overlay.

### 3.5 Supervisor states (A in the script, C in the app)

```
IDLE ──GO──► CALIBRATE ──1 lap or 60 s──► RUN ──lane exit──► RECOVER ──back in lane──► RUN
                                            │                    │
                                            └──── 3 recoveries in a row / MAX_RUN / stale data ──► HALT
```
- **CALIBRATE** drives at `min(p.apBase, 35)` and learns `speedScale`/`trimLearned`. It is skipped when `rrLearn.v1` is less than 2 h old.
- **RECOVER**, without vision: today's sweep (pivot toward the last touched edge, flip after 1.2 s, back up and retry). **With a trusted pose** (`!lost`, `sigma < 20`): pivot toward the bearing of the nearest lane point instead of guessing. This is where vision makes the car self-healing.
- **HALT** keeps today's behavior: motors 0 and a log line telling the user what to do.
- App-level gates stay **unchanged and outside the scripts**: stale `?LINE` → stop (`apTimeoutMs`), background/pagehide → `S`, STOP clears the queue and sends `S` first. Vision being stale is **never** a reason to stop. It only drops back to lane-only mode.

---

## 4. File ownership (to avoid merge collisions)

| File | Owner | Others may |
|---|---|---|
| `vision-core.js`, `tools/vision-synth.js`, `vision.js` | B | read only |
| `autopilot/explore.js` (estimator, map, supervisor-in-script, L1) | A | send A a message for changes |
| `app.js`, `ui.html`, `style.css` (vision toggle, video element, Worker start, `s.vision`, Reset learning button) | C | B hands C a `startVision(videoEl, { onMeasurement })` / `stopVision()` API, and C wires it in. B doesn't edit app.js |
| Adapter logic | C writes it as a section of `explore.js` **or** as `autopilot/lib/learn.js` once scripts can import (they can't today, since scripts are function bodies). For now, C sends A the code block, or the two agree on a clearly delimited `// ---- learn (C)` section | |
| `tools/smoke.mjs` | everyone, append-only | add checks, don't rewrite others' |
| `tools/simrun.mjs`, `sim.js` | A + C | C adds `--vision=off,good,noisy,wrong` |
| `index.html`, `autopilot/lane.js` | nobody | `lane.js` is the frozen fallback for the final |

Only C's `claude/vision-control-loop` currently uses a branch. A and B push to `main`. **Before 13:55, B and C stay on branches or keep vision dark** (not mounted unless `?vision` is in the URL), so `main` can't regress the attempt.

---

## 5. Tests that prove convergence (run before merging the layers together)

Add these to `tools/smoke.mjs` / `simrun`:

1. **No-vision parity:** with `s.vision` undefined, `explore.js` gives the same lap result in `simrun` as before the change (±5 %).
2. **Good vision:** synthetic vision at 10 Hz, σ = 5 cm, 100 ms latency, sim `VMAX` set 30 % off the script's `VMAX`. The estimator stays `!lost` for 3 laps, `speedScale` converges within 10 %, and lap time doesn't get worse.
3. **Hostile vision:** 20 % of measurements are wrong (shifted one strand, or random). The estimator never jumps strands, rejections are logged, and nothing gets worse than no-vision.
4. **Dropout:** vision for 20 s, then nothing for 60 s, then back. No stop, no lost pose for longer than it needs, and recovery when vision returns.
5. **Varying link** (`--link=varying`): AIMD speed per section keeps lane exits ≤ the no-learning baseline.
6. **CPU:** with vision on in the headless Chromium smoke, tick interval p95 stays ≤ `tickMs + 10`.
7. **Safety unchanged:** the existing smoke checks (STOP, background stop, stale-data stop) still pass with vision on.

---

## 6. Order of work

1. **Now (all):** read this, reply to the user with any disagreement on §3 in one message, then build to the contract.
2. **B:** keep proving correctness in the Vision tab. Expose `startVision`/`stopVision` and `window.__rrVision` v1. Add hostile frames to `vision-synth.js`. Do the Bluefy camera + BLE check on the phone first (§1.2).
3. **A:** refactor the 📍 path into `fix({..., sigma, source})` with gating + latency compensation, and read `s.vision`. Publish `mem.world`. Make RECOVER use the pose when it's trusted.
4. **C:** add `s.vision` plumbing (one line in `runAutopilot`), mount vision behind `?vision` / a Tune toggle, default off, and add simrun `--vision` modes. Write the adapter (§3.4) and send it to A as an `explore.js` section.
5. **Merge order:** C plumbing (no-op without vision) → A estimator (works with taps alone) → B mount via C → adapter. Run smoke + simrun after each step, and test on the robot before the next.
6. **The final:** whatever is proven on the robot before 13:55. Anything else waits for the judges' demo / after the event.

## 7. What not to do

- Don't let vision send `MS`/`S`, edit `PARAMS`, or move a slider.
- Don't add a second motor-owning loop (`setInterval` in vision.js that drives). There is exactly one `tick()`.
- Don't make the car stop or slow down because vision is missing. Missing vision means falling back to lane-only.
- Don't store learned state in more than one place, or under unversioned keys.
- Don't use `PING` time as the link delay in the auto loop (CLAUDE.md: it queues behind `?LINE`).
- Don't push to `main` after the freeze without the user asking.
