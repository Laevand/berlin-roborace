// Vision pilot: steers from the camera while the vision feed is live (Tune → Vision, or the simulated camera in the tests),
// and drives exactly like the Lane keeper (line sensors only) whenever it isn't.
//
// Pure pursuit: aim at the lane center "look-ahead" cm ahead of where the car will be when this command
// lands (s.vis is already predicted that far), and slow down before bends so the sideways acceleration
// stays below "corner grip". Wheel speeds become motor values through the learned car model (s.vis.model:
// speed at motor 100, deadband, wheelbase), so self-calibration makes it steer and brake more exactly.
// Safety: nose off the lane (both line sensors black) for GIVE_UP_MS stops the car, camera or not. If the
// camera puts the car far outside the lane while the line sensors see the lane (a wrong registration), it
// drives by the line sensors, and after FAR_MS ignores the camera for DISTRUST_MS. A missing or wrong camera
// never stops the car by itself.
// Tune: Base speed (top speed), Vision pilot: look-ahead, Vision pilot: corner grip.

const GIVE_UP_MS = 2500;
const FAR_MS = 1000;
const DISTRUST_MS = 5000;
const base = Math.max(p.apBase, p.minSpeed + 5, 30);
const v = s.vis;

// ---- line-sensor lane keeper (same as lane.js). Runs on every reading so its memory stays current.
function laneKeeper() {
  const k = mem.lk || (mem.lk = { bias: 0, inAt: s.t });
  const gain = (p.apCurve ?? 10) / 100;
  const fade = p.apCurveDecay ?? 1500;
  const limit = (b) => Math.max(-0.8, Math.min(0.8, b));
  k.bias *= Math.exp(-(s.dt || 0) / fade); // + = steer right
  const code = (s.L ? 2 : 0) + (s.R ? 1 : 0);
  if (code !== 3) k.inAt = s.t;
  if (code === 2) k.edge = { side: 'L', t: s.t };
  if (code === 1) k.edge = { side: 'R', t: s.t };
  if (code === 3) {
    if (k.outAt !== k.inAt) {
      k.outAt = k.inAt;
      k.flipped = false;
      const recent = k.edge && s.t - k.edge.t < 400;
      k.right = recent ? k.edge.side === 'L' : k.bias < 0;
    }
    if (!k.flipped && s.t - k.inAt > 1200) { k.flipped = true; k.right = !k.right; }
    return k.right ? [base, p.apHard] : [p.apHard, base];
  }
  if (code === 2) { k.bias = limit(k.bias + gain); return [base, p.apTurn]; }
  if (code === 1) { k.bias = limit(k.bias - gain); return [p.apTurn, base]; }
  const inner = base - (base - p.minSpeed) * Math.min(1, Math.abs(k.bias) / 0.8);
  return k.bias >= 0 ? [base, inner] : [inner, base];
}
const lk = laneKeeper();

if (mem.gaveUp) return [0, 0];
if (s.t - mem.lk.inAt > GIVE_UP_MS) {
  mem.gaveUp = true;
  ctx.log(`vision pilot: nose off the lane for ${GIVE_UP_MS} ms, stopped (camera ${v && v.fresh ? 'live' : 'not live'})`);
  return [0, 0];
}

// The camera must agree with the line sensors: far outside the lane per the camera while the nose still
// sees the lane means a wrong registration, so don't steer by it, and after FAR_MS stop trusting it for a while.
// Far outside with the nose off the lane too is believable, and then the camera steers the car back.
const camOk = v && v.fresh && v.ahead && v.model && !(s.t < mem.distrustUntil);
const camWrong = camOk && Math.abs(v.e) > v.half + 25 && !(s.L && s.R);
if (camWrong) {
  mem.farAt = mem.farAt ?? s.t;
  if (s.t - mem.farAt > FAR_MS) {
    mem.distrustUntil = s.t + DISTRUST_MS;
    mem.farAt = null;
    ctx.log(`vision pilot: camera says ${Math.abs(v.e).toFixed(0)} cm off the lane center but the line sensors see the lane, ignoring the camera for ${DISTRUST_MS / 1000} s`);
  }
} else mem.farAt = null;

if (camOk && !camWrong) {
  if (mem.mode !== 'cam') { mem.mode = 'cam'; ctx.log('vision pilot: steering from the camera'); }
  const M = v.model;
  const [fx, fy] = v.ahead;
  const L2 = fx * fx + fy * fy;
  // Pure pursuit curvature (1/cm, + = left). Target behind the car: turn hard toward it.
  const kappa = fx <= 0 ? (fy >= 0 ? 0.3 : -0.3) : L2 > 1 ? (2 * fy) / L2 : 0;
  const g = M.vmax / Math.max(1, 100 - M.deadband);      // cm/s per motor unit above the deadband
  const vTop = g * (base - M.deadband);                  // Base speed, in cm/s
  const vMin = g * Math.max(5, p.minSpeed + 5 - M.deadband);
  const bend = Math.max(Math.abs(v.kAhead || 0), Math.abs(kappa), 1e-4);
  const vt = Math.max(vMin, Math.min(vTop, Math.sqrt(p.visGrip / bend)));
  const half = (kappa * M.wheelbase) / 2;
  const motor = (w) => (Math.abs(w) < 0.5 ? 0 : Math.sign(w) * Math.min(100, M.deadband + Math.abs(w) / g));
  return [motor(vt * (1 - half)), motor(vt * (1 + half))];
}

if (mem.mode !== 'line') {
  if (mem.mode === 'cam') ctx.log('vision pilot: no usable camera, steering by the line sensors');
  mem.mode = 'line';
}
return lk;
