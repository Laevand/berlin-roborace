// Lane keeper for the rally mat: the whole lane (pink/purple/blue and its white edge lines) reads WHITE
// and the mat outside reads BLACK. Keep "Invert sensors" OFF. s.L / s.R are true when that side of the
// car's nose sees black, i.e. it is leaving the lane on that side.
//
// It steers away from whichever edge it touches, and learns the bend: every reading with the right
// sensor out adds steering bias to the left (and the other way round). Inside the lane it keeps steering
// by that bias, which fades on straights (Curve memory), so it follows curves instead of bouncing along
// the outside edge. Tune: Base speed, Inner wheel soft turn (edge touch), Inner wheel line lost (nose
// fully out), Curve learning, Curve memory.

const base = p.apBase;
const gain = (p.apCurve ?? 10) / 100;
const fade = p.apCurveDecay ?? 1500;
const limit = (b) => Math.max(-0.8, Math.min(0.8, b));
mem.bias = (mem.bias || 0) * Math.exp(-(s.dt || 0) / fade); // + = steer right

const code = (s.L ? 2 : 0) + (s.R ? 1 : 0);
const prev = mem.prev ?? 0;
mem.prev = code;

if (code === 3) {
  // Nose fully off the lane. Which edge did we cross? The sensor that went out first tells us.
  // Both at once while curving means we cut across the lane: turn back against the bend.
  if (prev === 2) mem.right = true;
  else if (prev === 1) mem.right = false;
  else if (prev === 0) mem.right = mem.bias < 0;
  return mem.right ? [base, p.apHard] : [p.apHard, base];
}
if (code === 2) { mem.bias = limit(mem.bias + gain); return [base, p.apTurn]; }  // left edge: steer right
if (code === 1) { mem.bias = limit(mem.bias - gain); return [p.apTurn, base]; }  // right edge: steer left

// Inside the lane: follow the learned bend. Inner wheel goes from base down to the slowest moving speed.
const inner = base - (base - p.minSpeed) * Math.min(1, Math.abs(mem.bias) / 0.8);
return mem.bias >= 0 ? [base, inner] : [inner, base];
