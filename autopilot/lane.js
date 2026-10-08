// Lane keeper for the rally mat: the whole lane (pink/purple/blue and its white edge lines) reads WHITE
// and the mat outside reads BLACK. Keep "Invert sensors" OFF. s.L / s.R are true when that side of the
// car's nose sees black, i.e. it is leaving the lane on that side.
//
// It steers away from whichever edge it touches, and learns the bend: every reading with one sensor out
// adds steering bias away from that side. Inside the lane it keeps steering by that bias, which fades on
// straights (Curve memory), so it follows curves instead of bouncing along the outside edge.
// When the nose is fully out it pivots back in, toward the side it last touched. If that doesn't bring
// it back it tries the other way, and after GIVE_UP_MS it stops and says why in the Log (press GO again).
// Tune: Base speed, Inner wheel soft turn (edge touch), Inner wheel line lost (nose fully out),
// Curve learning, Curve memory.

const GIVE_UP_MS = 2500;

const base = p.apBase;
const gain = (p.apCurve ?? 10) / 100;
const fade = p.apCurveDecay ?? 1500;
const limit = (b) => Math.max(-0.8, Math.min(0.8, b));
mem.bias = (mem.bias || 0) * Math.exp(-(s.dt || 0) / fade); // + = steer right
if (mem.inAt === undefined) mem.inAt = s.t;

const code = (s.L ? 2 : 0) + (s.R ? 1 : 0);
if (code !== 3) mem.inAt = s.t;
if (code === 2) mem.edge = { side: 'L', t: s.t };
if (code === 1) mem.edge = { side: 'R', t: s.t };
if (mem.gaveUp) return [0, 0];

if (code === 3) {
  const out = s.t - mem.inAt;
  if (out > GIVE_UP_MS) {
    mem.gaveUp = true;
    ctx.log(`lane: nose off the lane for ${GIVE_UP_MS} ms, stopped. Turning ${mem.right ? 'right' : 'left'}, bias ${mem.bias.toFixed(2)}`);
    return [0, 0];
  }
  if (mem.outAt !== mem.inAt) {
    // Just left the lane. Turn away from the edge we touched last (if recent), else against the learned bend.
    mem.outAt = mem.inAt;
    mem.flipped = false;
    const recent = mem.edge && s.t - mem.edge.t < 400;
    mem.right = recent ? mem.edge.side === 'L' : mem.bias < 0;
  }
  if (!mem.flipped && out > 1200) {
    // Still out after a long pivot: the guess was wrong, turn the other way.
    mem.flipped = true;
    mem.right = !mem.right;
    ctx.log(`lane: still off the lane, trying the other way`);
  }
  return mem.right ? [base, p.apHard] : [p.apHard, base];
}
if (code === 2) { mem.bias = limit(mem.bias + gain); return [base, p.apTurn]; }  // left edge: steer right
if (code === 1) { mem.bias = limit(mem.bias - gain); return [p.apTurn, base]; }  // right edge: steer left

// Inside the lane: follow the learned bend. Inner wheel goes from base down to the slowest moving speed.
const inner = base - (base - p.minSpeed) * Math.min(1, Math.abs(mem.bias) / 0.8);
return mem.bias >= 0 ? [base, inner] : [inner, base];
