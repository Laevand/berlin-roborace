// Mapping run: drives exactly like the Lane keeper (same sliders), and writes a map of the track to the
// Log tab, one line per second since GO:
//   map 12s turn R+0.42 bias +0.35 edges L0 R3 out 0ms
// turn = average steering over that second (R = right, L = left, 0 = straight; ±1 = pivot on one wheel),
// edges = how many times the nose touched the left/right edge, out = time with the nose fully off the lane.
// Run it slowly (Base speed ~35-40) for a full lap, then screenshot the Log. That gives the order and
// length of the straights and bends, for tuning Auto per section.

const GIVE_UP_MS = 2500;

const base = Math.max(p.apBase, p.minSpeed + 5, 30);
const gain = (p.apCurve ?? 10) / 100;
const fade = p.apCurveDecay ?? 1500;
const limit = (b) => Math.max(-0.8, Math.min(0.8, b));

function drive() {
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
      ctx.log(`map: nose off the lane for ${GIVE_UP_MS} ms, stopped.`);
      return [0, 0];
    }
    if (mem.outAt !== mem.inAt) {
      mem.outAt = mem.inAt;
      mem.flipped = false;
      const recent = mem.edge && s.t - mem.edge.t < 400;
      mem.right = recent ? mem.edge.side === 'L' : mem.bias < 0;
    }
    if (!mem.flipped && out > 1200) {
      mem.flipped = true;
      mem.right = !mem.right;
      ctx.log(`map: still off the lane, trying the other way`);
    }
    return mem.right ? [base, p.apHard] : [p.apHard, base];
  }
  if (code === 2) { mem.bias = limit(mem.bias + gain); return [base, p.apTurn]; }
  if (code === 1) { mem.bias = limit(mem.bias - gain); return [p.apTurn, base]; }

  const inner = base - (base - p.minSpeed) * Math.min(1, Math.abs(mem.bias) / 0.8);
  return mem.bias >= 0 ? [base, inner] : [inner, base];
}

const out = drive();

// ---- map logging
if (mem.t0 === undefined) {
  mem.t0 = s.t;
  mem.sec = 0;
  mem.acc = { turn: 0, w: 0, L: 0, R: 0, out: 0 };
  ctx.log(`map: start, base ${base}`);
}
const a = mem.acc;
const sum = Math.abs(out[0]) + Math.abs(out[1]);
const dt = s.dt || 0;
if (sum > 0) { a.turn += ((out[0] - out[1]) / sum) * dt; a.w += dt; }
if (s.code !== mem.lastCode) {
  if (s.L && !s.R) a.L++;
  if (s.R && !s.L) a.R++;
  mem.lastCode = s.code;
}
if (s.L && s.R) a.out += dt;
const sec = Math.floor((s.t - mem.t0) / 1000);
if (sec > mem.sec && !mem.mapDone) {
  const turn = a.w ? a.turn / a.w : 0;
  const dir = Math.abs(turn) < 0.05 ? '0' : (turn > 0 ? 'R' : 'L') + Math.abs(turn).toFixed(2);
  ctx.log(`map ${sec}s turn ${dir} bias ${(mem.bias >= 0 ? '+' : '') + mem.bias.toFixed(2)} edges L${a.L} R${a.R} out ${Math.round(a.out)}ms`);
  mem.sec = sec;
  mem.acc = { turn: 0, w: 0, L: 0, R: 0, out: 0 };
  if (mem.gaveUp) mem.mapDone = true;
}
return out;
