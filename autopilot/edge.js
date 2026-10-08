// Edge follower: rides a boundary with the LEFT sensor on the line and the RIGHT sensor off it.
// On the rally mat, keep "Invert sensors" OFF: there the line is the black mat, so the car follows the
// lane's left edge with black on its left and the white lane on its right. Place the car with its
// nose on that edge, facing the driving direction.
// To ride the right edge instead (white on the left, black on the right), turn "Invert sensors" ON.
//
// Corrections start gentle and grow the longer the car stays off the edge, reaching
// "Inner wheel, soft turn" after "Line lost → hard turn after (ms)".
// If it can't get back onto the edge within GIVE_UP_MS it stops (no wild circles) and says why in the Log.
// Press GO to try again.

const GIVE_UP_MS = 2000;

const base = p.apBase;
const code = (s.L ? 2 : 0) + (s.R ? 1 : 0);
if (mem.onEdgeAt === undefined) { mem.onEdgeAt = s.t; mem.counts = [0, 0, 0, 0]; }
if (code !== mem.code) { mem.code = code; mem.since = s.t; mem.counts[code]++; }
if (code === 2) mem.onEdgeAt = s.t;
if (mem.gaveUp) return [0, 0];
if (s.t - mem.onEdgeAt > GIVE_UP_MS) {
  mem.gaveUp = true;
  const what = ['both white (in the lane)', 'left white + right black (wrong way round?)', 'on edge', 'both black (off the lane)'][code];
  ctx.log(`edge: lost for ${GIVE_UP_MS} ms, stopped. Now: ${what}. Seen: on edge x${mem.counts[2]}, in lane x${mem.counts[0]}, off lane x${mem.counts[3]}, reversed x${mem.counts[1]}`);
  return [0, 0];
}
const k = Math.min(1, (s.t - mem.since) / Math.max(1, p.apLostMs));
const inner = base - (base - p.apTurn) * (0.3 + 0.7 * k);

if (code === 2) return [base, base];                       // on the edge
if (code === 3) { mem.side = 1; return [base, inner]; }    // drifted onto the line side: steer right
if (code === 0) { mem.side = -1; return [inner, base]; }   // drifted off it: steer left
// Left off, right on: we crossed a thin line or turned around. Turn back the way we were correcting.
return mem.side > 0 ? [base, p.apHard] : [p.apHard, base];
