// Edge follower: rides a boundary with the LEFT sensor on the line and the RIGHT sensor off it.
// On the rally mat, keep "Invert sensors" OFF: there the line is the black mat, so the car follows the
// lane's left edge with black on its left and the white lane on its right. Place the car with its
// nose on that edge, facing the driving direction.
// To ride the right edge instead (white on the left, black on the right), turn "Invert sensors" ON.
//
// Corrections start gentle and grow the longer the car stays off the edge, reaching
// "Inner wheel, soft turn" after "Line lost → hard turn after (ms)".

const base = p.apBase;
const code = (s.L ? 2 : 0) + (s.R ? 1 : 0);
if (code !== mem.code) { mem.code = code; mem.since = s.t; }
const k = Math.min(1, (s.t - mem.since) / Math.max(1, p.apLostMs));
const inner = base - (base - p.apTurn) * (0.3 + 0.7 * k);

if (code === 2) return [base, base];                       // on the edge
if (code === 3) { mem.side = 1; return [base, inner]; }    // drifted onto the line side: steer right
if (code === 0) { mem.side = -1; return [inner, base]; }   // drifted off it: steer left
// Left off, right on: we crossed a thin line or turned around. Turn back the way we were correcting.
return mem.side > 0 ? [base, p.apHard] : [p.apHard, base];
