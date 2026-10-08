// Edge follower: rides the boundary of a wide stripe, with the LEFT sensor on the line and the
// RIGHT sensor off it. Useful when the line is wider than the gap between the sensors,
// or for hugging one edge of a track lane. To hug the other edge, swap the two turn lines below.
// "Invert sensors" in Tune flips what counts as "on the line".

const base = p.apBase;
mem.lostAt = mem.lostAt || 0;

if (s.L && !s.R) { mem.lostAt = 0; return [base, base]; }          // on the edge
if (s.L && s.R)  { mem.lostAt = 0; mem.side = 1; return [base, p.apTurn]; } // too far onto the line: turn right
if (!s.L && !s.R) {                                                   // fell off the line: turn left
  mem.lostAt = mem.lostAt || s.t;
  mem.side = -1;
  const inner = s.t - mem.lostAt > p.apLostMs ? p.apHard : p.apTurn;
  return [inner, base];
}
// !s.L && s.R: the edge is on the wrong side (crossed a thin line). Turn back the way we were correcting.
return mem.side > 0 ? [base, p.apHard] : [p.apHard, base];
