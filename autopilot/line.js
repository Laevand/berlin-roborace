// Classic two-sensor line follower.
// s.L / s.R are true when that sensor is on the line. Turn on "Invert sensors" in Tune
// to follow a white line on a dark floor instead of a black line on a light one.
// Return [leftMotor, rightMotor] in -100..100.

const base = p.apBase;
if (s.L && s.R) { mem.side = 0; mem.lostAt = 0; return [base, base]; }   // centered
if (s.L)        { mem.side = -1; mem.lostAt = 0; return [p.apTurn, base]; } // line drifting left: turn left
if (s.R)        { mem.side = 1;  mem.lostAt = 0; return [base, p.apTurn]; } // line drifting right: turn right

// Line lost: keep turning toward the side it was last seen, harder after apLostMs.
mem.lostAt = mem.lostAt || s.t;
const inner = s.t - mem.lostAt > p.apLostMs ? p.apHard : p.apTurn;
if (mem.side < 0) return [inner, base];
if (mem.side > 0) return [base, inner];
return [base * 0.6, base * 0.6]; // never saw it: creep forward
