// Explore & map: drives the lane like the Lane keeper (same sliders) for as long as you let it, and builds
// a map while it goes. The map is drawn over the middle of the screen, on top of the traced rally track (gray):
//   blue = where the nose saw the white lane, purple = where it saw the black mat (lane edge),
//   red = something the distance sensor saw ahead (cone, wall), brighter = seen more often, green = the car.
// Position is dead reckoning from the wheel commands (VMAX, WHEELBASE below), so it drifts over minutes.
// It assumes the car starts on the start straight heading right (as the gray track is drawn).
// Put your finger on the map where the car really is and drag the way it faces, then lift: it jumps there
// and the Log says how far off it was. A tap without a drag keeps the heading. The first placement also
// moves everything mapped so far; later ones only fix the car.
//
// It reads ?DIST itself every DIST_MS (only if "Read distance every N" is 0), so leave "Stop for obstacle" at 0.
// Something closer than NEAR_CM ahead, or no progress for STUCK_MS (distance and line sensors both frozen
// while driving forward): it backs up, turns a random way and carries on. Off the lane for GIVE_UP_MS:
// it backs up and searches again, and stops for good after MAX_RETRIES of those in a row.
// The Log gets a summary line every 15 s plus each new obstacle and stuck event. Stops by itself after MAX_RUN_MS.

const VMAX = 50;          // cm/s at motor 100 (guess; measure on the track and fix)
const WHEELBASE = 9;      // cm
const NOSE = 6;           // cm from the axle to the line sensors / sonar
const CELL = 5;           // cm per map cell
const DIST_MS = 250;      // ask ?DIST this often
const SONAR_MAX = 80;     // ignore echoes farther than this (cm)
const NEAR_CM = 12;       // closer than this ahead = blocked
const STUCK_MS = 2500;
const GIVE_UP_MS = 2500;
const MAX_RETRIES = 3;
const MAX_RUN_MS = 12 * 60 * 1000;
const BACK_MS = 600, TURN_MS = 450;

const base = Math.max(p.apBase, p.minSpeed + 5, 30);
const gain = (p.apCurve ?? 10) / 100;
const fade = p.apCurveDecay ?? 1500;
const limit = (b) => Math.max(-0.8, Math.min(0.8, b));
const dt = s.dt || 0;

if (mem.t0 === undefined) {
  mem.t0 = s.t;
  mem.x = 0; mem.y = 0; mem.h = 0;     // cm, cm, rad (0 = start direction, + = left)
  mem.cells = {};                      // "i,j" -> [lane, edge, obstacle]
  mem.obst = 0; mem.stuck = 0; mem.retries = 0;
  mem.prog = { t: s.t, dist: s.dist, code: s.code };
  mem.distAt = 0; mem.logAt = s.t; mem.drawAt = 0; mem.saveAt = s.t;
  ctx.log(`explore: start, base ${base}${p.apStopDist > 0 ? ' — set "Stop for obstacle" to 0 or it will freeze at cones' : ''}`);
}
const runMs = s.t - mem.t0;
const web = typeof document !== 'undefined';

// ---- the traced rally track from sim.js (needs a build that exports it; without it there is no gray outline)
if (web && !window.__rrTrack) {
  window.__rrTrack = 'loading';
  import(new URL('sim.js', document.baseURI).href)
    .then((m) => { window.__rrTrack = m.RALLY || 'none'; })
    .catch(() => { window.__rrTrack = 'none'; });
}
const track = web && Array.isArray(window.__rrTrack) ? window.__rrTrack : null;

// Move the car to a new pose. Until the user has placed it, everything mapped so far moves along with it.
function place(x, y, h, why) {
  const off = Math.hypot(x - mem.x, y - mem.y);
  const dh = h - mem.h;
  if (!mem.userPlaced) {
    const moved = {};
    const c = Math.cos(dh), sn = Math.sin(dh);
    for (const k of Object.keys(mem.cells)) {
      const [i, j] = k.split(',').map(Number);
      const rx = i * CELL - mem.x, ry = j * CELL - mem.y;
      const key = Math.round((x + rx * c - ry * sn) / CELL) + ',' + Math.round((y + rx * sn + ry * c) / CELL);
      const a = mem.cells[k], b = moved[key] || (moved[key] = [0, 0, 0]);
      for (let n = 0; n < 3; n++) b[n] += a[n];
    }
    mem.cells = moved;
  }
  if (why) ctx.log(`explore: ${why} at ${Math.round(x)},${Math.round(y)} cm heading ${Math.round((h * 180) / Math.PI)}° after ${Math.round(runMs / 1000)} s — dead reckoning was ${Math.round(off)} cm and ${Math.round((((dh * 180) / Math.PI + 540) % 360) - 180)}° off`);
  mem.x = x; mem.y = y; mem.h = h;
}
if (track && !mem.anchored) {
  mem.anchored = true;
  place(track[0][0], track[0][1], 0, null);
}
if (web && window.__rrPlace) {
  const f = window.__rrPlace;
  window.__rrPlace = null;
  place(f.x, f.y, f.h ?? mem.h, 'you placed the car');
  mem.userPlaced = true;
}

// ---- dead reckoning from what was actually sent to the motors during the last tick
const speed = (m) => {
  const a = Math.abs(m);
  return a <= p.minSpeed ? 0 : (Math.sign(m) * (Math.min(a, 100) - p.minSpeed) / (100 - p.minSpeed)) * VMAX;
};
const vl = speed(s.out[0]), vr = speed(s.out[1]);
mem.h += ((vr - vl) / WHEELBASE) * (dt / 1000);
mem.x += Math.cos(mem.h) * ((vl + vr) / 2) * (dt / 1000);
mem.y += Math.sin(mem.h) * ((vl + vr) / 2) * (dt / 1000);

const mark = (x, y, k) => {
  const key = Math.round(x / CELL) + ',' + Math.round(y / CELL);
  const c = mem.cells[key] || (mem.cells[key] = [0, 0, 0]);
  c[k]++;
  return c[k];
};
const nx = mem.x + Math.cos(mem.h) * NOSE, ny = mem.y + Math.sin(mem.h) * NOSE;
mark(nx, ny, s.L && s.R ? 1 : 0);

// ---- distance sensor
if (!(p.apDistEvery > 0) && s.t - mem.distAt > DIST_MS) { ctx.send('?DIST'); mem.distAt = s.t; }
const distFresh = s.dist > 2 && s.distAge < 2 * DIST_MS + 200;
if (distFresh && s.distAge < dt + 50 && s.dist < SONAR_MAX) {
  const d = NOSE + s.dist;
  if (mark(mem.x + Math.cos(mem.h) * d, mem.y + Math.sin(mem.h) * d, 2) === 2) {
    mem.obst++;
    ctx.log(`explore: obstacle #${mem.obst} at ${Math.round(mem.x + Math.cos(mem.h) * d)},${Math.round(mem.y + Math.sin(mem.h) * d)} cm (${s.dist} cm ahead)`);
  }
}

// ---- driving
function laneKeeper() {
  mem.bias = (mem.bias || 0) * Math.exp(-dt / fade); // + = steer right
  if (mem.inAt === undefined) mem.inAt = s.t;
  const code = (s.L ? 2 : 0) + (s.R ? 1 : 0);
  if (code !== 3) mem.inAt = s.t;
  if (code === 2) mem.edge = { side: 'L', t: s.t };
  if (code === 1) mem.edge = { side: 'R', t: s.t };
  if (code === 3) {
    const out = s.t - mem.inAt;
    if (mem.outAt !== mem.inAt) {
      mem.outAt = mem.inAt;
      mem.flipped = false;
      const recent = mem.edge && s.t - mem.edge.t < 400;
      mem.right = recent ? mem.edge.side === 'L' : mem.bias < 0;
    }
    if (!mem.flipped && out > 1200) { mem.flipped = true; mem.right = !mem.right; }
    return mem.right ? [base, p.apHard] : [p.apHard, base];
  }
  if (code === 2) { mem.bias = limit(mem.bias + gain); return [base, p.apTurn]; }
  if (code === 1) { mem.bias = limit(mem.bias - gain); return [p.apTurn, base]; }
  const inner = base - (base - p.minSpeed) * Math.min(1, Math.abs(mem.bias) / 0.8);
  return mem.bias >= 0 ? [base, inner] : [inner, base];
}

function recover(why) {
  mem.man = { until: s.t + BACK_MS + TURN_MS, turnAt: s.t + BACK_MS, right: Math.random() < 0.5 };
  mem.prog = { t: s.t + BACK_MS + TURN_MS, dist: s.dist, code: s.code };
  mem.inAt = s.t + BACK_MS + TURN_MS;
  ctx.log(`explore: ${why}, backing up and turning ${mem.man.right ? 'right' : 'left'}`);
}

let cmd;
if (mem.done) {
  cmd = [0, 0];
} else if (runMs > MAX_RUN_MS) {
  mem.done = true;
  ctx.log(`explore: ran ${Math.round(MAX_RUN_MS / 60000)} min, stopped`);
  cmd = [0, 0];
} else if (mem.man && s.t < mem.man.until) {
  const back = -base;
  const turn = mem.man.right ? [base, -base] : [-base, base];
  cmd = s.t < mem.man.turnAt ? [back, back] : turn;
} else {
  mem.man = null;
  const fwd = s.out[0] > p.minSpeed && s.out[1] > p.minSpeed;
  if (!(s.L && s.R)) mem.lastIn = s.t;
  if (!(s.L && s.R) && s.t - (mem.retryAt || 0) > 2000) mem.retries = 0;
  if (fwd && distFresh && s.dist < NEAR_CM) {
    recover(`blocked ${s.dist} cm ahead`);
  } else if (s.L && s.R && s.t - (mem.lastIn ?? s.t) > GIVE_UP_MS) {
    mem.retries++;
    mem.retryAt = s.t;
    if (mem.retries > MAX_RETRIES) {
      mem.done = true;
      ctx.log(`explore: lost the lane ${MAX_RETRIES} times in a row, stopped. Put it back on the lane and press GO.`);
    } else {
      mem.lastIn = s.t + BACK_MS + TURN_MS;
      recover(`off the lane (try ${mem.retries}/${MAX_RETRIES})`);
    }
  } else {
    // Stuck: driving forward but neither the distance nor the line sensors changed for STUCK_MS.
    const moved = s.code !== mem.prog.code || (distFresh && Math.abs(s.dist - (mem.prog.dist ?? s.dist)) > 2);
    if (!fwd || moved) mem.prog = { t: s.t, dist: s.dist, code: s.code };
    else if (s.t - mem.prog.t > STUCK_MS && distFresh && s.dist < 150) {
      mem.stuck++;
      recover(`no progress for ${STUCK_MS} ms (stuck #${mem.stuck})`);
    }
  }
  cmd = mem.man ? [-base, -base] : mem.done ? [0, 0] : laneKeeper();
}

// ---- report, save, draw
if (s.t - mem.logAt > 15000) {
  mem.logAt = s.t;
  const n = Object.keys(mem.cells).length;
  ctx.log(`explore ${Math.round(runMs / 1000)}s at ${Math.round(mem.x)},${Math.round(mem.y)} cm heading ${Math.round((mem.h * 180) / Math.PI) % 360}° | ${n} cells, ${mem.obst} obstacles, ${mem.stuck} stuck`);
}
if (s.t - mem.saveAt > 10000) {
  mem.saveAt = s.t;
  try { localStorage.setItem('rrExploreMap', JSON.stringify({ cell: CELL, at: Date.now(), cells: mem.cells })); } catch (e) { /* storage full or unavailable */ }
}
if (web && s.t - mem.drawAt > 300) {
  mem.drawAt = s.t;
  const dpr = window.devicePixelRatio || 1;
  let cv = document.getElementById('rrMap');
  if (!cv) {
    cv = document.createElement('canvas');
    cv.id = 'rrMap';
    cv.style.cssText = 'position:fixed;top:56px;left:50%;transform:translateX(-50%);background:rgba(0,0,0,.85);' +
      'border:1px solid #555;border-radius:8px;z-index:50;touch-action:none';
    document.body.appendChild(cv);
    // Finger down = where the car is, drag = which way it faces, lift = apply.
    const at = (e) => { const r = cv.getBoundingClientRect(); return [(e.clientX - r.left) * dpr, (e.clientY - r.top) * dpr]; };
    cv.addEventListener('pointerdown', (e) => {
      e.preventDefault(); e.stopPropagation();
      cv.setPointerCapture(e.pointerId);
      window.__rrTouch = { a: at(e), b: at(e) };
    });
    cv.addEventListener('pointermove', (e) => { if (window.__rrTouch) window.__rrTouch.b = at(e); });
    const end = (e) => {
      const tc = window.__rrTouch, v = window.__rrMapView;
      window.__rrTouch = null;
      if (!tc || !v || e.type === 'pointercancel') return;
      const [ax, ay] = tc.a, [bx, by] = tc.b;
      const drag = Math.hypot(bx - ax, by - ay) > 15 * dpr;
      window.__rrPlace = { x: v.x0 + ax / v.sc, y: v.y1 - ay / v.sc, h: drag ? Math.atan2(-(by - ay), bx - ax) : null };
    };
    cv.addEventListener('pointerup', end);
    cv.addEventListener('pointercancel', end);
  }
  cv.style.display = '';
  clearTimeout(window.__rrMapHide);
  window.__rrMapHide = setTimeout(() => { cv.style.display = 'none'; }, 5000); // hides once the script stops

  // View in cm: the track plus everything mapped plus the car, with a margin.
  const keys = Object.keys(mem.cells);
  let x0 = mem.x, x1 = mem.x, y0 = mem.y, y1 = mem.y;
  const grow = (x, y) => { x0 = Math.min(x0, x); x1 = Math.max(x1, x); y0 = Math.min(y0, y); y1 = Math.max(y1, y); };
  if (track) for (const q of track) grow(q[0], q[1]);
  for (const k of keys) { const [i, j] = k.split(',').map(Number); grow(i * CELL, j * CELL); }
  const pad = 25;
  x0 -= pad; x1 += pad; y0 -= pad; y1 += pad;
  const vw = Math.min(window.innerWidth * 0.6, 520), vh = Math.min(window.innerHeight - 70, 420);
  const scCss = Math.min(vw / (x1 - x0), vh / (y1 - y0));
  cv.style.width = Math.round((x1 - x0) * scCss) + 'px';
  cv.style.height = Math.round((y1 - y0) * scCss) + 'px';
  cv.width = Math.round((x1 - x0) * scCss * dpr);
  cv.height = Math.round((y1 - y0) * scCss * dpr);
  const sc = scCss * dpr;
  window.__rrMapView = { x0, y1, sc };
  const px = (x) => (x - x0) * sc, py = (y) => (y1 - y) * sc;
  const g = cv.getContext('2d');
  if (track) {
    g.lineJoin = 'round';
    g.strokeStyle = '#3a3a48';
    g.lineWidth = 20 * sc;
    g.beginPath();
    track.forEach((q, n) => (n ? g.lineTo(px(q[0]), py(q[1])) : g.moveTo(px(q[0]), py(q[1]))));
    g.closePath();
    g.stroke();
  }
  const cs = Math.max(CELL * sc, 2);
  for (const k of keys) {
    const [i, j] = k.split(',').map(Number);
    const [lane, edge, obst] = mem.cells[k];
    g.fillStyle = obst ? `rgba(255,60,60,${Math.min(1, 0.3 + obst * 0.15)})` : edge > lane ? '#8a3fd1' : '#3a7bd5';
    g.fillRect(px(i * CELL) - cs / 2, py(j * CELL) - cs / 2, cs, cs);
  }
  const arrow = (x, y, h, color) => {
    const r = Math.max(9 * dpr, 8 * sc);
    g.fillStyle = color;
    g.beginPath();
    g.moveTo(x + Math.cos(h) * r, y - Math.sin(h) * r);
    g.lineTo(x + Math.cos(h + 2.5) * r * 0.7, y - Math.sin(h + 2.5) * r * 0.7);
    g.lineTo(x + Math.cos(h - 2.5) * r * 0.7, y - Math.sin(h - 2.5) * r * 0.7);
    g.fill();
  };
  arrow(px(mem.x), py(mem.y), mem.h, '#3f3');
  const tc = window.__rrTouch;
  if (tc) {
    const drag = Math.hypot(tc.b[0] - tc.a[0], tc.b[1] - tc.a[1]) > 15 * dpr;
    arrow(tc.a[0], tc.a[1], drag ? Math.atan2(-(tc.b[1] - tc.a[1]), tc.b[0] - tc.a[0]) : mem.h, '#ff0');
  }
  g.fillStyle = '#ccc';
  g.font = `${Math.round(12 * dpr)}px sans-serif`;
  g.fillText(`${Math.round(runMs / 1000)}s  ${mem.obst} obst  ${mem.stuck} stuck${mem.userPlaced ? '' : '  · touch where the car is, drag its heading'}`, 6 * dpr, 16 * dpr);
}
return cmd;
