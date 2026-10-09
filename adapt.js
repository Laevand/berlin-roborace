// Vision feedback for the control loop. A camera pipeline (or anything else that can see the car) reports
// where the car is on the track. This module turns that into:
//   1. s.vis for autopilot scripts: the pose predicted to the moment the next motor command lands, the lane
//      error, the bend ahead and a look-ahead point on the lane center (see autopilot/vision-pilot.js);
//   2. self-calibration: trim, speed at motor 100, deadband, effective wheelbase and command delay, fitted
//      from what the car actually did compared with the motor commands that were sent;
//   3. lap events when the car crosses the start of the track.
// No DOM and no dependencies: app.js, tools/simrun.mjs and tools/smoke.mjs all import it.
// Feed format: VISION.md (messages, or the window.__rrVision v1 contract of docs/AUTONOMY-ARCHITECTURE.md).
// Learned values are applied by app.js on top of the user's sliders and stored in rrLearn.v1, never in a slider.

const TAU = 2 * Math.PI;
export const wrap = (a) => ((((a + Math.PI) % TAU) + TAU) % TAU) - Math.PI;
const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const fin = (v) => v !== null && v !== '' && typeof v !== 'boolean' && Number.isFinite(Number(v));

const WIN = 250;            // ms of motion per calibration sample
const SAMPLE_EVERY = 80;    // ms between calibration samples
const MAX_SAMPLES = 500;    // about 40 s of driving
const MIN_SAMPLES = 40;
export const DELAYS = Array.from({ length: 21 }, (_, i) => i * 25); // command -> motion delays tried, ms
const DEADBANDS = Array.from({ length: 18 }, (_, i) => 6 + i * 2);  // motor deadbands tried
const W_NOM = 9;            // cm, scales yaw-rate errors into speed units for the fit

// One feed message (object or JSON text) -> { kind: 'track', center, half, closed } | { kind: 'pose', poses } | null.
export function parseMessage(raw) {
  let m = raw;
  if (typeof m === 'string') { try { m = JSON.parse(m); } catch { return null; } }
  if (m && m.rrVision) m = m.rrVision; // postMessage envelope
  if (!m || typeof m !== 'object') return null;
  const k = fin(m.scale) ? Number(m.scale) : 1;
  const fy = m.yDown ? -1 : 1;
  if (m.type === 'track') {
    const src = m.center || m.points;
    if (!Array.isArray(src)) return null;
    const center = src.filter((q) => Array.isArray(q) && fin(q[0]) && fin(q[1])).map((q) => [q[0] * k, q[1] * k * fy]);
    if (center.length < 3) return null;
    return { kind: 'track', center, half: (fin(m.width) ? m.width * k : 20) / 2, closed: m.closed !== false };
  }
  if (m.v === 1 && 'robot' in m) {
    // window.__rrVision v1: one robot in track cm, t on the performance.now() clock, sigma = 1-σ error in cm.
    const r = m.robot;
    if (!r || !fin(r.x) || !fin(r.y) || !fin(m.t)) return null;
    const hs = fin(r.hSigma) ? Number(r.hSigma) : null;
    return { kind: 'pose', poses: [{
      id: '', x: Number(r.x), y: Number(r.y), h: fin(r.h) && !(hs > 0.5) ? wrap(Number(r.h)) : null,
      t: null, tPerf: Number(m.t), age: null, sigma: fin(r.sigma) ? Number(r.sigma) : null,
      conf: fin(m.conf) ? clamp(Number(m.conf), 0, 1) : 1,
    }] };
  }
  if (m.type && m.type !== 'pose' && m.type !== 'poses') return null;
  const list = Array.isArray(m.robots) ? m.robots : [m];
  const poses = [];
  for (const r of list) {
    if (!r || !fin(r.x) || !fin(r.y)) continue;
    let h = fin(r.h) ? Number(r.h) : fin(r.hdeg) ? (r.hdeg * Math.PI) / 180 : null;
    if (h !== null) h = wrap(h * fy);
    poses.push({
      id: r.id != null ? String(r.id) : '',
      x: r.x * k, y: r.y * k * fy, h,
      t: fin(r.t) ? Number(r.t) : fin(m.t) ? Number(m.t) : null,
      age: fin(r.age) ? Number(r.age) : fin(m.age) ? Number(m.age) : null,
      sigma: fin(r.sigma) ? Number(r.sigma) : null,
      conf: fin(r.conf) ? clamp(Number(r.conf), 0, 1) : 1,
    });
  }
  return poses.length ? { kind: 'pose', poses } : null;
}

// Lane center line as a polyline (cm). Direction of travel = order of the points (or reversed, see Vision.dir).
export class Track {
  constructor(center, half = 10, closed = true) {
    this.half = half;
    this.closed = closed;
    const pts = center.map((q) => [Number(q[0]), Number(q[1])]);
    const n = pts.length;
    this.seg = [];
    let s = 0;
    for (let i = 0; i < (closed ? n : n - 1); i++) {
      const [ax, ay] = pts[i], [bx, by] = pts[(i + 1) % n];
      const len = Math.hypot(bx - ax, by - ay);
      if (len < 1e-6) continue;
      this.seg.push({ ax, ay, ux: (bx - ax) / len, uy: (by - ay) / len, len, s0: s, th: Math.atan2(by - ay, bx - ax) });
      s += len;
    }
    this.len = s;
    this.step = s / Math.max(1, this.seg.length);
  }

  norm(s) { return this.closed ? ((s % this.len) + this.len) % this.len : clamp(s, 0, this.len); }

  segAt(s) {
    s = this.norm(s);
    let lo = 0, hi = this.seg.length - 1;
    while (lo < hi) { const mid = (lo + hi + 1) >> 1; if (this.seg[mid].s0 <= s) lo = mid; else hi = mid - 1; }
    return lo;
  }

  // point and heading at arc length s
  at(s) {
    s = this.norm(s);
    const g = this.seg[this.segAt(s)];
    const u = clamp(s - g.s0, 0, g.len);
    return { x: g.ax + g.ux * u, y: g.ay + g.uy * u, th: g.th };
  }

  // 1/cm, + = bends left (in point order), averaged over ±8 cm so a hand-traced outline isn't too jumpy
  curvature(s) { return wrap(this.at(s + 8).th - this.at(s - 8).th) / 16; }

  // Nearest center-line point: { s, e (cm, + = left of the point order), d, th }. With a hint (previous s)
  // it stays on the same strand where strands run side by side, unless that is more than a lane width off.
  locate(x, y, hint = null) {
    const m = this.seg.length;
    let best = null;
    const scan = (i0, count) => {
      for (let k = 0; k < count; k++) {
        let i = i0 + k;
        if (this.closed) i = ((i % m) + m) % m; else if (i < 0 || i >= m) continue;
        const g = this.seg[i];
        const u = clamp((x - g.ax) * g.ux + (y - g.ay) * g.uy, 0, g.len);
        const px = g.ax + g.ux * u, py = g.ay + g.uy * u;
        const d = Math.hypot(x - px, y - py);
        if (!best || d < best.d) best = { s: g.s0 + u, e: -(x - px) * g.uy + (y - py) * g.ux, d, th: g.th };
      }
    };
    if (hint !== null) {
      const K = Math.ceil(60 / this.step) + 1;
      scan(this.segAt(hint) - K, 2 * K + 1);
    }
    if (!best || best.d > 2 * this.half) { best = null; scan(0, m); }
    return best;
  }
}

// Least squares for the wheel gains (cm/s per motor unit above the deadband) and the effective wheelbase.
// Model: v = (gL·f(l) + gR·f(r)) / 2, w = (gR·f(r) − gL·f(l)) / W, f(m) = sign(m)·max(0, |m| − d0).
function solve(S, di, d0, W0, fitW) {
  const n = S.length;
  const fl = new Float64Array(n), fr = new Float64Array(n);
  const f = (m) => Math.sign(m) * Math.max(0, Math.abs(m) - d0);
  for (let i = 0; i < n; i++) { fl[i] = f(S[i].cmd[di][0]); fr[i] = f(S[i].cmd[di][1]); }
  let W = W0, gL = 0, gR = 0;
  for (let it = 0; it < (fitW ? 4 : 1); it++) {
    let ll = 0, lv = 0, rr = 0, rv = 0;
    for (let i = 0; i < n; i++) {
      const s = S[i];
      ll += fl[i] * fl[i]; lv += fl[i] * (s.v - (s.w * W) / 2);
      rr += fr[i] * fr[i]; rv += fr[i] * (s.v + (s.w * W) / 2);
    }
    if (ll < 1 || rr < 1) return null;
    gL = lv / ll; gR = rv / rr;
    if (!fitW) break;
    let dd = 0, dw = 0;
    for (let i = 0; i < n; i++) { const D = gR * fr[i] - gL * fl[i]; dd += D * D; dw += D * S[i].w; }
    if (dw > 0) W = clamp(dd / dw, 4, 30);
  }
  let ev = 0, ew = 0;
  const res = new Float64Array(n);
  for (let i = 0; i < n; i++) {
    const a = S[i].v - (gL * fl[i] + gR * fr[i]) / 2;
    const b = S[i].w - (gR * fr[i] - gL * fl[i]) / W;
    ev += a * a; ew += b * b;
    res[i] = a * a + W_NOM * W_NOM * b * b;
  }
  return { gL, gR, W, err: (ev + W_NOM * W_NOM * ew) / n, rmsV: Math.sqrt(ev / n), rmsW: Math.sqrt(ew / n), res };
}

const median = (a) => { const b = [...a].sort((x, y) => x - y); return b.length ? b[b.length >> 1] : 0; };

// Trim that makes both wheels equally fast at motor m (trimmed() scales the whole command, deadband included).
const trimFor = (gL, gR, d0, m) => (100 * (gR - gL) * (m - d0)) / (m * (gL + gR));

export class Adapter {
  constructor() {
    this.track = null;
    this.poses = [];            // { t (ms, performance.now clock, capture time), x, y, h, conf }
    this.cmds = [[-1e12, 0, 0]]; // [t, left, right] motor commands as sent (after trim)
    this.samples = [];          // { t, v (cm/s), w (rad/s), cmd: [[l, r] per DELAYS] }
    this.n = 0;
    this.arrivals = [];
    this.est = null;
    this.dirty = false;
    this.lastSampleT = 0;
    this.hint = null;
    this.dir = 1;
    this.laps = [];
    this.lapStart = null;
    this.prevProg = null;
    this.maxProg = 0;
  }

  setTrack(center, half = 10, closed = true) {
    const T = new Track(center, half, closed);
    if (T.seg.length < 3 || !(half > 0)) return false;
    this.track = T;
    this.hint = null;
    this.prevProg = null;
    this.lapStart = null;
    this.maxProg = 0;
    return true;
  }

  // Record a motor command at the time it was queued for the robot.
  command(t, l, r) {
    const c = this.cmds;
    const last = c[c.length - 1];
    if (last[1] === l && last[2] === r) return;
    c.push([t, l, r]);
    while (c.length > 2 && c[1][0] < t - 5000) c.shift();
  }

  cmdAt(t) {
    const c = this.cmds;
    let i = c.length - 1;
    while (i > 0 && c[i][0] > t) i--;
    return [c[i][1], c[i][2]];
  }

  // time-weighted average command over [a, b]
  avgCmd(a, b) {
    const c = this.cmds;
    let i = c.length - 1;
    while (i > 0 && c[i][0] > a) i--;
    let sl = 0, sr = 0, t = a;
    for (; i < c.length && t < b; i++) {
      const end = i + 1 < c.length ? Math.min(c[i + 1][0], b) : b;
      if (end > t) { sl += c[i][1] * (end - t); sr += c[i][2] * (end - t); t = end; }
    }
    const span = b - a || 1;
    return [sl / span, sr / span];
  }

  maxAbsCmd(a, b) {
    const c = this.cmds;
    let i = c.length - 1;
    while (i > 0 && c[i][0] > a) i--;
    let mx = 0;
    for (; i < c.length && c[i][0] <= b; i++) mx = Math.max(mx, Math.abs(c[i][1]), Math.abs(c[i][2]));
    return mx;
  }

  // Our car among several tracked ones: by id, else the only one, else the one nearest the last pose.
  pick(poses, id) {
    const want = String(id || '').toLowerCase();
    if (want) { const r = poses.find((q) => q.id.toLowerCase() === want); if (r) return r; }
    if (poses.length === 1) return poses[0];
    const b = this.poses[this.poses.length - 1];
    if (!b) return null;
    let best = null, bd = 50;
    for (const q of poses) { const d = Math.hypot(q.x - b.x, q.y - b.y); if (d < bd) { bd = d; best = q; } }
    return best;
  }

  // Feed one message. opt.latency = assumed camera delay (ms) when the message has no t/age; opt.robotId.
  // Returns 'track', 'pose', 'lap' or null (ignored).
  ingest(msg, nowPerf, nowEpoch, opt = {}) {
    const m = parseMessage(msg);
    if (!m) return null;
    if (m.kind === 'track') return this.setTrack(m.center, m.half, m.closed) ? 'track' : null;
    const q = this.pick(m.poses, opt.robotId);
    if (!q) return null;
    let age = q.tPerf != null ? nowPerf - q.tPerf : q.t !== null ? nowEpoch - q.t : q.age;
    if (age === null || age < -50 || age > 3000) age = opt.latency ?? 120; // no or implausible timestamp
    const t = nowPerf - Math.max(0, age);
    const P = this.poses;
    const prev = P[P.length - 1];
    if (prev && t <= prev.t) return null; // late or duplicate frame
    let h = q.h;
    // Heading from the motion when the tracker gives none, or when it is 180° off (a symmetric outline).
    const ref = this.poseBefore(t - 150);
    if (ref && t - ref.t < 600) {
      const dx = q.x - ref.x, dy = q.y - ref.y;
      const fwd = (this.avgCmd(ref.t - 300, t)).reduce((a, b) => a + b, 0) / 2;
      if (Math.hypot(dx, dy) > 2 && Math.abs(fwd) > 5) {
        const course = Math.atan2(dy, dx) + (fwd < 0 ? Math.PI : 0);
        if (h === null) h = wrap(course);
        else if (Math.abs(wrap(course - h)) > 2.2) h = wrap(h + Math.PI);
      }
    }
    if (h === null && prev) h = prev.h;
    // Noisy positions (a hand-held camera) count as low confidence for calibration unless sigma is small.
    const conf = q.sigma != null && q.sigma > 15 ? Math.min(q.conf, 0.4) : q.conf;
    P.push({ t, x: q.x, y: q.y, h, conf, sig: q.sigma ?? 0.5 });
    while (P.length > 2 && P[0].t < t - 3000) P.shift();
    this.n++;
    this.arrivals.push(nowPerf);
    while (this.arrivals.length && this.arrivals[0] < nowPerf - 1000) this.arrivals.shift();
    this.addSample();
    return this.trackStep(t) ? 'lap' : 'pose';
  }

  poseBefore(t) {
    const P = this.poses;
    for (let i = P.length - 1; i >= 0; i--) if (P[i].t <= t) return P[i];
    return null;
  }

  // Lane position from the newest pose, travel direction and lap crossings. True when a lap was completed.
  trackStep(t) {
    const T = this.track;
    const b = this.poses[this.poses.length - 1];
    if (!T || !T.closed || b.h === null) return false;
    const L = T.locate(b.x, b.y, this.hint);
    this.hint = L.s;
    const c = Math.cos(b.h - L.th);
    if (c > 0.3) this.dir = 1; else if (c < -0.3) this.dir = -1;
    if (L.d > 2 * T.half) return false;
    const prog = this.dir > 0 ? L.s / T.len : 1 - L.s / T.len;
    let lap = false;
    if (this.prevProg !== null && this.prevProg > 0.9 && prog < 0.1) {
      if (this.lapStart !== null && this.maxProg > 0.85) { this.laps.push((t - this.lapStart) / 1000); lap = true; }
      if (this.lapStart === null || this.maxProg > 0.85) this.lapStart = t;
      this.maxProg = 0;
    } else if (prog > this.maxProg && prog - this.maxProg < 0.15) this.maxProg = prog;
    this.prevProg = prog;
    return lap;
  }

  // One calibration sample from the motion over the last WIN ms, if it tells us something.
  addSample() {
    const P = this.poses;
    const b = P[P.length - 1];
    if (b.h === null || b.conf < 0.5 || b.t - this.lastSampleT < SAMPLE_EVERY) return;
    // Longer windows for noisier positions, so the motion is large compared with the noise.
    const win = clamp(WIN * Math.max(1, b.sig / 2), WIN, 1000);
    let a = null;
    for (let i = P.length - 2; i >= 0; i--) {
      const dt = b.t - P[i].t;
      if (dt < win * 0.6) continue;
      if (dt > win * 1.6) break;
      if (!a || Math.abs(dt - win) < Math.abs(b.t - a.t - win)) a = P[i];
    }
    if (!a || a.h === null || a.conf < 0.5) return;
    const dt = (b.t - a.t) / 1000;
    const dh = wrap(b.h - a.h);
    const hm = a.h + dh / 2;
    const v = ((b.x - a.x) * Math.cos(hm) + (b.y - a.y) * Math.sin(hm)) / dt;
    const w = dh / dt;
    if (Math.abs(v) > 300 || Math.abs(w) > 15) return;          // tracking glitch, or the car was picked up
    if (this.maxAbsCmd(a.t - 700, b.t) < 1) return;             // motors off: parked, or pushed by hand
    this.samples.push({ t: b.t, v, w, cmd: DELAYS.map((d) => this.avgCmd(a.t - d, b.t - d)) });
    if (this.samples.length > MAX_SAMPLES) this.samples.shift();
    this.lastSampleT = b.t;
    this.dirty = true;
  }

  forget() { this.samples = []; this.est = null; this.dirty = false; }

  // Fit the car model to the samples. prior = { d0, W, d } (Tune values) for what the data can't pin down.
  // Returns est: { ok, n, d, d0, W, gL, gR, vmax, trim, trimOk, ident: { d, d0, W }, rmsV, rmsW }.
  fit(prior) {
    this.dirty = false;
    const all = this.samples;
    if (all.length < MIN_SAMPLES) return (this.est = { ok: false, n: all.length });
    const wrms = Math.sqrt(all.reduce((acc, s) => acc + s.w * s.w, 0) / all.length);
    const fitW = wrms > 0.4; // enough turning to tell the wheelbase
    // Coarse grid over delay x deadband (it also gives the error profiles), then the neighbours of the best.
    // Kept small: this runs on the phone's main thread next to the control loop.
    const prof = { d: new Map(), d0: new Map() };
    const grid = (S, dis, d0s, best = null) => {
      for (const di of dis) {
        for (const d0 of d0s) {
          const r = solve(S, di, d0, prior.W, fitW);
          if (!r || r.gL <= 0 || r.gR <= 0) continue;
          if (!best || r.err < best.err) best = { ...r, di, d0 };
          if (S === all) {
            if (!(prof.d.get(di) <= r.err)) prof.d.set(di, r.err);
            if (!(prof.d0.get(d0) <= r.err)) prof.d0.set(d0, r.err);
          }
        }
      }
      return best;
    };
    const near = (b) => [
      [b.di - 1, b.di, b.di + 1].filter((i) => i >= 0 && i < DELAYS.length),
      [b.d0 - 2, b.d0, b.d0 + 2].filter((d) => d >= DEADBANDS[0] && d <= DEADBANDS[DEADBANDS.length - 1]),
    ];
    let best = grid(all, DELAYS.map((_, i) => i).filter((i) => i % 2 === 0), DEADBANDS.filter((_, i) => i % 2 === 0));
    if (!best) return (this.est = { ok: false, n: all.length });
    best = grid(all, ...near(best), best);
    // Drop outliers (tracking glitches, bumps, the car held by hand), then fit again near the same spot.
    const cut = 6 * median(best.res);
    let S = all.filter((_, i) => best.res[i] <= cut);
    if (S.length >= MIN_SAMPLES) best = grid(S, ...near(best)) || best; else S = all;
    // A parameter counts as measured when moving it away from the best value makes the fit clearly worse.
    const peaked = (map, at, away) => {
      const e0 = map.get(at);
      const keys = [...map.keys()];
      const lo = keys.filter((k) => k <= at - away).map((k) => map.get(k));
      const hi = keys.filter((k) => k >= at + away).map((k) => map.get(k));
      const worse = (arr) => !arr.length || Math.min(...arr) > e0 * 1.05;
      return (lo.length || hi.length) > 0 && worse(lo) && worse(hi);
    };
    const ident = {
      d: peaked(prof.d, best.di, 4),
      d0: peaked(prof.d0, best.d0, 6),
      W: fitW,
    };
    // Fall back to the priors for whatever isn't measured, and solve again with those.
    const di = ident.d ? best.di : DELAYS.reduce((bi, d, i) => (Math.abs(d - prior.d) < Math.abs(DELAYS[bi] - prior.d) ? i : bi), 0);
    const d0 = ident.d0 ? best.d0 : prior.d0;
    const f = solve(S, di, d0, prior.W, fitW);
    if (!f || f.gL <= 0 || f.gR <= 0) return (this.est = { ok: false, n: all.length });
    const moving = S.map((s) => (Math.abs(s.cmd[di][0]) + Math.abs(s.cmd[di][1])) / 2).filter((m) => m > d0 + 3);
    const mref = clamp(median(moving) || 60, d0 + 10, 100);
    const trim = trimFor(f.gL, f.gR, d0, mref);
    // Trim must agree between the older and the newer half of the data.
    const h1 = solve(S.slice(0, S.length >> 1), di, d0, f.W, false);
    const h2 = solve(S.slice(S.length >> 1), di, d0, f.W, false);
    const t1 = h1 && trimFor(h1.gL, h1.gR, d0, mref), t2 = h2 && trimFor(h2.gL, h2.gR, d0, mref);
    const trimOk = S.length >= 60 && h1 && h2 && Math.abs(t1 - t2) < 1.5 && Math.abs(trim) < 25;
    return (this.est = {
      ok: true, n: all.length, kept: S.length, d: DELAYS[di], d0, W: f.W, gL: f.gL, gR: f.gR,
      vmax: ((f.gL + f.gR) / 2) * (100 - d0), trim, trimOk: !!trimOk, trimHalves: [t1, t2], mref,
      ident, rmsV: f.rmsV, rmsW: f.rmsW,
    });
  }

  // Where the car will be at tEnd: dead reckoning from the last pose with the commands sent since.
  // M = { gL, gR, d0, W, d } (car model). At most 600 ms ahead of the last pose.
  predict(b, tEnd, M) {
    let { x, y, h } = b;
    if (!M) return { x, y, h };
    const f = (m) => Math.sign(m) * Math.max(0, Math.abs(m) - M.d0);
    tEnd = Math.min(tEnd, b.t + 600);
    for (let t = b.t; t < tEnd; t += 10) {
      const dt = Math.min(10, tEnd - t) / 1000;
      const [ml, mr] = this.cmdAt(t - M.d);
      const vl = M.gL * f(ml), vr = M.gR * f(mr);
      const v = (vl + vr) / 2, w = (vr - vl) / M.W;
      h += (w * dt) / 2;
      x += Math.cos(h) * v * dt;
      y += Math.sin(h) * v * dt;
      h += (w * dt) / 2;
    }
    return { x, y, h: wrap(h) };
  }

  // speed and turn rate from the poses of the last ~300 ms
  motion() {
    const P = this.poses;
    const b = P[P.length - 1];
    const a = this.poseBefore(b.t - 250) || P[0];
    const dt = (b.t - a.t) / 1000;
    if (dt <= 0 || a.h === null || b.h === null) return { v: 0, w: 0 };
    const dh = wrap(b.h - a.h);
    const hm = a.h + dh / 2;
    return { v: ((b.x - a.x) * Math.cos(hm) + (b.y - a.y) * Math.sin(hm)) / dt, w: dh / dt };
  }

  // What autopilot scripts get as s.vis, or null before the first pose.
  // o = { lead (ms to predict past now), timeout (ms), look (cm), model (see predict) }
  state(now, o = {}) {
    const b = this.poses[this.poses.length - 1];
    if (!b || b.h === null) return null;
    const age = now - b.t;
    const P = this.predict(b, now + (o.lead || 0), o.model);
    const { v, w } = this.motion();
    const out = {
      fresh: age < (o.timeout || 500), age, x: P.x, y: P.y, h: P.h, raw: { x: b.x, y: b.y, h: b.h },
      v, w, conf: b.conf, n: this.n, hz: this.arrivals.length, laps: this.laps,
    };
    const T = this.track;
    if (!T) return out;
    const L = T.locate(P.x, P.y, this.hint);
    const dir = this.dir;
    const look = o.look || 20;
    out.e = L.e * dir;
    out.he = wrap(P.h - L.th - (dir < 0 ? Math.PI : 0));
    out.s = L.s;
    out.len = T.len;
    out.half = T.half;
    out.dir = dir;
    out.k = T.curvature(L.s) * dir;
    let kA = 0;
    for (let j = 0; j <= 2 * look; j += 4) { const k = T.curvature(L.s + dir * j) * dir; if (Math.abs(k) > Math.abs(kA)) kA = k; }
    out.kAhead = kA;
    const A = T.at(L.s + dir * look);
    const dx = A.x - P.x, dy = A.y - P.y;
    out.ahead = [dx * Math.cos(P.h) + dy * Math.sin(P.h), -dx * Math.sin(P.h) + dy * Math.cos(P.h)];
    return out;
  }
}

// Learned values (rrLearn.v1, docs/AUTONOMY-ARCHITECTURE.md §3.4): defaults before anything is learned, and bounds.
// trimLearned is added to the user's Trim slider. turnGain = 9 cm / wheelbase, for dead reckoning in scripts.
export const LEARN_DEF = { trimLearned: 0, vmax: 50, deadband: 22, wheelbase: 9, delay: 150 };
export const LEARN_LIM = { trimLearned: [-8, 8], vmax: [10, 150], deadband: [0, 50], wheelbase: [5, 20], delay: [0, 600] };

// Changes to the learned values toward the fit: bounded steps, or all the way (full, the Apply fit button).
// cur = current learned values; sliderTrim = the user's Trim (the fit gives the total trim the car needs).
export function proposeChanges(est, cur, sliderTrim = 0, { maxStep = 1, full = false } = {}) {
  const out = {};
  if (!est || !est.ok) return out;
  const set = (k, target, max, q) => {
    const [lo, hi] = LEARN_LIM[k];
    const now = Number.isFinite(cur[k]) ? cur[k] : LEARN_DEF[k];
    const d = clamp(target, lo, hi) - now;
    const v = clamp(Math.round((now + (full ? d : clamp(d, -max, max))) / q) * q, lo, hi);
    if (Math.abs(v - now) >= q - 1e-9) out[k] = Number(v.toFixed(2));
  };
  if (est.trimOk) set('trimLearned', est.trim - sliderTrim, maxStep, 0.5);
  set('vmax', est.vmax, 10, 1);
  if (est.ident.d0) set('deadband', est.d0, 3, 1);
  if (est.ident.W) set('wheelbase', est.W, 1, 0.5);
  if (est.ident.d) set('delay', est.d, 50, 10);
  return out;
}
