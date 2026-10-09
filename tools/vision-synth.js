// Synthetic camera for tools/smoke.mjs: the rally track from sim.js rendered through a phone
// camera held by someone walking around the mat, following a robot that drives the lane. Every frame comes with
// ground truth (robot position and heading in the picture, its offset in the lane, the lane mask) to score the pipeline.
import { RALLY } from '../sim.js';

const HALF = 10;    // cm, lane half width
const EDGE = 1.2;   // cm, white edge line
const MARGIN = 30;  // cm of black mat around the track

const n = RALLY.length;
const S = [0];
for (let i = 0; i < n; i++) { const a = RALLY[i], b = RALLY[(i + 1) % n]; S.push(S[i] + Math.hypot(b[0] - a[0], b[1] - a[1])); }
const LEN = S[n];
const bx0 = Math.min(...RALLY.map((p) => p[0])) - MARGIN, by0 = Math.min(...RALLY.map((p) => p[1])) - MARGIN;
const GW = Math.ceil(Math.max(...RALLY.map((p) => p[0])) + MARGIN - bx0), GH = Math.ceil(Math.max(...RALLY.map((p) => p[1])) + MARGIN - by0);
// 1 cm grid: distance to the centerline and arc position (0..1) of the nearest point
const GD = new Float32Array(GW * GH).fill(99), GU = new Float32Array(GW * GH);
for (let i = 0; i < n; i++) {
  const [ax, ay] = RALLY[i], [bx, by] = RALLY[(i + 1) % n], vx = bx - ax, vy = by - ay, l2 = vx * vx + vy * vy || 1;
  const gx0 = Math.max(0, Math.floor(Math.min(ax, bx) - HALF - 2 - bx0)), gx1 = Math.min(GW - 1, Math.ceil(Math.max(ax, bx) + HALF + 2 - bx0));
  const gy0 = Math.max(0, Math.floor(Math.min(ay, by) - HALF - 2 - by0)), gy1 = Math.min(GH - 1, Math.ceil(Math.max(ay, by) + HALF + 2 - by0));
  for (let gy = gy0; gy <= gy1; gy++) {
    for (let gx = gx0; gx <= gx1; gx++) {
      const px = gx + 0.5 + bx0, py = gy + 0.5 + by0;
      const t = Math.max(0, Math.min(1, ((px - ax) * vx + (py - ay) * vy) / l2));
      const d = Math.hypot(px - ax - vx * t, py - ay - vy * t), k = gy * GW + gx;
      if (d < GD[k]) { GD[k] = d; GU[k] = (S[i] + t * (S[i + 1] - S[i])) / LEN; }
    }
  }
}

// point and unit tangent at arc length s
function along(s) {
  s = ((s % LEN) + LEN) % LEN;
  let lo = 0, hi = n;
  while (hi - lo > 1) { const m = (lo + hi) >> 1; if (S[m] <= s) lo = m; else hi = m; }
  const a = RALLY[lo], b = RALLY[(lo + 1) % n], l = S[lo + 1] - S[lo] || 1, t = (s - S[lo]) / l;
  return { x: a[0] + (b[0] - a[0]) * t, y: a[1] + (b[1] - a[1]) * t, tx: (b[0] - a[0]) / l, ty: (b[1] - a[1]) / l };
}

const mix = (a, b, t) => a.map((v, i) => v + (b[i] - v) * t);
const PINK = [235, 70, 160], PURPLE = [140, 60, 210], BLUE = [50, 110, 230];
const laneColor = (u) => (u < 0.5 ? mix(PINK, PURPLE, u * 2) : mix(PURPLE, BLUE, (u - 0.5) * 2));

export const CAMS = {
  follow: { dist: 70, height: 100, spin: 0.2 },   // walking after the robot, phone at chest height
  high: { dist: 35, height: 140, spin: 0.15 },    // held up high, looking down
  side: { dist: 150, height: 120, spin: 0.05 },   // standing back at the side of the mat
};

export class SynthCam {
  // beacon: my robot's headlights are green (what vision mode sets on the real car) instead of white.
  // others: more robots on the lane (white headlights), ahead of and behind mine, at the same speed.
  constructor(w, h, { cam = 'follow', speed = 35, fov = 63, seed = 1, beacon = true, others = 2 } = {}) {
    Object.assign(this, { w, h, cam, speed, F: w / 2 / Math.tan((fov * Math.PI) / 360), seed, beacon });
    this.cars = [{ s0: 0, wob: 5, ph: 0, beacon }, { s0: 75, wob: 4, ph: 2 }, { s0: -90, wob: 3, ph: 4 }, { s0: 170, wob: 4, ph: 1 }].slice(0, 1 + others);
  }

  rand() { // mulberry32, deterministic noise
    let t = (this.seed = (this.seed + 0x6d2b79f5) | 0);
    t = Math.imul(t ^ (t >>> 15), t | 1);
    t ^= t + Math.imul(t ^ (t >>> 7), t | 61);
    return ((t ^ (t >>> 14)) >>> 0) / 4294967296;
  }

  robotAt(t, car = this.cars[0]) {
    const s = car.s0 + this.speed * t, off = car.wob * Math.sin(0.9 * t + car.ph); // cm left of the centerline
    const c = along(s), c2 = along(s + 1), d = car.wob * 0.9 * Math.cos(0.9 * t + car.ph) / this.speed;
    const th = Math.atan2(c.ty, c.tx) + Math.atan(d);
    return { x: c.x - c.ty * off, y: c.y + c.tx * off, th, off, tx: c2.tx, ty: c2.ty };
  }

  cameraAt(t, rob) {
    const k = CAMS[this.cam] || CAMS.follow, az = -Math.PI / 2 + Math.sin(k.spin * t) * 1.4;
    const pos = [rob.x + Math.cos(az) * k.dist, rob.y + Math.sin(az) * k.dist, k.height + 4 * Math.sin(1.7 * t)];
    const tg = [rob.x + 8 * Math.cos(rob.th) + 4 * Math.sin(2.3 * t), rob.y + 8 * Math.sin(rob.th) + 4 * Math.cos(1.9 * t), 0];
    const f = norm([tg[0] - pos[0], tg[1] - pos[1], tg[2] - pos[2]]);
    const roll = 0.08 * Math.sin(0.7 * t);
    let r = norm([f[1], -f[0], 0]);
    let u = cross(r, f);
    [r, u] = [add(scale(r, Math.cos(roll)), scale(u, Math.sin(roll))), add(scale(u, Math.cos(roll)), scale(r, -Math.sin(roll)))];
    return { pos, f, r, u };
  }

  project(c, X, Y) {
    const d = [X - c.pos[0], Y - c.pos[1], -c.pos[2]], z = dot(d, c.f);
    if (z <= 1) return null;
    return [this.w / 2 + (this.F * dot(d, c.r)) / z, this.h / 2 - (this.F * dot(d, c.u)) / z];
  }

  // renders frame t (seconds) into out (RGBA); returns ground truth
  render(t, out) {
    const { w, h, F } = this, robs = this.cars.map((car) => ({ ...this.robotAt(t, car), beacon: car.beacon }));
    const rob = robs[0], c = this.cameraAt(t, rob);
    for (const r of robs) { r.cs = Math.cos(r.th); r.sn = Math.sin(r.th); }
    const lane = new Uint8Array(w * h);
    for (let py = 0; py < h; py++) {
      for (let px = 0; px < w; px++) {
        const ax = px + 0.5 - w / 2, ay = py + 0.5 - h / 2;
        const rx = F * c.f[0] + ax * c.r[0] - ay * c.u[0], ry = F * c.f[1] + ax * c.r[1] - ay * c.u[1], rz = F * c.f[2] + ax * c.r[2] - ay * c.u[2];
        let col;
        if (rz > -1e-6) col = [70, 72, 80]; // wall
        else {
          const tt = -c.pos[2] / rz, X = c.pos[0] + tt * rx, Y = c.pos[1] + tt * ry;
          let a = 99, b = 99, car = null;
          for (const r of robs) {
            const dx = X - r.x, dy = Y - r.y, ra = dx * r.cs + dy * r.sn, rb = -dx * r.sn + dy * r.cs;
            if (Math.abs(ra) <= 5 && Math.abs(rb) <= 4.5) { a = ra; b = rb; car = r; break; }
          }
          const gx = Math.floor(X - bx0), gy = Math.floor(Y - by0);
          const inMat = gx >= 0 && gy >= 0 && gx < GW && gy < GH;
          const d = inMat ? GD[gy * GW + gx] : 99;
          if (car) {
            col = a > 3.8 && Math.abs(b) > 2.6 ? (car.beacon ? [80, 245, 120] : [255, 235, 190]) : Math.abs(b) > 3.6 && Math.abs(a) < 2.5 ? [12, 12, 12] : Math.abs(a) < 2 && Math.abs(b) < 2.2 ? [45, 50, 45] : [28, 28, 32];
            if (d <= HALF) lane[py * w + px] = 1;
          } else if (!inMat) col = [150, 140, 125]; // booth floor
          else if (d <= HALF - EDGE) { col = laneColor(GU[gy * GW + gx]); lane[py * w + px] = 1; }
          else if (d <= HALF) { col = [235, 235, 235]; lane[py * w + px] = 1; }
          else col = [16, 16, 20]; // mat
        }
        const vig = 1 - 0.3 * ((ax * ax) / (w * w) + (ay * ay) / (h * h)) * 2, j = (py * w + px) * 4;
        for (let k = 0; k < 3; k++) out[j + k] = col[k] * vig + (this.rand() - 0.5) * 16;
        out[j + 3] = 255;
      }
    }
    const inPic = (q) => q && q[0] >= 0 && q[1] >= 0 && q[0] < w && q[1] < h;
    const others = robs.slice(1).map((r) => this.project(c, r.x, r.y)).filter(inPic);
    const p0 = this.project(c, rob.x, rob.y), p1 = this.project(c, rob.x + 5 * rob.cs, rob.y + 5 * rob.sn);
    let heading = null;
    if (p0 && p1) { const l = Math.hypot(p1[0] - p0[0], p1[1] - p0[1]) || 1; heading = [(p1[0] - p0[0]) / l, (p1[1] - p0[1]) / l]; }
    return { robot: inPic(p0) ? p0 : null, others, heading, offset: -rob.off / HALF, lane };
  }
}

const dot = (a, b) => a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
const cross = (a, b) => [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
const add = (a, b) => [a[0] + b[0], a[1] + b[1], a[2] + b[2]];
const scale = (a, k) => [a[0] * k, a[1] * k, a[2] * k];
const norm = (a) => { const l = Math.hypot(...a) || 1; return [a[0] / l, a[1] / l, a[2] / l]; };

// score one frame of the pipeline against the ground truth
export function score(res, gt, drive) {
  let inter = 0, uni = 0;
  for (let i = 0; i < gt.lane.length; i++) { const a = drive[i], b = gt.lane[i]; inter += a & b; uni += a | b; }
  const out = { iou: uni ? inter / uni : 1 };
  if (gt.robot) {
    out.err = res.robot ? Math.hypot(res.robot[0] - gt.robot[0], res.robot[1] - gt.robot[1]) / (res.Wr || res.W) : Infinity;
    out.ok = out.err < 0.5; // the track marked mine is on my robot
    if (res.offset != null && Math.abs(gt.offset) > 0.3) out.sideOk = Math.sign(res.offset) === Math.sign(gt.offset);
    if (res.heading && gt.heading) out.headOk = res.heading[0] * gt.heading[0] + res.heading[1] * gt.heading[1] > 0.7;
  }
  // every robot in view has a box on it
  out.seen = 0;
  out.all = 0;
  for (const q of [gt.robot, ...(gt.others || [])]) {
    if (!q) continue;
    out.all++;
    if (res.tracks.some((tr) => Math.hypot(tr.x - q[0], tr.y - q[1]) < 0.5 * res.W)) out.seen++;
  }
  return out;
}
