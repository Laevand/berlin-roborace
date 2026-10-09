// Headless autopilot runner: drives an autopilot script around the simulated "rally" track in virtual time,
// with the same ramp, line-query pipelining and Bluetooth delay model as the app. No browser needed.
//   node tools/simrun.mjs [script.js] [--secs=60] [--latency=70,200] [--link=steady,varying]
//                         [--seeds=3] [--vmax=50] [--apBase=40,50,60] [--apTurn=15] ...
//                         [--cam=80] [--skew=4] [--cal]
// Any other --key=a,b,c is an autopilot param; every combination is run. Prints one row per combination.
// --cam=ms feeds a fake camera (30 fps, that many ms late, a little noise) through adapt.js, so scripts get
// s.vis (autopilot/vision-pilot.js needs it). --skew makes the right wheel that % stronger. --cal prints the
// self-calibration fit of each run.
import { readFileSync } from 'node:fs';
import { SimCar, LinkModel, RALLY } from '../sim.js';
import { Adapter } from '../adapt.js';

const args = process.argv.slice(2);
const file = args.find((a) => !a.startsWith('--')) || 'autopilot/lane.js';
const opt = Object.fromEntries(args.filter((a) => a.startsWith('--')).map((a) => { const [k, v = ''] = a.slice(2).split('='); return [k, v.split(',')]; }));
const num = (k, d) => (opt[k] ? opt[k].map(Number) : [d]);
const secs = num('secs', 60)[0];
const seeds = num('seeds', 3)[0];
const links = opt.link || ['steady'];
const latencies = num('latency', 70);
const vmax = num('vmax', 50)[0];
const cam = num('cam', 0)[0];
const skew = num('skew', 0)[0];

const DEF = { trim: 0, minSpeed: 25, maxSpeed: 80, ramp: 50, apBase: 45, apTurn: 15, apHard: -35, apInvert: false,
  apDepth: 2, apCurve: 10, apCurveDecay: 1500, apTimeoutMs: 300, keepAliveMs: 400,
  visLook: 18, visGrip: 250, visTimeout: 500 };
const CAR = { vmax: 50, deadband: 22, wheelbase: 9, delay: 150 }; // learned-value defaults (rrLearn.v1)
const reserved = new Set(['secs', 'seeds', 'link', 'latency', 'vmax', 'cam', 'skew', 'cal']);
const grid = Object.keys(opt).filter((k) => !reserved.has(k));
const combos = grid.reduce((acc, k) => acc.flatMap((c) => opt[k].map((v) => ({ ...c, [k]: Number(v) }))), [{}]);

const src = readFileSync(file, 'utf8');
const mulberry = (a) => () => { a |= 0; a = (a + 0x6d2b79f5) | 0; let t = Math.imul(a ^ (a >>> 15), 1 | a); t = (t + Math.imul(t ^ (t >>> 7), 61 | t)) ^ t; return ((t ^ (t >>> 14)) >>> 0) / 4294967296; };
const clamp = (v, a, b) => Math.max(a, Math.min(b, v));

function run(p, link, latency, seed) {
  const rand = mulberry(seed);
  const car = new SimCar(() => ({ track: 'rally', vmax, skew }));
  const V = cam ? new Adapter() : null;
  if (V) V.setTrack(RALLY, 10);
  const g = CAR.vmax / (100 - CAR.deadband);
  const carModel = { gL: g, gR: g, d0: CAR.deadband, W: CAR.wheelbase, d: CAR.delay };
  const model = new LinkModel(rand);
  // eslint-disable-next-line no-new-func
  const fn = new Function('s', 'p', 'mem', 'ctx', '"use strict";\n' + src);
  const mem = {}, logs = [];
  const ctx = { send() {}, log: (m) => logs.push(String(m)) };
  const q = []; // [time ms, fn]
  let t = 0, out = [0, 0], apOut = null, lastMotor = 'S', lastMotorAt = -1e9, lineAt = -1e9, inflight = [];
  const at = (when, f) => q.push([when, f]);
  const half = () => model.rtt(t, link, latency) / 2 + 6 + rand() * 6;
  const sendMotor = (l, r) => at(t + half(), () => { car.l = l; car.r = r; });
  const queryLine = () => {
    inflight.push(t);
    at(t + half(), () => { const code = car.lineCode(); at(t + half(), () => onLine(code)); });
  };
  const ramped = (v, prev, step) => {
    if (!p.ramp || v === 0) return v;
    const from = Math.sign(v) === Math.sign(prev) ? Math.abs(prev) : 0;
    if (Math.abs(v) <= from) return v;
    return Math.sign(v) * Math.round(Math.min(Math.abs(v), Math.max(from, p.minSpeed) + step));
  };
  const pump = () => {
    while (inflight.length && t - inflight[0] > 400) inflight.shift(); // reply lost
    while (inflight.length < p.apDepth) queryLine();
  };
  function onLine(code) {
    inflight.shift();
    lineAt = t;
    const vis = V && V.poses.length ? V.state(t, { lead: CAR.delay, timeout: p.visTimeout, look: p.visLook, model: carModel }) : null;
    if (vis) vis.model = CAR;
    const s = { code, L: !!(code & 2), R: !!(code & 1), t, dt: mem._t ? t - mem._t : 0, dist: 0, distAge: 1e9, out: out.slice(), vis };
    mem._t = t;
    const r = fn(s, p, mem, ctx);
    apOut = [clamp(Number(r[0]) || 0, -100, 100), clamp(Number(r[1]) || 0, -100, 100)];
    drive();
    pump();
  }
  function drive() {
    const step = (p.ramp * Math.min(t - lastMotorAt, 200)) / 100;
    let [l, r] = apOut && t - lineAt < p.apTimeoutMs ? apOut : [0, 0];
    l = Math.round(clamp(l * (1 + p.trim / 100), -100, 100));
    r = Math.round(clamp(r * (1 - p.trim / 100), -100, 100));
    l = ramped(l, out[0], step);
    r = ramped(r, out[1], step);
    const cmd = l === 0 && r === 0 ? 'S' : `MS,${l},${r}`;
    if (cmd === lastMotor && t - lastMotorAt < p.keepAliveMs) return;
    lastMotor = cmd; lastMotorAt = t; out = [l, r];
    if (V) V.command(t, l, r);
    sendMotor(l, r);
  }
  pump();
  const DT = 5;
  for (t = 0; t < secs * 1000; t += DT) {
    q.sort((a, b) => a[0] - b[0]);
    while (q.length && q[0][0] <= t) q.shift()[1]();
    if (t % 50 === 0) { drive(); pump(); } // control tick: stale-data stop and lost-reply recovery
    if (V && t % 35 === 0) {
      const n = () => rand() - 0.5, cap = t;
      const msg = { type: 'pose', x: car.x + n(), y: car.y + n(), h: car.th + n() * 0.03, t: 1e12 + cap };
      at(t + cam, () => V.ingest(msg, cap + cam, 1e12 + cap + cam));
    }
    car.step(DT / 1000);
  }
  const est = V && opt.cal ? V.fit({ d0: CAR.deadband, W: CAR.wheelbase, d: CAR.delay }) : null;
  return { laps: car.laps, off: car.offTime, gaveUp: logs.some((m) => m.includes('stopped')), est };
}

const fmt = (x, n = 1) => (x === undefined ? '-' : x.toFixed(n));
console.log(`${file}  ${secs}s virtual, vmax ${vmax} cm/s, ${seeds} seeds`);
console.log(['link', 'rtt', ...grid, 'laps', 'best', 'avg', 'off s', 'gaveUp'].join('\t'));
for (const link of links) for (const lat of latencies) for (const c of combos) {
  const p = { ...DEF, ...c };
  const rs = Array.from({ length: seeds }, (_, i) => run(p, link, lat, i + 1));
  const laps = rs.flatMap((r) => r.laps);
  const best = laps.length ? Math.min(...laps) : undefined;
  const avg = laps.length ? laps.reduce((a, b) => a + b, 0) / laps.length : undefined;
  console.log([link, lat, ...grid.map((k) => c[k]), fmt(laps.length / seeds), fmt(best, 2), fmt(avg, 2),
    fmt(rs.reduce((a, r) => a + r.off, 0) / seeds), `${rs.filter((r) => r.gaveUp).length}/${seeds}`].join('\t'));
  for (const r of rs) {
    const e = r.est;
    if (e) console.log(e.ok ? `  fit ${e.n} samples: trim ${e.trim.toFixed(1)}${e.trimOk ? '' : '?'} vmax ${e.vmax.toFixed(0)} deadband ${e.d0}${e.ident.d0 ? '' : '?'} wheelbase ${e.W.toFixed(1)}${e.ident.W ? '' : '?'} delay ${e.d}${e.ident.d ? '' : '?'}` : `  fit: ${e.n} samples, not enough`);
  }
}
