// Roborace Pit: Web Bluetooth controller for the Next App / droidcon Cutebot firmware (microbitapi.js).
// Runs on iPhone inside the Bluefy browser (Safari has no Web Bluetooth). Plain ES module, no build step:
// push to main, wait for GitHub Pages, tap "New build" (or "Pull from repo" for autopilot scripts).

const T = window.BUILD_T || Date.now();
const UART_SERVICE = '6e400001-b5a3-f393-e0a9-e50e24dcca9e';
const UART_RX = '6e400003-b5a3-f393-e0a9-e50e24dcca9e'; // phone -> robot (write)
const UART_TX = '6e400002-b5a3-f393-e0a9-e50e24dcca9e'; // robot -> phone (indicate)
const MAX_CHUNK = 20; // micro:bit UART characteristic takes 20 bytes per write

const $ = (id) => document.getElementById(id);
const clamp = (v, lo, hi) => Math.max(lo, Math.min(hi, v));
const now = () => performance.now();
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

// ---------------------------------------------------------------- settings

const PARAMS = [
  { g: 'Calibrate', k: 'trim', label: 'Trim (+ = left wheel faster)', min: -25, max: 25, step: 0.5, def: 0, help: 'Drifts right on a straight? Lower it. Drifts left? Raise it.' },
  { g: 'Calibrate', k: 'testSpeed', label: 'Straight test speed', min: 25, max: 100, step: 1, def: 60 },
  { g: 'Calibrate', k: 'testMs', label: 'Straight test duration (ms)', min: 300, max: 4000, step: 100, def: 1500 },

  { g: 'Drive', k: 'maxSpeed', label: 'Max speed', min: 25, max: 100, step: 1, def: 80 },
  { g: 'Drive', k: 'minSpeed', label: 'Start speed (deadband)', min: 0, max: 50, step: 1, def: 25, help: 'Smallest non-zero motor value. Below ~25 the wheels only hum.' },
  { g: 'Drive', k: 'steerGain', label: 'Steer strength', min: 0.1, max: 1, step: 0.05, def: 0.6, help: '0.5 = inner wheel stops at full lock, 1 = inner wheel reverses.' },
  { g: 'Drive', k: 'steerExpo', label: 'Steer expo', min: 0, max: 1, step: 0.05, def: 0.35, help: 'Higher = softer response near the center.' },
  { g: 'Drive', k: 'pivot', label: 'Spin speed when not moving', min: 0, max: 1, step: 0.05, def: 0.5 },
  { g: 'Drive', k: 'ramp', label: 'Acceleration limit (motor units per 100 ms, 0 = off)', min: 0, max: 200, step: 5, def: 50, help: 'Stops wheelies and brownout reboots when you floor it. Lower = gentler. Braking is never limited.' },
  { g: 'Drive', k: 'padRadius', label: 'Joystick travel (px)', min: 30, max: 160, step: 5, def: 80 },
  { g: 'Drive', k: 'layout', label: 'Layout', type: 'select', options: ['dual', 'stick'], def: 'dual', help: 'dual = steer with left thumb, throttle with right. stick = one floating joystick.' },
  { g: 'Drive', k: 'tiltRange', label: 'Tilt for full lock (°)', min: 10, max: 60, step: 1, def: 30 },
  { g: 'Drive', k: 'tiltInvert', label: 'Invert tilt', type: 'bool', def: false },

  { g: 'Autopilot', k: 'apBase', label: 'Base speed', min: 0, max: 100, step: 1, def: 45 },
  { g: 'Autopilot', k: 'apTurn', label: 'Inner wheel, soft turn', min: -100, max: 100, step: 1, def: 15 },
  { g: 'Autopilot', k: 'apHard', label: 'Inner wheel, line lost', min: -100, max: 100, step: 1, def: -35 },
  { g: 'Autopilot', k: 'apLostMs', label: 'Line lost → hard turn after (ms)', min: 0, max: 1000, step: 10, def: 150 },
  { g: 'Autopilot', k: 'apInvert', label: 'Invert sensors (follow a white line on a dark floor)', type: 'bool', def: false },
  { g: 'Autopilot', k: 'apOverride', label: 'Touch pads to override Auto', type: 'bool', def: true },
  { g: 'Autopilot', k: 'apStopDist', label: 'Stop for obstacle closer than (cm, 0 = off)', min: 0, max: 50, step: 1, def: 0 },
  { g: 'Autopilot', k: 'apDepth', label: 'Line queries in flight', min: 1, max: 4, step: 1, def: 2, help: 'More = more sensor readings per second (BLE round trip is ~70 ms on iPhone), same delay per reading.' },
  { g: 'Autopilot', k: 'apDistEvery', label: 'Read distance every N line reads (0 = never)', min: 0, max: 50, step: 1, def: 0, help: '?DIST blocks the robot for up to ~30 ms, so keep this off unless you need it.' },
  { g: 'Autopilot', k: 'apCurve', label: 'Curve learning (% per edge reading)', min: 0, max: 40, step: 1, def: 10, help: 'Lane keeper: how fast it learns which way the track bends. 0 = just bounce off the edges.' },
  { g: 'Autopilot', k: 'apCurveDecay', label: 'Curve memory (ms)', min: 200, max: 5000, step: 100, def: 1500, help: 'Lane keeper: how long a learned bend lasts once the edges stop being touched.' },
  { g: 'Autopilot', k: 'apTimeoutMs', label: 'Stop if no sensor data for (ms)', min: 100, max: 1000, step: 10, def: 300 },
  { g: 'Autopilot', k: 'visLook', label: 'Vision pilot: look-ahead (cm)', min: 5, max: 60, step: 1, def: 18, help: 'Vision pilot aims at the lane center this far ahead. Longer = smoother, cuts corners more.' },
  { g: 'Autopilot', k: 'visGrip', label: 'Vision pilot: corner grip (cm/s²)', min: 50, max: 800, step: 10, def: 250, help: 'Slows down before bends so the sideways acceleration stays below this. Higher = faster corners.' },

  { g: 'Vision', k: 'visCal', label: 'Self-calibrate from the camera', type: 'select', options: ['off', 'suggest', 'auto'], def: 'suggest', help: 'Learned values sit on top of your sliders and never move them. off = ignore them (driving uses the sliders only). suggest = the Log says what the camera measured, Apply fit takes it. auto = learn by itself in small steps while you drive.' },
  { g: 'Vision', k: 'visMaxStep', label: 'Auto: largest learned-trim change per step', min: 0.5, max: 5, step: 0.5, def: 1 },
  { g: 'Vision', k: 'visLatency', label: 'Camera delay when the feed has no timestamps (ms)', min: 0, max: 1000, step: 10, def: 120 },
  { g: 'Vision', k: 'visTimeout', label: 'Camera counts as lost after (ms)', min: 100, max: 2000, step: 50, def: 500 },
  { g: 'Vision', k: 'visLaps', label: 'Camera taps Lap when the car crosses the start', type: 'bool', def: true },
  { g: 'Vision', k: 'visUrl', label: 'Feed URL (wss://… or an https:// event stream; empty = this page or another tab)', type: 'text', def: '' },

  { g: 'Link', k: 'tickMs', label: 'Control tick (ms)', min: 20, max: 150, step: 5, def: 50, help: 'In manual mode motors are updated at most once per tick. The firmware docs suggest 50–100.' },
  { g: 'Link', k: 'telemetry', label: 'Telemetry in manual mode', type: 'bool', def: true },
  { g: 'Link', k: 'telEvery', label: 'Telemetry query every N ticks', min: 1, max: 10, step: 1, def: 2 },
  { g: 'Link', k: 'keepAliveMs', label: 'Resend motor state every (ms)', min: 100, max: 2000, step: 50, def: 400 },
  { g: 'Link', k: 'withResponse', label: 'Write with response (slower, for debugging)', type: 'bool', def: false },
  { g: 'Link', k: 'autoReconnect', label: 'Auto-reconnect after a drop (brownout)', type: 'bool', def: true },

  { g: 'Lights', k: 'fx', label: 'Turn signals, brake light, underglow', type: 'bool', def: true },
  { g: 'Lights', k: 'headlight', label: 'Headlight brightness', min: 0, max: 255, step: 5, def: 80 },
  { g: 'Lights', k: 'blinkMs', label: 'Blink period (ms)', min: 150, max: 800, step: 10, def: 300 },
  { g: 'Lights', k: 'signalAt', label: 'Signal when steering above', min: 0.05, max: 0.9, step: 0.05, def: 0.25 },
];

const store = {
  get(k, d) {
    try { const v = localStorage.getItem('rr.' + k); return v == null ? d : JSON.parse(v); } catch { return d; }
  },
  set(k, v) {
    try { localStorage.setItem('rr.' + k, JSON.stringify(v)); } catch { /* private mode */ }
  },
};

const defaults = () => Object.fromEntries(PARAMS.map((x) => [x.k, x.def]));
const p = Object.assign(defaults(), pick(store.get('params', {})));
// v2: acceleration limit became on by default after wheelies at speed ~80 on the real robot.
if (store.get('paramsV', 1) < 2) {
  if (p.ramp === 0) p.ramp = 50;
  store.set('paramsV', 2);
}
function pick(obj) {
  const out = {};
  for (const x of PARAMS) if (obj && x.k in obj) out[x.k] = obj[x.k];
  return out;
}
const saveParams = () => store.set('params', p);

// ---------------------------------------------------------------- log

const logBuf = [];
let logDirty = false;
function log(kind, msg) {
  logBuf.push([kind, `${(now() / 1000).toFixed(2)}  ${msg}`]);
  if (logBuf.length > 500) logBuf.splice(0, logBuf.length - 500);
  logDirty = true;
}
const esc = (s) => s.replace(/[&<>]/g, (c) => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;' }[c]));
function renderLog() {
  if (!logDirty || !$('tab-log').classList.contains('on')) return;
  logDirty = false;
  const el = $('log');
  const atBottom = el.scrollTop + el.clientHeight >= el.scrollHeight - 30;
  el.innerHTML = logBuf.map(([k, m]) => `<span class="${k}">${esc(m)}</span>`).join('\n');
  if (atBottom) el.scrollTop = el.scrollHeight;
}
window.addEventListener('error', (e) => log('err', `JS error: ${e.message} (${e.filename?.split('/').pop()}:${e.lineno})`));
window.addEventListener('unhandledrejection', (e) => log('err', `Unhandled: ${e.reason?.message || e.reason}`));

// ---------------------------------------------------------------- link (queue + framing)

class Link {
  constructor() {
    this.t = null;          // transport: { connected, write(str), close() }
    this.urgent = [];       // stop commands, sent first
    this.motor = null;      // latest motor command, older ones are dropped
    this.fifo = [];         // everything else, in order
    this.busy = false;
    this.rxBuf = '';
    this.onMessage = () => {};
    this.txCount = 0;
    this.errCount = 0;
    this.rxCount = 0;
  }
  get connected() { return !!(this.t && this.t.connected); }
  backlog() { return this.urgent.length + this.fifo.length + (this.motor ? 1 : 0) + (this.busy ? 1 : 0); }
  send(cmd) {
    if (this.fifo.length > 30) this.fifo.shift();
    this.fifo.push(cmd);
    this.pump();
  }
  setMotor(cmd) { this.motor = cmd; this.pump(); }
  stopNow() {
    this.motor = null;
    this.fifo.length = 0;
    this.urgent.push('S');
    this.pump();
  }
  clear() { this.urgent.length = 0; this.fifo.length = 0; this.motor = null; this.rxBuf = ''; }
  async pump() {
    if (this.busy || !this.connected) return;
    this.busy = true;
    let cmd = null;
    let isMotor = false;
    try {
      for (;;) {
        isMotor = false;
        if (this.urgent.length) cmd = this.urgent.shift();
        else if (this.motor) { cmd = this.motor; this.motor = null; isMotor = true; }
        else if (this.fifo.length) cmd = this.fifo.shift();
        else break;
        if (!this.connected) break;
        await this.t.write(cmd + '#');
        this.txCount++;
        if (ui.traffic) log('tx', '→ ' + cmd);
      }
    } catch (e) {
      this.errCount++;
      log('err', `write "${cmd}" failed: ${e.message || e}`);
      if (isMotor && !this.motor) this.motor = cmd; // retry the latest motor state
      setTimeout(() => this.pump(), 20);
    } finally {
      this.busy = false;
    }
  }
  onText(text) {
    this.rxBuf += text;
    let i;
    while ((i = this.rxBuf.indexOf('\n')) >= 0) {
      const line = this.rxBuf.slice(0, i).trim().replace(/#$/, '');
      this.rxBuf = this.rxBuf.slice(i + 1);
      if (!line) continue;
      this.rxCount++;
      if (ui.traffic) log('rx', '← ' + line);
      this.onMessage(line);
    }
    if (this.rxBuf.length > 256) this.rxBuf = '';
  }
}

class BleTransport {
  constructor(link, onState) {
    this.link = link;
    this.onState = onState;
    this.device = null;
    this.rx = null;
    this.tx = null;
    this.connected = false;
    this.userClosed = false;
    this.enc = new TextEncoder();
    this.dec = new TextDecoder();
    this.onValue = (e) => this.link.onText(this.dec.decode(e.target.value));
    this.onGone = () => this.handleDisconnect();
  }
  static supported() { return !!(navigator.bluetooth && navigator.bluetooth.requestDevice); }
  async request(robotId) {
    const id = (robotId || '').trim();
    const filters = id ? [{ name: `BBC micro:bit [${id}]` }] : [{ namePrefix: 'BBC micro:bit' }];
    const device = await navigator.bluetooth.requestDevice({ filters, optionalServices: [UART_SERVICE] });
    await this.attach(device);
  }
  async attach(device) {
    if (this.device) this.device.removeEventListener('gattserverdisconnected', this.onGone);
    this.device = device;
    this.device.addEventListener('gattserverdisconnected', this.onGone);
    this.userClosed = false;
    await this.open();
  }
  async open() {
    this.onState('connecting');
    const server = await this.device.gatt.connect();
    const svc = await server.getPrimaryService(UART_SERVICE);
    this.rx = await svc.getCharacteristic(UART_RX);
    const tx = await svc.getCharacteristic(UART_TX);
    if (this.tx) this.tx.removeEventListener('characteristicvaluechanged', this.onValue);
    this.tx = tx;
    this.tx.removeEventListener('characteristicvaluechanged', this.onValue);
    this.tx.addEventListener('characteristicvaluechanged', this.onValue);
    await this.tx.startNotifications();
    this.canNoResp = typeof this.rx.writeValueWithoutResponse === 'function' &&
      (!this.rx.properties || this.rx.properties.writeWithoutResponse !== false);
    this.connected = true;
    this.onState('connected');
  }
  async write(str) {
    const bytes = this.enc.encode(str);
    for (let i = 0; i < bytes.length; i += MAX_CHUNK) {
      const chunk = bytes.slice(i, i + MAX_CHUNK);
      if (this.canNoResp && !p.withResponse) await this.rx.writeValueWithoutResponse(chunk);
      else if (this.rx.writeValueWithResponse) await this.rx.writeValueWithResponse(chunk);
      else await this.rx.writeValue(chunk);
    }
  }
  get name() { return this.device?.name || 'micro:bit'; }
  close() {
    this.userClosed = true;
    this.connected = false;
    try { this.device?.gatt?.disconnect(); } catch { /* already gone */ }
  }
  async handleDisconnect() {
    const wasUp = this.connected;
    this.connected = false;
    this.onState('disconnected');
    if (this.userClosed || !p.autoReconnect || !wasUp) return;
    log('err', 'Connection dropped. If the robot showed a sad then happy face, the batteries are low.');
    for (let i = 0; i < 20 && !this.userClosed && !this.connected; i++) {
      this.onState('reconnecting');
      try { await this.open(); log('ap', 'Reconnected'); return; } catch (e) { await sleep(Math.min(400 * (i + 1), 2000)); }
    }
    if (!this.connected) this.onState('disconnected');
  }
}

// ---------------------------------------------------------------- state

const ui = { traffic: false, tab: 'drive' };
const S = {
  mode: 'manual',            // manual | assist | auto
  armed: false,              // auto mode running
  steer: 0, thr: 0,          // current input, -1..1
  out: [0, 0],               // last motor values sent
  lastMotor: '', lastMotorAt: 0,
  apOut: null, apFn: null, mem: {},
  lineSent: [], lineReads: 0, lineTimes: [], lineHist: [],
  pingAt: 0,
  tel: { line: null, lineAt: 0, dist: null, distAt: 0, accel: null, light: null, temp: null, ping: null },
  fx: {}, fxPrevAvg: 0, brakeUntil: 0,
  telN: 0, rot: 0,
  tilt: { on: false, zero: null, steer: 0, lastEvent: 0 },
  gp: { prev: [] },
};
const link = new Link();
link.onMessage = onMessage;
let transport = null;

// ---------------------------------------------------------------- telemetry in

const waiters = {};
// Resolves with the reply's value (e.g. "3" for LINE:3, "" for PONG), or null after ms.
function expect(key, ms) {
  return new Promise((res) => {
    const timer = setTimeout(() => { if (waiters[key]?.res === res) delete waiters[key]; res(null); }, ms);
    waiters[key] = { res, timer };
  });
}

function onMessage(msg) {
  const i = msg.indexOf(':');
  const key = i < 0 ? msg : msg.slice(0, i);
  const val = i < 0 ? '' : msg.slice(i + 1);
  const t = now();
  const w = waiters[key];
  if (w) { delete waiters[key]; clearTimeout(w.timer); w.res(val); }
  S.telAt = 0;
  switch (key) {
    case 'LINE': {
      const code = parseInt(val, 10) & 3;
      S.tel.line = code; S.tel.lineAt = t;
      S.lineSent.shift();
      if (S.onLineTest) S.onLineTest();
      S.lineTimes.push(t);
      S.lineHist.push([t, code]);
      if (S.lineHist.length > 400) S.lineHist.splice(0, 100);
      if (S.mode !== 'manual') {
        runAutopilot(code, t);
        drive(t);
        pumpLine(t);
      }
      break;
    }
    case 'DIST': S.tel.dist = parseInt(val, 10); S.tel.distAt = t; break;
    case 'ACCEL': S.tel.accel = val.split(',').map(Number); break;
    case 'LIGHT': S.tel.light = parseInt(val, 10); break;
    case 'TEMP': S.tel.temp = parseInt(val, 10); break;
    case 'COMPASS': log('rx', 'compass ' + val); break;
    case 'PONG':
      if (S.pingAt) { S.tel.ping = t - S.pingAt; S.tel.pingT = t; S.pingAt = 0; }
      if (ui.pingManual) { log('rx', `PONG ${S.tel.ping?.toFixed(0)} ms`); ui.pingManual = false; }
      break;
    default:
      log('rx', msg);
  }
}

function query(q, t = now()) {
  if (q === 'PING') {
    if (S.pingAt && t - S.pingAt < 2000) return false;
    S.pingAt = t;
  }
  link.send(q);
  return true;
}

// Keeps apDepth ?LINE queries in flight, so readings arrive faster than one BLE round trip.
function pumpLine(t) {
  while (S.lineSent.length && t - S.lineSent[0] > 400) S.lineSent.shift(); // reply lost
  while (S.lineSent.length < p.apDepth) queryLine(t);
}

function queryLine(t) {
  S.lineSent.push(t);
  S.lineReads++;
  link.send('?LINE');
  if (p.apDistEvery > 0 && S.lineReads % p.apDistEvery === 0) link.send('?DIST');
  if (S.lineReads % 40 === 0) query('PING', t);
}

// ---------------------------------------------------------------- autopilot

function compileScript(src) {
  // eslint-disable-next-line no-new-func
  return new Function('s', 'p', 'mem', 'ctx', '"use strict";\n' + src);
}

const apCtx = {
  send: (cmd) => link.send(String(cmd)),
  log: (msg) => log('ap', String(msg)),
};

function runAutopilot(code, t) {
  if (!S.apFn) return;
  const onL = !!(code & 2) !== !!p.apInvert;
  const onR = !!(code & 1) !== !!p.apInvert;
  const s = {
    code, L: onL, R: onR, t,
    dt: S.mem._t ? t - S.mem._t : 0,
    dist: S.tel.dist, distAge: t - S.tel.distAt,
    out: S.out.slice(),
    vis: visState(t),
    // the camera's raw measurement, window.__rrVision v1 (docs/AUTONOMY-ARCHITECTURE.md §3.2)
    vision: window.__rrVision && window.__rrVision.robot && t - window.__rrVision.t < 1000 ? window.__rrVision : null,
  };
  S.mem._t = t;
  try {
    const r = S.apFn(s, p, S.mem, apCtx);
    if (Array.isArray(r) && r.length >= 2) {
      S.apOut = [clamp(Number(r[0]) || 0, -100, 100), clamp(Number(r[1]) || 0, -100, 100)];
    }
  } catch (e) {
    S.apOut = [0, 0];
    if (S.armed) disarm('script error');
    log('err', 'autopilot: ' + e.message);
    $('apStatus').textContent = 'Error: ' + e.message;
  }
}

// ---------------------------------------------------------------- vision feedback (adapt.js, VISION.md)

// A camera pipeline reports the car's pose. It reaches autopilot scripts as s.vis, and the fit of what the car
// did against the motor commands sent learns trim, speed, deadband, wheelbase and delay (Tune → Vision).
let V = null; // Adapter from adapt.js once loaded; until then (or if it fails) everything here is a no-op
const vis = { src: '', sock: null, timer: 0, calAt: 0, applyAt: 0, sugAt: 0, sugKey: '', renderAt: 0, lastObj: null };

// Learned values live under one versioned key shared with autopilot scripts (rrLearn.v1). They are added to the
// user's sliders, never written into them. Other keys in it belong to other code and are kept on every write.
const LEARN_KEY = 'rrLearn.v1';
const LEARN_MINE = ['trimLearned', 'vmax', 'deadband', 'wheelbase', 'delay', 'turnGain'];
const LEARN_DEF0 = { trimLearned: 0, vmax: 50, deadband: 22, wheelbase: 9, delay: 150 }; // = LEARN_DEF in adapt.js
function learnLoad() {
  try { const o = JSON.parse(localStorage.getItem(LEARN_KEY) || '{}'); return o && typeof o === 'object' ? o : {}; } catch { return {}; }
}
let learned = learnLoad();
const lv = (k) => (Number.isFinite(learned[k]) ? learned[k] : LEARN_DEF0[k]);
function learnWrite(ch, reset = false) {
  const o = learnLoad();
  if (reset) for (const k of LEARN_MINE) delete o[k];
  Object.assign(o, ch);
  if (!reset) { o.turnGain = Number((9 / (o.wheelbase || 9)).toFixed(3)); o.at = Date.now(); }
  learned = o;
  try { localStorage.setItem(LEARN_KEY, JSON.stringify(o)); } catch { /* private mode */ }
}

async function loadVision() {
  try {
    const m = await import(`./adapt.js?t=${T}`);
    V = new m.Adapter();
    vis.propose = m.proposeChanges;
  } catch (e) {
    log('err', 'Vision module failed to load: ' + e.message);
    return;
  }
  try { new BroadcastChannel('rr-vision').onmessage = (e) => visionIngest(e.data, 'tab'); } catch { /* unsupported */ }
  window.addEventListener('message', (e) => { if (e.origin === location.origin && e.data && e.data.rrVision) visionIngest(e.data.rrVision, 'page'); });
  visionConnect();
  setInterval(visCalTick, 1000);
}

// The in-page camera publishes window.__rrVision (a new object per frame, poses in the RALLY frame).
function pollCamera() {
  const m = window.__rrVision;
  if (!V || !m || m === vis.lastObj) return;
  vis.lastObj = m;
  if (!V.track && !vis.trackLoading) {
    vis.trackLoading = true;
    import(`./sim.js?t=${T}`).then((s) => { if (!V.track) visionIngest({ type: 'track', ...s.trackCenter('rally') }, 'camera'); }).catch(() => { vis.trackLoading = false; });
  }
  visionIngest(m, 'camera');
}

// One message (object, JSON text, or an array of them) from any source. Returns what it was, or null.
function visionIngest(msg, src = 'api') {
  if (!V) return null;
  if (Array.isArray(msg)) return msg.map((m) => visionIngest(m, src)).pop() ?? null;
  let r = null;
  try {
    r = V.ingest(msg, now(), Date.now(), { latency: p.visLatency, robotId: ui.name });
  } catch (e) {
    if (now() - (vis.errAt || 0) > 5000) { vis.errAt = now(); log('err', 'vision: ' + e.message); }
  }
  if (!r) return null;
  vis.src = src;
  if (r === 'track') log('ap', `vision: track outline ${V.track.len.toFixed(0)} cm long, lane ${(2 * V.track.half).toFixed(0)} cm wide (${src})`);
  if (r === 'lap') {
    log('ap', `vision: lap ${V.laps.length} in ${V.laps[V.laps.length - 1].toFixed(2)} s`);
    if (p.visLaps && lap.running) lapStartStop();
  }
  return r;
}

function visionConnect() {
  clearTimeout(vis.timer);
  const old = vis.sock;
  vis.sock = null;
  try { old?.close(); } catch { /* already closed */ }
  const url = String(p.visUrl || '').trim();
  if (!url || !V) return;
  let s;
  try { s = /^wss?:/i.test(url) ? new WebSocket(url) : new EventSource(url); } catch (e) { log('err', `vision feed ${url}: ${e.message}`); return; }
  vis.sock = s;
  s.onopen = () => { vis.errLogged = false; log('ap', 'vision feed connected: ' + url); };
  s.onmessage = (e) => { if (typeof e.data === 'string') visionIngest(e.data, 'net'); };
  s.onerror = () => { if (vis.sock === s && !vis.errLogged) { vis.errLogged = true; log('err', `vision feed ${url}: can't connect, retrying`); } };
  if (typeof s.send === 'function') s.onclose = () => { if (vis.sock === s) vis.timer = setTimeout(visionConnect, 2000); }; // EventSource retries by itself
}

// Car model for predicting the pose: Tune values, with the left/right balance from the fit.
// Car model for predicting the pose: learned values, with the left/right balance from the fit.
function visModel() {
  const g = lv('vmax') / Math.max(1, 100 - lv('deadband'));
  const e = V.est;
  const m = e && e.ok ? (e.gL + e.gR) / 2 : 0;
  return { gL: m ? (g * e.gL) / m : g, gR: m ? (g * e.gR) / m : g, d0: lv('deadband'), W: lv('wheelbase'), d: lv('delay') };
}

// s.vis: the pose predicted to when the next command lands (lead), lane error, bend ahead, and the learned
// car model (s.vis.model) for turning wheel speeds into motor values. Null without a feed.
function visState(t, lead = lv('delay')) {
  if (!V || !V.poses.length) return null;
  try {
    const st = V.state(t, { lead, timeout: p.visTimeout, look: p.visLook, model: visModel() });
    if (st) st.model = { vmax: lv('vmax'), deadband: lv('deadband'), wheelbase: lv('wheelbase'), delay: lv('delay') };
    return st;
  } catch (e) {
    return null;
  }
}

function visCalTick() {
  if (!V) return;
  const t = now();
  try {
    if (V.dirty && t - vis.calAt > 2000) { vis.calAt = t; V.fit({ d0: lv('deadband'), W: lv('wheelbase'), d: lv('delay') }); }
    if (!V.est?.ok) return;
    if (p.visCal === 'auto' && t - vis.applyAt > 3000) {
      vis.applyAt = t;
      visApply(vis.propose(V.est, learned, p.trim, { maxStep: p.visMaxStep }), 'learned');
    } else if (p.visCal === 'suggest' && t - vis.sugAt > 15000) {
      const ch = vis.propose(V.est, learned, p.trim, { full: true });
      const key = JSON.stringify(ch);
      if (Object.keys(ch).length && key !== vis.sugKey) {
        vis.sugAt = t;
        vis.sugKey = key;
        log('ap', `vision suggests ${Object.entries(ch).map(([k, v]) => `${k} ${lv(k)} → ${v}`).join(', ')} (${V.est.n} samples). Tune → Vision → Apply fit.`);
      }
    }
  } catch (e) {
    log('err', 'vision calibration: ' + e.message);
  }
}

// Stores learned values (rrLearn.v1). The sliders are never touched.
function visApply(ch, why) {
  const keys = Object.keys(ch);
  if (!keys.length) return;
  log('ap', `vision ${why}: ${keys.map((k) => `${k} ${lv(k)} → ${ch[k]}`).join(', ')}`);
  learnWrite(ch);
}

function visReset() {
  log('ap', `vision: learned values reset (were ${LEARN_MINE.filter((k) => k in learned).map((k) => `${k} ${learned[k]}`).join(', ') || 'none'})`);
  learnWrite({}, true);
}

function renderVision(t) {
  const st = visState(t, 0);
  const sign = (v, n = 0) => (v >= 0 ? '+' : '') + v.toFixed(n);
  $('vCam').textContent = !st ? '–' : st.fresh ? `${st.hz} fps` : 'lost';
  $('vLane').textContent = st && st.fresh && st.e != null ? `${sign(st.e)} cm` : '–';
  if (ui.tab !== 'tune' || t - vis.renderAt < 300) return;
  vis.renderAt = t;
  $('visStatus').textContent = !st ? 'No camera feed yet (VISION.md says how to send one).'
    : `${vis.src} · ${st.hz} fps · ${st.age.toFixed(0)} ms late${st.fresh ? '' : ' (lost)'} · at ${st.raw.x.toFixed(0)},${st.raw.y.toFixed(0)} cm ` +
      `${((st.raw.h * 180) / Math.PI).toFixed(0)}° · ${st.v.toFixed(0)} cm/s` +
      (st.e != null ? ` · lane ${sign(st.e)} cm · ${(st.s / st.len * 100).toFixed(0)}% round` : ' · no track outline') +
      (V.laps.length ? ` · laps ${V.laps.map((x) => x.toFixed(1)).join(' ')}` : '');
  const e = V.est;
  const q = (ok) => (ok ? '' : ' (assumed)');
  $('visFit').textContent = !e || !e.ok ? `Collecting: ${V.samples.length}/40 samples. Drive around in any mode while the camera sees the car.`
    : `Fit from ${e.n} samples: trim ${sign(e.trim, 1)} in total${e.trimOk ? '' : ' (unsure)'} · ${e.vmax.toFixed(0)} cm/s at 100 · ` +
      `deadband ${e.d0}${q(e.ident.d0)} · wheelbase ${e.W.toFixed(1)} cm${q(e.ident.W)} · delay ${e.d} ms${q(e.ident.d)} · ` +
      `off by ${e.rmsV.toFixed(1)} cm/s, ${e.rmsW.toFixed(2)} rad/s`;
  $('visLearned').textContent = `Learned${p.visCal === 'off' ? ' (ignored, Self-calibrate is off)' : ''}: trim ${sign(lv('trimLearned'), 1)} on top of the slider's ${sign(p.trim, 1)} · ` +
    `${lv('vmax')} cm/s at 100 · deadband ${lv('deadband')} · wheelbase ${lv('wheelbase')} cm · delay ${lv('delay')} ms${learned.at ? '' : ' (defaults, nothing learned yet)'}`;
}

// ---------------------------------------------------------------- driving

function expo(v, e) { return Math.sign(v) * ((1 - e) * Math.abs(v) + e * Math.abs(v) ** 3); }

// throttle/steer in -1..1 -> wheel fractions in -1..1
function mix(thr, steer) {
  const s = expo(steer, p.steerExpo);
  if (Math.abs(thr) < 0.08) return [s * p.pivot, -s * p.pivot];
  const inner = thr * (1 - 2 * p.steerGain * Math.abs(s));
  return s >= 0 ? [thr, inner] : [inner, thr];
}

// wheel fraction -> motor command, skipping the deadband
function toMotor(f) {
  const a = Math.min(1, Math.abs(f));
  if (a < 0.02) return 0;
  return Math.sign(f) * (p.minSpeed + (p.maxSpeed - p.minSpeed) * a);
}

// Trim slider plus the trim learned from the camera (0 until something is learned, or with Self-calibrate off).
const learnedTrim = () => (p.visCal === 'off' ? 0 : clamp(lv('trimLearned'), -8, 8));
function trimmed([l, r]) {
  const tr = p.trim + learnedTrim();
  const L = Math.round(clamp(l * (1 + tr / 100), -100, 100));
  const R = Math.round(clamp(r * (1 - tr / 100), -100, 100));
  return [L, R];
}

function manual() { return trimmed(mix(S.thr, S.steer).map(toMotor)); }

function target(t) {
  readInputs();
  if (S.testDrive) {
    // Touching either pad cancels the test.
    if (t < S.testDrive && !padSteer.active && !padThrottle.active) return trimmed([p.testSpeed, p.testSpeed]);
    S.testDrive = 0;
  }
  if (S.mode === 'manual') return manual();
  const fresh = S.apOut && t - S.tel.lineAt < p.apTimeoutMs;
  if (S.mode === 'assist') {
    // You hold the throttle, the autopilot steers. Steering hard yourself overrides it. Reverse is manual.
    if (S.thr < 0.02 || !fresh || Math.abs(S.steer) > 0.3) return manual();
    const base = Math.max(1, Math.abs(p.apBase));
    return trimmed(S.apOut.map((v) => toMotor(clamp((S.thr * v) / base, -1, 1))));
  }
  // Auto: touching a pad (or the gamepad) takes over while held; letting go hands control back.
  // A connected gamepad only counts while a stick or trigger is actually moved.
  const gpMoving = S.gp.active && (Math.abs(S.gp.steer) > 0 || Math.abs(S.gp.thr) > 0.02);
  if (!S.armed || (p.apOverride && (padSteer.active || padThrottle.active || gpMoving))) return manual();
  if (!fresh) return [0, 0];
  if (p.apStopDist > 0 && S.tel.dist > 0 && S.tel.dist < p.apStopDist && t - S.tel.distAt < 600) return [0, 0];
  return trimmed(S.apOut);
}

// Limits how fast a wheel can speed up (softens current spikes that brown out weak batteries).
function ramped(v, prev, step) {
  if (!p.ramp || v === 0) return v;
  const from = Math.sign(v) === Math.sign(prev) ? Math.abs(prev) : 0;
  if (Math.abs(v) <= from) return v;
  return Math.sign(v) * Math.round(Math.min(Math.abs(v), Math.max(from, p.minSpeed) + step));
}

function drive(t = now()) {
  if (!link.connected) return;
  const step = (p.ramp * Math.min(t - S.lastMotorAt, 200)) / 100; // S.out changes only when we send
  let [l, r] = target(t);
  l = ramped(l, S.out[0], step);
  r = ramped(r, S.out[1], step);
  const cmd = l === 0 && r === 0 ? 'S' : `MS,${l},${r}`;
  if (cmd === S.lastMotor && t - S.lastMotorAt < p.keepAliveMs) return;
  S.lastMotor = cmd;
  S.lastMotorAt = t;
  S.out = [l, r];
  if (V) V.command(t, l, r);
  link.setMotor(cmd);
}

function emergencyStop(why) {
  disarm(why);
  S.testDrive = 0;
  S.out = [0, 0];
  S.lastMotor = 'S';
  S.lastMotorAt = now();
  if (V) V.command(S.lastMotorAt, 0, 0);
  if (link.connected) link.stopNow();
  if (why) log('ap', 'STOP: ' + why);
}

function arm() {
  if (!link.connected) { log('err', 'Not connected'); return; }
  if (!S.apFn) { log('err', 'No autopilot script applied'); return; }
  S.mem = {};
  S.apOut = null;
  S.armed = true;
  pumpLine(now());
  if (!lap.running) lapStartStop();
  renderModeUi();
}
function disarm(why) {
  if (!S.armed) return;
  S.armed = false;
  renderModeUi();
  if (why) log('ap', 'Autopilot off: ' + why);
}

function setMode(m) {
  emergencyStop();
  S.mode = m;
  S.lineSent = [];
  S.apOut = null;
  S.mem = {};
  renderModeUi();
}

// ---------------------------------------------------------------- tests

// Measures what the real BLE link can do: ping, and how fast the LINE sensor loop can run.
async function linkTest() {
  if (!link.connected) { log('err', 'Link test: connect first'); return; }
  if (S.testing) return;
  setMode('manual');
  S.testing = true;
  const errs0 = link.errCount;
  log('ap', 'Link test running, motors off, about 5 s…');
  try {
    await sleep(300); // let replies to earlier telemetry queries drain
    const pings = [];
    for (let i = 0; i < 10; i++) {
      const t0 = now();
      link.send('PING');
      if ((await expect('PONG', 1000)) !== null) pings.push(now() - t0);
    }
    const rtts = [];
    let lost = 0;
    const end = now() + 3000;
    while (now() < end) {
      const t0 = now();
      link.send('?LINE');
      if ((await expect('LINE', 500)) === null) lost++; else rtts.push(now() - t0);
    }
    // Pipelined: 3 queries in flight, like the autopilot loop with apDepth 3.
    const sent = [];
    const lat = [];
    S.onLineTest = () => { if (sent.length) lat.push(now() - sent.shift()); };
    const end2 = now() + 3000;
    while (now() < end2) {
      while (sent.length < 3) { sent.push(now()); link.send('?LINE'); }
      await sleep(5);
    }
    await sleep(500);
    S.onLineTest = null;
    const stat = (a) => (a.length ? `${Math.min(...a).toFixed(0)}/${(a.reduce((x, y) => x + y, 0) / a.length).toFixed(0)}/${Math.max(...a).toFixed(0)} ms` : 'none');
    log('ap', `LINK TEST: ping min/avg/max ${stat(pings)} (${pings.length}/10 answered) | ` +
      `LINE round trip ${stat(rtts)} = ${(rtts.length / 3).toFixed(0)} Hz, ${lost} lost | ` +
      `LINE x3 in flight ${(lat.length / 3).toFixed(0)} Hz, delay ${stat(lat)}, ${sent.length} lost | write errors ${link.errCount - errs0} | ` +
      `robot ${transport?.name || '?'} | ${(navigator.userAgent.match(/iPhone OS [\d_]+/) || [''])[0]}`);
  } finally {
    S.testing = false;
  }
}

// Drives straight at the test speed for a fixed time so trim can be tuned without fighting the joystick.
function straightTest() {
  if (!link.connected) { log('err', 'Straight test: connect first'); return; }
  setMode('manual');
  S.testDrive = now() + p.testMs;
  log('ap', `Straight test: speed ${p.testSpeed} for ${p.testMs} ms, trim ${p.trim}${learnedTrim() ? ` + learned ${learnedTrim()}` : ''}`);
}

async function copyLog() {
  const text = logBuf.map(([, m]) => m).join('\n');
  try {
    await navigator.clipboard.writeText(text);
    log('ap', `Copied ${logBuf.length} log lines to the clipboard`);
  } catch {
    log('err', 'Clipboard blocked: long-press the log text to select and copy it.');
  }
}

// ---------------------------------------------------------------- lights

const AMBER = '255,110,0';
const OFF = '0,0,0';
function speedColor(v) {
  if (v < 1) return '24,0,48';
  // track gradient: blue -> purple -> pink
  const f = Math.round(clamp(v / 100, 0, 1) * 4) / 4;
  const stops = [[61, 123, 255], [176, 76, 255], [255, 50, 160]];
  const x = f * 2;
  const a = stops[Math.floor(Math.min(x, 1.999))];
  const b = stops[Math.floor(Math.min(x, 1.999)) + 1];
  const k = x - Math.floor(Math.min(x, 1.999));
  return a.map((c, i) => Math.round(c + (b[i] - c) * k)).join(',');
}

function fxTick(t) {
  if (!p.fx || !link.connected) return;
  const [l, r] = S.out;
  const avg = (l + r) / 2;
  if (avg < S.fxPrevAvg - 12 || (S.fxPrevAvg > 20 && Math.abs(avg) < 1)) S.brakeUntil = t + 400;
  if (avg > S.fxPrevAvg + 6) S.accelUntil = t + 300;
  S.fxPrevAvg = avg;
  const big = Math.max(Math.abs(l), Math.abs(r), 1);
  const steer = S.mode === 'manual' ? S.steer : (l - r) / big;
  const blinkOn = Math.floor(t / p.blinkMs) % 2 === 0;
  const hb = p.headlight;
  const white = `${hb},${hb},${hb}`;
  // Left/right are swapped on the robot's headlight LEDs: turning right must blink the right one.
  const turning = Math.abs(steer) > p.signalAt;
  // Underglow shows intent: red = braking/reversing, green = accelerating,
  // amber pulse in time with the indicator = turning, else the speed gradient.
  let ug = speedColor(Math.abs(avg));
  if (avg < -1 || t < S.brakeUntil) ug = '255,0,0';
  else if (t < (S.accelUntil || 0)) ug = '0,255,70';
  else if (turning && Math.abs(avg) > 1) ug = blinkOn ? AMBER : ug;
  const want = {
    HLL: steer > p.signalAt ? (blinkOn ? AMBER : OFF) : white,
    HLR: steer < -p.signalAt ? (blinkOn ? AMBER : OFF) : white,
    UG: ug,
  };
  for (const k of ['HLL', 'HLR', 'UG']) {
    if (S.fx[k] === want[k]) continue;
    if (link.backlog() > 1) return;
    S.fx[k] = want[k];
    link.send(`${k},${want[k]}`);
    return; // one light command per tick keeps the motor channel clear
  }
}

// ---------------------------------------------------------------- control loop

const ROTA = ['?LINE', '?DIST', '?LINE', '?ACCEL', '?LINE', 'PING', '?LINE', '?LIGHT', '?LINE', '?TEMP'];
let tickTimer = null;
function restartTick() {
  clearInterval(tickTimer);
  tickTimer = setInterval(tick, p.tickMs);
}
function tick() {
  const t = now();
  pollGamepad();
  if (V) pollCamera();
  if (!link.connected) return;
  drive(t);
  if (S.testing) return;
  if (S.mode === 'manual') {
    // One telemetry query in flight at most. On a slow link, queued replies make the robot's
    // command handler wait for Bluetooth, which delays motor commands.
    if (p.telemetry && ++S.telN % p.telEvery === 0 && link.backlog() === 0 && (!S.telAt || t - S.telAt > 600)) {
      if (query(ROTA[S.rot++ % ROTA.length], t)) S.telAt = t;
    }
  } else {
    pumpLine(t);
  }
  fxTick(t);
}

// ---------------------------------------------------------------- inputs

class Pad {
  constructor(el, onRelease) {
    this.el = el;
    this.knob = el.querySelector('.knob');
    this.id = null;
    this.x = 0;
    this.y = 0;
    el.addEventListener('pointerdown', (e) => {
      if (this.id !== null) return;
      e.preventDefault();
      this.id = e.pointerId;
      try { el.setPointerCapture(e.pointerId); } catch { /* not capturable */ }
      this.rect = el.getBoundingClientRect();
      this.ox = e.clientX;
      this.oy = e.clientY;
      el.classList.add('active');
      this.move(e);
    });
    el.addEventListener('pointermove', (e) => { if (e.pointerId === this.id) this.move(e); });
    const end = (e) => {
      if (e.pointerId !== this.id) return;
      this.id = null;
      this.x = 0;
      this.y = 0;
      el.classList.remove('active');
      this.draw();
      onRelease();
    };
    el.addEventListener('pointerup', end);
    el.addEventListener('pointercancel', end);
    el.addEventListener('lostpointercapture', end);
  }
  get active() { return this.id !== null; }
  move(e) {
    const R = p.padRadius;
    this.x = clamp((e.clientX - this.ox) / R, -1, 1);
    this.y = clamp(-(e.clientY - this.oy) / R, -1, 1);
    this.draw();
  }
  draw() {
    if (!this.active) { this.knob.style.left = '50%'; this.knob.style.top = '50%'; return; }
    const R = p.padRadius;
    this.knob.style.left = `${this.ox - this.rect.left + this.x * R}px`;
    this.knob.style.top = `${this.oy - this.rect.top - this.y * R}px`;
  }
}

let padSteer;
let padThrottle;

function readInputs() {
  let steer = 0;
  let thr = 0;
  if (p.layout === 'stick') { steer = padThrottle.x; thr = padThrottle.y; } else { steer = padSteer.x; thr = padThrottle.y; }
  if (S.tilt.on && !(p.layout === 'dual' && padSteer.active)) steer = S.tilt.steer;
  if (S.gp.active) { steer = S.gp.steer; thr = S.gp.thr; }
  S.steer = steer;
  S.thr = thr;
}

function onPadRelease() {
  // Lifting your thumb should stop the car right away, not on the next tick.
  if (S.mode !== 'auto' || !S.armed || p.apOverride) drive(now());
}

async function toggleTilt() {
  if (S.tilt.on) {
    S.tilt.on = false;
    window.removeEventListener('devicemotion', onMotion);
  } else {
    try {
      if (typeof DeviceMotionEvent !== 'undefined' && typeof DeviceMotionEvent.requestPermission === 'function') {
        const r = await DeviceMotionEvent.requestPermission();
        if (r !== 'granted') throw new Error('permission ' + r);
      }
      S.tilt.zero = null;
      S.tilt.on = true;
      window.addEventListener('devicemotion', onMotion);
      setTimeout(() => { if (S.tilt.on && !S.tilt.lastEvent) log('err', 'No motion data from this browser; tilt steering unavailable.'); }, 1500);
    } catch (e) {
      log('err', 'Tilt: ' + e.message);
    }
  }
  $('btnTilt').textContent = S.tilt.on ? 'Tilt: on' : 'Tilt: off';
}

function onMotion(e) {
  const g = e.accelerationIncludingGravity;
  if (!g || g.x == null) return;
  S.tilt.lastEvent = now();
  const ang = (Math.atan2(g.x, g.y) * 180) / Math.PI;
  if (S.tilt.zero == null) S.tilt.zero = ang;
  const d = ((ang - S.tilt.zero + 540) % 360) - 180;
  S.tilt.steer = clamp(d / p.tiltRange, -1, 1) * (p.tiltInvert ? -1 : 1);
}

function pollGamepad() {
  const list = navigator.getGamepads ? navigator.getGamepads() : [];
  let gp = null;
  for (const g of list) if (g && g.connected) { gp = g; break; }
  if (!gp) { S.gp.active = false; return; }
  const dz = (v) => (Math.abs(v) < 0.12 ? 0 : v);
  const btn = (i) => gp.buttons[i] || { value: 0, pressed: false };
  const trig = btn(7).value - btn(6).value;
  S.gp.steer = dz(gp.axes[0] || 0);
  S.gp.thr = Math.abs(trig) > 0.02 ? trig : -dz(gp.axes[3] || 0);
  S.gp.active = true;
  const pressed = gp.buttons.map((b) => b.pressed);
  const edge = (i) => pressed[i] && !S.gp.prev[i];
  if (edge(0)) link.send('HORN');
  if (edge(1)) emergencyStop('gamepad');
  if (edge(9)) (S.armed ? disarm('gamepad') : arm());
  if (edge(3)) lapStartStop();
  S.gp.prev = pressed;
}

// ---------------------------------------------------------------- lap timer

const lap = { running: false, start: 0, last: 0, laps: [] };
function lapStartStop() {
  const t = now();
  if (!lap.running) {
    lap.running = true;
    lap.start = lap.last = t;
    $('btnLap').textContent = 'Lap';
  } else {
    lap.laps.unshift(((t - lap.last) / 1000).toFixed(2));
    lap.last = t;
    $('lapList').textContent = lap.laps.map((x, i) => `L${lap.laps.length - i}: ${x}`).join('  ');
  }
}
function lapReset() {
  lap.running = false;
  lap.laps = [];
  $('btnLap').textContent = 'Start';
  $('lapTime').textContent = '0.00';
  $('lapList').textContent = '';
}

// ---------------------------------------------------------------- connection ui

function setState(st) {
  const dot = $('dot');
  dot.className = 'dot' + (st === 'connected' ? ' ok' : st === 'connecting' || st === 'reconnecting' ? ' busy' : '');
  const name = transport?.name?.match(/\[(.+)\]/)?.[1] || transport?.name || '';
  ui.name = name;
  $('status').textContent = { connected: name || 'Connected', connecting: 'Connecting…', reconnecting: 'Reconnecting…', disconnected: 'Offline' }[st];
  $('btnConnect').textContent = st === 'connected' || st === 'reconnecting' ? 'Disconnect' : 'Connect';
  if (st === 'connected') {
    link.clear();
    S.fx = {};
    S.lastMotor = '';
    S.lineSent = [];
    link.pump();
    wakeLock();
    log('ap', `Connected to ${transport?.name}`);
  } else {
    disarm(st === 'disconnected' ? 'disconnected' : '');
  }
}

async function connect() {
  if (transport && (transport.connected || $('btnConnect').textContent === 'Disconnect')) {
    emergencyStop();
    await sleep(80);
    transport.close();
    setState('disconnected');
    return;
  }
  if (!BleTransport.supported()) {
    log('err', 'No Web Bluetooth here. On iPhone open this page in the Bluefy browser.');
    return;
  }
  try {
    const bt = new BleTransport(link, setState);
    transport = bt;
    link.t = bt;
    await bt.request(store.get('robotId', ''));
    store.set('lastDevice', bt.device.id);
  } catch (e) {
    const why = [e?.name, e?.message].filter(Boolean).join(': ') || String(e);
    log('err', `Connect failed (${why}). Check: Bluetooth on, Bluefy allowed to use Bluetooth (iPhone Settings → Bluefy), ` +
      'Robot ID correct, and no other phone connected to this robot. Cancelling the device list also lands here.');
    setState('disconnected');
  }
}

async function tryAutoReconnect() {
  // Lets a page reload (new build) reconnect without the device picker where the browser supports getDevices().
  if (!BleTransport.supported() || !navigator.bluetooth.getDevices) return;
  try {
    const last = store.get('lastDevice', null);
    const devices = await navigator.bluetooth.getDevices();
    const d = devices.find((x) => x.id === last);
    if (!d) return;
    const bt = new BleTransport(link, setState);
    transport = bt;
    link.t = bt;
    await bt.attach(d);
  } catch (e) {
    setState('disconnected');
  }
}

// Test hook only (tools/smoke.mjs): drives a fake robot from sim.js. Nothing in the UI starts it. Change rr.simOpts live.
const simOpts = { latency: 70, link: 'steady', skew: 0, vision: true };
async function startSim(opts = {}) {
  Object.assign(simOpts, opts);
  const { SimTransport, trackCenter } = await import(`./sim.js?t=${T}`);
  const canvas = $('sim');
  canvas.classList.remove('hidden');
  const sim = new SimTransport((text) => link.onText(text), canvas, () => simOpts);
  transport = sim;
  link.t = sim;
  setState('connected');
  // Fake camera: the simulated car's pose 30 times a second, 80 ms late, with a little noise, through the
  // same path as a real feed. The track outline goes first.
  let sentTrack = false;
  const cam = setInterval(() => {
    if (!sim.connected) { clearInterval(cam); return; }
    if (!simOpts.vision || !V) return;
    if (!sentTrack) {
      sentTrack = true;
      visionIngest({ type: 'track', ...trackCenter() }, 'sim');
    }
    const n = () => Math.random() - 0.5;
    const msg = { type: 'pose', x: sim.x + n(), y: sim.y + n(), h: sim.th + n() * 0.03, t: Date.now() };
    setTimeout(() => visionIngest(msg, 'sim'), 80);
  }, 33);
}

let wl = null;
async function wakeLock() {
  try { if (navigator.wakeLock && !wl) { wl = await navigator.wakeLock.request('screen'); wl.addEventListener('release', () => { wl = null; }); } } catch { /* unsupported */ }
}

// The camera page lives in the Vision tab. It loads on first use and stops the camera when you leave the tab.
let visionMod = null;
async function showVision(on) {
  try {
    if (on && !visionMod) visionMod = import(`./vision.js?t=${T}`);
    const m = await visionMod;
    m?.setActive(on && ui.tab === 'vision');
  } catch (e) { log('err', 'Vision failed to load: ' + (e.message || e)); visionMod = null; }
}

// ---------------------------------------------------------------- forms

function buildForm(container, group) {
  const groups = [...new Set(PARAMS.map((x) => x.g))].filter((g) => !group || g === group);
  for (const g of groups) {
    const fs = document.createElement('fieldset');
    fs.innerHTML = `<legend>${g}</legend>`;
    fs.dataset.group = g;
    for (const x of PARAMS.filter((y) => y.g === g)) {
      const lab = document.createElement('label');
      const help = x.help ? `<span class="help">${x.help}</span>` : '';
      if (x.type === 'bool') {
        lab.className = 'inline';
        lab.innerHTML = `<input type="checkbox" data-k="${x.k}"> ${x.label}`;
      } else if (x.type === 'select') {
        lab.innerHTML = `${x.label} <select data-k="${x.k}">${x.options.map((o) => `<option>${o}</option>`).join('')}</select>${help}`;
      } else if (x.type === 'text') {
        lab.innerHTML = `${x.label}<input type="text" data-k="${x.k}" autocapitalize="off" autocorrect="off" spellcheck="false" style="width:100%">${help}`;
      } else {
        lab.innerHTML = `${x.label}<div class="slider"><input type="range" data-k="${x.k}" min="${x.min}" max="${x.max}" step="${x.step}">` +
          `<input type="number" data-k="${x.k}" min="${x.min}" max="${x.max}" step="${x.step}" inputmode="decimal"></div>${help}`;
      }
      fs.appendChild(lab);
      if (x.type === 'bool' && x.help) fs.insertAdjacentHTML('beforeend', help);
    }
    container.appendChild(fs);
  }
}

function syncInputs(k) {
  for (const el of document.querySelectorAll(`[data-k="${k}"]`)) {
    if (el === document.activeElement && el.type === 'number') continue;
    if (el.type === 'checkbox') el.checked = !!p[k];
    else el.value = p[k];
  }
}

function onParamInput(e) {
  const el = e.target;
  const k = el.dataset.k;
  const def = PARAMS.find((x) => x.k === k);
  if (!def) return;
  let v;
  if (def.type === 'bool') v = el.checked;
  else if (def.type === 'select' || def.type === 'text') v = el.value;
  else { v = parseFloat(el.value); if (Number.isNaN(v)) return; v = clamp(v, def.min, def.max); }
  p[k] = v;
  saveParams();
  syncInputs(k);
  onParamChanged(k);
}

function onParamChanged(k) {
  if (k === 'tickMs') restartTick();
  if (k === 'visUrl') { clearTimeout(vis.urlTimer); vis.urlTimer = setTimeout(visionConnect, 800); }
  if (k === 'layout') applyLayout();
  if (k === 'fx') {
    $('btnFx').textContent = p.fx ? 'FX: on' : 'FX: off';
    S.fx = {};
    if (!p.fx && link.connected) link.send('HO');
  }
}

function applyLayout() { $('tab-drive').classList.toggle('stick', p.layout === 'stick'); }

// ---------------------------------------------------------------- autopilot scripts ui

let scripts = [];
async function loadScripts(applySelected) {
  try {
    const idx = await (await fetch(`autopilot/index.json?t=${Date.now()}`, { cache: 'no-store' })).json();
    for (const it of idx) it.src = await (await fetch(`autopilot/${it.file}?t=${Date.now()}`, { cache: 'no-store' })).text();
    scripts = idx;
    $('apStatus').textContent = `Pulled ${idx.length} scripts`;
  } catch (e) {
    log('err', 'Could not load autopilot scripts: ' + e.message);
  }
  const sel = $('apSelect');
  const cur = store.get('apSel', scripts[0]?.file || 'custom');
  sel.innerHTML = scripts.map((s) => `<option value="${s.file}">${esc(s.name)}</option>`).join('') + '<option value="custom">My script (edited on phone)</option>';
  sel.value = [...sel.options].some((o) => o.value === cur) ? cur : sel.options[0].value;
  if (applySelected) selectScript(sel.value, true);
}

function scriptSrc(key) {
  if (key === 'custom') return store.get('apCustom', scripts[0]?.src || 'return [0, 0]');
  return scripts.find((s) => s.file === key)?.src ?? '';
}

function selectScript(key, apply) {
  store.set('apSel', key);
  $('apSelect').value = key;
  $('apCode').value = scriptSrc(key);
  if (apply) applyScript(false);
}

function applyScript(fromEditor) {
  const src = $('apCode').value;
  const key = $('apSelect').value;
  if (fromEditor && key !== 'custom' && src !== scriptSrc(key)) {
    store.set('apCustom', src);
    store.set('apSel', 'custom');
    $('apSelect').value = 'custom';
  } else if (key === 'custom') {
    store.set('apCustom', src);
  }
  try {
    S.apFn = compileScript(src);
    S.mem = {};
    $('apStatus').textContent = `Applied "${$('apSelect').selectedOptions[0]?.textContent}" at ${new Date().toLocaleTimeString()}`;
  } catch (e) {
    $('apStatus').textContent = 'Syntax error: ' + e.message;
    log('err', 'autopilot syntax: ' + e.message);
  }
}

// ---------------------------------------------------------------- render

function renderModeUi() {
  for (const b of document.querySelectorAll('#modeSeg button')) b.classList.toggle('on', b.dataset.mode === S.mode);
  const go = $('btnGo');
  go.classList.toggle('hidden', S.mode !== 'auto');
  go.classList.toggle('armed', S.armed);
  go.textContent = S.armed ? 'RUNNING' : 'GO';
}

function setBar(el, txt, v) {
  const w = Math.abs(v) / 2;
  el.style.width = `${w}%`;
  el.style.left = v >= 0 ? '50%' : `${50 - w}%`;
  txt.textContent = v;
}

let lastRender = 0;
function render(ts) {
  requestAnimationFrame(render);
  if (ts - lastRender < 60) return;
  lastRender = ts;
  const t = now();
  if (link.connected) {
    // Link health in the header: amber dot when Bluetooth is slow.
    const fresh = S.tel.ping != null && t - (S.tel.pingT || 0) < 6000;
    $('status').textContent = fresh ? `${ui.name || 'Connected'} · ${S.tel.ping.toFixed(0)} ms` : ui.name || 'Connected';
    $('dot').classList.toggle('slow', fresh && S.tel.ping > 150);
  }
  setBar($('mL'), $('mLv'), S.out[0]);
  setBar($('mR'), $('mRv'), S.out[1]);
  const code = t - S.tel.lineAt < 1500 ? S.tel.line : null;
  $('sL').classList.toggle('black', code != null && !!(code & 2));
  $('sR').classList.toggle('black', code != null && !!(code & 1));
  $('vDist').textContent = S.tel.dist != null ? `${S.tel.dist} cm` : '–';
  $('vPing').textContent = S.tel.ping != null ? `${S.tel.ping.toFixed(0)} ms` : '–';
  while (S.lineTimes.length && t - S.lineTimes[0] > 1000) S.lineTimes.shift();
  $('vHz').textContent = S.lineTimes.length ? `${S.lineTimes.length}` : '–';
  const a = S.tel.accel;
  $('vAccel').textContent = a ? a.map((v) => (v / 1000).toFixed(2)).join(' ') : '–';
  $('vEnv').textContent = S.tel.light != null || S.tel.temp != null ? `${S.tel.light ?? '–'} · ${S.tel.temp ?? '–'}°C` : '–';
  if (lap.running) $('lapTime').textContent = ((t - lap.start) / 1000).toFixed(2);
  if (ui.tab === 'pilot') drawLineHist(t);
  if (V) try { renderVision(t); } catch { /* never let the vision panel stop the log */ }
  renderLog();
}

function drawLineHist(t) {
  const c = $('lineHist');
  const w = (c.width = c.clientWidth * devicePixelRatio);
  const h = (c.height = 60 * devicePixelRatio);
  const g = c.getContext('2d');
  g.fillStyle = '#0e1118';
  g.fillRect(0, 0, w, h);
  const span = 4000;
  const hist = S.lineHist;
  for (let i = 0; i < hist.length; i++) {
    const [ht, code] = hist[i];
    if (t - ht > span) continue;
    const x0 = w - ((t - ht) / span) * w;
    const x1 = i + 1 < hist.length ? w - ((t - hist[i + 1][0]) / span) * w : w;
    const bw = Math.max(1, x1 - x0);
    g.fillStyle = code & 2 ? '#000' : '#fff';
    g.fillRect(x0, 2, bw, h / 2 - 4);
    g.fillStyle = code & 1 ? '#000' : '#fff';
    g.fillRect(x0, h / 2 + 2, bw, h / 2 - 4);
  }
}

// ---------------------------------------------------------------- build check

let buildTag = null;
async function probeBuild() {
  try {
    const tags = await Promise.all(['app.js', 'ui.html', 'style.css'].map(async (f) => {
      const r = await fetch(`${f}?probe=${Date.now()}`, { method: 'HEAD', cache: 'no-store' });
      return (r.headers.get('etag') || '') + (r.headers.get('last-modified') || '');
    }));
    const tag = tags.join('|');
    if (buildTag === null) {
      buildTag = tag;
      const r = await fetch(`app.js?probe=${Date.now()}`, { method: 'HEAD', cache: 'no-store' });
      const lm = r.headers.get('last-modified');
      $('build').textContent = lm ? 'build ' + new Date(lm).toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' }) : '';
    } else if (tag !== buildTag) {
      $('btnUpdate').classList.remove('hidden');
    }
  } catch { /* offline */ }
}

// ---------------------------------------------------------------- wire up

function init() {
  padSteer = new Pad($('padSteer'), onPadRelease);
  padThrottle = new Pad($('padThrottle'), onPadRelease);
  applyLayout();

  buildForm($('tuneForm'));
  $('tuneForm').querySelector('fieldset[data-group="Calibrate"]').insertAdjacentHTML('beforeend',
    '<div class="row"><button class="btn primary" id="btnStraight">Straight test</button>' +
    '<span class="muted small">Drives straight, then stops. Adjust trim until the car tracks straight.</span></div>');
  $('btnStraight').onclick = straightTest;
  $('tuneForm').querySelector('fieldset[data-group="Vision"] legend').insertAdjacentHTML('afterend',
    '<div id="visStatus" class="muted small">Vision module loading…</div><div id="visFit" class="small"></div><div id="visLearned" class="small"></div>' +
    '<div class="row"><button class="btn primary" id="btnVisApply">Apply fit</button><button class="btn" id="btnVisReset">Reset learning</button>' +
    '<button class="btn" id="btnVisForget">Forget samples</button></div>');
  $('btnVisApply').onclick = () => { if (V && V.est?.ok) visApply(vis.propose(V.est, learned, p.trim, { full: true }), 'fit applied'); else log('ap', 'vision: no fit yet'); };
  $('btnVisReset').onclick = visReset;
  $('btnVisForget').onclick = () => { if (V) { V.forget(); log('ap', 'vision: calibration samples cleared'); } };
  $('btnLinkTest').onclick = linkTest;
  $('btnCopyLog').onclick = copyLog;
  buildForm($('apParams'), 'Autopilot');
  for (const x of PARAMS) syncInputs(x.k);
  document.addEventListener('input', (e) => { if (e.target.dataset?.k) onParamInput(e); });
  document.addEventListener('change', (e) => { if (e.target.dataset?.k) onParamInput(e); });

  $('robotId').value = store.get('robotId', '');
  $('robotId').addEventListener('input', (e) => store.set('robotId', e.target.value.trim()));
  $('teamName').value = store.get('team', '');
  $('teamName').addEventListener('input', (e) => store.set('team', e.target.value));
  $('btnTeam').onclick = () => link.send('DISP,' + ($('teamName').value || 'TEAM').toUpperCase());

  $('btnConnect').onclick = connect;
  $('btnStop').onclick = () => emergencyStop('STOP button');
  $('btnGo').onclick = () => (S.armed ? emergencyStop('tapped RUNNING') : arm());
  $('btnUpdate').onclick = () => { emergencyStop(); setTimeout(() => location.reload(), 120); };
  $('btnLap').onclick = lapStartStop;
  $('btnLapReset').onclick = lapReset;
  $('btnFx').onclick = () => { p.fx = !p.fx; saveParams(); syncInputs('fx'); onParamChanged('fx'); };
  $('btnFx').textContent = p.fx ? 'FX: on' : 'FX: off';
  $('btnTilt').onclick = toggleTilt; // turning it on zeroes the current phone angle

  for (const b of document.querySelectorAll('#modeSeg button')) {
    b.onclick = () => setMode(b.dataset.mode);
  }
  for (const b of document.querySelectorAll('#tabSeg button')) {
    b.onclick = () => {
      ui.tab = b.dataset.tab;
      for (const x of document.querySelectorAll('#tabSeg button')) x.classList.toggle('on', x === b);
      for (const s of document.querySelectorAll('.tab')) s.classList.toggle('on', s.id === 'tab-' + ui.tab);
      logDirty = true;
      showVision(ui.tab === 'vision');
    };
  }
  document.addEventListener('click', (e) => {
    const b = e.target.closest('[data-cmd]');
    if (!b) return;
    if (b.dataset.cmd === 'PING') { ui.pingManual = true; query('PING'); } else link.send(b.dataset.cmd);
  });

  $('rawSend').onclick = () => {
    const v = $('rawCmd').value.trim().replace(/#$/, '');
    if (!v) return;
    if (!ui.traffic) log('tx', '→ ' + v);
    if (v === 'PING') { ui.pingManual = true; query('PING'); } else link.send(v);
  };
  $('rawCmd').addEventListener('keydown', (e) => { if (e.key === 'Enter') $('rawSend').click(); });
  $('logTraffic').onchange = (e) => { ui.traffic = e.target.checked; };
  $('logClear').onclick = () => { logBuf.length = 0; logDirty = true; };

  $('apSelect').onchange = (e) => selectScript(e.target.value, false);
  $('apApply').onclick = () => applyScript(true);
  $('apPull').onclick = () => loadScripts(true);
  $('apCode').addEventListener('keydown', (e) => {
    if (e.key === 'Tab') { e.preventDefault(); document.execCommand('insertText', false, '  '); }
  });

  $('btnExport').onclick = async () => {
    const json = JSON.stringify(p);
    try { await navigator.clipboard.writeText(json); log('ap', 'Settings copied to clipboard'); } catch { prompt('Copy settings:', json); }
  };
  $('btnImport').onclick = () => {
    const json = prompt('Paste settings JSON:');
    if (!json) return;
    try { Object.assign(p, pick(JSON.parse(json))); saveParams(); for (const x of PARAMS) { syncInputs(x.k); onParamChanged(x.k); } } catch (e) { log('err', 'Import: ' + e.message); }
  };
  $('btnDefaults').onclick = () => {
    if (!confirm('Reset all tuning to defaults?')) return;
    Object.assign(p, defaults());
    saveParams();
    for (const x of PARAMS) { syncInputs(x.k); onParamChanged(x.k); }
  };

  document.addEventListener('visibilitychange', () => {
    if (document.hidden) emergencyStop('app went to background');
    else if (link.connected) wakeLock();
  });
  window.addEventListener('pagehide', () => emergencyStop());
  document.addEventListener('gesturestart', (e) => e.preventDefault());

  renderModeUi();
  restartTick();
  requestAnimationFrame(render);
  loadVision();
  loadScripts(true);
  probeBuild();
  setInterval(() => { if (!document.hidden) probeBuild(); }, 20000);

  if (!BleTransport.supported()) {
    $('status').textContent = 'No Web BLE: use Bluefy';
    log('err', 'This browser has no Web Bluetooth. On iPhone, open this page in the Bluefy app.');
  }
  tryAutoReconnect();
  log('ap', 'Ready');
}

// Exposed for debugging from the Log tab / tests.
window.rr = {
  p, S, link, log, arm, disarm, emergencyStop, startSim, simOpts, linkTest, straightTest, get transport() { return transport; },
  vision: { ingest: (m) => visionIngest(m, 'api'), state: () => visState(now(), 0), reset: visReset, get learned() { return learned; }, get V() { return V; } },
};

init();
