// Camera debug page (vision.html): runs vision-core.js on the phone's camera, or on the synthetic camera with
// ground truth (vision.html?demo), and draws everything it finds. Not connected to driving yet: the goal is to
// see that it finds the lane and the robot reliably from any angle before it gets near the control loop.
const T = new URL(import.meta.url).search;
const { Vision, VDEFAULTS, classify } = await import('./vision-core.js' + T);

const UART_SERVICE = '6e400001-b5a3-f393-e0a9-e50e24dcca9e';
const UART_RX = '6e400003-b5a3-f393-e0a9-e50e24dcca9e'; // phone -> robot (write)

const SLIDERS = [
  ['procW', 'Resolution (px wide)', 160, 320, 40],
  ['hueLo', 'Lane hue from', 0, 360, 1],
  ['hueHi', 'Lane hue to', 0, 360, 1],
  ['satMin', 'Lane min saturation', 0, 1, 0.01],
  ['valMin', 'Lane min brightness', 0, 1, 0.01],
  ['whiteVal', 'White edge min brightness', 0, 1, 0.01],
  ['whiteSat', 'White edge max saturation', 0, 1, 0.01],
  ['closeK', 'Gap closing (lane widths)', 0.1, 0.8, 0.05],
  ['robotMin', 'Robot min size (W²)', 0.01, 0.5, 0.01],
  ['robotMax', 'Robot max size (W²)', 0.3, 3, 0.1],
  ['bHueLo', 'Beacon hue from', 0, 360, 1],
  ['bHueHi', 'Beacon hue to', 0, 360, 1],
  ['bSat', 'Beacon min saturation', 0, 1, 0.01],
  ['bVal', 'Beacon min brightness', 0, 1, 0.01],
  ['colorTol', 'Taught color tolerance', 0.01, 0.2, 0.005],
];
const DEFAULTS = { ...VDEFAULTS, procW: 240, view: 'overlay', tap: 'inspect', cam: 'follow' };
const load = () => { try { return { ...DEFAULTS, ...JSON.parse(localStorage.getItem('rr.vision') || '{}') }; } catch { return { ...DEFAULTS }; } };
const p = load();
const save = () => { try { localStorage.setItem('rr.vision', JSON.stringify(p)); } catch { /* private mode */ } };
const demo = new URLSearchParams(location.search).has('demo');

document.head.insertAdjacentHTML('beforeend', `<style>
#vmain { flex: 1; display: flex; min-height: 0; }
#vview { flex: 1; min-width: 0; min-height: 0; display: flex; align-items: center; justify-content: center; background: #000; position: relative; }
#vcan { touch-action: none; image-rendering: pixelated; }
#vvid { position: absolute; width: 2px; height: 2px; opacity: 0; pointer-events: none; }
#vpanel { width: 300px; overflow-y: auto; padding: 8px; border-left: 1px solid var(--line); -webkit-overflow-scrolling: touch; }
#vpanel .kv { display: flex; justify-content: space-between; padding: 2px 0; font-variant-numeric: tabular-nums; }
#vpanel .kv b { font-weight: 600; }
#vpanel label { display: block; margin: 6px 0 0; font-size: 12px; color: var(--muted); }
#vpanel label span { float: right; color: var(--text); }
#vpanel input[type=range] { width: 100%; }
#vpanel .row { display: flex; flex-wrap: wrap; gap: 6px; margin: 6px 0; }
#vpanel h4 { margin: 10px 0 4px; font-size: 13px; color: var(--muted); }
.good { color: var(--ok); } .bad { color: var(--bad); }
@media (orientation: portrait) { #vmain { flex-direction: column; } #vpanel { width: auto; height: 42%; border-left: 0; border-top: 1px solid var(--line); } }
</style>`);
document.body.innerHTML = `
<header id="bar">
  <a class="btn" href="./" style="text-decoration:none">◀ App</a>
  <button id="vStart" class="btn primary">${demo ? 'Demo' : 'Camera'}</button>
  <button id="vFreeze" class="btn">Freeze</button>
  <div class="seg" id="vView"><button data-v="overlay">Overlay</button><button data-v="mask">Mask</button><button data-v="raw">Raw</button></div>
  <div class="seg" id="vTap"><button data-v="inspect">Tap: inspect</button><button data-v="robot">Tap: robot</button></div>
  <button id="vBle" class="btn hidden">Beacon lights</button>
  <span id="vStatus" class="muted small" style="margin-left:auto"></span>
</header>
<div id="vmain">
  <div id="vview"><canvas id="vcan"></canvas><video id="vvid" playsinline muted autoplay></video></div>
  <div id="vpanel">
    <div id="vRead"></div>
    <div id="vScore"></div>
    <h4>Tap inspector</h4><div id="vInspect" class="muted small">Tap the picture to read a pixel's hue, saturation and brightness.</div>
    <div class="row"><button id="vCopy" class="btn">Copy report</button><button id="vReset" class="btn">Reset tracker</button><button id="vDefaults" class="btn">Defaults</button></div>
    <label>Robot detection <select id="vSrc"><option value="auto">auto: beacon, else dark gap</option><option value="beacon">green beacon only</option><option value="hole">dark gap in lane</option><option value="color">taught color (Tap: robot)</option></select></label>
    ${demo ? '<label>Demo camera <select id="vCam"><option value="follow">follow (chest height)</option><option value="high">held high</option><option value="side">side of the mat</option></select></label><label class="inline"><input type="checkbox" id="vNoBeacon"> demo car without beacon</label>' : ''}
    <div id="vSliders"></div>
    <p class="muted small">Overlay: magenta = lane, yellow = filled gaps (the robot sits in one), cyan = centerline, green ring = robot,
      arrow = heading, red/blue = distance to the left/right edge, white fan = free lane ahead, thick ray = suggested steering.
      Beacon: tap "Beacon lights" (Bluefy) to turn the headlights green; the camera finds the car and its front from the green.</p>
  </div>
</div>`;

const $ = (id) => document.getElementById(id);
const can = $('vcan'), g = can.getContext('2d'), video = $('vvid');
const proc = document.createElement('canvas'), pg = proc.getContext('2d', { willReadFrequently: true });
const mk = document.createElement('canvas'), mg = mk.getContext('2d');
let vis = null, synth = null, scoreFn = null, img = null, maskImg = null, res = null, gt = null, frozen = false, running = false;
let w = 0, h = 0, fps = 0, lastFrame = 0, simT = 0;
const stats = { n: 0, vis: 0, ok: 0, iou: 0, side: 0, sideN: 0, head: 0, headN: 0 };
const samples = [];
let inspect = null;

// ---- settings panel ----
function buildSliders() {
  $('vSliders').innerHTML = SLIDERS.map(([k, label, min, max, step]) =>
    `<label>${label} <span id="vv-${k}">${p[k]}</span><input type="range" id="vs-${k}" min="${min}" max="${max}" step="${step}" value="${p[k]}"></label>`).join('');
  for (const [k] of SLIDERS) {
    $('vs-' + k).oninput = (e) => { p[k] = Number(e.target.value); $('vv-' + k).textContent = p[k]; save(); if (k === 'procW') setup(); };
  }
  $('vSrc').value = p.robotSrc;
  if ($('vCam')) $('vCam').value = p.cam;
  segOn('vView', p.view);
  segOn('vTap', p.tap);
}
function segOn(id, v) { for (const b of $(id).children) b.classList.toggle('on', b.dataset.v === v); }
$('vView').onclick = (e) => { if (e.target.dataset.v) { p.view = e.target.dataset.v; segOn('vView', p.view); save(); draw(); } };
$('vTap').onclick = (e) => { if (e.target.dataset.v) { p.tap = e.target.dataset.v; segOn('vTap', p.tap); save(); } };
$('vSrc').onchange = (e) => { p.robotSrc = e.target.value; save(); vis?.reset(); };
if ($('vCam')) $('vCam').onchange = (e) => { p.cam = e.target.value; save(); setup(); };
if ($('vNoBeacon')) $('vNoBeacon').onchange = () => setup();
$('vFreeze').onclick = () => { frozen = !frozen; $('vFreeze').classList.toggle('on', frozen); $('vFreeze').textContent = frozen ? 'Frozen' : 'Freeze'; };
$('vReset').onclick = () => { vis?.reset(); Object.keys(stats).forEach((k) => (stats[k] = 0)); };
$('vDefaults').onclick = () => { Object.assign(p, DEFAULTS); save(); buildSliders(); setup(); };
$('vCopy').onclick = async () => {
  const keep = ['n', 'vis', 'ok', 'sideN', 'side', 'headN', 'head'];
  const rep = { build: T, demo, size: [w, h], fps: +fps.toFixed(1), ms: res && +res.ms.toFixed(1), W: res && +res.W.toFixed(1),
    lane: res && +res.laneFrac.toFixed(3), robot: res?.robot?.map(Math.round), heading: res?.headingFrom, offset: res?.offset?.toFixed(2),
    stats: demo ? Object.fromEntries(keep.map((k) => [k, stats[k]])) : undefined, samples, p: Object.fromEntries(Object.entries(p).filter(([k]) => k in DEFAULTS)) };
  try { await navigator.clipboard.writeText(JSON.stringify(rep)); $('vCopy').textContent = 'Copied'; } catch { prompt('Copy this:', JSON.stringify(rep)); }
  setTimeout(() => ($('vCopy').textContent = 'Copy report'), 1500);
};

// ---- sources ----
async function setup() {
  const sw = demo ? 4 : video.videoWidth || 4, sh = demo ? 3 : video.videoHeight || 3;
  w = Math.round(p.procW);
  h = Math.round((w * sh) / sw);
  proc.width = mk.width = w;
  proc.height = mk.height = h;
  vis = new Vision(w, h);
  img = pg.createImageData(w, h);
  maskImg = mg.createImageData(w, h);
  if (demo) {
    const { SynthCam, score: sc } = await import('./vision-synth.js' + T);
    scoreFn = sc;
    synth = new SynthCam(w, h, { cam: p.cam, beacon: !$('vNoBeacon')?.checked });
  }
  Object.keys(stats).forEach((k) => (stats[k] = 0));
  fit();
}
function fit() {
  if (!w) return;
  const box = $('vview').getBoundingClientRect(), k = Math.min(box.width / w, box.height / h);
  const dpr = window.devicePixelRatio || 1;
  can.style.width = `${Math.floor(w * k)}px`;
  can.style.height = `${Math.floor(h * k)}px`;
  can.width = Math.floor(w * k * dpr);
  can.height = Math.floor(h * k * dpr);
  draw();
}
addEventListener('resize', () => setTimeout(fit, 100));

$('vStart').onclick = async () => {
  if (running) return;
  if (!demo) {
    if (!navigator.mediaDevices?.getUserMedia) { status('No camera API in this browser. Try Safari.', true); return; }
    try {
      const stream = await navigator.mediaDevices.getUserMedia({ audio: false, video: { facingMode: { ideal: 'environment' }, width: { ideal: 1280 }, height: { ideal: 720 } } });
      video.srcObject = stream;
      await video.play();
      await new Promise((r) => (video.videoWidth ? r() : (video.onloadedmetadata = r)));
    } catch (e) { status(`Camera failed: ${e.name || ''} ${e.message || e}`, true); return; }
  }
  running = true;
  $('vStart').classList.remove('primary');
  await setup();
  requestAnimationFrame(loop);
};
function status(t, bad) { $('vStatus').textContent = t; $('vStatus').classList.toggle('bad', !!bad); }

// ---- main loop ----
function loop(ts) {
  requestAnimationFrame(loop);
  if (!vis || frozen) return;
  if (!demo && (video.readyState < 2 || video.videoWidth === 0)) return;
  if (!demo && Math.round((p.procW * video.videoHeight) / video.videoWidth) !== h) { setup(); return; } // rotated
  const dt = lastFrame ? Math.min(0.2, (ts - lastFrame) / 1000) : 0.05;
  lastFrame = ts;
  fps += (1 / Math.max(dt, 0.001) - fps) * 0.1;
  if (demo) {
    simT += dt;
    gt = synth.render(simT, img.data);
    pg.putImageData(img, 0, 0);
  } else {
    pg.drawImage(video, 0, 0, w, h);
    img = pg.getImageData(0, 0, w, h);
  }
  res = vis.process(img.data, ts, p);
  if (demo) score();
  draw();
  readouts();
}

function score() {
  const s = scoreFn(res, gt, vis.drive);
  stats.n++;
  stats.iou += s.iou;
  if (gt.robot) { stats.vis++; stats.ok += s.ok ? 1 : 0; }
  if (s.sideOk != null) { stats.sideN++; stats.side += s.sideOk ? 1 : 0; }
  if (s.headOk != null) { stats.headN++; stats.head += s.headOk ? 1 : 0; }
  stats.last = s;
}

// ---- drawing ----
function draw() {
  if (!res || !w) return;
  const k = can.width / w;
  g.setTransform(1, 0, 0, 1, 0, 0);
  g.imageSmoothingEnabled = !demo;
  if (p.view === 'mask') { g.fillStyle = '#000'; g.fillRect(0, 0, can.width, can.height); }
  else if (demo) g.drawImage(proc, 0, 0, can.width, can.height);
  else g.drawImage(video, 0, 0, can.width, can.height);
  if (p.view !== 'raw') {
    const d = maskImg.data, alpha = p.view === 'mask' ? 255 : 110;
    for (let i = 0, j = 0; i < w * h; i++, j += 4) {
      let c = null;
      if (vis.skel[i]) c = [0, 240, 255, 255];
      else if (vis.bea[i]) c = [40, 255, 90, 255];
      else if (vis.lane[i]) c = [230, 60, 230, alpha];
      else if (vis.drive[i]) c = [255, 220, 0, 200];
      if (c) { d[j] = c[0]; d[j + 1] = c[1]; d[j + 2] = c[2]; d[j + 3] = c[3]; } else d[j + 3] = 0;
    }
    mg.putImageData(maskImg, 0, 0);
    g.imageSmoothingEnabled = false;
    g.drawImage(mk, 0, 0, can.width, can.height);
  }
  g.setTransform(k, 0, 0, k, 0, 0);
  g.lineWidth = 1.5 / k * (window.devicePixelRatio || 1);
  for (const c of res.cands) { g.strokeStyle = 'rgba(255,170,0,.8)'; g.strokeRect(c.x0, c.y0, c.x1 - c.x0 + 1, c.y1 - c.y0 + 1); }
  if (gt?.robot) { g.strokeStyle = '#fff'; g.setLineDash([2, 2]); circle(gt.robot[0], gt.robot[1], res.W * 0.5); g.setLineDash([]); }
  if (res.robot) {
    const [x, y] = res.robot;
    g.strokeStyle = res.found ? '#2bd47d' : '#ffb020';
    g.lineWidth *= 2;
    circle(x, y, res.W * 0.35);
    g.lineWidth /= 2;
    if (res.fan) {
      for (const [a, r, fx, fy] of res.fan) {
        g.strokeStyle = a === res.steer ? '#fff' : 'rgba(255,255,255,.35)';
        g.lineWidth = (a === res.steer ? 3 : 1) / k * (window.devicePixelRatio || 1);
        line(x, y, x + fx * r, y + fy * r);
      }
      const [dx, dy] = res.heading;
      g.lineWidth = 3 / k * (window.devicePixelRatio || 1);
      g.strokeStyle = '#ff3b5c'; line(x, y, x + dy * res.dl, y - dx * res.dl);
      g.strokeStyle = '#3d7bff'; line(x, y, x - dy * res.dr, y + dx * res.dr);
    }
    if (res.heading) {
      const [dx, dy] = res.heading, L = res.W * 0.9;
      g.strokeStyle = '#2bd47d';
      g.lineWidth = 3 / k * (window.devicePixelRatio || 1);
      line(x, y, x + dx * L, y + dy * L);
      line(x + dx * L, y + dy * L, x + dx * L * 0.7 - dy * L * 0.2, y + dy * L * 0.7 + dx * L * 0.2);
      line(x + dx * L, y + dy * L, x + dx * L * 0.7 + dy * L * 0.2, y + dy * L * 0.7 - dx * L * 0.2);
    }
  }
  if (inspect) { g.strokeStyle = '#fff'; g.lineWidth = 1 / k * (window.devicePixelRatio || 1); circle(inspect[0] + 0.5, inspect[1] + 0.5, 3); }
}
function circle(x, y, r) { g.beginPath(); g.arc(x, y, r, 0, Math.PI * 2); g.stroke(); }
function line(a, b, c, d) { g.beginPath(); g.moveTo(a, b); g.lineTo(c, d); g.stroke(); }

const f = (v, d = 2) => (v == null || !isFinite(v) ? '–' : v.toFixed(d));
function readouts() {
  const r = res, kv = (k, v, cls = '') => `<div class="kv"><span>${k}</span><b class="${cls}">${v}</b></div>`;
  status(`${f(fps, 0)} fps · ${f(r.ms, 1)} ms · ${w}×${h}`);
  $('vRead').innerHTML =
    kv('Lane in view', `${f(r.laneFrac * 100, 0)} %`, r.laneFrac > 0.02 ? 'good' : 'bad') +
    kv('Lane width', `${f(r.W, 1)} px`) +
    kv('Robot', r.robot ? `${r.found ? 'found' : `predicted (${r.lost})`} via ${r.beaconMode ? 'beacon' : p.robotSrc === 'color' ? 'color' : 'gap'}` : 'not found', r.found ? 'good' : 'bad') +
    kv('Heading from', r.headingFrom || '–') +
    kv('Offset in lane', r.offset == null ? '–' : `${f(r.offset)} ${r.offset < -0.15 ? '◀ left' : r.offset > 0.15 ? 'right ▶' : 'center'}${r.offsetSure ? '' : ' (edge out of view)'}`) +
    kv('Free lane ahead', r.ahead == null ? '–' : `${f(r.ahead, 1)} widths`) +
    kv('Steer to', r.steer == null ? '–' : `${r.steer > 0 ? '+' : ''}${r.steer}°`);
  if (demo && stats.n) {
    const pc = (a, b) => (b ? `${((100 * a) / b).toFixed(1)} %` : '–');
    const s = stats.last || {};
    $('vScore').innerHTML = '<h4>Against ground truth (demo)</h4>' +
      kv('Robot within ½ lane width', pc(stats.ok, stats.vis), stats.ok === stats.vis ? 'good' : 'bad') +
      kv('Error now', s.err == null ? '–' : `${f(s.err)} W`, s.ok ? 'good' : 'bad') +
      kv('Lane overlap (IoU)', `${f(stats.iou / stats.n, 3)} · now ${f(s.iou, 3)}`) +
      kv('Offset side right', pc(stats.side, stats.sideN)) +
      kv('Heading right', pc(stats.head, stats.headN)) +
      kv('Frames', stats.n);
  }
}

// ---- taps ----
can.addEventListener('pointerdown', (e) => {
  if (!vis || !img) return;
  const rc = can.getBoundingClientRect();
  const x = Math.max(0, Math.min(w - 1, Math.floor(((e.clientX - rc.left) / rc.width) * w)));
  const y = Math.max(0, Math.min(h - 1, Math.floor(((e.clientY - rc.top) / rc.height) * h)));
  if (p.tap === 'robot') {
    if (p.robotSrc === 'color') vis.teach(img.data, x, y);
    else vis.seed(x, y);
    $('vInspect').textContent = `Robot set at ${x},${y}${p.robotSrc === 'color' ? ' (color learned)' : ''}`;
  } else {
    const j = (y * w + x) * 4, [rr, gg, bb] = [img.data[j], img.data[j + 1], img.data[j + 2]];
    const c = classify(rr, gg, bb, p);
    inspect = [x, y];
    const lane = vis.lane[y * w + x] ? 'lane' : vis.drive[y * w + x] ? 'gap (filled)' : 'off lane';
    samples.push([Math.round(c.h), +c.s.toFixed(2), +c.v.toFixed(2), c.cls]);
    if (samples.length > 12) samples.shift();
    $('vInspect').innerHTML = `<b>${x},${y}</b> rgb ${rr},${gg},${bb} · <b>hue ${Math.round(c.h)}° sat ${f(c.s)} val ${f(c.v)}</b> → ${c.cls}, mask: ${lane}` +
      `<br><span class="muted">recent: ${samples.map((s) => `${s[0]}°/${s[1]}/${s[2]} ${s[3]}`).join(' · ')}</span>`;
    draw();
  }
});

// ---- beacon lights over Bluetooth (Bluefy only; the main app's connection is separate) ----
let rx = null;
if (navigator.bluetooth?.requestDevice) $('vBle').classList.remove('hidden');
$('vBle').onclick = async () => {
  try {
    if (!rx) {
      let id = '';
      try { id = JSON.parse(localStorage.getItem('rr.robotId') || '""'); } catch { /* none */ }
      const filters = id ? [{ name: `BBC micro:bit [${id}]` }] : [{ namePrefix: 'BBC micro:bit' }];
      const dev = await navigator.bluetooth.requestDevice({ filters, optionalServices: [UART_SERVICE] });
      const srv = await (await dev.gatt.connect()).getPrimaryService(UART_SERVICE);
      rx = await srv.getCharacteristic(UART_RX);
      dev.addEventListener('gattserverdisconnected', () => { rx = null; $('vBle').textContent = 'Beacon lights'; });
    }
    for (const c of ['S', 'HLL,0,255,0', 'HLR,0,255,0', 'UG,0,0,0']) await rx.writeValue(new TextEncoder().encode(c + '#'));
    $('vBle').textContent = 'Beacon on';
  } catch (e) { status(`Bluetooth: ${e.message || e}`, true); }
};

buildSliders();
if (demo) $('vStart').click();
else status('Tap Camera, point it at the track');
window.rv = { get res() { return res; }, get stats() { return stats; }, p };
