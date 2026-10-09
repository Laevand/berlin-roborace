// In-app camera for Auto: looks at the mat, finds my robot (green beacon, or tap it), and publishes
// window.__rrCam = { t, found, offset, steer, ahead } for autopilot/camera.js (as s.cam). No map. Uses vision-core.js.

const PROC_W = 160;
const cam = { running: false };
let lastData, Vision, VDEFAULTS, Follower, follow, video, canvas, g, proc, pg, vis, w = 0, h = 0, img, res = null, last = 0;

// the Vision page's saved settings (rr.vision) apply here too, so what looks right there drives here
function params() {
  try { return { ...VDEFAULTS, ...JSON.parse(localStorage.getItem('rr.vision') || '{}') }; } catch { return VDEFAULTS; }
}

export function camState() { return window.__rrCam || null; }

function ensureDom() {
  if (video) return;
  const el = document.createElement('div');
  el.id = 'camView';
  el.className = 'hidden';
  el.innerHTML = '<video id="camVideo" playsinline muted autoplay></video><canvas id="camCanvas"></canvas>' +
    '<span id="camMsg">Tap your robot</span>';
  document.body.appendChild(el);
  video = el.querySelector('video');
  canvas = el.querySelector('canvas');
  g = canvas.getContext('2d');
  proc = document.createElement('canvas');
  pg = proc.getContext('2d', { willReadFrequently: true });
  canvas.addEventListener('pointerdown', (e) => {
    if (!vis || !lastData) return;
    const r = canvas.getBoundingClientRect();
    const x = ((e.clientX - r.left) / r.width) * w, y = ((e.clientY - r.top) / r.height) * h;
    follow.start(lastData, w, h, x, y, vis.W);
  });
}

export async function start(log) {
  ensureDom();
  document.getElementById('camView').classList.remove('hidden');
  const fitBar = () => document.getElementById('camView').style.setProperty('--barH', document.getElementById('bar').getBoundingClientRect().bottom + 'px');
  fitBar();
  addEventListener('resize', fitBar);
  if (cam.running) return;
  if (!Vision) ({ Vision, VDEFAULTS, Follower } = await import('./vision-core.js?t=' + (window.BUILD_T || Date.now())));
  if (!navigator.mediaDevices?.getUserMedia) { log('err', 'camera: not available in this browser'); return; }
  try {
    const stream = await navigator.mediaDevices.getUserMedia({ audio: false, video: { facingMode: { ideal: 'environment' }, width: { ideal: 1280 }, height: { ideal: 720 } } });
    video.srcObject = stream;
    await video.play();
    await new Promise((r) => (video.videoWidth ? r() : (video.onloadedmetadata = r)));
  } catch (e) { log('err', `camera failed: ${e.name || ''} ${e.message || e}`); return; }
  cam.running = true;
  cam.log = log;
  setup();
  requestAnimationFrame(loop);
  log('ap', 'camera on: tap your robot (a green beacon is found by itself)');
}

export function stop() {
  cam.running = false;
  try { video.srcObject?.getTracks().forEach((t) => t.stop()); } catch { /* ignore */ }
  window.__rrCam = null;
  document.getElementById('camView')?.classList.add('hidden');
}

function setup() {
  w = PROC_W;
  h = Math.round((w * video.videoHeight) / video.videoWidth);
  proc.width = w;
  proc.height = h;
  vis = new Vision(w, h);
  follow = new Follower();
  canvas.width = w * 3;
  canvas.height = h * 3;
}

function loop(ts) {
  if (!cam.running) return;
  requestAnimationFrame(loop);
  if (video.readyState < 2 || !video.videoWidth) return;
  if (Math.round((w * video.videoHeight) / video.videoWidth) !== h) setup(); // rotated
  if (ts - last < 40) return; // ~25 fps max
  last = ts;
  pg.drawImage(video, 0, 0, w, h);
  const data = lastData = pg.getImageData(0, 0, w, h).data;
  try { res = follow.apply(vis.process(data, ts, params()), data, vis, ts); } catch (e) { res = null; return; }
  const ok = res && res.found && res.offset != null && res.steer != null;
  window.__rrCam = {
    t: performance.now(), found: !!ok, mine: res ? res.mine != null : false,
    offset: ok ? res.offset : 0, steer: ok ? res.steer : 0, ahead: ok ? res.ahead : 0,
    lost: res ? res.lost : null, moving: !!res && res.headingFrom === 'motion',
  };
  draw(ok);
}

function draw(ok) {
  const k = 3;
  g.drawImage(video, 0, 0, canvas.width, canvas.height);
  if (res) {
    for (const tr of res.tracks) {
      const [x0, y0, x1, y1] = tr.box || [tr.x - 4, tr.y - 4, tr.x + 4, tr.y + 4];
      g.strokeStyle = tr.mine ? '#3f6' : '#ccc';
      g.lineWidth = tr.mine ? 3 : 1;
      g.strokeRect(x0 * k, y0 * k, (x1 - x0) * k, (y1 - y0) * k);
    }
    if (ok && res.heading) {
      const [x, y] = res.robot, a = ((res.steer * Math.PI) / 180), [dx, dy] = res.heading;
      const fx = dx * Math.cos(a) - dy * Math.sin(a), fy = dy * Math.cos(a) + dx * Math.sin(a), L = res.Wr * 2;
      g.strokeStyle = '#fff';
      g.lineWidth = 3;
      g.beginPath();
      g.moveTo(x * k, y * k);
      g.lineTo((x + fx * L) * k, (y + fy * L) * k);
      g.stroke();
    }
  }
  const m = document.getElementById('camMsg');
  m.textContent = ok ? `steer ${res.steer > 0 ? '+' : ''}${res.steer}°  offset ${res.offset.toFixed(2)}` : res && res.mine != null ? 'robot lost' : 'Tap your robot';
}
