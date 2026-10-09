// Smoke test: serves the repo, runs the app in headless Chromium against the demo simulator and a fake
// Web Bluetooth micro:bit, and fails on JS errors or broken driving. Run before pushing: `node tools/smoke.mjs`
// Needs Playwright (npm i -D playwright, or a global install).
import http from 'node:http';
import fs from 'node:fs';
import path from 'node:path';
import { createRequire } from 'node:module';
import { fileURLToPath } from 'node:url';

const root = path.resolve(path.dirname(fileURLToPath(import.meta.url)), '..');
let chromium;
for (const base of [import.meta.url, '/opt/node-tools/node_modules/', path.join(process.env.HOME || '', 'node_modules/')]) {
  try { ({ chromium } = createRequire(base)('playwright')); break; } catch { /* try next */ }
}
if (!chromium) { console.error('Playwright not found: npm i -D playwright'); process.exit(2); }

const types = { '.html': 'text/html', '.js': 'text/javascript', '.css': 'text/css', '.json': 'application/json' };
const server = http.createServer((req, res) => {
  const rel = decodeURIComponent(new URL(req.url, 'http://x').pathname).replace(/^\/+/, '') || 'index.html';
  const file = path.join(root, rel);
  if (!file.startsWith(root) || !fs.existsSync(file) || fs.statSync(file).isDirectory()) { res.writeHead(404); res.end(); return; }
  res.writeHead(200, { 'content-type': types[path.extname(file)] || 'application/octet-stream', 'last-modified': fs.statSync(file).mtime.toUTCString() });
  res.end(req.method === 'HEAD' ? undefined : fs.readFileSync(file));
});
await new Promise((r) => server.listen(0, '127.0.0.1', r));
const BASE = `http://127.0.0.1:${server.address().port}/`;

const failures = [];
const check = (ok, msg) => { console.log(`${ok ? 'ok  ' : 'FAIL'} ${msg}`); if (!ok) failures.push(msg); };
const errors = [];
const browser = await chromium.launch(process.env.CHROMIUM ? { executablePath: process.env.CHROMIUM } : {});
const ctx = await browser.newContext({ viewport: { width: 844, height: 390 }, hasTouch: true, isMobile: true });

try {
  // 1. Demo simulator: manual driving and autopilot
  const page = await ctx.newPage();
  page.on('pageerror', (e) => errors.push(e.message));
  await page.goto(BASE + '?demo');
  await page.waitForFunction(() => window.rr?.link.connected, null, { timeout: 5000 });
  await page.evaluate(() => Object.assign(window.rr.p, { simTrack: 'lane', simLatency: 70 }));
  await page.waitForTimeout(500);

  const pad = await page.locator('#padThrottle').boundingBox();
  const x0 = await page.evaluate(() => window.rr.transport.x);
  await page.mouse.move(pad.x + pad.width / 2, pad.y + pad.height / 2);
  await page.mouse.down();
  await page.mouse.move(pad.x + pad.width / 2, pad.y + pad.height / 2 - 80, { steps: 4 });
  await page.waitForTimeout(1000);
  const held = await page.evaluate(() => window.rr.S.out);
  await page.mouse.up();
  await page.waitForTimeout(200);
  const released = await page.evaluate(() => ({ out: window.rr.S.out, l: window.rr.transport.l, x: window.rr.transport.x }));
  check(held[0] > 25 && held[1] > 25, `throttle drives both wheels forward (${held})`);
  check(released.out[0] === 0 && released.l === 0, 'releasing the pad stops the motors');
  check(released.x > x0 + 5, 'simulated car moved');

  await page.evaluate(() => window.rr.transport.reset());
  await page.click('#modeSeg button[data-mode=auto]');
  await page.click('#btnGo');
  let maxOff = 0;
  let travelled = 0;
  let last = null;
  for (let i = 0; i < 24; i++) {
    await page.waitForTimeout(500);
    const st = await page.evaluate(() => { const t = window.rr.transport; return { off: t.constructor.offTrack(t.x, t.y), x: t.x, y: t.y }; });
    maxOff = Math.max(maxOff, st.off);
    if (last) travelled += Math.hypot(st.x - last.x, st.y - last.y);
    last = st;
  }
  check(await page.evaluate(() => window.rr.S.lineSent.length <= window.rr.p.apDepth), 'line queries in flight stay within apDepth');
  check(maxOff < 12, `lane keeper stays in the lane (center max ${maxOff.toFixed(1)} cm off, lane half 12)`);
  check(travelled > 100, `autopilot makes progress (${travelled.toFixed(0)} cm in 12 s)`);
  const pb = await page.locator('#padThrottle').boundingBox();
  await page.mouse.move(pb.x + pb.width / 2, pb.y + pb.height / 2);
  await page.mouse.down();
  await page.waitForTimeout(400);
  check(await page.evaluate(() => window.rr.S.armed && window.rr.transport.l === 0 && window.rr.transport.r === 0), 'touching a pad overrides Auto (idle thumb = stopped)');
  await page.mouse.up();
  await page.waitForTimeout(800);
  check(await page.evaluate(() => window.rr.S.armed && (window.rr.transport.l !== 0 || window.rr.transport.r !== 0)), 'releasing the pad hands control back to Auto');
  await page.click('#btnStop');
  await page.waitForTimeout(150);
  check(await page.evaluate(() => !window.rr.S.armed && window.rr.transport.l === 0), 'STOP disarms and stops');

  // Real robot: PING waits behind ?LINE so ping reads ~700 ms; with Start speed 0 the old cap sent MS,10,10.
  await page.evaluate(() => { Object.assign(window.rr.p, { minSpeed: 0 }); window.rr.transport.reset(); });
  await page.click('#btnGo');
  await page.waitForTimeout(400);
  await page.evaluate(() => { window.__pingHold = setInterval(() => { window.rr.S.tel.ping = 700; }, 5); });
  await page.waitForTimeout(1200);
  const slowRun = await page.evaluate(() => window.rr.S.out.slice());
  await page.click('#btnStop');
  await page.evaluate(() => { clearInterval(window.__pingHold); window.rr.p.minSpeed = 25; });
  check(slowRun[0] >= 30 && slowRun[1] >= 30, `lane keeper drives at 30+ even with a high ping reading (${slowRun})`);

  // An idle gamepad that is merely connected must not hold the car in Auto.
  await page.evaluate(() => { window.__gp = navigator.getGamepads; navigator.getGamepads = () => [{ connected: true, axes: [0, 0, 0, 0], buttons: [] }]; window.rr.transport.reset(); });
  await page.click('#btnGo');
  await page.waitForTimeout(1500);
  const gpRun = await page.evaluate(() => ({ l: window.rr.transport.l, r: window.rr.transport.r }));
  await page.click('#btnStop');
  await page.evaluate(() => { navigator.getGamepads = window.__gp; });
  check(gpRun.l > 0 && gpRun.r > 0, `Auto drives with an idle gamepad connected (${gpRun.l},${gpRun.r})`);

  await page.evaluate(() => window.rr.transport.reset());
  await page.click('#tabSeg button[data-tab=tune]');
  await page.click('#btnStraight');
  await page.waitForTimeout(800);
  const mid = await page.evaluate(() => ({ l: window.rr.transport.l, r: window.rr.transport.r }));
  await page.waitForTimeout(1200);
  const fin = await page.evaluate(() => ({ l: window.rr.transport.l, x: window.rr.transport.x, y: window.rr.transport.y }));
  check(mid.l === 60 && mid.r === 60, `straight test drives at the test speed (${mid.l},${mid.r})`);
  check(fin.l === 0 && fin.x > -25 && Math.abs(fin.y + 45) < 1, 'straight test goes straight, then stops');

  // Lost far from the lane: the lane keeper must stop on its own instead of circling.
  await page.click('#modeSeg button[data-mode=auto]');
  await page.evaluate(() => { const t = window.rr.transport; t.reset(); t.x = 0; t.y = 0; });
  await page.click('#btnGo');
  await page.waitForTimeout(4000);
  const lost = await page.evaluate(() => ({ l: window.rr.transport.l, r: window.rr.transport.r, gaveUp: !!window.rr.S.mem.gaveUp }));
  await page.click('#btnStop');
  check(lost.l === 0 && lost.r === 0 && lost.gaveUp, `lane keeper stops when it can't find the lane (${lost.l},${lost.r}, gave up ${lost.gaveUp})`);

  // Explore & map: drives the lane, asks ?DIST itself, draws the map overlay, backs up from a close obstacle.
  const pickScript = (file) => page.evaluate((f) => {
    const sel = document.getElementById('apSelect');
    sel.value = f;
    sel.dispatchEvent(new Event('change'));
    document.getElementById('apApply').click();
  }, file);
  await pickScript('explore.js');
  await page.evaluate(() => window.rr.transport.reset());
  await page.click('#btnGo');
  await page.waitForTimeout(2500);
  const ex = await page.evaluate(() => ({ l: window.rr.transport.l, r: window.rr.transport.r, dist: window.rr.S.tel.dist, cells: Object.keys(window.rr.S.mem.cells || {}).length, map: !!document.getElementById('rrMap') }));
  check(ex.l > 0 && ex.r > 0 && ex.dist > 0 && ex.cells > 3 && ex.map, `explore drives, reads distance, maps (${ex.l},${ex.r}, dist ${ex.dist}, ${ex.cells} cells, overlay ${ex.map})`);
  // The map lets touches through to the controls underneath; only 📍 makes it take one placement.
  const mb = await page.locator('#rrMap').boundingBox();
  const through = await page.evaluate(([x, y]) => { const el = document.elementFromPoint(x, y); return el && !el.closest('#rrMapBox') ? el.id || el.tagName : null; }, [mb.x + mb.width / 2, mb.y + mb.height / 2]);
  check(!!through, `explore map lets taps through to the controls (${through})`);
  await page.click('#rrMapPlace');
  await page.waitForTimeout(400);
  // A coarse tap 12 cm beside the top straight, no drag: it should snap to that lane and take its direction.
  const tap = await page.evaluate(() => {
    const tr = window.__rrTrack, v = window.__rrMapView, d = devicePixelRatio, m = window.rr.S.mem;
    let k = 0;
    tr.forEach((q, i) => { if (Math.abs(q[0]) < 30 && q[1] > tr[k][1]) k = i; });
    return { k, q: tr[k], d0: Math.hypot(tr[k][0] - m.x, tr[k][1] - m.y), px: ((tr[k][0] - v.x0) * v.sc) / d, py: ((v.y1 - tr[k][1] - 12) * v.sc) / d };
  });
  await page.mouse.click(mb.x + tap.px, mb.y + tap.py);
  await page.waitForFunction(() => window.rr.S.mem.userPlaced, null, { timeout: 2000 }).catch(() => {});
  const placed = await page.evaluate(() => { const m = window.rr.S.mem; return { placing: !!window.__rrPlacing, x: m.x, y: m.y, h: m.h, ti: m.ti, user: m.userPlaced, armed: window.rr.S.armed }; });
  const dq = Math.hypot(placed.x - tap.q[0], placed.y - tap.q[1]);
  check(!placed.placing && placed.user && placed.armed && dq < 0.25 * tap.d0 + 15 && Math.abs(Math.cos(placed.h)) > 0.8,
    `a coarse tap on the map pulls the car onto that lane, along it, without stopping Auto (${dq.toFixed(0)} cm from the lane point, was ${tap.d0.toFixed(0)}; heading ${((placed.h * 180) / Math.PI).toFixed(0)}°)`);
  if (process.env.SHOT) await page.screenshot({ path: process.env.SHOT });
  await page.evaluate(() => { window.__distHold = setInterval(() => { window.rr.S.tel.dist = 8; window.rr.S.tel.distAt = performance.now(); }, 5); });
  let back = { l: 0, r: 0 };
  for (let i = 0; i < 10 && !(back.l < 0 && back.r < 0); i++) {
    await page.waitForTimeout(60);
    back = await page.evaluate(() => ({ l: window.rr.transport.l, r: window.rr.transport.r }));
  }
  await page.evaluate(() => clearInterval(window.__distHold));
  await page.click('#btnStop');
  check(back.l < 0 && back.r < 0, `explore backs up from an obstacle 8 cm ahead (${back.l},${back.r})`);
  await pickScript('lane.js');
  // Rally track (standalone SimCar, so the app's control loop can't interfere): lane, motor lag, laps, link model.
  const { SimCar, LinkModel } = await import('../sim.js');
  const car = new SimCar(() => ({ track: 'rally' }));
  check(!car.black(...car.sensors()[0]) && !car.black(...car.sensors()[1]), 'rally track starts the car inside the lane');
  car.l = car.r = 60;
  car.step(0.05);
  const vEarly = car.v;
  for (let i = 0; i < 40; i++) car.step(0.05);
  check(vEarly < car.v * 0.5 && car.v > 5, `rally motors lag (v ${vEarly.toFixed(1)} → ${car.v.toFixed(1)})`);
  const lm = new LinkModel(() => 0.5);
  const rtts = new Set();
  for (let t = 0; t < 30000; t += 500) rtts.add(lm.rtt(t, 'varying', 70));
  check(rtts.has(70) && rtts.has(420) && new LinkModel().rtt(0, 'steady', 70) === 70, 'varying link flips between fast and slow');

  await page.click('#tabSeg button[data-tab=pilot]');
  await page.fill('#apCode', 'return [ broken');
  await page.click('#apApply');
  check((await page.textContent('#apStatus')).startsWith('Syntax error'), 'script syntax errors are reported');
  await page.close();

  // 2. Fake Web Bluetooth micro:bit (20-byte indications, disconnect + reconnect)
  const page2 = await ctx.newPage();
  page2.on('pageerror', (e) => errors.push(e.message));
  await page2.addInitScript(() => {
    const enc = new TextEncoder();
    const dec = new TextDecoder();
    const got = [];
    window.__ble = { got, maxWrite: 0 };
    const dev = new EventTarget();
    dev.id = 'dev1';
    dev.name = 'BBC micro:bit [tuzov]';
    dev.buf = '';
    const indicate = (s) => {
      const b = enc.encode(s);
      for (let i = 0; i < b.length; i += 20) {
        const chunk = b.slice(i, i + 20);
        setTimeout(() => { const ev = new Event('characteristicvaluechanged'); Object.defineProperty(ev, 'target', { value: { value: new DataView(chunk.buffer) } }); txc.dispatchEvent(ev); }, 8 + i + (window.__ble.delay || 0));
      }
    };
    const replies = { '?LINE': 'LINE:2', '?ACCEL': 'ACCEL:-1023,-1012,-1004', '?DIST': 'DIST:42', PING: 'PONG', '?LIGHT': 'LIGHT:140', '?TEMP': 'TEMP:23' };
    const txc = new EventTarget();
    txc.startNotifications = async () => txc;
    const rxc = {
      properties: { write: true, writeWithoutResponse: true },
      busy: false,
      async writeValueWithoutResponse(buf) {
        if (!dev.gatt.connected) throw new Error('GATT Server is disconnected');
        if (this.busy) throw new Error('GATT operation already in progress');
        this.busy = true; await new Promise((r) => setTimeout(r, 3)); this.busy = false;
        window.__ble.maxWrite = Math.max(window.__ble.maxWrite, buf.byteLength);
        dev.buf += dec.decode(buf);
        let i;
        while ((i = dev.buf.indexOf('#')) >= 0) { const c = dev.buf.slice(0, i); dev.buf = dev.buf.slice(i + 1); got.push(c); if (replies[c]) indicate(replies[c] + '#\n'); }
      },
    };
    dev.gatt = {
      connected: false,
      async connect() { await new Promise((r) => setTimeout(r, 20)); this.connected = true; return this; },
      disconnect() { this.connected = false; dev.dispatchEvent(new Event('gattserverdisconnected')); },
      async getPrimaryService() { return { getCharacteristic: async (u) => (u.startsWith('6e400003') ? rxc : txc) }; },
    };
    window.__ble.dev = dev;
    navigator.bluetooth = { requestDevice: async (opts) => { window.__ble.opts = opts; return dev; } };
  });
  await page2.goto(BASE);
  await page2.waitForFunction(() => window.rr, null, { timeout: 5000 });
  await page2.click('#tabSeg button[data-tab=tune]');
  await page2.fill('#robotId', 'tuzov');
  await page2.click('#btnConnect');
  await page2.waitForFunction(() => window.rr.link.connected, null, { timeout: 5000 });
  await page2.waitForTimeout(2500);
  const st = await page2.evaluate(() => ({ opts: window.__ble.opts, tel: window.rr.S.tel, status: document.getElementById('status').textContent }));
  check(st.opts.filters[0].name === 'BBC micro:bit [tuzov]', 'robot ID filters the device picker');
  check(st.status.startsWith('tuzov'), `status shows robot ID (${st.status})`);
  // Slow link: replies take 300 ms. Manual telemetry must not pile up queries.
  await page2.evaluate(() => { window.__ble.delay = 300; window.__ble.got.length = 0; });
  await page2.waitForTimeout(2000);
  const q = await page2.evaluate(() => window.__ble.got.filter((c) => c.startsWith('?') || c === 'PING').length);
  await page2.evaluate(() => { window.__ble.delay = 0; });
  await page2.waitForTimeout(400);
  check(q <= 8, `telemetry keeps one query in flight on a slow link (${q} queries in 2 s)`);
  check(st.tel.line === 2 && st.tel.dist === 42 && st.tel.ping > 0, 'telemetry LINE/DIST/PING parsed');
  check(JSON.stringify(st.tel.accel) === '[-1023,-1012,-1004]', 'reply split across 20-byte packets reassembled');
  await page2.evaluate(() => window.rr.link.send('DISP,A VERY LONG TEAM NAME 123'));
  await page2.waitForTimeout(200);
  check(await page2.evaluate(() => window.__ble.got.includes('DISP,A VERY LONG TEAM NAME 123') && window.__ble.maxWrite <= 20), 'long commands chunked to 20 bytes');
  await page2.click('#tabSeg button[data-tab=log]');
  await page2.click('#btnLinkTest');
  await page2.waitForFunction(() => document.getElementById('log').textContent.includes('LINK TEST'), null, { timeout: 15000 }).catch(() => {});
  const report = await page2.evaluate(() => [...document.querySelectorAll('#log span')].map((x) => x.textContent).find((t) => t.includes('LINK TEST')) || '');
  check(/10\/10 answered/.test(report) && / \d+ Hz, 0 lost/.test(report) && /x3 in flight [1-9]\d* Hz/.test(report), `link test reports ping and loop rate: ${report.replace(/^[\d.]+\s+/, '')}`);
  await page2.evaluate(() => { const d = window.__ble.dev; d.gatt.connected = false; d.dispatchEvent(new Event('gattserverdisconnected')); });
  await page2.waitForTimeout(1500);
  check(await page2.evaluate(() => window.rr.link.connected), 'auto-reconnects after a drop');
  await page2.evaluate(() => { window.__ble.got.length = 0; Object.defineProperty(document, 'hidden', { value: true, configurable: true }); document.dispatchEvent(new Event('visibilitychange')); });
  await page2.waitForTimeout(100);
  check(await page2.evaluate(() => window.__ble.got.includes('S')), 'sends S when the app goes to background');
  await page2.close();
} finally {
  await browser.close();
  server.close();
}
check(errors.length === 0, `no JS errors${errors.length ? ': ' + errors.join(' | ') : ''}`);
process.exit(failures.length ? 1 : 0);
