// Camera vision for the rally mat, used by vision.html. No DOM: it runs in the browser and in Node (tools/smoke.mjs).
// Input: one RGBA frame (w × h). Output: the lane mask, its centerline, the robot's position and where the robot sits
// in the lane, all measured straight in the picture. That needs no map, so it works from whatever angle the phone sees.
//   lane   = pink/purple/blue pixels, plus white pixels right next to them (the edge lines)
//   drive  = lane with small gaps closed and enclosed holes filled (the robot sits in one of those holes)
//   robots = every dark blob on the lane ("drive but not lane"), each joined with green headlights next to it
//            (the beacon: no green on the track, and it marks the front); or blobs of a color taught by tapping
//   tracks = robots followed from frame to frame (nearest to where each was heading); select() picks "mine",
//            which stays locked while the phone or the robots move. A beacon robot is picked as mine by itself.
//   pose   = for mine: heading from the beacon (else the lane direction, kept pointing the same way as last frame),
//            offset in the lane (-1 left edge … +1 right edge),
//            free lane ahead in lane widths, and the free direction to steer to (longest ray in a ±60° fan)

export const VDEFAULTS = {
  hueLo: 185, hueHi: 355,          // degrees: blue (≈220) → purple (≈280) → pink (≈330)
  satMin: 0.28, valMin: 0.3,       // colored lane pixels
  white: 1, whiteVal: 0.7, whiteSat: 0.25, // white edge lines count only next to colored lane
  closeK: 0.35,                    // gap-closing radius, in lane widths
  robotMin: 0.05, robotMax: 1.5,   // robot blob area, in lane widths²
  robotSrc: 'auto',                // 'auto' = beacon if seen else hole, 'beacon' = green headlights,
                                   // 'hole' = dark gap in the lane, 'color' = color taught by tapping the robot
  bHueLo: 75, bHueHi: 165, bSat: 0.35, bVal: 0.35, // beacon green
  colorTol: 0.06,                  // chromaticity distance for 'color'
};

export function rgb2hsv(r, g, b) {
  const mx = Math.max(r, g, b), mn = Math.min(r, g, b), c = mx - mn;
  let h = 0;
  if (c > 0) {
    if (mx === r) h = ((g - b) / c) % 6;
    else if (mx === g) h = (b - r) / c + 2;
    else h = (r - g) / c + 4;
    h *= 60;
    if (h < 0) h += 360;
  }
  return [h, mx ? c / mx : 0, mx / 255];
}

export const inHue = (h, lo, hi) => (lo <= hi ? h >= lo && h <= hi : h >= lo || h <= hi);

// 'lane', 'white', 'dark' or 'other' for one pixel, with the same rules as process() (for the tap inspector)
export function classify(r, g, b, p) {
  const [h, s, v] = rgb2hsv(r, g, b);
  if (v < p.valMin) return { h, s, v, cls: 'dark' };
  if (s >= p.satMin && inHue(h, p.hueLo, p.hueHi)) return { h, s, v, cls: 'lane' };
  if (v >= p.whiteVal && s <= p.whiteSat) return { h, s, v, cls: 'white' };
  return { h, s, v, cls: 'other' };
}

export class Vision {
  constructor(w, h) {
    this.w = w;
    this.h = h;
    const n = w * h;
    this.col = new Uint8Array(n);
    this.lane = new Uint8Array(n);
    this.drive = new Uint8Array(n);
    this.tmp = new Uint8Array(n);
    this.tmp2 = new Uint8Array(n);
    this.skel = new Uint8Array(n);
    this.cand = new Uint8Array(n);
    this.bea = new Uint8Array(n);
    this.labels = new Int32Array(n);
    this.stack = new Int32Array(n);
    this.dt = new Uint16Array(n);
    this.I = new Int32Array((w + 1) * (h + 1));
    this.W = Math.max(8, w / 10); // lane width in px, re-estimated every frame
    this.reset();
  }

  reset() {
    this.tracks = [];      // [{ id, pos, vel, box, age, missed, beacon, hd }]
    this.nextId = 1;
    this.mine = null;      // id of my robot's track
    this.mineBeacon = false; // my robot has shown the beacon: re-find it by the beacon when lost
    this.picked = false;   // the user picked a robot (then no automatic beacon pick to someone else)
    this.heading = null;   // my robot's heading, unit vector in the picture
    this.t = null;
  }

  // make the track nearest to (x, y) mine; returns its id, or null if no robot is near
  select(x, y) {
    let best = null, bd = Infinity;
    for (const tr of this.tracks) {
      const [x0, y0, x1, y1] = tr.box, reach = Math.max(this.W, Math.hypot(x1 - x0, y1 - y0) / 2 + 0.3 * this.W);
      const d = Math.hypot(tr.pos[0] - x, tr.pos[1] - y);
      if (d < reach && d < bd) { bd = d; best = tr; }
    }
    if (!best) return null;
    this.mine = best.id;
    this.mineBeacon = best.beacon > 0;
    this.picked = true;
    this.heading = best.hd || null;
    return best.id;
  }

  // a new track at (x, y), made mine (for the taught color, before it has been seen)
  seed(x, y) {
    const r = this.W / 4;
    const tr = { id: this.nextId++, pos: [x, y], vel: [0, 0], box: [x - r, y - r, x + r, y + r], age: 1, missed: 0, beacon: 0, hd: null };
    this.tracks.push(tr);
    this.mine = tr.id;
    this.picked = true;
    this.heading = null;
  }

  // learn the robot's color (chromaticity and brightness) from a 5×5 patch around (x, y)
  teach(data, x, y) {
    let r = 0, g = 0, b = 0, k = 0;
    for (let yy = Math.max(0, y - 2); yy <= Math.min(this.h - 1, y + 2); yy++) {
      for (let xx = Math.max(0, x - 2); xx <= Math.min(this.w - 1, x + 2); xx++) {
        const j = (yy * this.w + xx) * 4;
        r += data[j]; g += data[j + 1]; b += data[j + 2]; k++;
      }
    }
    const s = r + g + b || 1;
    this.color = { r: r / s, g: g / s, sum: s / k };
    if (this.select(x, y) == null) this.seed(x, y);
    return this.color;
  }

  // ---- binary image helpers (masks are Uint8Array of 0/1) ----

  integral(m) {
    const { w, h, I } = this, W1 = w + 1;
    for (let y = 0; y < h; y++) {
      let row = 0;
      for (let x = 0; x < w; x++) {
        row += m[y * w + x];
        I[(y + 1) * W1 + x + 1] = I[y * W1 + x + 1] + row;
      }
    }
  }

  // out = 1 where the (2r+1)² box around the pixel has any (dilate) or only (erode) set pixels; the box is clipped
  // at the image border, so the outside counts as "don't know" instead of empty
  box(m, r, out, erode) {
    const { w, h, I } = this, W1 = w + 1;
    this.integral(m);
    for (let y = 0; y < h; y++) {
      const y0 = Math.max(0, y - r), y1 = Math.min(h, y + r + 1);
      for (let x = 0; x < w; x++) {
        const x0 = Math.max(0, x - r), x1 = Math.min(w, x + r + 1);
        const s = I[y1 * W1 + x1] - I[y0 * W1 + x1] - I[y1 * W1 + x0] + I[y0 * W1 + x0];
        out[y * w + x] = erode ? (s === (x1 - x0) * (y1 - y0) ? 1 : 0) : s > 0 ? 1 : 0;
      }
    }
  }

  // 4-connected components of m; returns [{id, area, cx, cy, x0, y0, x1, y1, border}], labels in this.labels
  components(m) {
    const { w, h, labels, stack } = this, n = w * h;
    labels.fill(0);
    const comps = [];
    for (let i = 0; i < n; i++) {
      if (!m[i] || labels[i]) continue;
      const id = comps.length + 1;
      const c = { id, area: 0, cx: 0, cy: 0, x0: w, y0: h, x1: 0, y1: 0, border: false };
      let sp = 0;
      stack[sp++] = i;
      labels[i] = id;
      while (sp) {
        const k = stack[--sp], x = k % w, y = (k - x) / w;
        c.area++; c.cx += x; c.cy += y;
        if (x < c.x0) c.x0 = x;
        if (x > c.x1) c.x1 = x;
        if (y < c.y0) c.y0 = y;
        if (y > c.y1) c.y1 = y;
        if (x === 0 || y === 0 || x === w - 1 || y === h - 1) c.border = true;
        if (x > 0 && m[k - 1] && !labels[k - 1]) { labels[k - 1] = id; stack[sp++] = k - 1; }
        if (x < w - 1 && m[k + 1] && !labels[k + 1]) { labels[k + 1] = id; stack[sp++] = k + 1; }
        if (y > 0 && m[k - w] && !labels[k - w]) { labels[k - w] = id; stack[sp++] = k - w; }
        if (y < h - 1 && m[k + w] && !labels[k + w]) { labels[k + w] = id; stack[sp++] = k + w; }
      }
      c.cx /= c.area;
      c.cy /= c.area;
      comps.push(c);
    }
    return comps;
  }

  // Zhang-Suen thinning of m into this.skel
  thin(m) {
    const { w, h, skel: s } = this;
    s.set(m);
    for (let x = 0; x < w; x++) { s[x] = 0; s[(h - 1) * w + x] = 0; }
    for (let y = 0; y < h; y++) { s[y * w] = 0; s[y * w + w - 1] = 0; }
    const del = this.stack;
    for (let iter = 0; iter < 60; iter++) {
      let changed = 0;
      for (let pass = 0; pass < 2; pass++) {
        let nd = 0;
        for (let y = 1; y < h - 1; y++) {
          for (let x = 1; x < w - 1; x++) {
            const i = y * w + x;
            if (!s[i]) continue;
            const p2 = s[i - w], p3 = s[i - w + 1], p4 = s[i + 1], p5 = s[i + w + 1];
            const p6 = s[i + w], p7 = s[i + w - 1], p8 = s[i - 1], p9 = s[i - w - 1];
            const B = p2 + p3 + p4 + p5 + p6 + p7 + p8 + p9;
            if (B < 2 || B > 6) continue;
            const A = (!p2 && p3) + (!p3 && p4) + (!p4 && p5) + (!p5 && p6) + (!p6 && p7) + (!p7 && p8) + (!p8 && p9) + (!p9 && p2);
            if (A !== 1) continue;
            if (pass === 0 ? p2 * p4 * p6 || p4 * p6 * p8 : p2 * p4 * p8 || p2 * p6 * p8) continue;
            del[nd++] = i;
          }
        }
        for (let k = 0; k < nd; k++) s[del[k]] = 0;
        changed += nd;
      }
      if (!changed) break;
    }
  }

  // chamfer (3-4) distance to the nearest pixel outside m, in thirds of a pixel; the image border counts as inside
  distance(m) {
    const { w, h, dt } = this, BIG = 65000;
    for (let i = 0; i < w * h; i++) dt[i] = m[i] ? BIG : 0;
    for (let y = 0; y < h; y++) {
      for (let x = 0; x < w; x++) {
        const i = y * w + x;
        if (!dt[i]) continue;
        let v = dt[i];
        if (x > 0) v = Math.min(v, dt[i - 1] + 3);
        if (y > 0) {
          v = Math.min(v, dt[i - w] + 3);
          if (x > 0) v = Math.min(v, dt[i - w - 1] + 4);
          if (x < w - 1) v = Math.min(v, dt[i - w + 1] + 4);
        }
        dt[i] = v;
      }
    }
    for (let y = h - 1; y >= 0; y--) {
      for (let x = w - 1; x >= 0; x--) {
        const i = y * w + x;
        if (!dt[i]) continue;
        let v = dt[i];
        if (x < w - 1) v = Math.min(v, dt[i + 1] + 3);
        if (y < h - 1) {
          v = Math.min(v, dt[i + w] + 3);
          if (x < w - 1) v = Math.min(v, dt[i + w + 1] + 4);
          if (x > 0) v = Math.min(v, dt[i + w - 1] + 4);
        }
        dt[i] = v;
      }
    }
  }

  // march from (x, y) along (dx, dy) until the drive mask ends; edge = false if the ray left the picture first
  ray(x, y, dx, dy, max) {
    const { w, h, drive } = this;
    for (let t = 0; t < max; t += 0.5) {
      const xi = Math.round(x + dx * t), yi = Math.round(y + dy * t);
      if (xi < 0 || yi < 0 || xi >= w || yi >= h) return { t, edge: false };
      if (!drive[yi * w + xi]) return { t, edge: true };
    }
    return { t: max, edge: false };
  }

  // ---- the pipeline ----

  process(data, tMs, p = VDEFAULTS) {
    const t0 = typeof performance !== 'undefined' ? performance.now() : Date.now();
    const { w, h, col, lane, drive, tmp, tmp2, cand, bea } = this, n = w * h;
    const vMin = p.valMin * 255, wVal = p.whiteVal * 255, bVal = p.bVal * 255;
    const useBeacon = p.robotSrc === 'auto' || p.robotSrc === 'beacon';

    // 1. colored lane pixels, and white candidates in tmp2
    for (let i = 0, j = 0; i < n; i++, j += 4) {
      const r = data[j], g = data[j + 1], b = data[j + 2];
      const mx = r > g ? (r > b ? r : b) : g > b ? g : b;
      const mn = r < g ? (r < b ? r : b) : g < b ? g : b;
      col[i] = 0;
      tmp2[i] = 0;
      bea[i] = 0;
      if (mx < vMin && mx < bVal) continue;
      const c = mx - mn, s = c / mx;
      if (s >= p.satMin || s >= p.bSat) {
        let hh = mx === r ? ((g - b) / c) % 6 : mx === g ? (b - r) / c + 2 : (r - g) / c + 4;
        hh *= 60;
        if (hh < 0) hh += 360;
        if (useBeacon && s >= p.bSat && mx >= bVal && inHue(hh, p.bHueLo, p.bHueHi)) bea[i] = 1;
        else if (s >= p.satMin && mx >= vMin && inHue(hh, p.hueLo, p.hueHi)) col[i] = 1;
      } else if (mx < vMin) continue;
      else if (p.white && mx >= wVal && s <= p.whiteSat) tmp2[i] = 1;
    }
    // 2. lane = colored ∪ (white ∧ near colored); white floor far from the lane stays out
    if (p.white) {
      this.box(col, 2, tmp, false);
      for (let i = 0; i < n; i++) lane[i] = col[i] | (tmp2[i] & tmp[i]);
    } else lane.set(col);
    // 3. remove speckle (opening, radius 1), then drop blobs much smaller than the biggest
    this.box(lane, 1, tmp, true);
    this.box(tmp, 1, lane, false);
    let comps = this.components(lane);
    const big = comps.reduce((m, c) => Math.max(m, c.area), 0);
    const keepMin = Math.max(n * 0.002, big * 0.1);
    const keep = new Uint8Array(comps.length + 1);
    for (const c of comps) keep[c.id] = c.area >= keepMin ? 1 : 0;
    let laneN = 0;
    for (let i = 0; i < n; i++) { lane[i] = keep[this.labels[i]]; laneN += lane[i]; }

    // 4. drive = lane closed with a radius tied to the lane width, plus enclosed holes up to robot size
    const rc = Math.max(1, Math.round(p.closeK * this.W));
    this.box(lane, rc, tmp, false);
    this.box(tmp, rc, drive, true);
    for (let i = 0; i < n; i++) tmp[i] = drive[i] ? 0 : 1;
    const holeMax = p.robotMax * this.W * this.W;
    comps = this.components(tmp);
    const fill = new Uint8Array(comps.length + 1);
    for (const c of comps) fill[c.id] = !c.border && c.area <= holeMax ? 1 : 0;
    for (let i = 0; i < n; i++) if (tmp[i] && fill[this.labels[i]]) drive[i] = 1;

    // 5. centerline and lane width (2 × median distance-to-edge along the centerline)
    this.thin(drive);
    this.distance(drive);
    const ds = [];
    for (let i = 0; i < n; i++) if (this.skel[i] && this.dt[i] < 65000) ds.push(this.dt[i]);
    if (ds.length > 10) {
      ds.sort((a, b) => a - b);
      const Wm = (2 * ds[ds.length >> 1]) / 3 + 1;
      this.W += (Math.max(4, Math.min(w / 2, Wm)) - this.W) * 0.5;
    }
    const W = this.W;

    // 6. robot detections: holes in the lane (or the taught color), each joined with a beacon next to it
    const colorMode = p.robotSrc === 'color' && this.color;
    if (colorMode) {
      const { r: cr, g: cg, sum } = this.color;
      this.box(drive, Math.max(1, Math.round(W / 3)), tmp, false);
      for (let i = 0, j = 0; i < n; i++, j += 4) {
        const s = data[j] + data[j + 1] + data[j + 2];
        cand[i] = tmp[i] && s > sum * 0.4 && s < sum * 2.5 && Math.abs(data[j] / s - cr) + Math.abs(data[j + 1] / s - cg) < p.colorTol ? 1 : 0;
      }
    } else {
      for (let i = 0; i < n; i++) cand[i] = drive[i] & (lane[i] ^ 1);
    }
    const aMin = p.robotMin * W * W, aMax = p.robotMax * W * W;
    // robots are compact blobs; thin slivers along the lane edge are left over from gap closing
    const thick = Math.max(2, 0.2 * W);
    const holes = this.components(cand).filter((c) => c.area >= (colorMode ? 4 : aMin) && c.area <= aMax &&
      (colorMode || (Math.min(c.x1 - c.x0, c.y1 - c.y0) + 1 >= thick && c.area >= 0.3 * (c.x1 - c.x0 + 1) * (c.y1 - c.y0 + 1) &&
        Math.max(c.x1 - c.x0, c.y1 - c.y0) + 1 <= 3 * (Math.min(c.x1 - c.x0, c.y1 - c.y0) + 1))));
    let beacons = [];
    if (useBeacon) {
      this.box(bea, Math.max(1, Math.round(W * 0.15)), tmp, false); // the two headlights merge into one blob
      beacons = this.components(tmp).filter((c) => c.area >= 6 && c.area <= aMax);
    }
    const dets = [], used = new Set();
    for (const b of beacons) {
      let body = null, bd = 1.2 * W;
      for (const c of holes) {
        const d = Math.hypot(c.cx - b.cx, c.cy - b.cy);
        if (!used.has(c) && d < bd) { bd = d; body = c; }
      }
      if (body) used.add(body);
      const o = body || b;
      dets.push({ x: o.cx, y: o.cy, box: [Math.min(o.x0, b.x0), Math.min(o.y0, b.y0), Math.max(o.x1, b.x1), Math.max(o.y1, b.y1)], beacon: true, border: false,
        hd: body && bd > 0.08 * W ? [(b.cx - body.cx) / bd, (b.cy - body.cy) / bd] : null });
    }
    if (p.robotSrc !== 'beacon') {
      for (const c of holes) if (!used.has(c)) dets.push({ x: c.cx, y: c.cy, box: [c.x0, c.y0, c.x1, c.y1], beacon: !!colorMode, border: c.border, hd: null });
    }

    // 7. tracking: match each track to the nearest detection around where it was heading, closest pairs first
    const dt = this.t == null ? 0.05 : Math.min(0.5, Math.max(0.001, (tMs - this.t) / 1000));
    this.t = tMs;
    const pairs = [];
    for (const tr of this.tracks) {
      tr.pred = [tr.pos[0] + tr.vel[0] * dt, tr.pos[1] + tr.vel[1] * dt];
      const gate = Math.min(4 * W, Math.max(1.2 * W, 2 * Math.hypot(...tr.vel) * dt) * (1 + 0.3 * tr.missed));
      dets.forEach((d, k) => {
        const dist = Math.hypot(d.x - tr.pred[0], d.y - tr.pred[1]);
        if (dist <= gate) pairs.push([dist - (d.beacon && tr.beacon > 0 ? 0.5 * W : 0), tr, k]);
      });
    }
    pairs.sort((a, b) => a[0] - b[0]);
    const tMatched = new Set(), dMatched = new Set();
    const take = (tr, d) => {
      const k = Math.min(1, dt / 0.25);
      if (tr.age > 1 && !tr.missed) {
        tr.vel = [tr.vel[0] + ((d.x - tr.pos[0]) / dt - tr.vel[0]) * k, tr.vel[1] + ((d.y - tr.pos[1]) / dt - tr.vel[1]) * k];
      }
      tr.pos = [d.x, d.y];
      tr.box = d.box;
      tr.hd = d.hd;
      tr.beacon = d.beacon ? 30 : Math.max(0, tr.beacon - 1); // frames the beacon counts as seen
      tr.missed = 0;
      tr.age++;
      tMatched.add(tr);
    };
    for (const [, tr, k] of pairs) {
      if (tMatched.has(tr) || dMatched.has(k)) continue;
      dMatched.add(k);
      take(tr, dets[k]);
    }
    let mineTr = this.tracks.find((tr) => tr.id === this.mine) || null;
    // my robot shows the beacon: if its track lost it, the beacon robot is still mine
    const beaconDet = dets.findIndex((d, k) => d.beacon && !dMatched.has(k));
    if (beaconDet >= 0 && (mineTr ? !tMatched.has(mineTr) && this.mineBeacon : this.mineBeacon || !this.picked)) {
      if (!mineTr) {
        mineTr = { id: this.nextId++, pos: [0, 0], vel: [0, 0], box: null, age: 1, missed: 0, beacon: 0, hd: null };
        this.tracks.push(mineTr);
        this.mine = mineTr.id;
      }
      dMatched.add(beaconDet);
      take(mineTr, dets[beaconDet]);
      mineTr.vel = [0, 0]; // jumped: no speed from that
      this.mineBeacon = true;
    }
    for (const tr of this.tracks) {
      if (tMatched.has(tr)) continue;
      tr.missed++;
      tr.pos = tr.pred;
      tr.vel = [tr.vel[0] * 0.7, tr.vel[1] * 0.7];
      tr.beacon = Math.max(0, tr.beacon - 1);
    }
    // forget tracks not seen for a while (mine is kept longer, it may come back into view)
    this.tracks = this.tracks.filter((tr) => (tr.id === this.mine ? tr.missed <= 45 : tr.missed <= 8 && (tr.age > 2 || !tr.missed)));
    dets.forEach((d, k) => {
      if (dMatched.has(k) || d.border) return; // a blob cut by the picture's edge only continues a track
      this.tracks.push({ id: this.nextId++, pos: [d.x, d.y], vel: [0, 0], box: d.box, age: 1, missed: 0, beacon: d.beacon ? 30 : 0, hd: d.hd });
    });
    mineTr = this.tracks.find((tr) => tr.id === this.mine) || null;
    if (!mineTr && this.mine != null) { this.mine = null; this.heading = null; }
    if (mineTr && mineTr.beacon > 0) this.mineBeacon = true;

    // 8. pose of my robot in the lane
    const out = {
      W, laneFrac: laneN / n, dets, mine: this.mine,
      tracks: this.tracks.filter((tr) => tr.age >= 3 || tr.id === this.mine).map((tr) => ({ id: tr.id, x: tr.pos[0], y: tr.pos[1], box: tr.box, missed: tr.missed, beacon: tr.beacon > 0, mine: tr.id === this.mine })),
      robot: mineTr ? [...mineTr.pos] : null, found: !!mineTr && !mineTr.missed, lost: mineTr ? mineTr.missed : null,
    };
    if (out.robot) {
      const [x, y] = out.robot;
      // lane direction from the centerline near the robot (principal axis)
      let sx = 0, sy = 0, sxx = 0, syy = 0, sxy = 0, k = 0;
      const R = Math.ceil(W), x0 = Math.max(0, Math.round(x) - R), x1 = Math.min(w - 1, Math.round(x) + R);
      for (let yy = Math.max(0, Math.round(y) - R); yy <= Math.min(h - 1, Math.round(y) + R); yy++) {
        for (let xx = x0; xx <= x1; xx++) {
          if (!this.skel[yy * w + xx] || (xx - x) ** 2 + (yy - y) ** 2 > R * R) continue;
          sx += xx; sy += yy; sxx += xx * xx; syy += yy * yy; sxy += xx * yy; k++;
        }
      }
      let axis = null;
      if (k >= 3) {
        const cxx = sxx / k - (sx / k) ** 2, cyy = syy / k - (sy / k) ** 2, cxy = sxy / k - (sx / k) * (sy / k);
        const a = 0.5 * Math.atan2(2 * cxy, cxx - cyy);
        axis = [Math.cos(a), Math.sin(a)];
      }
      // heading: the beacon if seen; else the lane direction, pointing the same way as last frame (or as the
      // track's motion in the picture when there is no last frame; that is only right while the phone is held still)
      const sp = Math.hypot(...mineTr.vel);
      const prev = this.heading || (sp > W ? [mineTr.vel[0] / sp, mineTr.vel[1] / sp] : [0, -1]);
      const beaconHeading = mineTr.missed ? null : mineTr.hd;
      if (beaconHeading) this.heading = beaconHeading;
      else if (axis) this.heading = axis[0] * prev[0] + axis[1] * prev[1] < 0 ? [-axis[0], -axis[1]] : axis;
      out.axis = axis;
      out.heading = this.heading;
      out.headingFrom = beaconHeading ? 'beacon' : axis ? 'lane' : this.heading ? 'last' : null;
      const dir = this.heading;
      const xi = Math.round(x), yi = Math.round(y);
      out.onLane = xi >= 0 && yi >= 0 && xi < w && yi < h && !!drive[yi * w + xi];
      if (dir && out.onLane) {
        const [dx, dy] = dir;
        const max = 4 * W;
        const L = this.ray(x, y, dy, -dx, max), Rr = this.ray(x, y, -dy, dx, max);
        out.dl = L.t;
        out.dr = Rr.t;
        out.offset = (L.t - Rr.t) / Math.max(1, L.t + Rr.t); // -1 = on the left edge, +1 = on the right edge
        out.offsetSure = L.edge && Rr.edge;
        out.ahead = this.ray(x, y, dx, dy, max).t / W;
        out.fan = [];
        let bestA = 0, bestT = -1;
        for (let a = -60; a <= 60; a += 10) {
          const ar = (a * Math.PI) / 180, c = Math.cos(ar), s = Math.sin(ar);
          const fx = dx * c - dy * s, fy = dy * c + dx * s; // positive angle = to the robot's right
          const r = this.ray(x, y, fx, fy, max).t;
          out.fan.push([a, r, fx, fy]);
          if (r > bestT + 0.5 || (Math.abs(r - bestT) <= 0.5 && Math.abs(a) < Math.abs(bestA))) { bestT = r; bestA = a; }
        }
        out.steer = bestA; // degrees, + = right
      }
    }
    out.ms = (typeof performance !== 'undefined' ? performance.now() : Date.now()) - t0;
    return out;
  }
}

// Outline of a mask as line segments [x1, y1, x2, y2, …] in pixel units (marching squares on a 3×3 smoothed
// copy, so the line is smooth instead of stair-stepped). Nothing is drawn along the picture's border.
export function contour(m, w, h) {
  const f = new Uint8Array(w * h);
  for (let y = 0; y < h; y++) {
    for (let x = 0; x < w; x++) {
      let s = 0;
      for (let yy = Math.max(0, y - 1); yy <= Math.min(h - 1, y + 1); yy++) {
        for (let xx = Math.max(0, x - 1); xx <= Math.min(w - 1, x + 1); xx++) s += m[yy * w + xx];
      }
      f[y * w + x] = s;
    }
  }
  const T = 4.5, seg = [];
  const cut = (x1, y1, v1, x2, y2, v2) => { const t = (T - v1) / (v2 - v1); return [x1 + (x2 - x1) * t + 0.5, y1 + (y2 - y1) * t + 0.5]; };
  for (let y = 0; y < h - 1; y++) {
    for (let x = 0; x < w - 1; x++) {
      const a = f[y * w + x], b = f[y * w + x + 1], c = f[(y + 1) * w + x + 1], d = f[(y + 1) * w + x];
      const code = (a > T ? 1 : 0) | (b > T ? 2 : 0) | (c > T ? 4 : 0) | (d > T ? 8 : 0);
      if (code === 0 || code === 15) continue;
      const top = () => cut(x, y, a, x + 1, y, b), right = () => cut(x + 1, y, b, x + 1, y + 1, c);
      const bottom = () => cut(x, y + 1, d, x + 1, y + 1, c), left = () => cut(x, y, a, x, y + 1, d);
      const add = (p, q) => seg.push(p[0], p[1], q[0], q[1]);
      switch (code) {
        case 1: case 14: add(left(), top()); break;
        case 2: case 13: add(top(), right()); break;
        case 3: case 12: add(left(), right()); break;
        case 4: case 11: add(right(), bottom()); break;
        case 6: case 9: add(top(), bottom()); break;
        case 7: case 8: add(left(), bottom()); break;
        case 5: add(left(), top()); add(right(), bottom()); break;
        case 10: add(top(), right()); add(left(), bottom()); break;
      }
    }
  }
  return seg;
}
