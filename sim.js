// Demo mode: a fake Cutebot that speaks the same UART protocol as microbitapi.js.
// Three tracks (Tune → Demo): "lane" is a wide white lane on an oval, "line" is a thin black line on white,
// and "rally" is a 20 cm white lane with an S-bend and a long loop, with motor lag, a grip limit and a lap
// timer. The Bluetooth delay can also jump between fast and slow like the real link ("varying").
// SimCar (physics, track, laps) and LinkModel have no DOM, so tools/simrun.mjs runs them in Node.
// Tap the drawing to put the car back on the start line. Only a sanity check: the real robot will differ.

const STRAIGHT = 120;      // cm, length of each straight
const RADIUS = 45;         // cm, radius of the curves
const LINE_HALF = 1.2;     // cm, half width of the black line ("line" track)
const LANE_HALF = 12;      // cm, half width of the white lane ("lane" track)
const SENSOR_HALF = 0.8;   // cm, sensor offset left/right of center
const SENSOR_AHEAD = 6;    // cm, sensors ahead of the wheel axle
const WHEELBASE = 9;       // cm
const VMAX = 50;           // cm/s at motor 100
const DEADBAND = 20;       // motor values below this do not move the wheel

// "rally": the official mat (droidconHQ/CuteBotDriver images/race_booth_area.jpg, and the photo of the floor).
// Corners [x, y, fillet radius] in photo pixels, scaled by CM_PER_PX (the lane is ~35 px = 20 cm wide).
// Start on the bottom straight heading east, right-hand loop, long top strand west, then the meander:
// five vertical strands joined by four U-turns, ending on the bottom straight again.
// Traced by eye from a perspective photo: lane width 20 cm is measured, the rest is approximate.
const CM_PER_PX = 0.55;
const RALLY_CORNERS = [[700, 505, 0], [990, 505, 80], [990, 222, 80], [190, 222, 40], [190, 505, 35], [260, 505, 35],
  [260, 325, 35], [330, 325, 35], [330, 505, 35], [400, 505, 35], [400, 325, 35], [470, 325, 35], [470, 505, 35]];
const RALLY_HALF = 10;     // cm, lane is 20 cm wide
const MOTOR_TAU = 0.15;    // s, wheel speed lag (rally)
const GRIP_AY = 300;       // cm/s², lateral acceleration limit (rally)

// closed polyline through the corners, each rounded with its fillet radius, a point about every 4 cm
function buildRally() {
  const V = RALLY_CORNERS.map(([x, y, r]) => [(x - 580) * CM_PER_PX, -(y - 370) * CM_PER_PX, r * CM_PER_PX]);
  const pts = [];
  const line = (a, b) => {
    const n = Math.max(1, Math.ceil(Math.hypot(b[0] - a[0], b[1] - a[1]) / 4));
    for (let k = 0; k < n; k++) pts.push([a[0] + ((b[0] - a[0]) * k) / n, a[1] + ((b[1] - a[1]) * k) / n]);
  };
  let cur = null; // end of the previous fillet (or the start vertex)
  const n = V.length;
  for (let i = 0; i < n; i++) {
    const [vx, vy, r] = V[i], p = V[(i + n - 1) % n], q = V[(i + 1) % n];
    if (!r) { cur = [vx, vy]; continue; }
    const u1 = [p[0] - vx, p[1] - vy], u2 = [q[0] - vx, q[1] - vy];
    const l1 = Math.hypot(...u1), l2 = Math.hypot(...u2);
    u1[0] /= l1; u1[1] /= l1; u2[0] /= l2; u2[1] /= l2;
    const th = Math.acos(u1[0] * u2[0] + u1[1] * u2[1]);      // angle between the two sides
    const t = r / Math.tan(th / 2);                           // distance from corner to tangent points
    const a = [vx + u1[0] * t, vy + u1[1] * t], b = [vx + u2[0] * t, vy + u2[1] * t];
    const bis = [u1[0] + u2[0], u1[1] + u2[1]], bl = Math.hypot(...bis);
    const d = r / Math.sin(th / 2);
    const c = [vx + (bis[0] / bl) * d, vy + (bis[1] / bl) * d];
    line(cur, a);
    let a0 = Math.atan2(a[1] - c[1], a[0] - c[0]), a1 = Math.atan2(b[1] - c[1], b[0] - c[0]);
    let da = a1 - a0;
    while (da > Math.PI) da -= 2 * Math.PI;
    while (da < -Math.PI) da += 2 * Math.PI;
    const steps = Math.max(2, Math.ceil((Math.abs(da) * r) / 4));
    for (let k = 0; k < steps; k++) pts.push([c[0] + r * Math.cos(a0 + (da * k) / steps), c[1] + r * Math.sin(a0 + (da * k) / steps)]);
    cur = b;
  }
  line(cur, [V[0][0], V[0][1]]); // back to the start
  return pts;
}
export const RALLY = buildRally(); // also drawn by autopilot/explore.js
const RALLY_BOX = [0, 1].map((d) => [Math.min(...RALLY.map((q) => q[d])), Math.max(...RALLY.map((q) => q[d]))]);

// Round-trip Bluetooth delay model. 'steady' = base + jitter. 'varying' = flips between fast and slow
// (about 350 ms extra), like the iPhone link measured at the booth.
export class LinkModel {
  constructor(rand = Math.random) { this.rand = rand; this.slow = false; this.flipAt = 0; }
  rtt(t, mode, base) {
    if (mode === 'varying') {
      if (t >= this.flipAt) {
        this.slow = this.flipAt > 0 && !this.slow;
        this.flipAt = t + (this.slow ? 2000 + this.rand() * 4000 : 4000 + this.rand() * 8000);
      }
    } else this.slow = false;
    return base + (this.slow ? 350 : 0);
  }
}

export class SimCar {
  // opts() returns { track: 'lane' | 'line' | 'rally', vmax }, read live.
  constructor(opts = () => ({})) {
    this.opts = opts;
    this.time = 0;
    this.reset();
  }

  get rally() { return this.trackName === 'rally'; }

  reset() {
    this.trackName = this.opts().track || 'lane';
    this.l = 0;
    this.r = 0;
    this.v = 0;
    this.w = 0;
    this.vl = 0;
    this.vr = 0;
    this.trail = [];
    this.laps = [];
    this.offTime = 0;
    this.time = 0;
    this.lapStart = 0;
    this.maxProg = 0;
    this.prevIdx = 0;
    if (this.rally) {
      this.x = RALLY[0][0];
      this.y = RALLY[0][1];
      this.th = Math.atan2(RALLY[1][1] - RALLY[0][1], RALLY[1][0] - RALLY[0][0]);
    } else {
      this.x = -STRAIGHT / 4;
      this.y = -RADIUS;
      this.th = 0;
    }
  }

  wheel(s) {
    const a = Math.abs(s);
    return a <= DEADBAND ? 0 : (Math.sign(s) * (Math.min(a, 100) - DEADBAND) / (100 - DEADBAND)) * (this.opts().vmax || VMAX);
  }

  step(dt) {
    if ((this.opts().track || 'lane') !== this.trackName) this.reset();
    const tl = this.wheel(this.l);
    const tr = this.wheel(this.r);
    if (this.rally) {
      const k = 1 - Math.exp(-dt / MOTOR_TAU);
      this.vl += (tl - this.vl) * k;
      this.vr += (tr - this.vr) * k;
    } else {
      this.vl = tl;
      this.vr = tr;
    }
    this.v = (this.vl + this.vr) / 2;
    this.w = (this.vr - this.vl) / WHEELBASE;
    if (this.rally && Math.abs(this.v * this.w) > GRIP_AY) this.w = Math.sign(this.w) * GRIP_AY / Math.abs(this.v); // slides wide
    this.th += this.w * dt;
    this.x += Math.cos(this.th) * this.v * dt;
    this.y += Math.sin(this.th) * this.v * dt;
    this.time += dt;
    if (this.rally) this.lapStep(dt);
    if (this.v !== 0 || this.w !== 0) {
      this.trail.push([this.x, this.y]);
      if (this.trail.length > 600) this.trail.shift();
    }
  }

  // nearest point on the rally center line: { d: distance (cm), i: sample index }
  static nearestRally(x, y) {
    let best = Infinity, bi = 0;
    const n = RALLY.length;
    for (let i = 0; i < n; i++) {
      const [ax, ay] = RALLY[i], [bx, by] = RALLY[(i + 1) % n];
      const ex = bx - ax, ey = by - ay;
      const u = Math.max(0, Math.min(1, ((x - ax) * ex + (y - ay) * ey) / (ex * ex + ey * ey)));
      const d = Math.hypot(x - ax - u * ex, y - ay - u * ey);
      if (d < best) { best = d; bi = i; }
    }
    return { d: best, i: bi };
  }

  lapStep(dt) {
    const n = RALLY.length;
    const { d, i } = SimCar.nearestRally(this.x, this.y);
    if (d > RALLY_HALF) this.offTime += dt; // whole car center outside the lane
    const prog = i / n;
    if (this.prevIdx > 0.9 * n && i < 0.1 * n) {
      if (this.maxProg > 0.85) { this.laps.push(this.time - this.lapStart); this.lapStart = this.time; }
      this.maxProg = 0;
    } else if (prog > this.maxProg && prog - this.maxProg < 0.15) this.maxProg = prog;
    this.prevIdx = i;
  }

  // distance from a point to the oval's center line
  static offTrack(x, y) {
    const h = STRAIGHT / 2;
    if (Math.abs(x) <= h) return Math.abs(Math.abs(y) - RADIUS);
    const cx = Math.sign(x) * h;
    return Math.abs(Math.hypot(x - cx, y) - RADIUS);
  }

  sensors() {
    const fx = Math.cos(this.th);
    const fy = Math.sin(this.th);
    const lx = -fy;
    const ly = fx;
    const ax = this.x + fx * SENSOR_AHEAD;
    const ay = this.y + fy * SENSOR_AHEAD;
    return [
      [ax + lx * SENSOR_HALF, ay + ly * SENSOR_HALF],
      [ax - lx * SENSOR_HALF, ay - ly * SENSOR_HALF],
    ];
  }

  get lane() { return this.trackName !== 'line'; }

  // true when a point reads black: off the lane, or on the line
  black(x, y) {
    if (this.rally) return SimCar.nearestRally(x, y).d > RALLY_HALF;
    const d = SimCar.offTrack(x, y);
    return this.lane ? d > LANE_HALF : d < LINE_HALF;
  }

  lineCode() {
    const [left, right] = this.sensors();
    return (this.black(...left) ? 2 : 0) + (this.black(...right) ? 1 : 0);
  }
}

export class SimTransport extends SimCar {
  // opts() returns { track: 'lane' | 'line' | 'rally', latency: extra round-trip ms, link: 'steady' | 'varying' }, read live.
  constructor(onText, canvas, opts = () => ({})) {
    super(opts);
    this.onText = onText;
    this.canvas = canvas;
    this.connected = true;
    this.name = 'BBC micro:bit [demo]';
    this.inbuf = '';
    this.linkModel = new LinkModel();
    this.last = performance.now();
    this.timer = setInterval(() => this.tick(), 10);
    canvas.addEventListener('click', () => this.reset());
    this.draw = this.draw.bind(this);
    requestAnimationFrame(this.draw);
  }

  // current round-trip delay in ms
  rtt() {
    const o = this.opts();
    return this.linkModel.rtt(performance.now(), o.link, o.latency || 0);
  }

  async write(str) {
    this.inbuf += str;
    let i;
    while ((i = this.inbuf.indexOf('#')) >= 0) {
      const cmd = this.inbuf.slice(0, i).trim();
      this.inbuf = this.inbuf.slice(i + 1);
      if (cmd) setTimeout(() => this.exec(cmd), 6 + Math.random() * 6 + this.rtt() / 2);
    }
    await new Promise((r) => setTimeout(r, 2));
  }

  close() {
    this.connected = false;
    clearInterval(this.timer);
  }

  reply(s) {
    setTimeout(() => this.connected && this.onText(s + '#\n'), 8 + Math.random() * 10 + this.rtt() / 2);
  }

  exec(cmd) {
    const parts = cmd.split(',');
    const c = parts[0].trim();
    const n = (i, d) => (parts.length > i ? parseInt(parts[i], 10) : d);
    switch (c) {
      case 'F': this.l = this.r = n(1, 50); break;
      case 'B': this.l = this.r = -n(1, 50); break;
      case 'L': this.l = -n(1, 50); this.r = n(1, 50); break;
      case 'R': this.l = n(1, 50); this.r = -n(1, 50); break;
      case 'S': this.l = this.r = 0; break;
      case 'ML': this.l = n(1, this.l); break;
      case 'MR': this.r = n(1, this.r); break;
      case 'MS': if (parts.length > 2) { this.l = n(1, 0); this.r = n(2, 0); } break;
      case '?LINE': this.reply('LINE:' + this.lineCode()); break;
      case '?DIST': this.reply('DIST:' + Math.round(80 + Math.random() * 5)); break;
      case '?ACCEL': {
        const lat = Math.round((this.v * this.w * 1000) / 981);
        this.reply(`ACCEL:${lat},-1010,${Math.round(Math.random() * 20)}`);
        break;
      }
      case '?LIGHT': this.reply('LIGHT:' + Math.round(120 + Math.random() * 10)); break;
      case '?TEMP': this.reply('TEMP:24'); break;
      case '?COMPASS': this.reply('COMPASS:' + Math.round(((90 - (this.th * 180) / Math.PI) % 360 + 360) % 360)); break;
      case 'PING': this.reply('PONG'); break;
      default: break; // lights, sound and display are accepted silently
    }
  }

  tick() {
    const t = performance.now();
    const dt = Math.min(0.05, (t - this.last) / 1000);
    this.last = t;
    this.step(dt);
  }

  draw() {
    if (!this.connected) return;
    requestAnimationFrame(this.draw);
    const c = this.canvas;
    if (!c.clientWidth) return;
    const dpr = window.devicePixelRatio || 1;
    const W = (c.width = c.clientWidth * dpr);
    const H = (c.height = c.clientHeight * dpr);
    const g = c.getContext('2d');
    let worldW = STRAIGHT + 2 * RADIUS + 2 * LANE_HALF + 10;
    let worldH = 2 * RADIUS + 2 * LANE_HALF + 10;
    if (this.rally) {
      worldW = RALLY_BOX[0][1] - RALLY_BOX[0][0] + 2 * RALLY_HALF + 10;
      worldH = RALLY_BOX[1][1] - RALLY_BOX[1][0] + 2 * RALLY_HALF + 34; // room for the lap text on top
    }
    const k = Math.min(W / worldW, H / worldH);
    const mx = this.rally ? (RALLY_BOX[0][0] + RALLY_BOX[0][1]) / 2 : 0;
    const my = this.rally ? (RALLY_BOX[1][0] + RALLY_BOX[1][1]) / 2 + 12 : 0;
    g.setTransform(k, 0, 0, -k, W / 2 - mx * k, H / 2 + my * k);
    g.fillStyle = this.lane ? '#111' : '#f4f4f4';
    g.fillRect(mx - worldW, my - worldH, 2 * worldW, 2 * worldH);
    g.strokeStyle = this.lane ? '#f4f4f4' : '#111';
    g.lineWidth = (this.rally ? RALLY_HALF : this.lane ? LANE_HALF : LINE_HALF) * 2;
    g.beginPath();
    if (this.rally) {
      g.moveTo(...RALLY[0]);
      for (const q of RALLY) g.lineTo(...q);
      g.closePath();
    } else {
      g.moveTo(-STRAIGHT / 2, -RADIUS);
      g.lineTo(STRAIGHT / 2, -RADIUS);
      g.arc(STRAIGHT / 2, 0, RADIUS, -Math.PI / 2, Math.PI / 2);
      g.lineTo(-STRAIGHT / 2, RADIUS);
      g.arc(-STRAIGHT / 2, 0, RADIUS, Math.PI / 2, (3 * Math.PI) / 2);
    }
    g.stroke();
    if (this.rally) {
      g.strokeStyle = '#2bd47d';
      g.lineWidth = 1.5;
      g.beginPath();
      g.moveTo(RALLY[0][0], RALLY[0][1] - RALLY_HALF);
      g.lineTo(RALLY[0][0], RALLY[0][1] + RALLY_HALF);
      g.stroke();
    }
    if (this.trail.length > 1) {
      g.strokeStyle = 'rgba(176,76,255,.6)';
      g.lineWidth = 0.6;
      g.beginPath();
      g.moveTo(...this.trail[0]);
      for (const pt of this.trail) g.lineTo(...pt);
      g.stroke();
    }
    g.save();
    g.translate(this.x, this.y);
    g.rotate(this.th);
    g.fillStyle = 'rgba(61,123,255,.85)';
    g.fillRect(-3, -WHEELBASE / 2, SENSOR_AHEAD + 3, WHEELBASE);
    g.restore();
    const code = this.lineCode();
    const [sl, sr] = this.sensors();
    g.fillStyle = code & 2 ? '#2bd47d' : '#ff3b5c';
    g.beginPath(); g.arc(sl[0], sl[1], 0.9, 0, 7); g.fill();
    g.fillStyle = code & 1 ? '#2bd47d' : '#ff3b5c';
    g.beginPath(); g.arc(sr[0], sr[1], 0.9, 0, 7); g.fill();
    if (this.rally) {
      g.setTransform(1, 0, 0, 1, 0, 0);
      g.fillStyle = '#fff';
      g.font = `${10 * dpr}px monospace`;
      const best = this.laps.length ? Math.min(...this.laps) : 0;
      g.fillText(`${this.time.toFixed(0)}s  laps ${this.laps.length}  last ${(this.laps[this.laps.length - 1] || 0).toFixed(1)}  best ${best.toFixed(1)}  off ${this.offTime.toFixed(1)}${this.linkModel.slow ? '  SLOW LINK' : ''}`, 6 * dpr, 12 * dpr);
    }
  }
}
