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

// "rally": closed Catmull-Rom spline through these control points (cm). Placeholder shape until the real
// mat is measured: bottom straight (start), right bend, top S-bend, left bend.
const RALLY_PTS = [[-90, -60], [0, -60], [90, -60], [150, -35], [160, 15], [120, 60], [60, 70], [20, 42],
  [-20, 42], [-60, 70], [-110, 65], [-155, 30], [-155, -25], [-125, -52]];
const RALLY_HALF = 10;     // cm, lane is 20 cm wide
const MOTOR_TAU = 0.15;    // s, wheel speed lag (rally)
const GRIP_AY = 300;       // cm/s², lateral acceleration limit (rally)

function buildRally() {
  const P = RALLY_PTS, n = P.length, K = 24, pts = [];
  for (let i = 0; i < n; i++) {
    const [p0, p1, p2, p3] = [P[(i + n - 1) % n], P[i], P[(i + 1) % n], P[(i + 2) % n]];
    for (let k = 0; k < K; k++) {
      const t = k / K, t2 = t * t, t3 = t2 * t;
      pts.push([0, 1].map((d) => 0.5 * (2 * p1[d] + (p2[d] - p0[d]) * t + (2 * p0[d] - 5 * p1[d] + 4 * p2[d] - p3[d]) * t2 + (3 * p1[d] - p0[d] - 3 * p2[d] + p3[d]) * t3)));
    }
  }
  return pts;
}
const RALLY = buildRally();

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
      worldW = 2 * Math.max(...RALLY.map((q) => Math.abs(q[0]))) + 2 * RALLY_HALF + 10;
      worldH = 2 * Math.max(...RALLY.map((q) => Math.abs(q[1]))) + 2 * RALLY_HALF + 10;
    }
    const k = Math.min(W / worldW, H / worldH);
    g.setTransform(k, 0, 0, -k, W / 2, H / 2);
    g.fillStyle = this.lane ? '#111' : '#f4f4f4';
    g.fillRect(-worldW, -worldH, 2 * worldW, 2 * worldH);
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
      g.font = `${12 * dpr}px monospace`;
      const best = this.laps.length ? Math.min(...this.laps) : 0;
      g.fillText(`t ${this.time.toFixed(1)} s  laps ${this.laps.length}  last ${(this.laps[this.laps.length - 1] || 0).toFixed(2)}  best ${best.toFixed(2)}  off ${this.offTime.toFixed(1)} s${this.linkModel.slow ? '  [slow link]' : ''}`, 6 * dpr, 16 * dpr);
    }
  }
}
