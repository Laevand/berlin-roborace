// Demo mode: a fake Cutebot on an oval that speaks the same UART protocol as microbitapi.js.
// Two tracks (Tune → Demo): "lane" is a wide white lane on black, like the real rally mat, where the lane
// reads white and the mat black; "line" is a thin black line on white. Bluetooth delay is simulated too.
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

export class SimTransport {
  // opts() returns { track: 'lane' | 'line', latency: extra round-trip ms }, read live.
  constructor(onText, canvas, opts = () => ({})) {
    this.onText = onText;
    this.opts = opts;
    this.canvas = canvas;
    this.connected = true;
    this.name = 'BBC micro:bit [demo]';
    this.inbuf = '';
    this.reset();
    this.last = performance.now();
    this.timer = setInterval(() => this.step(), 10);
    canvas.addEventListener('click', () => this.reset());
    this.draw = this.draw.bind(this);
    requestAnimationFrame(this.draw);
  }

  reset() {
    this.x = -STRAIGHT / 4;
    this.y = -RADIUS;
    this.th = 0;
    this.l = 0;
    this.r = 0;
    this.v = 0;
    this.w = 0;
    this.trail = [];
  }

  async write(str) {
    this.inbuf += str;
    let i;
    while ((i = this.inbuf.indexOf('#')) >= 0) {
      const cmd = this.inbuf.slice(0, i).trim();
      this.inbuf = this.inbuf.slice(i + 1);
      if (cmd) setTimeout(() => this.exec(cmd), 6 + Math.random() * 6 + (this.opts().latency || 0) / 2);
    }
    await new Promise((r) => setTimeout(r, 2));
  }

  close() {
    this.connected = false;
    clearInterval(this.timer);
  }

  reply(s) {
    setTimeout(() => this.connected && this.onText(s + '#\n'), 8 + Math.random() * 10 + (this.opts().latency || 0) / 2);
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

  wheel(s) {
    const a = Math.abs(s);
    return a <= DEADBAND ? 0 : (Math.sign(s) * (Math.min(a, 100) - DEADBAND) / (100 - DEADBAND)) * VMAX;
  }

  step() {
    const t = performance.now();
    const dt = Math.min(0.05, (t - this.last) / 1000);
    this.last = t;
    const vl = this.wheel(this.l);
    const vr = this.wheel(this.r);
    this.v = (vl + vr) / 2;
    this.w = (vr - vl) / WHEELBASE;
    this.th += this.w * dt;
    this.x += Math.cos(this.th) * this.v * dt;
    this.y += Math.sin(this.th) * this.v * dt;
    if (this.v !== 0 || this.w !== 0) {
      this.trail.push([this.x, this.y]);
      if (this.trail.length > 600) this.trail.shift();
    }
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

  get lane() { return this.opts().track !== 'line'; }

  // true when a point reads black: off the lane, or on the line
  black(x, y) {
    const d = SimTransport.offTrack(x, y);
    return this.lane ? d > LANE_HALF : d < LINE_HALF;
  }

  lineCode() {
    const [left, right] = this.sensors();
    return (this.black(...left) ? 2 : 0) + (this.black(...right) ? 1 : 0);
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
    const worldW = STRAIGHT + 2 * RADIUS + 2 * LANE_HALF + 10;
    const worldH = 2 * RADIUS + 2 * LANE_HALF + 10;
    const k = Math.min(W / worldW, H / worldH);
    g.setTransform(k, 0, 0, -k, W / 2, H / 2);
    g.fillStyle = this.lane ? '#111' : '#f4f4f4';
    g.fillRect(-worldW, -worldH, 2 * worldW, 2 * worldH);
    g.strokeStyle = this.lane ? '#f4f4f4' : '#111';
    g.lineWidth = (this.lane ? LANE_HALF : LINE_HALF) * 2;
    g.beginPath();
    g.moveTo(-STRAIGHT / 2, -RADIUS);
    g.lineTo(STRAIGHT / 2, -RADIUS);
    g.arc(STRAIGHT / 2, 0, RADIUS, -Math.PI / 2, Math.PI / 2);
    g.lineTo(-STRAIGHT / 2, RADIUS);
    g.arc(-STRAIGHT / 2, 0, RADIUS, Math.PI / 2, (3 * Math.PI) / 2);
    g.stroke();
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
  }
}
