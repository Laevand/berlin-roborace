// Camera pilot: the camera (Drive → 📷 Camera, tap your robot) says where the free lane is relative to the
// car's nose (s.cam.steer, degrees, + = right) and where the car sits in the lane (s.cam.offset, -1 left edge …
// +1 right edge). Steer toward the free lane, nudge back to the middle, slow down in tight bends. No map.
// Safety: no fresh camera reading for LOST_MS stops the car (it restarts when the camera finds it again);
// the nose off the lane (both line sensors black) for GIVE_UP_MS stops it for good.
// Tune: Base speed (top speed), Camera: steer gain / center pull / bend slowdown.
const LOST_MS = 500;
const GIVE_UP_MS = 2500;
const c = s.cam;
if (mem.gaveUp) return [0, 0];
if (!(s.L && s.R)) mem.inAt = s.t;
else if (mem.inAt == null) mem.inAt = s.t;
if (s.t - mem.inAt > GIVE_UP_MS) {
  mem.gaveUp = true;
  ctx.log(`camera pilot: nose off the lane for ${GIVE_UP_MS} ms, stopped`);
  return [0, 0];
}
if (!c || !c.found || s.t - c.t > LOST_MS) {
  if (mem.mode !== 'lost') { mem.mode = 'lost'; ctx.log('camera pilot: camera lost the robot, stopped'); }
  return [0, 0];
}
if (mem.mode !== 'cam') { mem.mode = 'cam'; ctx.log('camera pilot: steering from the camera'); }
const base = Math.max(p.apBase, p.minSpeed + 5, 30);
const turn = Math.max(-1, Math.min(1, (c.steer / 45) * p.camGain - c.offset * p.camCenter)); // + = right
const sp = Math.max(p.minSpeed + 5, base * (1 - p.camSlow * Math.abs(turn)));
const l = sp * (1 + turn), r = sp * (1 - turn);
return [Math.max(-100, Math.min(100, l)), Math.max(-100, Math.min(100, r))];
