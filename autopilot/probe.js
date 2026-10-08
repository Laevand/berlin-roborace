// Sensor probe: motors stay off. Press GO in Auto mode, then push the car across the track by hand.
// Every change of the line sensors is written to the Log tab, so you can learn what the
// printed colors (pink, purple, blue, white edge, black mat) read as.

if (s.code !== mem.last) {
  ctx.log(`LINE ${s.code}  left=${s.code & 2 ? 'black' : 'white'}  right=${s.code & 1 ? 'black' : 'white'}`);
  mem.last = s.code;
}
return [0, 0];
