const {test} = require('node:test');
const assert = require('node:assert/strict');
const Keyboard = require('../static/keyboard.js');
function fixture() {
  const f = {messages: [], toggles: 0, stops: 0, mode: 'HUMAN_CONTROL', enabled: true, time: 0};
  f.keyboard = new Keyboard({
    now: () => f.time,
    publish: (speed, steering) => { f.messages.push([speed, steering]); return true; },
    toggle: () => f.toggles++, emergencyStop: () => f.stops++,
    mode: () => f.mode, enabled: () => f.enabled,
  });
  f.advance = (seconds) => { for (let i=0;i<seconds*20;i++) {f.time += 50; f.keyboard.tick();} };
  f.down = (code, extra = {}) => f.keyboard.keydown({code, preventDefault() {}, ...extra});
  f.up = code => f.keyboard.keyup({code, preventDefault() {}});
  return f;
}
test('speed and steering ramp, cap, and release each axis independently', () => {
  const f = fixture(); f.keyboard.tick(); assert.equal(f.messages.length, 0);
  f.down('KeyW'); f.advance(1); assert.ok(Math.abs(f.messages.at(-1)[0]-1)<1e-9);
  f.down('KeyA'); f.advance(.2); assert.ok(Math.abs(f.messages.at(-1)[1]-.1)<1e-9);
  f.up('KeyA'); assert.equal(f.messages.at(-1)[1],0); assert.ok(f.messages.at(-1)[0]>1);
  f.advance(5); assert.equal(f.messages.at(-1)[0],4);
  f.down('KeyD'); f.advance(1); assert.equal(f.messages.at(-1)[1],-.25);
  f.up('KeyW'); assert.equal(f.messages.at(-1)[0],0);
  f.up('KeyD'); assert.deepEqual(f.messages.at(-1),[0,0]);
  f.down('KeyS'); f.advance(5); assert.equal(f.messages.at(-1)[0],-4);
});
test('Caps Lock toggles once per physical press and stops held manual motion', () => {
  const f = fixture(); f.down('KeyW'); f.down('CapsLock');
  assert.equal(f.toggles, 1); assert.deepEqual(f.messages.at(-1), [0, 0]);
  f.down('CapsLock', {repeat:true}); assert.equal(f.toggles, 1);
  f.mode = 'AUTO_DRIVE'; f.down('KeyW'); f.keyboard.tick();
  assert.equal(f.keyboard.keys.size, 0);
  f.down('CapsLock'); assert.equal(f.toggles, 2);
  f.mode = 'EMERGENCY_STOP'; f.down('CapsLock'); assert.equal(f.toggles, 2);
});
test('focus/connection loss requires a new key press after recovery', () => {
  const f = fixture(); f.down('KeyW'); f.enabled = false; f.keyboard.tick();
  assert.deepEqual(f.messages.at(-1), [0, 0]);
  f.enabled = true; const count = f.messages.length;
  f.down('KeyW', {repeat:true}); f.keyboard.tick(); assert.equal(f.messages.length, count);
  f.down('KeyW'); assert.deepEqual(f.messages.at(-1), [0, 0]);
  f.advance(.1); assert.ok(f.messages.at(-1)[0] > 0 && f.messages.at(-1)[0] <= .1);
});
test('typing and modifier shortcuts do not control a vehicle', () => {
  const f = fixture();
  for (const extra of [{target:{closest:()=>true}}, {ctrlKey:true}, {altKey:true}, {metaKey:true}]) {
    f.down('KeyW', extra); f.down('CapsLock', extra); f.down('Space', extra);
  }
  assert.equal(f.messages.length, 0); assert.equal(f.toggles, 0); assert.equal(f.stops, 0);
});
test('Space stops and requests emergency stop, including autonomous mode', () => {
  const f = fixture(); f.down('KeyW'); f.down('Space');
  assert.deepEqual(f.messages.at(-1), [0, 0]); assert.equal(f.stops, 1);
  f.mode = 'AUTO_DRIVE'; f.down('Space'); assert.equal(f.stops, 2);
});
