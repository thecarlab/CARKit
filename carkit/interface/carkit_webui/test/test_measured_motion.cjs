const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const source = fs.readFileSync(path.join(__dirname, '../static/app.js'), 'utf8');

function fixture(chassis = 'vesc_rplidar') {
  const elements = {};
  const c = {state: {activeChassis: chassis, topicStamps: {}, health: {}}, Math, Number, Boolean,
    Date: {now: () => 10000}, $: id => elements[id] ??= {textContent: '', className: ''},
    markHealth() {}, rememberOdom() {}, scheduleDraw() {}};
  vm.createContext(c);
  vm.runInContext(source.slice(source.indexOf('function chassisFamily('), source.indexOf('function selectHardwareConfiguration(')), c);
  vm.runInContext(source.slice(source.indexOf('function stampSeconds('), source.indexOf('function rememberOdom(')), c);
  vm.runInContext(source.slice(source.indexOf('function handleRos('), source.indexOf('function ', source.indexOf('function handleRos(') + 9)), c);
  return {c, elements};
}

test('commands cannot animate the vehicle or change measured metrics', () => {
  const {c, elements} = fixture();
  c.handleRos('/ackermann_cmd', {drive: {speed: 3, steering_angle: .4}});
  assert.equal(c.state.lastMotion, undefined);
  assert.equal(c.state.odom, undefined);
  assert.deepEqual(elements, {});
});

test('measured speed and yaw rate come from odometry, including zero and reverse', () => {
  for (const speed of [0, -.25, .4]) {
    const {c, elements} = fixture();
    const message = {twist: {twist: {linear: {x: speed}, angular: {z: .2}}}};
    c.handleRos('/odom', message);
    assert.equal(elements.speed.textContent, `${speed.toFixed(2)} m/s`);
    assert.equal(elements['yaw-rate'].textContent, '11.5°/s');
    assert.equal(c.state.odom, message);
    assert.match(elements['motion-source'].textContent, /RealSense/);
    c.handleRos('/ackermann_cmd', {drive: {speed: 3, steering_angle: -.4}});
    assert.equal(elements.speed.textContent, `${speed.toFixed(2)} m/s`);
    c.state.map = {};
    c.expireMotion(10501);
    assert.equal(elements.speed.textContent, '—');
    assert.equal(elements['yaw-rate'].textContent, '—');
    assert.match(elements['map-status'].textContent, /stale/);
  }
});

test('OSRacer does not claim a RealSense source', () => {
  const {c, elements} = fixture('osracer');
  c.renderMeasuredMotion({twist: {twist: {linear: {x: 0}, angular: {z: 0}}}});
  assert.equal(elements['motion-source'].textContent, 'OSRacer odometry');
});

test('invalid motion is not displayed as a fresh measurement', () => {
  const {c, elements} = fixture();
  assert.equal(c.renderMeasuredMotion({twist: {twist: {linear: {x: NaN}, angular: {z: 0}}}}), false);
  assert.deepEqual(elements, {});
});
