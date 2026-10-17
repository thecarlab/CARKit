const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');
const source = fs.readFileSync(path.join(__dirname, '../static/app.js'), 'utf8');
test('camera JPEG subscription follows visible overview and reconnects', () => {
  const sent = [];
  const overview = {hidden: false};
  const c = {state: {socket: {readyState: 1, send: s => sent.push(JSON.parse(s))}},
    document: {hidden: false}, WebSocket: {OPEN: 1}, $: () => overview, Boolean, JSON};
  vm.createContext(c);
  vm.runInContext(source.slice(source.indexOf('function syncCameraSubscription()'), source.indexOf('function connectRos()')), c);
  c.syncCameraSubscription();
  assert.equal(sent.at(-1).op, 'subscribe');
  c.syncCameraSubscription(); assert.equal(sent.length, 1);
  overview.hidden = true; c.syncCameraSubscription(); assert.equal(sent.at(-1).op, 'unsubscribe');
  overview.hidden = false; c.document.hidden = true; c.syncCameraSubscription(); assert.equal(sent.length, 2);
  c.document.hidden = false; c.syncCameraSubscription(); assert.equal(sent.at(-1).op, 'subscribe');
  c.state.cameraSubscribed = false; c.syncCameraSubscription(); assert.equal(sent.length, 4);
});
