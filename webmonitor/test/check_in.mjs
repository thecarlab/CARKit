import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';

const source = await readFile(new URL('../index.js', import.meta.url), 'utf8');
const {default: worker} = await import(`data:text/javascript;base64,${Buffer.from(source).toString('base64')}`);
let writes = 0;
const DB = {
  exec: async () => {},
  prepare: () => ({run: async () => {}, bind: () => ({run: async () => { writes++; }})}),
};
const env = {DB, CARKIT_REPORTER_TOKEN: 'existing-fleet-token', CARKIT_REPORTER_TOKEN_ADA2: 'ada2-only-token'};
async function check(vehicle, token, expected) {
  const result = await worker.fetch(new Request('https://monitor.example/api/check-in', {
    method: 'POST', headers: {'Authorization': `Bearer ${token}`, 'Content-Type': 'application/json'},
    body: JSON.stringify({vehicle_id: vehicle, ip_address: '128.175.1.2', webui_port: 8080}),
  }), env);
  assert.equal(result.status, expected, `${vehicle}, ${token}`);
}
await check('ADA1', 'existing-fleet-token', 200);
await check('ADA2', 'existing-fleet-token', 200);
await check('ADA2', 'ada2-only-token', 200);
await check('ADA1', 'ada2-only-token', 401);
await check('ADA2', 'wrong-token', 401);
await check('ADA2', 'undefined', 401);
assert.equal(writes, 3);
const unauthenticated = await worker.fetch(new Request('https://monitor.example/api/check-in', {
  method: 'POST', body: 'not JSON',
}), env);
assert.equal(unauthenticated.status, 401, 'Reject unauthenticated requests before parsing the body');
console.log('Shared fleet and vehicle-scoped credential checks passed.');
