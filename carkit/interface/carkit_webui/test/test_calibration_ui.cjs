const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const path = require('node:path');

test('on-vehicle calibration survives blur, hidden tab, closed page and browser API failure', async () => {
  const elements = new Map(), listeners = {}, intervals = [], timeouts = [], calls = [];
  const element = id => {
    if (!elements.has(id)) elements.set(id, {
      open: false, checked: id !== 'calibration-new', value: '1.0', children: [],
      classList: {toggle(){},remove(){},add(){}}, setAttribute(){}, replaceChildren(){},
      addEventListener(name, fn){listeners[`dialog:${name}`] = fn;},
      showModal(){this.open=true;},close(){this.open=false;},
    });
    return elements.get(id);
  };
  let failApi = false;
  const snapshot = {chassis:'vesc_rplidar', supported:true, running:false, status:'requires_calibration', reason:'Running', next_stage:{id:'forward',label:'Forward only'},progress:{phase:'returning',stage_index:0}};
  const context = {
    document: {hidden:false,hasFocus:()=>true,getElementById:element,addEventListener:(k,v)=>listeners[`document:${k}`]=v},
    window: {addEventListener:(k,v)=>listeners[`window:${k}`]=v},
    state: {signedIn:true,runtimeRunning:true,bridgeAuthenticated:true,operator:true,controlMode:'HUMAN_CONTROL'},
    keyboard:{stop(){}},publishRos:(...args)=>calls.push(['ROS',...args]),refresh:async()=>{},
    api:async (url,body)=>{calls.push([url,body]);if(failApi)throw new Error('offline');if(url==='/api/calibration/start'){snapshot.running=true;return {run_id:'test',owner_token:'test'};}return {...snapshot};},
    setTimeout:fn=>{timeouts.push(fn);return timeouts.length;},clearTimeout(){},
    setInterval:fn=>intervals.push(fn),
  };
  vm.runInNewContext(fs.readFileSync(path.join(__dirname,'../static/calibration.js'),'utf8'),context);
  await new Promise(setImmediate);
  element('open-calibration').onclick();
  await new Promise(setImmediate);
  await element('calibration-start').onclick();
  assert.equal(calls.filter(c=>c[0]==='/api/calibration/start').length,1);
  element('calibration-close').onclick();
  listeners['dialog:cancel']?.();
  context.document.hidden=true;
  context.document.hasFocus=()=>false;
  for(const name of ['window:blur','document:visibilitychange','window:pagehide']) listeners[name]?.();
  for(const callback of intervals) await callback();
  failApi=true;
  await timeouts.at(-1)();
  assert.equal(calls.filter(c=>c[0]==='/api/calibration/cancel').length,0);
  assert.equal(calls.filter(c=>c[0]==='/api/calibration/heartbeat').length,0);
  assert.equal(calls.filter(c=>c[0]==='ROS').length,0);
  failApi=false;
  await element('calibration-stop').onclick();
  assert.equal(calls.filter(c=>c[0]==='/api/calibration/cancel').length,1);
  assert.equal(calls.find(c=>c[0]==='/api/calibration/cancel')[1].reason,'Stop calibration button pressed');
  snapshot.running=false;
  snapshot.progress={phase:'awaiting_confirmation',stage_index:4};
  snapshot.next_stage={id:'right_half',label:'Half maximum right turn'};
  snapshot.same_configuration=true;
  snapshot.completed_stages=['forward','reverse','left_full','left_half','right_full'];
  await new Promise(setImmediate);
  await timeouts.at(-1)();
  assert.equal(element('calibration-start').textContent,'Run 1 check: Half maximum right turn');
  assert.equal(element('calibration-clear').checked,false);
  assert.equal(element('calibration-start').disabled,true);
  await element('calibration-start').onclick();
  await timeouts.at(-1)();
  assert.equal(calls.filter(c=>c[0]==='/api/calibration/start').length,1);
  element('calibration-clear').checked=true;
  element('calibration-clear').onchange();
  await element('calibration-start').onclick();
  assert.equal(calls.filter(c=>c[0]==='/api/calibration/start').at(-1)[1].restart_sequence,false);
  assert.equal(calls.filter(c=>c[0]==='/api/calibration/start').at(-1)[1].stage_id,'right_half');
  snapshot.running=false;
  snapshot.progress={phase:'complete'};
  snapshot.next_stage=null;
  snapshot.completed_stages.push('right_half');
  await timeouts.at(-1)();
  assert.equal(element('calibration-start').textContent,'Sequence complete');
  assert.equal(element('calibration-start').disabled,true);
  element('calibration-new').checked=true;
  element('calibration-new').onchange();
  assert.equal(element('calibration-clear').checked,false);
  element('calibration-clear').checked=true;
  element('calibration-clear').onchange();
  assert.equal(element('calibration-start').textContent,'Run 1 check: Forward only');
  assert.equal(element('calibration-start').disabled,false);
  assert.equal(element('calibration-all').textContent,'Run all 6 checks');
  assert.equal(element('calibration-all').disabled,false);
  await element('calibration-all').onclick();
  assert.equal(calls.filter(c=>c[0]==='/api/calibration/start').at(-1)[1].mode,'all');
  assert.equal(calls.filter(c=>c[0]==='/api/calibration/start')[0][1].mode,'single');
});
