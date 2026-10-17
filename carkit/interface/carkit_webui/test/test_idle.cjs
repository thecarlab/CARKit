const test = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const source = fs.readFileSync(require('node:path').join(__dirname, '../static/app.js'), 'utf8');
function fixture(status, hidden = false) {
  const timers = [];
  const element = {value: '', classList: {toggle() {}}, textContent: ''};
  const context = {
    state: {signedIn: true, runtimeRunning: false, jobRunning: false, logCursor: 0},
    document: {hidden, querySelector: () => element},
    $: () => element, api: async () => status,
    setTimeout: (fn, delay) => {timers.push(delay); return timers.length;}, clearTimeout() {},
    Date, Boolean, keyboard: {stop() {}},
  };
  for (const name of ['setActivity','applyLaunchConfig','updateActiveMapFile','updateSetupLabels',
    'renderSystemTelemetry','renderBattery','renderChassisStatus','renderLogs','connectRos','openDrawer']) context[name] = () => {};
  vm.createContext(context);
  vm.runInContext(source.slice(source.indexOf('function scheduleRefresh('), source.indexOf('\n$("profile").addEventListener')), context);
  return {context, timers};
}
test('idle, active and background status polling retain realtime cadence', async () => {
  for (const [status, hidden, delay] of [
    [{running:false, installed:true},false,250],
    [{running:true, installed:true},false,250],
    [{running:false, job_running:true, installed:true},false,250],
    [{running:false, installed:true},true,250],
  ]) {
    const {context,timers} = fixture(status, hidden);
    await context.refresh(); assert.equal(timers.at(-1),delay);
  }
});
test('concurrent refresh calls share one in-flight request', async () => {
  const {context,timers} = fixture({running:false, installed:true});
  await Promise.all([context.refresh(),context.refresh()]);
  assert.equal(timers.length,1);
});
test('stopped runtime does not attempt a ROS bridge connection', () => {
  const context = {state:{signedIn:true,runtimeRunning:false}, WebSocket:class {constructor(){throw Error('unexpected connection');}}};
  vm.createContext(context);
  vm.runInContext(source.slice(source.indexOf('function connectRos()'), source.indexOf('  const socket = new WebSocket',source.indexOf('function connectRos()'))) + '\n}', context);
  context.connectRos();
});

test('map updates coalesce into the next display frame without a delay timer', () => {
  const frames=[]; let draws=0;
  const context={state:{drawPending:false},performance:{now:()=>10},draw:()=>draws++,requestAnimationFrame:fn=>frames.push(fn)};
  vm.createContext(context);
  vm.runInContext(source.slice(source.indexOf('function scheduleDraw()'),source.indexOf('function startViewDrag(')),context);
  context.scheduleDraw(); context.scheduleDraw(); assert.equal(frames.length,1);
  frames.shift()(); assert.equal(draws,1); context.scheduleDraw(); assert.equal(frames.length,1);
});
test('grid covers the viewport after pan, rotation and zoom for local and map views', () => {
  const context = {Math}; vm.createContext(context);
  vm.runInContext(source.slice(source.indexOf('function visibleGridBounds('), source.indexOf('function drawMetricGrid(')), context);
  for (const origin of [[400,300],[125,-70]]) for (const rotation of [0,.7,Math.PI/2]) for (const zoom of [.3,1,4]) {
    const view={panX:650,panY:-420,rotation,zoom};
    const b=context.visibleGridBounds(800,600,view,(x,y)=>[origin[0]+45*x,origin[1]-45*y],45);
    for (const [x,y] of [[0,0],[800,0],[800,600],[0,600]]) {
      const dx=x-400-view.panX,dy=y-300-view.panY;
      const wx=((dx*Math.cos(rotation)+dy*Math.sin(rotation))/zoom+400-origin[0])/45;
      const wy=(origin[1]-((-dx*Math.sin(rotation)+dy*Math.cos(rotation))/zoom+300))/45;
      assert.ok(wx>=b.minX-1e-9 && wx<=b.maxX+1e-9 && wy>=b.minY-1e-9 && wy<=b.maxY+1e-9);
    }
  }
});
test('pose tool requests ownership before arming and reports competing operators', () => {
  const elements={}; const sent=[];
  const c={state:{operator:false,bridgeAuthenticated:true,operatorAvailable:true,socket:{send:m=>sent.push(JSON.parse(m))}}, $:id=>elements[id]??=( {classList:{toggle(){}},textContent:''} )};
  vm.createContext(c); vm.runInContext(source.slice(source.indexOf('function armMapTool('),source.indexOf('function canvasPointToWorld(')),c);
  c.armMapTool('send-goal'); assert.equal(sent[0].op,'claim_control'); assert.equal(c.state.mapTool,undefined);
  c.state.operator=true;c.armMapTool('set-initial');assert.equal(c.state.mapTool,'set-initial');
  c.state.operator=false;c.state.operatorAvailable=false;c.armMapTool('send-goal');assert.match(elements['map-tool-help'].textContent,/Another browser/); assert.equal(sent.length,1);
});
test('laser mounting rotation maps rear sensor X to vehicle forward', () => {
  const c={Math};vm.createContext(c);vm.runInContext(source.slice(source.indexOf('function projectLaserPoint('),source.indexOf('function draw() {')),c);
  const p=c.projectLaserPoint(2,Math.PI,[.1,.2,.15,Math.PI,0,0]);
  assert.ok(Math.abs(p[0]-2.1)<1e-9);assert.ok(Math.abs(p[1]-.2)<1e-9);
  const q=c.projectLaserPoint(1,Math.PI/2,[0,0,0,0,0,Math.PI]);assert.ok(Math.abs(q[1]+1)<1e-9);
});
