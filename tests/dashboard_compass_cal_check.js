const assert=require('assert'); const fs=require('fs');
const els={}; const el=id=>els[id]||(els[id]={id,textContent:'',innerHTML:'',disabled:false,style:{},listeners:{},
  addEventListener(t,f){this.listeners[t]=f}, scrollIntoView(){scrolls.push(id)}, getContext:()=>({clearRect(){},beginPath(){},arc(){},stroke(){}}), width:128});
let scrolls=[]; const sent=[];
global.$=el; global.WebSocket={OPEN:1}; global.ws={readyState:1,send:b=>sent.push(b)};
global.BoatMessage={create:x=>x,encode:x=>({finish:()=>x})}; global.motorArmed=false;
global.setInterval=()=>0;
eval(fs.readFileSync(process.argv[2],'utf8').replace(/^let /mg,'var '));
// idle, calibrated
updateCompassCal({state:0,calibrated:true,fieldRatio:1.0,headingDeg:10,runId:0});
assert.strictEqual(el('ccal-start').disabled,false); assert.strictEqual(el('ccal-cancel').style.display,'none');
// press -> waiting until the boat answers with a new run id
el('ccal-start').listeners.click();
assert.deepStrictEqual(sent.pop(),{compassCal:{start:true,cancel:false,toleranceDeg:25}});
assert.strictEqual(el('ccal-start').disabled,true); assert.match(el('ccal-hint').textContent,/asking/);
updateCompassCal({state:0,calibrated:true,runId:0});           // old idle status: still waiting
assert.strictEqual(el('ccal-start').disabled,true);
updateCompassCal({state:1,stillProgress:0.3,runId:1});          // answered: running
assert.strictEqual(el('ccal-start').style.display,'none'); assert.strictEqual(el('ccal-cancel').disabled,false);
assert.deepStrictEqual(scrolls,['ccal']);
updateCompassCal({state:2,coverageMask:0xFFFFFFFF,turnDeg:600,runId:1});
assert.match(el('ccal-detail').innerHTML,/✓ directions 32\/32/); assert.match(el('ccal-detail').innerHTML,/✓ turns 1.7\/1.5/);
assert.doesNotMatch(el('ccal-detail').innerHTML,/keep it level/);
updateCompassCal({state:2,coverageMask:0x0000FFFF,turnDeg:200,runId:1,tilted:true,tiltSkipped:12});
assert.match(el('ccal-detail').innerHTML,/keep it level/);
updateCompassCal({state:2,coverageMask:0x0000FFFF,turnDeg:220,runId:1,tilted:false,tiltSkipped:12});
assert.match(el('ccal-detail').innerHTML,/12 tilted readings skipped/);
el('ccal-cancel').listeners.click(); assert.deepStrictEqual(sent.pop(),{compassCal:{start:false,cancel:true}});
updateCompassCal({state:4,axisRatio:1.1,fitRms:0.01,gyroScale:1,gyroDevDeg:1,radius:4000,calibrated:true,runId:1,lastState:4});
assert.match(el('ccal-step').textContent,/PASS/); assert.strictEqual(el('ccal-start').disabled,false);
assert.deepStrictEqual(scrolls,['ccal','ccal']);                // one scroll per active step, none for PASS
updateCompassCal({state:0,calibrated:true,fieldRatio:0.99,headingDeg:45,runId:1,lastState:4});
assert.match(el('ccal-detail').textContent,/last calibration: PASS/);
// armed: button disabled + boat refusal wording
motorArmed=true; updateCcalButtons(); assert.strictEqual(el('ccal-start').disabled,true); assert.match(el('ccal-hint').textContent,/DISARM/);
updateCompassCal({state:5,reason:8,runId:2,lastState:4,radius:4000});
assert.match(el('ccal-step').textContent,/DISARM first/); assert.doesNotMatch(el('ccal-detail').textContent,/squash/);
// first boot, only step 1 saved
motorArmed=false;
updateCompassCal({state:5,reason:7,gyroLevelSaved:true,runId:3,lastState:5,lastReason:7});
assert.match(el('ccal-detail').textContent,/Gyro drift \+ level were saved/);
updateCompassCal({state:0,calibrated:false,runId:3,lastState:5,lastReason:7,headingDeg:-1});
assert.match(el('ccal-detail').textContent,/last calibration: FAIL \(spin never started/);
// tolerance: Strict chosen -> sent; a gyro FAIL on Strict suggests Relaxed
el('ccal-tol').value='5';
updateCompassCal({state:0,calibrated:true,runId:3,savedDevDeg:4.2,savedToleranceDeg:5});
assert.match(el('ccal-badge').textContent,/calibrated · strict · gyro 4°/);
el('ccal-start').listeners.click();
assert.deepStrictEqual(sent.pop(),{compassCal:{start:true,cancel:false,toleranceDeg:5}});
updateCompassCal({state:5,reason:17,runId:4,toleranceDeg:5,gyroDevDeg:18,radius:3600});
assert.match(el('ccal-detail').textContent,/try Relaxed/);
updateCompassCal({state:4,runId:5,toleranceDeg:25,calibrated:true,savedDevDeg:18,savedToleranceDeg:25});
assert.match(el('ccal-step').textContent,/PASS \(relaxed\)/);
assert.match(el('ccal-badge').textContent,/relaxed · gyro 18°/);
console.log('dashboard logic: all checks passed');
