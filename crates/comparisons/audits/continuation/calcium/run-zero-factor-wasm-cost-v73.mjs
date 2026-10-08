import {readFileSync,writeFileSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {costSources,sha,json} from './zero-factor-wasm-cost-sources-v73.mjs';
import {config,flags,groups,key,shuffle,orders,modeOrders,validateRows} from './zero-factor-wasm-cost-protocol-v73.mjs';
import {captured} from './point-qualified-capture.mjs';
const source=costSources(),binaries=json('zero-factor-wasm-binaries-v72.json');for(const a of binaries.artifacts)assert.equal(sha(a.path),a.sha256);
assert.equal(readFileSync('/proc/self/status','utf8').match(/^Cpus_allowed_list:\s*(.+)$/m)?.[1],'0');
const origin={checkpoint:73,recorded:new Date().toISOString(),source,config,binariesSha256:sha('zero-factor-wasm-binaries-v72.json'),
 pairOrders:Object.fromEntries(config.instanceModes.map(m=>[m,orders(m+'-pair-order')])),modeOrders:modeOrders()};
writeFileSync('zero-factor-wasm-cost-origin-v73.json',JSON.stringify(origin,null,2)+'\n',{flag:'wx'});
await captured('zero-factor-wasm-cost-environment-before-v73','.','node',['zero-factor-wasm-cost-environment-v73.mjs']);
const runs=[],pilots={},iterationPlans={};
async function run(phase,round,variant,instanceMode,rows,plan){
 const tag='zero-factor-wasm-cost-'+phase+'-'+round+'-'+instanceMode+'-'+variant+'-v73';
 await captured(tag,'.','taskset',['-c','2','node',...flags,'zero-factor-wasm-cost-worker-v73.mjs',variant,instanceMode,resolve(plan)]);
 const {rows:observations,terminal}=validateRows('results/'+tag+'.stdout',rows,variant,instanceMode),g=json('results/'+tag+'.json');
 assert.equal(terminal.moduleSha256,binaries.artifacts.find(a=>a.variant===variant&&a.kind==='rational').sha256);
 assert.equal(terminal.planSha256,sha(plan));assert.equal(terminal.inputSha256,sha('zero-factor-cost-input-v71.json'));
 for(const r of observations)assert(Date.parse(r.started)>=Date.parse(g.started)&&Date.parse(r.finished)<=Date.parse(g.finished));
 runs.push({phase,round,variant,instanceMode,tag,plan,planSha256:sha(plan),rows:rows.length,started:g.started,finished:g.finished});
 return observations;
}
for(const instanceMode of config.instanceModes)for(const iterations of config.pilotIterations){
 const rows=shuffle(groups(iterations),'pilot-'+instanceMode+'-'+iterations),path='zero-factor-wasm-cost-pilot-'+instanceMode+'-'+iterations+'-plan-v73.json';
 writeFileSync(path,JSON.stringify(rows)+'\n',{flag:'wx'});
 for(const variant of iterations===1?['baseline','candidate']:['candidate','baseline'])
  pilots[instanceMode+':'+variant+':'+iterations]=await run('pilot',iterations,variant,instanceMode,rows,path);
}
for(const mode of config.instanceModes){
 const p=Object.fromEntries(['baseline','candidate'].map(v=>[v,new Map(pilots[mode+':'+v+':32'].map(r=>[key(r),r]))]));
 iterationPlans[mode]=groups().map(g=>{const baseline=p.baseline.get(key(g)).elapsed_ns,candidate=p.candidate.get(key(g)).elapsed_ns;
  return {...g,iterations:Math.max(1,Math.min(config.maxIterations,Math.ceil(config.targetNs/Math.max(baseline/32,candidate/32)))),baselinePilotNs:baseline,candidatePilotNs:candidate};});
}
writeFileSync('zero-factor-wasm-cost-iterations-v73.json',JSON.stringify(iterationPlans,null,2)+'\n',{flag:'wx'});
await captured('zero-factor-wasm-cost-environment-after-pilots-v73','.','node',['zero-factor-wasm-cost-environment-v73.mjs']);
for(let round=0;round<config.pairs;round++)for(const mode of origin.modeOrders[round]){
 const rows=shuffle(iterationPlans[mode].map(({baselinePilotNs,candidatePilotNs,...g})=>g),'cpu-'+mode+'-'+round),path='zero-factor-wasm-cost-cpu-'+mode+'-'+round+'-plan-v73.json';
 writeFileSync(path,JSON.stringify(rows)+'\n',{flag:'wx'});
 for(const variant of origin.pairOrders[mode][round])await run('cpu',round,variant,mode,rows,path);
 console.log(JSON.stringify({checkpoint:73,phase:'pair-complete',round,instanceMode:mode}));
}
await captured('zero-factor-wasm-cost-environment-after-v73','.','node',['zero-factor-wasm-cost-environment-v73.mjs']);
assert.deepEqual(costSources(),source);for(const a of binaries.artifacts)assert.equal(sha(a.path),a.sha256);
writeFileSync('zero-factor-wasm-cost-runs-v73.json',JSON.stringify({checkpoint:73,originSha256:sha('zero-factor-wasm-cost-origin-v73.json'),runs,
 finished:new Date().toISOString(),pilotRows:3648,cpuRows:43776},null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:73,status:'wasm-cost-collection-terminal',runs:runs.length,pilotRows:3648,cpuRows:43776,
 limits:'All complete reports checked; statistical analysis and consumer/size/retention decisions remain open.'}));
