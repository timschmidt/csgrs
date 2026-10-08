import {writeFileSync,readFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {evidence as replayEvidence} from './zero-factor-wasm-cost-evidence-v73.mjs';
import {sourceFiles,costSources,sha,json} from './zero-factor-wasm-cost-sources-v73.mjs';
import {gate} from './check-zero-factor-wasm-cost-v73.mjs';
// Recording binds the just-captured evidence. The ordinary verification path
// independently recomputes all statistics and control outcomes before accepting it.
const recording=process.argv.includes('--record'),captured=json('results/zero-factor-wasm-cost-evidence-v73.stdout');
gate('zero-factor-wasm-cost-evidence-v73');assert.equal(readFileSync('results/zero-factor-wasm-cost-evidence-v73.stderr').length,0);
assert.deepEqual(costSources(),json('zero-factor-wasm-cost-origin-v73.json').source);
const evidence=recording?captured:replayEvidence();assert.deepEqual(evidence,captured);
const gates=[...evidence.gates,'zero-factor-wasm-cost-evidence-v73'];assert.equal(gates.length,111);
const runs=json('zero-factor-wasm-cost-runs-v73.json').runs,controls=json('zero-factor-wasm-cost-controls-v73.json');
const files=[...new Set([...sourceFiles,'check-zero-factor-wasm-cost-v73.mjs','zero-factor-wasm-cost-evidence-v73.mjs',
 'verify-zero-factor-wasm-cost-v73.mjs','zero-factor-wasm-cost-v73-findings.md','zero-factor-wasm-cost-origin-v73.json',
 'zero-factor-wasm-cost-runs-v73.json','zero-factor-wasm-cost-iterations-v73.json','zero-factor-wasm-cost-summary-v73.json',
 'check-zero-factor-wasm-cost-controls-v73.mjs','zero-factor-wasm-cost-controls-v73.json',
 'zero-factor-wasm-v72-manifest.json','zero-factor-wasm-binaries-v72.json','zero-factor-wasm-sources-v72.mjs',
 'zero-factor-wasm-runner-v72.mjs','zero-factor-cost-input-v71.json','zero-factor-cost-input-v71.mjs','zero-factor-cost-protocol-v71.mjs',
 ...['json','stdout','stderr'].map(ext=>'results/zero-factor-wasm-verify-v72.'+ext),
 ...runs.map(r=>r.plan),...controls.streamResults.flatMap(r=>[r.path,r.plan]),
 ...gates.flatMap(t=>['json','stdout','stderr'].map(ext=>'results/'+t+'.'+ext))])].sort();
const path='zero-factor-wasm-cost-v73-manifest.json';
if(recording){
 const m={checkpoint:73,status:evidence.status,recorded:new Date().toISOString(),files:Object.fromEntries(files.map(p=>[p,sha(p)])),
  gates,evidence,retainedContinuationTransfers:6,newDonorLines:0,
  scope:'Matched WASM cost qualification of the unchanged isolated candidate. Representative consumer/size and full reference inventory remain open.'};
 writeFileSync(path,JSON.stringify(m,null,2)+'\n',{flag:'wx'});
}else{
 const m=json(path);assert.deepEqual(Object.keys(m.files),files);for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
 assert.deepEqual(m.evidence,evidence);assert.deepEqual(m.gates,gates);assert.equal(m.retainedContinuationTransfers,6);assert.equal(m.newDonorLines,0);
}
console.log(JSON.stringify({checkpoint:73,status:recording?'evidence-bound-awaiting-full-replay':evidence.status,artifacts:files.length,gates:gates.length,
 successfulCaptures:111,preservedFailedCaptures:0,liveFiles:956,candidateFiles:176,pilotRows:3648,cpuRows:43776,workerProcesses:104,
 groups:912,pairsPerInstanceMode:24,recordControls:evidence.controls.record,streamControls:evidence.controls.stream,
 storage:evidence.storage,retainedContinuationTransfers:6,
 next:'Representative consumer and stripped-size gates before deciding retention; full donor/reference inventory remains open.'}));
