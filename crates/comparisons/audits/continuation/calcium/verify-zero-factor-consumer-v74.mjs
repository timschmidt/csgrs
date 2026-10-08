import {readFileSync,writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {consumerEvidence,gate} from './check-zero-factor-consumer-v74.mjs';
import {sha,json} from './zero-factor-consumer-sources-v74.mjs';
const evidence=consumerEvidence();assert.deepEqual(evidence,json('results/zero-factor-consumer-check-v74.stdout'));
gate('zero-factor-consumer-check-v74','node',['check-zero-factor-consumer-v74.mjs']);
assert.equal(readFileSync('results/zero-factor-consumer-check-v74.stderr').length,0);
const gates=[...evidence.gates,'zero-factor-consumer-check-v74'];assert.equal(gates.length,19);
const files=[...new Set(['zero-factor-consumer-protocol-v74.md','zero-factor-consumer-sources-v74.mjs','zero-factor-consumer-origin-v74.json',
 'zero-factor-consumer-binding-v74.json','qualify-zero-factor-consumer-v74.mjs','zero-factor-consumer-run-origin-v74.json',
 'zero-factor-consumer-app-origin-v74.json','zero-factor-consumer-apps-v74.json','zero-factor-consumer-caller-review-v74.md',
 'check-zero-factor-consumer-v74.mjs','verify-zero-factor-consumer-v74.mjs','zero-factor-consumer-v74-findings.md','point-qualified-environment.mjs',
 'point-consumer-binding-v66.json','point-consumer-apps-v66.json','results/point-consumer-environment-v66.stdout','zero-factor-wasm-cost-v73-manifest.json','zero-factor-wasm-cost-sources-v73.mjs',
 'zero-factor-final-binding-v70.json','zero-factor-final-sources-v70.mjs','point-retained-sources-v67.mjs',
 ...['json','stdout','stderr'].map(ext=>'results/zero-factor-wasm-cost-verify-v73.'+ext),
 ...['json','stdout','stderr'].map(ext=>'results/point-consumer-release-v66.'+ext),
 ...gates.flatMap(t=>['json','stdout','stderr'].map(ext=>'results/'+t+'.'+ext))])].sort();
const path='zero-factor-consumer-v74-manifest.json';
if(process.argv.includes('--record')){
 const m={checkpoint:74,status:evidence.status,recorded:new Date().toISOString(),files:Object.fromEntries(files.map(p=>[p,sha(p)])),
  gates,evidence,retainedContinuationTransfers:6,newDonorLines:0,
  scope:'Isolated candidate consumer/size qualification. No live adoption; full remaining reference inventory and power sums stay open.'};
 writeFileSync(path,JSON.stringify(m,null,2)+'\n',{flag:'wx'});
}else{
 const m=json(path);assert.deepEqual(Object.keys(m.files),files);for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
 assert.deepEqual(m.evidence,evidence);assert.deepEqual(m.gates,gates);assert.equal(m.retainedContinuationTransfers,6);assert.equal(m.newDonorLines,0);
}
console.log(JSON.stringify({checkpoint:74,status:evidence.status,artifacts:files.length,gates:gates.length,successfulCaptures:19,preservedFailedCaptures:0,
 liveFiles:956,solverCandidateFiles:176,consumerFiles:355,copiedBytes:evidence.copiedBytes,tests:evidence.tests,graph:evidence.graph,
 representativeSizes:evidence.sizes.map(s=>({example:s.example,ordinaryDelta:s.deltas[0],strippedDelta:s.deltas[1]})),
 dedicatedFiles:4,dedicatedBytes:evidence.dedicatedBytes,retainedContinuationTransfers:6,
 next:'Review the retention decision, then exact live integration/regressions. Full donor/reference audit remains incomplete.'}));
