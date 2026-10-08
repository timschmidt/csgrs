import {readFileSync,writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sha,json,retainedSources} from './zero-factor-retained-sources-v75.mjs';
import {retentionEvidence} from './check-zero-factor-retained-v75.mjs';
assert(process.argv.slice(2).length===0||process.argv.slice(2).join(' ')==='--record');
const path='zero-factor-retained-v75-manifest.json',evidence=retentionEvidence();
assert.deepEqual(evidence,json('results/zero-factor-retained-check-v75.stdout'));
const check=json('results/zero-factor-retained-check-v75.json');assert.equal(check.code,0);assert.equal(check.signal,null);
assert.equal(check.command,'node');assert.deepEqual(check.args,['check-zero-factor-retained-v75.mjs']);
assert.equal(readFileSync('results/zero-factor-retained-check-v75.stderr').length,0);
assert(Date.parse(check.started)>=Date.parse(json('results/zero-factor-retained-gates-approved-v75.json').finished));
const successfulGates=[...evidence.gates,'zero-factor-retained-check-v75'];assert.equal(successfulGates.length,12);
const gates=[...evidence.preservedFailures,...successfulGates];assert.equal(gates.length,14);
const origin=json('zero-factor-retained-origin-v75.json');
const files=[...new Set(['zero-factor-retained-v75-findings.md','zero-factor-retained-sources-v75.mjs','zero-factor-retained-protocol-v75.md',
 'run-zero-factor-retained-v75.mjs','run-zero-factor-retained-approved-v75.mjs','check-zero-factor-retained-v75.mjs','verify-zero-factor-retained-v75.mjs',
 'zero-factor-retained-origin-v75.json','zero-factor-retained-run-origin-v75.json','zero-factor-retained-run-origin-approved-v75.json',
 'capture.mjs','point-qualified-capture.mjs','point-qualified-environment.mjs',
 ...Object.keys(origin.historicalFiles),...gates.flatMap(tag=>['json','stdout','stderr'].map(ext=>'results/'+tag+'.'+ext))])].sort();
if(process.argv.includes('--record')){
 const m={checkpoint:75,status:'retained',recorded:new Date().toISOString(),files:Object.fromEntries(files.map(p=>[p,sha(p)])),
  gates,successfulGates,preservedFailures:evidence.preservedFailures,evidence,liveSources:origin.after,previousSources:origin.before,
  changed:evidence.source.changed,retainedContinuationTransfers:7,newDonorLines:0,newDedicatedArtifacts:0,
  acceptedCosts:'Certified completeness and intended-path native/WASM gains justify disclosed bypass/new-answer costs and representative stripped-example growth of 5744 bytes each; no universal speed/memory/size reduction.',
  next:'Continue remaining donor/reference reads and full inventory reconciliation. Rebase any later power-sum trial onto this retained zero-factor baseline. Old dynamic live flags describe historical maps.'};
 writeFileSync(path,JSON.stringify(m,null,2)+'\n',{flag:'wx'});
 console.log(JSON.stringify({checkpoint:75,status:'recorded-retained',artifacts:files.length,gates:gates.length,successful:12,preservedFailed:2,liveFiles:957}));
}else{
 const m=json(path);assert.deepEqual(Object.keys(m.files),files);for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
 assert.equal(m.status,'retained');assert.deepEqual(m.evidence,evidence);assert.deepEqual(m.gates,gates);
 assert.deepEqual(m.successfulGates,successfulGates);assert.deepEqual(m.preservedFailures,evidence.preservedFailures);
 assert.deepEqual(m.liveSources,origin.after);assert.deepEqual(m.previousSources,origin.before);
 assert.deepEqual(m.changed,origin.changed);assert.deepEqual(retainedSources(),m.evidence.source);
 assert.equal(m.retainedContinuationTransfers,7);assert.equal(m.newDonorLines,0);assert.equal(m.newDedicatedArtifacts,0);
 console.log(JSON.stringify({checkpoint:75,status:'retained-verified',artifacts:files.length,gates:gates.length,successful:12,preservedFailed:2,
  changed:m.changed,liveFiles:957,liveTests:evidence.liveTests,consumer:evidence.consumer,graph:evidence.dependencyGraph,
  strippedSizeDeltas:evidence.sizes.map(s=>({example:s.example,bytes:s.deltas[1]})),retainedContinuationTransfers:7,
  newDedicatedArtifacts:0,limits:evidence.limits,next:m.next}));
}
