import {readFileSync,writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sha,json,retainedSources} from './point-retained-sources-v67.mjs';
import {retentionEvidence} from './check-point-retained-v67.mjs';
const path='point-retained-v67-manifest.json';
assert(process.argv.includes('--record')||process.argv.slice(2).join(' ')==='--point-live','Use --point-live for this retained state.');
const evidence=retentionEvidence();assert.deepEqual(evidence,json('results/point-retained-check-v67.stdout'));
const check=json('results/point-retained-check-v67.json');assert.equal(check.code,0);assert.equal(check.signal,null);
assert.equal(readFileSync('results/point-retained-check-v67.stderr').length,0);
const gates=[...evidence.gates,'point-retained-check-v67'];assert.equal(gates.length,10);
if(process.argv.includes('--record')){
 const origin=json('point-retained-origin-v67.json');
 const files=[...new Set(['point-retained-v67-findings.md','point-retained-sources-v67.mjs','run-point-retained-v67.mjs',
  'check-point-retained-v67.mjs','verify-point-retained-v67.mjs','point-retained-origin-v67.json',
  ...Object.keys(origin.historicalFiles),...gates.flatMap(tag=>['json','stdout','stderr'].map(ext=>'results/'+tag+'.'+ext))])].sort();
 const m={checkpoint:67,status:'retained',recorded:new Date().toISOString(),files:Object.fromEntries(files.map(p=>[p,sha(p)])),gates,evidence,
  liveSources:origin.after,previousSources:origin.before,changed:evidence.source.changed,retainedContinuationTransfers:6,
  newDonorLines:0,newDedicatedArtifacts:0,
  acceptedCosts:'Completeness-first: preserve exact proof replay while recovering lost point witnesses. Same-result allocation/peak penalties of the eager candidate are avoided, but recovered-answer retries add work/allocations and some timings remain slower. Representative stripped examples +1568/+1536 bytes, including layout effects; no universal speed/memory/size improvement claimed.',
  next:'Continue remaining donor/reference reads, separate power-sum work and complete inventory reconciliation. Current-source checks must use the post-retention map, not earlier historical live flags.'};
 writeFileSync(path,JSON.stringify(m,null,2)+'\n',{flag:'wx'});
 console.log(JSON.stringify({checkpoint:67,status:'recorded-retained',artifacts:files.length,gates:gates.length,changed:m.changed,liveFiles:956}));
}else{
 const m=json(path);for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
 assert.equal(m.status,'retained');assert.deepEqual(m.evidence,evidence);assert.deepEqual(m.gates,gates);
 const o=json('point-retained-origin-v67.json');assert.deepEqual(m.liveSources,o.after);assert.deepEqual(m.previousSources,o.before);
 assert.deepEqual(m.changed,['hypersolve/src/algebraic_binary.rs']);assert.deepEqual(retainedSources(),m.evidence.source);
 console.log(JSON.stringify({checkpoint:67,status:'retained-verified',artifacts:Object.keys(m.files).length,gates:gates.length,
  changed:m.changed,liveFiles:956,liveTests:evidence.liveTests,consumer:evidence.consumer,graph:evidence.dependencyGraph,
  sizes:evidence.sizes,retainedContinuationTransfers:6,newDedicatedArtifacts:0,limits:evidence.limits,next:m.next}));
}
