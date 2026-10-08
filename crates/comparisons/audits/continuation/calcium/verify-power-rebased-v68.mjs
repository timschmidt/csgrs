import {readFileSync,writeFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {rebasedEvidence,gateTags} from './check-power-rebased-v68.mjs';
import {sha,json,fixedSources,binding} from './power-rebased-fixed-sources-v68.mjs';
const path='power-rebased-v68-manifest.json',evidence=rebasedEvidence();
assert.deepEqual(evidence,json('results/power-rebased-check-v68.stdout'));
const check=json('results/power-rebased-check-v68.json');assert.equal(check.code,0);assert.equal(check.signal,null);
assert.equal(readFileSync('results/power-rebased-check-v68.stderr').length,0);
const gates=[...gateTags,'power-rebased-check-v68'];assert.equal(gates.length,43);
if(process.argv.includes('--record')){
 const scripts=['power-rebased-v68-findings.md','prepare-power-rebased-v68.mjs','power-rebased-origin-v68.json',
  'power-rebased-sources-v68.mjs','power-rebased-binding-v68.json','run-power-rebased-v68.mjs',
  'power-rebased-build-origin-v68.json','power-rebased-binaries-v68.json','power-rebased-initial-v68.rs',
  'power-rebased-fixed-sources-v68.mjs',binding,'run-power-rebased-fixed-v68.mjs','power-rebased-fixed-binaries-v68.json',
  'qualify-power-rebased-extra-v68.mjs','check-power-rebased-v68.mjs','verify-power-rebased-v68.mjs',
  'check-power-sums-kernel.mjs','check-power-sums-public.mjs','power-sums-polynomial-oracle.mjs',
  'check-point-extended.mjs','point-extended-field.mjs','point-qualified-capture.mjs','point-qualified-environment.mjs','capture.mjs',
  'point-retained-v67-manifest.json','power-sums-manifest.json',...Object.keys(fixedSources().harness),
  ...['baseline','candidate'].map(v=>'power-rebased-'+v+'-app-v68/Cargo.lock'),
  ...['json','stdout','stderr'].map(ext=>'results/point-retained-verify-v67.'+ext)];
 const files=[...new Set([...scripts,...gates.flatMap(t=>['json','stdout','stderr'].map(ext=>'results/'+t+'.'+ext))])].sort();
 const m={checkpoint:68,status:'isolated-numerical-qualified-not-retained',recorded:new Date().toISOString(),
  files:Object.fromEntries(files.map(p=>[p,sha(p)])),gates,evidence,source:fixedSources(),retainedContinuationTransfers:6,
  newDonorLines:0,coverage:{completeFiles:1415,partialFiles:20,uniqueReadLines:183653},
  next:'Broader coefficient/carrier qualification, matched native/WASM CPU and isolated allocation benchmarks, consumer/size gates and documentation before a retention decision; remaining donor/reference reads and full inventory reconciliation.'};
 writeFileSync(path,JSON.stringify(m,null,2)+'\n',{flag:'wx'});
 console.log(JSON.stringify({checkpoint:68,status:m.status,artifacts:files.length,gates:gates.length,retainedContinuationTransfers:6}));
}else{
 const m=json(path);for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
 assert.deepEqual(m.gates,gates);assert.deepEqual(m.evidence,evidence);assert.deepEqual(m.source,fixedSources());
 assert.equal(m.status,'isolated-numerical-qualified-not-retained');assert.equal(m.retainedContinuationTransfers,6);
 console.log(JSON.stringify({checkpoint:68,status:m.status,artifacts:Object.keys(m.files).length,gates:gates.length,
  tests:evidence.tests,kernelCases:evidence.kernel.counts.rows,publicRecordsPerVariantAndPolicy:6441,extendedRecordsPerVariant:384,
  successfulCaptures:41,preservedFailedCaptures:2,retainedContinuationTransfers:6,liveFiles:956,isolatedFiles:176,
  binaries:evidence.binaries,limits:evidence.limits,next:m.next}));
}
