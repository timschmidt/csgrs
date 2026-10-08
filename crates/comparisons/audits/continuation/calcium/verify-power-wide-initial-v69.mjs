import {writeFileSync,readFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {wideEvidence,gates} from './power-wide-evidence-v69.mjs';
import {sha,json} from './power-rebased-fixed-sources-v68.mjs';
const path='power-wide-v69-manifest.json',evidence=wideEvidence();
assert.deepEqual(evidence,json('results/power-wide-evidence-v69.stdout'));
const check=json('results/power-wide-evidence-v69.json');assert.equal(check.code,0);assert.equal(check.signal,null);
assert.equal(readFileSync('results/power-wide-evidence-v69.stderr').length,0);
const allGates=[...gates,'power-wide-evidence-v69'];assert.equal(allGates.length,15);
if(process.argv.includes('--record')){
 const files=[...new Set(['power-wide-v69-findings.md','power-wide-cases-v69.mjs','power-wide-input-v69.json',
  'power-wide-v69.rs','run-power-wide-v69.mjs','power-wide-origin-v69.json','power-wide-binaries-v69.json',
  'check-power-wide-v69.mjs','power-rational-oracle-v69.mjs','probe-zero-deflation-v69.mjs','power-wide-evidence-v69.mjs',
  'verify-power-wide-v69.mjs','power-sums-public.rs','power-sums-polynomial-oracle.mjs','check-power-sums-public.mjs',
  'power-rebased-v68-manifest.json','power-rebased-fixed-binding-v68.json','power-rebased-fixed-sources-v68.mjs',
  'point-qualified-capture.mjs','point-qualified-environment.mjs','capture.mjs',
  ...['baseline','candidate'].flatMap(v=>['Cargo.toml','Cargo.lock'].map(f=>'power-wide-'+v+'-app-v69/'+f)),
  ...['json','stdout','stderr'].map(ext=>'results/power-rebased-verify-v68.'+ext),
  ...allGates.flatMap(t=>['json','stdout','stderr'].map(ext=>'results/'+t+'.'+ext))])].sort();
 const m={checkpoint:69,status:'wide-qualified-completeness-opportunity-open',recorded:new Date().toISOString(),
  files:Object.fromEntries(files.map(p=>[p,sha(p)])),gates:allGates,evidence,retainedContinuationTransfers:6,newDonorLines:0,
  scope:'Wide corpus and independent factor-removal opportunity, not a retained change, performance campaign or full ecosystem completion.'};
 writeFileSync(path,JSON.stringify(m,null,2)+'\n',{flag:'wx'});
 console.log(JSON.stringify({checkpoint:69,status:m.status,artifacts:files.length,gates:allGates.length,liveFiles:956}));
}else{
 const m=json(path);for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);assert.deepEqual(m.evidence,evidence);assert.deepEqual(m.gates,allGates);
 assert.equal(m.status,'wide-qualified-completeness-opportunity-open');assert.equal(m.retainedContinuationTransfers,6);
 console.log(JSON.stringify({checkpoint:69,status:m.status,artifacts:Object.keys(m.files).length,gates:allGates.length,
  liveFiles:956,isolatedFiles:176,cases:evidence.wide.cases,recordsPerVariant:evidence.wide.recordsPerVariant,
  checks:evidence.wide.totalChecks,determinants:evidence.wide.independentDeterminants,
  maxInputNumeratorBits:evidence.wide.maxInputNumeratorBits,maxResultCoefficientBits:evidence.wide.maxResultCoefficientBits,
  opportunityRecords:70,opportunityChecks:810,retainedContinuationTransfers:6,artifactsBytes:evidence.artifacts,next:evidence.next}));
}
