import {writeFileSync,readFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {costEvidence} from './zero-factor-cost-evidence-v71.mjs';
import {harnessFiles,sha,json} from './zero-factor-cost-sources-v71.mjs';
const evidence=costEvidence(),path='zero-factor-cost-v71-manifest.json';
assert.deepEqual(evidence,json('results/zero-factor-cost-evidence-v71.stdout'));
const done=json('results/zero-factor-cost-evidence-v71.json');assert.equal(done.code,0);assert.equal(done.signal,null);
assert.equal(readFileSync('results/zero-factor-cost-evidence-v71.stderr').length,0);
const gates=[...evidence.gates,'zero-factor-cost-evidence-v71'];assert.equal(gates.length,138);
const runs=json('zero-factor-native-runs-v71.json').runs,probe=json('zero-factor-allocation-probe-v71.json').runs,
 controls=json('zero-factor-cost-controls-v71.json').results;
const files=[...new Set([...harnessFiles,'zero-factor-native-v71-findings.md','verify-zero-factor-cost-v71.mjs',
 'zero-factor-cost-origin-v71.json','zero-factor-cost-binaries-v71.json','zero-factor-cost-check-plan-v71.json',
 'zero-factor-native-origin-v71.json','zero-factor-native-runs-v71.json','zero-factor-native-iterations-v71.json',
 'zero-factor-native-summary-v71.json','run-zero-factor-native-v71.mjs','zero-factor-native-environment-v71.mjs',
 'check-zero-factor-native-v71.mjs','zero-factor-cost-evidence-v71.mjs','probe-zero-factor-allocation-v71.mjs',
 'zero-factor-allocation-probe-origin-v71.json','zero-factor-allocation-probe-v71.json',
 'check-zero-factor-cost-controls-v71.mjs','zero-factor-cost-controls-v71.json',
 'zero-factor-v70-manifest.json','zero-factor-final-sources-v70.mjs','zero-factor-final-binding-v70.json',
 'check-zero-factor-v70.mjs','zero-factor-polynomial-v70.mjs','zero-factor-degree-cases-v70.mjs','power-wide-cases-v69.mjs',
 'power-wide-input-v69.json','zero-factor-degree-input-v70.json','power-rational-oracle-v69.mjs','power-sums-polynomial-oracle.mjs',
 ...['baseline','candidate'].map(v=>'zero-factor-cost-'+v+'-app-v71/Cargo.lock'),
 ...['public','approx','extended'].map(k=>'results/power-rebased-baseline-'+k+'-v68.stdout'),
 ...['public','approx','wide','degree'].map(k=>'results/zero-factor-'+k+'-run-v70.stdout'),
 'results/zero-factor-degree-baseline-v70.stdout','results/power-wide-baseline-run-v69.stdout',
 ...['json','stdout','stderr'].map(ext=>'results/zero-factor-verify-v70.'+ext),
 ...runs.map(r=>r.plan),...probe.map(r=>r.plan),...controls.map(r=>r.path),
 ...gates.flatMap(t=>['json','stdout','stderr'].map(ext=>'results/'+t+'.'+ext))])].sort();
if(process.argv.includes('--record')){
 const m={checkpoint:71,status:evidence.status,recorded:new Date().toISOString(),files:Object.fromEntries(files.map(p=>[p,sha(p)])),
  gates,evidence,retainedContinuationTransfers:6,newDonorLines:0,
  scope:'Isolated native cost qualification with unchanged live sources; WASM execution, representative consumer/size and full ecosystem audit remain open.'};
 writeFileSync(path,JSON.stringify(m,null,2)+'\n',{flag:'wx'});
 console.log(JSON.stringify({checkpoint:71,status:m.status,artifacts:files.length,gates:gates.length,liveFiles:956}));
}else{
 const m=json(path);assert.deepEqual(Object.keys(m.files),files);for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
 assert.deepEqual(m.evidence,evidence);assert.deepEqual(m.gates,gates);assert.equal(m.retainedContinuationTransfers,6);
 console.log(JSON.stringify({checkpoint:71,status:m.status,artifacts:files.length,gates:gates.length,successfulCaptures:138,preservedFailedCaptures:0,
  liveFiles:956,candidateFiles:176,preflightChecks:1824,cpuRows:21888,allocationRows:7296,pilotRows:1824,
  equalResultGroups:408,newAnswerGroups:48,allocationAttribution:{processes:56,records:448,pairedEqual:224},negativeControls:9,
  nativeBoundedDeflation:{groups:196,pairedMedianRange:[0.2101956800271102,0.8093480003843853]},
  binaries:evidence.binaries,retainedContinuationTransfers:6,
  next:'WASM execution and representative consumer/size qualification before deciding retention; full reference inventory remains open.'}));
}
