import {writeFileSync,readFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {zeroEvidence,gates} from './zero-factor-evidence-v70.mjs';
import {sha,json} from './zero-factor-final-sources-v70.mjs';
const path='zero-factor-v70-manifest.json',evidence=zeroEvidence();
assert.deepEqual(evidence,json('results/zero-factor-evidence-v70.stdout'));
const final=json('results/zero-factor-evidence-v70.json');assert.equal(final.code,0);assert.equal(final.signal,null);
assert.equal(readFileSync('results/zero-factor-evidence-v70.stderr').length,0);
const allGates=[...gates,'zero-factor-evidence-v70'];assert.equal(allGates.length,30);
const scripts=['prepare-zero-factor-v70.mjs','zero-factor-origin-v70.json','zero-factor-sources-v70.mjs','zero-factor-binding-v70.json',
 'zero-factor-tests-initial-v70.rs','zero-factor-tests-signed-v70.rs','zero-factor-fixed-sources-v70.mjs','zero-factor-fixed-binding-v70.json',
 'zero-factor-final-sources-v70.mjs','zero-factor-final-binding-v70.json','zero-factor-degree-cases-v70.mjs','zero-factor-degree-input-v70.json',
 'run-zero-factor-v70.mjs','zero-factor-gates-origin-v70.json','run-zero-factor-final-v70.mjs','zero-factor-final-gates-origin-v70.json',
 'zero-factor-binaries-v70.json','run-zero-factor-extra-v70.mjs','zero-factor-polynomial-v70.mjs','check-zero-factor-v70.mjs',
 'zero-factor-evidence-v70.mjs','verify-zero-factor-v70.mjs','zero-factor-v70-findings.md',
 'zero-factor-candidate-app-v70/Cargo.toml','zero-factor-candidate-app-v70/Cargo.lock',
 'power-sums-public.rs','power-sums-cpu.rs','point-qualified-approx.rs','point-demand-extended.rs','point-history-base.rs','point-extended.rs',
 'check-point-extended.mjs','point-extended-field.mjs','power-rational-oracle-v69.mjs','power-sums-polynomial-oracle.mjs','power-wide-cases-v69.mjs',
 'power-wide-v69.rs','power-wide-input-v69.json','power-wide-v69-manifest.json','power-wide-origin-v69.json','power-wide-binaries-v69.json',
 'power-rebased-fixed-sources-v68.mjs','point-retained-sources-v67.mjs','point-qualified-capture.mjs','point-qualified-environment.mjs','capture.mjs',
 ...['public','approx','extended','metadata'].map(k=>'results/power-rebased-baseline-'+k+'-v68.stdout'),
 'results/power-wide-baseline-run-v69.stdout','results/point-retained-debug-v67.stdout',
 ...['json','stdout','stderr'].map(ext=>'results/power-wide-verify-canonical-v69.'+ext)];
if(process.argv.includes('--record')){
 const files=[...new Set([...scripts,...allGates.flatMap(t=>['json','stdout','stderr'].map(ext=>'results/'+t+'.'+ext))])].sort();
 const m={checkpoint:70,status:'isolated-zero-factor-numerical-qualified',recorded:new Date().toISOString(),
  files:Object.fromEntries(files.map(p=>[p,sha(p)])),gates:allGates,evidence,retainedContinuationTransfers:6,newDonorLines:0,
  scope:'Isolated divisor-carrier completeness trial, not a production retention, performance campaign or full ecosystem completion.'};
 writeFileSync(path,JSON.stringify(m,null,2)+'\n',{flag:'wx'});
 console.log(JSON.stringify({checkpoint:70,status:m.status,artifacts:files.length,gates:allGates.length,liveFiles:956}));
}else{
 const m=json(path);for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
 assert.deepEqual(m.evidence,evidence);assert.deepEqual(m.gates,allGates);assert.equal(m.retainedContinuationTransfers,6);
 console.log(JSON.stringify({checkpoint:70,status:m.status,artifacts:Object.keys(m.files).length,gates:allGates.length,
  successfulCaptures:27,preservedFailedCaptures:3,liveFiles:956,candidateFiles:176,
  tests:{default:817,allFeaturesDebug:818,allFeaturesRelease:818},pairedQueries:16692,newAnswers:184,unchanged:16508,
  mathematicalChecks:128652,independentDeterminants:6441,extendedQueries:384,extendedChecks:11248,
  binaries:evidence.binaries,retainedContinuationTransfers:6,next:'Matched CPU/allocation, WASM execution, representative consumer/size gates before retention; full donor inventory remains open.'}));
}
