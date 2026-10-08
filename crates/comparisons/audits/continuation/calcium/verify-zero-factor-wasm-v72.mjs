import {writeFileSync,readFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {wasmEvidence,gate} from './zero-factor-wasm-evidence-v72.mjs';
import {files as sources,sha,json} from './zero-factor-wasm-sources-v72.mjs';
const evidence=await wasmEvidence();
assert.deepEqual(evidence,json('results/zero-factor-wasm-evidence-v72.stdout'));gate('zero-factor-wasm-evidence-v72');
assert.equal(readFileSync('results/zero-factor-wasm-evidence-v72.stderr').length,0);
const gates=[...evidence.gates,'zero-factor-wasm-evidence-v72'];assert.equal(gates.length,23);
const files=[...new Set([...sources,'zero-factor-wasm-v72-findings.md','verify-zero-factor-wasm-v72.mjs',
 'zero-factor-wasm-origin-v72.json','zero-factor-wasm-binaries-v72.json','zero-factor-wasm-qualification-origin-v72.json',
 'zero-factor-wasm-qualification-v72.json','qualify-zero-factor-wasm-v72.mjs','zero-factor-wasm-runner-v72.mjs',
 'check-zero-factor-wasm-abi-v72.mjs','zero-factor-wasm-abi-origin-v72.json','zero-factor-wasm-abi-v72.json',
 'check-zero-factor-wasm-records-v72.mjs','check-zero-factor-wasm-corruptions-v72.mjs','zero-factor-wasm-corruptions-v72.json',
 'zero-factor-wasm-evidence-v72.mjs','results/zero-factor-wasm-failures-v72.jsonl','zero-factor-cost-v71-manifest.json',
 'zero-factor-cost-sources-v71.mjs','zero-factor-final-sources-v70.mjs','zero-factor-final-binding-v70.json','zero-factor-v70-manifest.json',
 ...['json','stdout','stderr'].map(ext=>'results/zero-factor-cost-verify-v71.'+ext),
 ...['baseline','candidate'].flatMap(v=>['rational','history'].map(k=>'zero-factor-wasm-'+v+'-'+k+'-v72/Cargo.lock')),
 ...gates.flatMap(t=>['json','stdout','stderr'].map(ext=>'results/'+t+'.'+ext))])].sort();
const path='zero-factor-wasm-v72-manifest.json';
if(process.argv.includes('--record')){
 const m={checkpoint:72,status:evidence.status,recorded:new Date().toISOString(),files:Object.fromEntries(files.map(p=>[p,sha(p)])),
  gates,evidence,retainedContinuationTransfers:6,newDonorLines:0,
  scope:'Isolated WASM correctness qualification; no live edit. Matched WASM costs, representative consumer/size and complete remaining reference inventory stay open.'};
 writeFileSync(path,JSON.stringify(m,null,2)+'\n',{flag:'wx'});
}else{
 const m=json(path);assert.deepEqual(Object.keys(m.files),files);for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
 assert.deepEqual(m.evidence,evidence);assert.deepEqual(m.gates,gates);assert.equal(m.retainedContinuationTransfers,6);assert.equal(m.newDonorLines,0);
}
console.log(JSON.stringify({checkpoint:72,status:evidence.status,artifacts:files.length,gates:gates.length,successfulCaptures:23,preservedFailedCaptures:0,
 liveFiles:956,candidateFiles:176,observations:38280,pairedRationalQueries:16692,newAnswers:184,unchangedAnswers:16508,extendedChecks:89984,
 abiNegativeControls:76,positiveSequences:4,additionalExpectedTraps:2,recordCorruptionsRejected:21,
 storage:evidence.storage,retainedContinuationTransfers:6,
 next:'Matched WASM costs, representative consumer/size gates before retention; full reference inventory remains open.'}));
