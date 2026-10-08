import {writeFileSync,appendFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sha} from './point-demand-sources.mjs';
import {validateExtendedObservation} from './check-point-extended.mjs';
import {config,variants,groups,groupKey,frame,wasmBindings,nativeExpected,validateWasmRecord,wasmRunner} from './point-wasm-protocol.mjs';
const binding=wasmBindings(),expected=nativeExpected(),path='results/point-wasm-qualification.jsonl',failurePath='results/point-wasm-qualification-failures.jsonl';
for(const p of [path,failurePath])writeFileSync(p,'',{flag:'wx'});
const started=new Date().toISOString(),runner=await wasmRunner(binding,failurePath);let observations=0;const checks={},statuses={};
for(const group of groups())for(const variant of variants)for(const iterations of config.qualificationBatchSizes){
 const row=runner.observe(variant,group,iterations);appendFileSync(path,JSON.stringify(row)+'\n');
 validateWasmRecord(row,{...group,variant,iterations});assert.deepEqual(frame(row),expected.get(variant+':'+groupKey(row)));
 const result=validateExtendedObservation(frame(row));assert.deepEqual(result.failures,[]);
 for(const[k,n]of Object.entries(result.checks))checks[k]=(checks[k]??0)+n;
 statuses[row.actual.status]=(statuses[row.actual.status]??0)+1;observations++;runner.completed();
}
assert.equal(observations,4608);assert.deepEqual(wasmBindings(),binding);
const result={status:'pass',started,finished:new Date().toISOString(),config,groups:768,observations,checks,
 totalChecks:Object.values(checks).reduce((a,b)=>a+b,0),statuses,binariesSha256:sha('point-wasm-binaries.json'),
 nativeReferenceSha256:sha('results/point-history-qualification.jsonl'),demandReferenceSha256:sha('results/point-demand-history.jsonl'),
 rowsSha256:sha(path),failuresSha256:sha(failurePath),runtime:runner.runtime(),
 limits:'Full final records independently checked at one/eight iterations after nine preconditioning calls, and compared with qualified native records. Intermediate batch iterations use a polynomial-length checksum. These qualification elapsed fields are not the balanced cost campaign.'};
writeFileSync('point-wasm-qualification.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});console.log(JSON.stringify(result));
