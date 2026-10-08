import {writeFileSync,appendFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sha,json} from './point-demand-sources.mjs';
import {config,variants,groups,groupKey,frame,wasmBindings,nativeExpected,validateWasmRecord,wasmRunner,
 iterationsFor,orderFor,summarizer} from './point-wasm-protocol.mjs';
const binding=wasmBindings(),expected=nativeExpected(),q=json('point-wasm-qualification.json');assert.equal(q.status,'pass');
assert.equal(q.binariesSha256,sha('point-wasm-binaries.json'));assert.equal(q.rowsSha256,sha('results/point-wasm-qualification.jsonl'));
const path='results/point-wasm-cost.jsonl',pilotPath='results/point-wasm-cost-pilots.jsonl',failurePath='results/point-wasm-cost-failures.jsonl';
for(const p of [path,pilotPath,failurePath])writeFileSync(p,'',{flag:'wx'});
const started=new Date().toISOString(),runner=await wasmRunner(binding,failurePath),summarize=summarizer(),summaries=[];let observations=0,pilotCount=0;
function run(variant,group,iterations,path,block){
 const row={...(block===undefined?{}:{block}),...runner.observe(variant,group,iterations)};
 appendFileSync(path,JSON.stringify(row)+'\n');validateWasmRecord(row,{...group,variant,iterations});
 assert.deepEqual(frame(row),expected.get(variant+':'+groupKey(row)));runner.completed();return row;
}
for(const group of groups()){
 const pilots=[],rows=[];for(const v of variants){pilots.push(run(v,group,config.cpuPilotIterations,pilotPath));pilotCount++;}
 const iterations=iterationsFor(pilots);
 for(let block=0;block<config.cpuBlocks;block++)for(const v of orderFor('cpu',block)){
  rows.push(run(v,group,iterations,path,block));observations++;
 }
 summaries.push(summarize('cpu',group,rows,pilots));
 console.log(JSON.stringify({group:groupKey(group),iterations,observations,finished:new Date().toISOString()}));
}
assert.equal(observations,55296);assert.equal(pilotCount,2304);assert.deepEqual(wasmBindings(),binding);
const result={status:'pass',started,finished:new Date().toISOString(),config,observations,pilots:pilotCount,
 binariesSha256:sha('point-wasm-binaries.json'),qualificationSha256:sha('point-wasm-qualification.json'),
 rowsSha256:sha(path),pilotsSha256:sha(pilotPath),failuresSha256:sha(failurePath),summaries,runtime:runner.runtime(),
 limits:'Direct three-variant optimized-tier WASM comparison, 768 authored groups, common calibration and twelve balanced palindromic blocks. Fresh instance and nine preconditioning queries precede each timed batch. Retained/fresh workload exactly matches the native HistoryWork prefix; fresh mode also retains its initial reference roots, as native does. Final report stays alive; host call/bookkeeping are timed, final drop/serialization/checking/GC/compilation/setup are not. Intermediate iterations only have a checksum. Core 6 affinity, no fixed-frequency/idle-host guarantee. Individual bootstrap CIs are unadjusted for multiplicity. Newly successful versus baseline queries are different work. Not a default-tier/browser/whole-application benchmark or a memory/retention claim.'};
writeFileSync('point-wasm-cost-summary.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({status:'pass',groups:768,observations,pilots:pilotCount,runtime:result.runtime}));
