import {writeFileSync,appendFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sha,json} from './point-demand-sources.mjs';
import {demandBindings} from './point-demand-bindings.mjs';
import {groups,groupKey,frame,lines,iterationsFor,validateRecord,observe} from './point-history-protocol.mjs';
import {config,variants,orderFor,summarizer} from './point-demand-cost-protocol.mjs';
const mode=process.argv[2];assert(['cpu','allocation'].includes(mode));const binding=demandBindings();
for(const tag of ['qualification','history-qualify'])assert.equal(json('results/point-demand-'+tag+'.json').code,0);
const q=json('point-demand-history.json');assert.equal(q.status,'pass');assert.equal(q.rowsSha256,sha('results/point-demand-history.jsonl'));
const expected=new Map();
for(const r of lines('results/point-history-qualification.jsonl').filter(r=>r.mode==='cpu')){
 const v=r.variant==='candidate'?'eager':'baseline';expected.set(v+':'+groupKey(r),frame(r));
}
for(const r of lines('results/point-demand-history.jsonl').filter(r=>r.mode==='cpu'))expected.set('demand:'+groupKey(r),frame(r));
assert.equal(expected.size,2304);
const started=new Date().toISOString();
if(mode==='allocation')assert(Date.parse(json('point-demand-cost-cpu-summary.json').finished)<=Date.parse(started));
const path='results/point-demand-cost-'+mode+'.jsonl',pilotPath='results/point-demand-cost-'+mode+'-pilots.jsonl',failurePath='results/point-demand-cost-'+mode+'-failures.jsonl';
for(const p of [path,pilotPath,failurePath])writeFileSync(p,'',{flag:'wx'});
const summarize=summarizer(),summaries=[];let observations=0,pilotCount=0;
async function run(variant,group,iterations,file,block){
 const row={...(block===undefined?{}:{block}),...await observe(binding,variant,mode,group,iterations,failurePath)};
 appendFileSync(file,JSON.stringify(row)+'\n');validateRecord(row,{...group,variant,mode,iterations});assert.deepEqual(frame(row),expected.get(variant+':'+groupKey(row)));
 return row;
}
for(const group of groups()){
 const pilots=[],rows=[];
 if(mode==='cpu')for(const v of variants){pilots.push(await run(v,group,config.cpuPilotIterations,pilotPath));pilotCount++;}
 const iterations=mode==='cpu'?iterationsFor(pilots):config.allocationIterations;
 for(let block=0;block<(mode==='cpu'?config.cpuBlocks:config.allocationBlocks);block++)for(const v of orderFor(mode,block)){
  rows.push(await run(v,group,iterations,path,block));observations++;
 }
 summaries.push(summarize(mode,group,rows,pilots));
 console.log(JSON.stringify({mode,group:groupKey(group),iterations,observations,finished:new Date().toISOString()}));
}
demandBindings();assert.equal(observations,mode==='cpu'?55296:6912);assert.equal(pilotCount,mode==='cpu'?2304:0);
const result={status:'pass',mode,started,finished:new Date().toISOString(),config,observations,pilots:pilotCount,
 binariesSha256:sha('point-demand-binaries.json'),previousBinariesSha256:sha('point-history-binaries.json'),qualificationSha256:sha('point-demand-history.json'),
 rowsSha256:sha(path),pilotsSha256:sha(pilotPath),failuresSha256:sha(failurePath),summaries,
 limits:'Three frozen variants measured together, not ratios of separate campaigns. Twelve palindromic six-observation blocks cover every permutation twice; common iteration count calibrated from all three pilots. Core 6 affinity, no fixed-frequency/idle-host guarantee. Independent allocation binary uses three rotated blocks of eight iterations. Nine preconditioning calls, retained/fresh input graphs and four explicit histories; not cold process-wide constants. Clock/counters exclude final result drop, serialization and full validation; final report remains alive. Every final full record matches independent qualification, intermediate iterations only carry a polynomial-length checksum. Requested bytes/net live/peak are not RSS or general memory bounds. Bootstrap intervals are individual and unadjusted for multiplicity. Demand/baseline groups with additional answers are different work. Native only; no consumer/WASM/retention claim.'};
writeFileSync('point-demand-cost-'+mode+'-summary.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({status:'pass',mode,groups:768,observations,pilots:pilotCount}));
