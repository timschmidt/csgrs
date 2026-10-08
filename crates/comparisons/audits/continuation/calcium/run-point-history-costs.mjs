import {writeFileSync,appendFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sha,json} from './point-qualified-sources.mjs';
import {config,variants,groups,groupKey,frame,checkBindings,validateRecord,observe,lines,iterationsFor,orderFor} from './point-history-protocol.mjs';
import {summarizer} from './point-history-statistics.mjs';
const mode=process.argv[2];assert(['cpu','allocation'].includes(mode));
const binding=checkBindings(),qualification=json('point-history-qualification.json');
assert.equal(qualification.status,'pass');assert.deepEqual(qualification.config,config);
assert.equal(sha('results/point-history-qualification.jsonl'),qualification.rowsSha256);
assert.equal(json('results/point-history-qualify.json').code,0);
const expected=new Map(lines('results/point-history-qualification.jsonl').filter(r=>r.mode==='cpu')
 .map(r=>[r.variant+':'+groupKey(r),frame(r)]));assert.equal(expected.size,1536);
const started=new Date().toISOString();
if(mode==='allocation')assert(Date.parse(json('point-history-cost-cpu-summary.json').finished)<=Date.parse(started));
const path='results/point-history-cost-'+mode+'.jsonl',pilotPath='results/point-history-cost-'+mode+'-pilots.jsonl',
 failurePath='results/point-history-cost-'+mode+'-failures.jsonl';
for(const p of [path,pilotPath,failurePath])writeFileSync(p,'',{flag:'wx'});
const summarize=summarizer(),summaries=[];let observations=0,pilotCount=0;
async function run(variant,group,iterations,file,block){
 const row={...(block===undefined?{}:{block}),...await observe(binding,variant,mode,group,iterations,failurePath)};
 appendFileSync(file,JSON.stringify(row)+'\n');
 validateRecord(row,{...group,variant,mode,iterations});assert.deepEqual(frame(row),expected.get(variant+':'+groupKey(row)));
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
checkBindings();assert.equal(observations,mode==='cpu'?36864:4608);assert.equal(pilotCount,mode==='cpu'?1536:0);
const result={status:'pass',mode,started,finished:new Date().toISOString(),config,observations,pilots:pilotCount,
 binariesSha256:sha('point-history-binaries.json'),qualificationSha256:sha('point-history-qualification.json'),
 rowsSha256:sha(path),pilotsSha256:sha(pilotPath),failuresSha256:sha(failurePath),summaries,
 limits:'CPU only from the uninstrumented binary, taskset core 6, twelve alternating ABBA/BAAB blocks per group. Separate allocator binary uses three paired blocks of eight iterations. Nine preconditioning calls precede timing; retained/fresh refer to input graph lifecycles, not cold processes or cold shared constants. Timing includes prior result drops, excludes the final result drop, and excludes serialization/full validation. Every final actual complete record is compared to independently qualified mathematical records; intermediate results have a polynomial-length checksum only. Final report remains alive at the allocation snapshot. Rust requested bytes/live/peak omit allocator overhead, RSS and stacks. Per-group bootstrap intervals are not adjusted for multiplicity. Changed outcomes perform different certified work and are not equal-work speedups. Native only; no WASM or retention claim.'};
writeFileSync('point-history-cost-'+mode+'-summary.json',JSON.stringify(result,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({mode,status:'pass',groups:768,observations,pilots:pilotCount}));
