import {createReadStream,statSync} from 'node:fs';
import {createInterface} from 'node:readline';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {sha,json} from './point-demand-sources.mjs';
import {lines} from './point-history-protocol.mjs';
import {validateExtendedObservation} from './check-point-extended.mjs';
import {config,flags,variants,groups,groupKey,frame,wasmBindings,nativeExpected,validateWasmRecord,
 orderFor,summarizer} from './point-wasm-protocol.mjs';
function timeRange(row,outer){
 assert(Date.parse(row.started)>=Date.parse(outer.started));assert(Date.parse(row.finished)>=Date.parse(row.started));
 assert(Date.parse(row.finished)<=Date.parse(outer.finished));
}
function runtime(r,b,count){
 assert.equal(r.affinity,'6');assert.equal(r.observations,count);assert.equal(r.gcs,count/8);assert.equal(r.gcEvery,8);
 assert.deepEqual(r.execArgv,flags);assert.equal(r.modules.length,3);
 for(const a of b.artifacts){const m=r.modules.find(m=>m.variant===a.variant);assert(m);for(const[k,v]of Object.entries(a))assert.equal(m[k],v);
  assert.deepEqual(m.imports,[]);assert.deepEqual(m.exports.map(x=>x.name).sort(),['memory','history_prepare','history_batch','history_finish','history_output_ptr'].sort());}
}
export function checkWasmQualification(){
 const b=wasmBindings(),q=json('point-wasm-qualification.json'),expected=nativeExpected();assert.equal(q.status,'pass');assert.deepEqual(q.config,config);
 assert.equal(q.binariesSha256,sha('point-wasm-binaries.json'));assert.equal(q.nativeReferenceSha256,sha('results/point-history-qualification.jsonl'));
 assert.equal(q.demandReferenceSha256,sha('results/point-demand-history.jsonl'));
 for(const[suffix,key]of [['.jsonl','rowsSha256'],['-failures.jsonl','failuresSha256']])assert.equal(sha('results/point-wasm-qualification'+suffix),q[key]);
 assert.equal(statSync('results/point-wasm-qualification-failures.jsonl').size,0);
 const rows=lines('results/point-wasm-qualification.jsonl');assert.equal(rows.length,4608);assert.equal(q.observations,4608);assert.equal(q.groups,768);
 const checks={},statuses={};let i=0,last=q.runtime.compiled;
 for(const g of groups())for(const v of variants)for(const n of config.qualificationBatchSizes){
  const r=rows[i++];validateWasmRecord(r,{...g,variant:v,iterations:n});assert.deepEqual(frame(r),expected.get(v+':'+groupKey(r)));
  timeRange(r,q);assert(Date.parse(r.started)>=Date.parse(last));last=r.finished;
  const c=validateExtendedObservation(frame(r));assert.deepEqual(c.failures,[]);
  for(const[k,n]of Object.entries(c.checks))checks[k]=(checks[k]??0)+n;statuses[r.actual.status]=(statuses[r.actual.status]??0)+1;
 }
 assert.deepEqual(q.checks,checks);assert.deepEqual(q.statuses,statuses);assert.equal(q.totalChecks,Object.values(checks).reduce((a,b)=>a+b,0));
 assert.equal(q.totalChecks,129856);runtime(q.runtime,b,4608);timeRange(q,json('results/point-wasm-qualify.json'));
 return{qualification:q,expected,binding:b};
}
async function* streamRows(path){
 const stream=createInterface({input:createReadStream(path),crlfDelay:Infinity});
 for await(const line of stream){assert(line.length);yield JSON.parse(line);}
}
export async function checkWasmCosts(){
 const {qualification:q,expected,binding:b}=checkWasmQualification(),s=json('point-wasm-cost-summary.json');
 assert.equal(s.status,'pass');assert.deepEqual(s.config,config);assert.equal(s.binariesSha256,sha('point-wasm-binaries.json'));
 assert.equal(s.qualificationSha256,sha('point-wasm-qualification.json'));
 const p='results/point-wasm-cost';
 for(const[suffix,key]of [['.jsonl','rowsSha256'],['-pilots.jsonl','pilotsSha256'],['-failures.jsonl','failuresSha256']])assert.equal(sha(p+suffix),s[key]);
 assert.equal(statSync(p+'-failures.jsonl').size,0);const pilots=lines(p+'-pilots.jsonl'),iterator=streamRows(p+'.jsonl')[Symbol.asyncIterator]();
 assert.equal(pilots.length,2304);assert.equal(s.pilots,2304);assert.equal(s.observations,55296);
 const summarize=summarizer(),recomputed=[];let count=0,pi=0,last=s.runtime.compiled;
 function consume(row,g,v,extra){
  validateWasmRecord(row,{...g,variant:v,...extra});assert.deepEqual(frame(row),expected.get(v+':'+groupKey(row)));
  timeRange(row,s);assert(Date.parse(row.started)>=Date.parse(last));last=row.finished;
 }
 for(const g of groups()){
  const ps=[],rows=[];
  for(const v of variants){const r=pilots[pi++];consume(r,g,v,{iterations:config.cpuPilotIterations});assert(!('block'in r));ps.push(r);}
  for(let block=0;block<config.cpuBlocks;block++)for(const v of orderFor('cpu',block)){
   const next=await iterator.next();assert.equal(next.done,false);consume(next.value,g,v,{block});rows.push(next.value);count++;
  }
  recomputed.push(summarize('cpu',g,rows,ps));
 }
 assert((await iterator.next()).done);assert.equal(count,55296);assert.deepEqual(s.summaries,recomputed);
 runtime(s.runtime,b,57600);assert(Date.parse(q.finished)<=Date.parse(s.started));
 const gate=json(p+'.json');assert.equal(gate.code,0);assert.equal(gate.signal,null);timeRange(s,gate);
 const compact=g=>({case:g.case,policy:g.policy,history:g.history,lifecycle:g.lifecycle,statuses:g.statuses,
  ratio:g.pairedMedianRatio,ci:g.pairedMedianBootstrap95,ns:g.nsPerQuery});
 const metric=gs=>gs.length?{groups:gs.length,pairedMedianRatioRange:[Math.min(...gs.map(g=>g.pairedMedianRatio)),Math.max(...gs.map(g=>g.pairedMedianRatio))],
  ciWhollyBelowOne:gs.filter(g=>g.pairedMedianBootstrap95[1]<1).length,ciWhollyAboveOne:gs.filter(g=>g.pairedMedianBootstrap95[0]>1).length,
  fastest:[...gs].sort((a,b)=>a.pairedMedianRatio-b.pairedMedianRatio).slice(0,5).map(compact),
  slowest:[...gs].sort((a,b)=>b.pairedMedianRatio-a.pairedMedianRatio).slice(0,5).map(compact),
  slowestConfirmed:[...gs].filter(g=>g.pairedMedianBootstrap95[0]>1).sort((a,b)=>b.pairedMedianRatio-a.pairedMedianRatio).slice(0,5).map(compact)}:{groups:0};
 const comparisons={};
 for(const ref of ['baseline','eager']){
  const all=s.summaries.map(g=>({...g,...g.comparisons[ref]})),same=all.filter(g=>g.sameFullResult),changed=all.filter(g=>!g.sameFullResult);
  assert.equal(same.length,ref==='baseline'?512:768);assert.equal(changed.length,ref==='baseline'?256:0);
  comparisons[ref]={sameResult:metric(same),changedResult:metric(changed)};
 }
 return{status:'pass',qualificationObservations:4608,independentQualificationChecks:129856,groups:768,
  cpuObservations:55296,cpuPilots:2304,comparisons,runtime:s.runtime,limits:s.limits};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(await checkWasmCosts()));
