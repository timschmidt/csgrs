import {createReadStream,statSync} from 'node:fs';
import {createInterface} from 'node:readline';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {sha,json} from './point-demand-sources.mjs';
import {demandBindings} from './point-demand-bindings.mjs';
import {validateExtendedObservation} from './check-point-extended.mjs';
import {groups,groupKey,frame,lines,validateRecord,config as historyConfig} from './point-history-protocol.mjs';
import {config,variants,orderFor,summarizer,counters} from './point-demand-cost-protocol.mjs';
function timeRange(row,outer){
 assert(Date.parse(row.started)>=Date.parse(outer.started));assert(Date.parse(row.finished)>=Date.parse(row.started));
 assert(Date.parse(row.finished)<=Date.parse(outer.finished));
}
export function checkDemandHistory(){
 demandBindings();const q=json('point-demand-history.json'),rows=lines('results/point-demand-history.jsonl');
 assert.equal(q.status,'pass');assert.deepEqual(q.config,historyConfig);assert.equal(q.binariesSha256,sha('point-demand-binaries.json'));
 assert.equal(q.rowsSha256,sha('results/point-demand-history.jsonl'));assert.equal(q.failuresSha256,sha('results/point-demand-history-failures.jsonl'));
 assert.equal(q.referenceSha256,sha('results/point-history-qualification.jsonl'));assert.equal(statSync('results/point-demand-history-failures.jsonl').size,0);
 assert.equal(rows.length,1536);assert.equal(q.observations,1536);assert.equal(q.groups,768);
 const expected=new Map();
 for(const r of lines('results/point-history-qualification.jsonl').filter(r=>r.mode==='cpu'))expected.set((r.variant==='candidate'?'eager':'baseline')+':'+groupKey(r),frame(r));
 let i=0,last=q.started;const checks={},statuses={};
 for(const group of groups())for(const mode of ['cpu','allocation']){
  const row=rows[i++];validateRecord(row,{variant:'demand',mode,...group,iterations:config.qualificationIterations[mode]});
  timeRange(row,q);assert(Date.parse(row.started)>=Date.parse(last));last=row.finished;
  const value=frame(row);assert.deepEqual(value,expected.get('eager:'+groupKey(row)));
  const result=validateExtendedObservation(value);assert.deepEqual(result.failures,[]);
  for(const[k,n]of Object.entries(result.checks))checks[k]=(checks[k]??0)+n;
  statuses[row.actual.status]=(statuses[row.actual.status]??0)+1;
  if(mode==='cpu')expected.set('demand:'+groupKey(row),value);
 }
 assert.equal(expected.size,2304);assert.deepEqual(q.checks,checks);assert.deepEqual(q.statuses,statuses);
 assert.equal(q.totalChecks,Object.values(checks).reduce((a,b)=>a+b,0));assert.equal(q.totalChecks,44992);
 return{qualification:q,expected};
}
async function* streamRows(path){
 const stream=createInterface({input:createReadStream(path),crlfDelay:Infinity});
 for await(const line of stream){assert(line.length);yield JSON.parse(line);}
}
export async function checkDemandCosts(){
 const {qualification,expected}=checkDemandHistory(),summaries={};
 for(const mode of ['cpu','allocation']){
  const s=json('point-demand-cost-'+mode+'-summary.json');assert.equal(s.status,'pass');assert.equal(s.mode,mode);assert.deepEqual(s.config,config);
  assert.equal(s.binariesSha256,sha('point-demand-binaries.json'));assert.equal(s.previousBinariesSha256,sha('point-history-binaries.json'));
  assert.equal(s.qualificationSha256,sha('point-demand-history.json'));
  const p='results/point-demand-cost-'+mode;
  for(const[suffix,key]of [['.jsonl','rowsSha256'],['-pilots.jsonl','pilotsSha256'],['-failures.jsonl','failuresSha256']])assert.equal(sha(p+suffix),s[key]);
  assert.equal(statSync(p+'-failures.jsonl').size,0);
  const pilots=lines(p+'-pilots.jsonl'),iterator=streamRows(p+'.jsonl')[Symbol.asyncIterator](),summarize=summarizer(),recomputed=[];
  assert.equal(pilots.length,mode==='cpu'?2304:0);assert.equal(s.pilots,pilots.length);
  let count=0,last=s.started,pilotIndex=0;
  const consume=(row,group,variant,extra)=>{
   validateRecord(row,{...group,mode,variant,...extra});assert.deepEqual(frame(row),expected.get(variant+':'+groupKey(row)));
   timeRange(row,s);assert(Date.parse(row.started)>=Date.parse(last));last=row.finished;
  };
  for(const group of groups()){
   const ps=[],rows=[];
   if(mode==='cpu')for(const variant of variants){const row=pilots[pilotIndex++];consume(row,group,variant,{iterations:config.cpuPilotIterations});assert(!('block'in row));ps.push(row);}
   for(let block=0;block<(mode==='cpu'?config.cpuBlocks:config.allocationBlocks);block++)for(const variant of orderFor(mode,block)){
    const next=await iterator.next();assert.equal(next.done,false);consume(next.value,group,variant,{block});rows.push(next.value);count++;
   }
   recomputed.push(summarize(mode,group,rows,ps));
  }
  assert.equal((await iterator.next()).done,true);assert.equal(count,mode==='cpu'?55296:6912);assert.equal(s.observations,count);assert.deepEqual(s.summaries,recomputed);
  const gate=json(p+'.json');assert.equal(gate.code,0);assert.equal(gate.signal,null);timeRange(s,gate);summaries[mode]=s;
 }
 assert(Date.parse(qualification.finished)<=Date.parse(summaries.cpu.started));
 assert(Date.parse(json('results/point-demand-qualification.json').finished)<=Date.parse(summaries.cpu.started));
 assert(Date.parse(summaries.cpu.finished)<=Date.parse(summaries.allocation.started));
 const comparisons={};
 for(const reference of ['baseline','eager']){
  const cpu=summaries.cpu.summaries.map(s=>({...s,...s.comparisons[reference]}));
  const same=cpu.filter(g=>g.sameFullResult),changed=cpu.filter(g=>!g.sameFullResult);
  assert.equal(same.length,reference==='baseline'?512:768);assert.equal(changed.length,reference==='baseline'?256:0);
  const compact=g=>({case:g.case,policy:g.policy,history:g.history,lifecycle:g.lifecycle,statuses:g.statuses,
   ratio:g.pairedMedianRatio,ci:g.pairedMedianBootstrap95,ns:g.nsPerQuery});
  const metric=xs=>xs.length?{groups:xs.length,pairedMedianRatioRange:[Math.min(...xs.map(g=>g.pairedMedianRatio)),Math.max(...xs.map(g=>g.pairedMedianRatio))],
   ciWhollyBelowOne:xs.filter(g=>g.pairedMedianBootstrap95[1]<1).length,ciWhollyAboveOne:xs.filter(g=>g.pairedMedianBootstrap95[0]>1).length,
   fastest:[...xs].sort((a,b)=>a.pairedMedianRatio-b.pairedMedianRatio).slice(0,5).map(compact),
   slowest:[...xs].sort((a,b)=>b.pairedMedianRatio-a.pairedMedianRatio).slice(0,5).map(compact)}:{groups:0};
  const counts=Object.fromEntries(counters.map(k=>[k,{sameResultLower:0,sameResultEqual:0,sameResultHigher:0,sameResultVariable:0,
   changedResultLower:0,changedResultEqual:0,changedResultHigher:0,changedResultVariable:0}]));
  const allocation=summaries.allocation.summaries.map((g,i)=>{
   assert.equal(groupKey(g),groupKey(cpu[i]));const sameFullResult=g.comparisons[reference].sameFullResult;assert.equal(sameFullResult,cpu[i].sameFullResult);
   const delta=Object.fromEntries(counters.map(k=>[k,{min:g.measurements.demand[k].min-g.measurements[reference][k].max,
    max:g.measurements.demand[k].max-g.measurements[reference][k].min}]));
   for(const k of counters){const d=delta[k],prefix=sameFullResult?'sameResult':'changedResult';
    const suffix=d.max<0?'Lower':d.min>0?'Higher':d.min===0&&d.max===0?'Equal':'Variable';counts[k][prefix+suffix]++;}
   return{case:g.case,policy:g.policy,history:g.history,lifecycle:g.lifecycle,sameFullResult,iterations:g.iterations,delta};
  });
  const ranges=Object.fromEntries([true,false].map(same=>{
   const gs=allocation.filter(g=>g.sameFullResult===same);return[same?'sameResult':'changedResult',gs.length?Object.fromEntries(counters.map(k=>[k,
    {min:Math.min(...gs.map(g=>g.delta[k].min)),max:Math.max(...gs.map(g=>g.delta[k].max))}])):null];
  }));
  comparisons[reference]={sameResult:metric(same),changedResult:metric(changed),allocationCounts:counts,allocationDeltaRanges:ranges,
   allocationChanges:allocation.filter(g=>Object.values(g.delta).some(d=>d.min!==0||d.max!==0))};
 }
 return{status:'pass',historyQualificationObservations:1536,historyExactChecks:44992,groups:768,
  cpuObservations:55296,cpuPilots:2304,allocationObservations:6912,comparisons,limits:summaries.cpu.limits};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(await checkDemandCosts()));
