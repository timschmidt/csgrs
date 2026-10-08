import {createReadStream,statSync} from 'node:fs';
import {createInterface} from 'node:readline';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import {isDeepStrictEqual} from 'node:util';
import assert from 'node:assert/strict';
import {sha,json} from './point-qualified-sources.mjs';
import {fieldSelfTest} from './point-extended-field.mjs';
import {validateExtendedObservation} from './check-point-extended.mjs';
import {config,variants,modes,groups,groupKey,frame,checkBindings,validateRecord,lines,orderFor} from './point-history-protocol.mjs';
import {summarizer} from './point-history-statistics.mjs';
const counters=['requests','requested_bytes','return_live_delta','peak_delta'];
function timeRange(row,outer){
 assert(Date.parse(row.started)>=Date.parse(outer.started));
 assert(Date.parse(row.finished)>=Date.parse(row.started));
 assert(Date.parse(row.finished)<=Date.parse(outer.finished));
}
export function checkHistoryQualification(){
 const binding=checkBindings(),q=json('point-history-qualification.json'),rows=lines('results/point-history-qualification.jsonl');
 assert.equal(q.status,'pass');assert.deepEqual(q.config,config);assert.equal(q.binariesSha256,sha('point-history-binaries.json'));
 assert.equal(q.rowsSha256,sha('results/point-history-qualification.jsonl'));
 assert.equal(q.failuresSha256,sha('results/point-history-qualification-failures.jsonl'));
 assert.equal(statSync('results/point-history-qualification-failures.jsonl').size,0);
 assert.equal(rows.length,3072);assert.equal(q.observations,3072);assert.equal(q.groups,768);fieldSelfTest();
 const checks={},statuses={},expected=new Map(),preconditioningDifferences=[];
 const old=Object.fromEntries(variants.map(v=>[v,new Map(lines('results/point-extended-public-'+v+'.stdout')
  .filter(r=>r.type==='query').map(r=>[[r.case,r.policy,r.history].join(':'),r]))]));
 let i=0,improved=0,unchanged=0,last=q.started;
 for(const group of groups()){
  const records={};
  for(const variant of variants)for(const mode of modes){
   const row=rows[i++];validateRecord(row,{...group,variant,mode,iterations:config.qualificationIterations[mode]});
   timeRange(row,q);assert(Date.parse(row.started)>=Date.parse(last));last=row.finished;
   const value=frame(row),result=validateExtendedObservation(value);assert.deepEqual(result.failures,[]);
   for(const[k,n]of Object.entries(result.checks))checks[k]=(checks[k]??0)+n;
   const key=[variant,mode,row.actual.status].join(':');statuses[key]=(statuses[key]??0)+1;
   if(mode==='cpu'){records[variant]=value;expected.set(variant+':'+groupKey(row),value);}
   else assert.deepEqual(value,records[variant]);
  }
  const a=records.baseline,b=records.candidate;
  if(a.report.status==='InvalidTransformedEvidence'){
   assert.equal(a.report.message,'a collapsed refinement interval requires an exact witness');
   assert.equal(b.report.status,'Transformed');assert.deepEqual({...b,report:a.report},a);improved++;
  }else{assert.deepEqual(a,b);unchanged++;}
  for(const v of variants){
   const prior=old[v].get([group.case,group.policy,group.history].join(':'));
   if(!isDeepStrictEqual(prior,records[v]))preconditioningDifferences.push({variant:v,...group,before:prior.report.status,after:records[v].report.status,
    inputChanged:JSON.stringify([prior.left,prior.right])!==JSON.stringify([records[v].left,records[v].right]),
    reportChanged:JSON.stringify(prior.report)!==JSON.stringify(records[v].report)});
  }
 }
 assert.deepEqual(q.checks,checks);assert.deepEqual(q.statuses,statuses);
 assert.equal(q.totalChecks,Object.values(checks).reduce((a,b)=>a+b,0));assert.equal(q.totalChecks,84864);
 assert.equal(q.improved,improved);assert.equal(improved,256);assert.equal(q.unchanged,unchanged);assert.equal(unchanged,512);
 assert.deepEqual(q.preconditioningDifferences,preconditioningDifferences);assert.equal(preconditioningDifferences.length,0);
 return{binding,qualification:q,expected};
}
async function* streamRows(path){
 const stream=createInterface({input:createReadStream(path),crlfDelay:Infinity});
 for await(const line of stream){assert(line.length>0);yield JSON.parse(line);}
}
export async function checkPointHistory(){
 const {qualification,expected}=checkHistoryQualification(),summaries={};
 for(const mode of modes){
  const s=json('point-history-cost-'+mode+'-summary.json');assert.equal(s.status,'pass');assert.equal(s.mode,mode);assert.deepEqual(s.config,config);
  assert.equal(s.binariesSha256,sha('point-history-binaries.json'));assert.equal(s.qualificationSha256,sha('point-history-qualification.json'));
  const p='results/point-history-cost-'+mode;
  for(const[suffix,key]of [['.jsonl','rowsSha256'],['-pilots.jsonl','pilotsSha256'],['-failures.jsonl','failuresSha256']])assert.equal(sha(p+suffix),s[key]);
  assert.equal(statSync(p+'-failures.jsonl').size,0);
  const pilots=lines(p+'-pilots.jsonl'),iterator=streamRows(p+'.jsonl')[Symbol.asyncIterator](),summarize=summarizer(),recomputed=[];
  assert.equal(pilots.length,mode==='cpu'?1536:0);assert.equal(s.pilots,pilots.length);
  let count=0,last=s.started,pilotIndex=0;
  const consume=(row,group,variant,extra)=>{
   validateRecord(row,{...group,mode,variant,...extra});assert.deepEqual(frame(row),expected.get(variant+':'+groupKey(row)));
   timeRange(row,s);assert(Date.parse(row.started)>=Date.parse(last));last=row.finished;
  };
  for(const group of groups()){
   const ps=[];
   if(mode==='cpu')for(const variant of variants){const row=pilots[pilotIndex++];consume(row,group,variant,{iterations:config.cpuPilotIterations});assert(!('block'in row));ps.push(row);}
   const rows=[];
   for(let block=0;block<(mode==='cpu'?config.cpuBlocks:config.allocationBlocks);block++)for(const variant of orderFor(mode,block)){
    const next=await iterator.next();assert.equal(next.done,false);consume(next.value,group,variant,{block});rows.push(next.value);count++;
   }
   recomputed.push(summarize(mode,group,rows,ps));
  }
  assert.equal((await iterator.next()).done,true);assert.equal(count,mode==='cpu'?36864:4608);assert.equal(s.observations,count);
  assert.deepEqual(s.summaries,recomputed);
  const gate=json(p+'.json');assert.equal(gate.code,0);assert.equal(gate.signal,null);timeRange(s,gate);
  summaries[mode]=s;
 }
 assert(Date.parse(qualification.finished)<=Date.parse(summaries.cpu.started));
 assert(Date.parse(summaries.cpu.finished)<=Date.parse(summaries.allocation.started));
 const same=summaries.cpu.summaries.filter(g=>g.sameFullResult),changed=summaries.cpu.summaries.filter(g=>!g.sameFullResult);
 assert.equal(same.length,512);assert.equal(changed.length,256);
 const compact=g=>({case:g.case,policy:g.policy,history:g.history,lifecycle:g.lifecycle,statuses:g.statuses,
  ratio:g.pairedMedianRatio,ci:g.pairedMedianBootstrap95,ns:g.nsPerQuery});
 const counts=Object.fromEntries(counters.map(k=>[k,{sameResultLower:0,sameResultEqual:0,sameResultHigher:0,sameResultVariable:0,
  changedResultLower:0,changedResultEqual:0,changedResultHigher:0,changedResultVariable:0}]));
 const allocation=summaries.allocation.summaries.map((g,i)=>{
  assert.equal(groupKey(g),groupKey(summaries.cpu.summaries[i]));assert.equal(g.sameFullResult,summaries.cpu.summaries[i].sameFullResult);
  const delta=Object.fromEntries(counters.map(k=>[k,{min:g.measurements.candidate[k].min-g.measurements.baseline[k].max,
   max:g.measurements.candidate[k].max-g.measurements.baseline[k].min}]));
  for(const k of counters){const d=delta[k],prefix=g.sameFullResult?'sameResult':'changedResult';
   const suffix=d.max<0?'Lower':d.min>0?'Higher':d.min===0&&d.max===0?'Equal':'Variable';counts[k][prefix+suffix]++;}
  return{case:g.case,policy:g.policy,history:g.history,lifecycle:g.lifecycle,sameFullResult:g.sameFullResult,iterations:g.iterations,delta};
 });
 const metric=xs=>({groups:xs.length,pairedMedianRatioRange:[Math.min(...xs.map(g=>g.pairedMedianRatio)),Math.max(...xs.map(g=>g.pairedMedianRatio))],
  ciWhollyBelowOne:xs.filter(g=>g.pairedMedianBootstrap95[1]<1).length,ciWhollyAboveOne:xs.filter(g=>g.pairedMedianBootstrap95[0]>1).length,
  fastest:[...xs].sort((a,b)=>a.pairedMedianRatio-b.pairedMedianRatio).slice(0,5).map(compact),
  slowest:[...xs].sort((a,b)=>b.pairedMedianRatio-a.pairedMedianRatio).slice(0,5).map(compact)});
 return{status:'pass',qualificationObservations:3072,exactQualificationChecks:84864,cpuObservations:36864,cpuPilots:1536,
  allocationObservations:4608,groups:768,sameResult:metric(same),changedResult:metric(changed),allocationCounts:counts,
  allocationDeltaRanges:Object.fromEntries([true,false].map(same=>[same?'sameResult':'changedResult',Object.fromEntries(counters.map(k=>{
   const gs=allocation.filter(g=>g.sameFullResult===same);return[k,{min:Math.min(...gs.map(g=>g.delta[k].min)),max:Math.max(...gs.map(g=>g.delta[k].max))}];}))])),
  allocationChanges:allocation.filter(g=>Object.values(g.delta).some(d=>d.min!==0||d.max!==0)),
  limits:summaries.cpu.limits};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(await checkPointHistory()));
