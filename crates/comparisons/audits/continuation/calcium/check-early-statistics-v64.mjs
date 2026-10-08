// This fourteen-checkpoint chain reproduces historical integrity, not the
// validity of its withdrawn estimator. It also rewrites an identical existing
// qualification-summary.json; its contents are checked by the old chain.
import './verify-polynomial-decision.mjs';
import assert from 'node:assert/strict';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import {json} from './point-demand-sources.mjs';
import {medianOrderInterval,bootstrapMedian} from './paired-statistics-v60.mjs';
import {campaigns,paths,readRows,reconstruct,historicalEstimator,reanalyse,counts} from './reanalyse-early-statistics-v64.mjs';
export function controls(){
 const rejected=[];
 function control(id,mode,name,mutate,index=0){
  const c=campaigns.find(c=>c.id===id),p=paths(c,mode),g=structuredClone(json(p.summary).summaries[index]),n=mode==='cpu'?48:12,
   rows=readRows(p.raw).slice(index*n,(index+1)*n);
  reconstruct(c,g,rows,mode);mutate(g,rows);
  assert.throws(()=>reconstruct(c,g,rows,mode),assert.AssertionError,id+'/'+name);rejected.push(id+'/'+name);
 }
 control('logV1','cpu','wrong-block',(_,r)=>{r[0].block++;});
 control('logV1','cpu','wrong-variant',(_,r)=>{r[0].variant='trial';});
 control('logV2','cpu','missing-row',(_,r)=>{r.pop();});
 control('logV2','cpu','extra-row',(_,r)=>{r.push(structuredClone(r[0]));});
 control('logExpanded','cpu','invented-baseline-equality',(_,r)=>{r[0].outcomes=[r[0].iterations,0,0];});
 control('logExpanded','cpu','wrong-matched-label',g=>{g.matched=true;});
 control('logExpanded','cpu','wrong-summary-outcomes',g=>{g.trialOutcomes=[0,0,1];});
 control('erf','cpu','zero-clock',(_,r)=>{r[0].elapsed_ns=0;});
 control('erf','cpu','fractional-clock',(_,r)=>{r[0].elapsed_ns+=.5;});
 control('erf','cpu','wrong-iteration-count',(_,r)=>{r[0].iterations++;});
 control('rootEager','cpu','invented-pilot-records',g=>{g.pilots=[];});
 control('rootEager','cpu','invented-row-timestamp',(_,r)=>{r[0].started='2026-09-08T00:00:00.000Z';});
 control('rootEager','cpu','wrong-paired-point',g=>{g.pairedMedianRatio++;});
 control('rootEager','cpu','wrong-marginal-point',g=>{g.trial_ns++;});
 control('signQuery','cpu','wrong-query-answer',(_,r)=>{r[1].outcomes=[0,0,r[1].iterations];});
 control('signNumeric','cpu','nonzero-allocation-count',(_,r)=>{r[0].alloc_calls=1;});
 control('signNumeric','cpu','nonzero-requested-bytes',(_,r)=>{r[0].allocated_bytes=1;});
 control('opaque','cpu','wrong-descriptor',(_,r)=>{r[0].case='identity-128';});
 control('polynomial','cpu','missing-preserved-pilot',g=>{g.pilots.pop();});
 control('polynomial','cpu','wrong-pilot-iterations',g=>{g.pilots[0].iterations++;});
 control('polynomial','cpu','wrong-known-status',(_,r)=>{r[0].known=0;});
 control('polynomial','cpu','wrong-degree',(_,r)=>{r[0].degree++;});
 control('polynomial','cpu','wrong-calibration',g=>{g.iterations--;});
 control('erf','alloc','wrong-fixed-allocation-iterations',g=>{g.iterations=101;});
 control('signNumeric','alloc','wrong-allocation-summary',g=>{g.sign_alloc_bytes++;});
 control('opaque','alloc','negative-request-count',(_,r)=>{r[0].alloc_calls=-1;});
 assert.throws(()=>medianOrderInterval([1,2,3]),assert.AssertionError);rejected.push('allocation/three-block-order-interval');
 const pathological=[0,0,0,1],old=historicalEstimator('run-log-bench.mjs')(pathological);
 assert.deepEqual(old,[0,0]);
 const corrected=bootstrapMedian(pathological,'early-v64/pathological-control',50000);
 assert.deepEqual(corrected.interval,[0,1]);
 return{status:'pass',rejectedMutations:rejected,pathologicalFourBlockControl:{oldWithdrawn:old,corrected:corrected.interval,
  conclusion:'The reproduced old estimator is degenerate on this corpus; successful archival reproduction is not statistical validation.'}};
}
export function summary(a){
 const groups=a.cpu.flatMap(c=>c.groups),eligible=groups.filter(g=>g.eligibility==='historical-conditional');
 const strata=Object.fromEntries(['historical-conditional','archival-source-limited','rejected-format-overlap'].map(s=>{
  const own=groups.filter(g=>g.eligibility===s);return[s,{rawRows:own.length*48,...counts(own)}];
 }));
 const relations=Object.fromEntries(['numeric-contract-checked-separately','matching-recorded-outcome','additional-answer'].map(s=>[s,counts(eligible.filter(g=>g.relation===s))]));
 assert.equal(eligible.length,298);assert.equal(strata['archival-source-limited'].comparisons,34);assert.equal(strata['rejected-format-overlap'].comparisons,1);
 const selected=[];
 for(const c of a.cpu)for(const g of c.groups)if(
  c.id==='logExpanded'&&g.name==='log-transcendental-unknown'&&g.lifecycle==='warm'||
  c.id==='erf'&&g.name==='erf-third'&&g.lifecycle==='fresh'||
  c.id==='rootEager'&&g.name==='exp-sqrt2'&&['fresh','hot-operand'].includes(g.lifecycle)||
  c.id==='opaque'&&g.name==='unresolved-128'&&g.lifecycle==='warm-pair'||
  c.id==='polynomial'&&g.name==='log-self'&&g.degree===16&&g.lifecycle==='retained'){
  const alloc=a.allocation.find(x=>x.id===c.id)?.groups.find(q=>q.name===g.name&&q.lifecycle===g.lifecycle&&q.degree===g.degree);
  selected.push({id:c.id,name:g.name,lifecycle:g.lifecycle,...(g.degree===undefined?{}:{degree:g.degree}),relation:g.relation,
   pairedRatio:g.pairedMedianRatio,bootstrap:g.bootstrap.interval,orderStatistic:g.orderStatistic.interval,
   nsPerQuery:Object.fromEntries(Object.entries(g.measurements).map(([v,m])=>[v,m.ns])),
   allocationPerQuery:alloc?Object.fromEntries(Object.entries(alloc.measurements).map(([v,m])=>[v,{requests:m.alloc_calls,requestedBytes:m.allocated_bytes}])):null});
 }
 return{status:'pass',rawRows:a.rawRows,comparisons:a.totalComparisons,allocationRows:a.allocationRows,withdrawnAllocationIntervals:a.withdrawnAllocationIntervals,
  strata,eligibleRelations:relations,campaigns:a.cpu.map(c=>({id:c.id,rawRows:c.rawRows,counts:c.counts,strata:c.strata})),
  allocations:a.allocation.map(c=>({id:c.id,rawRows:c.rawRows,groups:c.groups.length,withdrawnTimingIntervals:c.withdrawnTimingIntervals})),selected,limits:a.limits};
}
export function check(){
 const tests=controls(),a=reanalyse();assert.deepEqual(a,json('early-statistics-v64-analysis.json'));
 return{...summary(a),controls:tests,
  scope:'Full corrected recomputation and fourteen-checkpoint historical evidence chain, including preserved numerical failures and memory qualifications. No new Rust/backend/Memcheck/consumer/size/build/benchmark execution. All nine formerly remaining known-sampler scripts addressed in these dataset scopes; this is not a complete statistical or ecosystem inventory.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(check()));
