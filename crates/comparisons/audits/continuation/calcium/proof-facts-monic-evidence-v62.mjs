import {readFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {json,sha} from './point-demand-sources.mjs';
import {sourceEvidence,readRows} from './reanalyse-proof-facts-monic-v62.mjs';
export function evidence(){
 const sources=sourceEvidence(),a=json('proof-facts-monic-v62-analysis.json');assert.equal(a.status,'pass');assert.deepEqual(a.sources,sources);
 for(const c of [...a.cpu,...a.allocation]){assert.equal(sha(c.summary),c.summarySha256);assert.equal(sha(c.raw),c.rawSha256);}
 const specs=[['proof-facts-monic-historical-check','node',['verify-monic-costs.mjs']],
  ['proof-facts-monic-reanalysis','node',['reanalyse-proof-facts-monic-v62.mjs']],
  ['proof-facts-monic-check','node',['check-proof-facts-monic-v62.mjs']],
  ['proof-facts-monic-capacity','df',['-B1','/tmp','.']]];
 const gates={};for(const[tag,command,args]of specs){
  const g=json('results/'+tag+'.json');gates[tag]=g;
  for(const[k,v]of Object.entries({tag,command,args,cwd:resolve('.'),code:0,signal:null}))assert.deepEqual(g[k],v,tag+' '+k);
  assert(!g.error);assert(g.elapsedSeconds>=0&&Date.parse(g.finished)>=Date.parse(g.started));assert.equal(readFileSync('results/'+tag+'.stderr').length,0);
 }
 assert(Date.parse(gates['proof-facts-monic-historical-check'].finished)<=Date.parse(gates['proof-facts-monic-reanalysis'].started));
 assert(Date.parse(gates['proof-facts-monic-reanalysis'].finished)<=Date.parse(gates['proof-facts-monic-check'].started));
 const oldText=readFileSync('results/proof-facts-monic-historical-check.stdout','utf8'),checkText=readFileSync('results/proof-facts-monic-check.stdout','utf8');
 assert.equal(readRows('results/proof-facts-monic-historical-check.stdout').length,18);
 assert(checkText.startsWith(oldText));const rows=readRows('results/proof-facts-monic-check.stdout');assert.equal(rows.length,19);
 const checked=rows.at(-1);assert.equal(checked.status,'pass');assert.equal(checked.controls.rejectedMutations.length,20);
 assert.deepEqual(checked.eligible,{rawRows:24048,comparisons:675,changedIntervals:626,lostDirectional:39,gainedDirectional:0,reversedDirectional:0});
 assert.deepEqual(checked.rejectedHistoricalOnly,{rawRows:1728,comparisons:48,changedIntervals:41,lostDirectional:2,gainedDirectional:0,reversedDirectional:0});
 assert.equal(checked.rawRows,a.rawRows);assert.equal(checked.comparisons,a.totalComparisons);assert.equal(checked.allocationRows,a.allocationRows);
 assert.equal(a.withdrawnAllocationIntervals,441);assert.equal(checked.withdrawnAllocationIntervals,441);
 assert.deepEqual(checked.campaigns,a.cpu.map(c=>({id:c.id,rawRows:c.rawRows,inferenceEligible:c.inferenceEligible,counts:c.counts,strata:c.strata})));
 assert.deepEqual(checked.allocations,a.allocation.map(c=>({id:c.id,rawRows:c.rawRows,groups:c.groups.length,withdrawnTimingIntervals:c.withdrawnTimingIntervals})));
 const progress=readRows('results/proof-facts-monic-reanalysis.stdout'),expected=[];
 for(const c of a.cpu){
  expected.push({id:c.id,mode:'cpu',rawRows:c.rawRows,inferenceEligible:c.inferenceEligible,counts:c.counts,strata:c.strata});
  const alloc=a.allocation.find(g=>g.id===c.id);if(alloc)expected.push({id:alloc.id,mode:'alloc',rawRows:alloc.rawRows,groups:alloc.groups.length,withdrawnTimingIntervals:alloc.withdrawnTimingIntervals});
 }
 expected.push({status:'pass',rawRows:25776,comparisons:723,allocationRows:5040,withdrawnAllocationIntervals:441,output:'proof-facts-monic-v62-analysis.json'});
 assert.deepEqual(progress,expected);
 const monic=a.allocation.find(c=>c.id==='monic').groups;
 const monicMetrics=Object.fromEntries(['requests','requested_bytes','peak_delta'].map(k=>{
  const deltas=monic.map(g=>g.measurements.trial[k].max-g.measurements.baseline[k].min);
  return[k,{lower:deltas.filter(x=>x<0).length,equal:deltas.filter(x=>x===0).length,higher:deltas.filter(x=>x>0).length}];
 }));assert.deepEqual(monicMetrics,{requests:{lower:56,equal:106,higher:0},requested_bytes:{lower:56,equal:106,higher:0},peak_delta:{lower:44,equal:118,higher:0}});
 for(const g of monic)assert.deepEqual(g.measurements.trial.live_delta,g.measurements.baseline.live_delta);
 const prior=json('retained-statistics-v61-analysis.json'),all=json('point-statistics-v60-analysis.json').inventory.files.map(f=>f.path),
  addressed=[...sources.reads.map(r=>r.path),...prior.sources.reads.map(r=>r.path),'point-demand-cost-protocol.mjs','point-image-cost-statistics.mjs','point-history-statistics.mjs'],
  remainingMatches=all.filter(p=>!addressed.includes(p));assert.equal(remainingMatches.length,21);
 return{sources,gates:specs.map(([tag])=>tag),rawRows:a.rawRows,comparisons:a.totalComparisons,eligible:checked.eligible,
  rejectedHistoricalOnly:checked.rejectedHistoricalOnly,allocationRows:a.allocationRows,withdrawnAllocationIntervals:441,
  campaigns:checked.campaigns,allocations:checked.allocations,controls:checked.controls,monicMetrics,remainingMatches,
  analysisBytes:statSync('proof-facts-monic-v62-analysis.json').size,newTemporaryBytes:0,
  next:'Review the remaining 21 known sampler matches/campaign scopes and inventory any other statistical estimators; do not treat corrected accepted datasets as full historical closure. Continue point-candidate attribution/consumer/size work, power sums, supporting donor reads and the complete original ecosystem inventory.'};
}
