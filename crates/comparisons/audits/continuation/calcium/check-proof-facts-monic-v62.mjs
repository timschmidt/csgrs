// Reproduce archived evidence, including the withdrawn old estimator, without
// treating its confidence classifications as valid inference.
import './verify-monic-costs.mjs';
import assert from 'node:assert/strict';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import {json} from './point-demand-sources.mjs';
import {medianOrderInterval} from './paired-statistics-v60.mjs';
import {campaigns,paths,readRows,reconstruct,reanalyse} from './reanalyse-proof-facts-monic-v62.mjs';

export function controls(){
 assert.throws(()=>medianOrderInterval([1,2,3]),assert.AssertionError);
 // Direct enumeration: three fair binary signs have eight equiprobable
 // sequences; the bounded min/max interval misses the two unanimous signs.
 let covered=0;for(let bits=0;bits<8;bits++)if(bits!==0&&bits!==7)covered++;
 assert.equal(covered,6);assert(covered*100<95*8);
 const rejected=[];
 function control(id,name,mutate){
  const c=campaigns.find(c=>c.id===id),p=paths(c,'cpu'),g=structuredClone(json(p.summary).summaries[0]),rows=readRows(p.raw).slice(0,12*c.variants.length*2);
  reconstruct(c,g,rows,'cpu');mutate(g,rows);
  assert.throws(()=>reconstruct(c,g,rows,'cpu'),assert.AssertionError,id+'/'+name);rejected.push(id+'/'+name);
 }
 control('proofQuery','wrong-block',(_,r)=>{r[0].block++;});
 control('proofQuery','wrong-variant',(_,r)=>{r[0].variant='proof';});
 control('proofQuery','invented-baseline-equality',(_,r)=>{r[0].outcomes=[r[0].iterations,0,0];});
 control('proofNumeric','missing-row',(_,r)=>{r.pop();});
 control('proofNumeric','extra-row',(_,r)=>{r.push(structuredClone(r[0]));});
 control('proofNumeric','zero-clock',(_,r)=>{r[0].elapsed_ns=0;});
 control('proofNumeric','fractional-clock',(_,r)=>{r[0].elapsed_ns+=.5;});
 control('proofNumeric','nonzero-cpu-allocation',(_,r)=>{r[0].allocated_bytes=1;});
 control('reuseConfirm','changed-paired-median',g=>{g.comparisons.baseline.pairedMedianRatio++;});
 control('reuseNumeric','missing-recorded-pilots',g=>{delete g.pilots;});
 control('reuseNumeric','changed-marginal',g=>{g.measurements.reuse.elapsed_ns++;});
 control('reuseFirstTouch','wrong-worker-count',(_,r)=>{r[0].workers++;});
 control('reuseFirstTouch','wrong-warm-count',(_,r)=>{r[0].warm_queries_per_worker--;});
 control('reuseFirstTouch','impossible-lifecycle-clock',(_,r)=>{r[0].lifecycle_ns=r[0].first_ns;});
 control('reuseFirstTouch','wrong-warm-normalization',g=>{g.measurements.reuse.warm_ns*=16;});
 control('polynomialFacts','wrong-known-count',(_,r)=>{r[0].known=0;});
 control('polynomialFacts','wrong-descriptor',(_,r)=>{r[0].degree++;});
 control('monic','wrong-result-degree',(_,r)=>{r[0].degree++;});
 control('monic','wrong-mode',(_,r)=>{r[0].mode='allocation';});
 control('monic','wrong-observation-count',g=>{g.observations--;});
 return{status:'pass',rejectedMutations:rejected,threeBlockMedianCoverage:{covered:6,total:8,coverage:0.75,
  conclusion:'No finite observed min/max median interval achieves 95% coverage at n=3 under continuous IID sampling; no CPU inference from instrumented allocation clocks.'}};
}
export function summary(a){
 const eligible=a.cpu.filter(c=>c.inferenceEligible),rejected=a.cpu.filter(c=>!c.inferenceEligible);
 const sum=(cs,key)=>cs.reduce((n,c)=>n+c.counts[key],0);
 const aggregate=cs=>({rawRows:cs.reduce((n,c)=>n+c.rawRows,0),comparisons:sum(cs,'comparisons'),
  changedIntervals:sum(cs,'changedIntervals'),lostDirectional:sum(cs,'lostDirectional'),gainedDirectional:sum(cs,'gainedDirectional'),reversedDirectional:sum(cs,'reversedDirectional')});
 const accepted=aggregate(eligible);assert.equal(accepted.rawRows,24048);assert.equal(accepted.comparisons,675);
 return{status:'pass',rawRows:a.rawRows,comparisons:a.totalComparisons,eligible:accepted,rejectedHistoricalOnly:aggregate(rejected),
  allocationRows:a.allocationRows,withdrawnAllocationIntervals:a.withdrawnAllocationIntervals,
  campaigns:a.cpu.map(c=>({id:c.id,rawRows:c.rawRows,inferenceEligible:c.inferenceEligible,counts:c.counts,strata:c.strata})),
  allocations:a.allocation.map(c=>({id:c.id,rawRows:c.rawRows,groups:c.groups.length,withdrawnTimingIntervals:c.withdrawnTimingIntervals})),
  limits:a.limits};
}
export function check(){
 const tests=controls(),actual=reanalyse();assert.deepEqual(actual,json('proof-facts-monic-v62-analysis.json'));
 return{...summary(actual),controls:tests,scope:'Full corrected recomputation plus historical 18-checkpoint evidence rechecking. No fresh numerical execution, Rust tests, memory checks or benchmarks. Archived inference remains withdrawn except for the explicitly corrected eligible CPU campaigns.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(check()));
