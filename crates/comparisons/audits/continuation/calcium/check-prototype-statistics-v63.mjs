// Archived estimator reproduction is integrity evidence, not valid inference.
import './verify-rank-costs.mjs';
import {readFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {json} from './point-demand-sources.mjs';
import {checkComplexProduct} from './snapshot-v43-check-complex-product.mjs';
import {checkComplexProduct as checkComplexV2} from './snapshot-v43-check-complex-product-v2.mjs';
import {checkSignFilter} from './check-sign-filter.mjs';
import {checkSignFilterMask} from './check-sign-filter-mask.mjs';
import {verifyInventory} from './prototype-statistics-inventory-v63.mjs';
import {campaigns,paths,readRows,reconstruct,reanalyse,counts} from './reanalyse-prototype-statistics-v63.mjs';
export function controls(){
 const rejected=[];
 function control(id,mode,name,mutate,index=0){
  const c=campaigns.find(c=>c.id===id),p=paths(c,mode),m=structuredClone(json(p.summary)),g=m.summaries[index],n=mode==='cpu'?48:6,
   rows=readRows(p.raw).slice(index*n,(index+1)*n);
  reconstruct(c,g,rows,mode,m);mutate(g,rows,m);
  assert.throws(()=>reconstruct(c,g,rows,mode,m),assert.AssertionError,id+'/'+name);rejected.push(id+'/'+name);
 }
 control('complexV1','cpu','wrong-block',(_,r)=>{r[0].block++;});
 control('complexV1','cpu','wrong-variant',(_,r)=>{r[0].variant='candidate';});
 control('complexV1','cpu','false-numeric-preflight',(_,r)=>{r[0].preflight=false;});
 control('complexV1','cpu','zero-clock',(_,r)=>{r[0].ns=0;});
 control('complexV1','cpu','fractional-clock',(_,r)=>{r[0].ns+=.5;});
 control('complexV1','cpu','CPU-allocation-instrumented',(_,r)=>{r[0].allocation=[0,0,0,0];});
 control('complexV2','cpu','missing-row',(_,r)=>{r.pop();});
 control('complexV2','cpu','extra-row',(_,r)=>{r.push(structuredClone(r[0]));});
 control('complexV2','cpu','changed-paired-estimate',g=>{g.pairedMedianRatio++;});
 control('complexV2','cpu','changed-marginal-median',g=>{g.nsPerQuery.candidate++;});
 control('complexV2','cpu','missing-pilot',g=>{g.pilots.pop();});
 control('complexV2','cpu','wrong-pilot-iterations',g=>{g.pilots[0].iterations++;});
 control('rank','cpu','wrong-known-count',(_,r)=>{r[0].known=0;});
 control('rank','cpu','invented-baseline-answer',(_,r)=>{r[0].known=r[0].iterations;},8);
 control('rank','cpu','wrong-descriptor',(_,r)=>{r[0].width++;});
 control('rank','cpu','wrong-calibration',g=>{g.iterations++;});
 control('rank','cpu','nonzero-CPU-request-count',(_,r)=>{r[0].requests++;});
 control('signSummary','cpu','wrong-sign-value',(_,r)=>{r[0].outcome=r[0].outcome.replace('Positive','Negative');});
 control('signSummary','cpu','wrong-certainty',(_,r)=>{r[0].outcome=r[0].outcome.replace('certainty: Exact','certainty: Approximate');});
 control('signSummary','cpu','wrong-decision-stage',(_,r)=>{r[0].outcome=r[0].outcome.replace('stage: Exact','stage: Refined');});
 control('signMask','cpu','observation-before-pilots',(_,r,m)=>{r[0].started=m.started;});
 control('signMask','cpu','observation-outside-campaign',(_,r,m)=>{r[0].finished=new Date(Date.parse(m.finished)+1).toISOString();});
 control('signMask','cpu','reversed-clock-envelope',(_,r)=>{r[0].finished=new Date(Date.parse(r[0].started)-1).toISOString();});
 control('signMask','cpu','wrong-summary-outcome',g=>{g.outcome='Unknown';});
 control('complexV1','allocation','wrong-allocation-label',g=>{g.measurements.baseline[0].name='bytes';});
 control('complexV2','allocation','nondeterministic-request-count',(_,r)=>{r[0].allocation[0]++;});
 control('rank','allocation','unreported-live-retention',(_,r)=>{r[0].live_delta++;});
 control('signSummary','allocation','changed-allocation-summary',g=>{g.measurements.baseline.requested_bytes.min++;});
 control('signMask','allocation','invented-allocation-timing-interval',g=>{g.pairedMedianBootstrap95=[1,1];});
 const inventory=structuredClone(json('prototype-statistics-v63-inventory-corrected.json'));
 inventory.files.push(structuredClone(inventory.files[0]));
 assert.throws(()=>verifyInventory(inventory),assert.AssertionError);rejected.push('inventory/duplicated-membership');
 return{status:'pass',rejectedMutations:rejected};
}
export function summary(a){
 const sum=k=>a.cpu.reduce((n,c)=>n+c.counts[k],0);
 const selected=g=>g.bits>=256&&!['unbalanced','mixed-scale','zero'].includes(g.family)&&!(g.route==='lattice'&&g.lifecycle==='reused');
 const v2=a.cpu.find(c=>c.id==='complexV2').groups;
 const rank=a.cpu.find(c=>c.id==='rank').groups.find(g=>g.case===4&&g.width===32&&g.lifecycle==='retained_analysis'),
  alloc=a.allocation.find(c=>c.id==='rank').groups.find(g=>g.case===4&&g.width===32&&g.lifecycle==='retained_analysis');
 assert.equal(v2.filter(selected).length,60);
 return{status:'pass',rawRows:a.rawRows,comparisons:a.totalComparisons,allocationRows:a.allocationRows,
  aggregate:Object.fromEntries(['changedIntervals','lostDirectional','gainedDirectional','reversedDirectional'].map(k=>[k,sum(k)])),
  campaigns:a.cpu.map(c=>({id:c.id,rawRows:c.rawRows,counts:c.counts,strata:c.strata})),
  allocations:a.allocation.map(c=>({id:c.id,rawRows:c.rawRows,groups:c.groups.length,deltas:c.deltas})),
  complexV2HistoricalDispatchStrata:{selected:counts(v2.filter(selected)),bypass:counts(v2.filter(g=>!selected(g)))},
  unresolvedRankWidth32Retained:{pairedRatio:rank.pairedMedianRatio,bootstrap:rank.bootstrap.interval,orderStatistic:rank.orderStatistic.interval,
   nsPerQuery:rank.nsPerQuery,requestedBytesPerQuery:Object.fromEntries(['baseline','candidate'].map(v=>[v,alloc.measurements[v].requested_bytes.min/alloc.iterations]))},
  limits:a.limits};
}
export function check(){
 const historical={};for(const[stem,fn]of [['complex-product',checkComplexProduct],['complex-product-v2',checkComplexV2],
  ['sign-filter',checkSignFilter],['sign-filter-mask',checkSignFilterMask]]){
  const checks=fn();assert.deepEqual(checks,json(stem+'-experiment.json').checks);historical[stem]=checks;
 }
 assert.equal(readFileSync('results/prototype-statistics-ring-oracle-qualified.stdout','utf8'),readFileSync('results/sign-filter-ring-oracle.stdout','utf8'));
 const tests=controls(),a=reanalyse();assert.deepEqual(a,json('prototype-statistics-v63-analysis.json'));
 const historicalSummary={complex:Object.fromEntries(['complex-product','complex-product-v2'].map(k=>[k,
  {correctness:historical[k].correctness,memory:historical[k].memory,traceRowsPerVariant:historical[k].traceRowsPerVariant,candidateSelections:historical[k].candidateSelections}])),
  sign:Object.fromEntries(['sign-filter','sign-filter-mask'].map(k=>[k,{testsPerProfile:historical[k].tests.perProfile,
   publicTraces:historical[k].publicTraces,independentRingSigns:historical[k].independentRingSigns,benchBinaryDeltas:historical[k].benchBinaryDeltas}]))};
 return{...summary(a),controls:tests,historical:historicalSummary,
  scope:'Full corrected recomputation, historical 23-checkpoint rank evidence chain plus four archived cost/correctness checkers, and an independently recomputed 24-case exact rational/Machin sign oracle. No fresh Rust tests, numerical backend execution, memory test, build, or benchmark. Archived bootstrap reproduction does not qualify the withdrawn inference.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(check()));
