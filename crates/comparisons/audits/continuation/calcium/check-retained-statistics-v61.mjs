import assert from 'node:assert/strict';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import {json} from './point-demand-sources.mjs';
import {medianOrderInterval} from './paired-statistics-v60.mjs';
import {campaigns,readRows,reconstructGroup,reanalyseRetainedStatistics} from './reanalyse-retained-statistics-v61.mjs';
import {checkEPlan} from './check-e-plan.mjs';
import {checkEQualified} from './check-e-qualified.mjs';
import {costs as derivativeCosts} from './check-derivative-costs.mjs';
import {qualificationCosts} from './check-derivative-qualification-costs.mjs';

const digest=x=>createHash('sha256').update(JSON.stringify(x)).digest('hex');
export function additionalControls(){
 // Independently convolve n fair Bernoulli trials using BigInt counts. This
 // does not reuse the binomial recurrence in the interval implementation.
 const orderChecks=[];
 for(const n of [6,12,40]){
  let counts=[1n];for(let trial=0;trial<n;trial++){
   const next=Array(counts.length+1).fill(0n);
   for(let j=0;j<counts.length;j++){next[j]+=counts[j];next[j+1]+=counts[j];}counts=next;
  }
  const total=counts.reduce((a,b)=>a+b,0n);assert.equal(total,1n<<BigInt(n));let expected;
  for(let k=1;k<=Math.floor((n+1)/2);k++){
   const covered=counts.slice(k,n-k+1).reduce((a,b)=>a+b,0n);
   if(covered*100n>=95n*total)expected={interval:[k,n-k+1],lowerRank:k,upperRank:n-k+1,
    coverageNumerator:String(covered),coverageDenominator:String(total),nominalCoverage:Number(covered)/Number(total)};
  }
  const actual=medianOrderInterval(Array.from({length:n},(_,i)=>i+1));assert.deepEqual(actual,expected);orderChecks.push({n,...actual});
 }
 const c=campaigns[0],m=json(c.summary),g=m.summaries[0],rows=readRows(c.raw).slice(0,c.blocks*4);
 reconstructGroup(c,g,rows,m);
 const mutations=[
  ['block',(_,r)=>{r[0].block++;}],['variant',(_,r)=>{r[0].variant='candidate';}],
  ['iterations',(_,r)=>{r[0].iterations++;}],['descriptor',(_,r)=>{r[0].p++;}],
  ['clock-zero',(_,r)=>{r[0].ns=0;}],['clock-not-integer',(_,r)=>{r[0].ns+=.5;}],
  ['fingerprint',(_,r)=>{r[0].fingerprint='not-original';}],
  ['timestamp',(_,r)=>{r[0].finished='1970-01-01T00:00:00.000Z';}],
  ['marginal-median',s=>{s.nsPerQuery.baseline++;}],['paired-median',s=>{s.pairedMedianRatio++;}],
  ['summary-count',s=>{s.observations--;}],['missing-row',(_,r)=>{r.pop();}],
  ['extra-row',(_,r)=>{r.push(structuredClone(r[0]));}],['pilot-count',s=>{s.pilots[0].iterations++;}],
 ];
 for(const [name,mutate]of mutations){
  const summary=structuredClone(g),observed=structuredClone(rows);mutate(summary,observed);
  assert.throws(()=>reconstructGroup(c,summary,observed,m),assert.AssertionError,name);
 }
 return{status:'pass',orderChecks,rejectedMutations:mutations.map(([name])=>name),
  scope:'Independent binomial-convolution controls for ranks at n=6/12/40 and fail-closed reconstruction checks. Not a proof that measured timing blocks are IID.'};
}

export function checkRetainedStatistics(){
 const controls=additionalControls();
 // The old checkers reproduce the archived defective intervals ONLY to bind
 // historical records. Their confidence classifications are not endorsed.
 // Their raw pairing, independent numerical checks, tests and allocation
 // parsing provide evidence separate from the new inference calculation.
 const plan=checkEPlan(),qualified=checkEQualified(),derivative=derivativeCosts(),endpoint=qualificationCosts();
 const historicalChecks=[['e-plan-experiment.json','checks',plan],['e-qualified-experiment.json','checks',qualified],
  ['derivative-demand-experiment.json','costs',derivative],['derivative-qualification-experiment.json','costs',endpoint]];
 for(const [p,key,value]of historicalChecks)assert.deepEqual(json(p)[key],value,p);
 const actual=reanalyseRetainedStatistics();assert.deepEqual(actual,json('retained-statistics-v61-analysis.json'));
 return{status:'pass',rawRows:actual.rawRows,comparisons:actual.comparisons,controls,
  historicalChecks:historicalChecks.map(([manifest,key,value])=>({manifest,key,sha256:digest(value)})),
  recordedEvidence:{scope:'Rechecks existing outputs and exact oracles, not new Rust tests/benchmarks or memory executions.',
   planner:{planChecks:plan.exactBigIntPlanChecks,enclosures:plan.exactBigIntEnclosureChecks,
    allocationRows:plan.costs.allocation.observations,allocationGroups:plan.costs.allocation.groups,memory:plan.memory},
   qualifiedPlanner:{rust:qualified.rust,wasm:qualified.wasm,appSizes:qualified.appSizes},
   derivative:{allocationRows:derivative.allocationRows,allocation:derivative.allocation,
    nonzeroLiveGroups:derivative.nonzeroLiveGroups,maxSharedLiveDelta:derivative.maxSharedLiveDelta},
   endpoint:{retentionRows:endpoint.retentionRows,plateauGroups:endpoint.plateauGroups,maxLiveAt32:endpoint.maxLiveAt32,
    allocationRows:endpoint.allocationRows,allocationEqualGroups:endpoint.endpointAllocationEqualGroups,
    allocationNonzeroLiveGroups:endpoint.endpointAllocationNonzeroLiveGroups}},
  campaigns:actual.campaigns.map(c=>({id:c.id,rawRows:c.rawRows,counts:c.counts})),
  limits:actual.limits};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(checkRetainedStatistics()));
