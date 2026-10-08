import {readFileSync} from 'node:fs';
import {resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {sha,json,checkPointImageSources} from './point-image-sources.mjs';
import {summarizePointImageCosts} from './point-image-cost-statistics.mjs';
const lines=p=>{const s=readFileSync(p,'utf8').trim();return s?s.split('\n').map(s=>JSON.parse(s)):[];};
export function checkPointImageCosts(){
 checkPointImageSources();const binding=json('point-image-guard-binaries.json');
 for(const[p,h]of Object.entries(binding.guardSources))assert.equal(sha('point-image-guard/'+p),h,p);
 const summaries={};
 for(const mode of ['cpu','allocation']){
  const s=json('point-image-cost-'+mode+'-summary.json'),r=lines('results/point-image-cost-'+mode+'.jsonl'),p=lines('results/point-image-cost-'+mode+'-pilots.jsonl');
  assert.equal(s.mode,mode);assert.equal(s.cpu,6);
  assert.deepEqual(s.binaries,{baseline:binding.baselineBinaries['baseline-'+mode],candidate:binding.binaries[mode]});
  for(const b of Object.values(s.binaries))assert.equal(sha(b.path),b.sha256);
  assert.deepEqual(s.summaries,summarizePointImageCosts(mode,r,p));
  const expected=Object.fromEntries(['baseline','candidate'].map(v=>[v,new Map(lines('results/'+(v==='baseline'?'power-sums-public-baseline':'point-image-guard-public')+'.stdout')
   .filter(r=>r.type==='cost-case').map(r=>[r.case,r.report]))]));
  for(const row of [...r,...p]){
   assert.equal(row.mode,mode);assert(['baseline','candidate'].includes(row.variant));assert(['retained','fresh'].includes(row.lifecycle));
   assert(row.iterations>0&&row.elapsed_ns>0);assert.deepEqual(row.expected,expected[row.variant].get(row.case));
   assert.equal(row.checksum,row.iterations*(row.expected.root?.polynomial.length??1));
   assert(Date.parse(row.started)>=Date.parse(s.started));assert(Date.parse(row.finished)>=Date.parse(row.started));assert(Date.parse(row.finished)<=Date.parse(s.finished));
   if(mode==='cpu')for(const k of ['requests','requested_bytes','live_delta','peak_delta'])assert.equal(row[k],0);
  }
  // Verify the actual per-group counterbalanced observation order, not only totals.
  for(let which=0;which<40;which++)for(const lifecycle of ['retained','fresh'])for(let block=0;block<(mode==='cpu'?12:3);block++){
   const order=mode==='cpu'?(block%2?['candidate','baseline','baseline','candidate']:['baseline','candidate','candidate','baseline']):['baseline','candidate'];
   assert.deepEqual(r.filter(row=>row.case===which&&row.lifecycle===lifecycle&&row.block===block).map(row=>row.variant),order);
  }
  assert.equal(json('results/point-image-cost-'+mode+'.json').code,0);summaries[mode]=s;
 }
 assert(Date.parse(json('results/point-image-consumer-debug.json').finished)<=Date.parse(summaries.cpu.started));
 assert(Date.parse(summaries.cpu.finished)<=Date.parse(summaries.allocation.started));
 const same=summaries.cpu.summaries.filter(g=>g.sameFullResult),changed=summaries.cpu.summaries.filter(g=>!g.sameFullResult);
 const compact=g=>({case:g.case,lifecycle:g.lifecycle,statuses:g.statuses,ratio:g.pairedMedianRatio,ci:g.pairedMedianBootstrap95,ns:g.nsPerQuery});
 const allocation=summaries.allocation.summaries.map(g=>{
  const delta=Object.fromEntries(['requests','requested_bytes','live_delta','peak_delta'].map(k=>[k,{
   min:g.measurements.candidate[k].min-g.measurements.baseline[k].max,
   max:g.measurements.candidate[k].max-g.measurements.baseline[k].min}]));
  return{case:g.case,lifecycle:g.lifecycle,sameFullResult:g.sameFullResult,iterations:g.iterations,delta};
 });
 const counts={};for(const key of ['requests','requested_bytes','live_delta','peak_delta']){
  counts[key]={sameResultLower:0,sameResultEqual:0,sameResultHigher:0,sameResultVariable:0,changedResultLower:0,changedResultEqual:0,changedResultHigher:0,changedResultVariable:0};
  for(const g of allocation){const d=g.delta[key],prefix=g.sameFullResult?'sameResult':'changedResult';
   const suffix=d.max<0?'Lower':d.min>0?'Higher':d.min===0&&d.max===0?'Equal':'Variable';counts[key][prefix+suffix]++;}
 }
 return{status:'pass',cpuObservations:3840,cpuPilots:160,allocationObservations:480,groups:80,
  sameResultGroups:same.length,changedResultGroups:changed.length,
  sameResultRatioRange:[Math.min(...same.map(g=>g.pairedMedianRatio)),Math.max(...same.map(g=>g.pairedMedianRatio))],
  sameResultCiWhollyBelowOne:same.filter(g=>g.pairedMedianBootstrap95[1]<1).length,
  sameResultCiWhollyAboveOne:same.filter(g=>g.pairedMedianBootstrap95[0]>1).length,
  fastestSame:[...same].sort((a,b)=>a.pairedMedianRatio-b.pairedMedianRatio).slice(0,5).map(compact),
  slowestSame:[...same].sort((a,b)=>b.pairedMedianRatio-a.pairedMedianRatio).slice(0,5).map(compact),
  changed:changed.map(compact),allocationCounts:counts,allocationChanges:allocation.filter(g=>Object.values(g.delta).some(d=>d.min!==0||d.max!==0)),
  limits:'Per-group paired bootstrap intervals are not adjusted for multiple comparisons. Changed outcomes perform additional certified work and are not equal-work speedups. Corpus has rational endpoints and preconditioned process-local caches; nonrational/Unknown endpoint costs, other platforms and representative application sizes remain unqualified.'};
}
if(process.argv[1]&&resolve(process.argv[1])===fileURLToPath(import.meta.url))console.log(JSON.stringify(checkPointImageCosts()));
