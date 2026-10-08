import {readFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {json,sources} from './snapshot-v43-complex-product-sources.mjs';
const read=p=>readFileSync(new URL(p,import.meta.url),'utf8');
const lines=p=>read(p).trimEnd().split('\n').map(x=>JSON.parse(x));
const key=r=>JSON.stringify([r.bits,r.family,r.scale,r.mask,r.route,r.lifecycle]);
const median=a=>{const v=[...a].sort((x,y)=>x-y);return(v[Math.floor((v.length-1)/2)]+v[Math.floor(v.length/2)])/2;};
export function checkComplexProduct() {
 assert.deepEqual(sources(),json('complex-product-binaries.json').sourceMap);
 const expected={status:'pass',fixtures:6912,components:82944,inputChecks:27648,zeroComponents:4608};
 const native=read('results/complex-product-check-baseline.stdout'),memory={};
 const widths=[32,64,127,128,129,191,192,193,255,256,257,511,512,513,1024,2048];
 for(const v of ['baseline','candidate']) {
  for(const kind of ['check','memcheck']) {
   const tag='complex-product-'+kind+'-'+v;
   assert.equal(json('results/'+tag+'.json').code,0);
   assert.equal(read('results/'+tag+'.stdout'),native);
   const rows=lines('results/'+tag+'.stdout');assert.equal(rows.length,17);assert.deepEqual(rows.at(-1),expected);
   for(let i=0;i<16;i++)assert.deepEqual(rows[i],{bits:widths[i],fixtures:432*(i+1),components:5184*(i+1),inputChecks:1728*(i+1),zeroComponents:288*(i+1)});
  }
  assert.equal(read('results/complex-product-check-'+v+'.stderr'),'');
  const mem=read('results/complex-product-memcheck-'+v+'.stderr');
  assert.match(mem,/ERROR SUMMARY: 0 errors from 0 contexts \(suppressed: 0 from 0\)/);
  for(const kind of ['definitely','indirectly','possibly'])assert.match(mem,new RegExp(kind+' lost: 0 bytes in 0 blocks'));
  assert.match(mem,/still reachable: 13,832 bytes in 136 blocks/);
  const values=mem.match(/total heap usage: ([\d,]+) allocs, ([\d,]+) frees, ([\d,]+) bytes allocated/).slice(1).map(x=>Number(x.replaceAll(',','')));
  memory[v]={allocations:values[0],frees:values[1],cumulativeBytesIncludingOracle:values[2],reachableBytes:13832,reachableBlocks:136};
 }
 const trace={};
 for(const v of ['baseline','candidate']) {
  const tag='complex-product-trace-'+v;assert.equal(json('results/'+tag+'.json').code,0);
  assert.equal(read('results/'+tag+'.stderr'),'');trace[v]=lines('results/'+tag+'.stdout');assert.equal(trace[v].length,5832);
  assert.equal(new Set(trace[v].map(r=>JSON.stringify([r.bits,r.family,r.scale,r.mask,r.route,r.stage]))).size,5832);
  for(const r of trace[v]) {
   assert([0,1].includes(r.candidateSelections));
   if(v==='baseline'||r.bits<=128||['unbalanced','zero'].includes(r.family))assert.equal(r.candidateSelections,0);
   if(v==='candidate'&&r.bits>128&&!['unbalanced','mixed-scale','zero'].includes(r.family))
    assert.equal(r.candidateSelections,r.route==='lattice'?r.latticeFused:1);
   if(r.route!=='lattice'){assert.equal(r.latticeFused,0);assert.equal(r.latticeReuse,0);}
  }
 }
 for(let i=0;i<5832;i++) {
  const {candidateSelections:a,...x}=trace.baseline[i],{candidateSelections:b,...y}=trace.candidate[i];
  assert.deepEqual(x,y);assert.equal(a,0);assert(b>=0);
 }
 const cost={},scenarios=[['dense','integer',0],['dense','dyadic',15],['dense','odd',6],['near-cancel','dyadic',0],
  ['sum-zero','integer',2],['unbalanced','integer',0],['mixed-scale','odd',0],['zero','integer',0]];
 const membership=[];
 for(const bits of [64,192,256,1024,4096,16384])for(const[f,s,m]of scenarios)for(const route of ['product','lattice'])for(const lifecycle of ['fresh','reused'])membership.push(JSON.stringify([bits,f,s,m,route,lifecycle]));
 for(const mode of ['cpu','allocation']) {
  const summary=json('complex-product-'+mode+'-summary.json'),raw=lines('results/complex-product-'+mode+'.jsonl');
  assert.equal(summary.mode,mode);assert.equal(summary.cpu,6);assert.deepEqual(summary.summaries.map(key),membership);
  const observations=mode==='cpu'?48:6;assert.equal(raw.length,192*observations);
  let seed=31337;const random=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
  for(let i=0;i<192;i++) {
   const s=summary.summaries[i],rows=raw.slice(i*observations,(i+1)*observations);
   assert.equal(s.observations,observations);
   for(let j=0;j<rows.length;j++) {
    const r=rows[j],block=Math.floor(j/(mode==='cpu'?4:2));
    assert.equal(key(r),key(s));assert.equal(r.block,block);assert.equal(r.iterations,s.iterations);assert(r.ns>0);assert.equal(r.preflight,true);
    const order=mode==='cpu'?(block%2?['candidate','baseline','baseline','candidate']:['baseline','candidate','candidate','baseline']):['baseline','candidate'];
    assert.equal(r.variant,order[j%order.length]);assert.equal(r.allocation===null,mode==='cpu');
   }
   if(mode==='cpu') {
    const ratios=Array.from({length:12},(_,b)=>{
     const clock=v=>median(rows.filter(r=>r.block===b&&r.variant===v).map(r=>r.ns));return clock('candidate')/clock('baseline');
    });
    const bootstrap=Array.from({length:5000},()=>median(ratios.map(()=>ratios[random(12)]))).sort((a,b)=>a-b);
    assert.equal(s.pairedMedianRatio,median(ratios));assert.deepEqual(s.pairedMedianBootstrap95,[bootstrap[125],bootstrap[4875]]);
    for(const v of ['baseline','candidate'])assert.equal(s.nsPerQuery[v],median(rows.filter(r=>r.variant===v).map(r=>r.ns/r.iterations)));
   } else for(const v of ['baseline','candidate']) {
    assert.deepEqual(s.measurements[v],['requests','requestedBytes','liveDelta','peakDelta'].map((name,i)=>{
     const a=rows.filter(r=>r.variant===v).map(r=>r.allocation[i]);return{name,min:Math.min(...a),max:Math.max(...a)};
    }));
   }
  }
  const groups=summary.summaries;
  cost[mode]={groups:192,observations:raw.length,queries:raw.reduce((n,r)=>n+r.iterations,0)};
  if(mode==='cpu')Object.assign(cost.cpu,{
   minRatio:Math.min(...groups.map(s=>s.pairedMedianRatio)),maxRatio:Math.max(...groups.map(s=>s.pairedMedianRatio)),
   below:groups.filter(s=>s.pairedMedianBootstrap95[1]<1).length,
   above:groups.filter(s=>s.pairedMedianBootstrap95[0]>1).length,
   overlap:groups.filter(s=>s.pairedMedianBootstrap95[0]<=1&&s.pairedMedianBootstrap95[1]>=1).length,
  });
  else cost.allocation.metrics=[0,1,2,3].map(i=>({name:groups[0].measurements.baseline[i].name,
   lower:groups.filter(s=>s.measurements.candidate[i].max<s.measurements.baseline[i].min).length,
   higher:groups.filter(s=>s.measurements.candidate[i].min>s.measurements.baseline[i].max).length,
   same:groups.filter(s=>JSON.stringify(s.measurements.candidate[i])===JSON.stringify(s.measurements.baseline[i])).length,
  }));
 }
 for(const v of ['baseline','candidate'])for(const kind of ['check','memcheck','trace'])
  assert(Date.parse(json('results/complex-product-'+kind+'-'+v+'.json').finished)<Date.parse(json('results/complex-product-cpu.json').started));
 assert(Date.parse(json('results/complex-product-allocation.json').finished)<Date.parse(json('results/complex-product-cpu.json').started));
 return{correctness:expected,memory,traceRowsPerVariant:5832,candidateSelections:trace.candidate.reduce((n,r)=>n+r.candidateSelections,0),cost};
}
if(process.argv.includes('--complex-product-summary'))console.log(JSON.stringify(checkComplexProduct()));
