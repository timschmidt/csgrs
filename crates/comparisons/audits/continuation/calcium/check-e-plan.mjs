import {readFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {json} from './e-plan-sources.mjs';
const read=p=>readFileSync(p,'utf8'),rows=p=>read(p).trim().split('\n').map(s=>JSON.parse(s));
const median=a=>{const v=[...a].sort((a,b)=>a-b);return(v[(v.length-1)>>1]+v[v.length>>1])/2;};
const bitLength=n=>n.toString(2).length;
export function checkEPlan() {
 const requests=new Set(Array.from({length:4113},(_,i)=>i-4096));
 const selected=new Set([20,21,22,31,32,33,63,64,65,127,128,129,255,256,257,
  511,512,513,1023,1024,1025,2047,2048,2049,4095,4096,4097,8191,8192]);
 let q=1n;
 for(let n=1;n<=8192;n++) {q*=BigInt(n);if(selected.has(n))for(const delta of [-1,0,1]) {
  const needed=bitLength(q)+delta;if(needed>4)requests.add(4-needed);
 }}
 for(const b of [16384,32768,65536,120700,262144])requests.add(-b);
 const planPositions=[...requests].sort((a,b)=>b-a),expected=new Map();q=1n;let k=1;
 for(const p of planPositions) {const needed=p<0?-p+4:4;while(bitLength(q)<=needed) {k++;q*=BigInt(k);}expected.set(p,k-1);}
 assert.equal(planPositions.length,4151);assert.equal(k,20367);
 const numericPositions=[...new Set([...Array.from({length:73},(_,i)=>i-64),
  -127,-128,-129,-255,-256,-257,-511,-512,-513,-1023,-1024,-1025,-4095,-4096,-4097,-16384,-32768,-65536,-120700,-262144])].sort((a,b)=>b-a);
 assert.equal(numericPositions.length,93);
 const numericShape=[...numericPositions.flatMap(p=>[['kernel',p],['public-refine',p]]),
  ...[...numericPositions].reverse().map(p=>['public-coarsen',p])];
 const stateShape=[['cancel-recovery',-4096],...[-8192,-4096,-128,0].map(p=>['serde',p]),
  ...[-16384,-512,0].map(p=>['exp-one',p]),...[4096,8192,16384,32768].flatMap(b=>[-b,-b/2,-64].map(p=>['thread',p])),
  ['post-thread-refine',-65536]];assert.equal(stateShape.length,21);
 // Independent exact rational enclosure: start with 1/0!, update the full
 // partial numerator and factorial, then bound every omitted term geometrically.
 // This uses JavaScript BigInt, not Rust num, GMP, MPFR or the binary-split tree.
 let numerator=1n,denominator=1n,n=0;
 while(bitLength(denominator)<=262144+16) {n++;denominator*=BigInt(n);numerator=numerator*BigInt(n)+1n;}
 const tailDenominator=denominator*BigInt(n+1),tailNumerator=numerator*BigInt(n+1)+2n;
 let rationalChecks=0,planChecks=0,matchedRows=0;const memory={};
 for(const v of ['baseline','candidate']) {
  const plans=rows('results/e-plan-native-'+v+'-plans.stdout'),summary=plans.pop();
  assert.deepEqual(summary,{kind:'plan-summary',requests:4151,extraTermRequests:0,invariantSteps:20367});
  assert.deepEqual(plans.map(r=>r.p),planPositions);
  for(const r of plans) {assert.deepEqual(r,{kind:'plan',p:r.p,actual:expected.get(r.p),expected:expected.get(r.p)});planChecks++;}
  for(const kind of ['numeric','state']) {
   const observed=rows('results/e-plan-native-'+v+'-'+kind+'.stdout'),summary=observed.pop();
   assert.deepEqual(summary,kind==='numeric'?{kind:'numeric-summary',requests:93,enclosures:279}:{kind:'state-summary',enclosures:21,threads:4});
   assert.deepEqual(observed.map(r=>[r.kind,r.p]),kind==='numeric'?numericShape:stateShape);
   for(const r of observed) {
    assert.match(r.integer,/^[0-9a-f]+$/);const a=BigInt('0x'+r.integer);
    if(r.p<0) {
     assert((a-1n)*denominator<(numerator<<BigInt(-r.p)));
     assert((a+1n)*tailDenominator>(tailNumerator<<BigInt(-r.p)));
    } else {
     assert(((a-1n)*denominator<<BigInt(r.p))<numerator);
     assert(((a+1n)*tailDenominator<<BigInt(r.p))>tailNumerator);
    }
    rationalChecks++;
   }
  }
  for(const kind of ['plans','numeric']) {
   const native=read('results/e-plan-native-'+v+'-'+kind+'.stdout'),mem=read('results/e-plan-memcheck-'+v+'-'+kind+'.stdout');
   assert.equal(mem,native);matchedRows+=native.trim().split('\n').length;
   const stderr=read('results/e-plan-memcheck-'+v+'-'+kind+'.stderr');
   assert.match(stderr,/ERROR SUMMARY: 0 errors from 0 contexts \(suppressed: 0 from 0\)/);
   for(const loss of ['definitely','indirectly','possibly'])assert.match(stderr,new RegExp(loss+' lost: 0 bytes in 0 blocks'));
   assert.match(stderr,/suppressed: 0 bytes in 0 blocks/);
   const heap=stderr.match(/in use at exit: ([\d,]+) bytes in ([\d,]+) blocks/),total=stderr.match(/total heap usage: ([\d,]+) allocs, ([\d,]+) frees, ([\d,]+) bytes allocated/);
   assert(heap&&total);const numbers=m=>m.slice(1).map(s=>Number(s.replaceAll(',','')));
   const[reachableBytes,reachableBlocks]=numbers(heap),[allocations,frees,cumulativeBytes]=numbers(total);
   assert.equal(allocations-frees,reachableBlocks);assert.equal(reachableBytes,kind==='plans'?544:35656);assert.equal(reachableBlocks,kind==='plans'?1:15);
   memory[v+'-'+kind]={reachableBytes,reachableBlocks,allocations,frees,cumulativeBytes};
  }
 }
 const costs={};
 for(const mode of ['cpu','allocation']) {
  const m=json('e-plan-'+mode+'-summary.json'),raw=rows('results/e-plan-'+mode+'.jsonl');
  assert.equal(m.mode,mode);assert.equal(m.cpu,6);assert.equal(m.summaries.length,71);
  assert.equal(raw.length,mode==='cpu'?3408:426);let seed=42819;
  const random=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
  const groups=[],scenarios=[];
  for(const p of [0,-8,-32,-64,-128,-512,-4096,-16384,-65536,-262144]) {
   for(const route of ['planner','kernel'])scenarios.push([p,route,'fresh']);
   for(const lifecycle of ['fresh','warm','coarsen',...(p<0?['refine']:[])])scenarios.push([p,'public',lifecycle]);
  }
  for(const p of [-64,-4096,-65536])for(const lifecycle of ['fresh','warm','coarsen','refine'])scenarios.push([p,'pi-control',lifecycle]);
  assert.deepEqual(m.summaries.map(g=>[g.p,g.route,g.lifecycle]),scenarios);
  let cursor=0;
  for(const g of m.summaries) {
   const observed=raw.slice(cursor,cursor+g.observations);cursor+=g.observations;
   assert.equal(g.observations,mode==='cpu'?48:6);assert.equal(g.pilots.length,2);
   const single=['public','pi-control'].includes(g.route)&&['fresh','refine'].includes(g.lifecycle);
   const calibrated=single?1:mode==='cpu'?Math.max(2,Math.min(1048576,Math.ceil(6e6/Math.max(...g.pilots.map(r=>r.ns/r.iterations))))):16;
   assert.equal(g.iterations,calibrated);
   for(const[r,i]of observed.map((r,i)=>[r,i])) {
    const block=Math.floor(i/(mode==='cpu'?4:2)),slot=i%(mode==='cpu'?4:2);
    const order=mode==='cpu'?(block%2?['candidate','baseline','baseline','candidate']:['baseline','candidate','candidate','baseline']):['baseline','candidate'];
    assert.equal(r.block,block);assert.equal(r.variant,order[slot]);assert.equal(r.iterations,g.iterations);
   }
   for(const r of [...g.pilots,...observed]) {
    assert.deepEqual([r.p,r.route,r.lifecycle],[g.p,g.route,g.lifecycle]);assert(r.ns>0);
    assert.equal(r.allocation===null,mode==='cpu');assert(Date.parse(r.started)<=Date.parse(r.finished));
    assert(Date.parse(m.started)<=Date.parse(r.started)&&Date.parse(r.finished)<=Date.parse(m.finished));
   }
   assert.equal(new Set([...g.pilots,...observed].map(r=>r.fingerprint+':'+r.answerBits)).size,1);
   if(mode==='cpu') {
    const ratios=Array.from({length:12},(_,b)=>{
     const clock=v=>median(observed.filter(r=>r.block===b&&r.variant===v).map(r=>r.ns));return clock('candidate')/clock('baseline');
    });
    const bootstrap=Array.from({length:5000},()=>median(ratios.map(()=>ratios[random(12)]))).sort((a,b)=>a-b);
    assert.equal(g.pairedMedianRatio,median(ratios));assert.deepEqual(g.pairedMedianBootstrap95,[bootstrap[125],bootstrap[4875]]);
    assert.deepEqual(g.nsPerQuery,Object.fromEntries(['baseline','candidate'].map(v=>[v,median(observed.filter(r=>r.variant===v).map(r=>r.ns/r.iterations))])));
    groups.push({p:g.p,route:g.route,lifecycle:g.lifecycle,ratio:g.pairedMedianRatio,interval:g.pairedMedianBootstrap95,nsPerQuery:g.nsPerQuery});
   } else {
    const measurements=Object.fromEntries(['baseline','candidate'].map(v=>[v,['requests','requestedBytes','liveDelta','peakDelta'].map((name,i)=>{
     const a=observed.filter(r=>r.variant===v).map(r=>r.allocation[i]);assert(a.every(n=>n>=0));assert.equal(Math.min(...a),Math.max(...a));return{name,min:Math.min(...a),max:Math.max(...a)};
    })]));assert.deepEqual(g.measurements,measurements);
    groups.push({p:g.p,route:g.route,lifecycle:g.lifecycle,iterations:g.iterations,measurements});
   }
  }
  assert.equal(cursor,raw.length);
  costs[mode]={groups:groups.length,observations:raw.length,
   ...(mode==='cpu'?{ratios:[Math.min(...groups.map(g=>g.ratio)),Math.max(...groups.map(g=>g.ratio))],
    below:groups.filter(g=>g.interval[1]<1).length,above:groups.filter(g=>g.interval[0]>1).length,
    overlap:groups.filter(g=>g.interval[0]<=1&&g.interval[1]>=1).length}:{}),details:groups};
 }
 return{planRequestsPerVariant:4151,exactBigIntPlanChecks:planChecks,candidateExtraTermRequests:0,
  lowerInvariantStepsPerVariant:20367,directedMpfrChecksPerVariant:300,exactBigIntEnclosureChecks:rationalChecks,
  exactSeriesOracleTerms:n,matchedNativeMemcheckRows:matchedRows,memory,costs};
}
if(process.argv.includes('--e-plan-summary'))console.log(JSON.stringify(checkEPlan()));
