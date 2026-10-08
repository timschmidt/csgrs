import {readFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
import {json} from './e-plan-qualified-sources.mjs';
const read=p=>readFileSync(p,'utf8'),rows=p=>read(p).trim().split('\n').map(s=>JSON.parse(s));
const median=a=>{const s=[...a].sort((a,b)=>a-b);return(s[(s.length-1)>>1]+s[s.length>>1])/2;};
const digest=v=>createHash('sha256').update(JSON.stringify(v)).digest('hex');
function tests(tag) {
 const g=json('results/'+tag+'.json');assert.equal(g.code,0);assert.equal(g.signal,null);
 const out=read('results/'+tag+'.stdout'),err=read('results/'+tag+'.stderr');assert(!/^warning(?:\[|:)|^error(?:\[|:)/m.test(err));
 const membership=[...out.matchAll(/^test (.+) \.\.\. (ok|ignored[^\n]*)$/gm)].map(m=>m[1]+'|'+m[2]).sort();
 const suites=[...out.matchAll(/^test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;/gm)].map(m=>m.slice(1).map(Number));
 assert(suites.length>0);const totals=suites.reduce((s,r)=>s.map((v,i)=>v+r[i]),[0,0,0,0,0]);
 assert.equal(totals[1],0);assert.equal(totals[3],0);assert.equal(totals[4],0);assert.equal(membership.length,totals[0]+totals[2]);
 return{tag,membership,suites:suites.length,totals,membershipSha256:digest(membership)};
}
function costs(wasm) {
 const name=wasm?'e-qualified-wasm-cpu':'e-qualified-controls',m=json(name+'-summary.json'),raw=rows('results/'+name+'.jsonl');
 const scenarios=wasm?[-64,-4096,-65536,-262144].flatMap(p=>[[0,'fresh'],[1,'fresh'],[2,'fresh'],[2,'warm']].map(([r,l])=>[p,r,l]))
  .concat([-64,-4096,-65536].flatMap(p=>['fresh','warm'].map(l=>[p,3,l]))):
  [[0,'public','warm'],[0,'public','coarsen'],[-64,'public','fresh'],[-4096,'public','fresh'],[-65536,'public','fresh'],
   [-262144,'public','fresh'],[-262144,'public','coarsen'],[-4096,'pi-control','warm'],[-65536,'pi-control','fresh'],[-65536,'public','warm']];
 assert.deepEqual(m.summaries.map(g=>[g.p,g.route,g.lifecycle]),scenarios);assert.equal(raw.length,wasm?1056:1600);
 let seed=wasm?43189:43177;const random=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
 const blocks=wasm?12:40,groups=[];let cursor=0;
 for(const g of m.summaries) {
  const observed=raw.slice(cursor,cursor+g.observations);cursor+=g.observations;assert.equal(observed.length,4*blocks);assert.equal(g.pilots.length,2);
  const single=g.lifecycle==='fresh'&&(!wasm||g.route>=2);
  const pilotCount=wasm?(single?1:g.lifecycle==='warm'?8192:Math.abs(g.p)>=4096?2:64):(single?1:8192);
  if(wasm)assert.equal(g.probeCount,pilotCount);
  for(const r of g.pilots)assert.equal(r.iterations,pilotCount);
  const iterations=single?1:Math.max(wasm?2:8192,Math.min(1048576,Math.ceil((wasm?6e6:10e6)/Math.max(...g.pilots.map(r=>r.ns/r.iterations)))));
  assert.equal(g.iterations,iterations);
  for(const[r,i]of observed.map((r,i)=>[r,i])) {
   const block=Math.floor(i/4),order=block%2?['candidate','baseline','baseline','candidate']:['baseline','candidate','candidate','baseline'];
   assert.equal(r.block,block);assert.equal(r.variant,order[i%4]);assert.equal(r.iterations,g.iterations);
  }
  for(const r of [...g.pilots,...observed]) {
   assert.deepEqual([r.p,r.route,r.lifecycle],[g.p,g.route,g.lifecycle]);assert(r.ns>0);
   if(!wasm)assert.equal(r.allocation,null);else assert(r.linearMemoryBytes>0&&r.linearMemoryBytes%65536===0);
   assert(Date.parse(r.started)<=Date.parse(r.finished));assert(Date.parse(r.started)>=Date.parse(m.started));assert(Date.parse(r.finished)<=Date.parse(m.finished));
  }
  assert.equal(new Set([...g.pilots,...observed].map(r=>r.fingerprint+':'+r.answerBits)).size,1);
  const ratios=Array.from({length:blocks},(_,b)=>{
   const clock=v=>median(observed.filter(r=>r.block===b&&r.variant===v).map(r=>r.ns));return clock('candidate')/clock('baseline');
  });
  const boot=Array.from({length:5000},()=>median(ratios.map(()=>ratios[random(blocks)]))).sort((a,b)=>a-b);
  assert.equal(g.pairedMedianRatio,median(ratios));assert.deepEqual(g.pairedMedianBootstrap95,[boot[125],boot[4875]]);
  const nsPerQuery=Object.fromEntries(['baseline','candidate'].map(v=>[v,median(observed.filter(r=>r.variant===v).map(r=>r.ns/r.iterations))]));
  assert.deepEqual(g.nsPerQuery,nsPerQuery);
  if(wasm)assert.deepEqual(g.linearMemoryBytes,Object.fromEntries(['baseline','candidate'].map(v=>{
   const a=observed.filter(r=>r.variant===v).map(r=>r.linearMemoryBytes);return[v,[Math.min(...a),Math.max(...a)]];
  })));
  groups.push({p:g.p,route:g.route,lifecycle:g.lifecycle,ratio:g.pairedMedianRatio,interval:g.pairedMedianBootstrap95,nsPerQuery,
   ...(wasm?{linearMemoryBytes:g.linearMemoryBytes}:{})});
 }
 assert.equal(cursor,raw.length);
 return{groups:groups.length,observations:raw.length,range:[Math.min(...groups.map(g=>g.ratio)),Math.max(...groups.map(g=>g.ratio))],
  below:groups.filter(g=>g.interval[1]<1).length,above:groups.filter(g=>g.interval[0]>1).length,
  overlap:groups.filter(g=>g.interval[0]<=1&&g.interval[1]>=1).length,details:groups};
}
export function checkEQualified() {
 const rust=[],added=['direct_e_kernel_has_strict_directed_enclosures','shared_e_recovers_from_cancellation_and_serialization',
  'shared_e_refines_and_coarsens_within_its_enclosure','term_planner_preserves_exact_factorial_thresholds']
  .map(n=>'computable::approximation::e_plan_tests::'+n+'|ok').sort();
 for(const feature of ['default','all'])for(const profile of ['debug','release']) {
  const baseline=tests(feature==='all'?'e-plan-tests-baseline-'+profile:'e-qualified-baseline-default-'+profile),candidate=tests('e-qualified-candidate-'+feature+'-'+profile);
  assert.deepEqual(baseline.totals,[feature==='all'?855:752,0,0,0,0]);assert.deepEqual(candidate.totals,[feature==='all'?859:756,0,0,0,0]);
  assert.deepEqual(candidate.membership,[...baseline.membership,...added].sort());assert.equal(candidate.suites,baseline.suites);
  for(const v of [baseline,candidate]) {const{membership,...summary}=v;rust.push(summary);}
 }
 for(const crate of ['hypersolve','hypercurve']) {
  const baseline=tests('e-qualified-baseline-'+crate+'-release'),candidate=tests('e-qualified-candidate-'+crate+'-release');
  assert.deepEqual(candidate.membership,baseline.membership);assert.deepEqual(candidate.totals,crate==='hypersolve'?[803,0,0,0,0]:[1764,0,9,0,0]);
  assert.equal(candidate.suites,crate==='hypersolve'?8:46);
  for(const v of [baseline,candidate]) {const{membership,...summary}=v;rust.push(summary);}
 }
 const doc=tests('e-qualified-candidate-doc');assert.deepEqual(doc.totals,[24,0,0,0,0]);const{membership,...docSummary}=doc;rust.push(docSummary);
 const wasmRows=rows('results/e-qualified-wasm-check.jsonl'),wasmSummary=json('e-qualified-wasm-check-summary.json');assert.equal(wasmRows.length,8902);
 assert.deepEqual(wasmSummary.summaries,[{variant:'baseline',plans:4151,enclosures:300},{variant:'candidate',plans:4151,enclosures:300}]);
 assert.equal(wasmSummary.rows,8902);assert.equal(wasmSummary.oracleTerms,20368);
 const expectedPlans=rows('results/e-plan-native-baseline-plans.stdout').filter(r=>r.kind==='plan');let factorial=1n,k=1;
 for(const r of expectedPlans) {const needed=r.p<0?-r.p+4:4;while(factorial.toString(2).length<=needed) {k++;factorial*=BigInt(k);}assert.equal(r.expected,k-1);}
 const positions=[...new Set([...Array.from({length:73},(_,i)=>i-64),-127,-128,-129,-255,-256,-257,-511,-512,-513,
  -1023,-1024,-1025,-4095,-4096,-4097,-16384,-32768,-65536,-120700,-262144])].sort((a,b)=>b-a);
 const shape=[...positions.flatMap(p=>[['kernel',p],['public-refine',p]]),...[...positions].reverse().map(p=>['public-coarsen',p]),
  ...[-8,-64,-128,-512,-4096,-16384,-65536,-262144].map(p=>['exp-one',p]),
  ...[0,-1,-8,-32,-64,-128,-512,-4096,-16384,-32768,-65536,-120700,-262144].map(p=>['fresh-instance',p])];
 let n=0,num=1n,den=1n;while(den.toString(2).length<=262144+16) {n++;den*=BigInt(n);num=num*BigInt(n)+1n;}
 assert.equal(n,20368);const tailDen=den*BigInt(n+1),tailNum=num*BigInt(n+1)+2n;let plans=0,enclosures=0;
 const byVariant={};
 for(const variant of ['baseline','candidate']) {
  const observed=wasmRows.filter(r=>r.variant===variant),p=observed.filter(r=>r.kind==='plan'),numbers=observed.filter(r=>r.kind!=='plan');
  assert.deepEqual(p.map(r=>[r.p,r.actual,r.expected]),expectedPlans.map(r=>[r.p,r.expected,r.expected]));plans+=p.length;
  assert.deepEqual(numbers.map(r=>[r.kind,r.p]),shape);
  for(const r of numbers) {
   assert.match(r.integer,/^[0-9a-f]+$/);const a=BigInt('0x'+r.integer);
   if(r.p<0) {assert((a-1n)*den<(num<<BigInt(-r.p)));assert((a+1n)*tailDen>(tailNum<<BigInt(-r.p)));}
   else {assert(((a-1n)*den<<BigInt(r.p))<num);assert(((a+1n)*tailDen<<BigInt(r.p))>tailNum);}enclosures++;
  }
  const native=rows('results/e-plan-native-'+variant+'-numeric.stdout').filter(r=>r.kind==='kernel');
  assert.deepEqual(numbers.filter(r=>r.kind==='kernel').map(r=>[r.p,r.integer]),native.map(r=>[r.p,r.integer]));
  byVariant[variant]=numbers.map(({variant,...r})=>r);
 }
 assert.deepEqual(byVariant.candidate,byVariant.baseline);
 const apps=json('e-qualified-app-size-summary.json'),appSizes=[];
 for(const[crate,example]of [['hyperreal','readme_quickstart'],['hypercurve','basic'],['hypercurve','arrangement']]) {
  const b=apps.artifacts.find(a=>a.variant==='baseline'&&a.crate===crate&&a.example===example),c=apps.artifacts.find(a=>a.variant==='candidate'&&a.crate===crate&&a.example===example);assert(b&&c);
  appSizes.push({crate,example,baselineBytes:b.files.map(f=>f.bytes),candidateBytes:c.files.map(f=>f.bytes),delta:c.files.map((f,i)=>f.bytes-b.files[i].bytes)});
  for(const v of ['baseline','candidate']) {
   const tag='e-qualified-app-'+v+'-'+crate+'-'+example.replaceAll('_','-');
   const s=read('results/'+tag+'-size.stdout').trim().split('\n').slice(1).map(line=>line.trim().split(/\s+/));assert.equal(s.length,2);
   assert.deepEqual(s[0].slice(0,5),s[1].slice(0,5));assert(s.every(r=>r.slice(0,4).every(n=>/^\d+$/.test(n))));
   assert.equal(json('results/'+tag+'-run.json').code,0);assert.equal(read('results/'+tag+'-run.stderr'),'');
  }
 }
 assert.deepEqual(appSizes.map(s=>s.delta),[[0,0],[-272,-944],[-208,-928]]);
 return{rust,addedTests:4,testModuleLines:112,wasm:{exactPlanChecks:plans,exactEnclosures:enclosures,rows:8902,
  bitIdenticalVariants:true,kernelBitIdenticalToNative:true,node:wasmSummary.node,v8:wasmSummary.v8},
  nativeControls:costs(false),wasmCosts:costs(true),appSizes};
}
if(process.argv.includes('--e-qualified-summary'))console.log(JSON.stringify(checkEQualified()));
