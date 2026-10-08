import {readFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {json} from './sign-filter-mask-sources.mjs';
const read=p=>readFileSync(p,'utf8');
const median=a=>{const v=[...a].sort((x,y)=>x-y);return(v[Math.floor((v.length-1)/2)]+v[Math.floor(v.length/2)])/2;};
export function checkSignFilterMask() {
 let testNames;
 const suiteCounts=[255,10,14,19,3,7,3,2,9,4,3,2,6,4,5,18];
 for(const v of ['baseline','mask'])for(const p of ['debug','release']) {
  const s=read('results/sign-filter-'+v+'-full-'+p+'.stdout');
  const names=[...s.matchAll(/^test (.+) \.\.\. ok$/gm)].map(m=>m[1]);assert.equal(names.length,364);
  if(testNames)assert.deepEqual(names,testNames);else testNames=names;
  assert.deepEqual([...s.matchAll(/^test result: ok\. (\d+) passed; 0 failed; 0 ignored; 0 measured; 0 filtered out;/gm)].map(m=>Number(m[1])),suiteCounts);
 }
 const traces=read('results/sign-filter-baseline-trace.stdout');
 assert.equal(traces,read('results/sign-filter-mask-trace.stdout'));assert.equal(traces,read('results/nfloat-complex-filter-probe-native.stdout'));
 assert.equal(traces.trimEnd().split('\n').length,45);
 const oracle=JSON.parse(read('results/sign-filter-ring-oracle.stdout'));assert.equal(oracle.cases,24);
 const variants=['baseline','candidate'],result={tests:{perProfile:364,suites:suiteCounts,names:testNames,focused:0,
  exhaustiveSequences:299593,longSequences:1280,privateTraceCases:32},publicTraces:45,independentRingSigns:24};
 for(const mode of ['cpu','allocation']) {
  const summary=json('sign-filter-mask-'+mode+'-summary.json'),raw=read('results/sign-filter-mask-'+mode+'.jsonl').trimEnd().split('\n').map(JSON.parse);
  assert.equal(summary.mode,mode);assert.equal(summary.cpu,6);assert.equal(summary.summaries.length,48);
  assert.equal(raw.length,mode==='cpu'?2304:288);let groupIndex=0,rawIndex=0,seed=31337;
  const random=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
  const aggregate={groups:48,observations:raw.length,timedQueries:raw.reduce((n,r)=>n+r.iterations,0),below:0,above:0,straddles:0,
   requestImproved:0,byteImproved:0,peakImproved:0,requestEqual:0,byteEqual:0,peakEqual:0,liveEqual:0,ratios:[],regressions:[]};
  for(const fixture of ['rational','pi-far','pi-near','pi-reversed'])for(const vertices of [3,4,8,16,32,128])for(const lifecycle of ['retained','cloned-ring']) {
   const g=summary.summaries[groupIndex++];assert.equal(g.fixture,fixture);assert.equal(g.vertices,vertices);assert.equal(g.lifecycle,lifecycle);
   const o=oracle.rows.find(r=>r.fixture===fixture&&r.vertices===vertices);assert(o);const sign=o.sign===1?'Positive':'Negative';
   const stage=fixture==='rational'?'Exact':fixture==='pi-far'?'Structural':'Refined';
   assert.equal(g.outcome,`Decided { value: ${sign}, certainty: Exact, stage: ${stage} }`);
   assert.equal(g.pilots.length,2);const iterations=mode==='cpu'?Math.max(8,Math.min(20000,Math.ceil(6e6/Math.max(...g.pilots.map(v=>v.elapsed_ns/v.iterations))))):16;
   assert.equal(g.iterations,iterations);const rows=[],blocks=mode==='cpu'?12:3;
   const check=(r,v,n)=>{
    for(const[k,want]of Object.entries({mode,variant:v,fixture,vertices,lifecycle,iterations:n,outcome:g.outcome}))assert.equal(r[k],want);
    assert(r.elapsed_ns>0);assert(Date.parse(r.finished)>=Date.parse(r.started));
    if(mode==='cpu')for(const k of ['requests','requested_bytes','live_delta','peak_delta'])assert.equal(r[k],0);
   };
   g.pilots.forEach((r,i)=>check(r,variants[i],2));
   for(let block=0;block<blocks;block++)for(const v of mode==='cpu'?(block%2?['candidate','baseline','baseline','candidate']:['baseline','candidate','candidate','baseline']):variants) {
    const r=raw[rawIndex++];assert.equal(r.block,block);check(r,v,iterations);rows.push(r);
   }
   assert.equal(g.observations,rows.length);
   if(mode==='cpu') {
    const ratios=Array.from({length:blocks},(_,b)=>{const clock=v=>median(rows.filter(r=>r.block===b&&r.variant===v).map(r=>r.elapsed_ns));return clock('candidate')/clock('baseline');});
    const boot=Array.from({length:5000},()=>median(ratios.map(()=>ratios[random(blocks)]))).sort((a,b)=>a-b);
    assert.equal(g.pairedMedianRatio,median(ratios));assert.deepEqual(g.pairedMedianBootstrap95,[boot[125],boot[4875]]);
    assert.deepEqual(g.nsPerQuery,Object.fromEntries(variants.map(v=>[v,median(rows.filter(r=>r.variant===v).map(r=>r.elapsed_ns/r.iterations))])));
    aggregate.ratios.push(g.pairedMedianRatio);
    if(g.pairedMedianBootstrap95[1]<1)aggregate.below++;
    else if(g.pairedMedianBootstrap95[0]>1){aggregate.above++;aggregate.regressions.push({fixture,vertices,lifecycle,ratio:g.pairedMedianRatio,ci:g.pairedMedianBootstrap95});}
    else aggregate.straddles++;
   }else {
    const measured=Object.fromEntries(variants.map(v=>[v,Object.fromEntries(['requests','requested_bytes','live_delta','peak_delta'].map(k=>{
     const values=rows.filter(r=>r.variant===v).map(r=>r[k]);return[k,{min:Math.min(...values),max:Math.max(...values)}];
    }))]));assert.deepEqual(g.measurements,measured);
    assert.deepEqual(measured,json('sign-filter-allocation-summary.json').summaries[groupIndex-1].measurements);
    for(const x of Object.values(measured))for(const v of Object.values(x))assert.equal(v.min,v.max);
    const b=measured.baseline,c=measured.candidate,reaches=fixture==='pi-near'||fixture==='pi-reversed';
    assert.equal(b.requests.min-c.requests.min,reaches?iterations:0);
    assert.equal(b.requested_bytes.min-c.requested_bytes.min,reaches?iterations*2*vertices:0);
    assert.equal(b.live_delta.min,0);assert.equal(c.live_delta.min,0);assert(c.peak_delta.min<=b.peak_delta.min);
    aggregate[b.requests.min>c.requests.min?'requestImproved':'requestEqual']++;
    aggregate[b.requested_bytes.min>c.requested_bytes.min?'byteImproved':'byteEqual']++;
    aggregate[b.peak_delta.min>c.peak_delta.min?'peakImproved':'peakEqual']++;aggregate.liveEqual++;
   }
  }
  assert.equal(rawIndex,raw.length);
  if(mode==='cpu'){aggregate.ratioRange=[Math.min(...aggregate.ratios),Math.max(...aggregate.ratios)];delete aggregate.ratios;
   assert.deepEqual([aggregate.below,aggregate.above,aggregate.straddles],[2,7,39]);}
  else {delete aggregate.ratios;assert.deepEqual([aggregate.requestImproved,aggregate.byteImproved,aggregate.peakImproved,aggregate.liveEqual],[24,24,12,48]);}
  result[mode]=aggregate;
 }
 const bins=json('sign-filter-mask-binaries.json');result.benchBinaryDeltas={};
 for(const kind of ['cpu','allocation']) {
  const b=bins['baseline-'+kind],c=bins['candidate-'+kind];
  const sizes=x=>x.size.split('\n')[1].split(/\s+/).filter(Boolean).slice(0,3).map(Number);
  result.benchBinaryDeltas[kind]={file:c.bytes-b.bytes,sections:sizes(c).map((v,i)=>v-sizes(b)[i])};
 }
 assert.deepEqual(result.benchBinaryDeltas,{cpu:{file:-1176,sections:[-576,-8,600]},allocation:{file:-1264,sections:[-584,-8,600]}});
 return result;
}
if(process.argv.includes('--sign-filter-mask-summary'))console.log(JSON.stringify(checkSignFilterMask()));
