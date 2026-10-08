import { readFileSync } from 'node:fs';
import { dirname,resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
import { json } from './derivative-demand-sources.mjs';
const here=dirname(fileURLToPath(import.meta.url));
const rows=p=>readFileSync(resolve(here,'results/'+p+'.jsonl'),'utf8').trimEnd().split('\n').map(JSON.parse);
const median=a=>{const v=[...a].sort((x,y)=>x-y);return(v[Math.floor((v.length-1)/2)]+v[Math.floor(v.length/2)])/2;};
const keys=['requests','requested_bytes','live_delta','peak_delta'];
export function qualificationCosts() {
 const retention=rows('derivative-retention-staircase');assert.equal(retention.length,320);
 const plateaus=[];let at=0;
 for(const degree of [8,24])for(const kind of [0,1])for(const order of degree===8?[128]:[0,24,128])
 for(const lifecycle of ['retained_curve','fresh_curve']) {
  const measurements=[];
  for(const iterations of [1,8,32,128,512]) {
   const group=[];
   for(let repeat=0;repeat<2;repeat++)for(const variant of ['baseline','candidate']) {
    const r=retention[at++];for(const[k,v]of Object.entries({repeat,variant,mode:'allocation',kind,degree,order,lifecycle,iterations}))assert.equal(r[k],v);
    assert(Date.parse(r.started)<=Date.parse(r.finished));group.push(r);
   }
   for(const k of keys) {assert.equal(group[0][k],group[2][k]);assert.equal(group[1][k],group[3][k]);}
   assert.equal(group[0].live_delta,group[1].live_delta);assert.equal(group[0].peak_delta,group[1].peak_delta);
   for(const k of ['requests','requested_bytes'])assert(group[1][k]<=group[0][k]);
   measurements.push({iterations,live:group[0].live_delta,peak:group[0].peak_delta,
    baseline:Object.fromEntries(keys.slice(0,2).map(k=>[k,group[0][k]])),
    candidate:Object.fromEntries(keys.slice(0,2).map(k=>[k,group[1][k]]))});
  }
  for(const k of ['live','peak'])for(const i of [3,4])assert.equal(measurements[2][k],measurements[i][k]);
  for(const v of ['baseline','candidate'])for(const k of ['requests','requested_bytes']) {
   const rate=(i,j)=>(measurements[j][v][k]-measurements[i][v][k])/(measurements[j].iterations-measurements[i].iterations);
   assert.equal(rate(2,3),rate(3,4));
  }
  plateaus.push({kind,degree,order,lifecycle,liveAt32:measurements[2].live,peakAt32:measurements[2].peak});
 }
 assert.equal(at,retention.length);
 const summary={retentionRows:320,plateauGroups:16,maxLiveAt32:Math.max(...plateaus.map(p=>p.liveAt32)),plateaus,
  endpointGroups:16,cpuRows:768,allocationRows:96,intervalBelowOne:0,intervalAboveOne:0,endpointRatios:[],
  endpointAllocationEqualGroups:0,endpointAllocationNonzeroLiveGroups:0};
 let seed=270927;const random=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
 for(const mode of ['cpu','allocation']) {
  const raw=rows('derivative-endpoint-'+mode),reported=json('derivative-endpoint-'+mode+'-summary.json');
  assert.equal(raw.length,mode==='cpu'?768:96);assert.equal(reported.summaries.length,16);
  let cursor=0,groupIndex=0;
  for(const kind of [0,1])for(const degree of [1,3,8,24])for(const lifecycle of ['retained_graph','fresh_graph']) {
   const s=reported.summaries[groupIndex++],group=[];
   for(const[k,v]of Object.entries({kind,degree,lifecycle,observations:mode==='cpu'?48:6}))assert.equal(s[k],v);
   assert.equal(s.pilots.length,2);
   for(const [i,v]of ['baseline','candidate'].entries()) {
    const p=s.pilots[i];for(const[k,value]of Object.entries({variant:v,mode,kind,degree,lifecycle,iterations:2}))assert.equal(p[k],value);
    assert(p.elapsed_ns>0);
   }
   assert.equal(s.iterations,mode==='cpu'?Math.max(2,Math.min(500,Math.ceil(20e6/Math.max(...s.pilots.map(r=>r.elapsed_ns/r.iterations))))):8);
   for(let block=0;block<(mode==='cpu'?12:3);block++) {
    const variants=mode==='allocation'?['baseline','candidate']:block%2?['candidate','baseline','baseline','candidate']:['baseline','candidate','candidate','baseline'];
    for(const variant of variants) {
     const r=raw[cursor++];for(const[k,v]of Object.entries({block,variant,mode,kind,degree,lifecycle,iterations:s.iterations}))assert.equal(r[k],v);
     assert(r.elapsed_ns>0);assert(Date.parse(r.started)<=Date.parse(r.finished));
     assert(Date.parse(r.started)>=Date.parse(reported.started)&&Date.parse(r.finished)<=Date.parse(reported.finished));
     if(mode==='cpu')for(const k of keys)assert.equal(r[k],0);
     group.push(r);
    }
   }
   if(mode==='cpu') {
    const ratios=Array.from({length:12},(_,block)=>{
     const clock=v=>median(group.filter(r=>r.block===block&&r.variant===v).map(r=>r.elapsed_ns));return clock('candidate')/clock('baseline');
    });
    const samples=Array.from({length:5000},()=>median(ratios.map(()=>ratios[random(12)]))).sort((a,b)=>a-b);
    assert.equal(s.pairedMedianRatio,median(ratios));
    assert.deepEqual(s.pairedMedianBootstrap95,[samples[125],samples[4875]]);
    for(const variant of ['baseline','candidate'])assert.equal(s.nsPerQuery[variant],median(group.filter(r=>r.variant===variant).map(r=>r.elapsed_ns/r.iterations)));
    summary.endpointRatios.push(s.pairedMedianRatio);
    summary.intervalBelowOne+=+(s.pairedMedianBootstrap95[1]<1);
    summary.intervalAboveOne+=+(s.pairedMedianBootstrap95[0]>1);
   } else {
    for(const variant of ['baseline','candidate'])for(const k of keys) {
     const values=group.filter(r=>r.variant===variant).map(r=>r[k]);
     assert.deepEqual(s.measurements[variant][k],{min:Math.min(...values),max:Math.max(...values)});
     assert.equal(Math.min(...values),Math.max(...values));
    }
    assert.deepEqual(s.measurements.baseline,s.measurements.candidate);
    summary.endpointAllocationEqualGroups++;
    summary.endpointAllocationNonzeroLiveGroups+=+(s.measurements.baseline.live_delta.min!==0);
   }
  }
  assert.equal(cursor,raw.length);
 }
 return summary;
}
if(process.argv.includes('--cost-summary'))console.log(JSON.stringify(qualificationCosts()));
