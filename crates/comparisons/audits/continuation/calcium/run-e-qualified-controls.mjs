import {spawn} from 'node:child_process';
import {writeFileSync,appendFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sources,sha,json} from './e-plan-qualified-sources.mjs';
const sourceMap=sources(),frozen=json('e-plan-binaries.json'),raw='results/e-qualified-controls.jsonl';
for(const v of ['baseline','candidate'])assert.equal(sha(frozen.binaries[v+'-cpu'].path),frozen.binaries[v+'-cpu'].sha256);
writeFileSync(raw,'',{flag:'wx'});const started=new Date().toISOString();
const groups=[[0,'public','warm'],[0,'public','coarsen'],[-64,'public','fresh'],[-4096,'public','fresh'],
 [-65536,'public','fresh'],[-262144,'public','fresh'],[-262144,'public','coarsen'],[-4096,'pi-control','warm'],
 [-65536,'pi-control','fresh'],[-65536,'public','warm']];
const median=a=>{const s=[...a].sort((a,b)=>a-b);return(s[(s.length-1)>>1]+s[s.length>>1])/2;};
let seed=43177;const random=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
async function run(variant,args,iterations) {
 const started=new Date().toISOString();
 const row=await new Promise((ok,fail)=>{
  const c=spawn('taskset',['-c','6',frozen.binaries[variant+'-cpu'].path,...args.map(String),String(iterations)],{stdio:['ignore','pipe','pipe']});let out='',err='';
  c.stdout.on('data',s=>out+=s);c.stderr.on('data',s=>err+=s);c.on('error',fail);
  c.on('close',(code,signal)=>code===0&&!signal&&!err?ok(JSON.parse(out)):fail(Error(JSON.stringify({code,signal,err}))));
 });
 assert.deepEqual([row.p,row.route,row.lifecycle],args);assert.equal(row.iterations,iterations);assert.equal(row.allocation,null);assert(row.ns>0);
 return{started,finished:new Date().toISOString(),variant,...row};
}
const summaries=[];
for(const args of groups) {
 const single=args[2]==='fresh',pilots=[];for(const v of ['baseline','candidate'])pilots.push(await run(v,args,single?1:8192));
 const iterations=single?1:Math.max(8192,Math.min(1048576,Math.ceil(10e6/Math.max(...pilots.map(r=>r.ns/r.iterations)))));
 const observed=[],blocks=40;
 for(let block=0;block<blocks;block++)for(const v of block%2?['candidate','baseline','baseline','candidate']:['baseline','candidate','candidate','baseline']) {
  const r={block,...await run(v,args,iterations)};observed.push(r);appendFileSync(raw,JSON.stringify(r)+'\n');
 }
 assert.equal(new Set([...pilots,...observed].map(r=>r.fingerprint+':'+r.answerBits)).size,1);
 const ratios=Array.from({length:blocks},(_,b)=>{
  const clock=v=>median(observed.filter(r=>r.block===b&&r.variant===v).map(r=>r.ns));return clock('candidate')/clock('baseline');
 });
 const boot=Array.from({length:5000},()=>median(ratios.map(()=>ratios[random(blocks)]))).sort((a,b)=>a-b);
 const r={p:args[0],route:args[1],lifecycle:args[2],iterations,pilots,observations:observed.length,pairedMedianRatio:median(ratios),
  pairedMedianBootstrap95:[boot[125],boot[4875]],nsPerQuery:Object.fromEntries(['baseline','candidate'].map(v=>[v,median(observed.filter(r=>r.variant===v).map(r=>r.ns/r.iterations))]))};
 summaries.push(r);console.log(JSON.stringify(r));
}
assert.deepEqual(sources(),sourceMap);
writeFileSync('e-qualified-controls-summary.json',JSON.stringify({started,finished:new Date().toISOString(),cpu:6,sourceMap,summaries,
 limits:'Explicit follow-up corpus includes all three earlier slower-interval controls, small/large cold e and additional unchanged cache/pi controls. Reuses the frozen42 CPU executables because the qualified candidate only adds cfg(test) code to its unchanged production planner. Forty ABBA/BAAB blocks per group;8192-query calibration for cached paths, target10ms; first-use remains one query per fresh process. Per-group bootstrap intervals unadjusted. This supplements rather than replaces the original71 groups and all unfavorable evidence. No new mathematical oracle, representative application throughput or universal speedup.'},null,2)+'\n',{flag:'wx'});
