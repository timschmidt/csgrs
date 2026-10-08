import {readFileSync,writeFileSync,appendFileSync} from 'node:fs';
import assert from 'node:assert/strict';
import {sources,sha,json} from './e-plan-qualified-sources.mjs';
assert.equal(typeof global.gc,'function','run Node with --expose-gc');
const sourceMap=sources(),frozen=json('e-qualified-wasm-binaries.json'),modules={};
assert.deepEqual(sourceMap,frozen.sourceMap);assert.equal(json('results/e-qualified-wasm-check.json').code,0);
for(const v of ['baseline','candidate']) {const b=frozen.modules[v];assert.equal(sha(b.path),b.sha256);modules[v]=new WebAssembly.Module(readFileSync(b.path));}
const raw='results/e-qualified-wasm-cpu.jsonl';writeFileSync(raw,'',{flag:'wx'});const started=new Date().toISOString();
const scenarios=[];
for(const p of [-64,-4096,-65536,-262144])for(const[route,lifecycle]of [[0,'fresh'],[1,'fresh'],[2,'fresh'],[2,'warm']])scenarios.push([p,route,lifecycle]);
for(const p of [-64,-4096,-65536])for(const lifecycle of ['fresh','warm'])scenarios.push([p,3,lifecycle]);
assert.equal(scenarios.length,22);
const median=a=>{const s=[...a].sort((a,b)=>a-b);return(s[(s.length-1)>>1]+s[s.length>>1])/2;};
let seed=43189;const random=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
function run(variant,args,iterations) {
 global.gc();const[p,route,lifecycle]=args,instance=new WebAssembly.Instance(modules[variant],{}).exports;
 if(lifecycle==='warm')instance.evaluate(p,route===3?2:1);
 const started=new Date().toISOString(),before=process.hrtime.bigint(),result=instance.time_loop(p,route,iterations);
 const ns=Number(process.hrtime.bigint()-before),linearMemoryBytes=instance.memory.buffer.byteLength;
 let answer;
 if(route===0) {answer=BigInt(instance.plan(p));assert.equal(result,iterations%2?Number(answer):0);}
 else {
  assert.equal(result,0);const len=instance.evaluate(p,route===1?0:route===2?1:2);answer=0n;
  for(let i=len-1;i>=0;i--)answer=(answer<<32n)+BigInt(instance.answer_word(i)>>>0);
 }
 const hex=answer.toString(16);let fingerprint=0xcbf29ce484222325n;
 for(const c of hex)fingerprint=BigInt.asUintN(64,(fingerprint^BigInt(c.charCodeAt(0)))*0x100000001b3n);
 assert(ns>0);return{variant,p,route,lifecycle,iterations,started,finished:new Date().toISOString(),ns,linearMemoryBytes,
  fingerprint:fingerprint.toString(16),answerBits:answer===0n?0:answer.toString(2).length};
}
const summaries=[];
for(const args of scenarios) {
 const[p,route,lifecycle]=args,single=route>=2&&lifecycle==='fresh',pilots=[];
 const probeCount=single?1:lifecycle==='warm'?8192:Math.abs(p)>=4096?2:64;
 // Compiled modules are reused, but each query group uses fresh linear memory.
 // These fixed warm-up calls are not cold-process/engine startup measurements.
 for(let i=0;i<4;i++)for(const v of ['baseline','candidate'])run(v,args,probeCount);
 for(const v of ['baseline','candidate'])pilots.push(run(v,args,probeCount));
 const iterations=single?1:Math.max(2,Math.min(1048576,Math.ceil(6e6/Math.max(...pilots.map(r=>r.ns/r.iterations)))));
 const observed=[],blocks=12;
 for(let block=0;block<blocks;block++)for(const v of block%2?['candidate','baseline','baseline','candidate']:['baseline','candidate','candidate','baseline']) {
  const r={block,...run(v,args,iterations)};observed.push(r);appendFileSync(raw,JSON.stringify(r)+'\n');
 }
 assert.equal(new Set([...pilots,...observed].map(r=>r.fingerprint+':'+r.answerBits)).size,1);
 const ratios=Array.from({length:blocks},(_,b)=>{
  const clock=v=>median(observed.filter(r=>r.block===b&&r.variant===v).map(r=>r.ns));return clock('candidate')/clock('baseline');
 });
 const boot=Array.from({length:5000},()=>median(ratios.map(()=>ratios[random(blocks)]))).sort((a,b)=>a-b);
 const g={p,route,lifecycle,iterations,probeCount,pilots,observations:48,pairedMedianRatio:median(ratios),pairedMedianBootstrap95:[boot[125],boot[4875]],
  nsPerQuery:Object.fromEntries(['baseline','candidate'].map(v=>[v,median(observed.filter(r=>r.variant===v).map(r=>r.ns/r.iterations))])),
  linearMemoryBytes:Object.fromEntries(['baseline','candidate'].map(v=>{const a=observed.filter(r=>r.variant===v).map(r=>r.linearMemoryBytes);return[v,[Math.min(...a),Math.max(...a)]];}))};
 summaries.push(g);console.log(JSON.stringify(g));
}
assert.deepEqual(sources(),sourceMap);
writeFileSync('e-qualified-wasm-cpu-summary.json',JSON.stringify({started,finished:new Date().toISOString(),node:process.version,v8:process.versions.v8,
 sourceMap,summaries,limits:'22 predeclared wasm32 groups in local V8,12 ABBA/BAAB blocks and5000 paired bootstrap resamples per group; intervals unadjusted. Caller pins Node to CPU6. Both compiled modules are reused; four warmups and one pilot per variant use predeclared counts:1 for fresh public,8192 for warm public,2 for deep uncached p<=-4096,64 for small uncached. Every observation has a new module instance/linear memory. Wrapper construction and output destruction are inside the Rust time_loop; module instantiation, explicit host GC, setup and postflight export are excluded. Fresh means fresh constant cache, not cold V8 engine or process. Native and WASM timings are separate experiments. Linear-memory high-water bytes include static/setup/runtime and are not live allocation, peak RSS or arbitrary retention bounds. Matched postflight fingerprints are not numerical oracles; full-word exact enclosure qualification is a separate gate. Not physical ARM/RISC-V performance or a universal speedup.'},null,2)+'\n',{flag:'wx'});
