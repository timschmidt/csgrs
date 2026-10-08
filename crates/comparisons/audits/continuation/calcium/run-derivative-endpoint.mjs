import { spawn } from 'node:child_process';
import { writeFileSync, appendFileSync, copyFileSync, constants, statSync, mkdtempSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
import { sources, sha, json } from './derivative-demand-sources.mjs';
const here=dirname(fileURLToPath(import.meta.url)),mode=process.argv[2];
assert(['cpu','allocation'].includes(mode));sources();
function command(file,args) {return new Promise((ok,fail)=>{
 const c=spawn(file,args,{stdio:['ignore','pipe','pipe']});let out='',err='';
 c.stdout.on('data',s=>out+=s);c.stderr.on('data',s=>err+=s);c.on('error',fail);
 c.on('close',code=>code===0?ok(out):fail(Error(file+': '+code+': '+err)));
});}
let binaries;
if(mode==='cpu') {
 const root=mkdtempSync('/tmp/calcium-derivative-endpoint.');binaries={};
 for(const variant of ['baseline','candidate'])for(const kind of ['cpu','allocation']) {
  assert.equal(json('results/derivative-endpoint-'+variant+'-build-v2.json').code,0);
  const source='/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse/release/calcium-derivative-endpoint-'+variant+'-'+kind,path=root+'/'+variant+'-'+kind;
  copyFileSync(source,path,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
  binaries[variant+'-'+kind]={path,bytes:statSync(path).size,sha256:sha(path),size:(await command('size',[path])).trim()};
 }
 writeFileSync(resolve(here,'derivative-endpoint-binaries.json'),JSON.stringify(binaries,null,2)+'\n',{flag:'wx'});
} else binaries=json('derivative-endpoint-binaries.json');
for(const b of Object.values(binaries))assert.equal(sha(b.path),b.sha256);
const started=new Date().toISOString();
// All known qualification/build/memory jobs must be terminal before CPU timing.
if(mode==='cpu')for(const tag of ['derivative-app-size','derivative-candidate-consumer-all-debug',
 'derivative-candidate-clippy-all','derivative-baseline-wasm-library','derivative-candidate-wasm-library',
 'derivative-state-baseline-memcheck','derivative-state-candidate-memcheck',
 'derivative-endpoint-baseline-build-v2','derivative-endpoint-candidate-build-v2']) {
 const g=json('results/'+tag+'.json');assert([0,97].includes(g.code));assert(Date.parse(g.finished)<=Date.parse(started));
}
const path=resolve(here,'results/derivative-endpoint-'+mode+'.jsonl');writeFileSync(path,'',{flag:'wx'});
const median=a=>{const v=[...a].sort((x,y)=>x-y);return(v[Math.floor((v.length-1)/2)]+v[Math.floor(v.length/2)])/2;};
let seed=270927;const random=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
async function run(variant,kind,degree,lifecycle,iterations) {
 const begin=new Date().toISOString();
 const r=JSON.parse(await command('taskset',['-c','6',binaries[variant+'-'+mode].path,variant,String(kind),String(degree),lifecycle,String(iterations)]));
 for(const[k,v]of Object.entries({variant,mode,kind,degree,lifecycle,iterations}))assert.equal(r[k],v);
 assert(r.elapsed_ns>0);
 return {started:begin,finished:new Date().toISOString(),...r};
}
const summaries=[];
for(const kind of [0,1])for(const degree of [1,3,8,24])for(const lifecycle of ['retained_graph','fresh_graph']) {
 const pilots=[];for(const v of ['baseline','candidate'])pilots.push(await run(v,kind,degree,lifecycle,2));
 const iterations=mode==='cpu'?Math.max(2,Math.min(500,Math.ceil(20e6/Math.max(...pilots.map(r=>r.elapsed_ns/r.iterations))))):8;
 const rows=[],blocks=mode==='cpu'?12:3;
 for(let block=0;block<blocks;block++) {
  const variants=mode==='allocation'?['baseline','candidate']:block%2?['candidate','baseline','baseline','candidate']:['baseline','candidate','candidate','baseline'];
  for(const v of variants) {const r={block,...await run(v,kind,degree,lifecycle,iterations)};rows.push(r);appendFileSync(path,JSON.stringify(r)+'\n');}
 }
 const summary={kind,degree,lifecycle,iterations,observations:rows.length,pilots};
 if(mode==='cpu') {
  const ratios=Array.from({length:blocks},(_,block)=>{
   const time=v=>median(rows.filter(r=>r.block===block&&r.variant===v).map(r=>r.elapsed_ns));return time('candidate')/time('baseline');
  });
  const samples=Array.from({length:5000},()=>median(ratios.map(()=>ratios[random(blocks)]))).sort((a,b)=>a-b);
  Object.assign(summary,{pairedMedianRatio:median(ratios),pairedMedianBootstrap95:[samples[125],samples[4875]],
   nsPerQuery:Object.fromEntries(['baseline','candidate'].map(v=>[v,median(rows.filter(r=>r.variant===v).map(r=>r.elapsed_ns/r.iterations))]))});
 } else summary.measurements=Object.fromEntries(['baseline','candidate'].map(v=>[v,Object.fromEntries(
  ['requests','requested_bytes','live_delta','peak_delta'].map(k=>{const a=rows.filter(r=>r.variant===v).map(r=>r[k]);return[k,{min:Math.min(...a),max:Math.max(...a)}];}))]));
 summaries.push(summary);console.log(JSON.stringify(summary));
}
sources();
writeFileSync(resolve(here,'derivative-endpoint-'+mode+'-summary.json'),JSON.stringify({mode,started,finished:new Date().toISOString(),cpu:6,binaries,summaries,
 limits:'Sixteen known closed-square four-fragment public tangent-order traversal workloads: degrees 1/3/8/24, rational and pi-scaled positive-weight curves, retained/fresh graphs from shared prebuilt input controls. Every result must certify the known single closed chain. Eight warm traversals after a precheck. End endpoint requests order one, start requests order three and is unchanged by candidate. Thus this measures the complete consumer, not an isolated high-order endpoint helper. Uninstrumented pinned CPU; separate Rust allocation counts, not RSS. Per-group bootstrap intervals are not multiplicity-adjusted or universal.'
},null,2)+'\n',{flag:'wx'});
