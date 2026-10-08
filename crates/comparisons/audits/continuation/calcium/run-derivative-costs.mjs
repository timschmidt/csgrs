import { spawn } from 'node:child_process';
import { writeFileSync, appendFileSync, copyFileSync, constants, statSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
import { sources, sha, json } from './derivative-demand-sources.mjs';
const here=dirname(fileURLToPath(import.meta.url)), mode=process.argv[2];
assert(['cpu','allocation'].includes(mode)); sources();
async function command(file,args) {
  return new Promise((ok,fail)=>{
    const c=spawn(file,args,{stdio:['ignore','pipe','pipe']});let out='',err='';
    c.stdout.on('data',s=>out+=s);c.stderr.on('data',s=>err+=s);c.on('error',fail);
    c.on('close',code=>code===0?ok(out):fail(Error(file+' '+args.join(' ')+': '+code+': '+err)));
  });
}
const variants=['baseline','candidate'];
for(const v of variants)assert.equal(json('results/derivative-cost-'+v+'-build.json').code,0);
let binaries;
if(mode==='cpu') {
  const root=process.argv[3];assert(/^\/tmp\/calcium-derivative-costs\.[a-zA-Z0-9]+$/.test(root));binaries={};
  for(const v of variants)for(const kind of ['cpu','allocation']) {
    const source='/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse/release/calcium-derivative-'+v+'-'+kind,path=root+'/'+v+'-'+kind;
    copyFileSync(source,path,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
    binaries[v+'-'+kind]={path,sha256:sha(path),bytes:statSync(path).size,size:(await command('size',[path])).trim()};
    assert.equal(sha(source),sha(path));
  }
  writeFileSync(resolve(here,'derivative-cost-binaries.json'),JSON.stringify(binaries,null,2)+'\n',{flag:'wx'});
} else binaries=json('derivative-cost-binaries.json');
for(const b of Object.values(binaries))assert.equal(sha(b.path),b.sha256);
const started=new Date().toISOString();
for(const v of variants)assert(Date.parse(json('results/derivative-cost-'+v+'-build.json').finished)<=Date.parse(started));
if(mode==='allocation')assert(Date.parse(json('derivative-cost-cpu-summary.json').finished)<=Date.parse(started));
const path=resolve(here,'results/derivative-cost-'+mode+'.jsonl');writeFileSync(path,'',{flag:'wx'});
const median=a=>{const v=[...a].sort((x,y)=>x-y);return(v[Math.floor((v.length-1)/2)]+v[Math.floor(v.length/2)])/2;};
let seed=9261;const random=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
const summaries=[];
async function run(variant,kind,degree,order,lifecycle,iterations) {
  const begin=new Date().toISOString();
  const g=JSON.parse(await command('taskset',['-c','6',binaries[variant+'-'+mode].path,variant,String(kind),String(degree),String(order),lifecycle,String(iterations)]));
  const end=new Date().toISOString();
  for(const[k,v]of Object.entries({mode,variant,kind,degree,order,lifecycle,iterations}))assert.equal(g[k],v);
  assert(g.elapsed_ns>0);
  if(mode==='cpu')for(const k of ['requests','requested_bytes','live_delta','peak_delta'])assert.equal(g[k],0);
  return {started:begin,finished:end,...g};
}
for(let kind=0;kind<2;kind++)for(const degree of [1,3,8,24])for(const order of [0,1,3,24,128])
for(const lifecycle of ['retained_curve','fresh_curve']) {
  const pilots=[];for(const v of variants)pilots.push(await run(v,kind,degree,order,lifecycle,2));
  const iterations=mode==='cpu'?Math.max(2,Math.min(5000,Math.ceil(6e6/Math.max(...pilots.map(v=>v.elapsed_ns/v.iterations))))):8;
  const rows=[],blocks=mode==='cpu'?12:3;
  for(let block=0;block<blocks;block++) {
    const orderOfRuns=mode==='cpu'?(block%2?['candidate','baseline','baseline','candidate']:['baseline','candidate','candidate','baseline']):variants;
    for(const v of orderOfRuns){const row={block,...await run(v,kind,degree,order,lifecycle,iterations)};rows.push(row);appendFileSync(path,JSON.stringify(row)+'\n');}
  }
  const summary={kind,degree,order,lifecycle,iterations,observations:rows.length,pilots};
  if(mode==='cpu') {
    const ratios=Array.from({length:blocks},(_,b)=>{
      const clock=v=>median(rows.filter(r=>r.block===b&&r.variant===v).map(r=>r.elapsed_ns));return clock('candidate')/clock('baseline');
    });
    const bootstrap=Array.from({length:5000},()=>median(ratios.map(()=>ratios[random(blocks)]))).sort((a,b)=>a-b);
    Object.assign(summary,{pairedMedianRatio:median(ratios),pairedMedianBootstrap95:[bootstrap[125],bootstrap[4875]],
      nsPerQuery:Object.fromEntries(variants.map(v=>[v,median(rows.filter(r=>r.variant===v).map(r=>r.elapsed_ns/r.iterations))]))});
  } else summary.measurements=Object.fromEntries(variants.map(v=>[v,Object.fromEntries(['requests','requested_bytes','live_delta','peak_delta'].map(k=>{
      const values=rows.filter(r=>r.variant===v).map(r=>r[k]);return[k,{min:Math.min(...values),max:Math.max(...values)}];
    }))]));
  summaries.push(summary);console.log(JSON.stringify(summary));
}
writeFileSync(resolve(here,'derivative-cost-'+mode+'-summary.json'),JSON.stringify({mode,started,finished:new Date().toISOString(),cpu:6,binaries,summaries,
  limits:'80 groups: rational polynomial and pi-scaled nonconstant-denominator curves, four degrees, five derivative orders, retained/fresh curve lifecycles. Each process prechecks every result against BigRational Bernstein expansion/direct monomial derivatives and the standard quotient formula, then warms eight queries. Fresh curve clones prebuilt controls/weights and builds its power basis, not cold scalar construction. Timed queries include public derivative-vector construction and dropping. Generic parameter is exactly 1/3; no endpoint-helper CPU claim. Uninstrumented CPU, separately counted Rust allocation requests/bytes/live/peak; not RSS, stacks or allocator/native overhead. Per-group bootstrap estimates are not multiplicity-adjusted or universal. No production retention decision at this checkpoint.'},null,2)+'\n',{flag:'wx'});
