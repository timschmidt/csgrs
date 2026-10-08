import { spawn } from 'node:child_process';
import { readFileSync, writeFileSync, appendFileSync, copyFileSync, constants, statSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here=dirname(fileURLToPath(import.meta.url)), mode=process.argv[2];
assert(['cpu','allocation'].includes(mode));
const json=p=>JSON.parse(readFileSync(resolve(here,p),'utf8'));
const sha=p=>createHash('sha256').update(readFileSync(resolve(here,p))).digest('hex');
async function command(file,args) {
  return new Promise((ok,fail)=>{
    const c=spawn(file,args,{stdio:['ignore','pipe','pipe']});let out='',err='';
    c.stdout.on('data',s=>out+=s);c.stderr.on('data',s=>err+=s);c.on('error',fail);
    c.on('close',code=>code===0?ok(out):fail(Error(`${file} ${args.join(' ')}: ${code}: ${err}`)));
  });
}
const retained=json('retained-monic.json'), prior=json('matrix-solve-experiment.json');
for(const[p,h]of Object.entries(retained.liveSources))assert.equal(sha(`${retained.frozenSnapshot}/${p}`),h);
for(const[p,h]of Object.entries(prior.candidateSources))assert.equal(sha(`rank-dominance-trial/${p}`),h);
const variants=['baseline','candidate'];
for(const v of variants)assert.equal(json(`results/rank-cost-${v}-build.json`).code,0);
let binaries;
if(mode==='cpu') {
  const root=process.argv[3];assert(/^\/tmp\/calcium-rank-costs\.[a-zA-Z0-9]+$/.test(root));binaries={};
  for(const v of variants)for(const kind of ['cpu','allocation']) {
    const source=`/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse/release/calcium-rank-${v}-${kind}`,path=`${root}/${v}-${kind}`;
    copyFileSync(source,path,constants.COPYFILE_EXCL|constants.COPYFILE_FICLONE);
    binaries[`${v}-${kind}`]={path,sha256:sha(path),bytes:statSync(path).size,size:(await command('size',[path])).trim()};
    assert.equal(sha(source),sha(path));
  }
  writeFileSync(resolve(here,'rank-cost-binaries.json'),JSON.stringify(binaries,null,2)+'\n',{flag:'wx'});
} else binaries=json('rank-cost-binaries.json');
for(const b of Object.values(binaries))assert.equal(sha(b.path),b.sha256);
const started=new Date().toISOString();
for(const v of variants)assert(Date.parse(json(`results/rank-cost-${v}-build.json`).finished)<=Date.parse(started));
if(mode==='allocation')assert(Date.parse(json('rank-cost-cpu-summary.json').finished)<=Date.parse(started));
const path=resolve(here,`results/rank-cost-${mode}.jsonl`);writeFileSync(path,'',{flag:'wx'});
const median=a=>{const v=[...a].sort((x,y)=>x-y);return(v[Math.floor((v.length-1)/2)]+v[Math.floor(v.length/2)])/2;};
let seed=5233;const random=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
const summaries=[];
async function run(variant,which,width,lifecycle,iterations) {
  const begin=new Date().toISOString();
  const g=JSON.parse(await command('taskset',['-c','6',binaries[`${variant}-${mode}`].path,variant,String(which),String(width),lifecycle,String(iterations)]));
  const end=new Date().toISOString();
  assert.equal(g.mode,mode);assert.equal(g.variant,variant);assert.equal(g.case,which);assert.equal(g.width,width);
  assert.equal(g.lifecycle,lifecycle);assert.equal(g.iterations,iterations);assert(g.elapsed_ns>0);
  const known=[0,5,6].includes(which)||(variant==='candidate'&&[1,3].includes(which));
  assert.equal(g.known,known?iterations:0);
  if(mode==='cpu')for(const k of ['requests','requested_bytes','live_delta','peak_delta'])assert.equal(g[k],0);
  return {started:begin,finished:end,...g};
}
for(let which=0;which<7;which++)for(const width of [4,8,16,32])for(const lifecycle of ['fresh_problem','retained_analysis']) {
  const pilots=[];for(const v of variants)pilots.push(await run(v,which,width,lifecycle,4));
  const iterations=mode==='cpu'?Math.max(4,Math.min(5000,Math.ceil(8e6/Math.max(...pilots.map(v=>v.elapsed_ns/v.iterations))))):16;
  const rows=[],blocks=mode==='cpu'?12:3;
  for(let block=0;block<blocks;block++) {
    const order=mode==='cpu'?(block%2?['candidate','baseline','baseline','candidate']:['baseline','candidate','candidate','baseline']):variants;
    for(const v of order){const row={block,...await run(v,which,width,lifecycle,iterations)};rows.push(row);appendFileSync(path,JSON.stringify(row)+'\n');}
  }
  const summary={case:which,width,lifecycle,iterations,observations:rows.length,pilots};
  if(mode==='cpu') {
    const ratios=Array.from({length:blocks},(_,b)=>{
      const clock=v=>median(rows.filter(r=>r.block===b&&r.variant===v).map(r=>r.elapsed_ns));return clock('candidate')/clock('baseline');
    });
    const bootstrap=Array.from({length:5000},()=>median(ratios.map(()=>ratios[random(blocks)]))).sort((a,b)=>a-b);
    Object.assign(summary,{pairedMedianRatio:median(ratios),pairedMedianBootstrap95:[bootstrap[125],bootstrap[4875]],
      nsPerQuery:Object.fromEntries(variants.map(v=>[v,median(rows.filter(r=>r.variant===v).map(r=>r.elapsed_ns/r.iterations))]))});
  } else {
    summary.measurements=Object.fromEntries(variants.map(v=>[v,Object.fromEntries(['requests','requested_bytes','live_delta','peak_delta'].map(k=>{
      const values=rows.filter(r=>r.variant===v).map(r=>r[k]);return[k,{min:Math.min(...values),max:Math.max(...values)}];
    }))]));
  }
  summaries.push(summary);console.log(JSON.stringify(summary));
}
writeFileSync(resolve(here,`rank-cost-${mode}-summary.json`),JSON.stringify({mode,started,finished:new Date().toISOString(),cpu:6,binaries,summaries,
  limits:'Seven affine-rank patterns/four widths/two lifecycles. Fresh_problem constructs and analyzes a new Problem from a shared prechecked tiny scalar; retained_analysis reuses the ProblemAnalysis. Eight warmup queries, not cold scalar/process timing. New rank decisions are different work from baseline Unknown. CPU uses uninstrumented binaries; separate allocation counters measure requested Rust bytes, not native/allocator overhead, RSS or stacks. Bootstrap intervals are per-group host/corpus estimates, not multiplicity-adjusted or universal bounds. No production transfer or broader state/consumer qualification.'},null,2)+'\n',{flag:'wx'});
