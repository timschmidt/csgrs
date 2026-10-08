import { spawn } from 'node:child_process';
import { readFileSync, writeFileSync, appendFileSync, copyFileSync, constants, statSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here = dirname(fileURLToPath(import.meta.url));
const mode = process.argv[2]; assert(['cpu','allocation'].includes(mode));
const json = p => JSON.parse(readFileSync(resolve(here,p),'utf8'));
const sha = p => createHash('sha256').update(readFileSync(resolve(here,p))).digest('hex');
const variants = ['baseline','trial'];
const target = '/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse/release';
async function command(file,args) {
  return new Promise((ok,fail) => {
    const child = spawn(file,args,{stdio:['ignore','pipe','pipe']}); let out='',err='';
    child.stdout.on('data',s => out += s); child.stderr.on('data',s => err += s);
    child.on('error',fail); child.on('close',code => code === 0 ? ok(out) : fail(Error(`${file} ${args.join(' ')}: ${code}: ${err}`)));
  });
}
const prior = json('polynomial-closure-experiment.json'), retained = json('retained-polynomial-facts.json');
for (const [p,h] of Object.entries(prior.candidateSources)) assert.equal(sha(`polynomial-monic-trial/${p}`),h);
for (const [p,h] of Object.entries(retained.liveSources)) assert.equal(sha(`${retained.frozenSnapshot}/${p}`),h);
for (const variant of variants) assert.equal(json(`results/monic-cost-${variant}-build.json`).code,0);
let binaries;
if (mode === 'cpu') {
  const destination = process.argv[3]; assert(/^\/tmp\/calcium-monic-costs\.[a-zA-Z0-9]+$/.test(destination));
  binaries = {};
  for (const variant of variants) for (const kind of ['cpu','allocation']) {
    const source = `${target}/calcium-monic-${variant}-${kind}`, path = `${destination}/${variant}-${kind}`;
    copyFileSync(source,path,constants.COPYFILE_EXCL | constants.COPYFILE_FICLONE);
    binaries[`${variant}-${kind}`] = {path,sha256:sha(path),bytes:statSync(path).size,size:(await command('size',[path])).trim()};
    assert.equal(sha(source),sha(path));
  }
  writeFileSync(resolve(here,'monic-cost-binaries.json'),JSON.stringify(binaries,null,2)+'\n',{flag:'wx'});
} else binaries = json('monic-cost-binaries.json');
for (const b of Object.values(binaries)) assert.equal(sha(b.path),b.sha256);
const started = new Date().toISOString();
for (const variant of variants) assert(Date.parse(json(`results/monic-cost-${variant}-build.json`).finished) <= Date.parse(started));
const path = resolve(here,`results/monic-cost-${mode}.jsonl`); writeFileSync(path,'',{flag:'wx'});
const median = a => { const v=[...a].sort((x,y)=>x-y); return (v[Math.floor((v.length-1)/2)] + v[Math.floor(v.length/2)])/2; };
let seed=3109;
const random = n => { seed=(Math.imul(seed,1664525)+1013904223)>>>0; return seed%n; };
const summaries=[];
async function run(variant,kind,code,lifecycle,iterations) {
  const g=JSON.parse(await command('taskset',['-c','6',binaries[`${variant}-${mode}`].path,String(kind),String(code),lifecycle,String(iterations)]));
  assert.equal(g.mode,mode); assert.equal(g.kind,kind); assert.equal(g.code,code);
  assert.equal(g.lifecycle,lifecycle); assert.equal(g.iterations,iterations); assert(g.elapsed_ns>0);
  const known=!(kind===2&&[7,8,15,16,17,19,20,21,22,23,24,25,26].includes(code));
  assert.equal(g.known,known?iterations:0);
  assert.equal(g.degree,[code%3,Math.floor(code/3)%3,Math.floor(code/9)%3].filter(Boolean).length);
  if(mode==='cpu') for(const key of ['requests','requested_bytes','live_delta','peak_delta']) assert.equal(g[key],0);
  return g;
}
for(let kind=0;kind<3;kind++) for(let code=0;code<27;code++) for(const lifecycle of ['fresh','retained']) {
  const rows=[],pilots=[];
  for(const variant of variants) pilots.push(await run(variant,kind,code,lifecycle,10));
  const iterations=mode==='cpu'?Math.max(10,Math.min(10000,Math.ceil(8e6/Math.max(...pilots.map(v=>v.elapsed_ns/v.iterations))))):64;
  const blocks=mode==='cpu'?12:3;
  for(let block=0;block<blocks;block++) {
    const order=mode==='cpu'?(block%2?['trial','baseline','baseline','trial']:['baseline','trial','trial','baseline']):variants;
    for(const variant of order) {
      const row={variant,block,...await run(variant,kind,code,lifecycle,iterations)};
      rows.push(row); appendFileSync(path,JSON.stringify(row)+'\n');
    }
  }
  const summary={kind,code,lifecycle,iterations,observations:rows.length,pilots};
  if(mode==='cpu') {
    const ratios=Array.from({length:blocks},(_,block)=> {
      const clock=variant=>median(rows.filter(v=>v.block===block&&v.variant===variant).map(v=>v.elapsed_ns));
      return clock('trial')/clock('baseline');
    });
    const bootstrap=Array.from({length:5000},()=>median(ratios.map(()=>ratios[random(blocks)]))).sort((a,b)=>a-b);
    Object.assign(summary,{pairedMedianRatio:median(ratios),pairedMedianBootstrap95:[bootstrap[125],bootstrap[4875]],
      nsPerQuery:Object.fromEntries(variants.map(v=>[v,median(rows.filter(r=>r.variant===v).map(r=>r.elapsed_ns/r.iterations))]))});
  } else {
    summary.measurements=Object.fromEntries(variants.map(variant=>[variant,Object.fromEntries(['requests','requested_bytes','live_delta','peak_delta'].map(key=> {
      const values=rows.filter(r=>r.variant===variant).map(r=>r[key]);
      return [key,{min:Math.min(...values),max:Math.max(...values)}];
    }))]));
  }
  summaries.push(summary); console.log(JSON.stringify(summary));
}
writeFileSync(resolve(here,`monic-cost-${mode}-summary.json`),JSON.stringify({mode,started,finished:new Date().toISOString(),cpu:6,binaries,summaries,
  limits:'81 known-factor recipes; retained includes required Vec cloning, fresh includes construction and teardown. Eight preconditioning queries, not cold-process timing. Same Some/None and degrees do not imply identical output representation or downstream proof availability. CPU clocks only from uninstrumented binaries. Requested heap/peak deltas exclude allocator overhead, native allocations, RSS and thread stacks. Paired bootstrap intervals are host/corpus observations, not universal guarantees.'},null,2)+'\n',{flag:'wx'});
