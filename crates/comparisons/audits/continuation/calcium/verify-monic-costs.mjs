import './verify-polynomial-closure.mjs';
import { readFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here=dirname(fileURLToPath(import.meta.url)),workspace=resolve(here,'../../../..');
const read=p=>readFileSync(resolve(here,p),'utf8'),json=p=>JSON.parse(read(p));
const sha=p=>createHash('sha256').update(readFileSync(resolve(here,p))).digest('hex');
const manifest=process.argv.includes('--draft-monic-costs')?(await import('./bind-monic-costs.mjs')).manifest:json('monic-cost-experiment.json');
for(const [p,h]of Object.entries(manifest.files))assert.equal(sha(p),h,p);
for(const [p,h]of Object.entries(manifest.binaries))assert.equal(sha(p),h,p);
assert.equal(Object.keys(manifest.binaries).length,7);
const coverage=json('coverage.json'),inventory=json('inventory.json'),selection=json('monic-cost-read-selection.json');
assert.equal(selection.length,46);assert.equal(new Set(selection.map(v=>`${v.repo}:${v.path}`)).size,46);
assert.deepEqual(manifest.reads,selection.map(e=>coverage.find(c=>c.repo===e.repo&&c.path===e.path)));
assert.equal(manifest.reads.reduce((n,v)=>n+v.ranges.reduce((s,[a,b])=>s+b-a+1,0),0),2740);
for(const v of manifest.reads){const f=inventory.sources.find(s=>s.repo===v.repo).files.find(f=>f.path===v.path);
  assert.deepEqual(v.ranges,[[1,f.lines]]);assert.equal(sha(resolve(workspace,'exact-real-references',v.repo,v.path)),f.sha256);}
for(const [repo,count]of [['calcium',56],['flint',53]]){
  const files=inventory.sources.find(s=>s.repo===repo).files.filter(f=>/^(src\/)?ca_poly\/[^/]+\.c$/.test(f.path));
  assert.equal(files.length,count);
  for(const f of files)assert.deepEqual(coverage.find(c=>c.repo===repo&&c.path===f.path)?.ranges,[[1,f.lines]]);
}
const tags=['monic-cost-baseline-build','monic-cost-trial-build','monic-cost-cpu-run','monic-cost-allocation-run',
  'monic-cost-churn-run','monic-compose-compile','monic-compose-native','monic-compose-memcheck',
  'monic-fractionfree-debug','monic-fractionfree-release','monic-fractionfree-memcheck','monic-candidate-clippy','monic-candidate-wasm'];
assert.deepEqual(manifest.gates,tags);
function gate(tag){const g=json(`results/${tag}.json`);assert.equal(g.code,0,tag);assert.equal(g.signal,null);assert.equal(g.tag,tag);
  assert(Date.parse(g.finished)>=Date.parse(g.started));
  assert.equal(g.cwd,resolve(here,tag.startsWith('monic-candidate-')?'polynomial-monic-trial/hypersolve':'../../..'));
  return {...g,stdout:read(`results/${tag}.stdout`),stderr:read(`results/${tag}.stderr`)};}
for(const tag of tags)gate(tag);
const uncertain=new Set([7,8,15,16,17,19,20,21,22,23,24,25,26]);
function row(r,mode,kind,code,lifecycle,iterations){
  assert.equal(r.mode,mode);assert.equal(r.kind,kind);assert.equal(r.code,code);assert.equal(r.lifecycle,lifecycle);assert.equal(r.iterations,iterations);
  assert.equal(r.known,kind===2&&uncertain.has(code)?0:iterations);
  assert.equal(r.degree,[code%3,Math.floor(code/3)%3,Math.floor(code/9)%3].filter(Boolean).length);
  assert(Number.isSafeInteger(r.elapsed_ns)&&r.elapsed_ns>0);
  for(const field of ['requests','requested_bytes','peak_delta'])assert(Number.isSafeInteger(r[field])&&r[field]>=0);
  assert(Number.isSafeInteger(r.live_delta));
  if(mode==='cpu')for(const field of ['requests','requested_bytes','live_delta','peak_delta'])assert.equal(r[field],0);
}
const median=input=>{const v=input.slice().sort((a,b)=>a-b);return(v[Math.floor((v.length-1)/2)]+v[Math.floor(v.length/2)])/2;};
const lines=p=>read(p).trimEnd().split('\n').map(v=>JSON.parse(v));
let seed=3109;
const random=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
const cpu=json('monic-cost-cpu-summary.json');
assert.equal(cpu.mode,'cpu');assert.equal(cpu.cpu,6);assert.equal(cpu.summaries.length,162);
const cpuStart=Date.parse(cpu.started),cpuEnd=Date.parse(cpu.finished);
assert(cpuEnd>cpuStart);
const cpuGate=gate('monic-cost-cpu-run');
assert(Date.parse(cpuGate.started)<=cpuStart&&cpuEnd<=Date.parse(cpuGate.finished));
for(const tag of tags.filter(t=>t!=='monic-cost-cpu-run')){
  const g=gate(tag);assert(Date.parse(g.finished)<=cpuStart||Date.parse(g.started)>=cpuEnd,`CPU overlap ${tag}`);
}
for(const variant of ['baseline','trial'])assert(Date.parse(gate(`monic-cost-${variant}-build`).finished)<=cpuStart);
const binaries=json('monic-cost-binaries.json');assert.deepEqual(cpu.binaries,binaries);
for(const b of Object.values(binaries)){assert.equal(sha(b.path),b.sha256);assert.equal(readFileSync(b.path).length,b.bytes);}
const cpuRows=lines('results/monic-cost-cpu.jsonl');assert.equal(cpuRows.length,7776);
const allocation=json('monic-cost-allocation-summary.json'),allocRows=lines('results/monic-cost-allocation.jsonl');
assert.equal(allocation.mode,'allocation');assert.equal(allocation.summaries.length,162);assert.equal(allocRows.length,972);
assert.deepEqual(allocation.binaries,binaries);assert(Date.parse(allocation.started)>=cpuEnd);
let group=0,requestSavings=0,peakSavings=0;
for(let kind=0;kind<3;kind++)for(let code=0;code<27;code++)for(const lifecycle of ['fresh','retained']){
  const s=cpu.summaries[group],a=allocation.summaries[group];
  assert.deepEqual([s.kind,s.code,s.lifecycle,s.observations],[kind,code,lifecycle,48]);
  assert.equal(s.pilots.length,2);for(const p of s.pilots)row(p,'cpu',kind,code,lifecycle,10);
  assert.equal(s.iterations,Math.max(10,Math.min(10000,Math.ceil(8e6/Math.max(...s.pilots.map(v=>v.elapsed_ns/v.iterations))))));
  const rows=cpuRows.slice(group*48,(group+1)*48);
  for(let block=0;block<12;block++){
    const order=block%2?['trial','baseline','baseline','trial']:['baseline','trial','trial','baseline'];
    for(let pos=0;pos<4;pos++){const r=rows[block*4+pos];assert.equal(r.block,block);assert.equal(r.variant,order[pos]);row(r,'cpu',kind,code,lifecycle,s.iterations);}
  }
  const ratios=Array.from({length:12},(_,block)=>{
    const r=rows.filter(v=>v.block===block),t=variant=>median(r.filter(v=>v.variant===variant).map(v=>v.elapsed_ns));
    return t('trial')/t('baseline');
  });
  const bootstrap=Array.from({length:5000},()=>median(ratios.map(()=>ratios[random(12)]))).sort((a,b)=>a-b);
  assert.equal(s.pairedMedianRatio,median(ratios));assert.deepEqual(s.pairedMedianBootstrap95,[bootstrap[125],bootstrap[4875]]);
  for(const variant of ['baseline','trial'])assert.equal(s.nsPerQuery[variant],median(rows.filter(v=>v.variant===variant).map(v=>v.elapsed_ns/v.iterations)));
  assert.deepEqual([a.kind,a.code,a.lifecycle,a.iterations,a.observations],[kind,code,lifecycle,64,6]);
  assert.equal(a.pilots.length,2);for(const p of a.pilots)row(p,'allocation',kind,code,lifecycle,10);
  const ar=allocRows.slice(group*6,(group+1)*6);
  for(let i=0;i<6;i++){assert.equal(ar[i].variant,i%2?'trial':'baseline');assert.equal(ar[i].block,Math.floor(i/2));row(ar[i],'allocation',kind,code,lifecycle,64);}
  for(const variant of ['baseline','trial'])for(const field of ['requests','requested_bytes','live_delta','peak_delta']){
    const values=ar.filter(v=>v.variant===variant).map(v=>v[field]);
    assert.deepEqual(a.measurements[variant][field],{min:Math.min(...values),max:Math.max(...values)});
    assert.equal(Math.min(...values),Math.max(...values)); // observed corpus repeatability, not a universal cache invariant
  }
  for(const field of ['requests','requested_bytes','peak_delta'])assert(a.measurements.trial[field].max<=a.measurements.baseline[field].min);
  assert.deepEqual(a.measurements.trial.live_delta,a.measurements.baseline.live_delta);
  if(a.measurements.trial.requested_bytes.max<a.measurements.baseline.requested_bytes.min)requestSavings++;
  if(a.measurements.trial.peak_delta.max<a.measurements.baseline.peak_delta.min)peakSavings++;
  group++;
}
assert.equal(requestSavings,56);assert.equal(peakSavings,44);
const churn=lines('results/monic-cost-churn.jsonl');assert.equal(churn.length,108);
assert.equal(json('monic-cost-churn-summary.json').observations,108);let index=0;
for(const kind of [0,1,2])for(const code of [5,14,26])for(const lifecycle of ['fresh','retained'])
for(const iterations of [1,100,1000])for(const variant of ['baseline','trial']){
  const r=churn[index++];assert.equal(r.variant,variant);row(r,'allocation',kind,code,lifecycle,iterations);
  const first=churn.find(v=>v.kind===kind&&v.code===code&&v.lifecycle===lifecycle&&v.variant===variant&&v.iterations===1);
  assert.equal(r.live_delta,first.live_delta);assert(r.live_delta<=384);if(lifecycle==='retained')assert.equal(r.live_delta,0);
}
function memcheck(tag,native){const g=gate(tag);assert.equal(g.command,'valgrind');
  for(const flag of ['--leak-check=full','--show-leak-kinds=all','--errors-for-leak-kinds=definite,indirect,possible','--error-exitcode=99'])assert(g.args.includes(flag));
  assert.match(g.stderr,/ERROR SUMMARY: 0 errors from 0 contexts/);
  if(native)assert.match(g.stderr,/in use at exit: 0 bytes in 0 blocks/);
  else for(const kind of ['definitely','indirectly','possibly'])assert.match(g.stderr,new RegExp(`${kind} lost: 0 bytes in 0 blocks`));
}
const native=gate('monic-compose-native').stdout.trimEnd().split('\n');
assert.equal(native.shift(),'mode,outer_length,inner_length,shape,checks,failures');
assert.deepEqual(JSON.parse(native.pop()),{suite:'polynomial-compose',cases:384,checks:4608,failures:0});
assert.deepEqual(native.map(r=>r.split(',').map(Number)),[0,1,2,3].flatMap(m=>[0,1,2,7,8,21].flatMap(a=>[0,1,2,5].flatMap(b=>[0,1,2,3].map(s=>[m,a,b,s,12,0])))));
assert.equal(gate('monic-compose-native').stdout,gate('monic-compose-memcheck').stdout);memcheck('monic-compose-memcheck',true);
const prs=gate('monic-fractionfree-debug').stdout.trimEnd().split('\n');
assert.equal(prs.shift(),'kind,code,floor,expected_gcd_degree,outcome,result_degree');
assert.deepEqual(JSON.parse(prs.pop()),{suite:'monic-fractionfree',cases:162,known:128,unknown:34});
const prsUnknown=new Set([5,7,8,11,13,14,15,16,17,19,20,21,22,23,24,25,26]);index=0;
for(let kind=0;kind<3;kind++)for(let code=0;code<27;code++)for(const floor of [-32,-512]){
  const r=prs[index++].split(','),degree=[code%3,Math.floor(code/3)%3,Math.floor(code/9)%3].filter(v=>v===2).length;
  assert.deepEqual(r.slice(0,4).map(Number),[kind,code,floor,degree]);
  assert.deepEqual(r.slice(4),kind===2&&prsUnknown.has(code)?['Unknown','-1']:['Known',String(degree)]);
}
assert.equal(prs.length,162);assert([...uncertain].every(v=>prsUnknown.has(v)));
for(const phase of ['release','memcheck'])assert.equal(gate(`monic-fractionfree-${phase}`).stdout,gate('monic-fractionfree-debug').stdout);
assert(gate('monic-fractionfree-release').args.includes('--release'));memcheck('monic-fractionfree-memcheck',false);
for(const flag of ['clippy','--offline','--locked','--all-features','--all-targets','-D','warnings'])assert(gate('monic-candidate-clippy').args.includes(flag));
for(const flag of ['check','--offline','--locked','--all-features','--lib','wasm32-unknown-unknown'])assert(gate('monic-candidate-wasm').args.includes(flag));
console.log(JSON.stringify({checkpoint:'monic costs and polynomial implementation closure',readFiles:46,readLines:2740,
  completeTopLevelPolynomialFiles:{calcium:56,flint:53},cpuObservations:7776,cpuGroups:162,
  cpuRatioRange:[Math.min(...cpu.summaries.map(v=>v.pairedMedianRatio)),Math.max(...cpu.summaries.map(v=>v.pairedMedianRatio))],
  allocationObservations:972,lowerAllocationGroups:requestSavings,lowerPeakGroups:peakSavings,churnObservations:108,
  nativeCompositionChecks:4608,fractionfreeCasesPerProfile:162,fractionfreeKnown:128,fractionfreeUnknown:34,
  status:manifest.status,limits:manifest.limits}));
