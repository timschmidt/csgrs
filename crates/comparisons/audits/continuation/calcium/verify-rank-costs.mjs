import './verify-matrix-solves.mjs';
import { readFileSync, statSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here=dirname(fileURLToPath(import.meta.url)), workspace=resolve(here,'../../../..');
const read=p=>readFileSync(resolve(here,p),'utf8'),json=p=>JSON.parse(read(p));
const sha=p=>createHash('sha256').update(readFileSync(resolve(here,p))).digest('hex');
const m=process.argv.includes('--draft-rank-costs')?(await import('./bind-rank-costs.mjs')).manifest:json('rank-cost-experiment.json');
for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
for(const[p,h]of Object.entries(m.binaries))assert.equal(sha(p),h,p);
assert.equal(Object.keys(m.binaries).length,5);assert.equal(sha(m.nativeLibrary.path),m.nativeLibrary.sha256);
const retained=json('retained-monic.json');
for(const[p,ranges]of Object.entries(m.hyperReadRanges)) {
  const text=read(`${retained.frozenSnapshot}/${p}`),n=text.split('\n').length-+text.endsWith('\n');
  assert(ranges.every(([a,b])=>a>=1&&b>=a&&b<=n),p);
}
const coverage=json('coverage.json'),inventory=json('inventory.json'),selection=json('rank-cost-read-selection.json');
assert.equal(selection.length,40);assert.equal(new Set(selection.map(e=>`${e.repo}:${e.path}`)).size,40);
assert.deepEqual(m.reads,selection.map(e=>coverage.find(v=>v.repo===e.repo&&v.path===e.path)));
let readLines=0;
for(const e of m.reads) {
  const f=inventory.sources.find(s=>s.repo===e.repo).files.find(f=>f.path===e.path);
  assert.deepEqual(e.ranges,[[1,f.lines]]);readLines+=f.lines;
  assert.equal(sha(resolve(workspace,'exact-real-references',e.repo,e.path)),f.sha256);
}
assert.equal(readLines,2521);
const tags=['rank-cost-baseline-build','rank-cost-candidate-build','rank-cost-cpu-run','rank-cost-allocation-run',
  'matrix-support-native-compile','matrix-support-native','matrix-support-memcheck'];
assert.deepEqual(m.gates,tags);
function gate(tag) {
  assert(tags.includes(tag));const g=json(`results/${tag}.json`);
  assert.equal(g.tag,tag);assert.equal(g.code,0,tag);assert.equal(g.signal,null);
  assert(Date.parse(g.finished)>=Date.parse(g.started));
  return {...g,stdout:read(`results/${tag}.stdout`),stderr:read(`results/${tag}.stderr`)};
}
for(const tag of tags)gate(tag);
for(const v of ['baseline','candidate']) {
  const g=gate(`rank-cost-${v}-build`);assert.equal(g.cwd,resolve(here,`rank-cost-${v}`));
  assert.equal(g.command,'env');
  assert.deepEqual(g.args,['CARGO_TARGET_DIR=/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse',
    'CARGO_INCREMENTAL=0','CARGO_PROFILE_DEV_DEBUG=0','CARGO_BUILD_JOBS=2','cargo','build','--offline','--release','--bins']);
}
const binaries=json('rank-cost-binaries.json');
assert.deepEqual(Object.keys(binaries).sort(),['baseline-allocation','baseline-cpu','candidate-allocation','candidate-cpu']);
for(const b of Object.values(binaries)) {
  assert.equal(m.binaries[b.path],b.sha256);assert.equal(statSync(b.path).size,b.bytes);
  const line=b.size.split('\n')[1].split(/\s+/);assert.equal(line[5],b.path);
  assert.equal(+line[3],+line[0]+ +line[1]+ +line[2]);assert.equal(parseInt(line[4],16),+line[3]);
}
const median=a=>{const v=[...a].sort((x,y)=>x-y);return(v[Math.floor((v.length-1)/2)]+v[Math.floor(v.length/2)])/2;};
const variants=['baseline','candidate'],metrics=['requests','requested_bytes','live_delta','peak_delta'];
let seed=5233;const random=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
const summaries={};
for(const mode of ['cpu','allocation']) {
  const s=json(`rank-cost-${mode}-summary.json`),g=gate(`rank-cost-${mode}-run`);
  assert.equal(s.mode,mode);assert.equal(s.cpu,6);assert.deepEqual(s.binaries,binaries);
  assert.equal(g.command,'node');assert.equal(g.cwd,here);
  assert.equal(resolve(g.cwd,g.args[0]),resolve(here,'run-rank-costs.mjs'));assert.equal(g.args[1],mode);
  assert.deepEqual(g.args.slice(2),mode==='cpu'?['/tmp/calcium-rank-costs.lbsszv']:[]);
  assert.equal(g.stderr,'');assert.equal(s.summaries.length,56);
  assert.deepEqual(g.stdout.trimEnd().split('\n').map(l=>JSON.parse(l)),s.summaries);
  const begin=Date.parse(s.started),end=Date.parse(s.finished);
  assert(Date.parse(g.started)<=begin&&end<=Date.parse(g.finished)&&begin<=end);
  const rows=read(`results/rank-cost-${mode}.jsonl`).trimEnd().split('\n').map(l=>JSON.parse(l));
  assert.equal(rows.length,mode==='cpu'?2688:336);
  let index=0,group=0,last=begin;
  function validate(r,which,width,lifecycle,iterations,variant) {
    assert.equal(r.mode,mode);assert.equal(r.variant,variant);assert.equal(r.case,which);
    assert.equal(r.width,width);assert.equal(r.lifecycle,lifecycle);assert.equal(r.iterations,iterations);
    assert(Number.isSafeInteger(r.elapsed_ns)&&r.elapsed_ns>0);
    const a=Date.parse(r.started),b=Date.parse(r.finished);assert(a>=last&&a<=b&&b<=end);last=b;
    const known=[0,5,6].includes(which)||(variant==='candidate'&&[1,3].includes(which));
    assert.equal(r.known,known?iterations:0);
    for(const k of metrics)assert(Number.isSafeInteger(r[k])&&r[k]>=0);
    if(mode==='cpu')for(const k of metrics)assert.equal(r[k],0);
    else {assert(r.requests>0&&r.requested_bytes>0&&r.peak_delta>0);assert.equal(r.live_delta,0);}
  }
  for(let which=0;which<7;which++)for(const width of [4,8,16,32])for(const lifecycle of ['fresh_problem','retained_analysis']) {
    const q=s.summaries[group++];assert.deepEqual([q.case,q.width,q.lifecycle],[which,width,lifecycle]);
    assert.equal(q.pilots.length,2);
    q.pilots.forEach((r,i)=>validate(r,which,width,lifecycle,4,variants[i]));
    const iterations=mode==='cpu'?Math.max(4,Math.min(5000,Math.ceil(8e6/Math.max(...q.pilots.map(r=>r.elapsed_ns/4))))):16;
    assert.equal(q.iterations,iterations);const blocks=mode==='cpu'?12:3,local=[];
    for(let block=0;block<blocks;block++) {
      const order=mode==='cpu'?(block%2?['candidate','baseline','baseline','candidate']:['baseline','candidate','candidate','baseline']):variants;
      for(const v of order){const r=rows[index++];assert.equal(r.block,block);validate(r,which,width,lifecycle,iterations,v);local.push(r);}
    }
    assert.equal(q.observations,local.length);
    if(mode==='cpu') {
      const ratios=Array.from({length:blocks},(_,b)=>{
        const clocks=v=>median(local.filter(r=>r.block===b&&r.variant===v).map(r=>r.elapsed_ns));
        return clocks('candidate')/clocks('baseline');
      });
      const bootstrap=Array.from({length:5000},()=>median(ratios.map(()=>ratios[random(blocks)]))).sort((a,b)=>a-b);
      assert.equal(q.pairedMedianRatio,median(ratios));
      assert.deepEqual(q.pairedMedianBootstrap95,[bootstrap[125],bootstrap[4875]]);
      assert.deepEqual(q.nsPerQuery,Object.fromEntries(variants.map(v=>[v,median(local.filter(r=>r.variant===v).map(r=>r.elapsed_ns/r.iterations))])));
    } else {
      const counts=Object.fromEntries(variants.map(v=>[v,Object.fromEntries(metrics.map(k=>{
        const values=local.filter(r=>r.variant===v).map(r=>r[k]);return[k,{min:Math.min(...values),max:Math.max(...values)}];
      }))]));
      assert.deepEqual(q.measurements,counts);
      for(const v of variants)for(const k of metrics)assert.equal(counts[v][k].min,counts[v][k].max);
    }
  }
  assert.equal(index,rows.length);summaries[mode]=s;
}
// Recorded builds and memory/native controls do not overlap the CPU campaign.
const cpu=summaries.cpu;
for(const tag of tags.filter(t=>t!=='rank-cost-cpu-run')) {
  const g=gate(tag);assert(Date.parse(g.finished)<=Date.parse(cpu.started)||Date.parse(g.started)>=Date.parse(cpu.finished),tag);
}
assert(Date.parse(summaries.allocation.started)>=Date.parse(cpu.finished));
const allocationCounts={};
for(const key of ['requests','requested_bytes','peak_delta']) {
  const delta=summaries.allocation.summaries.map(q=>q.measurements.candidate[key].min-q.measurements.baseline[key].min);
  allocationCounts[key]={lower:delta.filter(v=>v<0).length,equal:delta.filter(v=>v===0).length,higher:delta.filter(v=>v>0).length};
}
assert.deepEqual(allocationCounts,{requests:{lower:0,equal:24,higher:32},requested_bytes:{lower:0,equal:24,higher:32},peak_delta:{lower:0,equal:32,higher:24}});
const raw=gate('matrix-support-native').stdout,rows=raw.trimEnd().split('\n');
assert.equal(rows.shift(),'family,kind,r,inner,c,alias,method,parameter,equality');
assert.deepEqual(JSON.parse(rows.pop()),{suite:'matrix-support',rows:1528,failures:0});assert.equal(rows.length,1528);
let index=0;const dims=[0,1,2,3,4,6,8],shapes=[[0,4,3],[3,0,4],[3,4,0],[2,3,4],[3,4,5],[5,6,3]];
for(let kind=0;kind<6;kind++) {
  for(const n of dims)for(let alias=0;alias<4;alias++)for(let method=0;method<2;method++)
    assert.equal(rows[index++],['mul',kind,n,n,n,alias,method,0,'True'].join(','));
  for(const [r,k,c]of shapes)for(let method=0;method<2;method++)
    assert.equal(rows[index++],['mul',kind,r,k,c,0,method,0,'True'].join(','));
}
for(let kind=0;kind<4;kind++)for(const n of dims)for(let alias=0;alias<2;alias++)
for(const p of [0,1,2,3,4,7,8,15,16,31])for(const family of ['pow','poly'])
  assert.equal(rows[index++],[family,kind,n,n,n,alias,0,p,'True'].join(','));
assert.equal(index,rows.length);
const memory=gate('matrix-support-memcheck');assert.equal(memory.stdout,raw);assert.equal(memory.command,'valgrind');
for(const arg of ['--vgdb=no','--tool=memcheck','--error-exitcode=97','--leak-check=full','--show-leak-kinds=all','--errors-for-leak-kinds=definite,indirect,possible'])assert(memory.args.includes(arg));
assert.match(memory.stderr,/ERROR SUMMARY: 0 errors from 0 contexts/);assert.match(memory.stderr,/in use at exit: 0 bytes in 0 blocks/);
assert.equal(memory.args.at(-1),gate('matrix-support-native').command);
assert.equal(gate('matrix-support-native-compile').args.at(-1),gate('matrix-support-native').command);
assert(m.binaries[gate('matrix-support-native').command]);
const unresolved=cpu.summaries.find(q=>q.case===4&&q.width===32&&q.lifecycle==='retained_analysis');
const demand=summaries.allocation.summaries.find(q=>q.case===4&&q.width===32&&q.lifecycle==='retained_analysis');
console.log(JSON.stringify({checkpoint:'rank costs and matrix support',completeReads:40,readLines,groups:56,cpuRows:2688,allocationRows:336,
  allocationCounts,unresolvedWidth32Retained:{pairedRatio:unresolved.pairedMedianRatio,bootstrap95:unresolved.pairedMedianBootstrap95,
    requestedBytesPerQuery:Object.fromEntries(variants.map(v=>[v,demand.measurements[v].requested_bytes.min/demand.iterations]))},
  nativeMatrixRows:1528,nativeMemory:'zero errors/all allocations freed',status:m.status,limits:m.limits}));
