import './verify-sign-work-cost-checkpoint.mjs';
import { readFileSync, readdirSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here=dirname(fileURLToPath(import.meta.url));
const read=p=>readFileSync(resolve(here,p),'utf8');
const json=p=>JSON.parse(read(p));
const hash=p=>createHash('sha256').update(readFileSync(resolve(here,p))).digest('hex');
const manifest=json('root-exp-reuse-experiment.json');
for(const [p,h] of Object.entries({...manifest.sourceHashes,...manifest.evidenceHashes})) assert.equal(hash(p),h,p);
for(const [tag,code] of Object.entries(manifest.expectedCaptureCodes)) {
  const r=json(`results/${tag}.json`); assert.equal(r.code,code,tag); assert.equal(r.signal,null,tag);
}
const base=json('baseline-hyperreal.json');
for(const f of base.files) if(!manifest.trial.changed.includes(f.path))
  assert.equal(hash(`${manifest.trial.directory}/${f.path}`),f.sha256,f.path);
function files(path,prefix='') {
  return readdirSync(resolve(here,path,prefix),{withFileTypes:true}).flatMap(f=>
    f.isDirectory()?files(path,`${prefix}${f.name}/`):[`${prefix}${f.name}`]);
}
assert.deepEqual(files(manifest.trial.directory).sort(),[...base.files.map(f=>f.path),...manifest.trial.added].sort());
const inv=json('inventory.json'),coverage=json('coverage.json');
for(const source of inv.sources) {
  const selected=source.files.filter(f=>new RegExp(source.repo==='calcium'?'^ca/[^/]+[.]c$':'^src/ca/[^/]+[.]c$').test(f.path));
  assert.equal(selected.length,manifest.coverage.scalarTopLevelC[source.repo]);
  for(const f of selected) {
    const c=coverage.find(c=>c.repo===source.repo&&c.path===f.path);
    assert.deepEqual(c?.ranges,[[1,f.lines]],f.path);
    assert.equal(hash(`../../../../exact-real-references/${source.repo}/${f.path}`),f.sha256,f.path);
  }
}
const native=read('results/randtest-canonical-native.stdout');
assert(native.includes('wrong_one input=2/2 canonical=1'));
assert(native.endsWith('cases=10000 canonical_controls=10000 noncanonical=2221 wrong_one=107\n'));
assert.equal(read('results/randtest-canonical-memcheck.stdout'),native);
assert(read('results/randtest-canonical-compile.stderr').includes('deprecated'));
for(const tag of ['randtest-canonical-memcheck','root-exp-reuse-memcheck-deep-fresh']) {
  const text=read(`results/${tag}.stderr`);
  assert(text.includes('ERROR SUMMARY: 0 errors'));
  assert(text.includes('All heap blocks were freed') ||
    /definitely lost: 0 bytes/.test(text)&&/indirectly lost: 0 bytes/.test(text)&&/possibly lost: 0 bytes/.test(text));
}
const memRow=JSON.parse(read('results/root-exp-reuse-memcheck-deep-fresh.stdout'));
assert.deepEqual([memRow.case,memRow.lifecycle,memRow.iterations,memRow.outcomes],['identity-128','fresh',100,[100,0,0]]);
function testNames(tag,count) {
  const text=read(`results/${tag}.stdout`);
  assert(text.includes(`test result: ok. ${count} passed; 0 failed; 0 ignored; 0 measured; 0 filtered out;`));
  const names=[...text.matchAll(/^test (.+) \.\.\. ok$/gm)].map(m=>m[1]).sort();
  assert.equal(names.length,count); assert.equal(new Set(names).size,count); return names;
}
const debug=testNames('root-exp-reuse-debug',780),release=testNames('root-exp-reuse-release',780);
assert.deepEqual(debug,release);
assert.equal(debug.filter(n=>n.startsWith('computable::node::exp_relation_reuse_tests::')).length,8);
assert.deepEqual(debug.filter(n=>!n.startsWith('computable::node::exp_relation_reuse_tests::')),
  testNames('root-exp-sign-full-debug',772));
const first=json('results/run-root-exp-reuse-cpu.json'),mem=json('results/root-exp-reuse-memcheck-deep-fresh.json');
const confirm=json('results/run-root-exp-reuse-confirm-cpu.json');
assert(Date.parse(first.started)<Date.parse(mem.finished)&&Date.parse(mem.started)<Date.parse(first.finished));
assert(Date.parse(confirm.started)>Math.max(Date.parse(first.finished),Date.parse(mem.finished)));
const median=x=>{const a=[...x].sort((a,b)=>a-b);return(a[Math.floor((a.length-1)/2)]+a[Math.floor(a.length/2)])/2;};
const benchmarks=[];
for(const stem of ['root-exp-reuse-cpu','root-exp-reuse-confirm-cpu','root-exp-reuse-confirm-alloc']) {
  const mode=stem.endsWith('alloc')?'alloc':'cpu', blocks=mode==='cpu'?12:3;
  const rows=read(`results/${stem}.jsonl`).trim().split('\n').map(JSON.parse),report=json(`${stem}-summary.json`);
  assert.equal(rows.length,24*blocks*6); assert.equal(report.summaries.length,24); assert.equal(report.cpu,6);
  let seed=17911;
  const random=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
  const variants=['baseline','sign','reuse'];
  for(const kind of ['identity','unresolved']) for(const depth of [1,8,32,128]) for(const life of ['fresh','warm-pair','warm-difference']) {
    const name=`${kind}-${depth}`,g=rows.filter(r=>r.case===name&&r.lifecycle===life);
    const summaries=report.summaries.filter(s=>s.name===name&&s.lifecycle===life);assert.equal(summaries.length,1);
    const s=summaries[0];assert.equal(g.length,blocks*6);assert.equal(s.observations,g.length);
    for(let block=0;block<blocks;block++) {
      const b=g.filter(r=>r.block===block),order=variants.map((_,i)=>variants[(i+block)%3]);
      assert.deepEqual(b.map(r=>r.variant),[...order,...order.toReversed()]);
      for(const r of b) {
        assert.equal(r.iterations,s.iterations);assert(r.elapsed_ns>0);
        const outcome=kind==='identity'&&r.variant!=='baseline'?0:2;
        assert.deepEqual(r.outcomes,[0,1,2].map(i=>i===outcome?r.iterations:0));
        if(mode==='cpu')assert.deepEqual([r.alloc_calls,r.allocated_bytes],[0,0]);
        else assert.equal(r.iterations,life==='fresh'?10:100);
      }
    }
    for(const control of ['baseline','sign']) {
      const ratios=Array.from({length:blocks},(_,block)=>{
        const v=name=>median(g.filter(r=>r.block===block&&r.variant===name).map(r=>r.elapsed_ns));
        return v('reuse')/v(control);
      });
      assert.equal(s.comparisons[control].pairedMedianRatio,median(ratios));
      const boot=Array.from({length:5000},()=>median(ratios.map(()=>ratios[random(ratios.length)]))).sort((a,b)=>a-b);
      assert.deepEqual(s.comparisons[control].pairedMedianBootstrap95,[boot[125],boot[4875]]);
    }
    for(const v of variants) for(const [key,raw] of [['ns','elapsed_ns'],['alloc_calls','alloc_calls'],['allocated_bytes','allocated_bytes']])
      assert.equal(s.measurements[v][key],median(g.filter(r=>r.variant===v).map(r=>r[raw]/r.iterations)));
  }
  benchmarks.push({stem,observations:rows.length,accepted:stem!=='root-exp-reuse-cpu',
    fileBytesVsBaseline:report.binaries.reuse.bytes-report.binaries.baseline.bytes,
    fileBytesVsSign:report.binaries.reuse.bytes-report.binaries.sign.bytes});
}
console.log(JSON.stringify({checkpoint:'scalar source closure and weak-key proof reuse',testsPerProfile:780,
  randtest:manifest.randtest,benchmarks,status:manifest.status,limits:manifest.limits}));
