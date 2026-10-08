import './verify-sign-consumer-checkpoint.mjs';
import { readFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here=dirname(fileURLToPath(import.meta.url));
const read=p=>readFileSync(resolve(here,p),'utf8');
const json=p=>JSON.parse(read(p));
const manifest=json('sign-work-cost-experiment.json');
for(const [p,hash] of Object.entries({...manifest.sourceHashes,...manifest.evidenceHashes}))
  assert.equal(createHash('sha256').update(readFileSync(resolve(here,p))).digest('hex'),hash,p);
function capture(tag,code=0) {
  const result=json(`results/${tag}.json`); assert.equal(result.code,code,tag); assert.equal(result.signal,null,tag);
  return read(`results/${tag}.stdout`);
}
capture('sign-consumer-hypercurve-baseline-debug',101);
assert(read('results/sign-consumer-hypercurve-baseline-debug.stderr').includes('Disk quota exceeded'));
const consumer=[];
for(const [v,tag] of [['baseline','sign-consumer-hypercurve-baseline-debug-storage-retry'],['sign','sign-consumer-hypercurve-sign-debug']]) {
  const text=capture(tag), results=[]; let current;
  for(const line of text.split('\n')) {
    const start=/^running (\d+) tests?$/.exec(line);
    if(start) { assert(!current); current={expected:+start[1],names:[]}; }
    const test=/^test (.+) \.\.\. (ok|ignored)(?:, (.*))?$/.exec(line);
    if(test) { assert(current); current.names.push([test[1],test[2],test[3]||'']); }
    const end=/^test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;/.exec(line);
    if(end) {
      assert(current); const [passed,failed,ignored,measured,filtered]=end.slice(1).map(Number);
      assert.deepEqual([failed,measured,filtered],[0,0,0]); assert.equal(passed+ignored,current.expected);
      assert.equal(current.names.length,current.expected); assert.equal(new Set(current.names.map(n=>n[0])).size,current.expected);
      assert.equal(current.names.filter(n=>n[1]==='ok').length,passed);
      results.push({passed,ignored,names:current.names.sort((a,b)=>a[0].localeCompare(b[0]))}); current=undefined;
    }
  }
  assert(!current); assert.equal(results.length,45);
  assert.equal(results.reduce((n,s)=>n+s.passed,0),1761); assert.equal(results.reduce((n,s)=>n+s.ignored,0),9);
  const stderr=read(`results/${tag}.stderr`);
  const ran=['hypercurve',...[...stderr.matchAll(/^\s+Running tests\/([^ ]+)\.rs /gm)].map(m=>m[1])].sort();
  const metadata=json(`results/sign-consumer-metadata-all-features-${v}.stdout`);
  const expected=metadata.packages.find(p=>p.name==='hypercurve').targets
    .filter(t=>t.kind.includes('lib')||t.kind.includes('test')).map(t=>t.name).sort();
  assert.deepEqual(ran,expected); consumer.push(results);
}
assert.deepEqual(consumer[0],consumer[1],'Hypercurve test membership differs');
const median=values=>{const a=[...values].sort((x,y)=>x-y);return(a[Math.floor((a.length-1)/2)]+a[Math.floor(a.length/2)])/2;};
const benchmarks=[];
for(const mode of ['cpu','alloc']) {
  capture(`run-root-exp-opaque-${mode}`);
  for(const v of ['baseline','sign']) capture(`root-exp-opaque-${v}-build-${mode}`);
  const summary=json(`root-exp-opaque-${mode}-summary.json`);
  const rows=read(`results/root-exp-opaque-${mode}.jsonl`).trim().split('\n').map(JSON.parse);
  const blocks=mode==='cpu'?12:3; assert.equal(rows.length,16*blocks*4);
  assert.equal(summary.cpu,6); assert.equal(summary.mode,mode); assert.equal(summary.phase,'query');
  assert.equal(summary.summaries.length,16);
  assert(Date.parse(summary.finished)>=Date.parse(summary.started));
  for(const v of ['baseline','sign']) {assert(/^[a-f0-9]{64}$/.test(summary.binaries[v].sha256));assert(summary.binaries[v].bytes>0);}
  let seed=17911;
  const random=n=>{seed=(Math.imul(seed,1664525)+1013904223)>>>0;return seed%n;};
  for(const kind of ['identity','unresolved']) for(const depth of [1,8,32,128]) for(const life of ['warm-pair','warm-difference']) {
    const name=`${kind}-${depth}`, group=rows.filter(r=>r.case===name&&r.lifecycle===life);
    const matching=summary.summaries.filter(s=>s.name===name&&s.lifecycle===life); assert.equal(matching.length,1);
    const s=matching[0]; assert.equal(group.length,blocks*4); assert.equal(s.observations,group.length);
    assert(Number.isSafeInteger(s.iterations)&&s.iterations>=20&&s.iterations<=100000);
    if(mode==='alloc')assert.equal(s.iterations,100);
    const ratios=[];
    for(let block=0;block<blocks;block++) {
      const r=group.filter(r=>r.block===block);
      assert.deepEqual(r.map(r=>r.variant),block%2?['sign','baseline','baseline','sign']:['baseline','sign','sign','baseline']);
      for(const row of r) {
        assert.equal(row.iterations,s.iterations);assert(Number.isFinite(row.elapsed_ns)&&row.elapsed_ns>0);
        const outcome=kind==='identity'&&row.variant==='sign'?0:2;
        assert.deepEqual(row.outcomes,[0,1,2].map(i=>i===outcome?s.iterations:0));
        if(mode==='cpu') {assert.equal(row.alloc_calls,0);assert.equal(row.allocated_bytes,0);}
        else {assert(Number.isSafeInteger(row.alloc_calls)&&row.alloc_calls>=0);assert(Number.isSafeInteger(row.allocated_bytes)&&row.allocated_bytes>=0);}
      }
      ratios.push(median(r.filter(r=>r.variant==='sign').map(r=>r.elapsed_ns))/median(r.filter(r=>r.variant==='baseline').map(r=>r.elapsed_ns)));
    }
    assert.equal(s.pairedMedianRatio,median(ratios));
    const bootstrap=Array.from({length:5000},()=>median(ratios.map(()=>ratios[random(ratios.length)]))).sort((a,b)=>a-b);
    assert.deepEqual(s.pairedMedianBootstrap95,[bootstrap[125],bootstrap[4875]]);
    for(const v of ['baseline','sign'])for(const [key,raw]of[['ns','elapsed_ns'],['alloc_calls','alloc_calls'],['alloc_bytes','allocated_bytes']])
      assert.equal(s[`${v}_${key}`],median(group.filter(r=>r.variant===v).map(r=>r[raw]/s.iterations)));
    if(mode==='alloc'&&kind==='unresolved'&&life==='warm-difference') {
      assert.equal(s.baseline_alloc_calls,s.sign_alloc_calls);assert.equal(s.baseline_alloc_bytes,s.sign_alloc_bytes);
    }
  }
  benchmarks.push({mode,observations:rows.length,driverFileByteDelta:summary.binaries.sign.bytes-summary.binaries.baseline.bytes});
}
const cpu=json('root-exp-opaque-cpu-summary.json').summaries.find(s=>s.name==='unresolved-128'&&s.lifecycle==='warm-pair');
const alloc=json('root-exp-opaque-alloc-summary.json').summaries.find(s=>s.name==='unresolved-128'&&s.lifecycle==='warm-pair');
assert(cpu.pairedMedianBootstrap95[0]>4);assert.deepEqual([alloc.baseline_alloc_calls,alloc.sign_alloc_calls,alloc.baseline_alloc_bytes,alloc.sign_alloc_bytes],[25,39,1768,11220]);
console.log(JSON.stringify({checkpoint:'completed consumer gates and opaque work costs',hypercurveTestsPerVariant:1761,
  ignoredHypercurveTestsPerVariant:9,totalConsumerTestsPerVariant:3309,benchmarks,
  deepUnresolvedPairedRatio:cpu.pairedMedianRatio,status:manifest.status,limits:manifest.limits}));
