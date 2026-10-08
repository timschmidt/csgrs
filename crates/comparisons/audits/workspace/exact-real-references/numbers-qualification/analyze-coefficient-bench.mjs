import {readFileSync,statSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
const dir=import.meta.dirname,root=resolve(dir,'../..');
const hash=p=>createHash('sha256').update(readFileSync(resolve(root,p))).digest('hex');
const hashes={before:'921104ff0c82ec7a36c581b4fef13e1070d7bccf2e2d4b13e6ca574b2052cdfd',after:'d61fd5051597fde9c1b457b671e1e19bcd9654a2ff433ffb8b3520cfff27136c'};
const source='961561139b341c6b805e4e4e708ee8673a2c1f0fb93a4256b83f65368d427a64';
assert.equal(hash('exact-real-references/numbers-qualification/coefficient_controls.rs'),source);
for(const v of ['before','after'])assert.equal(hash(`.audit-numbers.rjcbha/coefficient-controls-${v}`),hashes[v]);
const all=JSON.parse(readFileSync(resolve(dir,'coefficient-paired-runs.json'),'utf8'));
assert.equal(all.length,198);assert.equal(new Set(all.map(r=>r.label)).size,198);
for(const r of all){
  const [round,variant,op,family]=r.label.split('-');Object.assign(r,{round:Number(round),variant,op,family,key:`${op}-${family}`});
  assert.equal(r.status,0);assert(!r.error&&!r.signal);assert.equal(r.sha256,hashes[variant]);assert.equal(r.source,source);
  assert.deepEqual(r.args,[op,family]);assert(r.round>=0&&r.round<=8);
  const f=r.stdout.trim().split('\t');assert.equal(f.length,8);assert.equal(f[0],op);assert.equal(f[1],family);
  const count=family==='high'?1:256;assert.equal(Number(f[2]),count);
  Object.assign(r,{count,cpu:Number(f[3]),wall:Number(f[4]),calls:Number(f[5]),bytes:Number(f[6]),checksum:f[7]});
  assert(r.cpu>0&&r.wall>0);
}
const rows=all.filter(r=>r.round>0);
const median=xs=>{xs=[...xs].sort((a,b)=>a-b);const m=xs.length>>1;return xs.length%2?xs[m]:(xs[m-1]+xs[m])/2;};
let seed=0x453acd17;
function randomIndex(n){seed=(Math.imul(seed,1664525)+1013904223)>>>0;return Math.floor(seed/4294967296*n);}
function ci(xs){const bs=Array.from({length:20000},()=>median(xs.map(()=>xs[randomIndex(xs.length)]))).sort((a,b)=>a-b);return [bs[500],bs[19499]];}
const paired=[];
for(const key of [...new Set(rows.map(r=>r.key))]){
  const before=rows.filter(r=>r.key===key&&r.variant==='before'),after=rows.filter(r=>r.key===key&&r.variant==='after');
  assert.equal(before.length,8);assert.equal(after.length,8);
  for(const variant of [before,after])for(const field of ['checksum','calls','bytes'])assert.equal(new Set(variant.map(r=>r[field])).size,1);
  assert.equal(before[0].checksum,after[0].checksum);
  const ratios=before.map(a=>a.cpu/after.find(b=>b.round===a.round).cpu);
  paired.push({family:key,count:before[0].count,cpuNsBefore:median(before.map(r=>r.cpu)),cpuNsAfter:median(after.map(r=>r.cpu)),
    pairedCpuSpeedupMedian:median(ratios),pairedBootstrap95:ci(ratios),callsBefore:before[0].calls,callsAfter:after[0].calls,
    bytesBefore:before[0].bytes,bytesAfter:after[0].bytes,identicalOutputChecksum:true});
}
console.log(JSON.stringify({observations:198,afterWarmup:176,checkedOutputs:all.reduce((a,r)=>a+r.count,0),paired,
  binaryFileBytes:Object.fromEntries(['before','after'].map(v=>[v,statSync(resolve(root,`.audit-numbers.rjcbha/coefficient-controls-${v}`)).size])),
  limitations:['One host, CPU6-pinned fresh processes; eight post-warmup pairs per family, alternating order.',
    'Concurrent system work was not excluded; intervals are descriptive, not a universal throughput claim.',
    'Allocator counters are inside timings; allocated bytes are cumulative requests, not peak memory.',
    'Inputs are preconstructed. Timed work includes clone, function construction and approximation.',
    'Every output is enclosed by directed MPFR outside timing: 4096 oracle bits for 32/128-bit ordinary requests, 184256 for 184000-bit high requests.',
    'Only mathematically correct baseline requests are timed; no ratio uses the frozen incorrect 192000-bit baseline.']},null,2));
