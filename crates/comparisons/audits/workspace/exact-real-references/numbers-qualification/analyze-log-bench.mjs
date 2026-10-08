import {readFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
const dir=import.meta.dirname,root=resolve(dir,'../..');
const hash=p=>createHash('sha256').update(readFileSync(resolve(root,p))).digest('hex');
const hashes={before:'de78b99d1b52b2beed3c310f1591ccc8c6a57a1d36671af8e643866a8fba34f2',after:'f07512865352dfe999b8a257fd8dbac35d95e02e0e8f8b0d2deab93a26ad91e4'};
const source='e28729898ed85f06f45c72440ce1a8bf3b4908faa3aa04a00a81ef7d1b480ca9';
assert.equal(hash('exact-real-references/numbers-qualification/log_controls.rs'),source);
for(const v of ['before','after'])assert.equal(hash(`.audit-numbers.rjcbha/log-controls-${v}`),hashes[v]);
const all=JSON.parse(readFileSync(resolve(dir,'log-repair-paired-runs.json'),'utf8'));
assert.equal(all.length,198);assert.equal(new Set(all.map(r=>r.label)).size,198);
for(const r of all){
  const [round,variant,...family]=r.label.split('-');Object.assign(r,{round:Number(round),variant,family:family.join('-')});
  assert.equal(r.status,0);assert(!r.error&&!r.signal);assert.equal(r.sha256,hashes[variant]);assert.equal(r.source,source);
  assert.deepEqual(r.args,[r.family,'256']);
  const f=r.stdout.trim().split('\t');assert.equal(f.length,7);assert.equal(f[0],r.family);assert.equal(f[1],'256');
  Object.assign(r,{cpu:Number(f[2]),wall:Number(f[3]),calls:Number(f[4]),bytes:Number(f[5]),checksum:f[6]});
  assert(r.cpu>0&&r.wall>0);
}
const rows=all.filter(r=>r.round>0);
const median=xs=>{xs=[...xs].sort((a,b)=>a-b);const m=xs.length>>1;return xs.length%2?xs[m]:(xs[m-1]+xs[m])/2;};
let seed=0x453acd17;
function randomIndex(n){seed=(Math.imul(seed,1664525)+1013904223)>>>0;return Math.floor(seed/4294967296*n);}
function ci(xs){const bs=Array.from({length:20000},()=>median(xs.map(()=>xs[randomIndex(xs.length)]))).sort((a,b)=>a-b);return [bs[500],bs[19499]];}
const paired=[];
for(const family of [...new Set(rows.map(r=>r.family))]){
  const before=rows.filter(r=>r.family===family&&r.variant==='before'),after=rows.filter(r=>r.family===family&&r.variant==='after');
  assert.equal(before.length,8);assert.equal(after.length,8);
  assert.equal(new Set(before.map(r=>r.checksum)).size,1);assert.equal(new Set(after.map(r=>r.checksum)).size,1);
  const ratios=before.map(a=>a.cpu/after.find(b=>b.round===a.round).cpu);
  paired.push({family,cpuNsBefore:median(before.map(r=>r.cpu)),cpuNsAfter:median(after.map(r=>r.cpu)),
    pairedCpuSpeedupMedian:median(ratios),pairedBootstrap95:ci(ratios),callsBefore:median(before.map(r=>r.calls)),callsAfter:median(after.map(r=>r.calls)),
    bytesBefore:median(before.map(r=>r.bytes)),bytesAfter:median(after.map(r=>r.bytes)),identicalOutputChecksum:before[0].checksum===after[0].checksum});
}
console.log(JSON.stringify({observations:198,afterWarmup:176,samplesPerProcess:256,paired,
  limitations:['One host, CPU6-pinned fresh processes; eight post-warmup pairs per family, alternating order.',
    'Concurrent system work was not excluded; intervals are descriptive, not a universal throughput claim.',
    'Allocator counters are inside timings; allocated bytes are cumulative requests, not peak memory.',
    'Inputs are preconstructed. Timed work includes clone, function construction, approximation; every output is enclosed by directed 4096-bit MPFR outside timing.',
    'Different valid integer approximations are allowed by the public error contract; cross-variant checksums need not match.']},null,2));
