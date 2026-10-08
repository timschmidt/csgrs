import {readFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
const dir=import.meta.dirname,root=resolve(dir,'../..');
const hash=p=>createHash('sha256').update(readFileSync(resolve(root,p))).digest('hex');
const hashes={before:'a6445ed8b7bc6af34869a243a2e5a1b3291b127f51481baf931bc9a9dbb51f50',after:'8cba21f1c7405d19e7c6c983f7c741b1efedac2deaf7f14bf392bc77df6b43d3'};
const source='62c0679bf765377d72300d99b1d7ed61fa6ab08640d8d7010f01411553094d8c';
assert.equal(hash('exact-real-references/numbers-qualification/neighbor_controls.rs'),source);
assert.equal(hash('.audit-numbers.rjcbha/neighbor-controls-before'),hashes.before);
assert.equal(hash('.audit-numbers.rjcbha/neighbor-controls-sample'),hashes.after);
function load(mode,n){
  const rows=JSON.parse(readFileSync(resolve(dir,`neighbor-repair-${mode}-runs.json`),'utf8'));
  assert.equal(rows.length,n);assert.equal(new Set(rows.map(r=>r.label)).size,n);
  for(const r of rows){
    const [round,variant,op,...family]=r.label.split('-');
    Object.assign(r,{round:Number(round),variant,op,family:family.join('-')});
    assert.equal(r.status,0);assert(!r.error&&!r.signal);assert.equal(r.sha256,hashes[variant]);assert.equal(r.source,source);
    assert.deepEqual(r.args,[r.op,r.family,'256']);
    const f=r.stdout.trim().split('\t');assert.equal(f.length,8);assert.equal(f[0],r.op);assert.equal(f[1],r.family);assert.equal(f[2],'256');
    Object.assign(r,{cpu:Number(f[3]),wall:Number(f[4]),calls:Number(f[5]),bytes:Number(f[6]),checksum:f[7]});
    assert(r.cpu>0&&r.wall>0);
  }
  return rows.filter(r=>r.round>0);
}
const rows=load('paired',252),repaired=load('repaired',18);
const median=xs=>{xs=[...xs].sort((a,b)=>a-b);let m=xs.length>>1;return xs.length%2?xs[m]:(xs[m-1]+xs[m])/2;};
let seed=0x234abcde;
function randomIndex(n){seed=(Math.imul(seed,1664525)+1013904223)>>>0;return Math.floor(seed/4294967296*n);}
function ci(xs){const bs=Array.from({length:20000},()=>median(xs.map(()=>xs[randomIndex(xs.length)]))).sort((a,b)=>a-b);return [bs[500],bs[19499]];}
const paired=[];
for(const op of ['asin','atanh'])for(const family of [...new Set(rows.map(r=>r.family))]){
  const before=rows.filter(r=>r.op===op&&r.family===family&&r.variant==='before');
  const after=rows.filter(r=>r.op===op&&r.family===family&&r.variant==='after');
  assert.equal(before.length,8);assert.equal(after.length,8);
  const ratios=before.map(a=>{const b=after.find(b=>b.round===a.round);assert.equal(a.checksum,b.checksum);return a.cpu/b.cpu;});
  paired.push({op,family,cpuNsBefore:median(before.map(r=>r.cpu)),cpuNsAfter:median(after.map(r=>r.cpu)),
    pairedCpuSpeedupMedian:median(ratios),pairedBootstrap95:ci(ratios),callsBefore:median(before.map(r=>r.calls)),callsAfter:median(after.map(r=>r.calls)),
    bytesBefore:median(before.map(r=>r.bytes)),bytesAfter:median(after.map(r=>r.bytes))});
}
const repairedSummary=['asin','atanh'].map(op=>{
  const rs=repaired.filter(r=>r.op===op);assert.equal(rs.length,8);assert(rs.every(r=>r.variant==='after'&&r.family==='repaired'));
  return {op,cpuNs:median(rs.map(r=>r.cpu)),calls:median(rs.map(r=>r.calls)),bytes:median(rs.map(r=>r.bytes)),baseline:'Numerically incorrect; no timing ratio claimed.'};
});
console.log(JSON.stringify({observations:270,afterWarmup:240,samplesPerProcess:256,paired,repaired:repairedSummary,
  limitations:['One host, CPU6-pinned fresh processes; eight post-warmup pairs per family.',
    'Concurrent system work was not excluded; medians and paired bootstrap intervals are reported, not a universal throughput claim.',
    'Timings include allocator counters; allocated bytes are cumulative requests, not peak memory.',
    'Inputs are preconstructed; timed work includes clone, function construction and approximation. Every output is independently enclosed by directed 4096-bit MPFR outside timing.']},null,2));
