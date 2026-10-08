import {readFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
const dir=import.meta.dirname,root=resolve(dir,'../..');
const hash=p=>createHash('sha256').update(readFileSync(resolve(root,p))).digest('hex');
const hashes={before:'98e5ed482cfc8f0bfba37baa311e03bb1cea9c49b9c0cc790c48bd293c443ff9',after:'bfd68b49b7b38f3831233f634b9b26c8ff2e61b2d2bc54936ef382075c050255'};
assert.equal(hash('exact-real-references/numbers-qualification/atan_controls.rs'),'8c21ed59ba739380dd3e7ae16cef20151f9049f1a20c4e1b92e2a9ea984462b2');
for(const v of ['before','after']) assert.equal(hash(`.audit-numbers.rjcbha/atan-${v}`),hashes[v]);
function load(mode,n) {
  const rows=JSON.parse(readFileSync(resolve(dir,`atan-${mode}-runs.json`),'utf8'));
  assert.equal(rows.length,n);
  for(const row of rows) {
    assert.equal(row.status,0);assert(!row.error&&!row.signal);assert.equal(row.sha256,hashes[row.variant]);
    assert.equal(row.args[0],'-c');assert.equal(row.args[1],'6');
    const f=row.stdout.trim().split('\t');assert.equal(f.length,7);assert.equal(f[0],row.family);assert.equal(f[1],'256');
    row.cpu=Number(f[2]);row.wall=Number(f[3]);row.calls=Number(f[4]);row.bytes=Number(f[5]);row.checksum=f[6];
    assert(row.cpu>0&&row.wall>0);
  }
  return rows.filter(r=>r.round>0);
}
const rows=load('paired',126),repaired=load('repaired',18);
const median=xs=>{xs=[...xs].sort((a,b)=>a-b);const m=xs.length>>1;return xs.length%2?xs[m]:(xs[m-1]+xs[m])/2;};
let seed=0x1234ace1;
function randomIndex(n) {seed=(Math.imul(seed,1664525)+1013904223)>>>0;return Math.floor((seed/4294967296)*n);}
function ci(xs) {
  const bs=Array.from({length:20000},()=>median(xs.map(()=>xs[randomIndex(xs.length)]))).sort((a,b)=>a-b);
  return [bs[500],bs[19499]];
}
const summary=[];
for(const family of [...new Set(rows.map(r=>r.family))]) {
  const before=rows.filter(r=>r.family===family&&r.variant==='before');
  const after=rows.filter(r=>r.family===family&&r.variant==='after');
  assert.equal(before.length,8);assert.equal(after.length,8);
  const ratios=before.map(a=>{const b=after.find(b=>b.round===a.round);assert.equal(a.checksum,b.checksum);return a.cpu/b.cpu;});
  summary.push({family,medianCpuNsBefore:median(before.map(r=>r.cpu)),medianCpuNsAfter:median(after.map(r=>r.cpu)),
    pairedCpuSpeedupMedian:median(ratios),pairedBootstrap95:ci(ratios),
    allocationCallsBefore:median(before.map(r=>r.calls)),allocationCallsAfter:median(after.map(r=>r.calls)),
    requestedBytesBefore:median(before.map(r=>r.bytes)),requestedBytesAfter:median(after.map(r=>r.bytes))});
}
const repairedSummary=[...new Set(repaired.map(r=>r.family))].map(family=>{
  const rs=repaired.filter(r=>r.family===family);assert.equal(rs.length,8);
  return {family,medianCpuNs:median(rs.map(r=>r.cpu)),allocationCalls:median(rs.map(r=>r.calls)),requestedBytes:median(rs.map(r=>r.bytes)),
    baseline:'nontermination cap, not a timing ratio'};
});
console.log(JSON.stringify({observations:144,afterWarmup:128,samplesPerProcess:256,paired:summary,repaired:repairedSummary,
  auditBinaryDelta:{fileBytes:-64,textBytes:-68,dataBytes:0,bssBytes:64,loadedTotalBytes:-4},
  limitations:['One host, fresh CPU6-pinned process pairs; 8 post-warmup pairs/family.',
    'Times include allocator counters; cumulative requested bytes are not peak residency.',
    'No throughput claim for the donor or ratio against a timed-out baseline.']},null,2));
