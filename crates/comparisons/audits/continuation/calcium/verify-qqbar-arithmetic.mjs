import './verify-qqbar-decisions.mjs';
import {readFileSync,statSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {effectiveCoverage,effectiveSummary} from './effective-coverage.mjs';
import {checkQqbarArithmetic} from './check-qqbar-arithmetic.mjs';
const sha=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const json=p=>JSON.parse(readFileSync(p,'utf8'));
const m=json('qqbar-arithmetic-manifest.json'),workspace=resolve('../../../..');
assert.equal(m.checkpoint,51);assert.equal(Object.keys(m.files).length,28);
for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
assert.deepEqual(m.readRecords,json('qqbar-arithmetic-read-records.json'));assert.equal(m.readRecords.length,54);
const inv=json('inventory.json'),ext=json('coverage-extensions.json'),coverage=effectiveCoverage();
let lines=0;
for(const r of m.readRecords){
 assert(ext.some(c=>JSON.stringify(c)===JSON.stringify(r)));
 const f=inv.sources.find(s=>s.repo===r.repo).files.find(f=>f.path===r.path);assert(f?.text);
 assert.deepEqual(r.ranges,[[1,f.lines]]);lines+=f.lines;
 assert.equal(m.donorSources[r.repo+':'+r.path],f.sha256);
 assert.equal(sha(resolve(workspace,'exact-real-references',r.repo,r.path)),f.sha256);
 assert.deepEqual(coverage.find(c=>c.repo===r.repo&&c.path===r.path).ranges,[[1,f.lines]]);
}
assert.equal(lines,7223);assert.equal(m.newDonorLines,lines);
for(const[p,h]of Object.entries({...m.libraries,...m.configurationFiles}))assert.equal(sha(p),h,p);
assert.equal(m.binaries.length,1);
for(const b of m.binaries){assert.equal(sha(b.path),b.sha256);assert.equal(statSync(b.path).size,b.bytes);}
const live=json(m.liveSourceManifest);assert.equal(Object.keys(live.liveSources).length,956);
for(const[p,h]of Object.entries(live.liveSources))assert.equal(sha(resolve(workspace,p)),h,p);
for(const[p,ranges]of Object.entries(m.hyperReadRanges)){
 assert(live.liveSources[p]);const s=readFileSync(resolve(workspace,p),'utf8'),n=s.split('\n').length-Number(s.endsWith('\n'));
 for(const[a,b]of ranges)assert(a>=1&&b>=a&&b<=n,p);
}
assert.equal(m.gates.length,5);
for(const tag of m.gates){const r=json('results/'+tag+'.json');assert.equal(r.code,0);assert.equal(r.signal,null);
 if(!tag.endsWith('-memcheck'))assert.equal(readFileSync('results/'+tag+'.stderr').length,0);}
const native=readFileSync('results/qqbar-arithmetic-native.stdout'),mem=readFileSync('results/qqbar-arithmetic-memcheck.stdout');
assert(native.equals(mem));assert.equal(native.length,2032198);
const memory=readFileSync('results/qqbar-arithmetic-memcheck.stderr','utf8');
assert(memory.includes('in use at exit: 0 bytes in 0 blocks'));
assert(memory.includes('3,180,130 allocs, 3,180,130 frees, 150,670,769 bytes allocated'));
assert(memory.includes('ERROR SUMMARY: 0 errors from 0 contexts (suppressed: 0 from 0)'));
assert(readFileSync('results/qqbar-arithmetic-linked.stdout','utf8').includes('libflint.so.25 => /home/tim/Documents/GitHub/workspace/exact-real-references/flint/libflint.so.25'));
const checked=checkQqbarArithmetic('results/qqbar-arithmetic-native.stdout');
assert.deepEqual(checked,checkQqbarArithmetic('results/qqbar-arithmetic-memcheck.stdout'));
assert.deepEqual(checked,json('results/qqbar-arithmetic-check.stdout'));assert.deepEqual(checked,m.checks);
assert.equal(checked.status,'pass');assert.equal(checked.totalChecks,40352);assert.deepEqual(checked.failures,[]);
assert.deepEqual(checked.counts,{value:7344,composed:1008,relation:2560,preserved:32,terminal:1});
assert.deepEqual(m.coverageBefore,json(m.previousManifest).coverageAtBinding);
assert.deepEqual(m.coverageAtBinding,[{repo:'calcium',reviewed:552,complete:552,partial:0,readLines:60286},
 {repo:'flint',reviewed:883,complete:863,partial:20,readLines:123367}]);
for(const now of effectiveSummary()){
 const then=m.coverageAtBinding.find(s=>s.repo===now.repo);assert(then);
 assert(now.complete>=then.complete&&now.readLines>=then.readLines);
}
console.log(JSON.stringify({checkpoint:51,status:'independent-arithmetic-and-source-evidence-integrity-pass',boundFiles:28,
 liveFiles:956,readRecords:54,newDonorLines:7223,coverage:m.coverageAtBinding,
 mathematicalStatus:checked.status,totalChecks:40352,records:10945,numericalOutputBytes:native.length,
 focusedMemory:'Zero errors and zero live blocks for the stated bounded corpus only.',binaries:m.binaries,
 qualification:m.qualification,production:m.production,candidate:m.candidate,scope:m.scope}));
