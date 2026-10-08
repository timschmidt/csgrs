import './verify-fexpr-formatting.mjs';
import {readFileSync,statSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {effectiveCoverage,effectiveSummary} from './effective-coverage.mjs';
import {checkQqbarDecisions} from './check-qqbar-decisions.mjs';
const sha=p=>createHash('sha256').update(readFileSync(p)).digest('hex');
const json=p=>JSON.parse(readFileSync(p,'utf8'));
const m=json('qqbar-decisions-manifest.json'),workspace=resolve('../../../..');
assert.equal(m.checkpoint,50);assert.equal(Object.keys(m.files).length,33);
for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
assert.deepEqual(m.readRecords,json('qqbar-decisions-read-records.json'));assert.equal(m.readRecords.length,92);
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
assert.equal(lines,8646);assert.equal(m.newDonorLines,lines);
for(const[p,h]of Object.entries({...m.libraries,...m.configurationFiles}))assert.equal(sha(p),h,p);
for(const b of m.binaries){assert.equal(sha(b.path),b.sha256);assert.equal(statSync(b.path).size,b.bytes);}
assert.equal(m.binaries.length,1);assert.equal(m.binaries[0].bytes,18152);
const live=json(m.liveSourceManifest);assert.equal(Object.keys(live.liveSources).length,956);
for(const[p,h]of Object.entries(live.liveSources))assert.equal(sha(resolve(workspace,p)),h,p);
for(const[p,ranges]of Object.entries(m.hyperReadRanges)){
 assert(live.liveSources[p]);const s=readFileSync(resolve(workspace,p),'utf8'),n=s.split('\n').length-Number(s.endsWith('\n'));
 for(const[a,b]of ranges)assert(a>=1&&b>=a&&b<=n,p);
}
assert.equal(m.gates.length,5);
for(const tag of m.gates){
 const r=json('results/'+tag+'.json');assert.equal(r.code,tag.endsWith('-check')?1:0);assert.equal(r.signal,null);
 if(!tag.endsWith('-memcheck'))assert.equal(readFileSync('results/'+tag+'.stderr').length,0);
}
const native=readFileSync('results/qqbar-decisions-native.stdout'),mem=readFileSync('results/qqbar-decisions-memcheck.stdout');
assert(native.equals(mem));assert.equal(native.length,658089);
const memory=readFileSync('results/qqbar-decisions-memcheck.stderr','utf8');
assert(memory.includes('in use at exit: 0 bytes in 0 blocks'));
assert(memory.includes('2,729,471 allocs, 2,729,471 frees, 154,454,444 bytes allocated'));
assert(memory.includes('ERROR SUMMARY: 0 errors from 0 contexts (suppressed: 0 from 0)'));
assert(readFileSync('results/qqbar-decisions-linked.stdout','utf8').includes('libflint.so.25 => /home/tim/Documents/GitHub/workspace/exact-real-references/flint/libflint.so.25'));
const wire=value=>JSON.parse(JSON.stringify(value));
const checked=wire(checkQqbarDecisions('results/qqbar-decisions-native.stdout'));
assert.deepEqual(checked,wire(checkQqbarDecisions('results/qqbar-decisions-memcheck.stdout')));
assert.deepEqual(checked,json('results/qqbar-decisions-check.stdout'));assert.deepEqual(checked,m.checks);
assert.equal(checked.status,'mathematical-contract-fail');assert.equal(checked.totalChecks,35826);assert.equal(checked.failures.length,264);
assert.deepEqual(checked.failedByCheck,{copyRootOrder:84,rootOrder:84,'exact-dyadic-imag':48,'exact-dyadic-real':48});
const initial=json('results/qqbar-decisions-record-initial.json');
assert.equal(initial.code,1);assert.equal(initial.signal,null);
assert(readFileSync('results/qqbar-decisions-record-initial.stderr','utf8').includes('ERR_ASSERTION'));
assert.deepEqual(m.coverageBefore,json(m.previousManifest).coverageAtBinding);
assert.deepEqual(m.coverageAtBinding,[{repo:'calcium',reviewed:525,complete:525,partial:0,readLines:56632},
 {repo:'flint',reviewed:856,complete:836,partial:20,readLines:119798}]);
for(const now of effectiveSummary()){
 const then=m.coverageAtBinding.find(s=>s.repo===now.repo);assert(then);
 assert(now.complete>=then.complete&&now.readLines>=then.readLines);
}
console.log(JSON.stringify({checkpoint:50,status:'recorded-failures-and-source-evidence-integrity-pass',boundFiles:33,
 liveFiles:956,readRecords:92,newDonorLines:8646,coverage:m.coverageAtBinding,
 mathematicalStatus:checked.status,totalChecks:35826,failedAssertions:264,failedByCheck:checked.failedByCheck,
 numericalOutputBytes:native.length,focusedMemory:'zero errors and zero live blocks for stated corpus only',
 qualification:m.qualification,production:m.production,scope:m.scope}));
