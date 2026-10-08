import {readFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import './analyze-atan-bench.mjs';
const dir=import.meta.dirname,root=resolve(dir,'../..');
const read=p=>readFileSync(resolve(dir,p),'utf8');
const hash=p=>createHash('sha256').update(readFileSync(resolve(root,p))).digest('hex');
for(const [p,h] of [
  ['.audit-numbers.rjcbha/atan-retained-roots.rs','e63ecc1bdd838d6744c2c9b3324dea620c3c2fd1a31f179a6d02f54013efe11f'],
  ['.audit-numbers.rjcbha/atan-retained-tests.rs','67324eac110694b13d92722b1f778710923ffc76a47ebc8fb455d0f94e9be1b3'],
  ['.audit-numbers.rjcbha/atan-retained-inverse-trig.rs','65ceb888d05a40e7d8022aa8b27f4ac1ee4022329a867f65801217de8c39a110'],
  ['exact-real-references/numbers-qualification/atan_grid.rs','aefd04d590c687d098255e6bea3240b484b08e429446ff435fbcbdae7df99397'],
  ['exact-real-references/numbers-qualification/neighbor_probe.rs','ff5ed272647aa1485f900dc0edc46e6ba21abe5cab1a2a08cb944d200ae28d80'],
  ['exact-real-references/numbers-qualification/log_boundary.rs','64609c2ab917cc4fdabedf26544bbda6cbc59e7e386f5bf8c7d3d2d6436465c7'],
  ['.audit-numbers.rjcbha/log-boundary-before','9ba171a5ef78127394ed82300d43a503742800dcca30e918540f2f7f3d0a4a11'],
]) assert.equal(hash(p),h,p);
function testCount(file,passes,ignored=0) {
  const s=read(file),rows=[...s.matchAll(/^test result: ok\. (\d+) passed; 0 failed; (\d+) ignored;/gm)];
  assert(rows.length>0);assert(!s.includes('test result: FAILED'));
  assert.equal(rows.reduce((n,r)=>n+Number(r[1]),0),passes);
  assert.equal(rows.reduce((n,r)=>n+Number(r[2]),0),ignored);
}
assert(read('atan-regressions-before.log').includes('test result: FAILED. 0 passed; 3 failed;'));
testCount('atan-hyperreal-debug-final.log',728);
testCount('atan-hyperreal-release-all-features-final.log',833);
for(const [crate,count] of [['hyperlattice',19],['hyperlimit',242],['hypertri',3],['hypersolve',432]]) testCount(`atan-${crate}.log`,count);
testCount('atan-hypercurve.log',893,1);
assert(read('atan-hypercurve.log').includes('finished in 581.51s'));
assert.equal(read('atan-fmt-final.log'),'');assert(read('atan-clippy.log').includes('Finished'));
assert(read('atan-grid-memcheck.log').includes('ERROR SUMMARY: 0 errors'));
for(const mode of ['debug','release']) {
  assert(read(`atan-grid-${mode}.log`).includes('total=6456'));
  const rows=JSON.parse(read(`atan-validation-${mode}-runs.json`));assert.equal(rows.length,21);
  assert(rows.every(r=>r.status===0&&!r.error&&!r.signal));
  assert.equal(rows.filter(r=>r.stdout.startsWith('PASS opaque atan')).length,18);
  assert.equal(rows.filter(r=>r.stdout.includes('TOTAL hyper\tchecks=1184\tfailures=0')).length,1);
  assert.equal(rows.filter(r=>r.stdout.includes('controls=219')).length,1);
  assert.equal(rows.filter(r=>r.stdout.includes('PASS oracle exact identities=43')).length,1);
}
const size=read('atan-size.log').trim().split('\n').slice(1).map(s=>s.trim().split(/\s+/).slice(0,4).map(Number));
assert.deepEqual(size,[[1661651,219048,4456,1885155],[1661583,219048,4520,1885151]]);
assert.equal(readFileSync(resolve(root,'.audit-numbers.rjcbha/atan-before')).length,2209000);
assert.equal(readFileSync(resolve(root,'.audit-numbers.rjcbha/atan-after')).length,2208936);
const neighbor=[];
for(const [mode,h] of [['debug','673ea4981322485e9be2eac60e4d6be04157d02250adcac31e33644d2b5860a4'],['release','cc17927ecad505f1e56422c577983d3640da2833e5239ba42c9236f3164b80e3']]) {
  assert.equal(hash(`.audit-numbers.rjcbha/neighbor-before-${mode}`),h);
  const rows=JSON.parse(read(`neighbor-before-${mode}-runs.json`));assert.equal(rows.length,80);
  assert.equal(new Set(rows.map(r=>r.args.join(' '))).size,80);
  assert(rows.every(r=>r.sha256===h&&!r.error&&!r.signal));
  assert.equal(rows.filter(r=>r.status===0&&r.stdout.startsWith('PASS')).length,42);
  assert.equal(rows.filter(r=>r.status===0&&r.stdout.startsWith('FAIL')).length,mode==='debug'?37:38);
  const exceptions=rows.filter(r=>r.status!==0);
  assert.equal(exceptions.length,mode==='debug'?1:0);
  neighbor.push({mode,passes:42,numericalFailures:mode==='debug'?37:38,exceptions:exceptions.map(r=>({args:r.args,stderr:r.stderr}))});
}
assert.equal(read('log-boundary-before.log'),read('log-boundary-before-debug.log'));
assert(read('log-boundary-before.log').includes('TOTAL log-boundary checks=77 failures=8'));
console.log(JSON.stringify({retained:'atan signed/exact-domain dispatcher repair',debugPasses:728,releaseAllFeaturePasses:833,
  externalAtanGridPerBuild:6456,downstreamLibraryPasses:1589,ignored:1,neighborOpen:neighbor,
  uncheckedLog1pOpen:{checksPerBuild:77,failures:8,debugReleaseOutputsIdentical:true},
  scope:'numbers and broad audit remain OPEN; this validator credits only the frozen atan slice, not future source edits.'},null,2));
