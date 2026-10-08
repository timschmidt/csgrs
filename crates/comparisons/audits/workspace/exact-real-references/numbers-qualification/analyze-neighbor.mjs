import {readFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import './analyze-neighbor-bench.mjs';
const dir=import.meta.dirname,root=resolve(dir,'../..');
const read=p=>readFileSync(resolve(dir,p),'utf8');
const hash=p=>createHash('sha256').update(readFileSync(resolve(root,p))).digest('hex');
for(const [p,h] of [
  ['.audit-numbers.rjcbha/neighbor-sample-roots.rs','ec631190625f987238bf7d1f76a2f9e88633f42671c0e1870e63ad28f65282ae'],
  ['.audit-numbers.rjcbha/neighbor-sample-inverse-trig.rs','1e24d4ea4f51ef1340e3314a5d552dfbf3cc6bdcd3a34808f59eb12e488f03e2'],
  ['.audit-numbers.rjcbha/neighbor-sample-inverse-hyperbolic.rs','b4ec61a2cd4f5da23893a5f3b1596a740fcaa169d39a875bd396c63e7fa875be'],
  ['.audit-numbers.rjcbha/neighbor-sample-tests.rs','4d34d411e181b77215d64045ce743ce3daa0e76c77a8966a344ec3edceec5a51'],
  ['exact-real-references/numbers-qualification/neighbor_probe.rs','ff5ed272647aa1485f900dc0edc46e6ba21abe5cab1a2a08cb944d200ae28d80'],
]) assert.equal(hash(p),h,p);
function testCount(file,passes,ignored=0){
  const s=read(file),rows=[...s.matchAll(/^test result: ok\. (\d+) passed; 0 failed; (\d+) ignored;/gm)];
  assert(rows.length>0);assert(!s.includes('test result: FAILED'));
  assert.equal(rows.reduce((n,r)=>n+Number(r[1]),0),passes,file);
  assert.equal(rows.reduce((n,r)=>n+Number(r[2]),0),ignored,file);
}
assert(read('neighbor-regressions-before.log').includes('test result: FAILED. 0 passed; 4 failed;'));
testCount('neighbor-sample-hyperreal-debug.log',735);
testCount('neighbor-sample-hyperreal-release-all-features.log',841);
for(const [crate,n] of [['hyperlattice',19],['hyperlimit',242],['hypertri',3],['hypersolve',432]])testCount(`neighbor-${crate}.log`,n);
assert.equal(read('neighbor-sample-fmt.log'),'');assert(read('neighbor-sample-clippy.log').includes('Finished'));
const mem=read('neighbor-memcheck.log');assert(mem.includes('ERROR SUMMARY: 0 errors'));
testCount('neighbor-memcheck.log',3);
for(const [mode,h] of [['debug','eb1cb882a18c80583fd431f2f753b61c94defa3800c5bee4b3e73f43b7af1529'],['release','723d3382ff16449f0d4c7ebca67a68edf707c6f8895037c8828be1246aa7c9e3']]){
  assert.equal(hash(`.audit-numbers.rjcbha/neighbor-sample-${mode}`),h);
  const after=JSON.parse(read(`neighbor-repair-${mode}-runs.json`));
  const before=JSON.parse(read(`neighbor-before-${mode}-runs.json`));
  assert.equal(after.length,80);assert.equal(before.length,80);
  assert.equal(new Set(after.map(r=>r.args.join(' '))).size,80);
  assert.deepEqual(after.map(r=>r.args),before.map(r=>r.args));
  assert(after.every(r=>r.status===0&&!r.error&&!r.signal&&r.stdout.startsWith('PASS')&&r.sha256===h));
  assert.equal(before.filter(r=>r.status===0&&r.stdout.startsWith('PASS')).length,42);
}
const size=read('neighbor-size.log').trim().split('\n').slice(1).map(s=>s.trim().split(/\s+/).slice(0,4).map(Number));
assert.deepEqual(size,[[1670415,218920,4000,1893335],[1671231,218968,3120,1893319]]);
assert.equal(readFileSync(resolve(root,'.audit-numbers.rjcbha/neighbor-controls-before')).length,2216160);
assert.equal(readFileSync(resolve(root,'.audit-numbers.rjcbha/neighbor-controls-sample')).length,2217096);
console.log(JSON.stringify({qualified:'sample-reusing asin/atanh series-domain guards',debugPasses:735,releaseAllFeaturePasses:841,
  directedMpfrBoundaryChecksPerBuild:46674,frozenPublicRequestsPerBuild:80,downstreamLibraryPasses:696,memcheckErrors:0,
  remaining:'ln_1p repair and final-source Hypercurve rerun remain separately tracked in the root ledger; numbers and ecosystem audits are OPEN.',
  scope:'This validator credits frozen scalar-source snapshots only, not later worktree changes.'},null,2));
