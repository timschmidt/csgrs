import {readFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import './analyze-log-bench.mjs';
const dir=import.meta.dirname,root=resolve(dir,'../..');
const read=p=>readFileSync(resolve(dir,p),'utf8');
const hash=p=>createHash('sha256').update(readFileSync(resolve(root,p))).digest('hex');
for(const [p,h] of [
  ['.audit-numbers.rjcbha/log-retained-kernel.rs','eb05426db39726dfc3b5e5e121850656fe7e6fd03c6bade0fe0533779319952b'],
  ['.audit-numbers.rjcbha/log-retained-node.rs','9874420245a129a67ab7e38fd2a87befa0bf418dcafa9b3f177600b30a2eb971'],
  ['.audit-numbers.rjcbha/log-retained-tests.rs','222605dde1cf69b3d7eeb5d004cfe8c7a63133b9c8f470734d77256ec313bc6c'],
  ['exact-real-references/numbers-qualification/log_boundary.rs','64609c2ab917cc4fdabedf26544bbda6cbc59e7e386f5bf8c7d3d2d6436465c7'],
  ['.audit-numbers.rjcbha/log-boundary-before','9ba171a5ef78127394ed82300d43a503742800dcca30e918540f2f7f3d0a4a11'],
])assert.equal(hash(p),h,p);
function testCount(file,passes,ignored=0){
  const s=read(file),rows=[...s.matchAll(/^test result: ok\. (\d+) passed; 0 failed; (\d+) ignored;/gm)];
  assert(rows.length>0);assert(!s.includes('test result: FAILED'));
  assert.equal(rows.reduce((n,r)=>n+Number(r[1]),0),passes,file);
  assert.equal(rows.reduce((n,r)=>n+Number(r[2]),0),ignored,file);
}
assert(read('log1p-regression-before.log').includes('test result: FAILED. 0 passed; 1 failed;'));
assert(read('log1p-regression-before.log').includes('ln1p(-255/256) at 0'));
testCount('log-hyperreal-debug-final.log',741);
testCount('log-hyperreal-release-all-features-final.log',848);
for(const [crate,n] of [['hyperlattice',19],['hyperlimit',242],['hypertri',3],['hypersolve',432]])testCount(`log-${crate}.log`,n);
testCount('log-hypercurve-final.log',897,1);assert(read('log-hypercurve-final.log').includes('finished in 604.24s'));
assert(read('neighbor-hypercurve.log').includes('signal: 15, SIGTERM'));
assert.equal(read('log-fmt-final.log'),'');
for(const file of ['log-clippy.log','log-clippy-all-targets.log']){assert(read(file).includes('Finished'));assert(!/^error[:\[]/m.test(read(file)));}
assert(read('log-memcheck.log').includes('ERROR SUMMARY: 0 errors'));testCount('log-memcheck.log',6);
assert.equal(read('log-boundary-before.log'),read('log-boundary-before-debug.log'));
assert(read('log-boundary-before.log').includes('TOTAL log-boundary checks=77 failures=8'));
for(const [mode,h,neighbor] of [
  ['debug','941430a3b763b73801eda3c69cfb650a296b0d9bb39c0e64791a1b619952ee17','23601468084237cb1bf11a2294a7cba82563dbe3637ea50e416fd6f89016d085'],
  ['release','aa60718550918a8af9e018d6f415c5534b5f1a8c6b904976598af552da8e7ed0','730062a6032d03fa5490a4c78571e37653d0bed7f975e6e45fd8a4a183fa6179'],
]){
  assert.equal(hash(`.audit-numbers.rjcbha/log-boundary-after-${mode}`),h);
  const rows=JSON.parse(read(`log-repair-${mode}-runs.json`));assert.equal(rows.length,1);
  const r=rows[0];assert.equal(r.status,0);assert(!r.error&&!r.signal);assert.equal(r.sha256,h);
  assert.equal((r.stdout.match(/^PASS /gm)||[]).length,77);assert(r.stdout.includes('TOTAL log-boundary checks=77 failures=0'));
  assert.equal(hash(`.audit-numbers.rjcbha/log-neighbor-after-${mode}`),neighbor);
  const cross=JSON.parse(read(`log-repair-neighbor-${mode}-runs.json`));assert.equal(cross.length,80);
  assert(cross.every(r=>r.status===0&&!r.error&&!r.signal&&r.sha256===neighbor&&r.stdout.startsWith('PASS')));
  assert.deepEqual(cross.map(r=>r.args),JSON.parse(read(`neighbor-before-${mode}-runs.json`)).map(r=>r.args));
}
const size=read('log-size.log').trim().split('\n').slice(1).map(s=>s.trim().split(/\s+/).slice(0,4).map(Number));
assert.deepEqual(size,[[1571219,218792,4880,1794891],[1571931,218816,4184,1794931]]);
assert.equal(readFileSync(resolve(root,'.audit-numbers.rjcbha/log-controls-before')).length,2106696);
assert.equal(readFileSync(resolve(root,'.audit-numbers.rjcbha/log-controls-after')).length,2107584);
console.log(JSON.stringify({retained:'sample-checked ln1p with contracting logarithm fallback',debugPasses:741,releaseAllFeaturePasses:848,
  directedMpfrChecksPerBuild:4186,originalLogBoundaryChecksPerBuild:77,currentBuildInverseCrossChecksPerBuild:80,
  downstreamLibraryPasses:1593,ignored:1,memcheckErrors:0,
  scope:'Frozen scalar repair qualified. Broader reference audit and separately noted extreme-series coefficient limits remain OPEN.'},null,2));
