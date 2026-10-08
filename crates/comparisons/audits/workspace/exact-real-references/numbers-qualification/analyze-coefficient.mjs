import {readFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
const dir=import.meta.dirname,root=resolve(dir,'../..');
const read=name=>readFileSync(resolve(dir,name),'utf8');
const hash=path=>createHash('sha256').update(readFileSync(resolve(root,path))).digest('hex');
const hashes={
  'coefficient-retained-inverse-trig.rs':'a4ee2646773a3cc1f7a934a7cf6dbf8cf6ef614e102123abb6d8595160f5bed1',
  'coefficient-retained-inverse-hyperbolic.rs':'dd28f60617c20bbf0a35ba9d2261726f349e52035f30b7d011a3a26858c25e43',
  'coefficient-retained-tests.rs':'bafee0865639f035a4b481ba6cc1570c004267b2118b809da1b2581e0655363a',
  'coefficient-series-after-debug':'6f3e856c756430c952247a2d051a0fad31e86bfe482c7d3a3dc4edb704fef26b',
  'coefficient-series-after-release':'4524504b35969406dbee70f89bae8a4e88f74b3f6573e4f6dfd3ae6d51fb2847',
  'coefficient-hypercurve-before':'85db2f356d37a2788fb3b577717380be07431531d4c93839fd2d1ac0248c411b',
  'coefficient-hypercurve-after':'68f09cd8cda4c7592a80afc3618ca8c646c05699fd18ebf248c97190e3f4c7d7',
};
for(const [path,expected] of Object.entries(hashes))assert.equal(hash(`.audit-numbers.rjcbha/${path}`),expected,path);
const probe='532e283afdbd070a6dd0d53b0def3c62293b97a4e2a32c94e0eb5e1cf5839069';
assert.equal(hash('exact-real-references/numbers-qualification/series_limit.rs'),probe);
for(const mode of ['debug','release']){
  const baseline=read(`coefficient-regression-before-${mode}.log`);
  assert(baseline.includes('0 passed; 4 failed;'));
  for(const op of ['asin','asinh'])for(const kind of ['rational','computable'])assert(baseline.includes(`${op}_${kind}_series_wide_coefficients ... FAILED`));
  if(mode==='debug')assert.equal((baseline.match(/attempt to multiply with overflow/g)||[]).length,4);
  else assert.equal((baseline.match(/generic=(true|false), sign=-1 at 192000 bits/g)||[]).length,4);
  const rows=JSON.parse(read(`coefficient-${mode}-runs.json`));assert.equal(rows.length,8);
  assert.equal(new Set(rows.map(r=>r.label)).size,8);
  for(const op of ['asin','asinh'])for(const bits of [184000,192000,256000,524288]){
    const row=rows.find(r=>r.label===`${mode}-${op}-${bits}`);assert(row);
    assert.equal(row.status,0);assert(!row.error&&!row.signal);
    assert.deepEqual(row.args,[op,String(bits)]);assert.equal(row.source,probe);
    assert.equal(row.sha256,hashes[`coefficient-series-after-${mode}`]);
    assert.match(row.stdout,new RegExp(`^PASS ${op}\\(1/16\\) bits=${bits} elapsed_ns=\\d+\\n$`));
  }
}
const counts={};
for(const [file,expected] of [['hyperreal-debug-final',746],['hyperreal-release-all-features-final',853],['hyperlattice',19],['hyperlimit',242],['hypertri',3],['hypersolve',432]]){
  const body=read(`coefficient-${file}.log`);assert(!body.includes('FAILED')&&!body.includes('error:'));
  const matches=[...body.matchAll(/test result: ok\. (\d+) passed; 0 failed;/g)];assert(matches.length);
  const total=matches.reduce((n,m)=>n+Number(m[1]),0);assert.equal(total,expected);counts[file]=total;
}
assert.equal(read('coefficient-fmt.log'),'');
for(const f of ['clippy','wasm32-check']){const b=read(`coefficient-${f}.log`);assert(b.includes('Finished'));assert(!b.includes('error:')&&!b.includes('warning:'));}
const memcheck=read('coefficient-memcheck.log');assert(memcheck.includes('4 passed; 0 failed;'));assert(memcheck.includes('ERROR SUMMARY: 0 errors from 0 contexts'));
const curve=read('coefficient-hypercurve.log');assert(curve.includes('898 passed; 2 failed; 1 ignored;'));
for(const v of ['before','after']){
  const body=read(`coefficient-hypercurve-${v}-controls.log`);
  assert(body.includes('0 passed; 2 failed;'));
  assert(body.includes('size_of::<BezierAlgebraicCuspSemicircleParameterCacheEntry2>()'));
  assert(body.includes('left: Uncertain(Predicate)\n right: Decided(Greater)'));
}
const mirror='.audit-coefficient-control.UPsrn7';
for(const line of read('coefficient-hypercurve-launch.sha256').trim().split('\n')){
  const [expected,path]=line.split(/\s+/);assert.equal(hash(`${mirror}/${path}`),expected);
}
assert.equal(hash(`${mirror}/hyperreal/src/computable/approximation/inverse_trig.rs`),'1e24d4ea4f51ef1340e3314a5d552dfbf3cc6bdcd3a34808f59eb12e488f03e2');
assert.equal(hash(`${mirror}/hyperreal/src/computable/approximation/inverse_hyperbolic.rs`),'b4ec61a2cd4f5da23893a5f3b1596a740fcaa169d39a875bd396c63e7fa875be');
console.log(JSON.stringify({status:'qualified-retained',independentHighPrecisionProbes:16,counts,memcheckErrors:0,
  hypercurve:{passed:898,failed:2,ignored:1,twoFailuresReproduceWithoutRepair:true,laterUserEditNotQualified:true},
  scope:'coefficient defect only; full Numbers/donor and ecosystem audit remain OPEN'},null,2));
