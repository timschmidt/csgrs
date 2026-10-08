import {readFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import './inventory.mjs';
const dir=import.meta.dirname,root=resolve(dir,'../..');
const read=name=>readFileSync(resolve(dir,name),'utf8');
const hash=path=>createHash('sha256').update(readFileSync(resolve(root,path))).digest('hex');
assert.equal(hash('exact-real-references/numbers-qualification/Probe.hs'),'78e23e48ebfcf7aaa282d27c995b2e14622e8a058e2d2d1f19476cc9e116c304');
assert.equal(hash('exact-real-references/numbers-qualification/oracle.rs'),'c44198594ab7adebf23ffa61f2982aa4fceb76ad3d53c88d33b365320722bc9c');
assert.equal(hash('.audit-numbers.rjcbha/numbers-oracle-before'),'a7f436d72b107a7ade1f4adab337af1baf14c3974485377eab682ec01ed69d74');
const expectedBasic={add:[19208,0],sub:[19208,0],mul:[19208,0],min:[19208,0],max:[19208,0],
  division:[18816,0],unary:[392,0],sqrt:[2056,0],'public-decisions':[64,21],
  'properFraction-integer':[129,56],'public-bridge':[20,0],'polynomial-derivatives':[297,0],
  'atan-derivatives':[5,5],'approximate-zero-pruning':[1,1],'BigFloat-scaled-zero':[17,16]};
for(const opt of ['O0','O2']) {
  for(const mode of ['basic','elementary','fixed']) {
    const rows=JSON.parse(read(`${mode}-${opt}-runs.json`));
    assert.equal(rows.length,mode==='basic'?4:296);
    assert(rows.every(r=>r.status===0&&!r.error&&!r.signal));
    assert.equal(read(`${mode}-${opt}.log`),rows.map(r=>r.stdout).join(''));
  }
  const basic=Object.fromEntries([...read(`basic-${opt}.log`).matchAll(/^TOTAL (\S+) checks=(\d+) failures=(\d+)$/gm)]
    .map(m=>[m[1],[Number(m[2]),Number(m[3])]]));
  assert.deepEqual(basic,expectedBasic);
  assert(read(`elementary-${opt}-oracle.log`).includes('TOTAL elementary\tchecks=296\tfailures=12'));
  assert(read(`fixed-${opt}-oracle.log`).includes('TOTAL fixed\tchecks=296\tfailures=63'));
}
for(const mode of ['basic','elementary','fixed']) assert.equal(read(`${mode}-O0.log`),read(`${mode}-O2.log`));
for(const mode of ['elementary','fixed']) assert.equal(read(`${mode}-O0-oracle.log`),read(`${mode}-O2-oracle.log`));
assert(read('native-build.log').includes('No instance for'));
assert.equal((read('upstream-tests-final.log').match(/OK, passed 10000 tests/g)||[]).length,2);
assert(read('oracle-selfcheck.log').includes('PASS oracle exact identities=43'));
for(const opt of ['debug','release']) {
  assert(read(`hyper-${opt}.log`).includes('TOTAL hyper\tchecks=1184\tfailures=0'));
  assert(read(`hyper-exact-${opt}.log`).includes('controls=219'));
}
assert(read('hyper-memcheck.log').includes('ERROR SUMMARY: 0 errors'));
const baseline=JSON.parse(read('hyper-baseline-runs.json'));
assert.equal(baseline.length,18);
for(const r of baseline) {
  assert.equal(r.binarySha256,'a7f436d72b107a7ade1f4adab337af1baf14c3974485377eab682ec01ed69d74');
  const [,n,scale,,history]=r.args;
  const cap=n!=='-100'&&scale!=='1'&&history==='cold';
  if(cap) {assert.equal(r.error,'ETIMEDOUT');assert.equal(r.signal,'SIGKILL');}
  else {assert.equal(r.status,0);assert(!r.error);assert(r.stdout.startsWith('PASS opaque atan'));}
}
console.log(JSON.stringify({basicPerBuild:{checks:117837,failures:99,ordinaryApproximationChecks:117304,ordinaryFailures:0},
  elementaryPerBuild:{checks:296,failures:12},continuedFractionPerBuild:{checks:296,failures:63},
  optimizationRepeats:'all O0/O2 finite outputs and numerical verdicts identical',
  upstreamCompatibilityTests:'two properties, 10000 generated cases each',
  hyperPerBuild:{elementaryHistory:1184,exactSignTruncationHistory:219},
  frozenHyperColdWarmBaseline:{requests:18,passes:14,caps:4},
  status:'Baseline donor qualification preserved. See analyze-atan.mjs for the retained atan repair and measured tradeoffs; adjacent inverse-function repairs remain OPEN.'},null,2));
