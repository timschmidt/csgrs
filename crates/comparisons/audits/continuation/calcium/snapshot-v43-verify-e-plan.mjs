import './snapshot-v43-verify-mag-series.mjs';
import {readFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json} from './e-plan-sources.mjs';
import {checkEPlan} from './check-e-plan.mjs';
import {checkEPlanRegressions} from './check-e-plan-regressions.mjs';
const m=json('e-plan-experiment.json'),previous=json('mag-series-experiment.json'),frozen=json('e-plan-binaries.json'),o=json('e-plan-origin.json');
const read=p=>readFileSync(p,'utf8'),here=resolve('.'),workspace=resolve('../../../..');
for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
assert.deepEqual(m.sourceMap,frozen.sourceMap);assert.deepEqual(m.sourceMap.live,previous.liveSources);
assert.deepEqual(m.sourceMap.live,o.liveSources);assert.equal(Object.keys(m.sourceMap.live).length,955);
assert.equal(Object.keys(m.sourceMap.candidate).length,180);assert.equal(o.copiedFiles,180);assert.equal(o.copiedBytes,5312930);
if(process.argv.includes('--derivative-live'))for(const[p,h]of Object.entries(m.sourceMap.live))assert.equal(sha(resolve(workspace,p)),h,p);
const changed=[];
for(const[p,h]of Object.entries(m.sourceMap.candidate)) {
 assert.equal(sha(o.baseline+'/'+p),m.sourceMap.live[p]);assert.equal(sha(o.candidate+'/'+p),h);if(h!==m.sourceMap.live[p])changed.push(p);
}
assert.deepEqual(changed,['hyperreal/src/computable/approximation/constants.rs']);assert.deepEqual(m.sourceMap.changed,changed);
for(const[p,h]of Object.entries(m.sourceMap.files))assert.equal(m.files[p],h);
const a=read(o.baseline+'/'+changed[0]),b=read(o.candidate+'/'+changed[0]);
const start=s=>s.indexOf('fn e_terms_for_precision('),end=s=>s.indexOf('// Returns (P, Q)');
assert.equal(a.slice(0,start(a)),b.slice(0,start(b)));assert.equal(a.slice(end(a)),b.slice(end(b)));
assert.equal(b.split('\n').length-a.split('\n').length,8);
for(const v of ['baseline','candidate']) {
 const source=v==='baseline'?a:b;
 assert.equal(read('e-plan-app-'+v+'/kernel.rs'),'use num::{BigInt, BigUint, One};\ntype Precision = i32;\n'+source.slice(start(source))
  .replace('fn e_terms_for_precision(','pub fn e_terms_for_precision(').replace('fn e(p:','pub fn e(p:'));
}
assert.deepEqual(m.binaries,frozen.binaries);let total=0;
for(const binary of Object.values(m.binaries)) {assert.equal(sha(binary.path),binary.sha256);assert.equal(statSync(binary.path).size,binary.bytes);total+=binary.bytes;}
assert.equal(total,11204552);assert.equal(m.binaryBytes,total);assert.equal(m.newDonorLines,0);assert.deepEqual(m.readRecords,[]);
assert.deepEqual(m.coverageAtBinding,previous.coverageAtBinding);
for(const[p,ranges]of Object.entries(m.hyperReadRanges)) {
 const source=read(o.baseline+'/'+p),lines=source.split('\n').length-Number(source.endsWith('\n'));
 assert.equal(sha(o.baseline+'/'+p),m.sourceMap.live[p]);for(const[lo,hi]of ranges)assert(lo>=1&&hi>=lo&&hi<=lines,p);
}
assert.deepEqual(m.checks,checkEPlan());assert.deepEqual(m.regressions,checkEPlanRegressions());
assert.deepEqual(JSON.parse(read('results/e-plan-output-check.stdout')),m.checks);
assert.equal(m.gates.length,21);const gates=new Map();
for(const item of m.gates) {
 const g=json('results/'+item.tag+'.json');assert.equal(g.tag,item.tag);assert.equal(g.cwd,here);
 assert.equal(g.code,item.tag==='e-plan-build-baseline'?101:0);assert.equal(item.code,g.code);assert.equal(g.signal,null);assert.equal(item.signal,null);
 assert(Date.parse(g.started)<=Date.parse(g.finished));gates.set(g.tag.slice('e-plan-'.length),g);
}assert.equal(gates.size,21);
const env=['CARGO_TARGET_DIR=/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse','CARGO_INCREMENTAL=0','CARGO_PROFILE_DEV_DEBUG=0','CARGO_BUILD_JOBS=2'];
for(const t of ['build-baseline','build-baseline-fixed','build-candidate']) {
 const g=gates.get(t),v=t.includes('candidate')?'candidate':'baseline';assert.equal(g.command,'env');
 assert.deepEqual(g.args,[...env,'cargo','build','--release','--offline','--manifest-path','e-plan-app-'+v+'/Cargo.toml','--bins']);
 assert.equal(read('results/e-plan-'+t+'.stdout'),'');
 if(t!=='build-baseline')assert(!/^warning(?:\[|:)|^error(?:\[|:)/m.test(read('results/e-plan-'+t+'.stderr')));
}
const failed=read('results/e-plan-check.initial-build.rs');
assert.equal(failed.replace('use num::{BigInt, One};','use num::BigInt;').replace('Float::with_val(prec, &a - 1)','Float::with_val(prec, Integer::from(&a - 1))')
 .replace('Float::with_val(prec, &a + 1)','Float::with_val(prec, Integer::from(&a + 1))'),read('e-plan-check.rs'));
assert.equal((read('results/e-plan-build-baseline.stderr').match(/error\[E0277\]/g)||[]).length,2);
for(const v of ['baseline','candidate']) {
 for(const kind of ['plans','numeric','state']) {
  const g=gates.get('native-'+v+'-'+kind);assert.equal(g.command,m.binaries[v+'-check'].path);assert.deepEqual(g.args,[kind]);
  assert.equal(read('results/e-plan-native-'+v+'-'+kind+'.stderr'),'');
  assert(Date.parse(gates.get('freeze').finished)<Date.parse(g.started));
 }
 for(const kind of ['plans','numeric']) {
  const g=gates.get('memcheck-'+v+'-'+kind);assert.equal(g.command,'valgrind');
  assert.deepEqual(g.args,['--vgdb=no','--tool=memcheck','--error-exitcode=97','--leak-check=full',
   '--show-leak-kinds=all','--errors-for-leak-kinds=definite,indirect,possible',m.binaries[v+'-check'].path,kind]);
  assert(Date.parse(gates.get('native-'+v+'-'+kind).finished)<Date.parse(g.started));
 }
 for(const profile of ['debug','release']) {
  const g=gates.get('tests-'+v+'-'+profile);assert.equal(g.command,'env');
  assert.deepEqual(g.args,[...env,'cargo','test','--locked','--offline','--manifest-path',(v==='baseline'?o.baseline:o.candidate)+'/hyperreal/Cargo.toml',
   '--all-features','--lib','--tests',...(profile==='release'?['--release']:[]),'--','--test-threads=4']);
  assert(Date.parse(gates.get('allocation').finished)<Date.parse(g.started));
 }
}
for(const[t,args]of [['freeze',['run-e-plan-costs.mjs','freeze']],['cpu',['run-e-plan-costs.mjs','cpu']],
 ['allocation',['run-e-plan-costs.mjs','allocation']],['output-check',['check-e-plan.mjs','--e-plan-summary']]]) {
 const g=gates.get(t);assert.equal(g.command,'node');assert.deepEqual(g.args,args);assert.equal(read('results/e-plan-'+t+'.stderr'),'');
}
assert(Date.parse(gates.get('build-candidate').finished)<Date.parse(gates.get('freeze').started));
assert(Date.parse(gates.get('build-baseline-fixed').finished)<Date.parse(gates.get('freeze').started));
assert(Date.parse(gates.get('memcheck-candidate-numeric').finished)<Date.parse(gates.get('cpu').started));
assert(Date.parse(gates.get('cpu').finished)<Date.parse(gates.get('allocation').started));
const checks=structuredClone(m.checks);delete checks.costs.cpu.details;delete checks.costs.allocation.details;
console.log(JSON.stringify({checkpoint:'Lower-factorial e planning',files:Object.keys(m.files).length,gates:m.gates.length,
 liveFiles:955,candidateFiles:180,changedFiles:1,readLines:0,binaryBytes:total,checks,regressions:m.regressions,
 status:m.status,production:m.production,findings:m.findings,comparison:m.comparison,limits:m.limits,followup:m.followup}));
