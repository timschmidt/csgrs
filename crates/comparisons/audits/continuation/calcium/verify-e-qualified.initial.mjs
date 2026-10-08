import './verify-e-plan.mjs';
import {readFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import {createHash} from 'node:crypto';
import assert from 'node:assert/strict';
import {sources,sha,json} from './e-plan-qualified-sources.mjs';
import {specifications} from './e-qualified-gates.mjs';
import {checkEQualified} from './check-e-qualified.mjs';
const draft=process.argv.includes('--draft'),m=json(draft?'e-qualified-experiment-draft.json':'e-qualified-experiment.json'),o=json('e-plan-qualified-origin.json');
const read=p=>readFileSync(p,'utf8'),workspace=resolve('../../../..');
assert.equal(m.draft,draft);for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
assert.deepEqual(m.sourceMap,sources(false));assert.equal(Object.keys(m.sourceMap.baseline).length,955);assert.equal(Object.keys(m.sourceMap.candidate).length,956);
assert.deepEqual(m.liveSources,draft?m.sourceMap.baseline:m.sourceMap.candidate);
if(process.argv.includes('--e-plan-live')||process.argv.includes('--derivative-live'))
 for(const[p,h]of Object.entries(m.liveSources))assert.equal(sha(resolve(workspace,p)),h,p);
assert.equal(m.production.retained,!draft);assert.equal(m.production.algorithmNetLines,8);assert.equal(m.production.testModuleLines,112);
assert.equal(m.production.testRegistrationLines,4);assert.deepEqual(m.production.paths,[...m.sourceMap.changed,...m.sourceMap.added]);
assert.equal(o.copiedFiles,955);assert.equal(o.copiedBytes,45446168);
assert.equal(read(o.candidate+'/'+m.sourceMap.added[0]).split('\n').length-1,112);
assert.deepEqual(m.checks,checkEQualified());assert.deepEqual(JSON.parse(read('results/e-qualified-output-check.stdout')),m.checks);
assert.deepEqual(m.gates,specifications(!draft));assert.equal(m.gates.length,draft?50:55);
const gates=new Map();
for(const g of m.gates) {
 const result=json('results/'+g.tag+'.json');for(const k of ['tag','cwd','command','args'])assert.deepEqual(result[k],g[k],g.tag+': '+k);
 assert.equal(result.code,0,g.tag);assert.equal(result.signal,null);assert(Date.parse(result.started)<=Date.parse(result.finished));
 assert(!/^warning(?:\[|:)|^error(?:\[|:)/m.test(read('results/'+g.tag+'.stderr')),g.tag);gates.set(g.tag,result);
}
let binaryBytes=0;assert.equal(Object.keys(m.binaries).length,14);
for(const[p,b]of Object.entries(m.binaries)){assert.equal(sha(p),b.sha256);assert.equal(statSync(p).size,b.bytes);binaryBytes+=b.bytes;}
assert.equal(binaryBytes,108188255);assert.equal(m.binaryBytes,binaryBytes);
const wasm=json('e-qualified-wasm-binaries.json'),apps=json('e-qualified-app-size-summary.json');
assert.deepEqual(wasm.sourceMap,m.sourceMap);assert.deepEqual(apps.sourceMap,m.sourceMap);
for(const[p,h]of Object.entries(wasm.inputs))assert.equal(sha(p),h,p);
for(const v of ['baseline','candidate']) {
 const s=read((v==='baseline'?o.baseline:o.candidate)+'/hyperreal/src/computable/approximation/constants.rs');
 assert.equal(read('e-qualified-wasm-'+v+'/kernel.rs'),'use num::{BigInt, BigUint, One};\ntype Precision = i32;\n'+s.slice(s.indexOf('fn e_terms_for_precision('))
  .replace('fn e_terms_for_precision(','pub fn e_terms_for_precision(').replace('fn e(p:','pub fn e(p:'));
 const b=wasm.modules[v];assert.deepEqual(b.imports,[]);assert.deepEqual(m.binaries[b.path],{sha256:b.sha256,bytes:b.bytes});
}
for(const b of apps.artifacts.flatMap(a=>a.files))assert.deepEqual(m.binaries[b.path],{sha256:b.sha256,bytes:b.bytes});
assert.equal(apps.artifacts.length,6);assert.equal(wasm.modules.candidate.bytes-wasm.modules.baseline.bytes,-507);
const before=a=>Date.parse(gates.get('e-qualified-'+a).finished),after=a=>Date.parse(gates.get('e-qualified-'+a).started);
assert(before('app-size-resume')<after('controls'));assert(before('candidate-consumer-clippy')<after('controls'));
assert(before('candidate-wasm-consumer')<after('controls'));assert(before('controls')<after('wasm-cpu'));
assert(before('wasm-check')<after('wasm-cpu'));assert(before('wasm-cpu')<after('output-check'));
const cov=[...json('coverage.json'),...json('coverage-extensions.json')],inv=json('inventory.json').sources.find(s=>s.repo==='flint');let readLines=0;
assert.deepEqual(m.readRecords,json('e-qualified-read-records.json'));assert.equal(m.readRecords.length,18);
for(const r of m.readRecords) {
 assert(cov.some(x=>JSON.stringify(x)===JSON.stringify(r)));const f=inv.files.find(f=>f.path===r.path);assert(f?.text);
 assert.equal(sha(resolve(workspace,'exact-real-references/flint',r.path)),f.sha256);assert.deepEqual(r.ranges,[[1,f.lines]]);readLines+=f.lines;
}
assert.equal(readLines,1176);assert.equal(m.newDonorLines,1176);assert.equal(m.completedArfTopLevelCFiles,41);
const top=inv.files.filter(f=>/^src\/arf\/[^/]+\.c$/.test(f.path));assert.equal(top.length,41);
for(const f of top) {
 const seen=new Set(cov.filter(r=>r.repo==='flint'&&r.path===f.path).flatMap(r=>r.ranges).flatMap(([a,b])=>Array.from({length:b-a+1},(_,i)=>a+i)));
 assert.equal(seen.size,f.lines,f.path);
}
assert.deepEqual(m.coverageAtBinding,[{repo:'calcium',reviewed:375,complete:374,partial:1,readLines:37085},
 {repo:'flint',reviewed:657,complete:640,partial:17,readLines:93640}]);
for(const[p,ranges]of Object.entries(m.hyperReadRanges)) {
 const source=read(o.baseline+'/'+p),lines=source.split('\n').length-Number(source.endsWith('\n'));assert.equal(sha(o.baseline+'/'+p),m.sourceMap.baseline[p]);
 for(const[a,b]of ranges)assert(a>=1&&b>=a&&b<=lines,p);
}
assert.equal(m.preCaptureFailure.invalidTag,'e-qualified-app-baseline-hyperreal-readme_quickstart-strip');
assert(read(m.preCaptureFailure.source).includes("+'-'+example+'-strip'"));assert(read('capture.mjs').includes('/^[a-z0-9-]+$/'));
assert(read(m.preCaptureFailure.note).includes('before opening output files or spawning `strip`'));
if(!draft) {
 for(const profile of ['debug','release']) {
  const tag='e-qualified-retained-all-'+profile,out=read('results/'+tag+'.stdout');
  const names=[...out.matchAll(/^test (.+) \.\.\. (ok|ignored[^\n]*)$/gm)].map(m=>m[1]+'|'+m[2]).sort();assert.equal(names.length,859);
  const expected=m.checks.rust.find(g=>g.tag==='e-qualified-candidate-all-'+profile);
  assert.equal(createHash('sha256').update(JSON.stringify(names)).digest('hex'),expected.membershipSha256);
  const totals=[...out.matchAll(/^test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;/gm)]
   .map(m=>m.slice(1).map(Number)).reduce((s,r)=>s.map((v,i)=>v+r[i]),[0,0,0,0,0]);assert.deepEqual(totals,[859,0,0,0,0]);
  const focused=read('results/e-qualified-retained-focused-'+profile+'.stdout');assert.match(focused,/test result: ok\. 4 passed; 0 failed; 0 ignored;/);
 }
}
const checks=structuredClone(m.checks);delete checks.nativeControls.details;delete checks.wasmCosts.details;
console.log(JSON.stringify({checkpoint:'Qualified e planner and ARF source closure',draft,files:Object.keys(m.files).length,gates:m.gates.length,
 liveFiles:Object.keys(m.liveSources).length,binaries:14,binaryBytes,readLines,completeArfTopLevelCFiles:41,checks,
 production:m.production,status:m.status,findings:m.findings,comparison:m.comparison,sourceFindings:m.sourceFindings,limits:m.limits,followup:m.followup}));
