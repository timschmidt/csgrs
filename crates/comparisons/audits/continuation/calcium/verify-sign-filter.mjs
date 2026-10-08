import './verify-nfloat-complex.mjs';
import {readFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {signFilterSources,sha,json} from './sign-filter-sources.mjs';
import {checkSignFilter} from './check-sign-filter.mjs';
const m=json('sign-filter-experiment.json'),read=p=>readFileSync(p,'utf8'),here=resolve('.'),workspace=resolve('../../../..');
assert.equal(Object.keys(m.files).length,67);for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
assert.deepEqual(m.sourceMaps,signFilterSources());assert.deepEqual(m.liveSources,json('nfloat-complex-experiment.json').candidateSources.candidate);
assert.equal(Object.keys(m.liveSources).length,955);for(const map of Object.values(m.sourceMaps))assert.equal(Object.keys(map).length,956);
if(process.argv.includes('--derivative-live'))for(const[p,h]of Object.entries(m.liveSources))assert.equal(sha(resolve(workspace,p)),h,p);
assert.deepEqual(m.checks,checkSignFilter());assert.equal(Object.keys(m.binaries).length,6);
for(const b of Object.values(m.binaries)){assert.equal(sha(b.path),b.sha256);assert.equal(statSync(b.path).size,b.bytes);}
for(const[k,b]of Object.entries(json('sign-filter-binaries.json')))assert.deepEqual(m.binaries[k],b);
assert.equal(m.gates.length,15);assert.equal(new Set(m.gates.map(g=>g.tag)).size,15);
const gates=new Map(m.gates.map(({tag,code,signal})=>{
 const g=json('results/'+tag+'.json');assert.equal(g.tag,tag);assert.equal(code,0);assert.equal(signal,null);
 assert.equal(g.code,code);assert.equal(g.signal,signal);assert(Date.parse(g.finished)>=Date.parse(g.started));return[tag,g];
}));
const gate=t=>{const g=gates.get(t);assert(g,t);return g;};
const env=['CARGO_TARGET_DIR=/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse','CARGO_INCREMENTAL=0','CARGO_PROFILE_DEV_DEBUG=0','CARGO_BUILD_JOBS=2'];
for(const variant of ['baseline','candidate']) {
 const ctx=resolve('sign-filter-'+variant+'/hyperlimit');
 for(const profile of ['debug','release']) {
  const g=gate('sign-filter-'+variant+'-full-'+profile);assert.equal(g.cwd,ctx);assert.equal(g.command,'env');
  assert.deepEqual(g.args,[...env,'cargo','test','--offline','--locked',...(profile==='release'?['--release']:[]),'--all-features','--lib','--tests','--','--test-threads=1']);
 }
 const app=gate('sign-filter-'+variant+'-app-build');assert.equal(app.cwd,resolve('sign-filter-app-'+variant));assert.equal(app.command,'env');
 assert.deepEqual(app.args,[...env,'cargo','build','--offline','--release','--bins']);
 const trace=gate('sign-filter-'+variant+'-trace-build');assert.equal(trace.cwd,resolve('sign-filter-app-'+variant));assert.equal(trace.command,'env');
 assert.deepEqual(trace.args,[...env,'cargo','build','--offline','--locked','--release','--features','trace','--bin','audit-sign-filter-'+variant+'-trace']);
 const run=gate('sign-filter-'+variant+'-trace');assert.equal(run.cwd,here);assert.equal(run.command,m.binaries[variant+'-trace'].path);assert.deepEqual(run.args,[]);
 assert.equal(read('results/sign-filter-'+variant+'-trace.stderr'),'');assert(Date.parse(trace.finished)<Date.parse(run.started));
 assert(Date.parse(app.finished)<Date.parse(gate('sign-filter-cpu').started));
 assert(Date.parse(gate('sign-filter-'+variant+'-full-release').finished)<Date.parse(gate('sign-filter-cpu').started));
 assert(Date.parse(trace.finished)<Date.parse(gate('sign-filter-cpu').started));
}
const focused=gate('sign-filter-candidate-focused-debug');assert.equal(focused.cwd,resolve('sign-filter-candidate/hyperlimit'));assert.equal(focused.command,'env');
assert.deepEqual(focused.args,[...env,'cargo','test','--offline','--locked','--lib','--features','dispatch-trace','sign_summary','--','--test-threads=1']);
for(const mode of ['cpu','allocation']) {
 const g=gate('sign-filter-'+mode);assert.equal(g.cwd,here);assert.equal(g.command,'node');
 assert.deepEqual(g.args,['run-sign-filter-costs.mjs',mode,...(mode==='cpu'?['/tmp/calcium-sign-filter.TCGoGs']:[])]);
 assert.equal(read('results/sign-filter-'+mode+'.stderr'),'');
 const s=json('sign-filter-'+mode+'-summary.json');assert(Date.parse(s.started)>=Date.parse(g.started));assert(Date.parse(s.finished)<=Date.parse(g.finished));
 assert.deepEqual(read('results/sign-filter-'+mode+'.stdout').trimEnd().split('\n').map(JSON.parse),s.summaries);
}
assert(Date.parse(gate('sign-filter-cpu').finished)<Date.parse(gate('sign-filter-allocation').started));
const oracle=gate('sign-filter-ring-oracle');assert.equal(oracle.cwd,here);assert.equal(oracle.command,'node');assert.deepEqual(oracle.args,['sign-filter-ring-oracle.mjs']);
assert.equal(read('results/sign-filter-ring-oracle.stderr'),'');assert(Date.parse(oracle.finished)<Date.parse(gate('sign-filter-cpu').started));
const check=gate('sign-filter-output-check');assert.equal(check.cwd,here);assert.equal(check.command,'node');assert.deepEqual(check.args,['check-sign-filter.mjs','--sign-filter-summary']);
assert.deepEqual(JSON.parse(read('results/sign-filter-output-check.stdout')),m.checks);assert.equal(read('results/sign-filter-output-check.stderr'),'');
console.log(JSON.stringify({checkpoint:'two-sign constant-state filter trial',sourceFiles:956,liveFiles:955,
 tests:{perProfile:m.checks.tests.perProfile,suites:16,exhaustive:m.checks.tests.exhaustiveSequences,long:m.checks.tests.longSequences,
  privateTraces:m.checks.tests.privateTraceCases,publicTraces:m.checks.publicTraces},
 cpu:m.checks.cpu,allocation:m.checks.allocation,binaryDeltas:m.checks.benchBinaryDeltas,
 binaryBytes:Object.values(m.binaries).reduce((n,b)=>n+b.bytes,0),production:m.production,status:m.status,limits:m.limits,followup:m.followup}));
