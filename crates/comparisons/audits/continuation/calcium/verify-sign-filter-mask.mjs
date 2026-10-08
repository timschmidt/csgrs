import './verify-sign-filter.mjs';
import {readFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {signFilterSources,sha,json} from './sign-filter-mask-sources.mjs';
import {checkSignFilterMask} from './check-sign-filter-mask.mjs';
const m=json('sign-filter-mask-experiment.json'),previous=json('sign-filter-experiment.json');
const read=p=>readFileSync(p,'utf8'),here=resolve('.'),workspace=resolve('../../../..');
assert.equal(Object.keys(m.files).length,41);for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
assert.deepEqual(m.sourceMaps,signFilterSources());assert.deepEqual(m.liveSources,previous.liveSources);
assert.equal(Object.keys(m.liveSources).length,955);for(const s of Object.values(m.sourceMaps))assert.equal(Object.keys(s).length,956);
if(process.argv.includes('--derivative-live'))for(const[p,h]of Object.entries(m.liveSources))assert.equal(sha(resolve(workspace,p)),h,p);
for(const f of ['Cargo.toml','Cargo.lock'])assert.equal(read('sign-filter-app-mask/'+f),read('sign-filter-app-baseline/'+f).replaceAll('baseline','mask'));
assert.deepEqual(m.checks,checkSignFilterMask());assert.equal(Object.keys(m.binaries).length,6);
for(const b of Object.values(m.binaries)){assert.equal(sha(b.path),b.sha256);assert.equal(statSync(b.path).size,b.bytes);}
for(const[k,b]of Object.entries(json('sign-filter-mask-binaries.json')))assert.deepEqual(m.binaries[k],b);
for(const kind of ['cpu','allocation','trace'])assert.deepEqual(m.binaries['baseline-'+kind],previous.binaries['baseline-'+kind]);
assert.equal(m.binaries['candidate-trace'].bytes-m.binaries['baseline-trace'].bytes,-2304);
const inv=json('inventory.json').sources.find(s=>s.repo==='flint'),coverage=json('coverage.json');
assert.equal(m.readRecords.length,14);let readLines=0,completeReads=0;
for(const r of m.readRecords){
 assert.deepEqual(r,coverage.find(x=>x.repo===r.repo&&x.path===r.path));assert.equal(r.repo,'flint');
 const f=inv.files.find(x=>x.path===r.path);assert(f?.text);assert.equal(sha(resolve(workspace,'exact-real-references/flint',r.path)),f.sha256);
 const n=r.ranges.reduce((n,[a,b])=>{assert(a>=1&&b>=a&&b<=f.lines);return n+b-a+1;},0);
 readLines+=n;completeReads+=n===f.lines;
}
assert.equal(readLines,2196);assert.equal(completeReads,12);
assert.deepEqual(json('high-product-read-selection.json'),m.readRecords.map(r=>({repo:r.repo,path:r.path,newRanges:r.ranges})));
assert.equal(m.gates.length,8);assert.equal(new Set(m.gates.map(g=>g.tag)).size,8);
const gates=new Map(m.gates.map(({tag,code,signal})=>{
 const g=json('results/'+tag+'.json');assert.equal(g.tag,tag);assert.equal(code,0);assert.equal(signal,null);
 assert.equal(g.code,code);assert.equal(g.signal,signal);assert(Date.parse(g.finished)>=Date.parse(g.started));return[tag,g];
}));
const gate=t=>{const g=gates.get('sign-filter-mask-'+t);assert(g,t);return g;};
const env=['CARGO_TARGET_DIR=/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse','CARGO_INCREMENTAL=0','CARGO_PROFILE_DEV_DEBUG=0','CARGO_BUILD_JOBS=2'];
for(const profile of ['debug','release']){
 const g=gate('full-'+profile);assert.equal(g.cwd,resolve('sign-filter-mask/hyperlimit'));assert.equal(g.command,'env');
 assert.deepEqual(g.args,[...env,'cargo','test','--offline','--locked',...(profile==='release'?['--release']:[]),'--all-features','--lib','--tests','--','--test-threads=1']);
}
const app=gate('app-build');assert.equal(app.cwd,resolve('sign-filter-app-mask'));assert.equal(app.command,'env');
assert.deepEqual(app.args,[...env,'cargo','build','--offline','--release','--bins']);
const trace=gate('trace-build');assert.equal(trace.cwd,app.cwd);assert.equal(trace.command,'env');
assert.deepEqual(trace.args,[...env,'cargo','build','--offline','--locked','--release','--features','trace','--bin','audit-sign-filter-mask-trace']);
const run=gate('trace');assert.equal(run.cwd,here);assert.equal(run.command,m.binaries['candidate-trace'].path);assert.deepEqual(run.args,[]);
assert.equal(read('results/sign-filter-mask-trace.stderr'),'');assert(Date.parse(trace.finished)<Date.parse(run.started));
for(const t of ['app-build','trace-build','full-debug','full-release','trace'])assert(Date.parse(gate(t).finished)<Date.parse(gate('cpu').started));
for(const mode of ['cpu','allocation']){
 const g=gate(mode);assert.equal(g.cwd,here);assert.equal(g.command,'node');
 assert.deepEqual(g.args,['run-sign-filter-mask-costs.mjs',mode,...(mode==='cpu'?['/tmp/calcium-sign-filter-mask.ctu9Hl']:[])]);
 assert.equal(read('results/sign-filter-mask-'+mode+'.stderr'),'');
 const s=json('sign-filter-mask-'+mode+'-summary.json');assert(Date.parse(s.started)>=Date.parse(g.started));assert(Date.parse(s.finished)<=Date.parse(g.finished));
 assert.deepEqual(s.binaries,json('sign-filter-mask-binaries.json'));
 assert.deepEqual(read('results/sign-filter-mask-'+mode+'.stdout').trimEnd().split('\n').map(JSON.parse),s.summaries);
}
assert(Date.parse(gate('cpu').finished)<Date.parse(gate('allocation').started));
assert(Date.parse(json('results/sign-filter-ring-oracle.json').finished)<Date.parse(gate('cpu').started));
const check=gate('output-check');assert.equal(check.cwd,here);assert.equal(check.command,'node');
assert.deepEqual(check.args,['check-sign-filter-mask.mjs','--sign-filter-mask-summary']);
assert.deepEqual(JSON.parse(read('results/sign-filter-mask-output-check.stdout')),m.checks);assert.equal(read('results/sign-filter-mask-output-check.stderr'),'');
console.log(JSON.stringify({checkpoint:'bit-mask constant-state filter, unselected; high-product source pass',sourceFiles:956,liveFiles:955,
 tests:{perProfile:364,suites:16,exhaustive:299593,long:1280,privateTraces:32,publicTraces:45},
 cpu:m.checks.cpu,allocation:m.checks.allocation,binaryDeltas:m.checks.benchBinaryDeltas,readLines,completeReads,partialReads:2,
 newSnapshotBytes:Object.entries(m.binaries).filter(([k])=>k.startsWith('candidate-')).reduce((n,[,b])=>n+b.bytes,0),
 production:m.production,status:m.status,limits:m.limits,followup:m.followup}));
