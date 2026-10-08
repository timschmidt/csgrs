import './verify-derivative-demand.mjs';
import { readFileSync,statSync } from 'node:fs';
import { dirname,resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
import { sources,sha,json } from './derivative-demand-sources.mjs';
import { qualificationCosts } from './check-derivative-qualification-costs.mjs';
import { effectiveSummary } from './effective-coverage.mjs';
const here=dirname(fileURLToPath(import.meta.url)),workspace=resolve(here,'../../../..');
const read=p=>readFileSync(resolve(here,p),'utf8');
const draft=process.argv.includes('--draft-derivative-qualification');
const m=draft?(await import('./bind-derivative-qualification.mjs')).manifest:json('derivative-qualification-experiment.json');
assert.equal(m.draft,draft);
for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
assert.equal(Object.keys(m.files).length,draft?141:150);
assert.deepEqual(m.candidateSources,sources());assert.equal(Object.keys(m.candidateSources.candidate).length,955);
for(const[p,b]of Object.entries(m.binaries)){assert.equal(sha(p),b.sha256,p);assert.equal(statSync(p).size,b.bytes);}
assert.equal(Object.keys(m.binaries).length,14);
assert.deepEqual(m.costs,qualificationCosts());
assert.equal(m.costs.maxLiveAt32,196376);assert.equal(m.costs.intervalBelowOne,5);assert.equal(m.costs.intervalAboveOne,0);
assert.equal(m.costs.endpointAllocationEqualGroups,16);assert.equal(m.costs.endpointAllocationNonzeroLiveGroups,1);
const coverage=json('coverage.json'),inventory=json('inventory.json'),selection=json('derivative-qualification-read-selection.json');
assert.equal(selection.length,5);let addedLines=0,complete=0;
for(const [i,s]of selection.entries()) {
 const e=m.reads[i];assert.deepEqual(e,coverage.find(e=>e.repo===s.repo&&e.path===s.path));
 assert.deepEqual(e.ranges,s.newRanges);
 const f=inventory.sources.find(v=>v.repo===e.repo).files.find(v=>v.path===e.path);
 assert.equal(sha(resolve(workspace,'exact-real-references',e.repo,e.path)),f.sha256);
 addedLines+=e.ranges.reduce((n,[a,b])=>n+b-a+1,0);
 complete+=+(e.ranges.length===1&&e.ranges[0][0]===1&&e.ranges[0][1]===f.lines);
}
assert.equal(addedLines,1653);assert.equal(complete,4);
assert.deepEqual(m.coverageAtBinding,[{repo:'calcium',reviewed:375,complete:374,partial:1,readLines:37085},
 {repo:'flint',reviewed:419,complete:407,partial:12,readLines:47586}]);
for(const s of effectiveSummary()) {const old=m.coverageAtBinding.find(v=>v.repo===s.repo);assert(s.complete>=old.complete&&s.readLines>=old.readLines);}
const retained=json('retained-monic.json');
for(const[p,ranges]of Object.entries(m.hyperReadRanges)) {
 const source=retained.frozenSnapshot+'/'+p,n=read(source).trimEnd().split('\n').length;
 assert.equal(sha(source),retained.liveSources[p],p);assert(ranges.every(([a,b])=>a>=1&&b>=a&&b<=n));
}
assert.equal(m.gates.length,draft?37:40);assert.equal(new Set(m.gates.map(g=>g.tag)).size,m.gates.length);
const gates=new Map(m.gates.map(({tag,code,signal})=>{
 const g=json('results/'+tag+'.json');assert.equal(g.tag,tag);assert.equal(g.code,code,tag);assert.equal(g.signal,signal);assert.equal(signal,null);
 assert(Date.parse(g.started)<=Date.parse(g.finished));return[tag,{...g,stdout:read('results/'+tag+'.stdout'),stderr:read('results/'+tag+'.stderr')}];
}));
const gate=tag=>{const g=gates.get(tag);assert(g,tag);return g;};
const env=['CARGO_TARGET_DIR=/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse','CARGO_INCREMENTAL=0','CARGO_PROFILE_DEV_DEBUG=0','CARGO_BUILD_JOBS=2'];
function cargo(tag,cwd,args) {const g=gate(tag);assert.equal(g.command,'env');assert.equal(g.cwd,resolve(here,cwd));assert.deepEqual(g.args,[...env,'cargo',...args]);return g;}
const names=t=>[...t.matchAll(/^test (.+) \.\.\. (ok|ignored|FAILED).*$/gm)].map(m=>m[1]+':'+m[2]).sort();
const curve=cargo('derivative-candidate-consumer-all-debug','derivative-demand-candidate/hypercurve',['test','--offline','--locked','--all-features','--lib','--tests']);
const previous=names(read('results/monic-qualified-consumer-hypercurve-debug.stdout'));
const focused=['derivative_demand_checked_capacity_and_exact_zero_tails','derivative_demand_exact_monomial_oracles','derivative_demand_preserves_all_rational_quotient_orders']
 .map(n=>'rational_bezier_general::derivative_demand_tests::'+n+':ok').sort();
assert.equal(previous.length,1770);assert.deepEqual(names(curve.stdout),previous.concat(focused).sort());
const suites=[...curve.stdout.matchAll(/test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;/g)];
assert.equal(suites.length,45);assert.equal(suites.reduce((n,s)=>n+ +s[1],0),1764);assert.equal(suites.reduce((n,s)=>n+ +s[3],0),9);
assert(suites.every(s=>s[2]==='0'&&s[4]==='0'&&s[5]==='0'));
cargo('derivative-candidate-clippy-all','derivative-demand-candidate/hypercurve',['clippy','--offline','--locked','--all-features','--all-targets','--','-D','warnings']);
for(const v of ['baseline','candidate']) {
 const all=cargo(v==='baseline'?'derivative-baseline-wasm-all':'derivative-candidate-wasm','derivative-demand-'+v+'/hypercurve',
  ['check','--offline','--locked','--all-features','--lib','--target','wasm32-unknown-unknown']);
 assert.equal(all.code,101);assert.match(all.stderr,/wasm32-unknown-unknown targets are not supported by default/);
 assert.match(all.stderr,/getrandom-0\.4\.2/);
 const lib=cargo('derivative-'+v+'-wasm-library','derivative-demand-'+v+'/hypercurve',
  ['check','--offline','--locked','--lib','--features','dispatch-trace,triangulation,svg,hershey','--target','wasm32-unknown-unknown']);
 assert.equal(lib.code,0);assert(!lib.stderr.includes('error:'));
}
const tree=gate('derivative-wasm-dependency-tree');assert.equal(tree.command,'cargo');
assert.deepEqual(tree.args,['tree','--offline','--locked','--all-features','--target','wasm32-unknown-unknown','-i','getrandom@0.4.2']);
for(const p of ['getrandom v0.4.2','rand v0.10.2','curvo v0.1.91','hypercurve v0.3.1'])assert(tree.stdout.includes(p));
const states=['fresh','input-roundtrip','warm-32','warm-128','warm-512','after-abort',
 ...[0,1,2,3].flatMap(w=>[0,1,2,3].map(i=>'worker'+w+'-'+i))];
const expected=[];
for(let kind=0;kind<4;kind++)for(let parameter=0;parameter<3;parameter++)for(const order of [0,1,3,24,80,128])
 for(const state of states)expected.push({coordinates:2*order,kind,order,parameter,result:'Equal',state});
assert.equal(expected.length,1584);
const memoryArgs=['--vgdb=no','--tool=memcheck','--error-exitcode=97','--leak-check=full','--show-leak-kinds=all','--errors-for-leak-kinds=definite,indirect,possible'];
const apps=json('derivative-app-size-summary.json'),binaries=json('derivative-endpoint-binaries.json');
for(const v of ['baseline','candidate']) {
 for(const profile of ['debug','release']) {
  const g=cargo('derivative-state-'+v+'-'+profile,'derivative-state-'+v,['run','--offline',...(profile==='release'?['--locked','--release']:[]),'--bin','calcium-derivative-state-'+v]);
  const rows=g.stdout.trimEnd().split('\n').map(JSON.parse);
  assert.deepEqual(rows.pop(),{domain_rejections:10,queries:1584,suite:'derivative-state',unchanged_inputs:72,workers_per_case:4});
  assert.deepEqual(rows,expected);
 }
 const mem=gate('derivative-state-'+v+'-memcheck'),state=apps.state.find(s=>s.variant===v);
 assert.equal(mem.command,'valgrind');assert.deepEqual(mem.args,[...memoryArgs,state.path]);
 assert.equal(mem.stdout,gate('derivative-state-'+v+'-debug').stdout);assert.equal(mem.code,97);
 for(const kind of ['definitely','indirectly'])assert.match(mem.stderr,new RegExp(kind+' lost: 0 bytes in 0 blocks'));
 assert.match(mem.stderr,/possibly lost: 48 bytes in 1 blocks/);assert.match(mem.stderr,/still reachable: 347,608 bytes in 3,357 blocks/);
 assert.match(mem.stderr,/ERROR SUMMARY: 1 errors from 1 contexts \(suppressed: 0 from 0\)/);
 assert.match(mem.stderr,/48 bytes in 1 blocks are possibly lost[\s\S]*std::thread::thread::Thread[\s\S]*std::thread::current::init_current/);
 const failed=cargo('derivative-endpoint-'+v+'-build','derivative-endpoint-'+v,['build','--offline','--release','--bins']);
 assert.equal(failed.code,101);assert.match(failed.stderr,/error\[E0061\]/);assert.match(failed.stderr,/error\[E0599\]/);
 const fixed=cargo('derivative-endpoint-'+v+'-build-v2','derivative-endpoint-'+v,['build','--offline','--locked','--release','--bins']);
 assert.equal(fixed.code,0);assert(Date.parse(failed.finished)<Date.parse(fixed.started));
 const endpoint=gate('derivative-endpoint-'+v+'-memcheck');assert.equal(endpoint.command,'valgrind');
 assert.deepEqual(endpoint.args,[...memoryArgs,binaries[v+'-cpu'].path,v,'1','24','fresh_graph','8']);
 assert.equal(endpoint.code,0);assert.match(endpoint.stderr,/ERROR SUMMARY: 0 errors from 0 contexts/);
 for(const kind of ['definitely','indirectly','possibly'])assert.match(endpoint.stderr,new RegExp(kind+' lost: 0 bytes in 0 blocks'));
 assert.match(endpoint.stderr,/still reachable: 63,176 bytes in 489 blocks/);
 const out=JSON.parse(endpoint.stdout);for(const[k,x]of Object.entries({variant:v,mode:'cpu',kind:1,degree:24,lifecycle:'fresh_graph',iterations:8}))assert.equal(out[k],x);
 for(const k of ['requests','requested_bytes','live_delta','peak_delta'])assert.equal(out[k],0);
 cargo('derivative-app-'+v+'-build','derivative-demand-'+v+'/hypercurve',
  ['build','--offline','--locked','--release','--example','basic','--example','arrangement']);
 for(const example of ['basic','arrangement']) {
  const artifact=apps.artifacts.find(a=>a.variant===v&&a.example===example);assert(artifact);
  const [original,stripped]=artifact.files;
  const strip=gate('derivative-app-'+v+'-strip-'+example);assert.equal(strip.command,'strip');assert.deepEqual(strip.args,['--strip-all','-o',stripped.path,original.path]);
  const size=gate('derivative-app-'+v+'-size-'+example);assert.equal(size.command,'size');assert.deepEqual(size.args,[original.path,stripped.path]);
  const run=gate('derivative-app-'+v+'-run-'+example);assert.equal(run.command,stripped.path);assert.deepEqual(run.args,[]);assert.equal(run.stdout,'');assert.equal(run.stderr,'');
 }
}
assert.equal(gate('derivative-state-baseline-debug').stdout,gate('derivative-state-candidate-debug').stdout);
for(const [example,delta]of [['basic',224],['arrangement',208]]) {
 const b=apps.artifacts.find(a=>a.variant==='baseline'&&a.example===example),c=apps.artifacts.find(a=>a.variant==='candidate'&&a.example===example);
 assert.equal(c.files[1].bytes-b.files[1].bytes,delta);
}
assert(read('derivative-endpoint-corpus-initial.rs').includes('BezierParameter2::exact(Real::zero())'));
assert(!read('derivative-endpoint-corpus.rs').includes('BezierParameter2::exact(Real::zero())'));
const cpu=json('derivative-endpoint-cpu-summary.json');
for(const[tag,g]of gates)if(tag!=='derivative-endpoint-cpu')
 assert(Date.parse(g.finished)<=Date.parse(cpu.started)||Date.parse(g.started)>=Date.parse(cpu.finished),tag+' overlaps CPU');
for(const mode of ['cpu','allocation']) {
 const g=gate('derivative-endpoint-'+mode);assert.equal(g.command,'node');assert.equal(g.cwd,workspace);
 assert.deepEqual(g.args,['exactcore-hyper-comparison/audits/continuation/calcium/run-derivative-endpoint.mjs',mode]);
}
if(!draft) {
 for(const profile of ['debug','release']) {
  const g=gate('derivative-retained-focused-'+profile);
  assert.equal(g.cwd,resolve(workspace,'hypercurve'));assert.equal(g.command,'env');
  assert.deepEqual(g.args,[...env,'cargo','test','--offline','--locked',...(profile==='release'?['--release']:[]),'--lib','derivative_demand','--','--test-threads=1']);
  assert.deepEqual(names(g.stdout),focused);assert.match(g.stdout,/3 passed; 0 failed; 0 ignored/);
 }
 const fmt=gate('derivative-retained-fmt');assert.equal(fmt.command,'cargo');assert.deepEqual(fmt.args,['fmt','--all','--','--check']);assert.equal(fmt.stdout,'');assert.equal(fmt.stderr,'');
}
if(process.argv.includes('--derivative-live')) {
 assert(!draft);
 for(const[p,h]of Object.entries(m.candidateSources.candidate))assert.equal(sha(resolve(workspace,p)),h,'live '+p);
}
console.log(JSON.stringify({checkpoint:'derivative qualification and retention',draft,newReadLines:addedLines,completedFiles:complete,
 coverageAtBinding:m.coverageAtBinding,sourceFilesPerVariant:955,consumerPassed:1764,consumerIgnored:9,
 stateQueriesPerRun:1584,stateMemory:'48-byte possible Rust thread initialization loss in both, no definite/indirect losses',
 endpointMemory:'zero errors/losses, 63176 reachable bytes each',costs:m.costs,
 strippedExampleByteDeltas:[224,208],status:m.status,limits:m.limits}));
