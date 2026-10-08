import './verify-spectral.mjs';
import { readFileSync, statSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
import { sources, sha, json } from './derivative-demand-sources.mjs';
import { costs } from './check-derivative-costs.mjs';
import { effectiveSummary } from './effective-coverage.mjs';
const here=dirname(fileURLToPath(import.meta.url)),workspace=resolve(here,'../../../..');
const read=p=>readFileSync(resolve(here,p),'utf8');
const m=process.argv.includes('--draft-derivative-demand')?(await import('./bind-derivative-demand.mjs')).manifest:json('derivative-demand-experiment.json');
for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
assert.equal(Object.keys(m.files).length,70);
for(const[p,b]of Object.entries(m.binaries)){assert.equal(sha(p),b.sha256,p);assert.equal(statSync(p).size,b.bytes);}
assert.equal(Object.keys(m.binaries).length,5);
assert.deepEqual(m.candidateSources,sources());
for(const s of Object.values(m.candidateSources))assert.equal(Object.keys(s).length,955);
assert.deepEqual(m.costs,costs());
assert.deepEqual(m.costs.allocation,{requests:{lower:16,equal:64,higher:0},requested_bytes:{lower:16,equal:64,higher:0},peak_delta:{lower:0,equal:80,higher:0}});
assert.equal(m.costs.nonzeroLiveGroups,14);assert.equal(m.costs.maxSharedLiveDelta,126728);
assert.equal(m.costs.intervalBelowOne,55);assert.equal(m.costs.intervalAboveOne,1);
assert.deepEqual(m.nativeLibrary,json('spectral-experiment.json').nativeLibrary);assert.equal(sha(m.nativeLibrary.path),m.nativeLibrary.sha256);
const selection=json('derivative-lll-read-selection.json'),coverage=json('coverage.json'),inventory=json('inventory.json');
assert.equal(selection.length,15);assert.equal(m.reads.length,15);let lines=0;
for(const[i,s]of selection.entries()) {
  const e=m.reads[i],live=coverage.find(v=>v.repo===s.repo&&v.path===s.path);
  assert.equal(e.repo,s.repo);assert.equal(e.path,s.path);assert.deepEqual(live,e);
  const f=inventory.sources.find(v=>v.repo===e.repo).files.find(v=>v.path===e.path);
  assert.deepEqual(e.ranges,[[1,f.lines]]);assert.equal(sha(resolve(workspace,'exact-real-references',e.repo,e.path)),f.sha256);
  if(s.previous) {assert.deepEqual(s.previous.ranges,[[765,821]]);assert.deepEqual(s.newRanges,[[1,764]]);assert(e.note.startsWith(s.previous.note));}
  else assert.deepEqual(s.newRanges,e.ranges);
  lines+=s.newRanges.reduce((n,[a,b])=>n+b-a+1,0);
}
assert.equal(lines,2980);
assert.deepEqual(m.coverageAtBinding,[{repo:'calcium',reviewed:375,complete:374,partial:1,readLines:37085},
 {repo:'flint',reviewed:414,complete:403,partial:11,readLines:45933}]);
for(const s of effectiveSummary()) {const old=m.coverageAtBinding.find(v=>v.repo===s.repo);assert(s.readLines>=old.readLines&&s.complete>=old.complete);}
const retained=json('retained-monic.json');
for(const[p,ranges]of Object.entries(m.hyperReadRanges)) {
  const source=retained.frozenSnapshot+'/'+p,t=read(source),n=t.split('\n').length-+t.endsWith('\n');
  assert.equal(sha(source),retained.liveSources[p],p);assert(ranges.every(([a,b])=>a>=1&&b>=a&&b<=n));
}
const specs=[
 ['derivative-demand-candidate-focused-debug',101],
 ['derivative-demand-candidate-focused-debug-v2',0],['derivative-demand-baseline-focused-debug',0],
 ['derivative-demand-baseline-focused-release',0],['derivative-demand-candidate-focused-release',0],
 ['derivative-cost-baseline-build',0],['derivative-cost-candidate-build',0],
 ['derivative-demand-candidate-fmt',0],['derivative-public-baseline-memcheck',0],['derivative-public-candidate-memcheck',0],
 ['lll-controls-compile',0],['lll-controls-native',0],['lll-controls-memcheck',0],
 ['derivative-cost-cpu',0],['derivative-cost-allocation',0]];
assert.deepEqual(m.gates,specs.map(([tag,code])=>({tag,code,signal:null})));
function gate(tag) {
 const spec=specs.find(s=>s[0]===tag);assert(spec);const g=json('results/'+tag+'.json');
 assert.equal(g.tag,tag);assert.equal(g.code,spec[1]);assert.equal(g.signal,null);
 assert(Date.parse(g.started)<=Date.parse(g.finished));
 return {...g,stdout:read('results/'+tag+'.stdout'),stderr:read('results/'+tag+'.stderr')};
}
const env=['CARGO_INCREMENTAL=0','CARGO_PROFILE_DEV_DEBUG=0','CARGO_BUILD_JOBS=2','CARGO_TARGET_DIR=/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse'];
for(const variant of ['baseline','candidate'])for(const profile of ['debug','release']) {
 const tag='derivative-demand-'+variant+'-focused-'+profile+(variant==='candidate'&&profile==='debug'?'-v2':'');
 const g=gate(tag);assert.equal(g.command,'env');assert.equal(g.cwd,resolve(here,'derivative-demand-'+variant+'/hypercurve'));
 assert.deepEqual(g.args,[...env,'cargo','test','--offline',...(profile==='release'?['--release']:[]),'--lib','derivative_demand','--','--test-threads=1']);
 assert.match(g.stdout,/running 3 tests/);assert.match(g.stdout,/3 passed; 0 failed; 0 ignored; 0 measured; 911 filtered out/);
 const names=g.stdout.split('\n').filter(l=>l.startsWith('test rational_bezier_general::derivative_demand_tests::'));
 assert.equal(names.length,3);assert(names.every(l=>l.endsWith(' ... ok')));
 for(const name of ['derivative_demand_checked_capacity_and_exact_zero_tails','derivative_demand_exact_monomial_oracles','derivative_demand_preserves_all_rational_quotient_orders'])
   assert(names.some(l=>l.includes('::'+name+' ... ok')));
 assert(!g.stderr.includes('error:'));
}
const failed=gate('derivative-demand-candidate-focused-debug');
assert.equal(failed.stdout,'');assert.match(failed.stderr,/error\[E0308\]/);assert.match(failed.stderr,/expected .*i64.*found/);
assert(read('derivative-demand-harness-initial.rs').includes('HyperRational::new(q)'));
assert(!read('derivative-demand-candidate/hypercurve/src/derivative_demand_tests.rs').includes('HyperRational::new(q)'));
assert(Date.parse(failed.finished)<=Date.parse(gate('derivative-demand-candidate-focused-debug-v2').started));
for(const v of ['baseline','candidate']) {
 const g=gate('derivative-cost-'+v+'-build');assert.equal(g.command,'env');
 assert.equal(g.cwd,resolve(here,'derivative-cost-'+v));assert.deepEqual(g.args,[...env,'cargo','build','--offline','--release','--bins']);
 assert(g.stderr.includes('Finished `release` profile'));assert.equal(g.stdout,'');
}
const fmt=gate('derivative-demand-candidate-fmt');assert.equal(fmt.command,'cargo');assert.deepEqual(fmt.args,['fmt','--all','--','--check']);assert.equal(fmt.stdout,'');assert.equal(fmt.stderr,'');
const binaries=json('derivative-cost-binaries.json');
const memoryArgs=['--vgdb=no','--tool=memcheck','--error-exitcode=97','--leak-check=full','--show-leak-kinds=all','--errors-for-leak-kinds=definite,indirect,possible'];
for(const v of ['baseline','candidate']) {
 const g=gate('derivative-public-'+v+'-memcheck');assert.equal(g.command,'valgrind');
 assert.deepEqual(g.args,[...memoryArgs,'/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse/release/calcium-derivative-'+v+'-cpu',v,'1','24','128','fresh_curve','8']);
 const out=JSON.parse(g.stdout);for(const[k,x]of Object.entries({mode:'cpu',variant:v,kind:1,degree:24,order:128,lifecycle:'fresh_curve',iterations:8}))assert.equal(out[k],x);
 for(const key of ['requests','requested_bytes','live_delta','peak_delta'])assert.equal(out[key],0);
 assert.match(g.stderr,/ERROR SUMMARY: 0 errors from 0 contexts/);
 for(const kind of ['definitely','indirectly','possibly'])assert(new RegExp(kind+' lost: 0 bytes in 0 blocks').test(g.stderr));
 assert.match(g.stderr,/still reachable: 264,184 bytes in 2,137 blocks/);
 const b=binaries[v+'-cpu'];assert.equal(m.binaries[b.path].sha256,b.sha256);
}
const native=gate('lll-controls-native'),mem=gate('lll-controls-memcheck'),compile=gate('lll-controls-compile');
assert.equal(native.command,'/tmp/calcium-derivative-costs.FA5c2M/lll-controls');assert.deepEqual(native.args,[]);assert.equal(native.stderr,'');
assert.equal(compile.command,'gcc');assert.equal(compile.args.at(-1),native.command);assert.equal(compile.stdout,'');assert.equal(compile.stderr,'');
assert.deepEqual(compile.args,['-O2','-g0','-Wall','-Wextra','-Werror','-isystem','../exact-real-references/flint/src',
 'audits/continuation/calcium/flint-lll-controls.c','-L','../exact-real-references/flint',
 '-Wl,-rpath,/home/tim/Documents/GitHub/workspace/exact-real-references/flint','-lflint','-lmpfr','-lgmp','-lm','-lpthread','-o',native.command]);
assert.equal(mem.stdout,native.stdout);assert.equal(mem.command,'valgrind');assert.deepEqual(mem.args,[...memoryArgs,native.command]);
assert.match(mem.stderr,/ERROR SUMMARY: 0 errors from 0 contexts/);assert.match(mem.stderr,/in use at exit: 0 bytes in 0 blocks/);
assert.match(mem.stderr,/179,867 allocs, 179,867 frees, 42,930,504 bytes allocated/);
const rows=native.stdout.trimEnd().split('\n');
assert.equal(rows.shift(),'dimension,pattern,shift,representation,rounding,method,expected,result,round_preserved');
assert.deepEqual(JSON.parse(rows.pop()),{suite:'lll-controls',rows:3360,failures:0});assert.equal(rows.length,3360);
let index=0;const tally=[0,0,0],uncertified=[0,0,0];
for(let n=0;n<=6;n++)for(let p=0;p<4;p++)for(const shift of [0,24,60,500,1200])
for(let rep=0;rep<2;rep++)for(let round=0;round<4;round++)for(let method=0;method<3;method++) {
 const r=rows[index++].split(',').map(Number),expected=+(n<=1||p<2);
 assert.deepEqual(r.slice(0,7),[n,p,shift,rep,round,method,expected]);assert.equal(r[8],1);assert([0,1].includes(r[7]));
 assert(!r[7]||expected);if(method===2)assert.equal(r[7],expected);
 tally[method]+=r[7];uncertified[method]+=+(expected&&!r[7]);
}
assert.deepEqual(tally,[640,720,720]);assert.deepEqual(uncertified,[80,0,0]);
const cpu=json('derivative-cost-cpu-summary.json');
for(const[tag]of specs.filter(s=>s[0]!=='derivative-cost-cpu')) {
 const g=gate(tag);assert(Date.parse(g.finished)<=Date.parse(cpu.started)||Date.parse(g.started)>=Date.parse(cpu.finished),tag);
}
for(const mode of ['cpu','allocation']) {
 const g=gate('derivative-cost-'+mode);assert.equal(g.command,'node');assert.equal(g.cwd,workspace);
 assert.deepEqual(g.args,['exactcore-hyper-comparison/audits/continuation/calcium/run-derivative-costs.mjs',mode,...(mode==='cpu'?['/tmp/calcium-derivative-costs.FA5c2M']:[])]);
}
console.log(JSON.stringify({checkpoint:'derivative demand and LLL support',newReadLines:lines,completedFiles:15,coverageAtBinding:m.coverageAtBinding,
 sourceFilesPerVariant:955,focusedTests:'3 per baseline/candidate/profile; default features, not full consumer qualification',costs:m.costs,
 lllRows:3360,lllMemory:'zero errors/all allocations freed',publicMemory:'zero errors/losses; 264184 reachable bytes each',
 status:m.status,limits:m.limits}));
