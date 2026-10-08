import './verify-nfixed.mjs';
import {readFileSync,statSync} from 'node:fs';
import {dirname,resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {sources,sha,json} from './derivative-demand-sources.mjs';
import {effectiveCoverage,effectiveSummary} from './effective-coverage.mjs';
import {checkComplexControls,checkFilterProbe} from './check-nfloat-complex-controls.mjs';
const here=dirname(fileURLToPath(import.meta.url)),workspace=resolve(here,'../../../..');
const read=p=>readFileSync(resolve(here,p),'utf8'),m=json('nfloat-complex-experiment.json');
assert.equal(Object.keys(m.files).length,49);
for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
assert.deepEqual(m.candidateSources,sources());assert.deepEqual(m.candidateSources,json('nfixed-experiment.json').candidateSources);
assert.equal(Object.keys(m.candidateSources.candidate).length,955);
if(process.argv.includes('--derivative-live'))for(const[p,h]of Object.entries(m.candidateSources.candidate))assert.equal(sha(resolve(workspace,p)),h,'live '+p);
assert.equal(m.binaries.length,2);
for(const b of m.binaries){assert.equal(sha(b.path),b.sha256);assert.equal(statSync(b.path).size,b.bytes);}
assert.deepEqual(m.binaries.map(b=>b.bytes),[22504,2219752]);
for(const[p,h]of Object.entries(m.libraries))assert.equal(sha(p),h,p);
assert.deepEqual(m.libraries,json('nfixed-experiment.json').libraries);
assert.deepEqual(m.controls,checkComplexControls());assert.deepEqual(m.probe,checkFilterProbe());
const coverage=json('coverage.json'),inventory=json('inventory.json'),selection=json('nfloat-complex-read-selection.json');
assert.equal(selection.length,6);assert.equal(m.reads.length,6);let newLines=0;
for(const[i,s]of selection.entries()) {
 const e=coverage.find(e=>e.repo===s.repo&&e.path===s.path);assert.deepEqual(e,m.reads[i]);assert.deepEqual(e.ranges,s.newRanges);
 const f=inventory.sources.find(v=>v.repo===s.repo).files.find(v=>v.path===s.path);assert.deepEqual(e.ranges,[[1,f.lines]]);
 assert.equal(sha(resolve(workspace,'exact-real-references',s.repo,s.path)),f.sha256,s.path);newLines+=f.lines;
}
assert.equal(newLines,2976);assert.deepEqual(m.coverageAtBinding,[{repo:'calcium',reviewed:375,complete:374,partial:1,readLines:37085},
 {repo:'flint',reviewed:444,complete:432,partial:12,readLines:61416}]);
for(const s of effectiveSummary()){const old=m.coverageAtBinding.find(v=>v.repo===s.repo);assert(s.complete>=old.complete&&s.readLines>=old.readLines);}
const all=inventory.sources.find(s=>s.repo==='flint').files.filter(f=>f.path.startsWith('src/nfloat/'));
assert.equal(all.length,26);assert.equal(m.nfloatDirectory.length,26);
const effective=effectiveCoverage();
for(const f of all) {
 const e=m.nfloatDirectory.find(e=>e.path===f.path);assert(e);assert.deepEqual(e.ranges,[[1,f.lines]]);
 assert.deepEqual(e.ranges,effective.find(e=>e.repo==='flint'&&e.path===f.path).ranges);
}
for(const[p,ranges]of Object.entries(m.hyperReadRanges)) {
 const f='derivative-demand-candidate/'+p,lines=read(f).trimEnd().split('\n').length;assert.equal(sha(f),m.candidateSources.candidate[p],p);
 assert(ranges.every(([a,b],i)=>a>=1&&b>=a&&b<=lines&&(!i||a>ranges[i-1][1])),p);
}
assert.equal(m.gates.length,12);assert.equal(new Set(m.gates.map(g=>g.tag)).size,12);
const gates=new Map(m.gates.map(({tag,code,signal})=>{const g=json('results/'+tag+'.json');
 assert.equal(g.tag,tag);assert.equal(g.code,code,tag);assert.equal(g.signal,signal);assert.equal(signal,null);
 assert(Date.parse(g.finished)>=Date.parse(g.started));return[tag,g];}));
const gate=t=>{const g=gates.get(t);assert(g,t);return g;};
for(const tag of ['nfloat-complex-compile','nfloat-complex-compile-v2']) {
 const g=gate(tag);assert.equal(g.command,'gcc');assert.equal(g.cwd,resolve(workspace,'exactcore-hyper-comparison'));
 assert.deepEqual(g.args,['-O2','-g0','-Wall','-Wextra','-Werror','-isystem','../exact-real-references/flint/src',
  'audits/continuation/calcium/flint-nfloat-complex-controls.c','-L','../exact-real-references/flint',
  '-Wl,-rpath,'+resolve(workspace,'exact-real-references/flint'),'-lflint','-lmpfr','-lgmp','-lm','-lpthread','-o',m.binaries[0].path]);
 assert.equal(read('results/'+tag+'.stdout'),'');
}
assert.match(read('results/nfloat-complex-compile.stderr'),/Werror=misleading-indentation/);assert.equal(read('results/nfloat-complex-compile-v2.stderr'),'');
assert.equal(read('flint-nfloat-complex-controls-initial.c').replace('mpq_div(expected[i],expected[i],denom); break;',
 'mpq_div(expected[i],expected[i],denom);\n                    break;'),read('flint-nfloat-complex-controls.c'));
const native=gate('nfloat-complex-native');assert.equal(native.command,m.binaries[0].path);assert.deepEqual(native.args,[]);
assert.equal(read('results/nfloat-complex-native.stderr'),'');
const mem=gate('nfloat-complex-memcheck');assert.equal(mem.command,'valgrind');
assert.deepEqual(mem.args,['--vgdb=no','--tool=memcheck','--error-exitcode=97','--leak-check=full','--show-leak-kinds=all',
 '--errors-for-leak-kinds=definite,indirect,possible',m.binaries[0].path]);
const env=['CARGO_TARGET_DIR=/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse','CARGO_INCREMENTAL=0','CARGO_PROFILE_DEV_DEBUG=0','CARGO_BUILD_JOBS=2'];
for(const profile of ['debug','release']) {
 const g=gate('nfloat-complex-hyper-'+profile);assert.equal(g.command,'env');assert.equal(g.cwd,resolve(workspace,'hyperlattice'));
 assert.deepEqual(g.args,[...env,'cargo','test','--offline','--locked',...(profile==='release'?['--release']:[]),'--test','complex','--','--test-threads=1']);
}
for(const v2 of [false,true]) {
 const g=gate('nfloat-complex-filter-probe-build'+(v2?'-v2':''));assert.equal(g.command,'env');assert.equal(g.cwd,resolve(here,'sign-filter-probe'));
 assert.deepEqual(g.args,[...env,'cargo','build','--offline',...(v2?['--locked']:[]),'--release']);
}
assert.match(read('results/nfloat-complex-filter-probe-build.stderr'),/expected `u64`, found `i64`/);
assert.equal(read('sign-filter-probe-initial.rs').replace('(n-2) as i64','(n-2) as u64'),read('sign-filter-probe/src/main.rs'));
const probe=gate('nfloat-complex-filter-probe-native');assert.equal(probe.command,m.binaries[1].path);assert.deepEqual(probe.args,[]);
assert.equal(read('results/nfloat-complex-filter-probe-native.stderr'),'');
const check=gate('nfloat-complex-output-check');assert.equal(check.command,'node');assert.equal(check.cwd,here);
assert.deepEqual(check.args,['check-nfloat-complex-controls.mjs','--complex-summary']);assert.equal(read('results/nfloat-complex-output-check.stderr'),'');
assert.deepEqual(JSON.parse(read('results/nfloat-complex-output-check.stdout')),{controls:m.controls,probe:m.probe});
const inv=gate('nfloat-complex-inventory');assert.equal(inv.command,'node');assert.equal(inv.cwd,workspace);
assert.deepEqual(inv.args,['exactcore-hyper-comparison/audits/continuation/calcium/inventory.mjs','verify']);assert.equal(read('results/nfloat-complex-inventory.stderr'),'');
assert.deepEqual(read('results/nfloat-complex-inventory.stdout').trimEnd().split('\n').map(JSON.parse).map(s=>[s.repo,s.commit]),inventory.sources.map(s=>[s.repo,s.commit]));
const linked=gate('nfloat-complex-linked-libraries');assert.equal(linked.command,'ldd');assert.deepEqual(linked.args,[m.binaries[0].path]);
assert.equal(read('results/nfloat-complex-linked-libraries.stderr'),'');
assert.deepEqual(Object.keys(m.libraries),[...read('results/nfloat-complex-linked-libraries.stdout').matchAll(/=> (\/[^ ]+) \(/g)].map(m=>m[1]));
assert(Date.parse(gate('nfloat-complex-compile').finished)<Date.parse(gate('nfloat-complex-compile-v2').started));
assert(Date.parse(gate('nfloat-complex-compile-v2').finished)<Date.parse(native.started));assert(Date.parse(native.finished)<Date.parse(mem.started));
assert(Date.parse(gate('nfloat-complex-filter-probe-build').finished)<Date.parse(gate('nfloat-complex-filter-probe-build-v2').started));
assert(Date.parse(gate('nfloat-complex-filter-probe-build-v2').finished)<Date.parse(probe.started));
console.log(JSON.stringify({checkpoint:'nfloat complex closure',newReadLines:newLines,completedFiles:6,nfloatFiles:all.length,
 coverageAtBinding:m.coverageAtBinding,controls:m.controls.summary,memory:m.controls.memory,probe:m.probe,
 binaryBytes:m.binaries.reduce((n,b)=>n+b.bytes,0),sourceFiles:955,production:m.production,status:m.status,limits:m.limits,followup:m.followup}));
