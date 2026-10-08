import './verify-nfloat-support.mjs';
import {readFileSync,statSync} from 'node:fs';
import {dirname,resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {sources,sha,json} from './derivative-demand-sources.mjs';
import {effectiveSummary} from './effective-coverage.mjs';
import {checkFixedControls} from './check-nfixed-controls.mjs';
const here=dirname(fileURLToPath(import.meta.url)),workspace=resolve(here,'../../../..');
const read=p=>readFileSync(resolve(here,p),'utf8'),m=json('nfixed-experiment.json');
for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
assert.equal(Object.keys(m.files).length,47);assert.deepEqual(m.candidateSources,sources());
assert.equal(Object.keys(m.candidateSources.candidate).length,955);
if(process.argv.includes('--derivative-live'))for(const[p,h]of Object.entries(m.candidateSources.candidate))assert.equal(sha(resolve(workspace,p)),h,'live '+p);
assert.equal(sha(m.binary.path),m.binary.sha256);assert.equal(statSync(m.binary.path).size,m.binary.bytes);assert.equal(m.binary.bytes,27352);
assert.equal(sha(m.fenvBinary.path),m.fenvBinary.sha256);assert.equal(statSync(m.fenvBinary.path).size,m.fenvBinary.bytes);
for(const[p,h]of Object.entries(m.libraries))assert.equal(sha(p),h,p);
assert.deepEqual(m.libraries,json('nfloat-support-experiment.json').libraries);assert.deepEqual(m.controls,checkFixedControls());
const coverage=json('coverage.json'),inventory=json('inventory.json'),selection=json('nfixed-read-selection.json');
assert.equal(selection.length,13);assert.equal(m.reads.length,13);let newLines=0;
for(const[i,s]of selection.entries()) {
 const e=coverage.find(e=>e.repo===s.repo&&e.path===s.path);assert.deepEqual(e,m.reads[i]);assert.deepEqual(e.ranges,s.newRanges);
 const f=inventory.sources.find(v=>v.repo===s.repo).files.find(v=>v.path===s.path);assert.deepEqual(e.ranges,[[1,f.lines]]);
 assert.equal(sha(resolve(workspace,'exact-real-references',s.repo,s.path)),f.sha256,s.path);newLines+=f.lines;
}
assert.equal(newLines,3286);assert.deepEqual(m.coverageAtBinding,[{repo:'calcium',reviewed:375,complete:374,partial:1,readLines:37085},
 {repo:'flint',reviewed:438,complete:426,partial:12,readLines:58440}]);
for(const s of effectiveSummary()){const old=m.coverageAtBinding.find(v=>v.repo===s.repo);assert(s.complete>=old.complete&&s.readLines>=old.readLines);}
for(const[p,ranges]of Object.entries(m.hyperReadRanges)) {
 const f='derivative-demand-candidate/'+p,lines=read(f).trimEnd().split('\n').length;assert.equal(sha(f),m.candidateSources.candidate[p],p);
 assert(ranges.every(([a,b],i)=>a>=1&&b>=a&&b<=lines&&(!i||a>ranges[i-1][1])));
}
assert.equal(m.gates.length,12);assert.equal(new Set(m.gates.map(g=>g.tag)).size,12);
const gates=new Map(m.gates.map(({tag,code,signal})=>{const g=json('results/'+tag+'.json');
 assert.equal(g.tag,tag);assert.equal(g.code,code,tag);assert.equal(g.signal,signal);assert.equal(signal,null);
 assert(Date.parse(g.finished)>=Date.parse(g.started));return[tag,g];}));
const gate=t=>{const g=gates.get(t);assert(g,t);return g;};
const initial=gate('nfixed-output-check-initial');assert.equal(initial.command,'node');
assert.deepEqual(initial.args,['check-nfixed-controls.mjs','--nfixed-controls-summary']);
assert.match(read('results/nfixed-output-check-initial.stderr'),/AssertionError \[ERR_ASSERTION\]/);
assert.equal(read('results/nfixed-output-check-initial.stdout'),'');
assert(read('check-nfixed-controls-initial.mjs').includes("assert.equal(native,read('results/nfixed-controls-memcheck.stdout'))"));
const fcompile=gate('nfixed-fenv-compile');assert.equal(fcompile.command,'gcc');assert.equal(fcompile.cwd,resolve(workspace,'exactcore-hyper-comparison'));
assert.deepEqual(fcompile.args,['-O2','-g0','-frounding-math','-ffp-contract=off','-Wall','-Wextra','-Werror',
 'audits/continuation/calcium/fenv-product-control.c','-lmpfr','-lgmp','-lm','-o',m.fenvBinary.path]);
assert.equal(read('results/nfixed-fenv-compile.stdout'),'');assert.equal(read('results/nfixed-fenv-compile.stderr'),'');
const frun=gate('nfixed-fenv-native');assert.equal(frun.command,m.fenvBinary.path);assert.deepEqual(frun.args,[]);
assert.equal(read('results/nfixed-fenv-native.stderr'),'');
const fm=gate('nfixed-fenv-memcheck');assert.equal(fm.command,'valgrind');assert.equal(fm.code,1);
assert.deepEqual(fm.args,['--vgdb=no','--tool=memcheck','--error-exitcode=97','--leak-check=full','--show-leak-kinds=all',
 '--errors-for-leak-kinds=definite,indirect,possible',m.fenvBinary.path]);
for(const tag of ['nfixed-controls-compile','nfixed-controls-compile-v2']) {
 const g=gate(tag);assert.equal(g.command,'gcc');assert.equal(g.cwd,resolve(workspace,'exactcore-hyper-comparison'));
 assert.deepEqual(g.args,['-O2','-g0','-Wall','-Wextra','-Werror','-isystem','../exact-real-references/flint/src',
  'audits/continuation/calcium/flint-nfixed-controls.c','-L','../exact-real-references/flint',
  '-Wl,-rpath,'+resolve(workspace,'exact-real-references/flint'),'-lflint','-lmpfr','-lgmp','-lm','-lpthread','-o',m.binary.path]);
 assert.equal(read('results/'+tag+'.stdout'),'');
}
assert.match(read('results/nfixed-controls-compile.stderr'),/Werror=misleading-indentation/);assert.equal(read('results/nfixed-controls-compile-v2.stderr'),'');
assert.equal(read('flint-nfixed-controls-initial.c').replace('mpz_clear(ai[i]);for(slong','mpz_clear(ai[i]);\n    for(slong'),read('flint-nfixed-controls.c'));
const native=gate('nfixed-controls-native');assert.equal(native.command,m.binary.path);assert.deepEqual(native.args,[]);assert.equal(native.code,1);
assert.equal(read('results/nfixed-controls-native.stderr'),'');
const mem=gate('nfixed-controls-memcheck');assert.equal(mem.command,'valgrind');assert.equal(mem.code,1);
assert.deepEqual(mem.args,['--vgdb=no','--tool=memcheck','--error-exitcode=97','--leak-check=full','--show-leak-kinds=all',
 '--errors-for-leak-kinds=definite,indirect,possible',m.binary.path]);
const env=['CARGO_TARGET_DIR=/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse','CARGO_INCREMENTAL=0','CARGO_PROFILE_DEV_DEBUG=0','CARGO_BUILD_JOBS=2'];
for(const profile of ['debug','release']) {
 const g=gate('nfixed-hyper-filter-'+profile);assert.equal(g.command,'env');assert.equal(g.cwd,resolve(workspace,'hyperlimit'));
 assert.deepEqual(g.args,[...env,'cargo','test','--offline','--locked',...(profile==='release'?['--release']:[]),'--lib','signed_term_filter','--','--test-threads=1']);
}
const inv=gate('nfixed-inventory');assert.equal(inv.command,'node');assert.equal(inv.cwd,workspace);
assert.deepEqual(inv.args,['exactcore-hyper-comparison/audits/continuation/calcium/inventory.mjs','verify']);assert.equal(read('results/nfixed-inventory.stderr'),'');
assert.deepEqual(read('results/nfixed-inventory.stdout').trimEnd().split('\n').map(JSON.parse).map(s=>[s.repo,s.commit]),inventory.sources.map(s=>[s.repo,s.commit]));
const linked=gate('nfixed-controls-linked-libraries');assert.equal(linked.command,'ldd');assert.deepEqual(linked.args,[m.binary.path]);
assert.equal(read('results/nfixed-controls-linked-libraries.stderr'),'');
assert.deepEqual(Object.keys(m.libraries),[...read('results/nfixed-controls-linked-libraries.stdout').matchAll(/=> (\/[^ ]+) \(/g)].map(m=>m[1]));
assert(Date.parse(gate('nfixed-controls-compile').finished)<Date.parse(gate('nfixed-controls-compile-v2').started));
assert(Date.parse(gate('nfixed-controls-compile-v2').finished)<Date.parse(native.started));assert(Date.parse(native.finished)<Date.parse(mem.started));
console.log(JSON.stringify({checkpoint:'fixed-point kernels and bounds',newReadLines:newLines,completedFiles:13,
 coverageAtBinding:m.coverageAtBinding,controls:m.controls.summary,matrixGroups:m.controls.matrixGroups,
 memory:m.controls.memory,binaryBytes:m.binary.bytes+m.fenvBinary.bytes,sourceFiles:955,production:m.production,
 instrumentation:m.instrumentation,status:m.status,limits:m.limits,followup:m.followup}));
