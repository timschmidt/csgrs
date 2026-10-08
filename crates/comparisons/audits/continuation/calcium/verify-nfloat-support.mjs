import './verify-derivative-qualification.mjs';
import {readFileSync,statSync} from 'node:fs';
import {dirname,resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
import {sources,sha,json} from './derivative-demand-sources.mjs';
import {effectiveCoverage,effectiveSummary} from './effective-coverage.mjs';
import {checkControls} from './check-nfloat-controls.mjs';
const here=dirname(fileURLToPath(import.meta.url)),workspace=resolve(here,'../../../..');
const read=p=>readFileSync(resolve(here,p),'utf8'),m=json('nfloat-support-experiment.json');
for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
assert.equal(Object.keys(m.files).length,39);
assert.deepEqual(m.candidateSources,sources());
assert.equal(Object.keys(m.candidateSources.candidate).length,955);
if(process.argv.includes('--derivative-live'))for(const[p,h]of Object.entries(m.candidateSources.candidate))
 assert.equal(sha(resolve(workspace,p)),h,'live '+p);
assert.equal(sha(m.binary.path),m.binary.sha256);assert.equal(statSync(m.binary.path).size,m.binary.bytes);
assert.equal(m.binary.bytes,27928);
for(const[p,h]of Object.entries(m.libraries))assert.equal(sha(p),h,p);
assert.deepEqual(m.nativeLibrary,json('spectral-experiment.json').nativeLibrary);
assert.equal(sha(m.nativeLibrary.path),m.nativeLibrary.sha256);
assert.deepEqual(m.controls,checkControls());
const selection=json('nfloat-support-read-selection.json'),coverage=json('coverage.json'),extensions=json('coverage-extensions.json');
const effective=effectiveCoverage(),inventory=json('inventory.json');
assert.equal(selection.length,7);assert.equal(m.reads.length,7);
let newLines=0,completed=0;
for(const [i,s]of selection.entries()) {
 const e=(s.extension?extensions:coverage).find(e=>e.repo===s.repo&&e.path===s.path);
 assert.deepEqual(e,m.reads[i]);assert.deepEqual(e.ranges,s.newRanges);
 const f=inventory.sources.find(v=>v.repo===s.repo).files.find(v=>v.path===s.path);
 assert.equal(sha(resolve(workspace,'exact-real-references',s.repo,s.path)),f.sha256,s.path);
 newLines+=e.ranges.reduce((n,[a,b])=>n+b-a+1,0);
 const total=effective.find(e=>e.repo===s.repo&&e.path===s.path);
 completed+=+(total.ranges.length===1&&total.ranges[0][0]===1&&total.ranges[0][1]===f.lines);
}
assert.equal(newLines,7568);assert.equal(completed,6);
assert.deepEqual(m.coverageAtBinding,[{repo:'calcium',reviewed:375,complete:374,partial:1,readLines:37085},
 {repo:'flint',reviewed:425,complete:413,partial:12,readLines:55154}]);
for(const s of effectiveSummary()){const old=m.coverageAtBinding.find(v=>v.repo===s.repo);assert(s.complete>=old.complete&&s.readLines>=old.readLines);}
for(const[p,ranges]of Object.entries(m.hyperReadRanges)) {
 const file='derivative-demand-candidate/'+p,lines=read(file).trimEnd().split('\n').length;
 assert.equal(sha(file),m.candidateSources.candidate[p],p);
 assert(ranges.every(([a,b],i)=>a>=1&&b>=a&&b<=lines&&(!i||a>ranges[i-1][1])));
}
assert.equal(m.gates.length,9);assert.equal(new Set(m.gates.map(g=>g.tag)).size,9);
const gates=new Map(m.gates.map(({tag,code,signal})=>{
 const g=json('results/'+tag+'.json');assert.equal(g.tag,tag);assert.equal(g.code,code,tag);assert.equal(g.signal,signal);assert.equal(signal,null);
 assert(Date.parse(g.finished)>=Date.parse(g.started));return[tag,g];
}));
const gate=t=>{const g=gates.get(t);assert(g,t);return g;};
for(const tag of ['nfloat-controls-compile','nfloat-controls-compile-v2']) {
 const g=gate(tag);assert.equal(g.command,'gcc');assert.equal(g.cwd,resolve(workspace,'exactcore-hyper-comparison'));
 assert.deepEqual(g.args,['-O2','-g0','-Wall','-Wextra','-Werror','-isystem','../exact-real-references/flint/src',
  'audits/continuation/calcium/flint-nfloat-controls.c','-L','../exact-real-references/flint',
  '-Wl,-rpath,'+resolve(workspace,'exact-real-references/flint'),'-lflint','-lmpfr','-lgmp','-lm','-lpthread','-o',m.binary.path]);
 assert.equal(read('results/'+tag+'.stdout'),'');
}
assert.match(read('results/nfloat-controls-compile.stderr'),/implicit declaration of function .nfloat_set_arf/);
assert.equal(read('results/nfloat-controls-compile-v2.stderr'),'');
let corrected=read('flint-nfloat-controls-initial.c').replace('#include "gr_mat.h"\n','#include "gr_mat.h"\n#include "arf.h"\n')
 .replace('#include "fmpq.h"\n#include "arf.h"','#include "fmpq.h"')
 .replace('>= -20000 && NFLOAT_EXP(x) <= 20000','>= -200000 && NFLOAT_EXP(x) <= 200000')
 .replace('bits == 4224 ?','limbs == NFLOAT_MAX_LIMBS ?');
assert.equal(corrected,read('flint-nfloat-controls.c'));
const native=gate('nfloat-controls-native');assert.equal(native.command,m.binary.path);assert.deepEqual(native.args,[]);
assert.equal(read('results/nfloat-controls-native.stderr'),'');
const memory=gate('nfloat-controls-memcheck');assert.equal(memory.command,'valgrind');
assert.deepEqual(memory.args,['--vgdb=no','--tool=memcheck','--error-exitcode=97','--leak-check=full',
 '--show-leak-kinds=all','--errors-for-leak-kinds=definite,indirect,possible',m.binary.path]);
for(const tag of ['nfloat-support-inventory','nfloat-support-inventory-v2']) {
 const g=gate(tag);assert.equal(g.command,'node');assert.equal(g.cwd,workspace);
 assert.deepEqual(g.args,['exactcore-hyper-comparison/audits/continuation/calcium/inventory.mjs','verify']);
}
assert.match(read('results/nfloat-support-inventory.stderr'),/spawnSync git EPERM/);
assert.equal(read('results/nfloat-support-inventory-v2.stderr'),'');
const inv=read('results/nfloat-support-inventory-v2.stdout').trimEnd().split('\n').map(JSON.parse);assert.equal(inv.length,2);
assert.deepEqual(inv.map(s=>[s.repo,s.commit]),inventory.sources.map(s=>[s.repo,s.commit]));
const env=['CARGO_TARGET_DIR=/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse','CARGO_INCREMENTAL=0','CARGO_PROFILE_DEV_DEBUG=0','CARGO_BUILD_JOBS=2'];
for(const profile of ['debug','release']) {
 const g=gate('nfloat-hyper-dyadic-'+profile);assert.equal(g.command,'env');assert.equal(g.cwd,resolve(workspace,'hyperreal'));
 assert.deepEqual(g.args,[...env,'cargo','test','--offline','--locked',...(profile==='release'?['--release']:[]),'--lib','dyadic_dot_','--','--test-threads=1']);
}
const linked=gate('nfloat-controls-linked-libraries');assert.equal(linked.command,'ldd');assert.deepEqual(linked.args,[m.binary.path]);
assert.equal(read('results/nfloat-controls-linked-libraries.stderr'),'');
assert.deepEqual(Object.keys(m.libraries),[...read('results/nfloat-controls-linked-libraries.stdout').matchAll(/=> (\/[^ ]+) \(/g)].map(m=>m[1]));
assert(Date.parse(gate('nfloat-controls-compile').finished)<Date.parse(gate('nfloat-controls-compile-v2').started));
assert(Date.parse(gate('nfloat-controls-compile-v2').finished)<Date.parse(native.started));
assert(Date.parse(native.finished)<Date.parse(memory.started));
console.log(JSON.stringify({checkpoint:'nfloat arithmetic/conversion support',newReadLines:newLines,completedFiles:completed,
 coverageAtBinding:m.coverageAtBinding,controls:m.controls,binaryBytes:m.binary.bytes,sourceFiles:955,
 production:m.production,status:m.status,limits:m.limits}));
