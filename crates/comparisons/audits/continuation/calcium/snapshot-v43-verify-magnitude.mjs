import './snapshot-v43-verify-arb-dot.mjs';
import {readFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json} from './sign-filter-mask-sources.mjs';
import {checkMagnitudeControls} from './check-magnitude-controls.mjs';
const m=json('magnitude-experiment.json'),previous=json('arb-dot-experiment.json');
const here=resolve('.'),workspace=resolve('../../../..'),comparison=resolve('../../..'),read=p=>readFileSync(p,'utf8');
assert.equal(Object.keys(m.files).length,35);for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
assert.equal(m.files['flint-arb-dot-controls.c'],previous.files['flint-arb-dot-controls.c']);
const code=read('flint-magnitude-controls.c'),initial=read('flint-magnitude-controls-initial.c');
assert.match(code,/#define main arb_dot_checkpoint38_main\n#include "flint-arb-dot-controls.c"\n#undef main/);
assert.equal(code.replace(/\s/g,''),initial.replace(/\s/g,''));
assert.equal(read('record-magnitude-reads-initial.mjs').replace('assert.equal(lines,4078)','assert.equal(lines,3978)'),read('record-magnitude-reads.mjs'));
assert.deepEqual(m.liveSources,previous.liveSources);assert.equal(Object.keys(m.liveSources).length,955);
if(process.argv.includes('--derivative-live'))for(const[p,h]of Object.entries(m.liveSources))assert.equal(sha(resolve(workspace,p)),h,p);
assert.deepEqual(m.libraries,previous.libraries);assert.deepEqual(m.configurationFiles,previous.configurationFiles);
for(const[p,h]of Object.entries({...m.libraries,...m.configurationFiles}))assert.equal(sha(p),h,p);
assert.equal(sha(m.binary.path),m.binary.sha256);assert.equal(statSync(m.binary.path).size,m.binary.bytes);assert.equal(m.binary.bytes,47216);
const coverage=[...json('coverage.json'),...json('coverage-extensions.json')],inv=json('inventory.json').sources.find(s=>s.repo==='flint');
assert.equal(m.readRecords.length,40);let readLines=0,fullReads=0;
for(const r of m.readRecords){
 assert.equal(r.repo,'flint');assert(coverage.some(x=>JSON.stringify(x)===JSON.stringify(r)));
 const f=inv.files.find(f=>f.path===r.path);assert(f?.text);assert.equal(sha(resolve(workspace,'exact-real-references/flint',r.path)),f.sha256);
 const n=r.ranges.reduce((s,[a,b])=>{assert(a>=1&&b>=a&&b<=f.lines);return s+b-a+1;},0);readLines+=n;fullReads+=n===f.lines;
}
assert.equal(readLines,3978);assert.equal(m.newDonorLines,readLines);assert.equal(fullReads,36);assert.equal(m.newFullFileReads,fullReads);
assert.deepEqual(m.completedExistingPaths,['src/mag.h']);
const magOld=previous.readRecords.find(r=>r.path==='src/mag.h'),magNew=m.readRecords.find(r=>r.path==='src/mag.h');
assert.deepEqual(magOld.ranges,[[95,145]]);assert.deepEqual(magNew.ranges,[[1,94],[146,698]]);
for(const path of m.reusedDonorFiles){assert(previous.readRecords.some(r=>r.path===path));const f=inv.files.find(f=>f.path===path);assert.equal(sha(resolve(workspace,'exact-real-references/flint',path)),f.sha256);}
assert.deepEqual(json('magnitude-read-selection.json'),m.readRecords.map(r=>({repo:r.repo,path:r.path,newRanges:r.ranges})));
assert.deepEqual(m.coverageAtBinding,[{repo:'calcium',reviewed:375,complete:374,partial:1,readLines:37085},
 {repo:'flint',reviewed:566,complete:548,partial:18,readLines:84326}]);
for(const[p,ranges]of Object.entries(m.hyperReadRanges)){
 const content=read(resolve('derivative-demand-candidate',p)),n=content.split('\n').length-Number(content.endsWith('\n'));
 assert(m.liveSources[p]);assert.equal(sha(resolve('derivative-demand-candidate',p)),m.liveSources[p]);for(const[a,b]of ranges)assert(a>=1&&b>=a&&b<=n,p);
}
assert.deepEqual(m.checks,checkMagnitudeControls());assert.equal(m.gates.length,8);
const failures=['magnitude-compile','magnitude-read-count-initial'];
const gates=new Map(m.gates.map(({tag,code,signal})=>{
 const g=json('results/'+tag+'.json');assert.equal(g.tag,tag);assert.equal(g.code,failures.includes(tag)?1:0);
 assert.equal(code,g.code);assert.equal(signal,null);assert.equal(g.signal,null);assert(Date.parse(g.finished)>=Date.parse(g.started));return[tag,g];
}));
assert.equal(gates.size,8);const gate=t=>{const g=gates.get('magnitude-'+t);assert(g,t);return g;};
const compile=gate('compile-formatted'),first=gate('compile');assert.equal(compile.cwd,comparison);assert.equal(compile.command,'gcc');
assert.equal(first.cwd,compile.cwd);assert.equal(first.command,compile.command);assert.deepEqual(first.args,compile.args);
assert.deepEqual(compile.args,['-O2','-g0','-Wall','-Wextra','-Werror','-isystem','../exact-real-references/flint/src',
 'audits/continuation/calcium/flint-magnitude-controls.c','-L','../exact-real-references/flint',
 '-Wl,-rpath,'+resolve(workspace,'exact-real-references/flint'),'-lflint','-lmpfr','-lgmp','-lm','-lpthread','-o',m.binary.path]);
assert.equal(read('results/magnitude-compile.stdout'),'');assert.match(read('results/magnitude-compile.stderr'),/-Werror=misleading-indentation/);
assert.equal(read('results/magnitude-compile-formatted.stdout'),'');assert.equal(read('results/magnitude-compile-formatted.stderr'),'');
const countFailure=gate('read-count-initial');assert.equal(countFailure.cwd,here);assert.equal(countFailure.command,'node');
assert.deepEqual(countFailure.args,['record-magnitude-reads-initial.mjs']);assert.equal(read('results/magnitude-read-count-initial.stdout'),'');
assert.match(read('results/magnitude-read-count-initial.stderr'),/3978 !== 4078/);
const native=gate('native'),memory=gate('memcheck');assert.equal(native.cwd,comparison);assert.equal(native.command,m.binary.path);assert.deepEqual(native.args,[]);
assert.equal(memory.cwd,comparison);assert.equal(memory.command,'valgrind');
assert.deepEqual(memory.args,['--vgdb=no','--tool=memcheck','--error-exitcode=97','--leak-check=full',
 '--show-leak-kinds=all','--errors-for-leak-kinds=definite,indirect,possible',m.binary.path]);
assert(Date.parse(first.finished)<Date.parse(compile.started));assert(Date.parse(compile.finished)<Date.parse(native.started));assert(Date.parse(native.finished)<Date.parse(memory.started));
const linked=gate('linked-libraries');assert.equal(linked.cwd,comparison);assert.equal(linked.command,'ldd');assert.deepEqual(linked.args,[m.binary.path]);
assert.equal(read('results/magnitude-linked-libraries.stderr'),'');
const paths=[...read('results/magnitude-linked-libraries.stdout').matchAll(/=> (\/[^ ]+) \(/g)].map(x=>x[1]);assert.deepEqual(paths,Object.keys(m.libraries));
for(const[t,args]of [['output-check',['check-magnitude-controls.mjs','--magnitude-summary']],['inventory',['inventory.mjs','verify']]]){
 const g=gate(t);assert.equal(g.cwd,here);assert.equal(g.command,'node');assert.deepEqual(g.args,args);assert.equal(read('results/magnitude-'+t+'.stderr'),'');
}
assert.deepEqual(JSON.parse(read('results/magnitude-output-check.stdout')),m.checks);
console.log(JSON.stringify({checkpoint:'Magnitude bounds, comparison and Arb error inflation',files:35,gates:8,successfulGates:6,preservedFailedGates:2,
 liveFiles:955,fullReads,completedExistingFiles:1,partialReadRecords:4,readLines,binaryBytes:m.binary.bytes,checks:m.checks,
 status:m.status,production:m.production,corpus:m.corpus,findings:m.findings,comparison:m.comparison,corrections:m.corrections,limits:m.limits,followup:m.followup}));
