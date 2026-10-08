import './snapshot-v43-verify-mag-transcendental.mjs';
import {readFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json} from './sign-filter-mask-sources.mjs';
import {checkMagSeries} from './check-mag-series.mjs';
const m=json('mag-series-experiment.json'),previous=json('mag-transcendental-experiment.json');
const here=resolve('.'),workspace=resolve('../../../..'),comparison=resolve('../../..'),read=p=>readFileSync(p,'utf8');
assert.equal(Object.keys(m.files).length,26);for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
assert.deepEqual(m.liveSources,previous.liveSources);assert.equal(Object.keys(m.liveSources).length,955);
if(process.argv.includes('--derivative-live'))for(const[p,h]of Object.entries(m.liveSources))assert.equal(sha(resolve(workspace,p)),h,p);
assert.deepEqual(m.libraries,previous.libraries);assert.deepEqual(m.configurationFiles,previous.configurationFiles);
for(const[p,h]of Object.entries({...m.libraries,...m.configurationFiles}))assert.equal(sha(p),h,p);
assert.equal(sha(m.binary.path),m.binary.sha256);assert.equal(statSync(m.binary.path).size,m.binary.bytes);assert.equal(m.binary.bytes,38448);
const coverage=[...json('coverage.json'),...json('coverage-extensions.json')],inv=json('inventory.json').sources.find(s=>s.repo==='flint');
assert.equal(m.readRecords.length,30);let readLines=0,full=0;
for(const r of m.readRecords){
 assert.equal(r.repo,'flint');assert(coverage.some(x=>JSON.stringify(x)===JSON.stringify(r)));
 const f=inv.files.find(f=>f.path===r.path);assert(f?.text);assert.equal(sha(resolve(workspace,'exact-real-references/flint',r.path)),f.sha256);
 const n=r.ranges.reduce((s,[a,b])=>{assert(a>=1&&b>=a&&b<=f.lines);return s+b-a+1;},0);readLines+=n;full+=n===f.lines;
}
assert.equal(readLines,3478);assert.equal(m.newDonorLines,readLines);assert.equal(full,29);assert.equal(m.newFullFileReads,full);
assert.deepEqual(m.completedExistingPaths,['src/arf/get.c']);
assert.deepEqual(m.readRecords.find(r=>r.path==='src/arf/get.c').ranges,[[1,439],[531,604]]);
assert(coverage.some(r=>r.repo==='flint'&&r.path==='src/arf/get.c'&&JSON.stringify(r.ranges)==='[[440,530]]'));
assert.deepEqual(json('mag-series-read-selection.json'),m.readRecords.map(r=>({repo:r.repo,path:r.path,newRanges:r.ranges})));
assert.deepEqual(m.coverageAtBinding,[{repo:'calcium',reviewed:375,complete:374,partial:1,readLines:37085},
 {repo:'flint',reviewed:639,complete:622,partial:17,readLines:92464}]);
const mag=inv.files.filter(f=>f.path.startsWith('src/mag/'));assert.equal(mag.length,104);assert.equal(m.completedMagDirectoryFiles,104);
for(const f of mag)assert(coverage.some(r=>r.repo==='flint'&&r.path===f.path&&JSON.stringify(r.ranges)===JSON.stringify([[1,f.lines]])),f.path);
for(const path of m.reusedDonorFiles){assert(previous.readRecords.some(r=>r.path===path));const f=inv.files.find(f=>f.path===path);assert.equal(sha(resolve(workspace,'exact-real-references/flint',path)),f.sha256);}
for(const[p,ranges]of Object.entries(m.hyperReadRanges)){
 const content=read(resolve('derivative-demand-candidate',p)),n=content.split('\n').length-Number(content.endsWith('\n'));
 assert(m.liveSources[p]);assert.equal(sha(resolve('derivative-demand-candidate',p)),m.liveSources[p]);for(const[a,b]of ranges)assert(a>=1&&b>=a&&b<=n,p);
}
assert.deepEqual(m.checks,checkMagSeries());assert.equal(m.gates.length,6);
const gates=new Map(m.gates.map(({tag,code,signal})=>{
 const g=json('results/'+tag+'.json');assert.equal(g.tag,tag);assert.equal(g.code,0);assert.equal(code,0);
 assert.equal(signal,null);assert.equal(g.signal,null);assert(Date.parse(g.finished)>=Date.parse(g.started));return[tag,g];
}));
assert.equal(gates.size,6);const gate=t=>{const g=gates.get('mag-series-'+t);assert(g,t);return g;};
const compile=gate('compile');assert.equal(compile.cwd,comparison);assert.equal(compile.command,'gcc');
assert.deepEqual(compile.args,['-O2','-g0','-Wall','-Wextra','-Werror','-isystem','../exact-real-references/flint/src',
 'audits/continuation/calcium/flint-mag-series-controls.c','-L','../exact-real-references/flint',
 '-Wl,-rpath,'+resolve(workspace,'exact-real-references/flint'),'-lflint','-lmpfr','-lgmp','-lm','-lpthread','-o',m.binary.path]);
assert.equal(read('results/mag-series-compile.stdout'),'');assert.equal(read('results/mag-series-compile.stderr'),'');
const native=gate('native'),memory=gate('memcheck');assert.equal(native.cwd,comparison);assert.equal(native.command,m.binary.path);assert.deepEqual(native.args,[]);
assert.equal(memory.cwd,comparison);assert.equal(memory.command,'valgrind');
assert.deepEqual(memory.args,['--vgdb=no','--tool=memcheck','--error-exitcode=97','--leak-check=full',
 '--show-leak-kinds=all','--errors-for-leak-kinds=definite,indirect,possible',m.binary.path]);
assert(Date.parse(compile.finished)<Date.parse(native.started));assert(Date.parse(native.finished)<Date.parse(memory.started));
const linked=gate('linked-libraries');assert.equal(linked.cwd,comparison);assert.equal(linked.command,'ldd');assert.deepEqual(linked.args,[m.binary.path]);
assert.equal(read('results/mag-series-linked-libraries.stderr'),'');
const paths=[...read('results/mag-series-linked-libraries.stdout').matchAll(/=> (\/[^ ]+) \(/g)].map(x=>x[1]);assert.deepEqual(paths,Object.keys(m.libraries));
for(const[t,args]of [['output-check',['check-mag-series.mjs','--mag-series-summary']],['inventory',['inventory.mjs','verify']]]){
 const g=gate(t);assert.equal(g.cwd,here);assert.equal(g.command,'node');assert.deepEqual(g.args,args);assert.equal(read('results/mag-series-'+t+'.stderr'),'');
}
assert.deepEqual(JSON.parse(read('results/mag-series-output-check.stdout')),m.checks);
console.log(JSON.stringify({checkpoint:'Magnitude combinatorial/full-tail/conversion bounds',files:26,gates:6,liveFiles:955,
 fullReads:full,completedExistingFiles:1,readLines,completeMagDirectoryFiles:104,binaryBytes:m.binary.bytes,checks:m.checks,
 status:m.status,production:m.production,findings:m.findings,comparison:m.comparison,limits:m.limits,followup:m.followup}));
