import './snapshot-v43-verify-magnitude.mjs';
import {readFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json} from './sign-filter-mask-sources.mjs';
import {checkMagTranscendental} from './check-mag-transcendental.mjs';
const m=json('mag-transcendental-experiment.json'),previous=json('magnitude-experiment.json');
const here=resolve('.'),workspace=resolve('../../../..'),comparison=resolve('../../..'),read=p=>readFileSync(p,'utf8');
assert.equal(Object.keys(m.files).length,26);for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
assert.deepEqual(m.liveSources,previous.liveSources);assert.equal(Object.keys(m.liveSources).length,955);
if(process.argv.includes('--derivative-live'))for(const[p,h]of Object.entries(m.liveSources))assert.equal(sha(resolve(workspace,p)),h,p);
assert.deepEqual(m.libraries,previous.libraries);assert.deepEqual(m.configurationFiles,previous.configurationFiles);
for(const[p,h]of Object.entries({...m.libraries,...m.configurationFiles}))assert.equal(sha(p),h,p);
assert.equal(sha(m.binary.path),m.binary.sha256);assert.equal(statSync(m.binary.path).size,m.binary.bytes);assert.equal(m.binary.bytes,33256);
const coverage=[...json('coverage.json'),...json('coverage-extensions.json')],inv=json('inventory.json').sources.find(s=>s.repo==='flint');
assert.equal(m.readRecords.length,44);let readLines=0;
for(const r of m.readRecords){
 assert.equal(r.repo,'flint');assert(coverage.some(x=>JSON.stringify(x)===JSON.stringify(r)));
 const f=inv.files.find(f=>f.path===r.path);assert(f?.text);assert.equal(sha(resolve(workspace,'exact-real-references/flint',r.path)),f.sha256);
 assert.deepEqual(r.ranges,[[1,f.lines]]);readLines+=f.lines;
}
assert.equal(readLines,4660);assert.equal(m.newDonorLines,readLines);assert.equal(m.newFullFileReads,44);
assert.deepEqual(json('mag-transcendental-read-selection.json'),m.readRecords.map(r=>({repo:r.repo,path:r.path,newRanges:r.ranges})));
assert.deepEqual(m.coverageAtBinding,[{repo:'calcium',reviewed:375,complete:374,partial:1,readLines:37085},
 {repo:'flint',reviewed:610,complete:592,partial:18,readLines:88986}]);
const top=inv.files.filter(f=>/^src\/mag\/[^/]+\.c$/.test(f.path));assert.equal(top.length,52);assert.equal(m.completedTopLevelMagImplementations,52);
for(const f of top)assert(coverage.some(r=>r.repo==='flint'&&r.path===f.path&&JSON.stringify(r.ranges)===JSON.stringify([[1,f.lines]])),f.path);
for(const[p,ranges]of Object.entries(m.hyperReadRanges)){
 const content=read(resolve('derivative-demand-candidate',p)),n=content.split('\n').length-Number(content.endsWith('\n'));
 assert(m.liveSources[p]);assert.equal(sha(resolve('derivative-demand-candidate',p)),m.liveSources[p]);assert.deepEqual(ranges,[[1,n]]);
}
assert.deepEqual(m.checks,checkMagTranscendental());assert.equal(m.gates.length,6);
const gates=new Map(m.gates.map(({tag,code,signal})=>{
 const g=json('results/'+tag+'.json');assert.equal(g.tag,tag);assert.equal(g.code,0);assert.equal(code,0);
 assert.equal(signal,null);assert.equal(g.signal,null);assert(Date.parse(g.finished)>=Date.parse(g.started));return[tag,g];
}));
assert.equal(gates.size,6);const gate=t=>{const g=gates.get('mag-transcendental-'+t);assert(g,t);return g;};
const compile=gate('compile');assert.equal(compile.cwd,comparison);assert.equal(compile.command,'gcc');
assert.deepEqual(compile.args,['-O2','-g0','-Wall','-Wextra','-Werror','-isystem','../exact-real-references/flint/src',
 'audits/continuation/calcium/flint-mag-transcendental-controls.c','-L','../exact-real-references/flint',
 '-Wl,-rpath,'+resolve(workspace,'exact-real-references/flint'),'-lflint','-lmpfr','-lgmp','-lm','-lpthread','-o',m.binary.path]);
assert.equal(read('results/mag-transcendental-compile.stdout'),'');assert.equal(read('results/mag-transcendental-compile.stderr'),'');
const native=gate('native'),memory=gate('memcheck');assert.equal(native.cwd,comparison);assert.equal(native.command,m.binary.path);assert.deepEqual(native.args,[]);
assert.equal(memory.cwd,comparison);assert.equal(memory.command,'valgrind');
assert.deepEqual(memory.args,['--vgdb=no','--tool=memcheck','--error-exitcode=97','--leak-check=full',
 '--show-leak-kinds=all','--errors-for-leak-kinds=definite,indirect,possible',m.binary.path]);
assert(Date.parse(compile.finished)<Date.parse(native.started));assert(Date.parse(native.finished)<Date.parse(memory.started));
const linked=gate('linked-libraries');assert.equal(linked.cwd,comparison);assert.equal(linked.command,'ldd');assert.deepEqual(linked.args,[m.binary.path]);
assert.equal(read('results/mag-transcendental-linked-libraries.stderr'),'');
const paths=[...read('results/mag-transcendental-linked-libraries.stdout').matchAll(/=> (\/[^ ]+) \(/g)].map(x=>x[1]);assert.deepEqual(paths,Object.keys(m.libraries));
for(const[t,args]of [['output-check',['check-mag-transcendental.mjs','--mag-transcendental-summary']],['inventory',['inventory.mjs','verify']]]){
 const g=gate(t);assert.equal(g.cwd,here);assert.equal(g.command,'node');assert.deepEqual(g.args,args);assert.equal(read('results/mag-transcendental-'+t+'.stderr'),'');
}
assert.deepEqual(JSON.parse(read('results/mag-transcendental-output-check.stdout')),m.checks);
console.log(JSON.stringify({checkpoint:'Magnitude transcendental/root/exp-tail bounds',files:26,gates:6,liveFiles:955,
 fullReads:44,readLines,completeTopLevelMagImplementations:52,binaryBytes:m.binary.bytes,checks:m.checks,
 status:m.status,production:m.production,findings:m.findings,comparison:m.comparison,limits:m.limits,followup:m.followup}));
