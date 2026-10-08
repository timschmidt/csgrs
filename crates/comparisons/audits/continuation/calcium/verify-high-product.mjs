import './verify-sign-filter-mask.mjs';
import {readFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json} from './sign-filter-mask-sources.mjs';
import {checkHighProductControls} from './check-high-product-controls.mjs';
const m=json('high-product-experiment.json'),here=resolve('.'),workspace=resolve('../../../..'),comparison=resolve('../../..');
const read=p=>readFileSync(p,'utf8');
assert.equal(sha(m.initialBinding.path),m.initialBinding.sha256);
assert.deepEqual(json(m.initialBinding.path).hyperReadRanges['hyperreal/AGENTS.md'],[[1,18]]);
assert.deepEqual(m.hyperReadRanges['hyperreal/AGENTS.md'],[[1,14]]);
assert.equal(Object.keys(m.files).length,30);for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
assert.deepEqual(m.liveSources,json('sign-filter-mask-experiment.json').liveSources);assert.equal(Object.keys(m.liveSources).length,955);
if(process.argv.includes('--derivative-live'))for(const[p,h]of Object.entries(m.liveSources))assert.equal(sha(resolve(workspace,p)),h,p);
assert.deepEqual(m.libraries,json('nfloat-complex-experiment.json').libraries);
for(const[p,h]of Object.entries({...m.libraries,...m.configurationFiles}))assert.equal(sha(p),h,p);
assert.equal(Object.keys(m.configurationFiles).length,1);const config=read(Object.keys(m.configurationFiles)[0]);
assert.match(config,/#define FLINT_BITS 64/);assert.match(config,/#define FLINT_HAVE_ASSEMBLY_x86_64_adx 1/);
assert.match(config,/\/\* #undef FLINT_HAVE_FFT_SMALL \*\//);assert.match(config,/#define FLINT_WANT_ASSERT 1/);
assert.equal(sha(m.binary.path),m.binary.sha256);assert.equal(statSync(m.binary.path).size,m.binary.bytes);assert.equal(m.binary.bytes,22616);
const coverage=[...json('coverage.json'),...json('coverage-extensions.json')],inv=json('inventory.json').sources.find(s=>s.repo==='flint');
assert.equal(m.readRecords.length,9);let completeReads=0,readLines=0;
for(const r of m.readRecords){
 assert.equal(r.repo,'flint');assert(coverage.some(x=>JSON.stringify(x)===JSON.stringify(r)));
 const f=inv.files.find(f=>f.path===r.path);assert(f?.text);assert.equal(sha(resolve(workspace,'exact-real-references/flint',r.path)),f.sha256);
 const n=r.ranges.reduce((s,[a,b])=>{assert(a>=1&&b>=a&&b<=f.lines);return s+b-a+1;},0);readLines+=n;completeReads+=n===f.lines;
}
assert.equal(readLines,1708);assert.equal(completeReads,7);
assert.deepEqual(json('high-product-qualification-read-selection.json'),m.readRecords.map(r=>({repo:r.repo,path:r.path,newRanges:r.ranges})));
assert.deepEqual(m.coverageAtBinding,[{repo:'calcium',reviewed:375,complete:374,partial:1,readLines:37085},
 {repo:'flint',reviewed:465,complete:451,partial:14,readLines:65320}]);
for(const[p,ranges]of Object.entries(m.hyperReadRanges)){
 const content=read(resolve(workspace,p)),n=content.split('\n').length-Number(content.endsWith('\n'));
 assert(m.liveSources[p]);assert.equal(sha(resolve(workspace,p)),m.liveSources[p]);for(const[a,b]of ranges)assert(a>=1&&b>=a&&b<=n,p);
}
assert.deepEqual(m.checks,checkHighProductControls());assert.equal(m.gates.length,7);
const gates=new Map(m.gates.map(({tag,code,signal})=>{
 const g=json('results/'+tag+'.json');assert.equal(g.tag,tag);assert.equal(g.code,code);assert.equal(signal,null);assert.equal(g.signal,null);
 assert(Date.parse(g.finished)>=Date.parse(g.started));return[tag,g];
}));
assert.equal(gates.size,7);const gate=t=>{const g=gates.get('high-product-'+t);assert(g,t);return g;};
const compile=gate('compile');assert.equal(compile.cwd,comparison);assert.equal(compile.command,'gcc');assert.equal(compile.code,0);
assert.deepEqual(compile.args,['-O2','-g0','-Wall','-Wextra','-Werror','-isystem','../exact-real-references/flint/src',
 'audits/continuation/calcium/flint-high-product-controls.c','-L','../exact-real-references/flint',
 '-Wl,-rpath,'+resolve(workspace,'exact-real-references/flint'),'-lflint','-lmpfr','-lgmp','-lm','-lpthread','-o',m.binary.path]);
assert.equal(read('results/high-product-compile.stdout'),'');assert.equal(read('results/high-product-compile.stderr'),'');
const native=gate('native'),memory=gate('memcheck');
assert.equal(native.cwd,comparison);assert.equal(native.command,m.binary.path);assert.deepEqual(native.args,[]);assert.equal(native.code,1);
assert.equal(memory.cwd,comparison);assert.equal(memory.command,'valgrind');assert.equal(memory.code,1);
assert.deepEqual(memory.args,['--vgdb=no','--tool=memcheck','--error-exitcode=97','--leak-check=full',
 '--show-leak-kinds=all','--errors-for-leak-kinds=definite,indirect,possible',m.binary.path]);
assert(Date.parse(compile.finished)<Date.parse(native.started));assert(Date.parse(native.finished)<Date.parse(memory.started));
const linked=gate('linked-libraries');assert.equal(linked.cwd,comparison);assert.equal(linked.command,'ldd');assert.deepEqual(linked.args,[m.binary.path]);
assert.equal(linked.code,0);assert.equal(read('results/high-product-linked-libraries.stderr'),'');
const paths=[...read('results/high-product-linked-libraries.stdout').matchAll(/=> (\/[^ ]+) \(/g)].map(x=>x[1]);assert.deepEqual(paths,Object.keys(m.libraries));
for(const[t,args]of [['bigint-oracle',['high-product-bigint-oracle.mjs','--high-product-bigint-summary']],
 ['output-check',['check-high-product-controls.mjs','--high-product-summary']],['inventory',['inventory.mjs','verify']]]){
 const g=gate(t);assert.equal(g.cwd,here);assert.equal(g.command,'node');assert.deepEqual(g.args,args);assert.equal(g.code,0);
 assert.equal(read('results/high-product-'+t+'.stderr'),'');
}
assert.deepEqual(JSON.parse(read('results/high-product-output-check.stdout')),m.checks);
console.log(JSON.stringify({checkpoint:'independent high-product bounds and supporting source audit',files:30,liveFiles:955,
 completeReads,headerExtensions:2,readLines,binaryBytes:m.binary.bytes,checks:m.checks,
 status:m.status,production:m.production,limits:m.limits,finding:m.finding,followup:m.followup}));
