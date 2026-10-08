import './verify-complex-product-v2.mjs';
import {readFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json} from './sign-filter-mask-sources.mjs';
import {checkArbDotControls} from './check-arb-dot-controls.mjs';
const m=json('arb-dot-experiment.json'),previous=json('complex-product-v2-experiment.json'),old=json('arf-fused-experiment.json');
const here=resolve('.'),workspace=resolve('../../../..'),comparison=resolve('../../..'),read=p=>readFileSync(p,'utf8');
assert.equal(Object.keys(m.files).length,27);for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
assert.deepEqual(m.liveSources,previous.liveSources);assert.equal(Object.keys(m.liveSources).length,955);
if(process.argv.includes('--derivative-live'))for(const[p,h]of Object.entries(m.liveSources))assert.equal(sha(resolve(workspace,p)),h,p);
assert.deepEqual(m.libraries,old.libraries);assert.deepEqual(m.configurationFiles,old.configurationFiles);
for(const[p,h]of Object.entries({...m.libraries,...m.configurationFiles}))assert.equal(sha(p),h,p);
assert.equal(sha(m.binary.path),m.binary.sha256);assert.equal(statSync(m.binary.path).size,m.binary.bytes);assert.equal(m.binary.bytes,27768);
const coverage=[...json('coverage.json'),...json('coverage-extensions.json')],inv=json('inventory.json').sources.find(s=>s.repo==='flint');
assert.equal(m.readRecords.length,23);let readLines=0,completeReads=0;
for(const r of m.readRecords){
 assert.equal(r.repo,'flint');assert(coverage.some(x=>JSON.stringify(x)===JSON.stringify(r)));
 const f=inv.files.find(f=>f.path===r.path);assert(f?.text);assert.equal(sha(resolve(workspace,'exact-real-references/flint',r.path)),f.sha256);
 const n=r.ranges.reduce((s,[a,b])=>{assert(a>=1&&b>=a&&b<=f.lines);return s+b-a+1;},0);readLines+=n;completeReads+=n===f.lines;
}
assert.equal(readLines,3327);assert.equal(m.newDonorLines,readLines);assert.equal(completeReads,20);
assert.deepEqual(json('arb-dot-read-selection.json'),m.readRecords.map(r=>({repo:r.repo,path:r.path,newRanges:r.ranges})));
assert.deepEqual(m.coverageAtBinding,[{repo:'calcium',reviewed:375,complete:374,partial:1,readLines:37085},
 {repo:'flint',reviewed:529,complete:511,partial:18,readLines:80348}]);
for(const[p,ranges]of Object.entries(m.hyperReadRanges)){
 const content=read(resolve(workspace,p)),n=content.split('\n').length-Number(content.endsWith('\n'));
 assert(m.liveSources[p]);assert.equal(sha(resolve(workspace,p)),m.liveSources[p]);for(const[a,b]of ranges)assert(a>=1&&b>=a&&b<=n,p);
}
assert.deepEqual(m.checks,checkArbDotControls());assert.equal(m.checks.identicalNumericalBytes,161471);
assert.deepEqual(m.checks.memory,{errors:0,contexts:0,suppressed:0,liveBytes:0,liveBlocks:0,allocations:687105,frees:687105,cumulativeBytes:727232808});
assert.equal(m.gates.length,6);
const gates=new Map(m.gates.map(({tag,code,signal})=>{
 const g=json('results/'+tag+'.json');assert.equal(g.tag,tag);assert.equal(g.code,0);assert.equal(code,0);assert.equal(signal,null);assert.equal(g.signal,null);
 assert(Date.parse(g.finished)>=Date.parse(g.started));return[tag,g];
}));
assert.equal(gates.size,6);const gate=t=>{const g=gates.get('arb-dot-'+t);assert(g,t);return g;};
const compile=gate('compile');assert.equal(compile.cwd,comparison);assert.equal(compile.command,'gcc');
assert.deepEqual(compile.args,['-O2','-g0','-Wall','-Wextra','-Werror','-isystem','../exact-real-references/flint/src',
 'audits/continuation/calcium/flint-arb-dot-controls.c','-L','../exact-real-references/flint',
 '-Wl,-rpath,'+resolve(workspace,'exact-real-references/flint'),'-lflint','-lmpfr','-lgmp','-lm','-lpthread','-o',m.binary.path]);
assert.equal(read('results/arb-dot-compile.stdout'),'');assert.equal(read('results/arb-dot-compile.stderr'),'');
const native=gate('native'),memory=gate('memcheck');assert.equal(native.cwd,comparison);assert.equal(native.command,m.binary.path);assert.deepEqual(native.args,[]);
assert.equal(memory.cwd,comparison);assert.equal(memory.command,'valgrind');
assert.deepEqual(memory.args,['--vgdb=no','--tool=memcheck','--error-exitcode=97','--leak-check=full',
 '--show-leak-kinds=all','--errors-for-leak-kinds=definite,indirect,possible',m.binary.path]);
assert(Date.parse(compile.finished)<Date.parse(native.started));assert(Date.parse(native.finished)<Date.parse(memory.started));
const linked=gate('linked-libraries');assert.equal(linked.cwd,comparison);assert.equal(linked.command,'ldd');assert.deepEqual(linked.args,[m.binary.path]);
assert.equal(read('results/arb-dot-linked-libraries.stderr'),'');
const paths=[...read('results/arb-dot-linked-libraries.stdout').matchAll(/=> (\/[^ ]+) \(/g)].map(x=>x[1]);assert.deepEqual(paths,Object.keys(m.libraries));
for(const[t,args]of [['output-check',['check-arb-dot-controls.mjs','--arb-dot-summary']],['inventory',['inventory.mjs','verify']]]){
 const g=gate(t);assert.equal(g.cwd,here);assert.equal(g.command,'node');assert.deepEqual(g.args,args);assert.equal(read('results/arb-dot-'+t+'.stderr'),'');
}
assert.deepEqual(JSON.parse(read('results/arb-dot-output-check.stdout')),m.checks);
console.log(JSON.stringify({checkpoint:'Arb dot/error-bound source audit and independent rational endpoint qualification',files:27,gates:6,
 liveFiles:955,completeReads,partialReads:3,readLines,binaryBytes:m.binary.bytes,checks:m.checks,
 status:m.status,production:m.production,corpus:m.corpus,findings:m.findings,comparison:m.comparison,limits:m.limits,followup:m.followup}));
