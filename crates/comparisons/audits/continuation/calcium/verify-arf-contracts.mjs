import './verify-e-qualified-retained.mjs';
import {readFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json} from './e-plan-qualified-sources.mjs';
import {checkArfContracts} from './check-arf-contracts.mjs';
const m=json('arf-contract-experiment.json'),here=resolve('.'),workspace=resolve('../../../..'),comparison=resolve('../../..');
const read=p=>readFileSync(p,'utf8');assert.equal(Object.keys(m.files).length,27);
for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
assert.deepEqual(m.liveSources,json('e-qualified-experiment.json').liveSources);assert.equal(Object.keys(m.liveSources).length,956);
for(const[p,h]of Object.entries(m.liveSources))assert.equal(sha(resolve(workspace,p)),h,p);
assert.deepEqual(m.libraries,json('mag-series-experiment.json').libraries);
assert.deepEqual(m.configurationFiles,json('mag-series-experiment.json').configurationFiles);
for(const[p,h]of Object.entries({...m.libraries,...m.configurationFiles}))assert.equal(sha(p),h,p);
assert.equal(sha(m.binary.path),m.binary.sha256);assert.equal(statSync(m.binary.path).size,28392);assert.equal(m.binary.bytes,28392);
const cov=[...json('coverage.json'),...json('coverage-extensions.json')],inv=json('inventory.json').sources.find(s=>s.repo==='flint');
assert.deepEqual(m.readRecords,[...json('arf-contract-read-records.json'),...json('arf-contract-support-read-records.json')]);
assert.equal(m.readRecords.length,51);let lines=0;
for(const r of m.readRecords) {
 assert(cov.some(x=>JSON.stringify(x)===JSON.stringify(r)));const f=inv.files.find(f=>f.path===r.path);assert(f?.text);
 assert.equal(sha(resolve(workspace,'exact-real-references/flint',r.path)),f.sha256,r.path);
 for(const[a,b]of r.ranges){assert(a>=1&&b>=a&&b<=f.lines);lines+=b-a+1;}
}
assert.equal(lines,5849);assert.equal(m.newDonorLines,lines);assert.equal(m.arfReadLines+m.supportReadLines,lines);
const scope=inv.files.filter(f=>f.path.startsWith('src/arf/')||['src/arf.h','doc/source/arf.rst'].includes(f.path));
assert.equal(scope.length,104);assert.equal(m.completeArfScopeFiles,104);
for(const f of scope) {
 assert.equal(sha(resolve(workspace,'exact-real-references/flint',f.path)),f.sha256,f.path);
 const ranges=cov.filter(r=>r.repo==='flint'&&r.path===f.path).flatMap(r=>r.ranges);
 const seen=new Set(ranges.flatMap(([a,b])=>Array.from({length:b-a+1},(_,i)=>a+i)));assert.equal(seen.size,f.lines,f.path);
}
assert.deepEqual(m.coverageAtBinding,[{repo:'calcium',reviewed:375,complete:374,partial:1,readLines:37085},
 {repo:'flint',reviewed:706,complete:689,partial:17,readLines:99489}]);
for(const[p,ranges]of Object.entries(m.hyperReadRanges)) {
 const source=read(resolve(workspace,p)),n=source.split('\n').length-Number(source.endsWith('\n'));
 for(const[a,b]of ranges)assert(a>=1&&b>=a&&b<=n,p);
}
assert.deepEqual(m.checks,checkArfContracts());assert.deepEqual(json('results/arf-contract-output-check.stdout'),m.checks);
assert.equal(m.gates.length,5);const gates=new Map();
for(const g of m.gates){const r=json('results/'+g.tag+'.json');for(const k of ['tag','code','signal','cwd','command','args'])assert.deepEqual(r[k],g[k]);
 assert.equal(r.code,0);assert.equal(r.signal,null);assert(Date.parse(r.finished)>=Date.parse(r.started));gates.set(g.tag.replace('arf-contract-',''),r);}
const compile=gates.get('compile');assert.equal(compile.cwd,comparison);assert.equal(compile.command,'gcc');
assert.deepEqual(compile.args,['-O2','-g0','-Wall','-Wextra','-Werror','-isystem','../exact-real-references/flint/src',
 'audits/continuation/calcium/flint-arf-contract-controls.c','-L','../exact-real-references/flint',
 '-Wl,-rpath,'+resolve(workspace,'exact-real-references/flint'),'-lflint','-lmpfr','-lgmp','-lm','-lpthread','-o',m.binary.path]);
const native=gates.get('native'),mem=gates.get('memcheck'),linked=gates.get('linked-libraries'),out=gates.get('output-check');
assert.equal(native.cwd,comparison);assert.equal(native.command,m.binary.path);assert.deepEqual(native.args,[]);
assert.equal(mem.cwd,comparison);assert.equal(mem.command,'valgrind');assert.deepEqual(mem.args,['--vgdb=no','--tool=memcheck',
 '--error-exitcode=97','--leak-check=full','--show-leak-kinds=all','--errors-for-leak-kinds=definite,indirect,possible',m.binary.path]);
assert.equal(linked.cwd,comparison);assert.equal(linked.command,'ldd');assert.deepEqual(linked.args,[m.binary.path]);
assert.equal(out.cwd,here);assert.equal(out.command,'node');assert.deepEqual(out.args,['check-arf-contracts.mjs','--arf-contract-summary']);
assert(Date.parse(compile.finished)<Date.parse(native.started));assert(Date.parse(native.finished)<Date.parse(mem.started));
assert(Date.parse(mem.finished)<Date.parse(out.started));
console.log(JSON.stringify({checkpoint:'ARF contracts and test-source closure',files:27,gates:5,liveFiles:956,
 readRecords:51,newDonorLines:5849,completeArfScopeFiles:104,binaryBytes:28392,checks:m.checks,status:m.status,
 production:m.production,findings:m.findings,comparison:m.comparison,limits:m.limits,followup:m.followup}));
