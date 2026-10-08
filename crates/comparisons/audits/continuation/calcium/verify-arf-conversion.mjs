import './verify-arf-contracts.mjs';
import {readFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json} from './e-plan-qualified-sources.mjs';
import {checkArfConversion} from './check-arf-conversion.mjs';
const m=json('arf-conversion-experiment.json'),here=resolve('.'),workspace=resolve('../../../..'),comparison=resolve('../../..');
assert.equal(Object.keys(m.files).length,38);
for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
const previous=json('arf-contract-experiment.json');
assert.deepEqual(m.liveSources,previous.liveSources);assert.equal(Object.keys(m.liveSources).length,956);
for(const[p,h]of Object.entries(m.liveSources))assert.equal(sha(resolve(workspace,p)),h,p);
assert.deepEqual(m.libraries,previous.libraries);assert.deepEqual(m.configurationFiles,previous.configurationFiles);
for(const[p,h]of Object.entries({...m.libraries,...m.configurationFiles}))assert.equal(sha(p),h,p);
assert.equal(m.binaries.length,2);
for(const b of m.binaries){assert.equal(sha(b.path),b.sha256);assert.equal(statSync(b.path).size,b.bytes);}
assert.deepEqual(m.binaries.map(b=>b.bytes),[28664,28672]);
assert.deepEqual(m.binaries.map(b=>b.path),['controls','controls-v2'].map(s=>'/tmp/calcium-arf-conversion.ZnmLMf/'+s));
const inv=json('inventory.json').sources.find(s=>s.repo==='flint'),coverage=[...json('coverage.json'),...json('coverage-extensions.json')];
assert.deepEqual(m.readRecords,json('arf-conversion-read-records.json'));assert.equal(m.readRecords.length,1);
const r=m.readRecords[0],f=inv.files.find(f=>f.path===r.path);
assert.equal(r.repo,'flint');assert.equal(r.path,'src/fmpz/set.c');assert.equal(f.lines,238);
assert.deepEqual(r.ranges,[[1,238]]);assert.equal(m.newDonorLines,238);assert.equal(m.newCompleteFiles,1);
assert(coverage.some(x=>JSON.stringify(x)===JSON.stringify(r)));
assert.equal(sha(resolve(workspace,'exact-real-references/flint',r.path)),f.sha256);
assert.deepEqual(m.coverageAtBinding,[{repo:'calcium',reviewed:375,complete:374,partial:1,readLines:37085},
 {repo:'flint',reviewed:707,complete:690,partial:17,readLines:99727}]);
for(const[p,ranges]of Object.entries(m.hyperReadRanges)) {
 assert(m.liveSources[p]);const s=readFileSync(resolve(workspace,p),'utf8'),n=s.split('\n').length-Number(s.endsWith('\n'));
 for(const[a,b]of ranges)assert(a>=1&&b>=a&&b<=n,p);
}
assert.deepEqual(m.checks,checkArfConversion());assert.deepEqual(m.checks,json('results/arf-conversion-output-check.stdout'));
assert.equal(m.gates.length,8);const gates=new Map();
for(const g of m.gates){const r=json('results/'+g.tag+'.json');
 for(const k of ['tag','code','signal','cwd','command','args'])assert.deepEqual(r[k],g[k]);
 assert.equal(r.code,g.tag==='arf-conversion-native'?1:0);assert.equal(r.signal,null);
 assert(Date.parse(r.finished)>=Date.parse(r.started));gates.set(g.tag,r);}
for(let v=0;v<2;v++) {
 const prefix=v?'arf-conversion-v2':'arf-conversion',source=v?'flint-arf-conversion-v2-controls.c':'flint-arf-conversion-controls.c';
 const compile=gates.get(prefix+'-compile'),native=gates.get(prefix+'-native'),linked=gates.get(prefix+'-linked-libraries'),b=m.binaries[v];
 assert.equal(compile.cwd,comparison);assert.equal(compile.command,'gcc');
 assert.deepEqual(compile.args,['-O2','-g0','-Wall','-Wextra','-Werror','-isystem','../exact-real-references/flint/src',
  'audits/continuation/calcium/'+source,'-L','../exact-real-references/flint',
  '-Wl,-rpath,'+resolve(workspace,'exact-real-references/flint'),'-lflint','-lmpfr','-lgmp','-lm','-lpthread','-o',b.path]);
 assert.equal(native.cwd,comparison);assert.equal(native.command,b.path);assert.deepEqual(native.args,[]);
 assert.equal(linked.cwd,comparison);assert.equal(linked.command,'ldd');assert.deepEqual(linked.args,[b.path]);
 const linkedText=readFileSync('results/'+prefix+'-linked-libraries.stdout','utf8');
 const libraries=Object.fromEntries([...linkedText.matchAll(/=> (\/[^ ]+) \(/g)].map(match=>[match[1],sha(match[1])]));
 assert.deepEqual(libraries,m.libraries);assert(Date.parse(compile.finished)<Date.parse(native.started));
}
const mem=gates.get('arf-conversion-v2-memcheck'),output=gates.get('arf-conversion-output-check');
assert.equal(mem.cwd,comparison);assert.equal(mem.command,'valgrind');
assert.deepEqual(mem.args,['--vgdb=no','--tool=memcheck','--error-exitcode=97','--leak-check=full','--show-leak-kinds=all',
 '--errors-for-leak-kinds=definite,indirect,possible',m.binaries[1].path]);
assert.equal(output.cwd,here);assert.equal(output.command,'node');assert.deepEqual(output.args,['check-arf-conversion.mjs','--arf-conversion-summary']);
assert(Date.parse(gates.get('arf-conversion-native').finished)<Date.parse(gates.get('arf-conversion-v2-compile').started));
assert(Date.parse(gates.get('arf-conversion-v2-native').finished)<Date.parse(mem.started));
assert(Date.parse(mem.finished)<Date.parse(output.started));
assert.equal(json('results/arf-contract-verify-full.json').code,0);
console.log(JSON.stringify({checkpoint:'Finite ARF conversion and integer contracts',files:38,gates:8,
 successfulGates:7,preservedFailedGates:1,liveFiles:956,newDonorLines:238,newCompleteFiles:1,binaryBytes:57336,
 checks:m.checks,status:m.status,production:m.production,findings:m.findings,preservedFailure:m.preservedFailure,
 comparison:m.comparison,limits:m.limits,followup:m.followup}));
