import './verify-mpoly-rational.mjs';
import {readFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json} from './e-plan-qualified-sources.mjs';
import {checkMpolyBridge} from './check-mpoly-bridge.mjs';
const m=json('mpoly-bridge-experiment.json'),workspace=resolve('../../../..'),comparison=resolve('../../..');
assert.equal(Object.keys(m.files).length,30);for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
const previous=json('mpoly-rational-experiment.json');assert.deepEqual(m.liveSources,previous.liveSources);
assert.equal(Object.keys(m.liveSources).length,956);
for(const[p,h]of Object.entries(m.liveSources))assert.equal(sha(resolve(workspace,p)),h,p);
assert.deepEqual(m.libraries,previous.libraries);assert.deepEqual(m.configurationFiles,previous.configurationFiles);
for(const[p,h]of Object.entries({...m.libraries,...m.configurationFiles}))assert.equal(sha(p),h,p);
assert.equal(m.binaries.length,1);const binary=m.binaries[0];
assert.equal(binary.path,'/tmp/calcium-mpoly-bridge.7pQl1d/controls');assert.equal(binary.bytes,28488);
assert.equal(sha(binary.path),binary.sha256);assert.equal(statSync(binary.path).size,binary.bytes);
assert.deepEqual(m.readRecords,json('mpoly-bridge-read-records.json'));assert.equal(m.readRecords.length,56);
const inv=json('inventory.json'),coverage=[...json('coverage.json'),...json('coverage-extensions.json')];
let lines=0,complete=0;
for(const r of m.readRecords){assert(coverage.some(c=>JSON.stringify(c)===JSON.stringify(r)));
 const f=inv.sources.find(s=>s.repo===r.repo).files.find(f=>f.path===r.path);assert(f?.text);
 assert.equal(sha(resolve(workspace,'exact-real-references',r.repo,r.path)),f.sha256);
 let n=0;for(const[a,b]of r.ranges){assert(a>=1&&b>=a&&b<=f.lines);n+=b-a+1;}lines+=n;complete+=Number(n===f.lines);}
assert.equal(lines,5029);assert.equal(m.newDonorLines,lines);assert.equal(complete,54);
assert.equal(m.newCompleteFiles,complete);assert.equal(m.newPartialFiles,2);
const archived=inv.sources.find(s=>s.repo==='calcium').files.filter(f=>f.path.startsWith('fmpz_mpoly_q/')||f.path==='fmpz_mpoly_q.h'||f.path==='doc/source/fmpz_mpoly_q.rst');
assert.equal(archived.length,45);assert.equal(archived.reduce((n,f)=>n+f.lines,0),3765);
for(const f of archived)assert(coverage.some(r=>r.repo==='calcium'&&r.path===f.path&&JSON.stringify(r.ranges)===JSON.stringify([[1,f.lines]])));
assert.deepEqual(m.coverageAtBinding,[{repo:'calcium',reviewed:422,complete:421,partial:1,readLines:41035},
 {repo:'flint',reviewed:756,complete:735,partial:21,readLines:104761}]);
for(const[p,ranges]of Object.entries(m.hyperReadRanges)){assert(m.liveSources[p]);
 const s=readFileSync(resolve(workspace,p),'utf8'),n=s.split('\n').length-Number(s.endsWith('\n'));
 for(const[a,b]of ranges)assert(a>=1&&b>=a&&b<=n,p);}
assert.deepEqual(m.checks,checkMpolyBridge());assert.deepEqual(m.checks,json('results/mpoly-bridge-output-check.stdout'));
assert.equal(m.gates.length,5);const gates=new Map();
for(const g of m.gates){const r=json('results/'+g.tag+'.json');
 for(const k of['tag','code','signal','cwd','command','args'])assert.deepEqual(r[k],g[k]);
 assert.equal(r.code,0);assert.equal(r.signal,null);assert(Date.parse(r.finished)>=Date.parse(r.started));
 gates.set(g.tag.replace('mpoly-bridge-',''),r);}
assert.deepEqual([...gates.keys()],['compile','native','linked-libraries','memcheck','output-check']);
const compile=gates.get('compile');assert.equal(compile.cwd,comparison);assert.equal(compile.command,'gcc');
assert.deepEqual(compile.args,['-O2','-g0','-Wall','-Wextra','-Werror','-isystem','../exact-real-references/flint/src',
 'audits/continuation/calcium/flint-mpoly-bridge-controls.c','-L','../exact-real-references/flint',
 '-Wl,-rpath,'+resolve(workspace,'exact-real-references/flint'),'-lflint','-lmpfr','-lgmp','-lm','-lpthread','-o',binary.path]);
const native=gates.get('native'),mem=gates.get('memcheck'),linked=gates.get('linked-libraries'),check=gates.get('output-check');
assert.equal(native.cwd,comparison);assert.equal(native.command,binary.path);assert.deepEqual(native.args,[]);
assert.equal(mem.cwd,comparison);assert.equal(mem.command,'valgrind');assert.deepEqual(mem.args,['--vgdb=no','--tool=memcheck',
 '--error-exitcode=97','--leak-check=full','--show-leak-kinds=all','--errors-for-leak-kinds=definite,indirect,possible',binary.path]);
assert.equal(linked.cwd,comparison);assert.equal(linked.command,'ldd');assert.deepEqual(linked.args,[binary.path]);
assert.deepEqual(Object.fromEntries([...readFileSync('results/mpoly-bridge-linked-libraries.stdout','utf8').matchAll(/=> (\/[^ ]+) \(/g)].map(x=>[x[1],sha(x[1])])),m.libraries);
assert.equal(check.cwd,resolve('.'));assert.equal(check.command,'node');assert.deepEqual(check.args,['check-mpoly-bridge.mjs','--mpoly-bridge-summary']);
for(const[a,b]of[['compile','native'],['native','memcheck'],['memcheck','output-check']])
 assert(Date.parse(gates.get(a).finished)<Date.parse(gates.get(b).started));
assert.equal(json('results/mpoly-rational-verify-full.json').code,0);
console.log(JSON.stringify({checkpoint:'Archived rational functions and expression bridges',files:30,gates:5,liveFiles:956,
 readRecords:56,newDonorLines:5029,completeArchivedSliceFiles:45,binaryBytes:28488,checks:m.checks,
 status:m.status,production:m.production,findings:m.findings,comparison:m.comparison,limits:m.limits,followup:m.followup}));
