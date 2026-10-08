import './verify-arf-conversion.mjs';
import {readFileSync,statSync} from 'node:fs';
import {resolve} from 'node:path';
import assert from 'node:assert/strict';
import {sha,json} from './e-plan-qualified-sources.mjs';
import {checkMpolyRational} from './check-mpoly-rational.mjs';
import {checkMpolyRationalUpstream} from './check-mpoly-rational-upstream.mjs';
const m=json('mpoly-rational-experiment.json'),here=resolve('.'),workspace=resolve('../../../..'),comparison=resolve('../../..');
assert.equal(Object.keys(m.files).length,46);for(const[p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
const previous=json('arf-conversion-experiment.json');assert.deepEqual(m.liveSources,previous.liveSources);assert.equal(Object.keys(m.liveSources).length,956);
for(const[p,h]of Object.entries(m.liveSources))assert.equal(sha(resolve(workspace,p)),h,p);
assert.deepEqual(m.libraries,previous.libraries);assert.deepEqual(m.configurationFiles,previous.configurationFiles);
for(const[p,h]of Object.entries({...m.libraries,...m.configurationFiles}))assert.equal(sha(p),h,p);
assert.deepEqual(m.binaries.map(b=>b.bytes),[24280,36560]);
assert.deepEqual(m.binaries.map(b=>b.path),['controls','upstream'].map(s=>'/tmp/calcium-mpoly-rational.Hmk8Is/'+s));
for(const b of m.binaries){assert.equal(sha(b.path),b.sha256);assert.equal(statSync(b.path).size,b.bytes);}
assert.deepEqual(m.readRecords,[...json('mpoly-rational-read-records.json'),...json('mpoly-rational-support-read-records.json')]);
assert.equal(m.readRecords.length,40);const inv=json('inventory.json'),coverage=[...json('coverage.json'),...json('coverage-extensions.json')];
let lines=0,complete=0;
for(const r of m.readRecords){assert(coverage.some(c=>JSON.stringify(c)===JSON.stringify(r)));
 const f=inv.sources.find(s=>s.repo===r.repo).files.find(f=>f.path===r.path);assert(f?.text);
 assert.equal(sha(resolve(workspace,'exact-real-references',r.repo,r.path)),f.sha256);
 let n=0;for(const[a,b]of r.ranges){assert(a>=1&&b>=a&&b<=f.lines);n+=b-a+1;}lines+=n;complete+=Number(n===f.lines);}
assert.equal(lines,3955);assert.equal(m.newDonorLines,lines);assert.equal(complete,38);assert.equal(m.newCompleteFiles,complete);assert.equal(m.newPartialFiles,2);
const scope=inv.sources.find(s=>s.repo==='flint').files.filter(f=>f.path.startsWith('src/fmpz_mpoly_q/')||['src/fmpz_mpoly_q.h','doc/source/fmpz_mpoly_q.rst'].includes(f.path));
assert.equal(scope.length,36);assert.equal(m.completeCurrentSliceFiles,36);assert.equal(scope.reduce((n,f)=>n+f.lines,0),3564);
assert.equal(m.completeCurrentSliceLines,3564);for(const f of scope)assert(m.readRecords.some(r=>r.repo==='flint'&&r.path===f.path&&JSON.stringify(r.ranges)===JSON.stringify([[1,f.lines]])));
assert.deepEqual(m.coverageAtBinding,[{repo:'calcium',reviewed:376,complete:375,partial:1,readLines:37286},
 {repo:'flint',reviewed:746,complete:727,partial:19,readLines:103481}]);
for(const[p,ranges]of Object.entries(m.hyperReadRanges)){assert(m.liveSources[p]);const s=readFileSync(resolve(workspace,p),'utf8'),n=s.split('\n').length-Number(s.endsWith('\n'));
 for(const[a,b]of ranges)assert(a>=1&&b>=a&&b<=n,p);}
assert.deepEqual(m.checks,checkMpolyRational());assert.deepEqual(m.upstreamChecks,checkMpolyRationalUpstream());
assert.deepEqual(m.checks,json('results/mpoly-rational-output-check.stdout'));assert.deepEqual(m.upstreamChecks,json('results/mpoly-rational-upstream-check.stdout'));
assert.equal(m.gates.length,10);const gates=new Map();for(const g of m.gates){const r=json('results/'+g.tag+'.json');
 for(const k of ['tag','code','signal','cwd','command','args'])assert.deepEqual(r[k],g[k]);assert.equal(r.code,g.tag==='mpoly-rational-compile'?1:0);assert.equal(r.signal,null);
 assert(Date.parse(r.finished)>=Date.parse(r.started));gates.set(g.tag.replace('mpoly-rational-',''),r);}
const compileArgs=(source,binary)=>['-O2','-g0','-Wall','-Wextra','-Werror','-isystem','../exact-real-references/flint/src',source,
 '-L','../exact-real-references/flint','-Wl,-rpath,'+resolve(workspace,'exact-real-references/flint'),'-lflint','-lmpfr','-lgmp','-lm','-lpthread','-o',binary];
for(const[tag,source,binary]of [['compile','audits/continuation/calcium/flint-mpoly-rational-controls.c',m.binaries[0].path],
 ['compile-fixed','audits/continuation/calcium/flint-mpoly-rational-controls-v2.c',m.binaries[0].path],
 ['upstream-compile','../exact-real-references/flint/src/fmpz_mpoly_q/test/main.c',m.binaries[1].path]]) {
 const g=gates.get(tag);assert.equal(g.cwd,comparison);assert.equal(g.command,'gcc');assert.deepEqual(g.args,compileArgs(source,binary));}
const native=gates.get('native'),mem=gates.get('memcheck'),up=gates.get('upstream-native');
assert.equal(native.cwd,comparison);assert.equal(native.command,m.binaries[0].path);assert.deepEqual(native.args,[]);
assert.equal(mem.cwd,comparison);assert.equal(mem.command,'valgrind');assert.deepEqual(mem.args,['--vgdb=no','--tool=memcheck',
 '--error-exitcode=97','--leak-check=full','--show-leak-kinds=all','--errors-for-leak-kinds=definite,indirect,possible',m.binaries[0].path]);
assert.equal(up.cwd,comparison);assert.equal(up.command,'env');assert.deepEqual(up.args,['FLINT_TEST_MULTIPLIER=1',m.binaries[1].path]);
for(const[tag,binary]of [['linked-libraries',m.binaries[0].path],['upstream-linked-libraries',m.binaries[1].path]]) {
 const g=gates.get(tag);assert.equal(g.cwd,comparison);assert.equal(g.command,'ldd');assert.deepEqual(g.args,[binary]);
 const s=readFileSync('results/mpoly-rational-'+tag+'.stdout','utf8');
 assert.deepEqual(Object.fromEntries([...s.matchAll(/=> (\/[^ ]+) \(/g)].map(x=>[x[1],sha(x[1])])),m.libraries);}
for(const[tag,script,flag]of [['output-check','check-mpoly-rational.mjs','--mpoly-rational-summary'],
 ['upstream-check','check-mpoly-rational-upstream.mjs','--mpoly-rational-upstream-summary']]) {
 const g=gates.get(tag);assert.equal(g.cwd,here);assert.equal(g.command,'node');assert.deepEqual(g.args,[script,flag]);}
for(const[a,b]of [['compile','compile-fixed'],['compile-fixed','native'],['native','memcheck'],['memcheck','output-check'],
 ['upstream-compile','upstream-native'],['upstream-native','upstream-check']])assert(Date.parse(gates.get(a).finished)<Date.parse(gates.get(b).started));
assert.equal(json('results/arf-conversion-verify-full.json').code,0);
console.log(JSON.stringify({checkpoint:'Multivariate rational-function architecture',files:46,gates:10,successfulGates:9,preservedFailedGates:1,
 liveFiles:956,readRecords:40,newDonorLines:3955,completeCurrentSliceFiles:36,binaryBytes:60840,checks:m.checks,upstreamChecks:m.upstreamChecks,
 status:m.status,production:m.production,findings:m.findings,comparison:m.comparison,preservedFailure:m.preservedFailure,limits:m.limits,followup:m.followup}));
