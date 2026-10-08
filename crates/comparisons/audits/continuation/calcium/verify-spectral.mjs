import './verify-charpoly-domain.mjs';
import { readFileSync, statSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
import { effectiveCoverage, effectiveSummary } from './effective-coverage.mjs';
const here=dirname(fileURLToPath(import.meta.url)),workspace=resolve(here,'../../../..');
const read=p=>readFileSync(resolve(here,p),'utf8'),json=p=>JSON.parse(read(p));
const sha=p=>createHash('sha256').update(readFileSync(resolve(here,p))).digest('hex');
const m=process.argv.includes('--draft-spectral')?(await import('./bind-spectral.mjs')).manifest:json('spectral-experiment.json');
assert.equal(Object.keys(m.files).length,28);
for(const [p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
for(const [p,b]of Object.entries(m.binaries)){assert.equal(sha(p),b.sha256,p);assert.equal(statSync(p).size,b.bytes,p);}
assert.equal(Object.keys(m.binaries).length,2);
assert.equal(sha(m.failureCore.path),m.failureCore.sha256);assert.equal(statSync(resolve(here,m.failureCore.path)).size,m.failureCore.bytes);
assert.equal(m.failureCore.bytes,78024704);
assert.deepEqual(m.nativeLibrary,json('charpoly-domain-experiment.json').nativeLibrary);assert.equal(sha(m.nativeLibrary.path),m.nativeLibrary.sha256);
const retained=json('retained-monic.json');
for(const [p,ranges]of Object.entries(m.hyperReadRanges)) {
  const text=read(`${retained.frozenSnapshot}/${p}`),n=text.split('\n').length-+text.endsWith('\n');
  assert(ranges.every(([a,b])=>a>=1&&b>=a&&b<=n),p);assert.equal(sha(`${retained.frozenSnapshot}/${p}`),retained.liveSources[p],p);
}
const coverage=json('coverage.json'),effective=effectiveCoverage(),inventory=json('inventory.json'),selection=json('spectral-read-selection.json');
const contains=(outer,inner)=>inner.every(([a,b])=>outer.some(([c,d])=>c<=a&&d>=b));
assert.equal(selection.length,66);assert.equal(m.reads.length,66);assert.equal(new Set(selection.map(e=>`${e.repo}:${e.path}`)).size,66);
let completeReads=0,newReadLines=0;
for(const [i,s]of selection.entries()) {
  const e=m.reads[i],live=coverage.find(v=>v.repo===s.repo&&v.path===s.path);assert(live);
  assert.equal(e.repo,s.repo);assert.equal(e.path,s.path);
  const f=inventory.sources.find(v=>v.repo===e.repo).files.find(v=>v.path===e.path);assert(f?.text);
  assert.equal(sha(resolve(workspace,'exact-real-references',e.repo,e.path)),f.sha256);
  assert(contains(live.ranges,e.ranges)&&contains(e.ranges,s.newRanges));
  assert(e.ranges.every(([a,b],i)=>a>=1&&b>=a&&b<=f.lines&&(!i||a>e.ranges[i-1][1])));
  if(e.ranges.length===1&&e.ranges[0][0]===1&&e.ranges[0][1]===f.lines) {
    completeReads++;assert.deepEqual(live,e);assert.deepEqual(s.newRanges,e.ranges);
  } else if(s.previous) {
    assert(e.note.startsWith(s.previous.note));assert.deepEqual(e.ranges,[...s.previous.ranges,...s.newRanges].sort((a,b)=>a[0]-b[0]));
    assert(s.newRanges.every(([a,b])=>s.previous.ranges.every(([c,d])=>b<c||a>d)));
  } else assert.deepEqual(s.newRanges,e.ranges);
  newReadLines+=s.newRanges.reduce((n,[a,b])=>n+b-a+1,0);
}
assert.equal(completeReads,63);assert.equal(newReadLines,4825);
assert.equal(m.extensions.length,1);
for(const e of m.extensions) {
  assert(json('coverage-extensions.json').some(v=>JSON.stringify(v)===JSON.stringify(e)));
  assert.deepEqual(e.ranges,[[515,624]]);assert.equal(e.repo,'flint');assert.equal(e.path,'doc/source/ca_mat.rst');
  const base=coverage.find(v=>v.repo===e.repo&&v.path===e.path),v=effective.find(v=>v.repo===e.repo&&v.path===e.path);
  assert(e.ranges.every(([a,b])=>base.ranges.every(([c,d])=>b<c||a>d)));assert(contains(v.ranges,e.ranges));
  const f=inventory.sources.find(s=>s.repo===e.repo).files.find(f=>f.path===e.path);
  assert.equal(sha(resolve(workspace,'exact-real-references',e.repo,e.path)),f.sha256);newReadLines+=110;
}
assert.equal(newReadLines,4935);
assert.deepEqual(m.coverageAtBinding,[{repo:'calcium',reviewed:375,complete:374,partial:1,readLines:37085},
  {repo:'flint',reviewed:400,complete:388,partial:12,readLines:42953}]);
for(const s of effectiveSummary()) {
  const old=m.coverageAtBinding.find(v=>v.repo===s.repo);assert(s.readLines>=old.readLines&&s.complete>=old.complete);
}
const directories=[];
for(const s of inventory.sources) {
  const prefix=s.repo==='flint'?'src/ca_mat/':'ca_mat/',files=s.files.filter(f=>f.path.startsWith(prefix));
  assert.equal(files.length,s.repo==='flint'?109:107);
  for(const f of files)assert.deepEqual(effective.find(e=>e.repo===s.repo&&e.path===f.path)?.ranges,[[1,f.lines]]);
  directories.push({repo:s.repo,files:files.length,lines:files.reduce((n,f)=>n+f.lines,0)});
}
const specs=[['spectral-native-compile',0,null],['spectral-native',0,null],['spectral-memcheck',0,null],
  ['matrix-function-native-compile',0,null],['matrix-function-native',1,null],['matrix-function-memcheck',null,'SIGABRT']];
assert.deepEqual(m.gates,specs.map(([tag,code,signal])=>({tag,code,signal})));
function gate(tag) {
  const spec=specs.find(s=>s[0]===tag);assert(spec);const g=json(`results/${tag}.json`);
  assert.equal(g.tag,tag);assert.equal(g.code,spec[1]);assert.equal(g.signal,spec[2]);assert(Date.parse(g.finished)>=Date.parse(g.started));
  return {...g,stdout:read(`results/${tag}.stdout`),stderr:read(`results/${tag}.stderr`)};
}
function rows(tag,header,summary) {
  const g=gate(tag),out=g.stdout.trimEnd().split('\n');assert.equal(g.stderr,'');assert.equal(out.shift(),header);
  assert.deepEqual(JSON.parse(out.pop()),summary);assert.equal(out.length,summary.rows);assert.equal(new Set(out).size,out.length);return out;
}
const spectral=rows('spectral-native','kind,pattern,shape,method,success,blocks,chain,invertible',
  {suite:'spectral',rows:486,incorrect:0,unresolved:0});let index=0;
for(let kind=0;kind<3;kind++)for(let pattern=0;pattern<9;pattern++)for(let shape=0;shape<3;shape++)for(let method=0;method<6;method++) {
  const p=method!==0&&method!==4;
  assert.equal(spectral[index++],[kind,pattern,shape,method,1,'True',p?'True':'na',p?'True':'na'].join(','));
}
assert.equal(index,486);
const functions=rows('matrix-function-native','kind,pattern,shape,alias,function,exists,status,equality',
  {suite:'matrix-function',rows:648,incorrect:0,unresolved:14});index=0;
let unknown=0,absent=0,proved=0;
for(let kind=0;kind<6;kind++)for(let pattern=0;pattern<9;pattern++)for(let shape=0;shape<3;shape++)for(let alias=0;alias<2;alias++)
for(const fn of ['exp','log']) {
  const exists=fn==='exp'||pattern===0||!(kind===3||(kind===5&&[6,7,8].includes(pattern)));
  const undecided=pattern===7&&((kind===1&&((fn==='exp'&&shape>0)||(fn==='log'&&shape===1)))||
    (kind===2&&(fn==='exp'||(fn==='log'&&shape===1))));
  const eq=!exists?'na':undecided?'Unknown':'True';unknown+=eq==='Unknown';absent+=!exists;proved+=eq==='True';
  assert.equal(functions[index++],[kind,pattern,shape,alias,fn,+exists,exists?'True':'False',eq].join(','));
}
assert.equal(index,648);assert.equal(unknown,14);assert.equal(absent,66);assert.equal(proved,568);
const memoryArgs=['--vgdb=no','--tool=memcheck','--error-exitcode=97','--leak-check=full',
  '--show-leak-kinds=all','--errors-for-leak-kinds=definite,indirect,possible'];
for(const [name,source]of [['spectral','flint-spectral-probe.c'],['matrix-function','flint-matrix-function-probe.c']]) {
  const c=gate(`${name}-native-compile`),g=gate(`${name}-native`),mem=gate(`${name}-memcheck`);
  assert.equal(c.command,'gcc');assert.equal(c.cwd,resolve(workspace,'exactcore-hyper-comparison'));assert.equal(c.stdout,'');assert.equal(c.stderr,'');
  assert.deepEqual(c.args,['-O2','-g0','-Wall','-Wextra','-Werror','-isystem','../exact-real-references/flint/src',
    `audits/continuation/calcium/${source}`,'-L','../exact-real-references/flint',
    '-Wl,-rpath,/home/tim/Documents/GitHub/workspace/exact-real-references/flint','-lflint','-lmpfr','-lgmp','-lm','-lpthread','-o',g.command]);
  assert(m.binaries[g.command]);assert.deepEqual(g.args,[]);assert.equal(mem.command,'valgrind');assert.deepEqual(mem.args,[...memoryArgs,g.command]);
  assert(Date.parse(c.finished)<=Date.parse(g.started)&&Date.parse(c.finished)<=Date.parse(mem.started));
  if(name==='spectral') {
    assert.equal(mem.stdout,g.stdout);assert.match(mem.stderr,/ERROR SUMMARY: 0 errors from 0 contexts/);
    assert.match(mem.stderr,/in use at exit: 0 bytes in 0 blocks/);assert.match(mem.stderr,/All heap blocks were freed -- no leaks are possible/);
  } else {
    assert.equal(Buffer.byteLength(mem.stdout),4096);assert(g.stdout.startsWith(mem.stdout));
    assert.equal(mem.stdout.split('\n').length-2,168);assert(!mem.stdout.endsWith('\n'));
    assert.match(mem.stderr,/fmpz_lll\/is_reduced_d.c:813: fmpz_lll_is_reduced_d: Assertion/);
    assert.match(mem.stderr,/ca_field_build_ideal/);assert.match(mem.stderr,/acb_multi_lindep/);
    assert.match(mem.stderr,/Process terminating with default action of signal 6/);
    assert.match(mem.stderr,/ERROR SUMMARY: 15 errors from 15 contexts/);
    assert.match(mem.stderr,/possibly lost: 136,296 bytes in 4,065 blocks/);
    assert.match(mem.stderr,/still reachable: 120,712 bytes in 740 blocks/);
    assert.equal((mem.stderr.match(/are possibly lost in loss record/g)||[]).length,15);
    assert(!/Invalid (?:read|write|free)|uninitialised|uninitialized|Conditional jump/.test(mem.stderr));
  }
}
console.log(JSON.stringify({checkpoint:'spectral source closure and matrix functions',completeReads,newReadLines,directories,
  jordanRows:486,matrixFunctionRows:648,provedFunctionRows:proved,requiredSingularLogFailures:absent,unresolvedFunctionEqualities:unknown,
  jordanMemory:'zero errors/all blocks freed',matrixFunctionMemory:'SIGABRT in internal LLL assertion; partial buffered output, not a clean gate',
  coverageAtBinding:m.coverageAtBinding,status:m.status,limits:m.limits}));
