import './verify-rank-costs.mjs';
import { readFileSync, statSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here=dirname(fileURLToPath(import.meta.url)),workspace=resolve(here,'../../../..');
const read=p=>readFileSync(resolve(here,p),'utf8'),json=p=>JSON.parse(read(p));
const sha=p=>createHash('sha256').update(readFileSync(resolve(here,p))).digest('hex');
const m=process.argv.includes('--draft-charpoly-domain')?(await import('./bind-charpoly-domain.mjs')).manifest:json('charpoly-domain-experiment.json');
for(const [p,h]of Object.entries(m.files))assert.equal(sha(p),h,p);
assert.equal(Object.keys(m.files).length,72);
for(const [p,b]of Object.entries(m.binaries)){assert.equal(sha(p),b.sha256,p);assert.equal(statSync(p).size,b.bytes,p);}
assert.equal(Object.keys(m.binaries).length,7);assert.equal(sha(m.nativeLibrary.path),m.nativeLibrary.sha256);
assert.deepEqual(m.nativeLibrary,json('rank-cost-experiment.json').nativeLibrary);
const retained=json('retained-monic.json');
for(const [p,ranges]of Object.entries(m.hyperReadRanges)) {
  const text=read(`${retained.frozenSnapshot}/${p}`),n=text.split('\n').length-+text.endsWith('\n');
  assert(ranges.every(([a,b])=>a>=1&&b>=a&&b<=n),p);
  assert.equal(sha(`${retained.frozenSnapshot}/${p}`),retained.liveSources[p],p);
}
const coverage=json('coverage.json'),inventory=json('inventory.json'),selection=json('charpoly-domain-read-selection.json');
assert.equal(selection.length,36);assert.equal(new Set(selection.map(e=>`${e.repo}:${e.path}`)).size,36);
assert.equal(m.reads.length,36);let newReadLines=0,completeReads=0;
const contains=(outer,inner)=>inner.every(([a,b])=>outer.some(([c,d])=>c<=a&&d>=b));
for(const [i,s]of selection.entries()) {
  const e=m.reads[i],live=coverage.find(v=>v.repo===s.repo&&v.path===s.path);
  assert.equal(e.repo,s.repo);assert.equal(e.path,s.path);assert(live);
  const f=inventory.sources.find(v=>v.repo===e.repo).files.find(v=>v.path===e.path);assert(f?.text);
  assert.equal(sha(resolve(workspace,'exact-real-references',e.repo,e.path)),f.sha256);
  assert(contains(live.ranges,e.ranges)&&contains(e.ranges,s.newRanges));
  assert(e.ranges.every(([a,b],i)=>a>=1&&b>=a&&b<=f.lines&&(!i||a>e.ranges[i-1][1])));
  const full=e.ranges.length===1&&e.ranges[0][0]===1&&e.ranges[0][1]===f.lines;
  if(full){assert.deepEqual(live,e);assert.deepEqual(s.newRanges,e.ranges);completeReads++;}
  else if(s.previous) {
    assert.equal(s.previous.repo,s.repo);assert.equal(s.previous.path,s.path);
    assert(e.note.startsWith(s.previous.note));
    assert.deepEqual(e.ranges,[...s.previous.ranges,...s.newRanges].sort((a,b)=>a[0]-b[0]));
    assert(s.newRanges.every(([a,b])=>s.previous.ranges.every(([c,d])=>b<c||a>d)));
  } else assert.deepEqual(s.newRanges,e.ranges);
  newReadLines+=s.newRanges.reduce((n,[a,b])=>n+b-a+1,0);
}
assert.equal(completeReads,32);assert.equal(newReadLines,5913);
const codes={
  'solve-certificate-native-compile':0,'solve-certificate-native':0,'solve-certificate-memcheck':0,
  'charpoly-certificate-native-compile':0,'charpoly-certificate-native':0,'charpoly-certificate-memcheck':0,
  'ca-domain-native-compile':1,'ca-domain-native-compile-corrected':0,'ca-domain-native':1,'ca-domain-memcheck':1,
  'ca-arg-witness-compile':0,'ca-arg-witness-native':1,'ca-arg-witness-memcheck':1,
  'ca-negative-arg-witness-compile':0,'ca-negative-arg-witness-native':0,'ca-negative-arg-witness-memcheck':0,
  'hyper-domain-debug':0,'hyper-domain-release':0,'hyper-domain-memcheck':0};
assert.deepEqual(m.gates,Object.entries(codes).map(([tag,code])=>({tag,code})));
function gate(tag) {
  assert(tag in codes);const g=json(`results/${tag}.json`);
  assert.equal(g.tag,tag);assert.equal(g.code,codes[tag],tag);assert.equal(g.signal,null);
  assert(Number.isFinite(g.elapsedSeconds)&&g.elapsedSeconds>=0);assert(Date.parse(g.finished)>=Date.parse(g.started));
  return {...g,stdout:read(`results/${tag}.stdout`),stderr:read(`results/${tag}.stderr`)};
}
for(const tag of Object.keys(codes))gate(tag);
function corpus(tag,header,summary) {
  const g=gate(tag),rows=g.stdout.trimEnd().split('\n');assert.equal(g.stderr,'');
  assert.equal(rows.shift(),header);assert.deepEqual(JSON.parse(rows.pop()),summary);assert.equal(rows.length,summary.rows);
  assert.equal(new Set(rows).size,rows.length);return rows;
}
const solve=corpus('solve-certificate-native','family,kind,n,columns,upper,unit,singular,alias,method,status,equality',
  {suite:'matrix-solve-certificate',rows:4464,incorrect:0,unresolved:0});
let index=0;
for(let kind=0;kind<3;kind++)for(const n of [0,1,3,4,9,10,11])for(const cols of [0,1,3,9,10,11])
for(let upper=0;upper<2;upper++)for(let unit=0;unit<2;unit++)for(let alias=0;alias<2;alias++)for(let method=0;method<3;method++)
  assert.equal(solve[index++],['triangular',kind,n,cols,upper,unit,0,alias,method,'True','True'].join(','));
assert.equal(index,3024);
for(let kind=0;kind<3;kind++)for(const n of [0,1,2,3,4,5,6,10])for(const cols of [0,1,3,10])
for(let singular=0;singular<2;singular++) {
  if(singular&&n===0)continue;
  for(let alias=0;alias<2;alias++)for(let method=0;method<4;method++)
    assert.equal(solve[index++],['nonsingular',kind,n,cols,0,0,singular,alias,method,singular?'False':'True',singular?'na':'True'].join(','));
}
assert.equal(index,4464);
const charpoly=corpus('charpoly-certificate-native','family,kind,n,shape,seed,method,success,equality',
  {suite:'charpoly-certificate',rows:432,incorrect:0,unresolved:0});index=0;
for(let kind=0;kind<3;kind++)for(const n of [0,1,2,3,4,5,6,8,10])for(let shape=0;shape<4;shape++)
for(let seed=0;seed<2;seed++)for(let method=0;method<2;method++)
  assert.equal(charpoly[index++],['charpoly',kind,n,shape,seed,method,1,'True'].join(','));
assert.equal(index,432);
const domain=corpus('ca-domain-native','family,domain,operation,numerator,alias,status,real,algebraic,special',
  {suite:'ca-domain',rows:580,incorrect:14,unresolved:18});index=0;
// Reconstruct every observed status/predicate outcome; these preserve failures,
// not an endorsement of the reported parent-domain or vector-space claims.
let propertyFailures=0,nonrealSuccesses=0,unknownAlgebraicity=0;
for(let d=0;d<4;d++) {
  assert.equal(domain[index++],['property',d,'real_vector_space',0,0,'na','True','na','na'].join(','));
  propertyFailures+=d>=2;
  for(const n of [-4,-2,-1,0,1,2,4])for(const op of ['asin','acos','arg','sqrt','log','exp','pow_half'])
  for(let alias=0;alias<3;alias++) {
    if(op==='arg'&&n===0)continue;
    let status=0,real='True',alg='Unknown';
    if(op==='asin'||op==='acos') {
      const trivial=op==='asin'?n===0:n===2;alg=trivial?'True':'Unknown';
      real=Math.abs(n)<=2?'True':'False';if(d>=2&&!trivial)status=1;
    } else if(op==='arg')alg=n>0?'True':'Unknown';
    else if(op==='sqrt'||op==='pow_half') {
      alg='True';real=n<0?'False':'True';if(d%2===0&&n<0)status=1;
    } else if(op==='log') {
      alg=n===2?'True':'Unknown';real=n<0?'False':'True';
      if(n===0||(d%2===0&&n<0)||(d>=2&&n!==2))status=1;
    } else {alg=n===0?'True':'Unknown';if(d>=2&&n!==0)status=1;}
    assert.equal(domain[index++],['operation',d,op,n,alias,status,...(status?['na','na','na']:[real,alg,0])].join(','));
    nonrealSuccesses+=status===0&&d%2===0&&real==='False';
    unknownAlgebraicity+=status===0&&d>=2&&alg==='Unknown';
  }
}
assert.equal(index,580);assert.equal(propertyFailures,2);assert.equal(nonrealSuccesses,12);assert.equal(unknownAlgebraicity,18);
for(const negative of [false,true]) {
  const name=negative?'ca-negative-arg-witness':'ca-arg-witness';
  const rows=corpus(`${name}-native`,`domain,numerator,alias,status,equals_${negative?'negative_pi':'pi'}`,
    {suite:name,rows:36,failures:negative?0:36});index=0;
  for(let d=0;d<4;d++)for(const n of [-4,-2,-1])for(let alias=0;alias<3;alias++)
    assert.equal(rows[index++],[d,n,alias,0,negative?'True':'False'].join(','));
  assert.equal(index,36);
}
const valid=[false,true,true,true,true,true,false,true,false,true,false,false,true,false,true,false,false];
let hyperRaw;
for(const profile of ['debug','release']) {
  const rows=corpus(`hyper-domain-${profile}`,'input,state,operation,expected_valid,result',{suite:'hyper-domain',rows:102});index=0;
  for(let i=0;i<17;i++)for(let state=0;state<3;state++)for(const op of ['asin','acos'])
    assert.equal(rows[index++],[i,state,op,valid[i],valid[i]?'Ok':'NotANumber'].join(','));
  assert.equal(index,102);
  const g=gate(`hyper-domain-${profile}`);assert.equal(g.command,'env');assert.equal(g.cwd,resolve(here,'hyper-domain-probe'));
  assert.deepEqual(g.args,['CARGO_TARGET_DIR=/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse','CARGO_INCREMENTAL=0',
    'CARGO_PROFILE_DEV_DEBUG=0','CARGO_BUILD_JOBS=2','cargo','run','--offline',...(profile==='release'?['--locked']:[]),
    '--quiet',...(profile==='release'?['--release']:[])]);
  if(hyperRaw!==undefined)assert.equal(g.stdout,hyperRaw);hyperRaw=g.stdout;
}
const memArgs=['--vgdb=no','--tool=memcheck','--error-exitcode=97','--leak-check=full',
  '--show-leak-kinds=all','--errors-for-leak-kinds=definite,indirect,possible'];
const compiled=[
  ['solve-certificate','solve-certificate-native-compile','flint-matrix-solve-certificate-probe.c'],
  ['charpoly-certificate','charpoly-certificate-native-compile','flint-charpoly-certificate-probe.c'],
  ['ca-domain','ca-domain-native-compile-corrected','flint-ca-domain-probe.c'],
  ['ca-arg-witness','ca-arg-witness-compile','flint-ca-arg-witness-probe.c'],
  ['ca-negative-arg-witness','ca-negative-arg-witness-compile','flint-ca-negative-arg-witness-probe.c']];
for(const [name,compile,source]of compiled) {
  const c=gate(compile),g=gate(`${name}-native`),memory=gate(`${name}-memcheck`);
  assert.equal(c.command,'gcc');assert.equal(c.cwd,resolve(workspace,'exactcore-hyper-comparison'));
  assert.deepEqual(c.args,['-O2','-g0','-Wall','-Wextra','-Werror','-isystem','../exact-real-references/flint/src',
    `audits/continuation/calcium/${source}`,'-L','../exact-real-references/flint',
    '-Wl,-rpath,/home/tim/Documents/GitHub/workspace/exact-real-references/flint',
    '-lflint','-lmpfr','-lgmp','-lm','-lpthread','-o',g.command]);
  assert.equal(c.stdout,'');assert.equal(c.stderr,'');assert(m.binaries[g.command]);assert.deepEqual(g.args,[]);
  assert(Date.parse(c.finished)<=Date.parse(g.started));assert(Date.parse(c.finished)<=Date.parse(memory.started));
  assert.equal(memory.command,'valgrind');assert.deepEqual(memory.args,[...memArgs,g.command]);assert.equal(memory.stdout,g.stdout);
  assert.match(memory.stderr,/ERROR SUMMARY: 0 errors from 0 contexts/);
  assert.match(memory.stderr,/in use at exit: 0 bytes in 0 blocks/);
  assert.match(memory.stderr,/All heap blocks were freed -- no leaks are possible/);
}
const bad=gate('ca-domain-native-compile');assert.equal(bad.command,'gcc');assert.equal(bad.stdout,'');
assert.match(bad.stderr,/unknown type name .ca_ctx_ptr/);
assert.deepEqual(bad.args,gate('ca-domain-native-compile-corrected').args);
assert(Date.parse(bad.finished)<=Date.parse(gate('ca-domain-native-compile-corrected').started));
assert.equal(read('flint-ca-domain-probe-initial.c').replace('ca_ctx_ptr ca_ctx=(ca_ctx_ptr)',
  'ca_ctx_struct *ca_ctx=(ca_ctx_struct *)'),read('flint-ca-domain-probe.c'));
const hyperMemory=gate('hyper-domain-memcheck');assert.equal(hyperMemory.stdout,hyperRaw);assert.equal(hyperMemory.command,'valgrind');
assert.deepEqual(hyperMemory.args,[...memArgs,'/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse/release/calcium-hyper-domain-probe']);
assert(Date.parse(gate('hyper-domain-release').finished)<=Date.parse(hyperMemory.started));
assert.match(hyperMemory.stderr,/ERROR SUMMARY: 0 errors from 0 contexts/);
for(const kind of ['definitely','indirectly','possibly'])assert.match(hyperMemory.stderr,new RegExp(`${kind} lost: 0 bytes in 0 blocks`));
assert.match(hyperMemory.stderr,/still reachable: 3,112 bytes in 28 blocks/);
console.log(JSON.stringify({checkpoint:'matrix solve/characteristic certificates and scalar domains',completeReads,newReadLines,
  solveRows:4464,charpolyRows:432,domainRows:580,domainFailures:{propertyFailures,nonrealSuccesses,
    algebraicDomainSuccessesProvedTranscendentalByNegativePi:unknownAlgebraicity},
  principalArgReproductionRows:36,negativePiValueWitnessRows:36,hyperDomainRowsPerProfile:102,
  nativeMemory:'zero errors/all blocks freed; mathematical-failure processes still exit one',
  hyperMemory:'zero errors/no lost blocks; 3112 bytes still reachable',status:m.status,limits:m.limits}));
