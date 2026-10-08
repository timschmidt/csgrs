import './verify-monic-state.mjs';
import {readFileSync} from 'node:fs';
import {createHash} from 'node:crypto';
import {dirname,resolve} from 'node:path';
import {fileURLToPath} from 'node:url';
import assert from 'node:assert/strict';
const here=dirname(fileURLToPath(import.meta.url)),workspace=resolve(here,'../../../..');
const read=p=>readFileSync(resolve(here,p),'utf8'),json=p=>JSON.parse(read(p));
const sha=p=>createHash('sha256').update(readFileSync(resolve(here,p))).digest('hex');
const manifest=process.argv.includes('--draft-matrix-pivots')?(await import('./bind-matrix-pivots.mjs')).manifest:json('matrix-pivot-experiment.json');
for(const [p,h]of Object.entries(manifest.files))assert.equal(sha(p),h,p);
for(const [p,h]of Object.entries(manifest.binaries))assert.equal(sha(p),h,p);
assert.equal(Object.keys(manifest.binaries).length,6);
const coverage=json('coverage.json'),inventory=json('inventory.json'),selection=json('matrix-pivot-read-selection.json');
assert.equal(selection.length,47);assert.equal(new Set(selection.map(v=>`${v.repo}:${v.path}`)).size,47);
assert.deepEqual(manifest.reads,selection.map(e=>coverage.find(c=>c.repo===e.repo&&c.path===e.path)));
assert.equal(manifest.reads.reduce((n,v)=>n+v.ranges.reduce((s,[a,b])=>s+b-a+1,0),0),4517);
for(const v of manifest.reads){const f=inventory.sources.find(s=>s.repo===v.repo).files.find(f=>f.path===v.path);
  assert.deepEqual(v.ranges,[[1,f.lines]]);assert.equal(sha(resolve(workspace,'exact-real-references',v.repo,v.path)),f.sha256);}
const tags=['matrix-pivot-native-compile','matrix-pivot-native','matrix-pivot-native-memcheck',
  'matrix-pivot-hyper-debug','matrix-pivot-hyper-release','matrix-pivot-hyper-memcheck','matrix-pivot-donor-tests',
  'matrix-methods-debug','matrix-methods-release','matrix-methods-memcheck','monic-qualified-consumer-hypercurve-debug'];
assert.deepEqual(manifest.gates,tags);
function gate(tag){assert(tags.includes(tag));const g=json(`results/${tag}.json`);assert.equal(g.code,0,tag);assert.equal(g.signal,null);
  assert.equal(g.tag,tag);assert(Date.parse(g.finished)>=Date.parse(g.started));
  return {...g,stdout:read(`results/${tag}.stdout`),stderr:read(`results/${tag}.stderr`)};}
for(const tag of tags)gate(tag);
const native=gate('matrix-pivot-native').stdout.trimEnd().split('\n');
assert.equal(native.shift(),'kind,n,shape,expected_zero,default,berkowitz,bareiss_success,bareiss,lu_success,lu,nonsingular_lu');
assert.deepEqual(JSON.parse(native.pop()),{suite:'matrix-pivots',cases:198,failures:0,default_unknown:10,lu_unknown:33});
const hyper=gate('matrix-pivot-hyper-debug').stdout.trimEnd().split('\n');
assert.equal(hyper.shift(),'kind,n,shape,expected_zero,method,equality');
assert.deepEqual(JSON.parse(hyper.pop()),{suite:'hyper-matrix-pivots',cases:198,unknown:16,pivot_free:0});
assert.equal(native.length,198);assert.equal(hyper.length,198);
let index=0;
for(let kind=0;kind<3;kind++)for(let n=0;n<=10;n++)for(let shape=0;shape<6;shape++){
  const zero=n>0&&((shape>=1&&shape<=3)||(shape===5&&n>1));
  const luUnknown=(n>=4&&shape===1)||(n>=7&&shape===2);
  const defaultUnknown=kind===1&&n>=5&&luUnknown;
  assert.equal(native[index],[kind,n,shape,+zero,defaultUnknown?'Unknown':'True','True',1,'True',luUnknown?0:1,
    luUnknown?'Unknown':'True',luUnknown?'Unknown':zero?'False':'True'].join(','));
  const hyperUnknown=kind===2&&n>=3&&(shape===0||shape===4);
  assert.equal(hyper[index],[kind,n,shape,+zero,'FractionFree',hyperUnknown?'Unknown':'Equal'].join(','));index++;
}
for(const phase of ['release','memcheck'])assert.equal(gate(`matrix-pivot-hyper-${phase}`).stdout,gate('matrix-pivot-hyper-debug').stdout);
assert.equal(gate('matrix-pivot-native-memcheck').stdout,gate('matrix-pivot-native').stdout);
const methods=gate('matrix-methods-debug').stdout.trimEnd().split('\n');
assert.equal(methods.shift(),'family,kind,n,case,method,equality');
assert.deepEqual(JSON.parse(methods.pop()),{suite:'matrix-methods',inputs:310,queries:930});
assert.equal(methods.length,930);index=0;
const counts={Bareiss:{Equal:0,Unknown:0},Faddeev:{Equal:0,Unknown:0},Berkowitz:{Equal:0,Unknown:0}};
let gains=0,losses=0;
function methodGroup(family,kind,n,which){
  const result={};
  for(const method of ['Bareiss','Faddeev','Berkowitz']){
    let known;
    if(family==='structured')known=!(kind===2&&n>=3&&(which===4||(which===0&&method!=='Berkowitz')));
    else known=kind<=1||n<=1||(n===2&&which===3)||(n===3&&[1,2].includes(which)&&!(kind===2&&method==='Berkowitz'));
    const equality=known?'Equal':'Unknown';
    assert.equal(methods[index++],[family,kind,n,which,method,equality].join(','));counts[method][equality]++;result[method]=known;
  }
  if(!result.Bareiss&&result.Berkowitz)gains++;
  if(result.Bareiss&&!result.Berkowitz)losses++;
}
for(let kind=0;kind<3;kind++)for(let n=0;n<=10;n++)for(let shape=0;shape<6;shape++)methodGroup('structured',kind,n,shape);
for(let kind=0;kind<4;kind++)for(let n=0;n<=6;n++)for(let c=0;c<4;c++)methodGroup('dense',kind,n,c);
assert.deepEqual(counts,{Bareiss:{Equal:260,Unknown:50},Faddeev:{Equal:260,Unknown:50},Berkowitz:{Equal:266,Unknown:44}});
assert.equal(gains,8);assert.equal(losses,2);
for(const phase of ['release','memcheck'])assert.equal(gate(`matrix-methods-${phase}`).stdout,gate('matrix-methods-debug').stdout);
for(const tag of ['matrix-pivot-native-memcheck','matrix-pivot-hyper-memcheck','matrix-methods-memcheck']){
  const g=gate(tag);assert.equal(g.command,'valgrind');
  for(const flag of ['--vgdb=no','--tool=memcheck','--error-exitcode=97','--leak-check=full','--show-leak-kinds=all','--errors-for-leak-kinds=definite,indirect,possible'])assert(g.args.includes(flag));
  assert.match(g.stderr,/ERROR SUMMARY: 0 errors from 0 contexts/);
  if(tag==='matrix-pivot-native-memcheck')assert.match(g.stderr,/in use at exit: 0 bytes in 0 blocks/);
  else for(const kind of ['definitely','indirectly','possibly'])assert.match(g.stderr,new RegExp(`${kind} lost: 0 bytes in 0 blocks`));
}
for(const pkg of ['matrix-pivot-hyper','matrix-methods'])for(const profile of ['debug','release']){
  const g=gate(`${pkg}-${profile}`);assert(g.args.includes('run')&&g.args.includes('--offline'));
  assert.equal(g.args.includes('--release'),profile==='release');
}
const donor=gate('matrix-pivot-donor-tests');assert.equal(donor.command,'make');assert.deepEqual(donor.args,['-j2','check','MOD=ca_mat']);
const registered=[...read(resolve(workspace,'exact-real-references/flint/src/ca_mat/test/main.c')).matchAll(/TEST_FUNCTION\(([^)]+)\)/g)].map(m=>m[1]).sort();
const passed=[...donor.stdout.replace(/\x1b\[[0-9;]*m/g,'').matchAll(/^(ca_mat_\w+)\s+[\d.]+\s+\(PASS\)/gm)].map(m=>m[1]).sort();
assert.equal(registered.length,28);assert.deepEqual(passed,registered);
const curve=gate('monic-qualified-consumer-hypercurve-debug');
assert.equal(curve.cwd,resolve(here,'polynomial-monic-qualified-trial/hypercurve'));
for(const flag of ['test','--offline','--locked','--all-features','--lib','--tests'])assert(curve.args.includes(flag));
const names=text=>[...text.matchAll(/^test (.+) \.\.\. (ok|ignored|FAILED).*$/gm)].map(m=>`${m[1]}:${m[2]}`).sort();
assert.deepEqual(names(curve.stdout),names(read('results/polynomial-facts-consumer-hypercurve-debug.stdout')));
assert.equal(names(curve.stdout).length,1770);
const suites=[...curve.stdout.matchAll(/test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;/g)];
assert.equal(suites.length,45);assert.equal(suites.reduce((n,m)=>n+ +m[1],0),1761);assert.equal(suites.reduce((n,m)=>n+ +m[3],0),9);
assert(suites.every(m=>[m[2],m[4],m[5]].every(n=>n==='0')));
console.log(JSON.stringify({checkpoint:'matrix pivots and monic downstream',readFiles:47,readLines:4517,
  nativeCases:198,nativeDefaultUnknown:10,nativeLuUnknown:33,hyperCasesPerProfile:198,hyperUnresolvedEqualities:16,
  methodQueriesPerProfile:930,methodResults:counts,berkowitzGains:gains,berkowitzLosses:losses,
  donorTests:28,hypercurvePassed:1761,hypercurveIgnored:9,hypercurveSuites:45,status:manifest.status,limits:manifest.limits}));
