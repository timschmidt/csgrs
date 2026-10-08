import './verify-retained-monic.mjs';
import { readFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here = dirname(fileURLToPath(import.meta.url)), workspace = resolve(here, '../../../..');
const read = p => readFileSync(resolve(here, p), 'utf8'), json = p => JSON.parse(read(p));
const sha = p => createHash('sha256').update(readFileSync(resolve(here, p))).digest('hex');
const m = process.argv.includes('--draft-matrix-solves')
  ? (await import('./bind-matrix-solves.mjs')).manifest : json('matrix-solve-experiment.json');
for (const [p, h] of Object.entries(m.files)) assert.equal(sha(p), h, p);
for (const [p, h] of Object.entries(m.binaries)) assert.equal(sha(p), h, p);
assert.equal(Object.keys(m.binaries).length, 5);
assert.equal(sha(m.nativeLibrary.path), m.nativeLibrary.sha256);
const retained = json('retained-monic.json');
const sourceKeys = Object.keys(retained.liveSources).filter(p => !p.startsWith('hyperreal/')).sort();
assert.deepEqual(Object.keys(m.candidateSources).sort(), sourceKeys); assert.equal(sourceKeys.length, 774);
for (const [p, h] of Object.entries(m.candidateSources)) assert.equal(sha(`rank-dominance-trial/${p}`), h, p);
assert.deepEqual(sourceKeys.filter(p => m.candidateSources[p] !== retained.liveSources[p]), ['hypersolve/src/rank.rs']);
assert.deepEqual(m.changed, ['hypersolve/src/rank.rs']);
for (const [p, ranges] of Object.entries(m.hyperReadRanges)) {
  const text = read(`${retained.frozenSnapshot}/${p}`), lines = text.split('\n').length - +text.endsWith('\n');
  assert(ranges.every(([a,b]) => a >= 1 && b >= a && b <= lines), p);
}
const coverage = json('coverage.json'), inventory = json('inventory.json'), selection = json('matrix-solve-read-selection.json');
assert.equal(selection.length, 51); assert.equal(new Set(selection.map(v => `${v.repo}:${v.path}`)).size, 51);
assert.deepEqual(m.reads, selection.map(e => coverage.find(v => v.repo === e.repo && v.path === e.path)));
assert.equal(m.reads.reduce((n,v) => n + v.ranges.reduce((s,[a,b]) => s+b-a+1, 0), 0), 4229);
let full = 0;
for (const v of m.reads) {
  const f = inventory.sources.find(s => s.repo === v.repo).files.find(f => f.path === v.path);
  assert.equal(sha(resolve(workspace,'exact-real-references',v.repo,v.path)),f.sha256);
  if (v.path === 'doc/source/ca_mat.rst') assert.deepEqual(v.ranges, [[330,455],[465,510]]);
  else { assert.deepEqual(v.ranges,[[1,f.lines]]); full++; }
}
assert.equal(full,50);
const tags = ['matrix-output-native-compile', 'matrix-output-native', 'matrix-output-memcheck',
  'rank-dominance-baseline-debug', 'rank-dominance-candidate-debug', 'rank-dominance-baseline-release',
  'rank-dominance-candidate-release', 'rank-dominance-baseline-memcheck', 'rank-dominance-candidate-memcheck',
  'rank-dominance-tests-debug', 'rank-dominance-tests-release', 'rank-dominance-fmt'];
assert.deepEqual(m.gates,tags);
function gate(tag) {
  assert(tags.includes(tag)); const g = json(`results/${tag}.json`);
  assert.equal(g.tag,tag); assert.equal(g.code,['matrix-output-native','matrix-output-memcheck'].includes(tag)?1:0,tag);
  assert.equal(g.signal,null); assert(Date.parse(g.finished)>=Date.parse(g.started));
  return { ...g, stdout:read(`results/${tag}.stdout`), stderr:read(`results/${tag}.stderr`) };
}
for (const tag of tags) gate(tag);
const native = gate('matrix-output-native').stdout.trimEnd().split('\n');
assert.equal(native.shift(),'family,kind,r,c,alias,method,success,rank_or_det,equality');
assert.deepEqual(JSON.parse(native.pop()),{suite:'matrix-output',rows:1074,failures:157});
assert.equal(native.length,1074); let index = 0, incorrect = 0, unknown = 0;
for (let r=0;r<=6;r++) for (let c=0;c<=6;c++) for (let seed=0;seed<3;seed++)
for (let alias=0;alias<2;alias++) for (let method=0;method<3;method++) {
  const bad = r>0&&c>0&&seed>0&&alias===0&&method!==2;
  assert.equal(native[index++],['rref',seed,r,c,alias,method,1,0,bad?'False':'True'].join(','));
  incorrect += +bad;
}
for (let kind=0;kind<3;kind++) for (let n=0;n<=7;n++) for (let alias=0;alias<2;alias++) {
  for (let method=0;method<3;method++) {
    const unresolved = kind===1&&n>=6&&method===1, bad = n===2&&alias===1&&method!==2;
    assert.equal(native[index++],['adjugate',kind,n,n,alias,method,1,unresolved?'Unknown':'True',
      unresolved?'Unknown':bad?'False':'True'].join(','));
    incorrect += +bad; unknown += +unresolved;
  }
  const bad = n===2&&alias===1;
  assert.equal(native[index++],['inverse',kind,n,n,alias,0,'True','na',bad?'False':'True'].join(','));
  incorrect += +bad;
}
assert.equal(incorrect,153); assert.equal(unknown,4);
assert.equal(gate('matrix-output-memcheck').stdout,gate('matrix-output-native').stdout);
const counts = {};
for (const variant of ['baseline','candidate']) {
  const raw = gate(`rank-dominance-${variant}-debug`).stdout, rows = raw.trimEnd().split('\n');
  assert.equal(rows.shift(),'family,kind,width,position,floor,status,expected_rank');
  assert.deepEqual(JSON.parse(rows.pop()),{suite:'rank-dominance',queries:828}); assert.equal(rows.length,828);
  let index=0, certified=0, unresolved=0;
  function expect(family,kind,width,position,floor,rank) {
    const opaque=kind===3||kind===4;
    const known=!opaque || (family!=='blocked' && (variant==='candidate'||(family!=='rank2'&&position===0)));
    assert.equal(rows[index++],[family,kind,width,position,floor,known?'Certified':'Undecided',rank].join(','));
    certified += +known; unresolved += +!known;
  }
  for (const floor of [-32,-128,-512]) for (let kind=0;kind<6;kind++) {
    for (const width of [2,3,4,6,8]) for(let position=0;position<width;position++) expect('row',kind,width,position,floor,1);
    for (const width of [2,3,4,6]) for(let position=0;position<width;position++) expect('column',kind,width,position,floor,1);
    for (const width of [3,4,5,6,7]) expect('rank2',kind,width,0,floor,2);
    expect('blocked',kind,2,0,floor,kind<=3?2:1);
    expect('blocked',kind,2,1,floor,kind<=3?1:0);
    expect('blocked',kind,3,2,floor,kind<=3?2:1);
  }
  counts[variant]={certified,unresolved};
  for (const phase of ['release','memcheck']) assert.equal(gate(`rank-dominance-${variant}-${phase}`).stdout,raw);
  for (const profile of ['debug','release']) {
    const g=gate(`rank-dominance-${variant}-${profile}`);
    assert.equal(g.cwd,resolve(here,`rank-dominance-${variant}`));
    for (const arg of ['cargo','run','--offline']) assert(g.args.includes(arg));
    assert.equal(g.args.includes('--release'),profile==='release');
    if(profile==='release') assert(g.args.includes('--locked'));
  }
}
assert.deepEqual(counts,{baseline:{certified:606,unresolved:222},candidate:{certified:810,unresolved:18}});
for(const tag of ['matrix-output-memcheck','rank-dominance-baseline-memcheck','rank-dominance-candidate-memcheck']) {
  const g=gate(tag); assert.equal(g.command,'valgrind');
  for (const arg of ['--vgdb=no','--tool=memcheck','--error-exitcode=97','--leak-check=full','--show-leak-kinds=all','--errors-for-leak-kinds=definite,indirect,possible']) assert(g.args.includes(arg));
  assert.match(g.stderr,/ERROR SUMMARY: 0 errors from 0 contexts/);
  if(tag==='matrix-output-memcheck') assert.match(g.stderr,/in use at exit: 0 bytes in 0 blocks/);
  else for(const kind of ['definitely','indirectly','possibly']) assert.match(g.stderr,new RegExp(`${kind} lost: 0 bytes in 0 blocks`));
}
const names=s=>[...s.matchAll(/^test (.+) \.\.\. (ok|ignored|FAILED).*$/gm)].map(m=>`${m[1]}:${m[2]}`).sort();
for(const profile of ['debug','release']) {
  const g=gate(`rank-dominance-tests-${profile}`);
  assert.equal(g.cwd,resolve(here,'rank-dominance-trial/hypersolve'));
  for(const arg of ['cargo','test','--offline','--locked','--all-features']) assert(g.args.includes(arg));
  assert.equal(g.args.includes('--release'),profile==='release');
  assert.deepEqual(names(g.stdout),names(read(`results/retained-monic-${profile}.stdout`)));
  assert.equal(names(g.stdout).length,803);
  const suites=[...g.stdout.matchAll(/test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;/g)];
  assert.equal(suites.length,8); assert.equal(suites.reduce((n,m)=>n+ +m[1],0),803);
  assert(suites.every(m=>m.slice(2).every(v=>v==='0')));
}
assert.deepEqual(gate('rank-dominance-fmt').args,['fmt','--','--check']);
console.log(JSON.stringify({checkpoint:'matrix solve outputs and isolated rank dominance',readFiles:51,completeReads:50,
  readLines:4229,nativeOutputRows:1074,incorrectRows:incorrect,unresolvedNativeRows:unknown,rankQueriesPerVariantPerProfile:828,
  rankResults:counts,newlyCertified:204,preservedUnresolved:18,candidateTestsPerProfile:803,
  status:m.status,limits:m.limits}));
