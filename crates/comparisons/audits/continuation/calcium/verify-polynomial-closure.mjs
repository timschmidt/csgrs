import './verify-retained-polynomial-facts.mjs';
import { readFileSync, readdirSync, realpathSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here = dirname(fileURLToPath(import.meta.url)), workspace = resolve(here, '../../../..');
const read = p => readFileSync(resolve(here, p), 'utf8');
const json = p => JSON.parse(read(p));
const sha = p => createHash('sha256').update(readFileSync(resolve(here, p))).digest('hex');
const manifest = process.argv.includes('--draft')
  ? (await import('./bind-polynomial-closure.mjs')).manifest : json('polynomial-closure-experiment.json');
for (const [path, hash] of Object.entries(manifest.files)) assert.equal(sha(path), hash, path);
for (const [path, hash] of Object.entries(manifest.binaries)) assert.equal(sha(path), hash, path);
assert.equal(Object.keys(manifest.binaries).length, 8);
assert.equal(sha(manifest.nativeLibrary.path), manifest.nativeLibrary.sha256);
const baseline = json('retained-polynomial-facts.json'), changed = [];
assert.equal(Object.keys(manifest.candidateSources).length, 339);
function tree(path, prefix = '') {
  return readdirSync(path, { withFileTypes: true }).flatMap(v => {
    if (v.isSymbolicLink()) { assert.equal(`${prefix}${v.name}`, 'hyperreal'); return []; }
    return v.isDirectory() ? tree(resolve(path, v.name), `${prefix}${v.name}/`) : [`${prefix}${v.name}`];
  });
}
assert.deepEqual(tree(resolve(here, 'polynomial-monic-trial')).sort(), Object.keys(manifest.candidateSources).sort());
assert.equal(realpathSync(resolve(here, 'polynomial-monic-trial/hyperreal')), resolve(here, 'root-exp-reuse-trial-hyperreal'));
for (const [path, hash] of Object.entries(manifest.candidateSources)) {
  assert.equal(sha(`polynomial-monic-trial/${path}`), hash, path);
  assert.equal(sha(`${baseline.frozenSnapshot}/${path}`), baseline.liveSources[path], `baseline ${path}`);
  if (hash !== baseline.liveSources[path]) changed.push(path);
}
assert.deepEqual(changed, ['hypersolve/src/root_isolation.rs']);
assert.deepEqual(manifest.changed, changed);
const selection = json('polynomial-closure-read-selection.json'), coverage = json('coverage.json'), inventory = json('inventory.json');
assert.equal(selection.length, 34);
assert.deepEqual(manifest.reads, selection.map(s => coverage.find(v => v.repo === s.repo && v.path === s.path)));
assert.equal(new Set(selection.map(v => `${v.repo}:${v.path}`)).size, 34);
for (const v of manifest.reads) {
  const f = inventory.sources.find(s => s.repo === v.repo).files.find(f => f.path === v.path);
  assert.deepEqual(v.ranges, [[1, f.lines]]);
  assert.equal(sha(resolve(workspace, 'exact-real-references', v.repo, v.path)), f.sha256);
}
assert.equal(manifest.reads.reduce((n,v) => n + v.ranges[0][1],0), 4493);
const phases = ['series-compile', 'series-native', 'series-memcheck', 'sparse-compile',
  'sparse-native', 'sparse-memcheck', 'roots-compile', 'roots-native', 'roots-memcheck',
  'hyper-debug', 'observe-debug', 'observe-release', 'observe-memcheck',
  'monic-observe-debug', 'monic-observe-release', 'monic-observe-memcheck',
  'monic-tests-debug', 'monic-tests-release', 'monic-fmt'];
assert.deepEqual(manifest.gates, phases.map(p => `polynomial-closure-${p}`));
function gate(phase) {
  const tag = `polynomial-closure-${phase}`, g = json(`results/${tag}.json`);
  assert.equal(g.code, phase === 'hyper-debug' ? 101 : 0, tag); assert.equal(g.signal, null);
  assert.equal(g.tag, tag); assert(Date.parse(g.finished) >= Date.parse(g.started));
  assert.equal(g.cwd, resolve(here, phase.startsWith('monic-tests-') || phase === 'monic-fmt'
    ? 'polynomial-monic-trial/hypersolve' : '../../..'));
  return { ...g, stdout: read(`results/${tag}.stdout`), stderr: read(`results/${tag}.stderr`) };
}
for (const phase of phases) gate(phase);
function csv(phase, header) {
  const lines = gate(phase).stdout.trimEnd().split('\n');
  assert.equal(lines.shift(), header, phase);
  return { rows: lines.slice(0,-1).map(s => s.split(',')), summary: JSON.parse(lines.at(-1)) };
}
function memcheck(phase, native) {
  const g = gate(phase);
  assert.equal(g.command, 'valgrind');
  for (const flag of ['--leak-check=full', '--show-leak-kinds=all',
    '--errors-for-leak-kinds=definite,indirect,possible']) assert(g.args.includes(flag));
  assert(g.args.some(v => /^--error-exitcode=\d+$/.test(v) && !v.endsWith('=0')));
  assert.match(g.stderr, /ERROR SUMMARY: 0 errors from 0 contexts/);
  if (native) assert.match(g.stderr, /in use at exit: 0 bytes in 0 blocks/);
  else for (const kind of ['definitely', 'indirectly', 'possibly'])
    assert.match(g.stderr, new RegExp(`${kind} lost: 0 bytes in 0 blocks`));
}
for (const [phase, header, expected, summary] of [
  ['series', 'mode,length,constant,order,checks,failures',
    [0,1,2,3].flatMap(m => [1,2,4,8,9].flatMap(l => [1,2].flatMap(c => [0,1,2,3,8,15,17].map(n => [m,l,c,n,21,0])))),
    { suite: 'polynomial-series', cases: 280, checks: 5880, failures: 0 }],
  ['sparse', 'kind,degree,constant,order,checks,failures',
    [0,1].flatMap(k => Array.from({length:9},(_,i) => i+1).flatMap(d => [1,2].flatMap(c => Array.from({length:21},(_,n) => [k,d,c,n,4,0])))),
    { suite: 'polynomial-sparse-series', cases: 756, checks: 3024, failures: 0 }],
]) {
  const result = csv(`${phase}-native`, header);
  assert.deepEqual(result.rows.map(r => r.map(Number)), expected); assert.deepEqual(result.summary, summary);
  assert.equal(gate(`${phase}-native`).stdout, gate(`${phase}-memcheck`).stdout);
  memcheck(`${phase}-memcheck`, true);
}
const rootHeader = 'kind,code,set_equal,factor_success,factor_equal,squarefree_success,squarefree_equal,roots_success,root_count,matches';
const roots = csv('roots-native', rootHeader);
const cases = [0,1,2].flatMap(kind => Array.from({length:27},(_,code) => {
  const exponents = [code % 3, Math.floor(code/3) % 3, Math.floor(code/9) % 3];
  return { kind, code, exponents, degree: exponents.filter(Boolean).length };
}));
assert.deepEqual(roots.rows.map(r => r.map(Number)), cases.map(v => [v.kind,v.code,1,1,1,1,1,1,v.degree,v.degree]));
assert.deepEqual(roots.summary, {suite:'polynomial-roots',cases:81,failed_cases:0,factor_failures:0,root_failures:0,root_unmatched:0});
assert.equal(gate('roots-native').stdout, gate('roots-memcheck').stdout); memcheck('roots-memcheck', true);
const header = 'kind,code,expected_degree,outcome,result_degree,coordinate_statuses,projective_statuses,root_statuses';
const blocked = new Set([7,8,15,16,17,19,20,21,22,23,24,25,26]);
for (const variant of ['observe', 'monic-observe']) {
  const result = csv(`${variant}-debug`, header);
  assert.equal(result.rows.length, 81);
  for (let i = 0; i < cases.length; i++) {
    const { kind,code,degree } = cases[i], row = result.rows[i];
    assert.deepEqual(row.slice(0,3).map(Number), [kind,code,degree]);
    if (kind === 2 && blocked.has(code)) {
      assert.deepEqual(row.slice(3), ['Unknown','-1','','','']); continue;
    }
    assert.deepEqual(row.slice(3,5), ['Known',String(degree)]);
    const coordinates = Array(degree+1).fill('Equal');
    if (variant === 'observe' && kind === 2 && [5,11,14].includes(code)) coordinates.fill('Unknown',1);
    assert.equal(row[5], coordinates.join(';'));
    assert.equal(row[6], Array(degree+1).fill('Equal').join(';'));
    const residuals = Array(degree).fill('Equal');
    if (kind === 2 && ([12,13,14].includes(code) || variant === 'observe' && [5,11].includes(code)))
      residuals.fill('Unknown', code === 12 ? 0 : 1);
    assert.equal(row[7], residuals.join(';'));
  }
  assert.deepEqual(result.summary, {suite:'polynomial-closure-observe',cases:81,known:68,blocked:13,degree_failures:0});
  for (const phase of ['release','memcheck']) assert.equal(gate(`${variant}-${phase}`).stdout, gate(`${variant}-debug`).stdout);
  assert(gate(`${variant}-release`).args.includes('--release'));
  assert(!gate(`${variant}-debug`).args.includes('--release'));
  memcheck(`${variant}-memcheck`, false);
}
const initial = gate('hyper-debug');
assert.match(initial.stderr, /kind=2, code=5/);
assert.equal(initial.stdout.trimEnd().split('\n').length, 60); // header plus 59 completed cases
assert(!initial.stdout.includes('"suite"')); // deliberately preserved incomplete failed assertion
function testNames(text) { return [...text.matchAll(/^test (.+) \.\.\. (ok|ignored|FAILED).*$/gm)].map(m => `${m[1]}:${m[2]}`).sort(); }
for (const profile of ['debug','release']) {
  const g = gate(`monic-tests-${profile}`), base = read(`results/retained-polynomial-facts-${profile}.stdout`);
  for (const flag of ['test','--offline','--locked','--all-features','--lib','--tests']) assert(g.args.includes(flag));
  assert.equal(g.args.includes('--release'), profile === 'release');
  assert.deepEqual(testNames(g.stdout), testNames(base)); assert.equal(testNames(g.stdout).length,800);
  const suites = [...g.stdout.matchAll(/test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;/g)];
  assert.equal(suites.length,7); assert.equal(suites.reduce((n,v) => n+Number(v[1]),0),800);
  assert(suites.every(v => v.slice(2).every(n => n === '0')));
}
assert.deepEqual(gate('monic-fmt').args,['fmt','--all','--','--check']);
console.log(JSON.stringify({ checkpoint:'polynomial roots and series', readFiles:34,readLines:4493,
  denseChecks:5880,sparseChecks:3024,nativeRootCases:81,hyperCasesPerVariantPerProfile:81,
  knownSquarefreeResults:68,unresolvedSquarefreeResults:13,monicImprovedCoordinateCases:3,
  monicImprovedRootChecks:2,monicSolverTestsPerProfile:800,gates:19,
  initialHarnessFailure:'Direct coefficient equality was stronger than the squarefree contract; preserved, not a Hyper mathematical defect.',
  status:manifest.status,limits:manifest.limits }));
