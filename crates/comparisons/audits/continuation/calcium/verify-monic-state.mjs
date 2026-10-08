import './verify-monic-costs.mjs';
import { readFileSync, readdirSync, realpathSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here = dirname(fileURLToPath(import.meta.url)), workspace = resolve(here,'../../../..');
const read = p => readFileSync(resolve(here,p),'utf8'), json = p => JSON.parse(read(p));
const sha = p => createHash('sha256').update(readFileSync(resolve(here,p))).digest('hex');
const manifest = process.argv.includes('--draft-monic-state') ? (await import('./bind-monic-state.mjs')).manifest : json('monic-state-experiment.json');
for (const [p,h] of Object.entries(manifest.files)) assert.equal(sha(p),h,p);
for (const [p,h] of Object.entries(manifest.binaries)) assert.equal(sha(p),h,p);
assert.equal(Object.keys(manifest.binaries).length,11);
const snapshot = 'polynomial-monic-qualified-trial', retained = json('retained-polynomial-facts.json');
const paths = [];
function walk(relative) {
  for (const e of readdirSync(resolve(here,snapshot,relative),{withFileTypes:true})) {
    const p = relative ? `${relative}/${e.name}` : e.name;
    if (e.isSymbolicLink()) {
      assert.equal(p,'hyperreal');
      assert.equal(realpathSync(resolve(here,snapshot,p)),realpathSync(resolve(here,'root-exp-reuse-trial-hyperreal')));
    } else if (e.isDirectory()) walk(p);
    else { assert(e.isFile()); paths.push(p); }
  }
}
walk(''); assert.deepEqual(paths.sort(),Object.keys(manifest.candidateSources).sort());
assert.equal(paths.length,774);
const changed = [];
for (const [p,h] of Object.entries(manifest.candidateSources)) {
  assert.equal(sha(`${snapshot}/${p}`),h,p);
  if (h !== retained.liveSources[p]) changed.push(p);
}
assert.deepEqual(changed.sort(),['hypersolve/src/root_isolation.rs','hypersolve/src/root_isolation_monic_tests.rs']);
const testModule = '#[cfg(test)]\n#[path = "root_isolation_monic_tests.rs"]\nmod monic_tests;\n\n';
const qualified = read(`${snapshot}/hypersolve/src/root_isolation.rs`);
assert.equal(qualified.split(testModule).length,2);
assert.equal(qualified.replace(testModule,''),read('polynomial-monic-trial/hypersolve/src/root_isolation.rs'));
const coverage = json('coverage.json'), inventory = json('inventory.json'), selection = json('monic-state-read-selection.json');
assert.equal(selection.length,8); assert.equal(new Set(selection.map(v=>`${v.repo}:${v.path}`)).size,8);
assert.deepEqual(manifest.reads,selection.map(e=>coverage.find(c=>c.repo===e.repo&&c.path===e.path)));
assert.equal(manifest.reads.reduce((n,v)=>n+v.ranges.reduce((s,[a,b])=>s+b-a+1,0),0),579);
for (const v of manifest.reads) {
  const f = inventory.sources.find(s=>s.repo===v.repo).files.find(f=>f.path===v.path);
  assert.deepEqual(v.ranges,[[1,f.lines]]); assert.equal(sha(resolve(workspace,'exact-real-references',v.repo,v.path)),f.sha256);
}
for (const [repo,count,lines] of [['calcium',74,6371],['flint',70,5571]]) {
  const files = inventory.sources.find(s=>s.repo===repo).files.filter(f=>/^(src\/)?ca_poly\//.test(f.path));
  assert.equal(files.length,count); assert.equal(files.reduce((n,f)=>n+f.lines,0),lines);
  for (const f of files) assert.deepEqual(coverage.find(c=>c.repo===repo&&c.path===f.path)?.ranges,[[1,f.lines]]);
}
assert.equal(manifest.gates.length,28);
assert.equal(new Set(manifest.gates.map(g=>g.tag)).size,28);
const gates = new Map();
for (const {tag,code} of manifest.gates) {
  const g = json(`results/${tag}.json`); assert.equal(g.tag,tag); assert.equal(g.code,code,tag); assert.equal(g.signal,null);
  assert(Date.parse(g.finished)>=Date.parse(g.started));
  assert.equal(code,/^(monic-state-.+-memcheck(?:-novgdb)?|monic-thread-control-memcheck)$/.test(tag)?97:0);
  gates.set(tag,{...g,stdout:read(`results/${tag}.stdout`),stderr:read(`results/${tag}.stderr`)});
}
function gate(tag) { const g = gates.get(tag); assert(g,tag); return g; }
function hasFlags(tag,flags) { const g = gate(tag); for (const flag of flags) assert(g.args.includes(flag),`${tag}: ${flag}`); return g; }
const testNames = text => [...text.matchAll(/^test (.+) \.\.\. (ok|ignored|FAILED).*$/gm)].map(m=>`${m[1]}:${m[2]}`).sort();
const focused = [
  'monic_normalization_keeps_scalar_and_failed_reciprocal_boundaries',
  'monic_normalization_preserves_lower_coefficients_and_certified_unit',
  'square_free_log_factors_do_not_rebuild_an_opaque_leading_unit',
].map(n=>`root_isolation::monic_tests::${n}:ok`).sort();
assert.deepEqual(testNames(gate('monic-qualified-focused-debug').stdout),focused);
for (const profile of ['debug','release']) {
  const tag = `monic-qualified-tests-${profile}`, g = hasFlags(tag,['test','--offline','--locked','--all-features']);
  assert.equal(g.cwd,resolve(here,snapshot,'hypersolve')); assert.equal(g.args.includes('--release'),profile==='release');
  const original = testNames(read(`results/polynomial-closure-monic-tests-${profile}.stdout`));
  assert.equal(original.length,800); assert.deepEqual(testNames(g.stdout),original.concat(focused).sort());
  const suites = [...g.stdout.matchAll(/test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;/g)];
  assert.equal(suites.length,8); assert.equal(suites.reduce((n,v)=>n+Number(v[1]),0),803);
  assert(suites.every(v=>v.slice(2).every(n=>n==='0')));
}
hasFlags('monic-qualified-clippy',['clippy','--offline','--locked','--all-features','--all-targets','-D','warnings']);
assert.deepEqual(gate('monic-qualified-fmt').args,['fmt','--','--check']);
hasFlags('monic-qualified-wasm',['check','--offline','--locked','--all-features','--lib','--target','wasm32-unknown-unknown']);
const uncertain = new Set([7,8,15,16,17,19,20,21,22,23,24,25,26]);
const states = ['cold','input-roundtrip','warm-32','warm-128','warm-512','after-abort',
  ...[0,1,2,3].flatMap(w=>[0,1,2,3].map(i=>`worker${w}-${i}`))];
const recipes = [];
for (let kind=0;kind<3;kind++) for (let code=0;code<27;code++) for (const state of states) recipes.push({kind,code,state});
for (let kind=0;kind<2;kind++) for (const repeat of [3,4,8]) for (const code of [2,5,11,14,26])
  for (let scale=0;scale<5;scale++) recipes.push({kind,code,state:`expanded${repeat}-scale${scale}`});
assert.equal(recipes.length,1932);
const totals = {};
for (const variant of ['baseline','trial']) {
  const g = gate(`monic-state-${variant}-debug`);
  for (const phase of ['release','memcheck','memcheck-novgdb']) assert.equal(g.stdout,gate(`monic-state-${variant}-${phase}`).stdout);
  hasFlags(`monic-state-${variant}-release`,['run','--offline','--locked','--release']);
  const rows = g.stdout.trimEnd().split('\n').map(JSON.parse);
  assert.deepEqual(rows.pop(),{queries:1932,suite:'monic-state',unchanged_inputs:81,workers_per_case:4});
  assert.equal(rows.length,recipes.length);
  let known=0, coordinateUnknown=0, residualUnknown=0, projectiveEqual=0;
  rows.forEach((r,i)=>{
    const {kind,code,state} = recipes[i]; assert.deepEqual({kind:r.kind,code:r.code,state:r.state},{kind,code,state});
    if (kind===2&&uncertain.has(code)) { assert.deepEqual(r.result,{outcome:'Unknown'}); return; }
    known++;
    const digits = [code%3,Math.floor(code/3)%3,Math.floor(code/9)%3], roots = [0,1,2].filter(i=>digits[i]!==0), degree=roots.length;
    const coordinate = Array.from({length:degree+1},(_,j)=>variant==='baseline'&&kind===2&&[5,11,14].includes(code)&&j>0?'Unknown':'Equal');
    const residual = roots.map(j=>kind===2&&j!==0&&([12,13,14].includes(code)||(variant==='baseline'&&[5,11].includes(code)))?'Unknown':'Equal');
    assert.deepEqual(r.result,{coordinate,degree,outcome:'Known',output_roundtrip:Array(degree+1).fill('Equal'),
      projective:Array(degree+1).fill('Equal'),residual});
    coordinateUnknown += coordinate.filter(s=>s==='Unknown').length;
    residualUnknown += residual.filter(s=>s==='Unknown').length; projectiveEqual += degree+1;
  });
  totals[variant]={known,unknown:rows.length-known,coordinateUnknown,residualUnknown,projectiveEqual};
  assert.deepEqual(totals[variant],{known:1646,unknown:286,coordinateUnknown:variant==='baseline'?154:0,
    residualUnknown:variant==='baseline'?176:132,projectiveEqual:4836});
}
for (const tag of ['monic-thread-control-memcheck',...['baseline','trial'].flatMap(v=>[`monic-state-${v}-memcheck`,`monic-state-${v}-memcheck-novgdb`])]) {
  const g = hasFlags(tag,['--tool=memcheck','--leak-check=full','--show-leak-kinds=all','--errors-for-leak-kinds=definite,indirect,possible','--error-exitcode=97']);
  assert.equal(g.command,'valgrind');
  for (const kind of ['definitely','indirectly']) assert.match(g.stderr,new RegExp(`${kind} lost: 0 bytes in 0 blocks`));
  assert.match(g.stderr,/possibly lost: 48 bytes in 1 blocks/);
  assert.match(g.stderr,/ERROR SUMMARY: 1 errors from 1 contexts \(suppressed: 0 from 0\)/);
  assert.match(g.stderr,/48 bytes in 1 blocks are possibly lost[\s\S]*std::thread::thread::Thread[\s\S]*std::thread::current::init_current/);
  assert(!/Invalid (read|write|free)|uninitialised value|uninitialized value/.test(g.stderr));
  if (tag.includes('novgdb')||tag==='monic-thread-control-memcheck') {
    assert(g.args.includes('--vgdb=no')); assert(!g.stderr.includes('could not unlink'));
  }
}
assert.deepEqual(JSON.parse(gate('monic-thread-control-memcheck').stdout),{suite:'std-thread-only-control',scopes:81,workers_per_scope:4,checksum:810});
assert(Date.parse(gate('monic-state-baseline-memcheck-novgdb').finished)<=Date.parse(gate('monic-state-trial-memcheck-novgdb').started));
const donor = gate('monic-polynomial-donor-tests');
assert.equal(donor.command,'make'); assert.deepEqual(donor.args,['-j2','check','MOD=ca_poly']);
const registered = [...read(resolve(workspace,'exact-real-references/flint/src/ca_poly/test/main.c')).matchAll(/TEST_FUNCTION\(([^)]+)\)/g)].map(m=>m[1]).sort();
const passed = [...donor.stdout.replace(/\x1b\[[0-9;]*m/g,'').matchAll(/^(ca_poly_\w+)\s+[\d.]+\s+\(PASS\)/gm)].map(m=>m[1]).sort();
assert.equal(registered.length,15); assert.deepEqual(passed,registered);
const native = gate('monic-full-power-native').stdout.trimEnd().split('\n');
assert.equal(native.shift(),'mode,length,constant,exponent,checks,failures');
assert.deepEqual(JSON.parse(native.pop()),{suite:'full-polynomial-powers',cases:540,checks:1080,failures:0});
assert.deepEqual(native.map(r=>r.split(',').map(Number)),[0,1,2,3].flatMap(m=>[0,1,2,4,8].flatMap(l=>[-1,0,1].flatMap(c=>[0,1,2,3,4,5,6,7,8].map(e=>[m,l,c,e,2,0])))));
const nativeMemory = gate('monic-full-power-memcheck');
assert.equal(nativeMemory.stdout,gate('monic-full-power-native').stdout);
assert.match(nativeMemory.stderr,/ERROR SUMMARY: 0 errors from 0 contexts/);
assert.match(nativeMemory.stderr,/in use at exit: 0 bytes in 0 blocks/);
const app = json('monic-app-size-summary.json'); assert.deepEqual(app.artifacts.map(a=>a.example),['basic','arrangement']);
const appDeltas = [];
for (const artifact of app.artifacts) {
  const old = json('polynomial-facts-app-size-summary.json').artifacts.find(a=>a.example===artifact.example);
  assert.deepEqual(artifact.baseline,old.facts);
  for (const file of [...artifact.baseline,...artifact.monic]) { assert.equal(sha(file.path),file.sha256); assert.equal(readFileSync(file.path).length,file.bytes); }
  const e = artifact.example;
  assert.equal(gate(`monic-app-run-${e}`).command,artifact.monic[1].path);
  assert.deepEqual(gate(`monic-app-run-${e}`).args,[]);
  assert.equal(gate(`monic-app-run-${e}`).stderr,''); // examples are assertion-only and intentionally silent
  assert.equal(gate(`monic-app-run-${e}`).stdout,'');
  const parseSize = text => text.trimEnd().split('\n').slice(1).map(line=>line.trim().split(/\s+/).slice(0,4).map(Number));
  const sizes = parseSize(gate(`monic-app-size-${e}`).stdout), base = parseSize(read(`results/polynomial-facts-app-size-${e}.stdout`));
  assert.equal(sizes.length,2); assert.deepEqual(sizes[0],sizes[1]); assert.deepEqual(base[0],base[1]);
  const delta = sizes[0].map((n,i)=>n-base[0][i]);
  assert.deepEqual(delta,e==='basic'?[3008,-64,-2944,0]:[3008,-64,1152,4096]);
  const stripped = artifact.monic[1].bytes-artifact.baseline[1].bytes; assert.equal(stripped,2944);
  appDeltas.push({example:e,strippedFileBytes:stripped,text:delta[0],data:delta[1],bss:delta[2],loadedTotal:delta[3]});
}
console.log(JSON.stringify({checkpoint:'monic state/size and complete polynomial source directories',readFiles:8,readLines:579,
  completePolynomialFiles:{calcium:74,flint:70},solverTestsPerProfile:803,stateQueriesPerVariantPerProfile:1932,
  stateResults:totals,memcheck:'same 48-byte std thread possible loss in baseline/trial/control; no definite/indirect loss',
  donorTests:15,fullPowerChecks:1080,applicationDeltas:appDeltas,status:manifest.status,limits:manifest.limits}));
