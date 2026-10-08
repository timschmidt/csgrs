import { readFileSync, writeFileSync, copyFileSync, constants, statSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here = dirname(fileURLToPath(import.meta.url)), workspace = resolve(here, '../../../..');
const json = p => JSON.parse(readFileSync(resolve(here,p),'utf8'));
const sha = p => createHash('sha256').update(readFileSync(resolve(here,p))).digest('hex');
const draft = process.argv.includes('--draft-monic-state');
const manifest = {schema:1, recorded:new Date().toISOString(), files:{}, binaries:{}, gates:[], reads:[], candidateSources:{},
  status:'Monic state/size and complete polynomial-directory read checkpoint; candidate not retained, downstream still running; full ecosystem incomplete.',
  limits:'State probes are finite repeated recipes, not exhaustive concurrency or cancellation proof. Aborted numeric observations are discarded. Squarefree output is only promised up to nonzero scale. Both state probes and a std-only control report the same 48-byte possible loss in Rust thread initialization; no suppression or zero-error claim. Native powers use independent polynomial recurrences but share the scalar backend. Example file sizes include build/layout effects and are not whole-application or pure-code-size measurements. WASM is compile-only; ongoing Hypercurve output is not bound by this checkpoint.'};
const files = ['bind-monic-state.mjs','verify-monic-state.mjs','capture.mjs','prepare-monic-qualified.mjs',
  'prepare-monic-consumers.mjs','monic-state-probe.rs','monic-thread-control.rs','measure-monic-app-size.mjs',
  'monic-state-read-selection.json','flint-polynomial-full-power-probe.c','flint-polynomial-series-probe.c',
  'monic-cost-experiment.json','monic-app-size-summary.json','polynomial-facts-app-size-summary.json'];
for (const pkg of ['monic-state-baseline','monic-state-trial'])
  for (const file of ['Cargo.toml','Cargo.lock']) files.push(`${pkg}/${file}`);
const tags = ['monic-qualified-focused-debug','monic-qualified-tests-debug','monic-qualified-tests-release'];
for (const variant of ['baseline','trial']) for (const phase of ['debug','release','memcheck','memcheck-novgdb'])
  tags.push(`monic-state-${variant}-${phase}`);
tags.push('monic-qualified-clippy','monic-qualified-fmt','monic-qualified-wasm','monic-thread-control-compile',
  'monic-thread-control-memcheck','monic-polynomial-donor-tests','monic-full-power-compile',
  'monic-full-power-native','monic-full-power-memcheck','monic-app-size-run','monic-app-build');
for (const example of ['basic','arrangement']) for (const operation of ['strip','size','run'])
  tags.push(`monic-app-${operation}-${example}`);
for (const tag of tags) {
  const g = json(`results/${tag}.json`);
  const expected = /^(monic-state-.+-memcheck(?:-novgdb)?|monic-thread-control-memcheck)$/.test(tag) ? 97 : 0;
  assert.equal(g.code,expected,tag); assert.equal(g.signal,null); manifest.gates.push({tag,code:expected});
  for (const suffix of ['json','stdout','stderr']) files.push(`results/${tag}.${suffix}`);
}
for (const p of files) manifest.files[p] = sha(p);
const coverage = json('coverage.json');
manifest.reads = json('monic-state-read-selection.json').map(e => {
  const entry = coverage.find(c => c.repo === e.repo && c.path === e.path); assert(entry); return entry;
});
const source = 'polynomial-monic-qualified-trial';
for (const p of Object.keys(json('retained-polynomial-facts.json').liveSources).filter(p => !p.startsWith('hyperreal/')))
  manifest.candidateSources[p] = sha(`${source}/${p}`);
manifest.candidateSources['hypersolve/src/root_isolation_monic_tests.rs'] = sha(`${source}/hypersolve/src/root_isolation_monic_tests.rs`);
assert.equal(Object.keys(manifest.candidateSources).length,774);
const binaryRoot = '/tmp/calcium-monic-state.rk70Kr';
for (const variant of ['baseline','trial']) for (const profile of ['debug','release']) {
  const original = `/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse/${profile}/calcium-monic-state-${variant}`;
  const path = draft ? original : `${binaryRoot}/state-${variant}-${profile}`;
  if (!draft) copyFileSync(original,path,constants.COPYFILE_EXCL | constants.COPYFILE_FICLONE);
  assert.equal(sha(path),sha(original)); manifest.binaries[path] = sha(path);
}
for (const name of ['std-thread-control','flint-full-power']) manifest.binaries[`${binaryRoot}/${name}`] = sha(`${binaryRoot}/${name}`);
const donorOriginal = resolve(workspace,'exact-real-references/flint/build/ca_poly/test/main');
const donorPath = draft ? donorOriginal : `${binaryRoot}/flint-ca-poly-tests`;
if (!draft) copyFileSync(donorOriginal,donorPath,constants.COPYFILE_EXCL | constants.COPYFILE_FICLONE);
assert.equal(sha(donorPath),sha(donorOriginal)); manifest.binaries[donorPath] = sha(donorPath);
for (const a of json('monic-app-size-summary.json').artifacts) for (const file of a.monic) {
  assert.equal(sha(file.path),file.sha256); manifest.binaries[file.path] = file.sha256;
}
if (!draft) writeFileSync(resolve(here,'monic-state-experiment.json'),JSON.stringify(manifest,null,2)+'\n',{flag:'wx'});
console.log(JSON.stringify({checkpoint:'monic state binding',draft,files:files.length,gates:tags.length,
  reads:manifest.reads.length,sourceFiles:Object.keys(manifest.candidateSources).length,
  binaries:Object.keys(manifest.binaries).length,binaryBytes:Object.keys(manifest.binaries).reduce((n,p)=>n+statSync(p).size,0)}));
export {manifest};
