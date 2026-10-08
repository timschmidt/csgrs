// Mechanically preserve this checkpoint; never overwrite prior evidence.
import { readFileSync, writeFileSync, copyFileSync, constants, statSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here = dirname(fileURLToPath(import.meta.url));
const json = p => JSON.parse(readFileSync(resolve(here, p), 'utf8'));
const sha = p => createHash('sha256').update(readFileSync(resolve(here, p))).digest('hex');
const draft = process.argv.includes('--draft');
const manifest = { schema: 1, recorded: new Date().toISOString(), files: {}, binaries: {},
  candidateSources: {}, gates: [], reads: [],
  status: 'Polynomial source/probe checkpoint; isolated monic candidate, NOT retained. Full ecosystem incomplete.',
  limits: 'Required-True finite polynomial controls are independent recurrences, not independent scalar implementations or branch-coverage proof. Hyper squarefree output is compared up to nonzero scale; initial direct-coefficient assertion was too strong. Monic candidate has no CPU/allocation/application-size, state/serde, downstream or WASM qualification yet.' };
const files = ['capture.mjs', 'prepare-polynomial-monic.mjs', 'bind-polynomial-closure.mjs',
  'verify-polynomial-closure.mjs', 'polynomial-closure-read-selection.json',
  'flint-polynomial-series-probe.c', 'flint-polynomial-sparse-series-probe.c',
  'flint-polynomial-roots-probe.c', 'polynomial-closure-hyper.rs', 'polynomial-closure-observe.rs',
  'retained-polynomial-facts.json'];
for (const packageName of ['polynomial-closure-hyper', 'polynomial-closure-observe', 'polynomial-monic-observe'])
  for (const name of ['Cargo.toml', 'Cargo.lock']) files.push(`${packageName}/${name}`);
const phases = ['series-compile', 'series-native', 'series-memcheck', 'sparse-compile',
  'sparse-native', 'sparse-memcheck', 'roots-compile', 'roots-native', 'roots-memcheck',
  'hyper-debug', 'observe-debug', 'observe-release', 'observe-memcheck',
  'monic-observe-debug', 'monic-observe-release', 'monic-observe-memcheck',
  'monic-tests-debug', 'monic-tests-release', 'monic-fmt'];
for (const phase of phases) {
  const tag = `polynomial-closure-${phase}`, gate = json(`results/${tag}.json`);
  assert.equal(gate.code, phase === 'hyper-debug' ? 101 : 0, tag);
  assert.equal(gate.signal, null); manifest.gates.push(tag);
  for (const suffix of ['json', 'stdout', 'stderr']) files.push(`results/${tag}.${suffix}`);
}
for (const path of files) manifest.files[path] = sha(path);
const baseline = json('retained-polynomial-facts.json');
const changed = [];
for (const [path, hash] of Object.entries(baseline.liveSources)) {
  if (!['hyperlattice', 'hyperlimit', 'hypersolve'].includes(path.split('/')[0])) continue;
  const actual = sha(`polynomial-monic-trial/${path}`); manifest.candidateSources[path] = actual;
  if (actual !== hash) changed.push(path);
}
assert.equal(Object.keys(manifest.candidateSources).length, 339);
assert.deepEqual(changed, ['hypersolve/src/root_isolation.rs']);
manifest.changed = changed;
const coverage = json('coverage.json');
for (const entry of json('polynomial-closure-read-selection.json')) {
  const full = coverage.find(v => v.repo === entry.repo && v.path === entry.path);
  assert(full); manifest.reads.push(full);
}
assert.equal(manifest.reads.length, 34);
const binaryRoot = '/tmp/calcium-polynomial-closure.L3hLNa';
const target = '/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse';
for (const [variant, packageName, profiles] of [
  ['initial-assertion', 'calcium-polynomial-closure-hyper', ['debug']],
  ['baseline-observe', 'calcium-polynomial-closure-observe', ['debug', 'release']],
  ['monic-observe', 'calcium-polynomial-monic-observe', ['debug', 'release']],
]) for (const profile of profiles) {
  const source = `${target}/${profile}/${packageName}`;
  const path = draft ? source : `${binaryRoot}/${variant}-${profile}`;
  const hash = sha(source);
  if (!draft) copyFileSync(source, path, constants.COPYFILE_EXCL | constants.COPYFILE_FICLONE);
  assert.equal(sha(path), hash); manifest.binaries[path] = hash;
}
for (const name of ['series', 'sparse-series', 'roots']) {
  const path = `${binaryRoot}/flint-polynomial-${name}-probe`; manifest.binaries[path] = sha(path);
}
// Bind the already-built native donor as well, without another large copy.
const nativeLibrary = resolve(here, '../../../../exact-real-references/flint/libflint.so');
manifest.nativeLibrary = { path: nativeLibrary, sha256: sha(nativeLibrary) };
if (!draft) writeFileSync(resolve(here, 'polynomial-closure-experiment.json'), JSON.stringify(manifest, null, 2) + '\n', { flag: 'wx' });
console.log(JSON.stringify({ checkpoint: 'polynomial roots and series', files: files.length,
  gates: manifest.gates.length, readFiles: manifest.reads.length,
  readLines: manifest.reads.reduce((s,v) => s + v.ranges.reduce((n,[a,b]) => n+b-a+1,0),0),
  candidateFiles: 339, changed, binaries: Object.keys(manifest.binaries).length,
  preservedBinaryBytes: Object.keys(manifest.binaries).reduce((n,p) => n + statSync(p).size,0), draft, status: manifest.status }));
export { manifest };
