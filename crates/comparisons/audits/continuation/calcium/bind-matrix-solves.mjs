import { readFileSync, writeFileSync, copyFileSync, statSync, constants } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here = dirname(fileURLToPath(import.meta.url)), workspace = resolve(here, '../../../..');
const json = p => JSON.parse(readFileSync(resolve(here, p), 'utf8'));
const sha = p => createHash('sha256').update(readFileSync(resolve(here, p))).digest('hex');
const draft = process.argv.includes('--draft-matrix-solves');
const manifest = { schema: 1, recorded: new Date().toISOString(), files: {}, gates: [], reads: [],
  candidateSources: {}, binaries: {},
  status: 'Matrix solve/rank source and numerical-output checkpoint; rank-dominance candidate remains isolated. No production or donor change; full ecosystem audit incomplete.',
  limits: 'Native valid-input controls report 153 incorrect output rows and four unresolved rows, with clean Memcheck; repeated shapes/fields/routes are not distinct defects. In-place discrepancies are qualified separately from documentation that does not state a blanket alias promise. Archived mechanisms were independently read, not executed. Matrix oracles use closed-form integer coefficients and the same scalar backend. Rank capability has 828 queries per variant/profile, 204 newly certified and 18 preserved unresolved controls, not 204 independent identities. Candidate has existing 803 tests/profile and formatting, but no focused in-crate regressions, matched CPU/allocation/size, broader state/serde/concurrency, downstream, Clippy or WASM qualification. Continuing all same-order minors may be costly on unresolved inputs; no retention or speed/memory claim.' };
const files = ['bind-matrix-solves.mjs', 'verify-matrix-solves.mjs', 'capture.mjs',
  'record-matrix-solve-reads.mjs', 'matrix-solve-read-selection.json', 'prepare-rank-dominance.mjs',
  'flint-matrix-output-probe.c', 'rank-dominance-probe.rs', 'retained-monic.json', 'matrix-pivot-experiment.json'];
for (const variant of ['baseline', 'candidate']) for (const file of ['Cargo.toml', 'Cargo.lock'])
  files.push(`rank-dominance-${variant}/${file}`);
const tags = ['matrix-output-native-compile', 'matrix-output-native', 'matrix-output-memcheck',
  'rank-dominance-baseline-debug', 'rank-dominance-candidate-debug', 'rank-dominance-baseline-release',
  'rank-dominance-candidate-release', 'rank-dominance-baseline-memcheck', 'rank-dominance-candidate-memcheck',
  'rank-dominance-tests-debug', 'rank-dominance-tests-release', 'rank-dominance-fmt'];
for (const tag of tags) {
  const g = json(`results/${tag}.json`);
  assert.equal(g.code, ['matrix-output-native', 'matrix-output-memcheck'].includes(tag) ? 1 : 0, tag);
  assert.equal(g.signal, null); manifest.gates.push(tag);
  for (const ext of ['json', 'stdout', 'stderr']) files.push(`results/${tag}.${ext}`);
}
for (const p of files) manifest.files[p] = sha(p);
const coverage = json('coverage.json');
manifest.reads = json('matrix-solve-read-selection.json').map(e => {
  const v = coverage.find(c => c.repo === e.repo && c.path === e.path); assert(v); return v;
});
assert.equal(manifest.reads.length, 51);
const retained = json('retained-monic.json'), changed = [];
for (const [path, hash] of Object.entries(retained.liveSources)) {
  assert.equal(sha(resolve(workspace, path)), hash, `unchanged live ${path}`);
  const actual = sha(`rank-dominance-trial/${path}`);
  if (path.startsWith('hyperreal/')) { assert.equal(actual, hash); continue; }
  manifest.candidateSources[path] = actual;
  if (actual !== hash) changed.push(path);
}
assert.equal(Object.keys(manifest.candidateSources).length, 774);
assert.deepEqual(changed, ['hypersolve/src/rank.rs']); manifest.changed = changed;
manifest.hyperReadRanges = { 'hypersolve/src/rank.rs': [[1, 241]],
  'hypersolve/src/bareiss.rs': [[443, 884]], 'hyperlattice/src/matrix/inverse.rs': [[1, 6]],
  'hypersolve/src/model.rs': [[1, 85]], 'hypersolve/tests/smoke.rs': [[10958, 10995], [11555, 11635]] };
manifest.nativeLibrary = { path: resolve(workspace, 'exact-real-references/flint/libflint.so'),
  sha256: sha(resolve(workspace, 'exact-real-references/flint/libflint.so')) };
const root = '/tmp/calcium-matrix-solves.Pl5RBm';
manifest.binaries[`${root}/flint-matrix-output`] = sha(`${root}/flint-matrix-output`);
for (const variant of ['baseline', 'candidate']) for (const profile of ['debug', 'release']) {
  const original = `/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse/${profile}/calcium-rank-dominance-${variant}`;
  const path = draft ? original : `${root}/rank-dominance-${variant}-${profile}`;
  if (!draft) copyFileSync(original, path, constants.COPYFILE_EXCL | constants.COPYFILE_FICLONE);
  assert.equal(sha(path), sha(original)); manifest.binaries[path] = sha(path);
}
if (!draft) writeFileSync(resolve(here, 'matrix-solve-experiment.json'), JSON.stringify(manifest, null, 2) + '\n', { flag: 'wx' });
console.log(JSON.stringify({ checkpoint: 'matrix solve binding', draft, files: files.length,
  gates: tags.length, reads: manifest.reads.length, candidateSourceFiles: 774, changed,
  binaries: Object.keys(manifest.binaries).length,
  binaryBytes: Object.keys(manifest.binaries).reduce((n, p) => n + statSync(p).size, 0) }));
export { manifest };
