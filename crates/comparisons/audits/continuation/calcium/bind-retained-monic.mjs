import { readFileSync, writeFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here = dirname(fileURLToPath(import.meta.url)), workspace = resolve(here, '../../../..');
const json = p => JSON.parse(readFileSync(resolve(here, p), 'utf8'));
const sha = p => createHash('sha256').update(readFileSync(resolve(here, p))).digest('hex');
const draft = process.argv.includes('--draft-retained-monic');
const files = {};
for (const path of ['matrix-pivot-experiment.json', 'monic-state-experiment.json',
  'retained-polynomial-facts.json', 'retained-exp-proof.json', 'capture.mjs',
  'bind-retained-monic.mjs', 'verify-retained-monic.mjs']) files[path] = sha(path);
const gates = ['debug', 'release', 'clippy', 'fmt', 'wasm', 'metadata'];
for (const gate of gates) {
  const tag = `retained-monic-${gate}`, r = json(`results/${tag}.json`);
  assert.equal(r.code, 0, tag); assert.equal(r.signal, null);
  assert.equal(r.cwd, resolve(workspace, 'hypersolve'));
  for (const ext of ['json', 'stdout', 'stderr']) {
    const path = `results/${tag}.${ext}`; files[path] = sha(path);
  }
}
const frozenSnapshot = 'polynomial-monic-qualified-trial';
const liveSources = { ...json('monic-state-experiment.json').candidateSources };
assert.equal(Object.keys(liveSources).length, 774);
for (const [path, hash] of Object.entries(json('retained-exp-proof.json').liveSourceHashes))
  liveSources[`hyperreal/${path}`] = hash;
assert.equal(Object.keys(liveSources).length, 954);
const previous = json('retained-polynomial-facts.json').liveSources;
assert.equal(Object.keys(previous).length, 953);
for (const path of Object.keys(previous)) assert(path in liveSources, path);
const changed = [];
for (const [path, hash] of Object.entries(liveSources)) {
  assert.equal(sha(`${frozenSnapshot}/${path}`), hash, `snapshot ${path}`);
  assert.equal(sha(resolve(workspace, path)), hash, `live ${path}`);
  if (hash !== previous[path]) changed.push(path);
}
changed.sort();
assert.deepEqual(changed, ['hypersolve/src/root_isolation.rs', 'hypersolve/src/root_isolation_monic_tests.rs']);
const manifest = { schema: 1, recorded: new Date().toISOString(),
  status: 'Certified exact-one monic normalization retained in live Hypersolve; matrix algorithm probes remain isolated and the full ecosystem audit remains incomplete.',
  files, gates, liveSources, changed, frozenSnapshot,
  acceptedCosts: 'Completeness-first retention: three coefficient-comparison cases and two root checks become provable without losing the 68-known/13-unresolved result split. CPU paired medians span 0.9007–1.0667 of baseline, including slower controls. Requested allocations/bytes fall in 56 of 162 groups, never increase in the measured corpus, and incremental peaks fall in 44 groups. Two stripped Hypercurve examples each grow 2,944 bytes including build-path/layout effects. No public API, dependency, scalar representation or cache-field change. Production root_isolation.rs delta is 14 added/six removed lines including test-module inclusion, plus a new 101-line file containing three tests.',
  limits: '803 live solver tests in each profile; exact qualified test membership and all 954 live source/support hashes match the frozen scalar/consumer snapshot. Identical qualified downstream Hypercurve has 1,761 passed/nine pre-existing ignored across 45 suites; not rerun on the live path. State qualification covers 1,932 queries per variant/profile; both variants and a std-only thread control have the same unsuppressed 48-byte possible-loss Memcheck report, not clean memory gates. CPU 7,776 and allocation 972 observations, 108 bounded churn observations, and representative example sizes are scoped host/corpus evidence, not universal bounds, peak RSS, full CI or whole-application size. Live Clippy, formatting and WASM compile-only gates pass. Historical proofs remain immutable; --monic-live checks current live bytes independently of older --retained-live snapshots.' };
if (!draft) writeFileSync(resolve(here, 'retained-monic.json'), JSON.stringify(manifest, null, 2) + '\n', { flag: 'wx' });
console.log(JSON.stringify({ checkpoint: 'retained monic binding', draft, sourceFiles: 954,
  changed, gates: gates.length, boundFiles: Object.keys(files).length }));
export { manifest };
