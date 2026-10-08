import { readFileSync, writeFileSync, existsSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here = dirname(fileURLToPath(import.meta.url)), workspace = resolve(here, '../../../..');
const read = p => readFileSync(resolve(here, p));
const json = p => JSON.parse(read(p));
const hash = p => createHash('sha256').update(read(p)).digest('hex');
const baseline = json('baseline-hyperreal.json');
const changed = ['src/computable/node.rs', 'src/computable/node/structural_analysis.rs'];
const added = ['exp_relation.rs', 'exp_relation_tests.rs', 'exp_relation_reuse.rs', 'exp_relation_reuse_tests.rs']
  .map(f => `src/computable/node/${f}`);
const retained = [...baseline.files.map(f => f.path), ...added];
const liveSourceHashes = {};
for (const p of retained) {
  const live = resolve(workspace, 'hyperreal', p), frozen = `root-exp-reuse-trial-hyperreal/${p}`;
  assert(existsSync(live)); assert.equal(hash(live), hash(frozen), p); liveSourceHashes[p] = hash(live);
  const previous = baseline.files.find(f => f.path === p);
  if (previous && !changed.includes(p)) assert.equal(hash(live), previous.sha256, `user file changed: ${p}`);
}
const tags = ['debug', 'release', 'clippy', 'fmt', 'wasm', 'metadata'].map(s => `retained-exp-proof-${s}`);
for (const tag of tags) { const r = json(`results/${tag}.json`); assert.equal(r.code, 0, tag); assert.equal(r.signal, null, tag); }
const sources = ['reuse-cost-experiment.json', 'baseline-hyperreal.json', 'bind-retained-exp-proof.mjs', 'verify-retained-exp-proof.mjs'];
const evidence = tags.flatMap(t => ['json', 'stdout', 'stderr'].map(e => `results/${t}.${e}`));
const manifest = { schema: 1, recorded: new Date().toISOString(), status: 'Retained in live Hyperreal; full ecosystem audit remains incomplete.',
  sourceHashes: Object.fromEntries(sources.map(p => [p, hash(p)])),
  evidenceHashes: Object.fromEntries(evidence.map(p => [p, hash(p)])),
  liveSourceHashes, frozenSourceSnapshot: 'root-exp-reuse-trial-hyperreal',
  baselineSnapshot: 'baseline-hyperreal.json', changed, added, unchangedBaselineFiles: 174,
  libraryTestsPerProfile: 780, integrationTestsPerProfile: 75, suitesPerProfile: 14,
  rationale: 'Exactness/completeness first: positive logarithmic relation certificates prove additional equalities and nonzero signs without replacing numeric nodes, serialized data or valid approximation caches. Weak-key structural reuse keeps those rules and improves repeated deep comparisons. Existing user-worktree edits remain byte-preserved outside the two exact qualified deltas. No dependency/public API/version change.',
  acceptedCosts: 'Some unresolved queries are slower than baseline; deep independently rebuilt first queries are about3.77x baseline Unknown, while repeated deep comparisons are about0.284x v2. Numeric workload median ratios range0.927–1.048 baseline; same allocation totals as v2. Host weak retention up to2304 requested heap bytes per touched worker plus400 linked static-TLS bytes/thread; tested stripped Hypercurve examples each grow8656 bytes over baseline. Source addition234 non-test-file lines/9088 bytes, including test include directives. No universal CPU/peak-memory bound.',
  limits: '855 library/integration tests per profile, all-feature/all-target Clippy with warnings denied, formatting and all-feature wasm library compilation pass on live source. Prior independent MPFR, state, consumer, memory and benchmark gates bind the identical frozen source. Nine downstream ignored tests remain unrun; not full CI, wasm runtime qualification or all applications/platforms. Optional --live verifier checks current live bytes; default verifier preserves the historical checkpoint if later authorized work changes live code. No commit or push performed.' };
writeFileSync(resolve(here, 'retained-exp-proof.json'), JSON.stringify(manifest, null, 2) + '\n', { flag: 'wx' });
console.log(JSON.stringify({ retainedSourceFiles: retained.length, changed, added, status: manifest.status }));
