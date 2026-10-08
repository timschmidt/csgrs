// Mechanical immutable checkpoint, not proof of source reading or acceptance.
import { readFileSync, readdirSync, writeFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here = dirname(fileURLToPath(import.meta.url));
const read = p => readFileSync(resolve(here, p));
const hash = p => createHash('sha256').update(read(p)).digest('hex');
const baseline = JSON.parse(read('baseline-hyperreal.json'));
const changed = baseline.files.filter(f => hash(`root-exp-proof-trial-hyperreal/${f.path}`) !== f.sha256).map(f => f.path).sort();
assert.deepEqual(changed, ['src/computable/node.rs', 'src/computable/node/structural_analysis.rs']);
const added = ['src/computable/node/exp_relation.rs', 'src/computable/node/exp_relation_tests.rs'];
const source = [
  ...changed, ...added,
].map(p => `root-exp-proof-trial-hyperreal/${p}`).concat([
  'root-exp-probe.rs', 'root-exp-qualification.rs', 'root-exp-bench.rs',
  'root-exp-query-bench.rs', 'run-root-exp-proof-bench.mjs',
  'bind-root-exp-proof-experiment.mjs', 'verify-root-exp-proof-checkpoint.mjs',
  ...['root-exp-proof-probe', 'root-exp-proof-qualification', 'root-exp-proof-bench',
    'root-exp-query-baseline-bench', 'root-exp-query-proof-bench'].flatMap(d =>
      ['Cargo.toml', 'Cargo.lock'].map(f => `${d}/${f}`)),
]);
const evidence = readdirSync(resolve(here, 'results'))
  .filter(p => /^(root-exp-proof-|root-exp-query-|run-root-exp-proof-)/.test(p))
  .sort().map(p => `results/${p}`).concat(
    ['query', 'numeric'].flatMap(phase => ['cpu', 'alloc'].map(mode =>
      `root-exp-proof-${phase}-${mode}-summary.json`)));
const experiment = {
  schema: 1, recorded: new Date().toISOString(),
  status: 'Pending candidate: no live production transfer, not rejected.',
  changedLibraryFiles: changed, addedLibraryFiles: added,
  productionSourceDelta: { lines: 143, bytes: 5782 },
  testSourceDelta: { lines: 172, bytes: 6969 },
  identityQueries: 48, unequalControls: 48, testsPerProfile: 770,
  independentRationalCases: 512,
  oracleChecksPerProfile: 15050, stateChecksPerProfile: 261,
  cpuObservations: 5040, allocationObservations: 1260,
  sourceHashes: Object.fromEntries(source.map(p => [p, hash(p)])),
  evidenceHashes: Object.fromEntries(evidence.map(p => [p, hash(p)])),
  limits: 'All work remains isolated. No downstream qualification or retention decision. Structural collector limits do not bound all opaque structural-equality descendant work. Current proof is zero-only. Rebuilt difference queries repeat unsuccessful proof work; retained differences reuse existing Unknown cache. Different-outcome clocks are not equal-work speedups, allocation requests are not peak heap, linked benchmark drivers are not stripped downstream applications. Earlier two compile failures are preserved. Scope and coverage remain incomplete.',
};
writeFileSync(resolve(here, 'root-exp-proof-experiment.json'), JSON.stringify(experiment, null, 2) + '\n', { flag: 'wx' });
console.log(JSON.stringify({ sources: source.length, evidenceFiles: evidence.length, status: experiment.status }));
