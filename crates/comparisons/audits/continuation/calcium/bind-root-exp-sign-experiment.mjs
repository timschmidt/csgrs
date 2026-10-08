// Immutable evidence binding, not source-read certification or acceptance.
import { readFileSync, readdirSync, writeFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here = dirname(fileURLToPath(import.meta.url));
const hash = p => createHash('sha256').update(readFileSync(resolve(here, p))).digest('hex');
const baseline = JSON.parse(readFileSync(resolve(here, 'baseline-hyperreal.json')));
const changed = baseline.files.filter(f => hash(`root-exp-sign-trial-hyperreal/${f.path}`) !== f.sha256).map(f => f.path).sort();
assert.deepEqual(changed, ['src/computable/node.rs', 'src/computable/node/structural_analysis.rs']);
const added = ['src/computable/node/exp_relation.rs', 'src/computable/node/exp_relation_tests.rs'];
const sources = [...changed, ...added].map(p => `root-exp-sign-trial-hyperreal/${p}`).concat([
  'root-exp-probe.rs', 'root-exp-qualification.rs', 'root-exp-bench.rs', 'root-exp-query-bench.rs',
  'root-exp-sign-probe.rs', 'run-root-exp-sign-bench.mjs', 'flint-polynomial-state-probe.c',
  'bind-root-exp-sign-experiment.mjs', 'verify-root-exp-sign-checkpoint.mjs',
  ...['root-exp-sign-probe', 'root-exp-sign-qualification', 'root-exp-sign-bench',
    'root-exp-query-sign-bench', 'root-exp-sign-public-baseline', 'root-exp-sign-public-trial']
    .flatMap(d => ['Cargo.toml', 'Cargo.lock'].map(p => `${d}/${p}`)),
]);
const evidence = readdirSync(resolve(here, 'results')).filter(p => /^(root-exp-sign-|run-root-exp-sign-|polynomial-state-)/.test(p))
  .sort().map(p => `results/${p}`).concat(['query', 'numeric'].flatMap(phase => ['cpu', 'alloc']
    .map(mode => `root-exp-sign-${phase}-${mode}-summary.json`)));
const experiment = {
  schema: 1, recorded: new Date().toISOString(),
  status: 'Sign-capable candidate pending downstream/work-limit qualification; not retained in live Hyper.',
  changedLibraryFiles: changed, addedLibraryFiles: added,
  nonTestFileDelta: { lines: 150, bytes: 6102, includesTestOnlyWrapper: true },
  testFileDelta: { lines: 248, bytes: 10155 },
  testsPerProfile: 772, independentRationalCases: 512, additionalRemainderCases: 30,
  identityQueries: 48, unequalControls: 48,
  tinySignQueries: { newlyDecided: 96, unresolvedControls: 48 },
  oracleChecksPerProfile: 15050, stateChecksPerProfile: 261,
  cpuObservations: 5040, allocationObservations: 1260,
  donorPolynomialChecks: { cases: 64, integerFailures: 24, rationalFailures: 0 },
  sourceHashes: Object.fromEntries(sources.map(p => [p, hash(p)])),
  evidenceHashes: Object.fromEntries(evidence.map(p => [p, hash(p)])),
  limits: 'Scalar qualification only; no downstream consumer gate or live production transfer. Existing Unknown cache reuses work only on a retained difference. Collector limits do not bound all opaque structural-equality descendant work. Newly decided tiny signs are 16 formulas times two orientations times three precision floors, not 96 independent formulas. CPU results are host/session measurements; allocation bytes are cumulative requests, not peak memory; linked drivers are not stripped applications. Donor missing-output defect was executed only in current FLINT; archived source has the same mechanism. Full ecosystem audit remains incomplete.',
};
writeFileSync(resolve(here, 'root-exp-sign-experiment.json'), JSON.stringify(experiment, null, 2) + '\n', { flag: 'wx' });
console.log(JSON.stringify({ sources: sources.length, evidenceFiles: evidence.length, status: experiment.status }));
