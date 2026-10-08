// Mechanical evidence binding; source reads and disposition are recorded by
// the auditor, not inferred by this script. Refuses to overwrite a checkpoint.
import { readFileSync, readdirSync, writeFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
const here = dirname(fileURLToPath(import.meta.url));
const hash = p => createHash('sha256').update(readFileSync(resolve(here, p))).digest('hex');
const files = [
  'root-exp-probe.rs', 'root-exp-qualification.rs', 'root-exp-bench.rs',
  'run-root-exp-bench.mjs', 'flint-root-exp-probe.c', 'flint-trig-state-probe.c',
  'root-exp-trial-hyperreal/src/computable/node/roots_inverse_hyperbolic.rs',
  'bind-root-exp-experiment.mjs',
  ...['baseline', 'trial'].flatMap(v => ['probe', 'qualification', 'bench'].flatMap(k =>
    ['Cargo.toml', 'Cargo.lock'].map(f => `root-exp-${v}-${k}/${f}`))),
];
const evidence = [
  ...readdirSync(resolve(here, 'results')).filter(p => /^(root-exp-|trig-state-)/.test(p)).sort().map(p => `results/${p}`),
  'root-exp-paired-cpu-summary.json', 'root-exp-paired-alloc-summary.json',
];
const experiment = {
  schema: 1, recorded: new Date().toISOString(),
  status: 'Not selected as implemented; no live Hyper production transfer.',
  changedLibraryFiles: ['src/computable/node/roots_inverse_hyperbolic.rs'],
  librarySourceDelta: { lines: 25, bytes: 1394 },
  identityQueries: 48, unequalControls: 48,
  baselineIdentities: { Equal: 12, Unknown: 36 }, trialIdentities: { Equal: 48, Unknown: 0 },
  oracleChecksPerVariantPerProfile: 15050, stateChecksPerVariantPerProfile: 261,
  nativeTrigFailures: { destinationState: 25, inPlaceBranch: 1, totalControls: 181 },
  sourceHashes: Object.fromEntries(files.map(p => [p, hash(p)])),
  evidenceHashes: Object.fromEntries(evidence.map(p => [p, hash(p)])),
  limits: 'Capability probes do not match donor/Hyper precision-cost contracts. Tests and MPFR samples are not proofs of all inputs. No downstream qualification for rejected scalar rewrite. CPU benchmark results describe this host/session; allocated bytes are cumulative requests, not peak memory; linked driver size is not downstream application size. Forty additional complete source reads plus partial documentation do not complete either donor or the ecosystem inventory. Earlier unsuccessful harness/prototype stages remain bound as failures, not silently replaced.',
};
writeFileSync(resolve(here, 'root-exp-experiment.json'), JSON.stringify(experiment, null, 2) + '\n', { flag: 'wx' });
console.log(JSON.stringify({ sources: files.length, evidenceFiles: evidence.length, status: experiment.status }));
