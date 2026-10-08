import { readFileSync, writeFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here = dirname(fileURLToPath(import.meta.url));
const read = p => readFileSync(resolve(here, p));
const json = p => JSON.parse(read(p));
const hash = p => createHash('sha256').update(read(p)).digest('hex');
const tags = { 'vector-state-compile': 0, 'vector-state-native': 1,
  'vector-state-memcheck': 1, 'reuse-wasm-all-features': 0 };
const phases = ['numeric', 'first-touch'], modes = ['cpu', 'alloc'], variants = ['baseline', 'sign', 'reuse'];
for (const phase of phases) for (const mode of modes) {
  tags[`run-reuse-cost-${phase}-${mode}`] = 0;
  for (const v of variants) tags[`reuse-cost-build-${phase}-${mode}-${v}`] = 0;
}
for (const v of variants) {
  tags[`reuse-app-build-${v}`] = 0;
  for (const example of ['basic', 'arrangement']) for (const stage of ['strip', 'size', 'run'])
    tags[`reuse-app-${stage}-${v}-${example}`] = 0;
}
for (const [tag, code] of Object.entries(tags)) {
  const r = json(`results/${tag}.json`); assert.equal(r.code, code, tag); assert.equal(r.signal, null, tag);
}
const sources = ['reuse-qualification-experiment.json', 'reuse-costs-config.json',
  'root-exp-bench.rs', 'reuse-first-touch-bench.rs', 'run-reuse-cost-bench.mjs',
  'run-reuse-cost-campaign.mjs', 'measure-reuse-app-size.mjs', 'flint-vector-state-probe.c',
  'bind-reuse-cost-experiment.mjs', 'verify-reuse-cost-checkpoint.mjs',
  ...phases.flatMap(p => variants.flatMap(v => ['Cargo.toml', 'Cargo.lock'].map(f => `reuse-${p}-${v}/${f}`)))];
const evidence = Object.keys(tags).flatMap(t => ['json', 'stdout', 'stderr'].map(e => `results/${t}.${e}`))
  .concat(phases.flatMap(p => modes.flatMap(m => [`results/reuse-cost-${p}-${m}.jsonl`, `reuse-cost-${p}-${m}-summary.json`])))
  .concat(['reuse-app-size-summary.json']);
const vectorCoverage = json('coverage.json').filter(c => /^(src\/)?ca_vec(\/|\.h$)/.test(c.path) || c.path === 'doc/source/ca_vec.rst');
assert.equal(vectorCoverage.length, 22);
const manifest = { schema: 1, recorded: new Date().toISOString(),
  status: 'V3 cost/portability/example-size qualification complete; retention decision must also preserve all earlier correctness gates and live user changes.',
  sourceHashes: Object.fromEntries(sources.map(p => [p, hash(p)])),
  evidenceHashes: Object.fromEntries(evidence.map(p => [p, hash(p)])), expectedCaptureCodes: tags,
  coverage: { calcium: { complete: 190, partial: 1, lines: 20778 }, flint: { complete: 180, partial: 6, lines: 22215 } },
  vectorCoverage, vectorProbe: { negationCases: 105, wrongPublicInPlace: 24, zeroTruthCases: 27, zeroTruthFailures: 0 },
  observations: { numeric: { cpu: 5184, alloc: 1296 }, firstTouch: { cpu: 1728, alloc: 432 } },
  limits: 'CPU snapshots and raw rows are preserved separately from instrumented allocations; all campaign console captures are empty, so coverage comes from exact raw rows/summaries. First-use workers have fresh TLS but warm numeric operands/constants and OS thread cache; not whole-program cold start or concurrent throughput. Numerical kernels are unchanged in source; measured small latency differences are not universal improvements. Representative Hypercurve examples are not all production applications or size-optimized builds. WASM compile-only, not browser/runtime performance. Vector in-place negation failure demonstrated only for current FLINT; archived mechanism source-read. No donor patch or external report.' };
writeFileSync(resolve(here, 'reuse-cost-experiment.json'), JSON.stringify(manifest, null, 2) + '\n', { flag: 'wx' });
console.log(JSON.stringify({ sourceFiles: sources.length, evidenceFiles: evidence.length, status: manifest.status }));
