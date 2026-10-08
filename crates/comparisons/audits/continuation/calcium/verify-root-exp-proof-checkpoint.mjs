// Evidence integrity and membership checks, not a theorem or retention decision.
import './verify-root-exp-checkpoint.mjs';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
const here = dirname(fileURLToPath(import.meta.url));
const read = p => readFileSync(resolve(here, p), 'utf8');
const json = p => JSON.parse(read(p));
const hash = p => createHash('sha256').update(readFileSync(resolve(here, p))).digest('hex');
function capture(tag, code = 0) {
  const r = json(`results/${tag}.json`);
  assert.equal(r.code, code, tag);
  assert.equal(r.signal, null, tag);
  return read(`results/${tag}.stdout`);
}
const experiment = json('root-exp-proof-experiment.json');
for (const [p, expected] of Object.entries({ ...experiment.sourceHashes, ...experiment.evidenceHashes }))
  assert.equal(hash(p), expected, p);
const baseline = json('baseline-hyperreal.json');
const changed = baseline.files.filter(f => hash(`root-exp-proof-trial-hyperreal/${f.path}`) !== f.sha256).map(f => f.path).sort();
assert.deepEqual(changed, experiment.changedLibraryFiles);
let productionLines = 0, productionBytes = 0, testLines = 0, testBytes = 0;
for (const p of [...changed, ...experiment.addedLibraryFiles]) {
  const original = baseline.files.some(f => f.path === p) ? read(`baseline-hyperreal/${p}`) : '';
  const trial = read(`root-exp-proof-trial-hyperreal/${p}`);
  const lines = trial.split('\n').length - original.split('\n').length;
  const bytes = Buffer.byteLength(trial) - Buffer.byteLength(original);
  if (p.endsWith('_tests.rs')) { testLines += lines; testBytes += bytes; }
  else { productionLines += lines; productionBytes += bytes; }
}
assert.deepEqual({ lines: productionLines, bytes: productionBytes }, experiment.productionSourceDelta);
assert.deepEqual({ lines: testLines, bytes: testBytes }, experiment.testSourceDelta);
for (const tag of ['root-exp-proof-public-debug-corrected', 'root-exp-proof-public-release', 'root-exp-proof-public-memcheck']) {
  const lines = capture(tag).trim().split('\n');
  assert.equal(lines.shift(), 'case,negative,precision,control,outcome');
  const rows = lines.map(s => s.split(','));
  assert.equal(rows.length, 96);
  const map = new Map(rows.map(r => [r.slice(0, 4).join(','), r[4]]));
  assert.equal(map.size, 96);
  for (let c = 0; c < 8; c++) for (const negative of [false, true]) for (const p of [-64, -256, -512])
    for (const control of ['identity', 'perturbed'])
      assert.equal(map.get([c, negative, p, control].join(',')), control === 'identity' ? 'Equal' : 'NotEqual', tag);
}
for (const profile of ['debug', 'release']) {
  const tests = capture(`root-exp-proof-final-full-${profile}`);
  assert(tests.includes('test result: ok. 770 passed; 0 failed; 0 ignored;'));
  for (const test of ['independent_rational_log_coefficients_and_unequal_controls',
    'products_and_inverse_roots_require_positive_exponential_leaves', 'cached_failure_does_not_prevent_later_separation',
    'proof_preserves_hot_operand_and_root_caches', 'structural_limits_fail_closed',
    'serialized_graph_recovers_proof_without_serialized_facts'])
    assert(tests.includes(`exp_relation_tests::${test} ... ok`), test);
  assert.deepEqual(JSON.parse(capture(`root-exp-proof-oracle-${profile}`)), { suite: 'oracle', inputs: 43, enclosure_checks: 15050 });
  assert.deepEqual(JSON.parse(capture(`root-exp-proof-state-${profile}`)), { suite: 'state', enclosure_checks: 261 });
}
assert(capture('root-exp-proof-focused-debug').includes('test result: ok. 6 passed; 0 failed;'));
capture('root-exp-proof-clippy');
assert(read('results/root-exp-proof-clippy.stderr').includes('Finished `dev` profile'));
capture('root-exp-proof-public-debug', 101);
assert(read('results/root-exp-proof-public-debug.stderr').includes('AddAssign'));
capture('root-exp-query-baseline-build-cpu', 101);
assert(read('results/root-exp-query-baseline-build-cpu.stderr').includes('no method named `reciprocal`'));
for (const tag of ['root-exp-proof-numeric-build-cpu', 'root-exp-proof-numeric-build-alloc',
  'root-exp-query-baseline-build-cpu-corrected', 'root-exp-query-baseline-build-alloc',
  'root-exp-query-proof-build-cpu', 'root-exp-query-proof-build-alloc',
  'run-root-exp-proof-query-cpu', 'run-root-exp-proof-numeric-cpu']) capture(tag);
const memcheck = read('results/root-exp-proof-public-memcheck.stderr');
for (const phrase of ['ERROR SUMMARY: 0 errors', 'definitely lost: 0 bytes', 'indirectly lost: 0 bytes',
  'possibly lost: 0 bytes', 'still reachable: 11,624 bytes']) assert(memcheck.includes(phrase));

const median = values => {
  const a = [...values].sort((x, y) => x - y);
  return (a[Math.floor((a.length - 1) / 2)] + a[Math.floor(a.length / 2)]) / 2;
};
const benchmark = [];
for (const phase of ['query', 'numeric']) for (const mode of ['cpu', 'alloc']) {
  const summary = json(`root-exp-proof-${phase}-${mode}-summary.json`);
  assert.equal(summary.cpu, 6); assert.equal(summary.mode, mode); assert.equal(summary.phase, phase);
  assert(Number.isFinite(Date.parse(summary.finished)) && Date.parse(summary.finished) >= Date.parse(summary.started));
  for (const variant of ['baseline', 'proof']) {
    assert(/^[0-9a-f]{64}$/.test(summary.binaries[variant].sha256));
    assert(summary.binaries[variant].bytes > 0);
  }
  const cases = phase === 'query' ? ['identity-small', 'identity-reduced', 'identity-large', 'identity-quotient',
    'identity-product', 'unresolved-near-exp', 'unresolved-trig-exp', 'unresolved-nonexp', 'separated-exp',
    'ordinary-root', 'ordinary-rational'] : ['exp-third', 'exp-sqrt2', 'exp-sine', 'exp-negative-sine',
    'exp-multiradical', 'exp-large', 'exp-tiny', 'exp-offset', 'exp-opaque-zero', 'ordinary-sqrt', 'ordinary-sine-root', 'perfect-square'];
  const lifecycles = phase === 'query' ? ['fresh', 'warm-pair', 'warm-difference'] :
    ['construct', 'fresh', 'warm', 'refine', 'hot-operand', 'construct-hot'];
  const observations = read(`results/root-exp-proof-${phase}-${mode}.jsonl`).trim().split('\n').map(JSON.parse);
  const blocks = mode === 'cpu' ? 12 : 3;
  assert.equal(observations.length, cases.length * lifecycles.length * blocks * 4);
  assert.equal(summary.summaries.length, cases.length * lifecycles.length);
  let seed = 17911;
  const rand = n => { seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0; return seed % n; };
  for (const name of cases) for (const lifecycle of lifecycles) {
    const group = observations.filter(r => r.case === name && r.lifecycle === lifecycle);
    const matches = summary.summaries.filter(r => r.name === name && r.lifecycle === lifecycle);
    assert.equal(matches.length, 1);
    const s = matches[0];
    assert.equal(group.length, blocks * 4); assert.equal(s.observations, group.length);
    assert(Number.isSafeInteger(s.iterations) && s.iterations >= 20 && s.iterations <= 100000);
    if (mode === 'alloc') assert.equal(s.iterations, 100);
    const ratios = [];
    for (let block = 0; block < blocks; block++) {
      const blockRows = group.filter(r => r.block === block);
      assert.deepEqual(blockRows.map(r => r.variant), block % 2 ? ['proof', 'baseline', 'baseline', 'proof'] :
        ['baseline', 'proof', 'proof', 'baseline']);
      const medians = {};
      for (const variant of ['baseline', 'proof']) {
        const pair = blockRows.filter(r => r.variant === variant);
        for (const r of pair) {
          assert.equal(r.iterations, s.iterations);
          assert(Number.isFinite(r.elapsed_ns) && r.elapsed_ns > 0);
          assert(r.alloc_calls >= 0 && r.allocated_bytes >= 0);
          if (mode === 'cpu') { assert.equal(r.alloc_calls, 0); assert.equal(r.allocated_bytes, 0); }
          if (mode === 'alloc' && lifecycle === 'fresh') assert(r.alloc_calls > 0);
          if (phase === 'query') {
            const outcome = name.startsWith('identity') ? variant === 'proof' ? 0 : 2 : name.startsWith('unresolved') ? 2 : 1;
            assert.deepEqual(r.outcomes, [0, 1, 2].map(i => i === outcome ? s.iterations : 0));
          }
        }
        medians[variant] = median(pair.map(r => r.elapsed_ns));
      }
      ratios.push(medians.proof / medians.baseline);
    }
    assert.equal(s.pairedMedianRatio, median(ratios));
    const bootstrap = Array.from({ length: 5000 }, () => median(ratios.map(() => ratios[rand(ratios.length)]))).sort((a, b) => a - b);
    assert.deepEqual(s.pairedMedianBootstrap95, [bootstrap[125], bootstrap[4875]]);
    for (const variant of ['baseline', 'proof']) for (const [key, raw] of [
      ['ns', 'elapsed_ns'], ['alloc_calls', 'alloc_calls'], ['alloc_bytes', 'allocated_bytes']])
      assert.equal(s[`${variant}_${key}`], median(group.filter(r => r.variant === variant).map(r => r[raw] / s.iterations)));
    if (phase === 'numeric' && mode === 'alloc') {
      assert.equal(s.baseline_alloc_calls, s.proof_alloc_calls);
      assert.equal(s.baseline_alloc_bytes, s.proof_alloc_bytes);
    }
  }
  benchmark.push({ phase, mode, observations: observations.length,
    driverFileByteDelta: summary.binaries.proof.bytes - summary.binaries.baseline.bytes });
}
assert.equal(benchmark.filter(b => b.mode === 'cpu').reduce((n, b) => n + b.observations, 0), experiment.cpuObservations);
assert.equal(benchmark.filter(b => b.mode === 'alloc').reduce((n, b) => n + b.observations, 0), experiment.allocationObservations);
console.log(JSON.stringify({ checkpoint: 'cache-preserving root/exponential proof',
  identityQueries: 48, unequalControls: 48, testsPerProfile: 770,
  oracleChecksPerProfile: 15050, stateChecksPerProfile: 261, benchmark,
  conclusion: experiment.status, limits: experiment.limits }));
