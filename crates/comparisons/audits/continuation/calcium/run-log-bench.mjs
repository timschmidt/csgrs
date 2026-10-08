import { execFileSync } from 'node:child_process';
import { readFileSync, writeFileSync, appendFileSync, statSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';

const here = dirname(fileURLToPath(import.meta.url));
const workspace = resolve(here, '../../../..');
const mode = process.argv[2] ?? 'cpu';
assert(['cpu', 'alloc'].includes(mode));
const version = process.argv[3] ?? 'v2';
assert(/^[a-z0-9-]+$/.test(version));
const output = resolve(here, `results/log-paired-${version}-${mode}.jsonl`);
writeFileSync(output, '', { flag: 'wx' });
const binaries = {
  baseline: resolve(workspace, '.audit-builds/calcium-baseline/release/calcium-log-baseline-bench'),
  trial: resolve(workspace, '.audit-builds/calcium-prototype/release/calcium-log-trial-bench'),
};
const metadata = { mode, version, cpu: 6, started: new Date().toISOString(), binaries: {} };
for (const [variant, path] of Object.entries(binaries)) {
  metadata.binaries[variant] = { path, bytes: statSync(path).size,
    sha256: createHash('sha256').update(readFileSync(path)).digest('hex'),
    size: execFileSync('size', [path], { encoding: 'utf8' }).trim() };
}
const cases = ['log-identity', 'algebraic-identity', 'log-near-nonzero',
  'log-near-unknown', 'log-algebraic-near-unknown', 'log-multiquadratic-unknown',
  'log-transcendental-unknown', 'nonlog-unknown', 'ordinary-log', 'ordinary-rational', 'exp-cancellation'];
const run = (variant, name, lifecycle, iterations) => JSON.parse(execFileSync('taskset',
  ['-c', '6', binaries[variant], name, lifecycle, String(iterations)],
  { encoding: 'utf8', timeout: 30000 }));
const median = xs => {
  const a = [...xs].sort((x, y) => x - y);
  return (a[Math.floor((a.length - 1) / 2)] + a[Math.floor(a.length / 2)]) / 2;
};
let seed = 8675309;
const random = n => {
  seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0;
  return seed % n;
};
const summaries = [];
for (const name of cases) for (const lifecycle of ['fresh', 'warm']) {
  const pilot = ['baseline', 'trial'].map(v => run(v, name, lifecycle, 100));
  const ns = Math.max(...pilot.map(p => p.elapsed_ns / p.iterations));
  const iterations = mode === 'alloc' ? 100 : Math.max(20, Math.min(100000, Math.ceil(1e7 / ns)));
  const blocks = mode === 'alloc' ? 3 : 12;
  const observations = [];
  for (let block = 0; block < blocks; block++) {
    const order = block % 2 === 0 ? ['baseline', 'trial', 'trial', 'baseline'] : ['trial', 'baseline', 'baseline', 'trial'];
    for (const variant of order) {
      const row = { variant, block, ...run(variant, name, lifecycle, iterations) };
      assert.equal(row.outcomes.reduce((a, b) => a + b, 0), iterations);
      if (mode === 'cpu') assert.equal(row.alloc_calls, 0, 'CPU binary has counting instrumentation');
      if (mode === 'alloc' && name === 'log-identity' && lifecycle === 'fresh')
        assert(row.alloc_calls > 0, 'Allocation binary lacks counting instrumentation');
      observations.push(row);
      appendFileSync(output, JSON.stringify(row) + '\n');
    }
  }
  const distribution = variant => observations.filter(r => r.variant === variant)
    .map(r => r.outcomes.map(n => n / iterations));
  const baselineOutcomes = distribution('baseline');
  const trialOutcomes = distribution('trial');
  for (const outcomes of baselineOutcomes) assert.deepEqual(outcomes, baselineOutcomes[0]);
  for (const outcomes of trialOutcomes) assert.deepEqual(outcomes, trialOutcomes[0]);
  const matched = JSON.stringify(baselineOutcomes[0]) === JSON.stringify(trialOutcomes[0]);
  if (!['log-identity', 'log-near-unknown', 'log-algebraic-near-unknown'].includes(name))
    assert(matched, `Unexpected capability difference: ${name}`);
  const paired = [];
  for (let block = 0; block < blocks; block++) {
    const value = variant => median(observations.filter(r => r.block === block && r.variant === variant).map(r => r.elapsed_ns));
    paired.push(value('trial') / value('baseline'));
  }
  const bootstrap = Array.from({ length: 5000 }, () => median(paired.map(() => paired[random(paired.length)]))).sort((a, b) => a - b);
  const med = (variant, key) => median(observations.filter(r => r.variant === variant).map(r => r[key] / iterations));
  const summary = { name, lifecycle, iterations, observations: observations.length, matched,
    baselineOutcomes: baselineOutcomes[0], trialOutcomes: trialOutcomes[0],
    baseline_ns: med('baseline', 'elapsed_ns'), trial_ns: med('trial', 'elapsed_ns'),
    pairedMedianRatio: median(paired), pairedMedianBootstrap95: [bootstrap[125], bootstrap[4875]],
    baseline_alloc_calls: med('baseline', 'alloc_calls'), trial_alloc_calls: med('trial', 'alloc_calls'),
    baseline_alloc_bytes: med('baseline', 'allocated_bytes'), trial_alloc_bytes: med('trial', 'allocated_bytes') };
  summaries.push(summary);
  console.log(JSON.stringify(summary));
}
writeFileSync(resolve(here, `log-paired-${version}-${mode}-summary.json`), JSON.stringify({ ...metadata,
  finished: new Date().toISOString(), summaries,
  limits: 'Bootstrap intervals are descriptive of this host/session; unmatched rows compare Unknown to a proof, not equivalent successful work. Allocation-build clocks are not CPU performance evidence.' }, null, 2) + '\n', { flag: 'wx' });
