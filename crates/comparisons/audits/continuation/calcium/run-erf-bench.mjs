import { execFileSync } from 'node:child_process';
import { readFileSync, writeFileSync, appendFileSync, statSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';

const here = dirname(fileURLToPath(import.meta.url));
const workspace = resolve(here, '../../../..');
const mode = process.argv[2];
assert(['cpu', 'alloc'].includes(mode));
const output = resolve(here, `results/erf-paired-${mode}.jsonl`);
writeFileSync(output, '', { flag: 'wx' });
const binaries = Object.fromEntries(['baseline', 'trial'].map(variant => [variant,
  resolve(workspace, `.audit-builds/calcium-${variant === 'trial' ? 'prototype' : 'baseline'}/release/calcium-erf-${variant}-bench`)]));
const metadata = { mode, cpu: 6, started: new Date().toISOString(), binaries: {} };
for (const [variant, path] of Object.entries(binaries)) metadata.binaries[variant] = {
  path, bytes: statSync(path).size,
  sha256: createHash('sha256').update(readFileSync(path)).digest('hex'),
  size: execFileSync('size', [path], { encoding: 'utf8' }).trim(),
};
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
const cases = ['erf-zero', 'erf-third', 'erf-negative', 'erf-tiny', 'erf-tail',
  'erfc-third', 'erfc-negative', 'erfc-tail', 'normal-cdf', 'complement', 'ordinary-add', 'ordinary-sqrt'];
const summaries = [];
for (const name of cases) for (const lifecycle of ['construct', 'fresh', 'warm', 'refine']) {
  const pilot = ['baseline', 'trial'].map(v => run(v, name, lifecycle, 20));
  const ns = Math.max(...pilot.map(p => p.elapsed_ns / p.iterations));
  const iterations = mode === 'alloc' ? 100 : Math.max(20, Math.min(100000, Math.ceil(1e7 / ns)));
  const blocks = mode === 'alloc' ? 3 : 12;
  const rows = [];
  for (let block = 0; block < blocks; block++) {
    const order = block % 2 === 0 ? ['baseline', 'trial', 'trial', 'baseline'] : ['trial', 'baseline', 'baseline', 'trial'];
    for (const variant of order) {
      const row = { variant, block, ...run(variant, name, lifecycle, iterations) };
      assert.equal(row.iterations, iterations);
      assert.equal(row.case, name);
      assert.equal(row.lifecycle, lifecycle);
      assert(row.elapsed_ns > 0);
      if (mode === 'cpu') assert.equal(row.alloc_calls, 0);
      if (mode === 'alloc' && lifecycle === 'fresh' && name === 'erf-third') assert(row.alloc_calls > 0);
      rows.push(row);
      appendFileSync(output, JSON.stringify(row) + '\n');
    }
  }
  const paired = Array.from({ length: blocks }, (_, block) => {
    const value = variant => median(rows.filter(r => r.block === block && r.variant === variant).map(r => r.elapsed_ns));
    return value('trial') / value('baseline');
  });
  const bootstrap = Array.from({ length: 5000 }, () => median(paired.map(() => paired[random(paired.length)]))).sort((a, b) => a - b);
  const med = (variant, key) => median(rows.filter(r => r.variant === variant).map(r => r[key] / iterations));
  const summary = { name, lifecycle, iterations, observations: rows.length,
    baseline_ns: med('baseline', 'elapsed_ns'), trial_ns: med('trial', 'elapsed_ns'),
    pairedMedianRatio: median(paired), pairedMedianBootstrap95: [bootstrap[125], bootstrap[4875]],
    baseline_alloc_calls: med('baseline', 'alloc_calls'), trial_alloc_calls: med('trial', 'alloc_calls'),
    baseline_alloc_bytes: med('baseline', 'allocated_bytes'), trial_alloc_bytes: med('trial', 'allocated_bytes') };
  summaries.push(summary);
  console.log(JSON.stringify(summary));
}
writeFileSync(resolve(here, `erf-paired-${mode}-summary.json`), JSON.stringify({ ...metadata,
  finished: new Date().toISOString(), summaries,
  limits: 'Absolute approximation work, not certified equality queries. Independent oracle checks accuracy. Warm global constants are preconditioned in every process. Allocation clocks are not CPU evidence; requested bytes are cumulative, not peak heap. Bootstrap intervals describe this host/session only.' }, null, 2) + '\n', { flag: 'wx' });
