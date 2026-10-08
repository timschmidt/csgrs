import { spawn } from 'node:child_process';
import { readFileSync, writeFileSync, appendFileSync, copyFileSync, constants, statSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here = dirname(fileURLToPath(import.meta.url));
const target = '/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse';
const snapshots = '/tmp/calcium-polynomial-decision.ItisYX';
const variants = ['baseline', 'trial'];
const gates = ['baseline-tests-debug', 'trial-tests-debug', 'trial-tests-release',
  'trial-tests-release-corrected', 'trial-clippy', 'baseline-bench-build', 'trial-bench-build'];
for (const gate of gates) {
  const result = JSON.parse(readFileSync(resolve(here, `results/polynomial-decision-${gate}.json`)));
  assert.equal(result.code, 0, gate);
}
async function command(file, args) {
  return new Promise((ok, fail) => {
    const child = spawn(file, args, { stdio: ['ignore', 'pipe', 'pipe'] });
    let stdout = '', stderr = '';
    child.stdout.on('data', s => stdout += s); child.stderr.on('data', s => stderr += s);
    child.on('error', fail);
    child.on('close', code => code === 0 ? ok(stdout) : fail(Error(`${file}: ${code}: ${stderr}`)));
  });
}
const binaries = {};
for (const variant of variants) {
  const path = resolve(snapshots, `polynomial-cpu-${variant}`);
  copyFileSync(resolve(target, 'release', `calcium-polynomial-decision-${variant}-bench`), path, constants.COPYFILE_EXCL);
  binaries[variant] = { path, bytes: statSync(path).size,
    sha256: createHash('sha256').update(readFileSync(path)).digest('hex'),
    size: (await command('size', [path])).trim() };
}
const started = new Date().toISOString();
const median = input => {
  const a = [...input].sort((x, y) => x - y);
  return (a[Math.floor((a.length - 1) / 2)] + a[Math.floor(a.length / 2)]) / 2;
};
let seed = 997;
const random = n => { seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0; return seed % n; };
const output = resolve(here, 'results/polynomial-decision-cpu.jsonl');
writeFileSync(output, '', { flag: 'wx' });
const summaries = [];
for (const name of ['rational-self', 'radical-self', 'log-self', 'log-plus-one', 'tiny-self', 'unknown-leading']) {
  for (const degree of [1, 8, 16]) for (const lifecycle of ['fresh', 'retained']) {
    const run = async (variant, iterations) => JSON.parse(await command('taskset',
      ['-c', '6', binaries[variant].path, name, String(degree), lifecycle, String(iterations)]));
    const pilots = [];
    for (const variant of variants) pilots.push(await run(variant, 10));
    const iterations = Math.max(10, Math.min(10000,
      Math.ceil(1e7 / Math.max(...pilots.map(p => p.elapsed_ns / p.iterations)))));
    const rows = [];
    for (let block = 0; block < 12; block++) {
      const order = block % 2 ? ['trial', 'baseline', 'baseline', 'trial'] : ['baseline', 'trial', 'trial', 'baseline'];
      for (const variant of order) {
        const row = { variant, block, ...await run(variant, iterations) };
        assert.equal(row.case, name); assert.equal(row.degree, degree);
        assert.equal(row.lifecycle, lifecycle); assert.equal(row.iterations, iterations);
        const expectedKnown = ['rational-self', 'radical-self'].includes(name)
          || (variant === 'trial' && name !== 'unknown-leading');
        assert.equal(row.known, expectedKnown ? iterations : 0);
        assert(row.elapsed_ns > 0);
        rows.push(row); appendFileSync(output, JSON.stringify(row) + '\n');
      }
    }
    const ratios = Array.from({ length: 12 }, (_, block) => {
      const time = variant => median(rows.filter(r => r.block === block && r.variant === variant).map(r => r.elapsed_ns));
      return time('trial') / time('baseline');
    });
    const bootstrap = Array.from({ length: 5000 }, () => median(ratios.map(() => ratios[random(12)]))).sort((a, b) => a - b);
    const summary = { name, degree, lifecycle, iterations, observations: rows.length, pilots,
      measurements: Object.fromEntries(variants.map(v => [v, median(rows.filter(r => r.variant === v).map(r => r.elapsed_ns / iterations))])),
      pairedMedianRatio: median(ratios), pairedMedianBootstrap95: [bootstrap[125], bootstrap[4875]] };
    summaries.push(summary); console.log(JSON.stringify(summary));
  }
}
writeFileSync(resolve(here, 'polynomial-decision-cpu-summary.json'), JSON.stringify({ started,
  finished: new Date().toISOString(), cpu: 6, binaries, summaries,
  limits: 'Uninstrumented CPU only. New known results do more work than baseline early Unknown; not equal-work speedups. Retained operands are preconditioned by eight queries; fresh includes construction and result teardown. All lower coefficients share one scalar node. No cold-process, concurrency, allocations, peak memory or application size qualification. Paired bootstrap intervals describe this host/session, not universal bounds.' }, null, 2) + '\n', { flag: 'wx' });
