import { spawn } from 'node:child_process';
import { readFileSync, writeFileSync, appendFileSync, statSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';

const here = dirname(fileURLToPath(import.meta.url));
const config = JSON.parse(readFileSync(resolve(here, 'reuse-costs-config.json')));
const [phase, mode] = process.argv.slice(2);
assert(['numeric', 'first-touch'].includes(phase) && ['cpu', 'alloc'].includes(mode));
const stem = `reuse-cost-${phase}-${mode}`;
const variants = ['baseline', 'sign', 'reuse'];
async function command(file, args) {
  return new Promise((ok, fail) => {
    const child = spawn(file, args, { stdio: ['ignore', 'pipe', 'pipe'] });
    let stdout = '', stderr = '';
    child.stdout.on('data', s => stdout += s); child.stderr.on('data', s => stderr += s);
    child.on('error', fail);
    child.on('close', code => code === 0 ? ok(stdout) : fail(Error(`${file} ${args}: ${code}\n${stderr}`)));
  });
}
const binaries = {};
for (const v of variants) {
  const path = resolve(config.binarySnapshots, `${phase}-${mode}-${v}`);
  binaries[v] = { path, bytes: statSync(path).size,
    sha256: createHash('sha256').update(readFileSync(path)).digest('hex'),
    size: (await command('size', [path])).trim() };
}
const metadata = { phase, mode, cpu: config.cpu, started: new Date().toISOString(), binaries };
const cases = phase === 'numeric' ? ['exp-third', 'exp-sqrt2', 'exp-sine', 'exp-negative-sine',
  'exp-multiradical', 'exp-large', 'exp-tiny', 'exp-offset', 'exp-opaque-zero',
  'ordinary-sqrt', 'ordinary-sine-root', 'perfect-square']
  : ['identity-1', 'identity-8', 'identity-32', 'identity-128',
    'unresolved-1', 'unresolved-8', 'unresolved-32', 'unresolved-128'];
const settings = phase === 'numeric' ? ['construct', 'fresh', 'warm', 'refine', 'hot-operand', 'construct-hot']
  : ['independent', 'shared', 'common-tail'];
const run = async (v, name, setting, count) => JSON.parse(await command('taskset',
  ['-c', String(config.cpu), binaries[v].path, name, setting, String(count)]));
const median = input => {
  const a = [...input].sort((x, y) => x - y);
  return (a[Math.floor((a.length - 1) / 2)] + a[Math.floor(a.length / 2)]) / 2;
};
let seed = 17911;
const random = n => { seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0; return seed % n; };
const output = resolve(here, `results/${stem}.jsonl`);
writeFileSync(output, '', { flag: 'wx' });
const summaries = [];
for (const name of cases) for (const setting of settings) {
  const pilots = [];
  for (const v of variants) pilots.push(await run(v, name, setting, phase === 'numeric' ? 20 : 2));
  const iterations = phase === 'first-touch' ? mode === 'cpu' ? 32 : 16 : mode === 'alloc' ? 100
    : Math.max(20, Math.min(100000, Math.ceil(1e7 / Math.max(...pilots.map(p => p.elapsed_ns / p.iterations)))));
  const blocks = mode === 'cpu' ? 12 : 3;
  const rows = [];
  for (let block = 0; block < blocks; block++) {
    const order = variants.map((_, i) => variants[(i + block) % 3]);
    for (const variant of [...order, ...order.toReversed()]) {
      const r = { variant, block, ...await run(variant, name, setting, iterations) };
      assert.equal(r.case, name);
      if (phase === 'numeric') {
        assert.equal(r.lifecycle, setting); assert.equal(r.iterations, iterations); assert(r.elapsed_ns > 0);
        if (mode === 'cpu') assert.deepEqual([r.alloc_calls, r.allocated_bytes], [0, 0]);
      } else {
        assert.equal(r.sharing, setting); assert.equal(r.workers, iterations); assert.equal(r.warm_queries_per_worker, 16);
        assert(r.first_ns > 0 && r.warm_ns > 0 && r.lifecycle_ns >= r.first_ns + r.warm_ns);
        // Baseline may already decide a shared representation; never require
        // it to lose a valid proof. Candidate identities must remain decidable.
        assert.equal(r.outcomes[1], 0);
        const expected = name.startsWith('unresolved') ? 2 : variant === 'baseline'
          ? pilots[0].outcomes[0] === 2 ? 0 : 2 : 0;
        assert.deepEqual(r.outcomes, [0, 1, 2].map(i => i === expected ? iterations : 0));
        if (mode === 'cpu') for (const key of ['first_calls', 'first_bytes', 'warm_calls', 'warm_bytes']) assert.equal(r[key], 0);
      }
      rows.push(r); appendFileSync(output, JSON.stringify(r) + '\n');
    }
  }
  const clocks = phase === 'numeric' ? ['elapsed_ns'] : ['first_ns', 'warm_ns', 'lifecycle_ns'];
  const comparisons = {};
  for (const control of ['baseline', 'sign']) {
    comparisons[control] = {};
    for (const clock of clocks) {
      const ratios = Array.from({ length: blocks }, (_, block) => {
        const get = v => median(rows.filter(r => r.block === block && r.variant === v).map(r => r[clock]));
        return get('reuse') / get(control);
      });
      const boot = Array.from({ length: 5000 }, () => median(ratios.map(() => ratios[random(ratios.length)]))).sort((a, b) => a - b);
      comparisons[control][clock] = { pairedMedianRatio: median(ratios), pairedMedianBootstrap95: [boot[125], boot[4875]] };
    }
  }
  const keys = phase === 'numeric' ? ['elapsed_ns', 'alloc_calls', 'allocated_bytes']
    : ['first_ns', 'warm_ns', 'clock_ns', 'lifecycle_ns', 'first_calls', 'first_bytes', 'warm_calls', 'warm_bytes'];
  const measurements = Object.fromEntries(variants.map(v => [v, Object.fromEntries(keys.map(key => [key,
    median(rows.filter(r => r.variant === v).map(r => r[key] / iterations / (key.startsWith('warm_') ? 16 : 1)))]))]));
  const summary = { name, setting, iterations, observations: rows.length, pilots, comparisons, measurements };
  summaries.push(summary); console.log(JSON.stringify(summary));
}
writeFileSync(resolve(here, `${stem}-summary.json`), JSON.stringify({ ...metadata,
  finished: new Date().toISOString(), summaries,
  limits: 'Same frozen numeric workload. First-touch operands/numeric caches and process constants preconditioned on parent; measured workers have fresh TLS, execute one first query then16 warm queries. Query clocks exclude thread start/exit; lifecycle clock includes17 queries and thread start/join/exit. Empty-clock samples reported without subtraction. Workers inherit CPU affinity and run sequentially. Shared/common-tail/independent graphs, not every collision or sharing pattern. Baseline Unknown versus candidate Equal is different work. Instrumented allocation clocks are not CPU evidence; requested Rust allocation bytes exclude native bookkeeping/allocator overhead. Snapshot binaries preserved separately by mode. Host/session confidence intervals, not universal bounds.' }, null, 2) + '\n', { flag: 'wx' });
