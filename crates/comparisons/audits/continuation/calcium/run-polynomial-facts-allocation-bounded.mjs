// Preserve the failed deterministic-live-byte campaign and reuse its binaries.
// The existing scalar cache uses pointer-indexed, bounded weak slots: occupancy
// can vary with addresses even when successful allocation request totals agree.
import { spawn } from 'node:child_process';
import { readFileSync, writeFileSync, appendFileSync, statSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here = dirname(fileURLToPath(import.meta.url));
const variants = ['baseline', 'trial', 'facts'];
const binaries = Object.fromEntries(variants.map(v => {
  const path = `/tmp/calcium-polynomial-facts.ZX8NqH/polynomial-allocation-${v}`;
  return [v, { path, bytes: statSync(path).size,
    sha256: createHash('sha256').update(readFileSync(path)).digest('hex') }];
}));
async function query(variant, name, degree, lifecycle, iterations) {
  return new Promise((ok, fail) => {
    const child = spawn(binaries[variant].path, [name, String(degree), lifecycle, String(iterations)],
      { stdio: ['ignore', 'pipe', 'pipe'] });
    let stdout = '', stderr = '';
    child.stdout.on('data', s => stdout += s); child.stderr.on('data', s => stderr += s);
    child.on('error', fail);
    child.on('close', code => {
      if (code !== 0) { fail(Error(`${variant}: ${code}: ${stderr}`)); return; }
      try {
        const r = { variant, ...JSON.parse(stdout) };
        assert.deepEqual([r.case, r.degree, r.lifecycle, r.iterations], [name, degree, lifecycle, iterations]);
        const known = ['rational-self', 'radical-self'].includes(name) || (variant !== 'baseline' && name !== 'unknown-leading');
        assert.equal(r.known, known ? iterations : 0);
        for (const key of ['alloc_calls', 'allocated_bytes']) assert(Number.isSafeInteger(r[key]) && r[key] >= 0);
        assert(Number.isSafeInteger(r.retained_bytes));
        if (lifecycle === 'retained' || name === 'rational-self') assert.equal(r.retained_bytes, 0);
        else if (name === 'tiny-self') {
          // 16 slots, two weak Node allocations each, 72 requested bytes per
          // native allocation: previously independently measured scalar bound.
          assert(r.retained_bytes >= 0 && r.retained_bytes <= 32 * 72);
          assert.equal(r.retained_bytes % 72, 0);
        } else assert.equal(r.retained_bytes, 384);
        ok(r);
      } catch (error) { fail(error); }
    });
  });
}
const output = resolve(here, 'results/polynomial-facts-allocation-bounded.jsonl');
writeFileSync(output, '', { flag: 'wx' });
const started = new Date().toISOString(), summaries = [];
for (const name of ['rational-self', 'radical-self', 'log-self', 'log-plus-one', 'tiny-self', 'unknown-leading']) {
  for (const degree of [1, 8, 16]) for (const lifecycle of ['fresh', 'retained']) {
    const rows = [], iterations = 100;
    for (let block = 0; block < 3; block++) {
      const forward = variants.map((_, i) => variants[(i + block) % 3]);
      for (const v of [...forward, ...forward.toReversed()]) {
        const r = { block, ...await query(v, name, degree, lifecycle, iterations) };
        rows.push(r); appendFileSync(output, JSON.stringify(r) + '\n');
      }
    }
    const measurements = {};
    for (const v of variants) {
      const records = rows.filter(r => r.variant === v);
      for (const r of records) assert.deepEqual([r.alloc_calls, r.allocated_bytes], [records[0].alloc_calls, records[0].allocated_bytes]);
      measurements[v] = { callsPerQuery: records[0].alloc_calls / iterations,
        bytesPerQuery: records[0].allocated_bytes / iterations,
        retainedByteDeltaRange: [Math.min(...records.map(r => r.retained_bytes)), Math.max(...records.map(r => r.retained_bytes))] };
    }
    summaries.push({ name, degree, lifecycle, iterations, observations: rows.length, measurements });
  }
}
// Longer fresh-graph churn checks the existing weak retention bound. This is
// memory evidence only; no clock from an instrumented executable is a CPU bench.
const churn = [];
for (const iterations of [100, 1000, 10000]) for (let repeat = 0; repeat < 3; repeat++)
  for (const v of variants) churn.push({ repeat, ...await query(v, 'tiny-self', 16, 'fresh', iterations) });
writeFileSync(resolve(here, 'polynomial-facts-allocation-bounded-summary.json'), JSON.stringify({ started,
  finished: new Date().toISOString(), binaries, summaries, churn,
  limits: 'Rust successful allocation/reallocation request totals and live requested-byte deltas, not peak RSS, allocator overhead, native allocations or CPU measurements. Prepared operands remain alive after eight preconditioning queries. Live-byte occupancy is address-dependent and summarized as a range; request totals must still agree. Three fresh-churn lengths independently check the existing native32-by72-byte weak-cache bound. Other test processes do not share this process-private allocator.' }, null, 2) + '\n', { flag: 'wx' });
console.log(JSON.stringify({ groups: summaries.length, observations: summaries.length * 18, churnObservations: churn.length }));
