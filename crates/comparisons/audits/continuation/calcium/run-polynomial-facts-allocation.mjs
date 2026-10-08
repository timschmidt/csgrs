import { spawn } from 'node:child_process';
import { readFileSync, writeFileSync, appendFileSync, copyFileSync, constants, statSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here = dirname(fileURLToPath(import.meta.url));
const target = '/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse';
const snapshots = '/tmp/calcium-polynomial-facts.ZX8NqH';
async function command(file, args) {
  return new Promise((ok, fail) => {
    const child = spawn(file, args, { stdio: ['ignore', 'pipe', 'pipe'] });
    let stdout = '', stderr = '';
    child.stdout.on('data', s => stdout += s); child.stderr.on('data', s => stderr += s);
    child.on('error', fail);
    child.on('close', code => code === 0 ? ok(stdout) : fail(Error(`${file}: ${code}: ${stderr}`)));
  });
}
const variants = ['baseline', 'trial', 'facts'], binaries = {};
for (const variant of variants) {
  const tag = `polynomial-facts-allocation-build-${variant}`;
  await command(process.execPath, [resolve(here, 'capture.mjs'), tag, here, 'env',
    `CARGO_TARGET_DIR=${target}`, 'CARGO_INCREMENTAL=0', 'CARGO_BUILD_JOBS=2',
    'cargo', 'build', '--offline', '--release', '--manifest-path', resolve(here, `polynomial-allocation-${variant}/Cargo.toml`)]);
  const path = resolve(snapshots, `polynomial-allocation-${variant}`);
  copyFileSync(resolve(target, 'release', `calcium-polynomial-allocation-${variant}`), path, constants.COPYFILE_EXCL);
  binaries[variant] = { path, bytes: statSync(path).size,
    sha256: createHash('sha256').update(readFileSync(path)).digest('hex') };
}
const output = resolve(here, 'results/polynomial-facts-allocation.jsonl');
writeFileSync(output, '', { flag: 'wx' });
const started = new Date().toISOString(), summaries = [];
for (const name of ['rational-self', 'radical-self', 'log-self', 'log-plus-one', 'tiny-self', 'unknown-leading']) {
  for (const degree of [1, 8, 16]) for (const lifecycle of ['fresh', 'retained']) {
    const rows = [], iterations = 100;
    for (let block = 0; block < 3; block++) {
      const forward = variants.map((_, i) => variants[(i + block) % 3]);
      for (const variant of [...forward, ...forward.toReversed()]) {
        const r = { variant, block, ...JSON.parse(await command(binaries[variant].path, [name, String(degree), lifecycle, String(iterations)])) };
        assert.equal(r.case, name); assert.equal(r.degree, degree); assert.equal(r.lifecycle, lifecycle);
        assert.equal(r.iterations, iterations);
        const known = ['rational-self', 'radical-self'].includes(name) || (variant !== 'baseline' && name !== 'unknown-leading');
        assert.equal(r.known, known ? iterations : 0);
        for (const key of ['alloc_calls', 'allocated_bytes']) assert(Number.isSafeInteger(r[key]) && r[key] >= 0);
        assert(Number.isSafeInteger(r.retained_bytes));
        rows.push(r); appendFileSync(output, JSON.stringify(r) + '\n');
      }
    }
    const measurements = {};
    for (const v of variants) {
      const records = rows.filter(r => r.variant === v).map(r => [r.alloc_calls, r.allocated_bytes, r.retained_bytes]);
      // Allocation counts should be deterministic within this single-thread workload.
      for (const record of records) assert.deepEqual(record, records[0]);
      measurements[v] = { callsPerQuery: records[0][0] / iterations,
        bytesPerQuery: records[0][1] / iterations, retainedByteDelta: records[0][2] };
    }
    const s = { name, degree, lifecycle, iterations, observations: rows.length, measurements };
    summaries.push(s); console.log(JSON.stringify(s));
  }
}
writeFileSync(resolve(here, 'polynomial-facts-allocation-summary.json'), JSON.stringify({ started,
  finished: new Date().toISOString(), binaries, summaries,
  limits: 'Instrumented clocks are not CPU evidence. Successful Rust allocation/reallocation requests and requested live-byte delta after100 queries; not peak RSS, allocator overhead or native allocations. Prepared operands remain alive at the final boundary, with eight preconditioning queries. Other test processes may run concurrently but do not share this allocator.' }, null, 2) + '\n', { flag: 'wx' });
