import './verify-root-exp-reuse-checkpoint.mjs';
import { readFileSync, readdirSync, realpathSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve, relative } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';

const here = dirname(fileURLToPath(import.meta.url));
const read = p => readFileSync(resolve(here, p), 'utf8');
const json = p => JSON.parse(read(p));
const hash = p => createHash('sha256').update(readFileSync(resolve(here, p))).digest('hex');
const manifest = json('reuse-qualification-experiment.json');
for (const [p, expected] of Object.entries({ ...manifest.sourceHashes, ...manifest.evidenceHashes }))
  assert.equal(hash(p), expected, p);
function capture(tag, code = 0) {
  const r = json(`results/${tag}.json`);
  assert.equal(r.code, code, tag); assert.equal(r.signal, null, tag);
  assert(Date.parse(r.finished) >= Date.parse(r.started), tag);
  return read(`results/${tag}.stdout`);
}
for (const [tag, code] of Object.entries(manifest.expectedCaptureCodes)) capture(tag, code);
for (const profile of ['debug', 'release']) {
  assert.deepEqual(JSON.parse(capture(`root-exp-reuse-oracle-${profile}`)), { suite: 'oracle', inputs: 43, enclosure_checks: 15050 });
  assert.deepEqual(JSON.parse(capture(`root-exp-reuse-state-${profile}`)), { suite: 'state', enclosure_checks: 261 });
  // Prior checkpoint verifies each expected case/sign, not merely a line count.
  assert.equal(capture(`root-exp-reuse-public-${profile}`), capture(`root-exp-sign-public-${profile}`));
  assert.equal(capture(`root-exp-reuse-tiny-${profile}`), capture(`root-exp-sign-tiny-trial-${profile}`));
}
const concurrent = { suite: 'concurrent-proof-state', sign_queries: 1732, enclosure_checks: 3456, workers_per_case: 8 };
for (const tag of ['reuse-proof-state-release', 'reuse-proof-state-public-debug', 'reuse-proof-state-public-release'])
  assert.deepEqual(JSON.parse(capture(tag)), concurrent);
assert.deepEqual(JSON.parse(capture('reuse-proof-state-memcheck', 99)), concurrent);
assert(read('results/reuse-proof-state-debug.stderr').includes('private method'));
assert(read('results/reuse-proof-state-release.stderr').includes('Blocking waiting for file lock'));
assert(read('results/reuse-consumer-hyperlattice-release.stderr').includes('ccache: error: Read-only file system'));

const snapshot = json('reuse-consumers.json');
assert.deepEqual(snapshot.crates, json('sign-consumers.json').crates);
assert.equal(realpathSync(resolve(here, 'reuse-consumers/hyperreal')), snapshot.scalar);
function filesBelow(root) {
  const paths = [];
  function walk(dir) {
    for (const e of readdirSync(dir, { withFileTypes: true })) {
      const p = resolve(dir, e.name);
      if (e.isDirectory()) walk(p); else { assert(e.isFile(), p); paths.push(relative(root, p)); }
    }
  }
  walk(root); return paths.sort();
}
let snapshotFiles = 0;
for (const crate of snapshot.crates) {
  const root = resolve(here, 'reuse-consumers', crate.crate);
  assert.deepEqual(filesBelow(root), crate.files.map(f => f.path).sort());
  for (const f of crate.files) {
    assert.equal(hash(resolve(root, f.path)), f.sha256);
    assert.equal(readFileSync(resolve(root, f.path)).length, f.bytes);
    snapshotFiles++;
  }
}
assert.equal(snapshotFiles, 773);
const metadata = JSON.parse(capture('reuse-consumer-metadata-all-features'));
for (const name of ['hyperreal', ...snapshot.crates.map(c => c.crate)]) {
  const packages = metadata.packages.filter(p => p.name === name); assert.equal(packages.length, 1);
  assert.equal(realpathSync(packages[0].manifest_path), realpathSync(resolve(here, 'reuse-consumers', name, 'Cargo.toml')));
}
function suites(text) {
  const result = []; let current;
  for (const line of text.split('\n')) {
    const start = /^running (\d+) tests?$/.exec(line);
    if (start) { assert(!current); current = { expected: +start[1], names: [] }; }
    const test = /^test (.+) \.\.\. (ok|ignored)(?:, (.*))?$/.exec(line);
    if (test) { assert(current); current.names.push([test[1], test[2], test[3] || '']); }
    const end = /^test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;/.exec(line);
    if (end) {
      assert(current); const [passed, failed, ignored, measured, filtered] = end.slice(1).map(Number);
      assert.deepEqual([failed, measured, filtered], [0, 0, 0]); assert.equal(passed + ignored, current.expected);
      assert.equal(current.names.length, current.expected); assert.equal(new Set(current.names.map(n => n[0])).size, current.expected);
      assert.equal(current.names.filter(n => n[1] === 'ok').length, passed);
      result.push({ passed, ignored, names: current.names.sort((a, b) => a[0].localeCompare(b[0])) }); current = undefined;
    }
  }
  assert(!current); assert(result.length); return result;
}
for (const [crate, total] of Object.entries(manifest.consumerTests)) {
  const suffix = crate === 'hyperlattice' ? 'release-unsandboxed' : 'debug';
  const tag = `reuse-consumer-${crate}-${suffix}`;
  const baselineTag = crate === 'hypercurve' ? 'sign-consumer-hypercurve-baseline-debug-storage-retry'
    : `sign-consumer-${crate}-baseline-${suffix}`;
  const result = suites(capture(tag));
  assert.deepEqual(result, suites(capture(baselineTag)), `${crate} test membership/status/reasons`);
  assert.equal(result.reduce((n, s) => n + s.passed, 0), total);
  assert.equal(result.reduce((n, s) => n + s.ignored, 0), manifest.ignoredConsumerTests[crate] || 0);
  const ran = [crate, ...[...read(`results/${tag}.stderr`).matchAll(/^\s+Running tests\/([^ ]+)\.rs /gm)].map(m => m[1])].sort();
  const expected = metadata.packages.find(p => p.name === crate).targets
    .filter(t => t.kind.includes('lib') || t.kind.includes('test')).map(t => t.name).sort();
  assert.deepEqual(ran, expected, `${crate} Cargo targets`);
}
for (const tag of ['reuse-consumer-probe-release', 'reuse-consumer-probe-memcheck'])
  assert.equal(capture(tag), capture('sign-consumer-probe-sign-release'));
for (const tag of ['reuse-memory-memcheck', 'reuse-consumer-probe-memcheck']) {
  const mem = read(`results/${tag}.stderr`);
  for (const s of ['ERROR SUMMARY: 0 errors', 'definitely lost: 0 bytes', 'indirectly lost: 0 bytes', 'possibly lost: 0 bytes']) assert(mem.includes(s), tag);
}
assert.deepEqual(JSON.parse(capture('reuse-thread-control-memcheck', 99)), { suite: 'std-scoped-thread-control', scopes: 18 });
for (const tag of ['reuse-proof-state-memcheck', 'reuse-thread-control-memcheck']) {
  const mem = read(`results/${tag}.stderr`);
  for (const s of ['ERROR SUMMARY: 1 errors from 1 contexts', 'definitely lost: 0 bytes', 'indirectly lost: 0 bytes',
    'possibly lost: 48 bytes in 1 blocks', 'suppressed: 0 bytes', 'std::thread::current::init_current']) assert(mem.includes(s), tag);
  assert.equal([...mem.matchAll(/are possibly lost in loss record/g)].length, 1);
}

const summary = json('reuse-memory-summary.json');
const rows = read('results/reuse-memory.jsonl').trim().split('\n').map(JSON.parse);
assert.equal(rows.length, 162); assert.equal(summary.observations, 162); assert.deepEqual(summary.rows, rows);
assert(Date.parse(summary.finished) >= Date.parse(summary.started));
const keys = new Set();
let maximumPerWorker = 0;
for (const r of rows) {
  const key = [r.depth, r.threads, r.rounds, r.repetition, r.variant].join(','); assert(!keys.has(key)); keys.add(key);
  assert([1, 8].includes(r.depth)); assert([1, 8, 32].includes(r.threads)); assert([1, 64, 512].includes(r.rounds));
  assert([0, 1, 2].includes(r.repetition)); assert(['baseline', 'sign', 'reuse'].includes(r.variant));
  assert.equal(r.queries, r.threads * r.rounds); assert.equal(r.equal, r.variant === 'baseline' ? 0 : r.queries);
  for (const key of ['worker_before', 'worker_after', 'process_before', 'process_after']) {
    assert.equal(r[key].length, 2); assert(r[key].every(n => Number.isSafeInteger(n) && n >= 0));
  }
  assert.equal(r.worker_retained_bytes, r.worker_after[0] - r.worker_before[0]);
  assert.equal(r.worker_retained_blocks, r.worker_after[1] - r.worker_before[1]);
  assert.equal(r.post_exit_bytes, r.process_after[0] - r.process_before[0]);
  assert.equal(r.post_exit_blocks, r.process_after[1] - r.process_before[1]);
  assert.deepEqual([r.post_exit_bytes, r.post_exit_blocks], [0, 0]);
  assert.equal(r.peak_over_worker_start, r.peak_bytes - r.worker_before[0]);
  assert(r.peak_over_worker_start >= r.worker_retained_bytes);
  if (r.variant !== 'reuse') assert.deepEqual([r.worker_retained_bytes, r.worker_retained_blocks], [0, 0]);
  else {
    assert(r.worker_retained_blocks >= 0 && r.worker_retained_blocks <= 32 * r.threads);
    assert.equal(r.worker_retained_bytes, 72 * r.worker_retained_blocks);
    maximumPerWorker = Math.max(maximumPerWorker, r.worker_retained_bytes / r.threads);
    if (r.depth === 8 && r.threads === 32 && r.rounds === 512)
      assert.deepEqual([r.worker_retained_bytes, r.worker_retained_blocks], [73728, 1024]);
  }
}
assert.equal(maximumPerWorker, 2304);
const elf = {};
for (const variant of ['baseline', 'sign', 'reuse']) {
  const binary = summary.binaries[variant]; assert.equal(hash(binary.path), binary.sha256);
  const line = capture(`reuse-memory-elf-${variant}`).split('\n').find(l => /^\s+TLS\s/.test(l)); assert(line);
  const fields = line.trim().split(/\s+/);
  elf[variant] = { fileBytes: Number(fields[4]), memoryBytes: Number(fields[5]), alignment: Number(fields[7]) };
  assert.equal(elf[variant].alignment, 8);
}
assert.deepEqual(elf.baseline, elf.sign);
assert.equal(elf.reuse.fileBytes - elf.sign.fileBytes, 400);
assert.equal(elf.reuse.memoryBytes - elf.sign.memoryBytes, 400);
const coverage = json('coverage.json'), inventory = json('inventory.json');
for (const expected of manifest.donorTestCoverage)
  assert.deepEqual(coverage.find(c => c.repo === expected.repo && c.path === expected.path), expected);
const donorTests = {};
for (const source of inventory.sources) {
  const tests = source.files.filter(f => /^(src\/)?ca\/test\/[^/]+\.c$/.test(f.path));
  for (const f of tests) assert.deepEqual(coverage.find(c => c.repo === source.repo && c.path === f.path)?.ranges, [[1, f.lines]]);
  donorTests[source.repo] = { files: tests.length, lines: tests.reduce((n, f) => n + f.lines, 0) };
}
assert.deepEqual(donorTests, { calcium: { files: 29, lines: 3400 }, flint: { files: 30, lines: 3225 } });
assert.deepEqual(coverage.find(c => c.repo === 'flint' && c.path === 'src/ca.h')?.ranges, [[1, 531]]);
console.log(JSON.stringify({ checkpoint: 'v3 scalar/consumer and live-memory qualification',
  donorTests, scalarChecksPerProfile: manifest.scalarChecksPerProfile, concurrent,
  passedConsumerTests: 3309, ignoredConsumerTests: 9, memoryObservations: rows.length,
  maxRequestedRetainedBytesPerWorker: maximumPerWorker, linkedTlsByteDelta: 400,
  scopedThreadMemcheck: '48-byte possible loss also reproduced in std-only control; unsuppressed',
  status: manifest.status, limits: manifest.limits }));
