import './verify-polynomial-decision.mjs';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
const here = dirname(fileURLToPath(import.meta.url)), workspace = resolve(here, '../../../..');
const read = path => readFileSync(resolve(here, path), 'utf8');
const json = path => JSON.parse(read(path));
const sha = path => createHash('sha256').update(readFileSync(path)).digest('hex');
const manifest = json('polynomial-facts-experiment.json');
for (const [path, hash] of Object.entries(manifest.files)) assert.equal(sha(resolve(here, path)), hash, path);
for (const [path, hash] of Object.entries(manifest.binaries)) assert.equal(sha(path), hash, path);
const prior = json('reuse-consumers.json'), changed = [];
let sources = 0;
for (const { crate, files } of prior.crates) for (const file of files) {
  const path = `${crate}/${file.path}`;
  assert.equal(sha(resolve(here, 'reuse-consumers', path)), file.sha256);
  if (process.argv.includes('--baseline-live')) assert.equal(sha(resolve(workspace, path)), file.sha256);
  if (sha(resolve(here, 'polynomial-facts-trial', path)) !== file.sha256) changed.push(path);
  sources++;
}
assert.equal(sources, 773); assert.deepEqual(changed, ['hypersolve/src/resultant.rs']);
const inventory = json('inventory.json'), coverage = json('coverage.json');
assert.equal(manifest.reads.length, 32);
let readLines = 0;
for (const entry of manifest.reads) {
  assert.deepEqual(coverage.find(c => c.repo === entry.repo && c.path === entry.path), entry);
  const file = inventory.sources.find(s => s.repo === entry.repo).files.find(f => f.path === entry.path);
  assert.deepEqual(entry.ranges, [[1, file.lines]]);
  assert.equal(sha(resolve(workspace, 'exact-real-references', entry.repo, entry.path)), file.sha256);
  readLines += file.lines;
}
assert.equal(readLines, 2959);
const result = tag => json(`results/polynomial-facts-${tag}.json`);
for (const tag of manifest.gates) assert.equal(result(tag).code, tag === 'allocation-run' ? 1 : 0, tag);
for (const tag of ['consumer-hypercurve-debug', 'tests-debug', 'tests-release', 'state-debug', 'state-release',
  'clippy', 'fmt', 'wasm', 'probe-memcheck', 'native-memcheck', 'allocation-bounded-run', 'app-size-run']) assert(manifest.gates.includes(tag));
assert(result('tests-release').args.includes('--release')); assert(!result('tests-debug').args.includes('--release'));
for (const tag of ['probe-debug', 'probe-release', 'probe-memcheck'])
  assert.equal(read(`results/polynomial-facts-${tag}.stdout`), read('results/polynomial-decision-trial-debug-final.stdout'));
const testNames = tag => [...read(`results/${tag}.stdout`).matchAll(/^test (.+) \.\.\. (ok|ignored|FAILED).*$/gm)].map(m => `${m[1]}:${m[2]}`).sort();
for (const profile of ['debug', 'release']) {
  assert.deepEqual(testNames(`polynomial-facts-tests-${profile}`), testNames('polynomial-decision-trial-tests-release-corrected'));
  const suites = [...read(`results/polynomial-facts-tests-${profile}.stdout`).matchAll(/test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;/g)];
  assert.equal(suites.length, 7); assert.equal(suites.reduce((n, m) => n + Number(m[1]), 0), 800);
  assert(suites.every(m => m[2] === '0' && m[3] === '0'));
  assert.deepEqual(JSON.parse(read(`results/polynomial-facts-state-${profile}.stdout`)),
    { suite: 'polynomial-facts-state', queries: 1090, unchanged_serializations: 64, workers_per_case: 4 });
}
assert.deepEqual(testNames('polynomial-facts-consumer-hypercurve-debug'), testNames('reuse-consumer-hypercurve-debug'));
const curveSuites = [...read('results/polynomial-facts-consumer-hypercurve-debug.stdout').matchAll(/test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;/g)];
assert.equal(curveSuites.length, 45);
assert.equal(curveSuites.reduce((n, m) => n + Number(m[1]), 0), 1761);
assert.equal(curveSuites.reduce((n, m) => n + Number(m[3]), 0), 9);
assert(curveSuites.every(m => m[2] === '0'));
for (const tag of ['probe-memcheck', 'native-memcheck']) {
  const text = read(`results/polynomial-facts-${tag}.stderr`);
  assert.match(text, /ERROR SUMMARY: 0 errors/);
  assert(!/definitely lost:\s+[1-9]|indirectly lost:\s+[1-9]|possibly lost:\s+[1-9]/.test(text));
}
assert.match(read('results/polynomial-facts-native-memcheck.stderr'), /All heap blocks were freed/);
const native = read('results/polynomial-facts-native.stdout');
assert.equal(native, read('results/polynomial-facts-native-memcheck.stdout'));
const nativeRows = native.trim().split('\n');
assert.equal(nativeRows.shift(), 'mode,left_length,right_length,checks,failures');
assert.deepEqual(JSON.parse(nativeRows.pop()), { suite: 'polynomial-arithmetic', cases: 180, checks: 4860, failures: 0 });
let nativeIndex = 0;
for (let mode = 0; mode < 5; mode++) for (const left of [0, 1, 3, 4, 10, 12]) for (const right of [0, 1, 3, 4, 10, 12])
  assert.deepEqual(nativeRows[nativeIndex++].split(',').map(Number), [mode, left, right, 27, 0]);
assert.equal(nativeRows.length, nativeIndex);

const variants = ['baseline', 'trial', 'facts'], groups = [];
for (const name of ['rational-self', 'radical-self', 'log-self', 'log-plus-one', 'tiny-self', 'unknown-leading'])
  for (const degree of [1, 8, 16]) for (const lifecycle of ['fresh', 'retained']) groups.push([name, degree, lifecycle]);
const median = input => { const a = [...input].sort((x, y) => x - y); return (a[Math.floor((a.length - 1) / 2)] + a[Math.floor(a.length / 2)]) / 2; };
const outcome = r => ['rational-self', 'radical-self'].includes(r.case) || (r.variant !== 'baseline' && r.case !== 'unknown-leading');
const cpu = json('polynomial-facts-cpu-summary.json');
const rows = read('results/polynomial-facts-cpu.jsonl').trim().split('\n').map(JSON.parse);
assert.equal(rows.length, 2592); assert.equal(cpu.summaries.length, 36); assert.equal(cpu.cpu, 6);
let seed = 997;
const random = n => { seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0; return seed % n; };
cpu.summaries.forEach((s, group) => {
  assert.deepEqual([s.name, s.degree, s.lifecycle], groups[group]); assert.equal(s.observations, 72);
  assert.equal(s.pilots.length, 3);
  s.pilots.forEach((r, v) => {
    assert.deepEqual([r.case, r.degree, r.lifecycle, r.iterations], [...groups[group], 10]);
    assert.equal(r.known, outcome({ ...r, variant: variants[v] }) ? 10 : 0); assert(r.elapsed_ns > 0);
  });
  assert.equal(s.iterations, Math.max(10, Math.min(10000, Math.ceil(1e7 / Math.max(...s.pilots.map(p => p.elapsed_ns / p.iterations))))));
  const selected = rows.slice(group * 72, (group + 1) * 72);
  for (let block = 0; block < 12; block++) {
    const forward = variants.map((_, i) => variants[(i + block) % 3]);
    const order = [...forward, ...forward.toReversed()];
    selected.slice(block * 6, block * 6 + 6).forEach((r, i) => {
      assert.deepEqual([r.case, r.degree, r.lifecycle, r.block, r.variant, r.iterations], [...groups[group], block, order[i], s.iterations]);
      assert.equal(r.known, outcome(r) ? r.iterations : 0); assert(r.elapsed_ns > 0);
    });
  }
  const measurements = Object.fromEntries(variants.map(v => [v, median(selected.filter(r => r.variant === v).map(r => r.elapsed_ns / r.iterations))]));
  assert.deepEqual(s.measurements, measurements);
  for (const control of ['baseline', 'trial']) {
    const ratios = Array.from({ length: 12 }, (_, block) => {
      const time = v => median(selected.filter(r => r.block === block && r.variant === v).map(r => r.elapsed_ns));
      return time('facts') / time(control);
    });
    const boot = Array.from({ length: 5000 }, () => median(ratios.map(() => ratios[random(12)]))).sort((a, b) => a - b);
    assert.deepEqual(s.comparisons[control], { pairedMedianRatio: median(ratios), pairedMedianBootstrap95: [boot[125], boot[4875]] });
  }
});
const start = Date.parse(cpu.started), finish = Date.parse(cpu.finished);
for (const tag of manifest.gates.filter(t => t !== 'cpu-run')) {
  const r = result(tag);
  assert(Date.parse(r.finished) <= start || Date.parse(r.started) >= finish, `CPU overlap: ${tag}`);
}
for (const b of Object.values(cpu.binaries)) assert.equal(sha(b.path), b.sha256);
const allocation = json('polynomial-facts-allocation-bounded-summary.json');
const allocRows = read('results/polynomial-facts-allocation-bounded.jsonl').trim().split('\n').map(JSON.parse);
assert.equal(allocRows.length, 648); assert.equal(allocation.summaries.length, 36);
function allocationRow(r) {
  assert.equal(r.known, outcome(r) ? r.iterations : 0);
  for (const key of ['alloc_calls', 'allocated_bytes']) assert(Number.isSafeInteger(r[key]) && r[key] >= 0);
  assert(Number.isSafeInteger(r.retained_bytes));
  if (r.lifecycle === 'retained' || r.case === 'rational-self') assert.equal(r.retained_bytes, 0);
  else if (r.case === 'tiny-self') {
    assert(r.retained_bytes >= 0 && r.retained_bytes <= 2304); assert.equal(r.retained_bytes % 72, 0);
  } else assert.equal(r.retained_bytes, 384);
}
allocation.summaries.forEach((s, group) => {
  assert.deepEqual([s.name, s.degree, s.lifecycle], groups[group]); assert.equal(s.iterations, 100); assert.equal(s.observations, 18);
  const selected = allocRows.slice(group * 18, (group + 1) * 18);
  for (let block = 0; block < 3; block++) {
    const forward = variants.map((_, i) => variants[(i + block) % 3]);
    const order = [...forward, ...forward.toReversed()];
    selected.slice(block * 6, block * 6 + 6).forEach((r, i) => {
      assert.deepEqual([r.case, r.degree, r.lifecycle, r.block, r.variant, r.iterations], [...groups[group], block, order[i], 100]);
      allocationRow(r);
    });
  }
  for (const v of variants) {
    const records = selected.filter(r => r.variant === v);
    for (const r of records) assert.deepEqual([r.alloc_calls, r.allocated_bytes], [records[0].alloc_calls, records[0].allocated_bytes]);
    assert.deepEqual(s.measurements[v], { callsPerQuery: records[0].alloc_calls / 100, bytesPerQuery: records[0].allocated_bytes / 100,
      retainedByteDeltaRange: [Math.min(...records.map(r => r.retained_bytes)), Math.max(...records.map(r => r.retained_bytes))] });
  }
});
assert.equal(allocation.churn.length, 27);
let churnIndex = 0;
for (const iterations of [100, 1000, 10000]) for (let repeat = 0; repeat < 3; repeat++) for (const v of variants) {
  const r = allocation.churn[churnIndex++];
  assert.deepEqual([r.case, r.degree, r.lifecycle, r.variant, r.iterations, r.repeat], ['tiny-self', 16, 'fresh', v, iterations, repeat]);
  allocationRow(r);
}
for (const b of Object.values(allocation.binaries)) assert.equal(sha(b.path), b.sha256);
// A failed initial assumption is retained, not relabeled as a passing campaign.
const failedRows = read('results/polynomial-facts-allocation.jsonl').trim().split('\n').map(JSON.parse);
assert.equal(failedRows.length, 450);
const finalFailedGroup = failedRows.slice(-18).filter(r => r.variant === 'trial');
assert.deepEqual([...new Set(finalFailedGroup.map(r => r.retained_bytes))].sort((a,b) => a-b), [2160, 2304]);
assert(finalFailedGroup.every(r => r.alloc_calls === 191800 && r.allocated_bytes === 11472000));
assert.match(read('results/polynomial-facts-allocation-run.stderr'), /AssertionError/);
const app = json('polynomial-facts-app-size-summary.json');
assert.deepEqual(app.artifacts.map(a => a.example), ['basic', 'arrangement']);
for (const a of app.artifacts) for (const b of [...a.baseline, ...a.facts]) assert.equal(sha(b.path), b.sha256);
assert.deepEqual(app.artifacts.map(a => a.facts[1].bytes - a.baseline[1].bytes), [576, 560]);
console.log(JSON.stringify({ checkpoint: 'polynomial fact-first qualification', readFiles: 32, readLines,
  publicCasesPerProfile: 630, newDecisions: 120, preservedUnknownLeadingControls: 60,
  testsPerProfile: 800, stateQueriesPerProfile: 1090, stableSerializationsPerProfile: 64,
  consumerTests: 1761, ignoredConsumerTests: 9, nativeChecks: 4860, cpuObservations: 2592,
  allocationObservations: 648, churnObservations: 27, status: 'Qualified frozen candidate; retention recorded separately. Whole ecosystem remains incomplete.',
  limits: 'Native donor control is scalar-convolution cross-checking, not a separate arbitrary-real oracle or branch-coverage instrument. Pointer-dependent live-byte totals have bounded ranges, not determinism. Different proof outcomes are not equal-work timings. CPU corpus shares lower scalar nodes; serialized/concurrent qualification is correctness only. Nine ignored downstream tests unrun. WASM compile-only. Example size changes are not full Alumina sizes.' }));
