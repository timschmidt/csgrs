import './verify-reuse-qualification-checkpoint.mjs';
import { readFileSync, statSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here = dirname(fileURLToPath(import.meta.url));
const read = p => readFileSync(resolve(here, p), 'utf8');
const json = p => JSON.parse(read(p));
const hash = p => createHash('sha256').update(readFileSync(resolve(here, p))).digest('hex');
const manifest = json('reuse-cost-experiment.json');
for (const [p, h] of Object.entries({ ...manifest.sourceHashes, ...manifest.evidenceHashes })) assert.equal(hash(p), h, p);
function capture(tag, code = 0) {
  const r = json(`results/${tag}.json`); assert.equal(r.code, code, tag); assert.equal(r.signal, null, tag);
  assert(Date.parse(r.finished) >= Date.parse(r.started)); return read(`results/${tag}.stdout`);
}
for (const [tag, code] of Object.entries(manifest.expectedCaptureCodes)) capture(tag, code);
const median = input => {
  const a = [...input].sort((x, y) => x - y);
  return (a[Math.floor((a.length - 1) / 2)] + a[Math.floor(a.length / 2)]) / 2;
};
const variants = ['baseline', 'sign', 'reuse'];
const phases = ['numeric', 'first-touch'], modes = ['cpu', 'alloc'];
const allCaptures = Object.keys(manifest.expectedCaptureCodes).map(t => json(`results/${t}.json`));
const benchmarks = [];
for (const phase of phases) for (const mode of modes) {
  const stem = `reuse-cost-${phase}-${mode}`, summary = json(`${stem}-summary.json`);
  const rows = read(`results/${stem}.jsonl`).trim().split('\n').map(JSON.parse);
  const cases = phase === 'numeric' ? ['exp-third', 'exp-sqrt2', 'exp-sine', 'exp-negative-sine',
    'exp-multiradical', 'exp-large', 'exp-tiny', 'exp-offset', 'exp-opaque-zero',
    'ordinary-sqrt', 'ordinary-sine-root', 'perfect-square']
    : ['identity-1', 'identity-8', 'identity-32', 'identity-128', 'unresolved-1', 'unresolved-8', 'unresolved-32', 'unresolved-128'];
  const settings = phase === 'numeric' ? ['construct', 'fresh', 'warm', 'refine', 'hot-operand', 'construct-hot']
    : ['independent', 'shared', 'common-tail'];
  const blocks = mode === 'cpu' ? 12 : 3;
  assert.equal(rows.length, cases.length * settings.length * blocks * 6);
  assert.equal(summary.summaries.length, cases.length * settings.length);
  assert.equal(summary.phase, phase); assert.equal(summary.mode, mode); assert.equal(summary.cpu, 6);
  for (const b of Object.values(summary.binaries)) { assert.equal(hash(b.path), b.sha256); assert.equal(statSync(b.path).size, b.bytes); }
  const interval = json(`results/run-${stem}.json`);
  // Confirm known builds, other campaigns and memory checks do not overlap CPU.
  if (mode === 'cpu') for (const other of allCaptures) if (other.tag !== interval.tag) {
    assert(Date.parse(other.finished) <= Date.parse(interval.started) || Date.parse(other.started) >= Date.parse(interval.finished),
      `overlap ${interval.tag} / ${other.tag}`);
  }
  let seed = 17911;
  const random = n => { seed = (Math.imul(seed, 1664525) + 1013904223) >>> 0; return seed % n; };
  for (const name of cases) for (const setting of settings) {
    const candidates = summary.summaries.filter(s => s.name === name && s.setting === setting); assert.equal(candidates.length, 1);
    const s = candidates[0], group = rows.filter(r => r.case === name && r[phase === 'numeric' ? 'lifecycle' : 'sharing'] === setting);
    assert.equal(group.length, blocks * 6); assert.equal(s.observations, group.length); assert.equal(s.pilots.length, 3);
    if (phase === 'numeric') {
      const expected = mode === 'alloc' ? 100 : Math.max(20, Math.min(100000, Math.ceil(1e7 / Math.max(...s.pilots.map(p => p.elapsed_ns / p.iterations)))));
      assert.equal(s.iterations, expected);
    } else assert.equal(s.iterations, mode === 'cpu' ? 32 : 16);
    for (let block = 0; block < blocks; block++) {
      const r = group.filter(r => r.block === block), order = variants.map((_, i) => variants[(i + block) % 3]);
      assert.deepEqual(r.map(r => r.variant), [...order, ...order.toReversed()]);
    }
    const counterKeys = phase === 'numeric' ? ['alloc_calls', 'allocated_bytes'] : ['first_calls', 'first_bytes', 'warm_calls', 'warm_bytes'];
    for (const r of group) {
      assert.equal(r[phase === 'numeric' ? 'iterations' : 'workers'], s.iterations);
      for (const key of counterKeys) { assert(Number.isSafeInteger(r[key]) && r[key] >= 0); if (mode === 'cpu') assert.equal(r[key], 0); }
      if (phase === 'numeric') assert(Number.isSafeInteger(r.elapsed_ns) && r.elapsed_ns > 0);
      else {
        for (const key of ['first_ns', 'warm_ns', 'clock_ns', 'lifecycle_ns']) assert(Number.isSafeInteger(r[key]) && r[key] > 0);
        assert(r.lifecycle_ns >= r.first_ns + r.warm_ns); assert.equal(r.warm_queries_per_worker, 16);
        const expected = name.startsWith('identity') && r.variant !== 'baseline' ? 0 : 2;
        assert.deepEqual(r.outcomes, [0, 1, 2].map(i => i === expected ? s.iterations : 0));
      }
    }
    const clocks = phase === 'numeric' ? ['elapsed_ns'] : ['first_ns', 'warm_ns', 'lifecycle_ns'];
    for (const control of ['baseline', 'sign']) for (const clock of clocks) {
      const ratios = Array.from({ length: blocks }, (_, block) => {
        const get = v => median(group.filter(r => r.block === block && r.variant === v).map(r => r[clock]));
        return get('reuse') / get(control);
      });
      assert.equal(s.comparisons[control][clock].pairedMedianRatio, median(ratios));
      const boot = Array.from({ length: 5000 }, () => median(ratios.map(() => ratios[random(ratios.length)]))).sort((a, b) => a - b);
      assert.deepEqual(s.comparisons[control][clock].pairedMedianBootstrap95, [boot[125], boot[4875]]);
    }
    const keys = phase === 'numeric' ? ['elapsed_ns', ...counterKeys] : ['first_ns', 'warm_ns', 'clock_ns', 'lifecycle_ns', ...counterKeys];
    for (const v of variants) for (const key of keys) assert.equal(s.measurements[v][key],
      median(group.filter(r => r.variant === v).map(r => r[key] / s.iterations / (key.startsWith('warm_') ? 16 : 1))));
    if (mode === 'alloc') {
      const keys = phase === 'numeric' ? counterKeys : ['first_calls', 'first_bytes'];
      for (const key of keys) assert.equal(s.measurements.reuse[key], s.measurements.sign[key], `${name}/${setting}/${key}`);
    }
  }
  benchmarks.push({ phase, mode, observations: rows.length, groups: summary.summaries.length,
    fileBytesVsBaseline: summary.binaries.reuse.bytes - summary.binaries.baseline.bytes,
    fileBytesVsSign: summary.binaries.reuse.bytes - summary.binaries.sign.bytes });
}
const native = capture('vector-state-native', 1); assert.equal(native, capture('vector-state-memcheck', 1));
const lines = native.trim().split('\n'); assert.equal(lines.shift(), 'kind,length,mode,correct');
assert.deepEqual(JSON.parse(lines.pop()), { suite: 'vector-state', negation_cases: 105, negation_failed: 24, zero_cases: 27, zero_failed: 0 });
const map = new Map(lines.map(l => l.split(',')).map(r => [r.slice(0, 3).join(','), r[3]]));
assert.equal(lines.length, 105); assert.equal(map.size, 105);
for (let kind = 0; kind < 7; kind++) for (const len of [0, 1, 2, 5, 16]) for (const mode of ['separate', 'public-in-place', 'raw-in-place'])
  assert.equal(map.get([kind, len, mode].join(',')), mode === 'public-in-place' && kind !== 0 && len !== 0 ? '0' : '1');
const mem = read('results/vector-state-memcheck.stderr'); assert(mem.includes('ERROR SUMMARY: 0 errors') && mem.includes('All heap blocks were freed'));
const inventory = json('inventory.json'), coverage = json('coverage.json');
for (const e of manifest.vectorCoverage) {
  assert.deepEqual(coverage.find(c => c.repo === e.repo && c.path === e.path), e);
  const file = inventory.sources.find(s => s.repo === e.repo).files.find(f => f.path === e.path);
  assert.deepEqual(e.ranges, [[1, file.lines]]);
}
assert.equal(manifest.vectorCoverage.reduce((n, e) => n + e.ranges[0][1], 0), 1595);
const apps = json('reuse-app-size-summary.json'); assert.equal(apps.artifacts.length, 6);
const appSizes = [];
for (const example of ['basic', 'arrangement']) {
  const sizes = {};
  for (const v of variants) {
    const a = apps.artifacts.filter(a => a.variant === v && a.example === example); assert.equal(a.length, 1); assert.equal(a[0].files.length, 2);
    for (const f of a[0].files) { assert.equal(hash(f.path), f.sha256); assert.equal(statSync(f.path).size, f.bytes); }
    assert(a[0].files[1].path.endsWith('.stripped')); assert(a[0].files[1].bytes < a[0].files[0].bytes);
    capture(`reuse-app-run-${v}-${example}`);
    const sizesOut = capture(`reuse-app-size-${v}-${example}`).trim().split('\n').slice(1).map(l => l.trim().split(/\s+/));
    assert.equal(sizesOut.length, 2); assert.deepEqual(sizesOut[0].slice(0, 5), sizesOut[1].slice(0, 5));
    sizes[v] = a[0].files[1].bytes;
  }
  appSizes.push({ example, strippedBytes: sizes, v3VsBaseline: sizes.reuse - sizes.baseline, v3VsV2: sizes.reuse - sizes.sign });
}
assert(json('results/reuse-wasm-all-features.json').args.includes('wasm32-unknown-unknown'));
console.log(JSON.stringify({ checkpoint: 'v3 numerical/first-touch costs, vector source and example sizes', benchmarks,
  appSizes, vectorReads: { files: 22, lines: 1595 }, vectorFailures: 24, vectorZeroControls: 27,
  wasm: 'all-feature library compile passes; not runtime tested', status: manifest.status, limits: manifest.limits }));
