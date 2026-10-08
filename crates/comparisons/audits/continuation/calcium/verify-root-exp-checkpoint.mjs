// Verify successes, known failures, experiment isolation and complete corpus
// membership. This is not an independent certification of human source reading.
import './verify-erf-checkpoint.mjs';
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
const experiment = json('root-exp-experiment.json');
for (const [p, expected] of Object.entries({ ...experiment.sourceHashes, ...experiment.evidenceHashes }))
  assert.equal(hash(p), expected, p);
const baseline = json('baseline-hyperreal.json');
const changed = baseline.files.filter(f => hash(`root-exp-trial-hyperreal/${f.path}`) !== f.sha256).map(f => f.path).sort();
assert.deepEqual(changed, ['src/computable/node/roots_inverse_hyperbolic.rs']);
const sourcePath = changed[0];
const original = read(`baseline-hyperreal/${sourcePath}`), trial = read(`root-exp-trial-hyperreal/${sourcePath}`);
assert.equal(Buffer.byteLength(trial) - Buffer.byteLength(original), 1394);
assert.equal(trial.split('\n').length - original.split('\n').length, 25);
capture('root-exp-trial-source-diff', 1);

function corpus(tag, equal, unknown) {
  const lines = capture(tag).trim().split('\n');
  assert.equal(lines.shift(), 'case,negative,precision,control,outcome');
  const rows = lines.map(s => s.split(','));
  assert.equal(rows.length, 96, tag);
  const map = new Map(rows.map(r => [r.slice(0, 4).join(','), r[4]]));
  assert.equal(map.size, 96, tag);
  for (let c = 0; c < 8; c++) for (const negative of [false, true]) for (const p of [-64, -256, -512]) {
    assert.equal(map.get([c, negative, p, 'perturbed'].join(',')), 'NotEqual', tag);
    assert(['Equal', 'Unknown'].includes(map.get([c, negative, p, 'identity'].join(','))), tag);
  }
  assert.equal(rows.filter(r => r[3] === 'identity' && r[4] === 'Equal').length, equal, tag);
  assert.equal(rows.filter(r => r[3] === 'identity' && r[4] === 'Unknown').length, unknown, tag);
  return rows;
}
for (const tag of ['root-exp-baseline-debug-corrected', 'root-exp-baseline-release']) {
  for (const row of corpus(tag, 12, 36)) if (row[3] === 'identity')
    assert.equal(row[4], ['0', '4'].includes(row[0]) ? 'Equal' : 'Unknown');
}
corpus('root-exp-trial-direct-debug', 30, 18);
for (const tag of ['root-exp-trial-reduced-debug-corrected', 'root-exp-trial-public-release',
  'root-exp-trial-public-memcheck', 'root-exp-flint-native', 'root-exp-flint-memcheck']) corpus(tag, 48, 0);
for (const profile of ['debug', 'release']) {
  assert(capture(`root-exp-trial-all-features-${profile}`).includes('test result: ok. 764 passed; 0 failed; 0 ignored;'));
  for (const variant of ['baseline', 'trial']) {
    const oracleTag = `root-exp-${variant}-oracle-${profile}${profile === 'debug' ? '-public' : ''}`;
    assert.deepEqual(JSON.parse(capture(oracleTag)), { suite: 'oracle', inputs: 43, enclosure_checks: 15050 });
    assert.deepEqual(JSON.parse(capture(`root-exp-${variant}-state-${profile}`)), { suite: 'state', enclosure_checks: 261 });
  }
}
// Preserve harness compile mistakes, not misclassified numerical failures.
capture('root-exp-baseline-debug', 101);
assert(read('results/root-exp-baseline-debug.stderr').includes('unwrap'));
capture('root-exp-trial-reduced-debug', 101);
assert(read('results/root-exp-trial-reduced-debug.stderr').includes('expected `i64`, found `i32`'));
for (const variant of ['baseline', 'trial']) {
  capture(`root-exp-${variant}-oracle-debug`, 101);
  assert(read(`results/root-exp-${variant}-oracle-debug.stderr`).includes('method `shift_left` is private'));
}

const native = capture('trig-state-native', 1);
assert.equal(capture('trig-state-memcheck', 1), native);
const lines = native.trim().split('\n');
assert.equal(lines.shift(), 'kind,function,input,state,expected,actual,passed');
const rows = lines.map(s => s.split(','));
assert.equal(rows.length, 181);
const map = new Map(rows.map(r => [r.slice(0, 4).join(','), r]));
assert.equal(map.size, 181);
let specialFailures = 0;
for (const f of ['tan', 'tan_direct', 'tan_exponential', 'tan_sine_cosine', 'cot'])
  for (const input of ['unknown', 'undefined', 'uinf', 'posinf', 'neginf', 'posiinf', 'negiinf'])
    for (const state of ['zero', 'unknown', 'undefined', 'i', 'in-place']) {
      const row = map.get(['special', f, input, state].join(','));
      assert(row);
      const expected = input === 'unknown' ? 'unknown' : ['posiinf', 'negiinf'].includes(input)
        ? (input === 'posiinf') === (f !== 'cot') ? 'i' : 'minus_i' : 'undefined';
      assert.equal(row[4], expected);
      const wrong = input === 'unknown' && ['zero', 'undefined', 'i'].includes(state)
        || ['undefined', 'uinf'].includes(input) && state === 'unknown';
      assert.equal(row[6], wrong ? '0' : '1');
      assert.equal(row[5], wrong ? expected === 'unknown' ? 'undefined' : 'unknown' : expected);
      specialFailures += Number(wrong);
    }
assert.equal(specialFailures, 25);
for (const input of ['2i', '-2i']) for (const state of ['direct', 'separate', 'in-place']) {
  const row = map.get(['branch', 'atan_logarithm', input, state].join(','));
  const wrong = input === '2i' && state === 'in-place';
  assert.deepEqual(row.slice(4), ['Equal', wrong ? 'NotEqual' : 'Equal', wrong ? '0' : '1']);
}
for (const tag of ['root-exp-trial-public-memcheck', 'root-exp-flint-memcheck', 'trig-state-memcheck']) {
  const err = read(`results/${tag}.stderr`);
  assert(err.includes('ERROR SUMMARY: 0 errors'), tag);
  assert(err.includes('All heap blocks were freed') ||
    /definitely lost: 0 bytes/.test(err) && /indirectly lost: 0 bytes/.test(err) && /possibly lost: 0 bytes/.test(err), tag);
}

const cases = ['exp-third', 'exp-sqrt2', 'exp-sine', 'exp-negative-sine', 'exp-multiradical',
  'exp-large', 'exp-tiny', 'exp-offset', 'exp-opaque-zero', 'ordinary-sqrt', 'ordinary-sine-root', 'perfect-square'];
const lifecycles = ['construct', 'fresh', 'warm', 'refine', 'hot-operand', 'construct-hot'];
const median = values => {
  const a = [...values].sort((x, y) => x - y);
  return (a[Math.floor((a.length - 1) / 2)] + a[Math.floor(a.length / 2)]) / 2;
};
const benchmarks = [];
for (const mode of ['cpu', 'alloc']) {
  const summary = json(`root-exp-paired-${mode}-summary.json`);
  const observations = read(`results/root-exp-paired-${mode}.jsonl`).trim().split('\n').map(JSON.parse);
  const blocks = mode === 'cpu' ? 12 : 3;
  assert.equal(observations.length, 72 * 4 * blocks);
  assert.equal(summary.summaries.length, 72);
  assert.equal(summary.cpu, 6);
  for (const name of cases) for (const lifecycle of lifecycles) {
    const group = observations.filter(r => r.case === name && r.lifecycle === lifecycle);
    const s = summary.summaries.find(r => r.name === name && r.lifecycle === lifecycle);
    assert(s);
    assert.equal(group.length, blocks * 4);
    const pairs = [];
    for (let block = 0; block < blocks; block++) {
      const sums = {};
      for (const variant of ['baseline', 'trial']) {
        const pair = group.filter(r => r.block === block && r.variant === variant);
        assert.equal(pair.length, 2);
        for (const r of pair) {
          assert.equal(r.iterations, s.iterations);
          assert(r.elapsed_ns > 0);
          if (mode === 'cpu') assert.equal(r.alloc_calls, 0);
          if (mode === 'alloc' && name === 'exp-third' && lifecycle === 'fresh') assert(r.alloc_calls > 0);
        }
        sums[variant] = median(pair.map(r => r.elapsed_ns));
      }
      pairs.push(sums.trial / sums.baseline);
    }
    assert.equal(s.pairedMedianRatio, median(pairs));
    for (const variant of ['baseline', 'trial']) for (const [key, raw] of [
      ['ns', 'elapsed_ns'], ['alloc_calls', 'alloc_calls'], ['alloc_bytes', 'allocated_bytes']])
      assert.equal(s[`${variant}_${key}`], median(group.filter(r => r.variant === variant).map(r => r[raw] / s.iterations)));
  }
  benchmarks.push({ mode, observations: observations.length,
    linkedDriverFileByteDelta: summary.binaries.trial.bytes - summary.binaries.baseline.bytes });
}
console.log(JSON.stringify({ checkpoint: 'root/exponential and trig state', changed,
  oracleChecksPerVariantPerProfile: 15050, stateChecksPerVariantPerProfile: 261,
  identityQueries: 48, unequalControls: 48, nativeTrigFailures: 26, benchmarks,
  conclusion: experiment.status, limits: experiment.limits }));
