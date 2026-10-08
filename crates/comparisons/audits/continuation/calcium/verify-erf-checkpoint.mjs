// Verify recorded successes AND known failures; neither source coverage nor
// this evidence verifier constitutes a completed ecosystem audit.
import './verify-checkpoint.mjs';
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
const experiment = json('erf-experiment.json');
for (const [p, expected] of Object.entries(experiment.sourceHashes)) assert.equal(hash(p), expected, p);
const baseline = json('baseline-hyperreal.json');
const changed = baseline.files.filter(f => hash(`erf-trial-hyperreal/${f.path}`) !== f.sha256).map(f => f.path).sort();
assert.deepEqual(changed, ['src/computable/approximation/statistics.rs', 'src/computable/node/algebra.rs']);
capture('erf-trial-source-diff', 1); // git diff's normal differences-found status.

for (const tag of ['erf-trial-nested-public', 'erf-trial-public-release', 'erf-trial-public-memcheck']) {
  const lines = capture(tag).trim().split('\n');
  assert.equal(lines.shift(), 'kind,case,precision,control,outcome');
  const rows = lines.map(s => s.split(','));
  assert.equal(rows.length, 126, tag);
  const keys = new Set(rows.map(r => r.slice(0, 4).join(',')));
  assert.equal(keys.size, 126, tag);
  for (const kind of ['complement', 'erf_odd', 'erfc_reflection']) for (let c = 0; c < 7; c++)
    for (const p of [-64, -256, -512]) for (const control of ['identity', 'perturbed'])
      assert(keys.has([kind, c, p, control].join(',')), tag);
  for (const r of rows) assert.equal(r[4], r[3] === 'identity' ? 'Equal' : 'NotEqual', tag);
}
for (const profile of ['debug', 'release']) {
  assert(capture(`erf-trial-all-features-${profile}`).includes('test result: ok. 764 passed; 0 failed; 0 ignored;'));
  for (const variant of ['baseline', 'trial']) {
    assert.deepEqual(JSON.parse(capture(`erf-${variant}-final-oracle-${profile}`)),
      { suite: 'oracle', inputs: 160, enclosure_checks: 5856 });
    assert.deepEqual(JSON.parse(capture(`erf-${variant}-state-expanded-${profile}`)),
      { suite: 'state', enclosure_checks: 270 });
  }
}
for (const variant of ['baseline', 'trial']) {
  const rows = capture(`erf-${variant}-facts`, variant === 'trial' ? 101 : 0).trim().split('\n').map(JSON.parse);
  assert.equal(rows.length, 4);
  for (const [i, r] of rows.entries()) {
    assert.equal(r.case, i);
    assert.equal(r.before, 'Some(Positive)');
    assert.equal(r.after, variant === 'trial' ? 'None' : 'Some(Positive)');
  }
}
// Initial oracle imported an unsigned numerator without its separate sign.
// Preserve that harness failure instead of calling it a Hyper defect.
for (const variant of ['baseline', 'trial']) {
  capture(`erf-${variant}-oracle-debug`, 101);
  assert(read(`results/erf-${variant}-oracle-debug.stderr`).includes('case=0, erfc=false, fresh, p=0'));
}

const native = capture('gamma-special-native', 1);
assert.equal(capture('gamma-special-memcheck', 1), native);
const gammaRows = native.trim().split('\n');
assert.equal(gammaRows.shift(), 'truth_encoding,true=0,false=1,unknown=2');
assert.equal(gammaRows.shift(), 'case,expected,actual,passed');
const expectedGamma = [
  ['positive_infinity', 'positive_infinity', 'undefined', '0'],
  ['negative_infinity', 'undefined', 'positive_infinity', '0'],
  ['unknown', 'unknown', 'positive_infinity', '0'],
  ['undefined', 'undefined', 'positive_infinity', '0'],
  ['unsigned_infinity', 'undefined', 'positive_infinity', '0'],
  ['positive_imaginary_infinity', 'zero', 'positive_infinity', '0'],
  ['negative_imaginary_infinity', 'zero', 'positive_infinity', '0'],
  ...[0, 1, 2].map(i => [`finite_control_${i}`, 'Equal', 'Equal', '1']),
];
assert.deepEqual(gammaRows.map(r => r.split(',')), expectedGamma);
for (const tag of ['gamma-special-memcheck', 'erf-trial-public-memcheck']) {
  const err = read(`results/${tag}.stderr`);
  assert(err.includes('ERROR SUMMARY: 0 errors'), tag);
  assert(err.includes('All heap blocks were freed') ||
    /definitely lost: 0 bytes/.test(err) && /indirectly lost: 0 bytes/.test(err) && /possibly lost: 0 bytes/.test(err), tag);
}

const cases = ['erf-zero', 'erf-third', 'erf-negative', 'erf-tiny', 'erf-tail',
  'erfc-third', 'erfc-negative', 'erfc-tail', 'normal-cdf', 'complement', 'ordinary-add', 'ordinary-sqrt'];
const benchmarks = [];
for (const mode of ['cpu', 'alloc']) {
  capture(`erf-paired-${mode}`);
  const summary = json(`erf-paired-${mode}-summary.json`);
  const rows = read(`results/erf-paired-${mode}.jsonl`).trim().split('\n').map(JSON.parse);
  const blocks = mode === 'cpu' ? 12 : 3;
  assert.equal(rows.length, 48 * 4 * blocks);
  assert.equal(summary.summaries.length, 48);
  for (const name of cases) for (const lifecycle of ['construct', 'fresh', 'warm', 'refine']) {
    const group = rows.filter(r => r.case === name && r.lifecycle === lifecycle);
    const s = summary.summaries.find(r => r.name === name && r.lifecycle === lifecycle);
    assert(s);
    assert.equal(group.length, blocks * 4);
    for (let block = 0; block < blocks; block++) for (const variant of ['baseline', 'trial']) {
      const pair = group.filter(r => r.block === block && r.variant === variant);
      assert.equal(pair.length, 2);
      for (const r of pair) {
        assert.equal(r.iterations, s.iterations);
        assert(r.elapsed_ns > 0);
        if (mode === 'cpu') assert.equal(r.alloc_calls, 0);
        if (mode === 'alloc' && name === 'erf-third' && lifecycle === 'fresh') assert(r.alloc_calls > 0);
      }
    }
  }
  benchmarks.push({ mode, observations: rows.length,
    linkedDriverFileByteDelta: summary.binaries.trial.bytes - summary.binaries.baseline.bytes });
}
console.log(JSON.stringify({ checkpoint: 'erf and Gamma', changed,
  oracleChecksPerVariantPerProfile: 5856, stateChecksPerVariantPerProfile: 270,
  identityQueries: 63, unequalControls: 63, lostSerializedSignFacts: 4,
  nativeGammaFailingSpecialControls: 7, benchmarks,
  conclusion: experiment.status, limits: experiment.limits }));
