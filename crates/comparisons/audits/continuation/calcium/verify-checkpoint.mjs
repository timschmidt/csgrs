// Checks recorded evidence; it does not turn an inventory into a source read,
// prove all numerical formulas, or qualify an unmeasured consumer workload.
import './verify-results.mjs';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const here = dirname(fileURLToPath(import.meta.url));
const read = path => readFileSync(resolve(here, path), 'utf8');
const json = path => JSON.parse(read(path));
const hash = path => createHash('sha256').update(readFileSync(resolve(here, path))).digest('hex');
function capture(tag, code = 0) {
  const result = json(`results/${tag}.json`);
  assert.equal(result.code, code, tag);
  assert.equal(result.signal, null, tag);
  return read(`results/${tag}.stdout`);
}

for (const [tag, tests] of [
  ['log-v2-final-full-debug', 773], ['log-v2-final-full-release', 773],
  ['log-baseline-full-debug', 764], ['log-baseline-full-release', 764],
  ['log-v2-rational-oracle-debug', 9],
]) assert(capture(tag).includes(`test result: ok. ${tests} passed; 0 failed; 0 ignored;`), tag);

for (const tag of ['log-v2-public-debug', 'log-v2-public-release', 'log-v2-public-memcheck']) {
  const rows = capture(tag).trim().split('\n').slice(1).map(s => s.split(','));
  assert.equal(rows.length, 270, tag);
  const keys = new Set(rows.map(r => r.slice(0, 4).join(',')));
  assert.equal(keys.size, 270, tag);
  for (const a of [1, 2, 3]) for (const d of [2, 3, 5, 6, 7, 10, 11, 13, 17, 19])
    for (const p of [-64, -256, -512]) for (const kind of ['algebraic_control', 'log_identity', 'perturbed_control'])
      assert(keys.has([kind, a, d, p].join(',')), tag);
  for (const row of rows) assert.equal(row[4], row[0] === 'perturbed_control' ? 'NotEqual' : 'Equal', tag);
}

const special = [];
for (const tag of ['special-hyper-debug', 'special-hyper-release', 'special-flint-native', 'special-flint-memcheck']) {
  const rows = capture(tag).trim().split('\n').slice(1).map(s => s.split(','));
  assert.equal(rows.length, 126, tag);
  const keys = new Set(rows.map(r => [r[0], r[1], Math.abs(Number(r[2])), r[3]].join(',')));
  assert.equal(keys.size, 126, tag);
  for (const kind of ['complement', 'erf_odd', 'erfc_reflection']) for (let c = 0; c < 7; c++)
    for (const p of [64, 256, 512]) for (const control of ['identity', 'perturbed'])
      assert(keys.has([kind, c, p, control].join(',')), tag);
  const counts = {};
  for (const [kind, c, p, control, outcome] of rows) {
    const expected = control === 'perturbed' ? 'NotEqual' :
      tag.includes('flint') || c === '2' ? 'Equal' : 'Unknown';
    assert.equal(outcome, expected, `${tag}:${kind}:${c}:${p}:${control}`);
    counts[outcome] = (counts[outcome] ?? 0) + 1;
  }
  special.push({ tag, rows: rows.length, counts });
}

const cases = ['log-identity', 'algebraic-identity', 'log-near-nonzero', 'log-near-unknown',
  'log-algebraic-near-unknown', 'log-multiquadratic-unknown', 'log-transcendental-unknown',
  'nonlog-unknown', 'ordinary-log', 'ordinary-rational', 'exp-cancellation'];
const unknownBoth = ['log-multiquadratic-unknown', 'log-transcendental-unknown', 'nonlog-unknown'];
const benchmark = [];
for (const mode of ['cpu', 'alloc']) {
  capture(`log-v2-expanded-${mode}`);
  const summary = json(`log-paired-v2-expanded-${mode}-summary.json`);
  assert.equal(summary.summaries.length, 22);
  const rows = read(`results/log-paired-v2-expanded-${mode}.jsonl`).trim().split('\n').map(JSON.parse);
  const blocks = mode === 'cpu' ? 12 : 3;
  assert.equal(rows.length, 22 * 4 * blocks);
  for (const name of cases) for (const lifecycle of ['fresh', 'warm']) {
    const group = rows.filter(r => r.case === name && r.lifecycle === lifecycle);
    assert.equal(group.length, blocks * 4);
    const s = summary.summaries.find(s => s.name === name && s.lifecycle === lifecycle);
    assert(s);
    for (const variant of ['baseline', 'trial']) {
      let outcome = 1;
      if (name === 'algebraic-identity' || (variant === 'trial' && name === 'log-identity')) outcome = 0;
      else if (unknownBoth.includes(name) || (variant === 'baseline' &&
        ['log-identity', 'log-near-unknown', 'log-algebraic-near-unknown'].includes(name))) outcome = 2;
      for (let block = 0; block < blocks; block++) {
        const pair = group.filter(r => r.variant === variant && r.block === block);
        assert.equal(pair.length, 2);
        for (const row of pair) {
          assert.equal(row.iterations, s.iterations);
          assert.deepEqual(row.outcomes, [0, 1, 2].map(i => i === outcome ? row.iterations : 0));
          assert(row.elapsed_ns > 0);
          if (mode === 'cpu') assert.equal(row.alloc_calls, 0);
        }
      }
    }
  }
  benchmark.push({ mode, rows: rows.length, fileByteDelta: summary.binaries.trial.bytes - summary.binaries.baseline.bytes });
}

for (const tag of ['log-v2-public-memcheck', 'special-flint-memcheck']) {
  const err = read(`results/${tag}.stderr`);
  assert(/ERROR SUMMARY: 0 errors/.test(err), tag);
  assert(/All heap blocks were freed/.test(err) ||
    /definitely lost: 0 bytes/.test(err) && /indirectly lost: 0 bytes/.test(err) && /possibly lost: 0 bytes/.test(err), tag);
}
// Preserve, rather than suppress, the test-runner finding. Both baseline and
// trial have the same 48-byte possible-loss stack in Rust's runner channel.
for (const tag of ['log-v2-proof-memcheck', 'log-baseline-runner-memcheck']) {
  capture(tag, 97);
  const err = read(`results/${tag}.stderr`);
  assert(/ERROR SUMMARY: 1 errors from 1 contexts/.test(err), tag);
  assert(/possibly lost: 48 bytes in 1 blocks/.test(err), tag);
  assert(/48 bytes in 1 blocks are possibly lost/.test(err), tag);
  assert(err.includes('test::console::run_tests_console'), tag);
  assert(/definitely lost: 0 bytes/.test(err) && /indirectly lost: 0 bytes/.test(err), tag);
}

const inventory = json('inventory.json');
const coverage = json('coverage.json');
for (const source of inventory.sources) for (const file of source.files) {
  if (!/^(src\/)?ca_(ext|field)\//.test(file.path)) continue;
  const entry = coverage.find(c => c.repo === source.repo && c.path === file.path);
  assert.deepEqual(entry?.ranges, [[1, file.lines]], `${source.repo}:${file.path}`);
}
const baseline = json('baseline-hyperreal.json');
const modified = baseline.files.filter(f => hash(`trial-hyperreal/${f.path}`) !== f.sha256).map(f => f.path).sort();
assert.deepEqual(modified, ['src/computable/node.rs', 'src/computable/node/approximation_queries.rs']);
const trialHashes = Object.fromEntries([...modified, 'src/computable/node/log_relation.rs',
  'src/computable/node/log_relation_tests.rs'].map(path => [path, hash(`trial-hyperreal/${path}`)]));
console.log(JSON.stringify({ special, benchmark, trialHashes,
  limits: 'No production change retained. Unit-runner Memcheck is not a clean gate. Timings with different proof outcomes are not equal-work speedups. Allocation totals are cumulative requested bytes, not peak heap. Source-directory closure is not whole-library audit completion.' }));
