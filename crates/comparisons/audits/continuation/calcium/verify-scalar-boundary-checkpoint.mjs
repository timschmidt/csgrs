import './verify-root-exp-proof-checkpoint.mjs';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
const here = dirname(fileURLToPath(import.meta.url));
const read = p => readFileSync(resolve(here, p), 'utf8');
const json = p => JSON.parse(read(p));
const hash = p => createHash('sha256').update(readFileSync(resolve(here, p))).digest('hex');
const record = json('scalar-boundary-experiment.json');
for (const [p, h] of Object.entries({ ...record.sourceHashes, ...record.evidenceHashes })) assert.equal(hash(p), h, p);
for (const [tag, code] of [['scalar-boundary-compile', 0], ['scalar-boundary-native', 1], ['scalar-boundary-memcheck', 1]]) {
  assert.equal(json(`results/${tag}.json`).code, code);
  assert.equal(json(`results/${tag}.json`).signal, null);
}
const native = read('results/scalar-boundary-native.stdout');
assert.equal(read('results/scalar-boundary-memcheck.stdout'), native);
const lines = native.trim().split('\n');
assert.equal(lines.shift(), 'kind,input,state,outcome,passed');
const rows = lines.slice(0, 30).map(s => s.split(','));
assert.equal(lines.length, 33);
const names = ['minus-one', 'minus-two', 'minus-third', 'minus-sqrt-two', 'zero', 'one', 'i',
  'minus-i', 'one-plus-i', 'one-minus-i', 'negative-infinity', 'positive-infinity', 'unknown', 'undefined', 'unsigned-infinity'];
const failed = ['minus-one', 'minus-two', 'minus-third', 'negative-infinity'];
const map = new Map(rows.map(r => [r.slice(0, 3).join(','), r]));
assert.equal(map.size, 30);
for (const name of names) for (const state of ['separate', 'in-place']) {
  const row = map.get(['phase', name, state].join(','));
  assert(row);
  assert.deepEqual(row.slice(3), failed.includes(name) ? ['NotEqual', '0'] : ['Equal', '1']);
}
assert.deepEqual(lines.slice(30).map(JSON.parse), [
  { suite: 'phase', cases: 30, failed: 8 }, { suite: 'binary64', cases: 10013, failed: 0 },
  { suite: 'rounding', cases: 84, failed: 0 },
]);
const memcheck = read('results/scalar-boundary-memcheck.stderr');
assert(memcheck.includes('ERROR SUMMARY: 0 errors'));
assert(memcheck.includes('All heap blocks were freed'));
for (const profile of ['debug', 'release'])
  assert(read(`results/log-baseline-full-${profile}.stdout`).includes('atan2_negative_x_axis_is_pi ... ok'));
const inventory = json('inventory.json'), coverage = json('coverage.json');
for (const source of inventory.sources) {
  for (const stem of ['arg', 'ceil', 'conj', 'floor', 'im', 're', 'set_d', 'set_d_d', 'sgn']) {
    const path = `${source.repo === 'flint' ? 'src/' : ''}ca/${stem}.c`;
    const file = source.files.find(f => f.path === path);
    assert.deepEqual(coverage.find(c => c.repo === source.repo && c.path === path)?.ranges, [[1, file.lines]]);
  }
  const path = `${source.repo === 'flint' ? 'src/' : ''}qqbar/log_pi_i.c`;
  assert.deepEqual(coverage.find(c => c.repo === source.repo && c.path === path)?.ranges, [[1, 33]]);
}
assert.deepEqual(coverage.find(c => c.repo === 'flint' && c.path === 'src/arf/set.c')?.ranges, [[1, 210]]);
console.log(JSON.stringify({ checkpoint: 'phase, rounding and binary64 boundaries',
  phase: record.phase, binary64: record.binary64, rounding: record.rounding,
  memcheck: 'zero errors; all blocks freed; process exit 1 is mathematical failure',
  limits: record.limits }));
