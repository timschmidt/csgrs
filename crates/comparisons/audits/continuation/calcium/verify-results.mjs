import assert from 'node:assert/strict';
import { readFileSync, writeFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const here = dirname(fileURLToPath(import.meta.url));
const read = name => readFileSync(resolve(here, name), 'utf8');
const baseline = JSON.parse(read('baseline-hyperreal.json'));
for (const file of baseline.files) {
  const bytes = readFileSync(resolve(here, 'baseline-hyperreal', file.path));
  assert.equal(createHash('sha256').update(bytes).digest('hex'), file.sha256,
    `Frozen baseline changed: ${file.path}`);
}
const summaries = [];
for (const [tag, native] of [
  ['hyper-proof-complete-snapshot-debug', false], ['hyper-proof-complete-snapshot-release', false],
  ['flint-proof-native', true],
]) {
  assert.equal(JSON.parse(read(`results/${tag}.json`)).code, 0);
  const rows = read(`results/${tag}.stdout`).trim().split('\n');
  assert.equal(rows.shift(), 'kind,a,d,precision,outcome,elapsed_ns');
  const seen = new Set();
  const counts = {};
  for (const line of rows) {
    const [kind, a, d, p, outcome, elapsed, ...extra] = line.split(',');
    assert.equal(extra.length, 0, `Malformed row: ${line}`);
    assert(Number.isFinite(Number(elapsed)) && Number(elapsed) >= 0);
    assert(['1', '2', '3'].includes(a));
    assert(['2', '3', '5', '6', '7', '10', '11', '13', '17', '19'].includes(d));
    const prec = Math.abs(Number(p));
    assert([64, 256, 512].includes(prec));
    assert(['algebraic_control', 'log_identity', 'perturbed_control', 'complex_branch_control'].includes(kind));
    assert(native || kind !== 'complex_branch_control');
    if (kind === 'complex_branch_control') assert.equal(prec, 512);
    const key = `${kind}/${a}/${d}/${prec}`;
    assert(!seen.has(key), `Duplicate row: ${key}`);
    seen.add(key);
    const expected = kind.endsWith('branch_control') || kind === 'perturbed_control'
      ? 'NotEqual' : kind === 'log_identity' && !native ? 'Unknown' : 'Equal';
    assert.equal(outcome, expected, key);
    counts[`${kind}:${outcome}`] = (counts[`${kind}:${outcome}`] ?? 0) + 1;
  }
  assert.equal(rows.length, native ? 300 : 270);
  // Uniqueness, bounded domain and total cardinality imply complete Cartesian coverage.
  summaries.push({ tag, rows: rows.length, counts });
}
const tests = read('results/flint-ca-tests.stdout');
assert.equal((tests.match(/PASS/g) ?? []).length, 29);
assert(tests.includes('All tests passed for ca.'));
assert.equal(JSON.parse(read('results/flint-field-tests.json')).code, 0);
const fieldTests = read('results/flint-field-tests.stdout');
assert.equal((fieldTests.match(/PASS/g) ?? []).length, 73);
assert(fieldTests.includes('All tests passed for ca_ext, ca_field, fmpz_mpoly_q and qqbar.'));
for (const tag of ['flint-proof-memcheck', 'hyper-proof-frozen-memcheck']) {
  assert.equal(JSON.parse(read(`results/${tag}.json`)).code, 0);
  assert(read(`results/${tag}.stderr`).includes('ERROR SUMMARY: 0 errors'));
}
const report = { baselineCommit: baseline.commit, baselineFilesVerified: baseline.files.length,
  summaries, nativeScalarTestFunctionsPassed: 29, nativeSupportingTestFunctionsPassed: 73,
  conclusion: 'Confirmed scoped log-identity completeness gap; no production change yet.',
  limits: 'Query clocks and numeric budgets differ; these runs are not comparative performance benchmarks.' };
writeFileSync(resolve(here, 'qualification-summary.json'), JSON.stringify(report, null, 2) + '\n');
console.log(JSON.stringify(report));
