import assert from 'node:assert/strict';
import { createHash } from 'node:crypto';
import { readFileSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';

const dir = dirname(fileURLToPath(import.meta.url));
const read = name => readFileSync(resolve(dir, name), 'utf8');
const hashes = {
  '../Plume/report.pdf': '3707551d0aba1e26f5729684d86ab4d9bb344b609553d6ae283c9d78df416952',
  '../Plume/report.txt': '1e08410b19402f9baaebdac0e5e10005be4de58032f06e2fb4349de5ecb24e8e',
  'late_oracle.rs': 'e37beb789ee61d654f5d8f4b221afd1f5425a26ffcb8c82160c33719a7d9a10a',
  'report-logistic-table.tsv': '98c19aa01511f8708bb8cc6e186dcde0342383134d0fb895b6c4be3cf9f857eb',
};
for (const [name, expected] of Object.entries(hashes)) {
  assert.equal(createHash('sha256').update(readFileSync(resolve(dir, name))).digest('hex'), expected, name);
}
// The ledger records human visual review, not something this script can prove.
assert(read('../PLUME_REPORT_COVERAGE.tsv').includes('physical pages1-133 of133'));

// Integer-only outward enclosure, independent of the MPFR oracle. Every bound
// is an integer multiple of 2^-bits. The polynomial is not monotone on [0,1],
// so use dependency-safe interval products rather than endpoint substitution.
function logisticBounds(bits, iterations) {
  const scale = 1n << BigInt(bits);
  let lo = 43n * scale / 64n, hi = lo;
  for (let k = 0; k < iterations; k++) {
    const lower = 4n * lo * (scale - hi);
    const upper = 4n * hi * (scale - lo);
    lo = lower / scale;
    hi = (upper + scale - 1n) / scale;
    if (hi > scale) hi = scale;
    assert(0n <= lo && lo <= hi && hi <= scale);
  }
  return { lo, hi, scale };
}

let exactChecks = 0;
// Validate enclosure against an unrounded, expanded rational recurrence.
// Deliberately stop at12: exact numerator/denominator sizes double each step.
for (const bits of [1024, 2048, 4096]) {
  let numerator = 43n, denominator = 64n;
  for (let k = 0; k <= 12; k++) {
    const { lo, hi, scale } = logisticBounds(bits, k);
    assert(lo * denominator <= numerator * scale);
    assert(numerator * scale <= hi * denominator);
    exactChecks++;
    if (k < 12) {
      numerator = 4n * numerator * denominator - 4n * numerator * numerator;
      denominator *= denominator;
    }
  }
}
const rows = read('report-logistic-table.tsv').trim().split('\n').map(line => line.split('\t'));
assert.equal(rows.length, 10);
assert.deepEqual(rows.map(row => Number(row[4])), [1, 5, 10, 15, 20, 25, 30, 40, 50, 60]);
let tableChecks = 0;
for (const row of rows) {
  assert.equal(row.length, 12);
  assert.deepEqual(row.slice(0, 4), ['printed-correct', 'greedy', '43', '64']);
  const center = BigInt(row[6]), denominator = BigInt(row[7]);
  assert.equal(denominator, 1000000n);
  assert.equal(row[8], '1'); assert.equal(row[9], '2000000');
  for (const bits of [1024, 2048, 4096]) {
    const { lo, hi, scale } = logisticBounds(bits, Number(row[4]));
    // The whole enclosure must lie strictly inside this six-decimal rounding
    // cell, not merely intersect it. No rounding-tie convention is involved.
    assert((2n * center - 1n) * scale < 2n * denominator * lo);
    assert(2n * denominator * hi < (2n * center + 1n) * scale);
    tableChecks++;
  }
}
let badicCounterexamples = 0;
for (const base of [2n, 3n, 10n]) {
  for (let n = 1n; n <= 12n; n++) {
    const c = base ** n;
    assert.equal(c - base ** n, 0n); // |1-c/B^n| = 0 < B^-n.
    assert.equal(c / (base ** n), 1n);
    assert(c * (base ** n) > 1n); // Printed c/(B^-n) = B^(2n).
    badicCounterexamples++;
  }
}
assert(read('report-logistic-table-oracle.log').includes('TOTAL logistic\tchecks=10\tfailures=0'));
assert(read('report-logistic-hyper.log').includes('PASS Hyper logistic k=60, four precision/history queries'));
console.log(JSON.stringify({
  primaryReportVisualPages: 133,
  exactRationalEnclosureSelfChecks: exactChecks,
  independentIntegerTableChecks: tableChecks,
  mpfrTableChecks: 10,
  badicCounterexamples,
  hyperHistoricalBinaryChecks: 4,
  limitations: [
    'Only the printed correct-result column is qualified; historical single/double runs and timings are not reproduced.',
    'The existing late_oracle binary predates the Display fix; its approximation-only check is not a fresh production-build qualification.',
    'Visual read credit is recorded by the human-readable ledger; this validator checks consistency and numerical fixtures, not whether pages were read.',
    'No new performance benchmark or Hyper production change is introduced.',
  ],
}, null, 2));
