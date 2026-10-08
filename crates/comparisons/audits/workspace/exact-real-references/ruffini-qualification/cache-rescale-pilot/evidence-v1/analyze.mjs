import fs from 'node:fs';
import path from 'node:path';
import crypto from 'node:crypto';
import assert from 'node:assert/strict';
import { fileURLToPath } from 'node:url';

const dir = path.dirname(fileURLToPath(import.meta.url));
const root = path.resolve(dir, '../../..');
const evidence = path.join(dir, 'evidence-v1');
const read = p => JSON.parse(fs.readFileSync(path.join(evidence, p), 'utf8'));
const sha = p => crypto.createHash('sha256').update(fs.readFileSync(p)).digest('hex');
const prepared = read('prepared.json');
const measured = read('measurements.json');
for (const [p, hash] of prepared.snapshot.files) assert.equal(sha(path.join(root, p)), hash, p);
for (const [p, hash] of prepared.binaries) assert.equal(sha(path.join(evidence, p)), hash, p);
const old = JSON.parse(fs.readFileSync(path.join(dir, '../square-root-pilot/snapshot.json')));
for (const [p, hash] of old.files) assert.equal(sha(path.join(root, p)), hash, p);
assert.deepEqual(prepared.verification.counts, [149504, 149784, 149784, 149784]);
assert.equal(prepared.verification.baseline_overflow_skips, 280);
assert.equal(measured.rows.length, 1210);
assert.equal(measured.memory.length, 264);
const raw = fs.readdirSync(evidence).filter(p => /^measure-[0-9]{4}\.json$/.test(p)).sort();
assert.equal(raw.length, measured.rows.length + measured.memory.length);
for (const [i, row] of [...measured.rows, ...measured.memory].entries()) {
  const record = read(raw[i]);
  assert.equal(record.status, 0);
  assert.equal(record.error, null);
  assert.equal(record.signal, null);
  assert.equal(record.stderr, '');
  const { round, variant, ...result } = row;
  assert.deepEqual(JSON.parse(record.stdout), result);
  assert.deepEqual(record.args, ['-c', '6', path.join(evidence, row.memory === null ? 'release' : 'memory'), 'bench', row.method, String(row.bits), String(row.gap), row.negative ? 'negative' : 'positive', String(row.reps)]);
  assert.equal(row.method, variant === 'original-control' ? 'original' : variant);
  assert(Number.isSafeInteger(round) && Number.isSafeInteger(row.checksum));
  assert(row.cpu_ns > 0 && row.wall_ns > 0);
}

function expectedChecksum(c, reps) {
  // Independent JS reconstruction of the deterministic signed integer and
  // nearest/ties-up quotient. This checks every recorded checksum, not timings.
  let state = 0xc4087c6a31927d5bn;
  const bytes = [];
  for (let i = 0; i < Math.ceil(c.bits / 8); i++) {
    state = BigInt.asUintN(64, state ^ (state << 13n));
    state = BigInt.asUintN(64, state ^ (state >> 7n));
    state = BigInt.asUintN(64, state ^ (state << 17n));
    bytes.push(Number(state & 255n));
  }
  const last = bytes.length - 1;
  bytes[last] &= 255 >> ((8 - c.bits % 8) % 8);
  bytes[last] |= 1 << ((c.bits - 1) % 8);
  let n = BigInt('0x' + bytes.reverse().map(b => b.toString(16).padStart(2, '0')).join(''));
  if (c.sign === 'negative') n = -n;
  const d = 1n << BigInt(c.gap);
  let q = n / d;
  let r = n % d;
  if (r < 0n) { q--; r += d; }
  if (2n * r >= d) q++;
  const magnitude = q < 0n ? -q : q;
  const bits = magnitude === 0n ? 0 : magnitude.toString(2).length;
  return Number(BigInt(bits + Number(q < 0n)) * BigInt(reps));
}

const median = values => {
  const x = [...values].sort((a, b) => a - b);
  return x.length % 2 ? x[(x.length - 1) / 2] : (x[x.length / 2 - 1] + x[x.length / 2]) / 2;
};
let random = 0x85a33931;
function sample(n) {
  random ^= random << 13;
  random ^= random >>> 17;
  random ^= random << 5;
  return (random >>> 0) % n;
}
function interval(values) {
  const boot = Array.from({ length: 20000 }, () => median(values.map(() => values[sample(values.length)]))).sort((a, b) => a - b);
  return [boot[500], boot[19499]];
}
const timings = [];
const memory = [];
for (const c of measured.cases) {
  const match = row => row.bits === c.bits && row.gap === c.gap && row.negative === (c.sign === 'negative');
  const rows = measured.rows.filter(match);
  assert.equal(rows.length, 55);
  const checksum = expectedChecksum(c, c.reps);
  for (const row of rows) { assert.equal(row.checksum, checksum); assert.equal(row.reps, c.reps); }
  for (const variant of measured.variants) {
    const group = rows.filter(r => r.variant === variant).sort((a, b) => a.round - b.round);
    assert.deepEqual(group.map(r => r.round), Array.from({ length: 11 }, (_, i) => i));
    const ratios = group.slice(1).map(row => row.cpu_ns / rows.find(r => r.round === row.round && r.variant === 'original').cpu_ns);
    timings.push({ ...c, variant, samples: ratios.length, ratio: median(ratios), interval95: interval(ratios), min: Math.min(...ratios), max: Math.max(...ratios), medianCpuNs: median(group.slice(1).map(r => r.cpu_ns)) });
  }
  for (const variant of measured.variants.slice(0, 4)) {
    const rows = measured.memory.filter(r => match(r) && r.variant === variant).sort((a, b) => a.round - b.round);
    assert.deepEqual(rows.map(r => r.round), [0, 1, 2]);
    for (const row of rows) {
      assert.equal(row.reps, 100);
      assert.equal(row.checksum, expectedChecksum(c, 100));
      assert.deepEqual(row.memory, rows[0].memory);
      assert.equal(row.memory.retained_delta, 0);
    }
    memory.push({ bits: c.bits, gap: c.gap, sign: c.sign, variant, ...rows[0].memory, repetitionsPerObservation: 100 });
  }
}
const result = {
  scope: 'Private cache-helper prototype; not a public-caller performance or production-retention qualification.',
  preparedHash: sha(path.join(evidence, 'prepared.json')),
  measurementsHash: sha(path.join(evidence, 'measurements.json')),
  analyzerHash: sha(fileURLToPath(import.meta.url)),
  raw: raw.map(p => [p, sha(path.join(evidence, p))]),
  verifiedHyperFiles: old.files.length,
  timingObservations: measured.rows.length,
  measuredObservationsAfterWarmup: measured.rows.filter(r => r.round > 0).length,
  memoryObservations: measured.memory.length,
  checkedTimedCalls: measured.rows.reduce((n, r) => n + r.reps, 0),
  timings, memory,
};
const output = path.join(evidence, 'analysis.json');
const encoded = JSON.stringify(result, null, 2) + '\n';
if (fs.existsSync(output)) assert.equal(fs.readFileSync(output, 'utf8'), encoded);
else fs.writeFileSync(output, encoded, { flag: 'wx' });
console.log(JSON.stringify({ timingObservations: result.timingObservations, memoryObservations: result.memoryObservations, checkedTimedCalls: result.checkedTimedCalls, verifiedHyperFiles: result.verifiedHyperFiles }));
for (const c of measured.cases.filter(c => c.sign === 'positive')) {
  const group = timings.filter(t => t.bits === c.bits && t.gap === c.gap && t.sign === c.sign);
  console.log(JSON.stringify({ bits: c.bits, gap: c.gap, timings: group.map(({ variant, ratio, interval95 }) => ({ variant, ratio, interval95 })), memory: memory.filter(m => m.bits === c.bits && m.gap === c.gap && m.sign === c.sign) }));
}
