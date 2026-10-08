import fs from 'node:fs';
import path from 'node:path';
import crypto from 'node:crypto';
import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
const dir = path.dirname(fileURLToPath(import.meta.url));
const evidence = path.join(dir, 'focused-v1');
const sha = p => crypto.createHash('sha256').update(fs.readFileSync(p)).digest('hex');
const read = p => JSON.parse(fs.readFileSync(p));
const write = (p, v) => fs.writeFileSync(p, JSON.stringify(v, null, 2) + '\n', { flag: 'wx' });
const stages = Object.fromEntries(['before', 'after'].map(s => [s, read(path.join(dir, s + '.json'))]));
const original = read(path.join(dir, 'paired-v1/measurements.json'));
const cases = [
  ['rational', true, 'warm', 64, 0, 1000000],
  ['rational', false, 'warm', 65536, 1, 100000],
  ['rational', false, 'warm', 64, 0, 1000000],
  ['rational', true, 'warm', 65536, 1, 100000],
  ['root', true, 'warm', 64, 0, 1000000],
  ['root', false, 'warm', 64, 1, 1000000],
  ['root', false, 'cold', 64, 0, 100000],
  ['rational', false, 'cold', 64, 0, 100000],
].map(([family, negative, mode, bits, gap, reps]) => ({ family, negative, mode, bits, gap, reps }));
const matches = (a, b) => ['family', 'negative', 'mode', 'bits', 'gap'].every(k => a[k] === b[k]);
const variants = ['before', 'after', 'before-control', 'after-control'];
function verify() {
  for (const s of Object.values(stages)) {
    for (const [p, h] of s.sources) assert.equal(sha(path.join(dir, p)), h, p);
    for (const [p, h] of s.binaries) assert.equal(sha(path.join(dir, s.evidence, p)), h, p);
  }
}
verify();
function validate(row) {
  const reference = original.rows.find(r => matches(r, row));
  assert(reference);
  assert.equal(row.expected, reference.expected);
  const expected = BigInt(row.expected);
  const magnitude = expected < 0n ? -expected : expected;
  const signature = magnitude === 0n ? 0 : magnitude.toString(2).length + Number(expected < 0n);
  assert.equal(row.checksum, signature * row.reps);
  assert(row.cpu_ns > 0 && row.wall_ns > 0 && row.memory === null);
}
if (process.argv[2] === 'run') {
  fs.mkdirSync(evidence);
  write(path.join(evidence, 'inputs.json'), { cases, variants, stages, priorAnalysisHash: sha(path.join(dir, 'paired-v1/analysis.json')), priorMeasurementsHash: sha(path.join(dir, 'paired-v1/measurements.json')), scriptHash: sha(fileURLToPath(import.meta.url)) });
  const rows = [];
  for (let round = 0; round < 25; round++) {
    const order = Array.from({ length: 4 }, (_, i) => variants[(i + round) % 4]);
    if (round % 2) order.reverse();
    for (let i = 0; i < cases.length; i++) {
      const c = cases[(i + round) % cases.length];
      for (const variant of order) {
        const stage = variant.startsWith('before') ? 'before' : 'after';
        const args = ['-c', '6', path.join(dir, stages[stage].evidence, 'release'), 'bench', c.family, c.negative ? 'negative' : 'positive', c.mode, String(c.bits), String(c.gap), String(c.reps)];
        const start = new Date().toISOString();
        const r = spawnSync('taskset', args, { cwd: dir, encoding: 'utf8', timeout: 60000, maxBuffer: 1024 * 1024 });
        const file = String(rows.length).padStart(4, '0') + '.json';
        write(path.join(evidence, file), { args, start, end: new Date().toISOString(), status: r.status, signal: r.signal, error: r.error?.message ?? null, stdout: r.stdout, stderr: r.stderr });
        assert.equal(r.status, 0); assert.equal(r.error, undefined); assert.equal(r.stderr, '');
        const result = JSON.parse(r.stdout); validate(result);
        rows.push({ round, variant, file, ...result });
      }
    }
    console.log(JSON.stringify({ round, observations: rows.length }));
  }
  verify();
  write(path.join(evidence, 'measurements.json'), { rows });
} else if (process.argv[2] === 'analyze') {
  const inputs = read(path.join(evidence, 'inputs.json'));
  assert.deepEqual(inputs, { cases, variants, stages, priorAnalysisHash: sha(path.join(dir, 'paired-v1/analysis.json')), priorMeasurementsHash: sha(path.join(dir, 'paired-v1/measurements.json')), scriptHash: sha(fileURLToPath(import.meta.url)) });
  const { rows } = read(path.join(evidence, 'measurements.json'));
  assert.equal(rows.length, 800);
  for (const row of rows) {
    const raw = read(path.join(evidence, row.file));
    const { round, variant, file, ...result } = row;
    assert.deepEqual(JSON.parse(raw.stdout), result);
    assert.equal(raw.status, 0); assert.equal(raw.error, null); assert.equal(raw.signal, null); assert.equal(raw.stderr, '');
    validate(row);
  }
  const median = values => { const v = [...values].sort((a, b) => a - b); return v.length % 2 ? v[(v.length - 1) / 2] : (v[v.length / 2 - 1] + v[v.length / 2]) / 2; };
  let state = 0xc1543809;
  const random = n => { state ^= state << 13; state ^= state >>> 17; state ^= state << 5; return (state >>> 0) % n; };
  const interval = v => { const b = Array.from({ length: 20000 }, () => median(v.map(() => v[random(v.length)]))).sort((a, b) => a - b); return [b[500], b[19499]]; };
  const results = [];
  for (const c of cases) {
    const group = rows.filter(r => matches(r, c));
    const observations = {};
    for (const variant of variants) {
      const selected = group.filter(r => r.variant === variant).sort((a, b) => a.round - b.round);
      assert.deepEqual(selected.map(r => r.round), Array.from({ length: 25 }, (_, i) => i));
      for (const r of selected) assert.equal(r.reps, c.reps);
      observations[variant] = selected.slice(1).map(r => r.cpu_ns);
    }
    const ratios = {};
    for (const [label, numerator, denominator] of [['after/before', 'after', 'before'], ['before-control/before', 'before-control', 'before'], ['after-control/after', 'after-control', 'after']]) {
      const v = observations[numerator].map((n, i) => n / observations[denominator][i]);
      ratios[label] = { median: median(v), interval95: interval(v), min: Math.min(...v), max: Math.max(...v) };
    }
    const pooled = observations.after.map((n, i) => (n + observations['after-control'][i]) / (observations.before[i] + observations['before-control'][i]));
    ratios['paired-control-pooled'] = { median: median(pooled), interval95: interval(pooled), min: Math.min(...pooled), max: Math.max(...pooled) };
    results.push({ ...c, samples: 24, ratios });
  }
  const analysis = { observations: rows.length, afterWarmup: 768, timedCalls: rows.reduce((n, r) => n + r.reps, 0), inputsHash: sha(path.join(evidence, 'inputs.json')), measurementsHash: sha(path.join(evidence, 'measurements.json')), rawHashes: rows.map(r => [r.file, sha(path.join(evidence, r.file))]), results };
  const encoded = JSON.stringify(analysis, null, 2) + '\n';
  const output = path.join(evidence, 'analysis.json');
  if (fs.existsSync(output)) assert.equal(fs.readFileSync(output, 'utf8'), encoded); else fs.writeFileSync(output, encoded, { flag: 'wx' });
  console.log(JSON.stringify({ observations: analysis.observations, afterWarmup: analysis.afterWarmup, timedCalls: analysis.timedCalls }));
  for (const result of results) console.log(JSON.stringify(result));
} else throw new Error('run | analyze');
