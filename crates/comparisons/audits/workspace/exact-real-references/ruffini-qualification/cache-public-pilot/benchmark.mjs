import fs from 'node:fs';
import path from 'node:path';
import crypto from 'node:crypto';
import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
const dir = path.dirname(fileURLToPath(import.meta.url));
const root = path.resolve(dir, '../../..');
const evidence = path.join(dir, 'paired-v1');
const sha = p => crypto.createHash('sha256').update(fs.readFileSync(p)).digest('hex');
const read = p => JSON.parse(fs.readFileSync(p));
const write = (p, v) => fs.writeFileSync(p, JSON.stringify(v, null, 2) + '\n', { flag: 'wx' });
const stages = Object.fromEntries(['before', 'after'].map(s => [s, read(path.join(dir, s + '.json'))]));
const variants = ['before', 'before-control', 'after'];
const cases = ['root', 'rational'].flatMap(family => [false, true].flatMap(negative => [
  ...[[64, 0, 100000], [64, 1, 100000], [64, 32, 100000], [64, 72, 100000],
    [4096, 1, 30000], [4096, 2048, 30000], [4096, 4104, 30000],
    [65536, 1, 5000], [65536, 32768, 5000], [65536, 65408, 5000], [65536, 65544, 5000]]
    .map(([bits, gap, reps]) => ({ family, negative, mode: 'warm', bits, gap, reps })),
  ...[64, 128].map(bits => ({ family, negative, mode: 'cold', bits, gap: 0, reps: 1000 })),
]));
function verifySources() {
  for (const s of Object.values(stages)) {
    for (const [p, hash] of s.sources) assert.equal(sha(path.join(dir, p)), hash, p);
    for (const [p, hash] of s.binaries) assert.equal(sha(path.join(dir, s.evidence, p)), hash, p);
  }
  const snapshot = read(path.join(dir, 'snapshot.json'));
  for (const [p, hash] of snapshot.files) assert.equal(sha(path.join(root, 'hyperreal', p)), hash, p);
  for (const [p, hash] of stages.after.changed) assert.equal(sha(path.join(dir, 'source/hyperreal', p)), hash, p);
}
verifySources();
let serial = 0;
function run(variant, c, memory) {
  const stage = variant === 'before-control' ? 'before' : variant;
  const binary = path.join(dir, stages[stage].evidence, memory ? 'memory' : 'release');
  const args = ['-c', '6', binary, 'bench', c.family, c.negative ? 'negative' : 'positive', c.mode, String(c.bits), String(c.gap), String(memory ? 100 : c.reps)];
  const begin = new Date().toISOString();
  const r = spawnSync('taskset', args, { cwd: dir, encoding: 'utf8', timeout: 60000, maxBuffer: 1024 * 1024 });
  const raw = { args, begin, end: new Date().toISOString(), status: r.status, signal: r.signal, error: r.error?.message ?? null, stdout: r.stdout, stderr: r.stderr };
  const file = String(serial++).padStart(4, '0') + '.json';
  write(path.join(evidence, file), raw);
  assert.equal(r.error, undefined, JSON.stringify(raw));
  assert.equal(r.status, 0, JSON.stringify(raw));
  assert.equal(r.stderr, '');
  const result = JSON.parse(r.stdout);
  if (memory) assert.equal(result.memory.retained_delta, 0);
  else assert.equal(result.memory, null);
  return { variant, file, ...result };
}
if (process.argv[2] === 'run') {
  fs.mkdirSync(evidence);
  write(path.join(evidence, 'inputs.json'), { cases, variants, stages, scriptHash: sha(fileURLToPath(import.meta.url)) });
  const rows = [];
  for (let round = 0; round < 13; round++) {
    const order = Array.from({ length: 3 }, (_, i) => variants[(i + round) % 3]);
    if (round % 2) order.reverse();
    for (let i = 0; i < cases.length; i++) {
      const c = cases[(i + round) % cases.length];
      for (const variant of order) rows.push({ round, ...run(variant, c, false) });
    }
    console.log(JSON.stringify({ round, observations: rows.length }));
  }
  const memory = [];
  for (let round = 0; round < 3; round++) for (const c of cases) for (const variant of ['before', 'after']) memory.push({ round, ...run(variant, c, true) });
  verifySources();
  write(path.join(evidence, 'measurements.json'), { rows, memory });
  console.log(JSON.stringify({ timing: rows.length, memory: memory.length }));
} else if (process.argv[2] === 'analyze') {
  const inputs = read(path.join(evidence, 'inputs.json'));
  assert.deepEqual(inputs, { cases, variants, stages, scriptHash: sha(fileURLToPath(import.meta.url)) });
  const data = read(path.join(evidence, 'measurements.json'));
  assert.equal(data.rows.length, 2028);
  assert.equal(data.memory.length, 312);
  const rawFiles = fs.readdirSync(evidence).filter(p => /^\d{4}\.json$/.test(p)).sort();
  assert.equal(rawFiles.length, 2340);
  for (const row of [...data.rows, ...data.memory]) {
    const raw = read(path.join(evidence, row.file));
    assert.equal(raw.status, 0); assert.equal(raw.signal, null); assert.equal(raw.error, null); assert.equal(raw.stderr, '');
    const { round, variant, file, ...result } = row;
    assert.deepEqual(result, JSON.parse(raw.stdout));
    const stage = variant === 'before-control' ? 'before' : variant;
    assert.deepEqual(raw.args, ['-c', '6', path.join(dir, stages[stage].evidence, row.memory ? 'memory' : 'release'), 'bench', row.family, row.negative ? 'negative' : 'positive', row.mode, String(row.bits), String(row.gap), String(row.reps)]);
    assert(Number.isInteger(round) && Number.isSafeInteger(row.cpu_ns) && row.cpu_ns >= 0 && row.wall_ns > 0);
    if (!row.memory) assert(row.cpu_ns > 0);
    // Independent exact public enclosure check in another integer implementation.
    let m = BigInt(row.expected);
    const magnitude = m < 0n ? -m : m;
    assert.equal(row.checksum, (magnitude === 0n ? 0 : magnitude.toString(2).length + Number(m < 0n)) * row.reps);
    const p = -row.bits + row.gap;
    if (p >= 8) assert.equal(m, 0n);
    else {
      let lo = m - 1n, hi = m + 1n;
      const scale = p < 0 ? 1n << BigInt(-p) : 1n;
      if (p >= 0) { lo <<= BigInt(p); hi <<= BigInt(p); }
      if (row.negative) [lo, hi] = [-hi, -lo];
      if (row.family === 'rational') assert(lo * 3n <= scale && hi * 3n >= scale);
      else { const target = 17n * scale * scale; assert(hi >= 0n && hi * hi >= target && (lo <= 0n || lo * lo <= target)); }
    }
  }
  const median = values => { const v = [...values].sort((a, b) => a - b); return v.length % 2 ? v[(v.length - 1) / 2] : (v[v.length / 2 - 1] + v[v.length / 2]) / 2; };
  let state = 0xa7846132;
  const random = n => { state ^= state << 13; state ^= state >>> 17; state ^= state << 5; return (state >>> 0) % n; };
  const interval = values => { const b = Array.from({ length: 20000 }, () => median(values.map(() => values[random(values.length)]))).sort((a, b) => a - b); return [b[500], b[19499]]; };
  const results = [];
  for (const c of cases) {
    const matches = r => ['family', 'negative', 'mode', 'bits', 'gap'].every(k => r[k] === c[k]);
    const rows = data.rows.filter(matches);
    assert.equal(rows.length, 39);
    for (const row of rows) { assert.equal(row.expected, rows[0].expected); assert.equal(row.reps, c.reps); }
    const timing = {};
    for (const variant of ['before-control', 'after']) {
      const selected = rows.filter(r => r.variant === variant).sort((a, b) => a.round - b.round);
      assert.deepEqual(selected.map(r => r.round), Array.from({ length: 13 }, (_, i) => i));
      const ratios = selected.slice(1).map(r => r.cpu_ns / rows.find(b => b.variant === 'before' && b.round === r.round).cpu_ns);
      timing[variant] = { ratio: median(ratios), interval95: interval(ratios), samples: ratios.length, min: Math.min(...ratios), max: Math.max(...ratios) };
    }
    const memory = {};
    for (const variant of ['before', 'after']) {
      const selected = data.memory.filter(r => matches(r) && r.variant === variant).sort((a, b) => a.round - b.round);
      assert.deepEqual(selected.map(r => r.round), [0, 1, 2]);
      for (const r of selected) { assert.equal(r.reps, 100); assert.equal(r.expected, rows[0].expected); assert.deepEqual(r.memory, selected[0].memory); }
      memory[variant] = selected[0].memory;
    }
    results.push({ ...c, timing, memory });
  }
  const analysis = { scope: 'Private current-source public cache candidate, not production retention or full-suite qualification.', timingObservations: data.rows.length, afterWarmup: data.rows.filter(r => r.round > 0).length, timedCalls: data.rows.reduce((s, r) => s + r.reps, 0), memoryObservations: data.memory.length, inputsHash: sha(path.join(evidence, 'inputs.json')), measurementsHash: sha(path.join(evidence, 'measurements.json')), rawHashes: rawFiles.map(p => [p, sha(path.join(evidence, p))]), results };
  const encoded = JSON.stringify(analysis, null, 2) + '\n';
  const target = path.join(evidence, 'analysis.json');
  if (fs.existsSync(target)) assert.equal(fs.readFileSync(target, 'utf8'), encoded);
  else fs.writeFileSync(target, encoded, { flag: 'wx' });
  console.log(JSON.stringify({ timingObservations: analysis.timingObservations, afterWarmup: analysis.afterWarmup, timedCalls: analysis.timedCalls, memoryObservations: analysis.memoryObservations }));
  for (const r of results.filter(r => r.mode === 'cold' || r.gap === 1 || r.gap === 65408)) console.log(JSON.stringify(r));
} else throw new Error('run | analyze');
