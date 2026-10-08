import fs from 'node:fs';
import path from 'node:path';
import crypto from 'node:crypto';
import { spawnSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';

const dir = path.dirname(fileURLToPath(import.meta.url));
const root = path.resolve(dir, '../../..');
const phase = process.argv[2];
const evidence = path.join(dir, 'evidence-v2');
const sha = p => crypto.createHash('sha256').update(fs.readFileSync(p)).digest('hex');
const write = (p, value) => fs.writeFileSync(p, JSON.stringify(value, null, 2) + '\n', { flag: 'wx' });
const env = { ...process.env, TMPDIR: path.join(root, '.audit-coefficient-tmp.XBlc5e') };
let serial = 0;
function run(command, args, timeout = 60_000) {
  const begin = new Date().toISOString();
  const result = spawnSync(command, args, { cwd: dir, env, encoding: 'utf8', timeout, maxBuffer: 8 * 1024 * 1024 });
  const record = { command, args, begin, end: new Date().toISOString(), status: result.status, signal: result.signal, error: result.error?.message ?? null, stdout: result.stdout, stderr: result.stderr };
  write(path.join(evidence, `${phase}-${String(serial++).padStart(4, '0')}.json`), record);
  assert.equal(result.error, undefined, JSON.stringify(record));
  assert.equal(result.status, 0, JSON.stringify(record));
  return record;
}
const owned = ['main.rs', 'allocation.rs', 'Cargo.toml', 'Cargo.lock', 'qualify.mjs'];
function sources() {
  const old = JSON.parse(fs.readFileSync(path.join(dir, '../square-root-pilot/snapshot.json')));
  for (const [p, hash] of old.files) assert.equal(sha(path.join(root, p)), hash, `Hyper source drift: ${p}`);
  return {
    priorSnapshot: sha(path.join(dir, '../square-root-pilot/snapshot.json')),
    verifiedHyperFiles: old.files.length,
    files: [...owned.map(p => [path.relative(root, path.join(dir, p)), sha(path.join(dir, p))]),
      ['hyperreal/Cargo.lock', sha(path.join(root, 'hyperreal/Cargo.lock'))]],
  };
}
if (phase === 'prepare') {
  fs.mkdirSync(evidence);
  const snapshot = sources();
  write(path.join(evidence, 'snapshot.json'), snapshot);
  run('rustc', ['-Vv']);
  run('cargo', ['-V']);
  run('uname', ['-a']);
  run('taskset', ['-pc', String(process.pid)]);
  run('cargo', ['build', '--offline', '--locked']);
  fs.copyFileSync(path.join(dir, 'target/debug/cache-rescale'), path.join(evidence, 'debug'), fs.constants.COPYFILE_EXCL);
  const debug = JSON.parse(run(path.join(evidence, 'debug'), ['verify']).stdout);
  const boundary = run(path.join(evidence, 'debug'), ['public-boundary']);
  assert.deepEqual(JSON.parse(boundary.stdout), { mode: 'public-boundary', result: 'panic' });
  assert.match(boundary.stderr, /node\/representation.rs:197/);
  assert.match(boundary.stderr, /attempt to subtract with overflow/);
  run('cargo', ['build', '--offline', '--locked', '--release']);
  fs.copyFileSync(path.join(dir, 'target/release/cache-rescale'), path.join(evidence, 'release'), fs.constants.COPYFILE_EXCL);
  const release = JSON.parse(run(path.join(evidence, 'release'), ['verify']).stdout);
  assert.deepEqual(release, debug);
  run('cargo', ['build', '--offline', '--locked', '--release', '--features', 'live-allocations']);
  fs.copyFileSync(path.join(dir, 'target/release/cache-rescale'), path.join(evidence, 'memory'), fs.constants.COPYFILE_EXCL);
  const memory = JSON.parse(run(path.join(evidence, 'memory'), ['verify']).stdout);
  assert.deepEqual(memory, debug);
  assert.deepEqual(sources(), snapshot);
  const binaries = ['debug', 'release', 'memory'].map(p => [p, sha(path.join(evidence, p)), fs.statSync(path.join(evidence, p)).size]);
  write(path.join(evidence, 'prepared.json'), { snapshot, binaries, verification: debug, boundary: JSON.parse(boundary.stdout) });
  console.log(JSON.stringify({ phase, verification: debug, boundary: JSON.parse(boundary.stdout), binaries }));
} else if (phase === 'measure') {
  const prepared = JSON.parse(fs.readFileSync(path.join(evidence, 'prepared.json')));
  assert.deepEqual(sources(), prepared.snapshot);
  for (const [p, hash] of prepared.binaries) assert.equal(sha(path.join(evidence, p)), hash);
  const cases = [
    [64, 0, 300000], [64, 1, 300000], [64, 32, 300000], [64, 65, 300000],
    [4096, 1, 100000], [4096, 2048, 100000], [4096, 4100, 100000],
    [65536, 1, 15000], [65536, 32768, 15000], [65536, 65537, 15000],
    [1048576, 1048448, 2000],
  ].flatMap(([bits, gap, reps]) => ['positive', 'negative'].map(sign => ({ bits, gap, reps, sign })));
  const variants = ['original', 'wide', 'borrowed', 'guarded', 'original-control'];
  const rows = [];
  for (let round = 0; round < 11; round++) {
    for (let j = 0; j < cases.length; j++) {
      const c = cases[(j + round) % cases.length];
      const order = Array.from({ length: variants.length }, (_, i) => variants[(i + round) % variants.length]);
      if (round % 2) order.reverse();
      for (const variant of order) {
        const method = variant === 'original-control' ? 'original' : variant;
        const record = run('taskset', ['-c', '6', path.join(evidence, 'release'), 'bench', method, String(c.bits), String(c.gap), c.sign, String(c.reps)]);
        const result = JSON.parse(record.stdout);
        assert.equal(record.stderr, '');
        assert.equal(result.memory, null);
        assert.equal(result.reps, c.reps);
        rows.push({ round, variant, ...result });
      }
    }
    console.log(JSON.stringify({ phase, round, observations: rows.length }));
  }
  const memory = [];
  for (let round = 0; round < 3; round++) {
    for (const c of cases) for (const variant of variants.slice(0, 4)) {
      const record = run('taskset', ['-c', '6', path.join(evidence, 'memory'), 'bench', variant, String(c.bits), String(c.gap), c.sign, '100']);
      const result = JSON.parse(record.stdout);
      assert.equal(record.stderr, '');
      assert.equal(result.memory.retained_delta, 0);
      memory.push({ round, variant, ...result });
    }
  }
  assert.deepEqual(sources(), prepared.snapshot);
  for (const [p, hash] of prepared.binaries) assert.equal(sha(path.join(evidence, p)), hash);
  write(path.join(evidence, 'measurements.json'), { cases, variants, rows, memory });
  console.log(JSON.stringify({ phase, observations: rows.length, memoryObservations: memory.length }));
} else {
  throw new Error('prepare | measure');
}
