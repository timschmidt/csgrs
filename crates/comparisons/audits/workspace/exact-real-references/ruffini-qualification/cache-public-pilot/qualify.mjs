import fs from 'node:fs';
import path from 'node:path';
import crypto from 'node:crypto';
import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
const dir = path.dirname(fileURLToPath(import.meta.url));
const root = path.resolve(dir, '../../..');
const stage = process.argv[2];
assert(['before', 'after'].includes(stage));
const evidence = fs.mkdtempSync(path.join(dir, stage + '-'));
const env = { ...process.env, TMPDIR: path.join(root, '.audit-coefficient-tmp.XBlc5e'), CARGO_TARGET_DIR: path.join(dir, 'target'), CARGO_INCREMENTAL: '0', CARGO_PROFILE_DEV_DEBUG: '0', CARGO_PROFILE_TEST_DEBUG: '0' };
const sha = p => crypto.createHash('sha256').update(fs.readFileSync(p)).digest('hex');
const write = (p, v) => fs.writeFileSync(p, JSON.stringify(v, null, 2) + '\n', { flag: 'wx' });
let serial = 0;
function run(command, args, expected = 0, timeout = 300000) {
  const begin = new Date().toISOString();
  const r = spawnSync(command, args, { cwd: dir, env, encoding: 'utf8', timeout, maxBuffer: 8 * 1024 * 1024 });
  const record = { command, args, begin, end: new Date().toISOString(), status: r.status, signal: r.signal, error: r.error?.message ?? null, stdout: r.stdout, stderr: r.stderr };
  write(path.join(evidence, String(serial++).padStart(3, '0') + '.json'), record);
  assert.equal(r.error, undefined, JSON.stringify(record));
  assert.equal(r.status, expected, JSON.stringify(record));
  console.log(JSON.stringify({ stage, step: serial, command, args, status: r.status }));
  return record;
}
const snapshot = JSON.parse(fs.readFileSync(path.join(dir, 'snapshot.json')));
function verifySources() {
  const changed = [];
  for (const [p, hash] of snapshot.files) {
    assert.equal(sha(path.join(root, 'hyperreal', p)), hash, `live source changed: ${p}`);
    const current = sha(path.join(dir, 'source/hyperreal', p));
    if (current !== hash) changed.push([p, current]);
  }
  assert.deepEqual(changed.map(r => r[0]).sort(), (stage === 'before' ? ['src/computable/node.rs'] : ['src/computable/node.rs', 'src/computable/node/representation.rs']).sort());
  const originalFacade = fs.readFileSync(path.join(root, 'hyperreal/src/computable/node.rs'), 'utf8');
  assert.equal(fs.readFileSync(path.join(dir, 'source/hyperreal/src/computable/node.rs'), 'utf8'), originalFacade + 'include!("../../../../cache_rescale_tests.rs");\n');
  return changed;
}
const changed = verifySources();
if (!fs.existsSync(path.join(dir, 'Cargo.lock'))) run('cargo', ['generate-lockfile', '--offline']);
const sources = ['Cargo.toml', 'Cargo.lock', 'main.rs', 'cache_rescale_tests.rs', 'qualify.mjs', '../cache-rescale-pilot/allocation.rs'].map(p => [p, sha(path.join(dir, p))]);
write(path.join(evidence, 'sources.json'), { changed, sources, env: Object.fromEntries(Object.entries(env).filter(([k]) => ['TMPDIR', 'CARGO_TARGET_DIR', 'CARGO_INCREMENTAL', 'CARGO_PROFILE_DEV_DEBUG', 'CARGO_PROFILE_TEST_DEBUG'].includes(k))) });
run('rustc', ['-Vv']);
const test = run('cargo', ['test', '--offline', '--locked', '--manifest-path', 'source/hyperreal/Cargo.toml', '--lib', 'cache_rescale_', '--', '--test-threads=1'], stage === 'before' ? 101 : 0);
if (stage === 'before') {
  assert.match(test.stdout, /2 passed; 3 failed/);
  assert.match(test.stdout, /attempt to subtract with overflow/);
} else assert.match(test.stdout, /5 passed; 0 failed/);
run('cargo', ['build', '--offline', '--release']);
fs.copyFileSync(path.join(dir, 'target/release/cache-public'), path.join(evidence, 'release'), fs.constants.COPYFILE_EXCL);
const release = JSON.parse(run(path.join(evidence, 'release'), ['verify']).stdout);
assert.deepEqual(release, { mode: 'verify', extreme: false, checked: 160 });
if (stage === 'after') assert.deepEqual(JSON.parse(run(path.join(evidence, 'release'), ['verify-extreme']).stdout), { mode: 'verify', extreme: true, checked: 180 });
run('cargo', ['build', '--offline', '--release', '--features', 'live-allocations']);
fs.copyFileSync(path.join(dir, 'target/release/cache-public'), path.join(evidence, 'memory'), fs.constants.COPYFILE_EXCL);
assert.deepEqual(JSON.parse(run(path.join(evidence, 'memory'), ['verify']).stdout), release);
if (stage === 'after') {
  run('cargo', ['test', '--offline', '--locked', '--manifest-path', 'source/hyperreal/Cargo.toml', '--release', '--lib', 'cache_rescale_', '--', '--test-threads=1']);
}
assert.deepEqual(verifySources(), changed);
for (const [p, hash] of sources) assert.equal(sha(path.join(dir, p)), hash);
const binaries = ['release', 'memory'].map(p => [p, sha(path.join(evidence, p)), fs.statSync(path.join(evidence, p)).size]);
run('size', [path.join(evidence, 'release')]);
write(path.join(dir, stage + '.json'), { evidence: path.basename(evidence), changed, sources, binaries, commands: serial, publicCases: release.checked });
console.log(JSON.stringify({ stage, evidence, binaries, commands: serial, publicCases: release.checked }));
