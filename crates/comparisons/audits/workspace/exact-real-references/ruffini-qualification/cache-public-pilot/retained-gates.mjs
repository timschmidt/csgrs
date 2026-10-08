import fs from 'node:fs';
import path from 'node:path';
import crypto from 'node:crypto';
import assert from 'node:assert/strict';
import { spawn, execFileSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
const dir = path.dirname(fileURLToPath(import.meta.url));
const root = path.resolve(dir, '../../..');
const baseline = JSON.parse(fs.readFileSync(path.join(dir, 'retention-baseline.json')));
const after = JSON.parse(fs.readFileSync(path.join(dir, 'after.json')));
const evidence = fs.mkdtempSync(path.join(dir, 'retained-gates-'));
const sha = p => crypto.createHash('sha256').update(fs.readFileSync(p)).digest('hex');
const canonical = p => execFileSync('rustfmt', ['--edition', '2024', '--emit', 'stdout'], { input: fs.readFileSync(p), encoding: 'utf8' });
function verify() {
  for (const [name, repo] of Object.entries(baseline.repositories)) {
    for (const [p, hash] of repo.files) {
      assert.equal(sha(path.join(baseline.control, name, p)), hash, `frozen baseline: ${name}/${p}`);
      if (name === 'hyperreal' && p === 'src/computable/node.rs') {
        const original = fs.readFileSync(path.join(baseline.control, name, p), 'utf8');
        assert.equal(fs.readFileSync(path.join(root, name, p), 'utf8'), original + 'include!("node/cache_rescale_tests.rs");\n');
      } else if (name === 'hyperreal' && p === 'src/computable/node/representation.rs') {
        assert.equal(sha(path.join(root, name, p)), after.changed.find(r => r[0] === p)[1]);
      } else assert.equal(sha(path.join(root, name, p)), hash, `live source: ${name}/${p}`);
    }
    const current = execFileSync('git', ['ls-files', '-z', '--cached', '--others', '--exclude-standard'], { cwd: path.join(root, name), encoding: 'utf8' }).split('\0').filter(Boolean);
    const expected = repo.files.map(r => r[0]);
    if (name === 'hyperreal') expected.push('src/computable/node/cache_rescale_tests.rs');
    assert.deepEqual([...new Set(current)].sort(), expected.sort(), `${name} file inventory drift`);
  }
  assert.equal(canonical(path.join(root, 'hyperreal/src/computable/node/cache_rescale_tests.rs')), canonical(path.join(dir, 'cache_rescale_tests.rs')));
}
verify();
const sourceHashes = ['src/computable/node.rs', 'src/computable/node/representation.rs', 'src/computable/node/cache_rescale_tests.rs'].map(p => [p, sha(path.join(root, 'hyperreal', p))]);
const env = { ...process.env, TMPDIR: path.join(root, '.audit-coefficient-tmp.XBlc5e'), CARGO_TARGET_DIR: path.join(dir, 'target'), CARGO_INCREMENTAL: '0', CARGO_PROFILE_DEV_DEBUG: '0', CARGO_PROFILE_TEST_DEBUG: '0' };
const m = name => ['--offline', '--locked', '--manifest-path', path.join(root, name, 'Cargo.toml')];
const commands = [
  ['hyperreal-debug', 'cargo', ['test', ...m('hyperreal')]],
  ['hyperreal-release-all', 'cargo', ['test', ...m('hyperreal'), '--release', '--all-features']],
  ['hyperreal-clippy', 'cargo', ['clippy', ...m('hyperreal'), '--all-features', '--all-targets', '--', '-D', 'warnings']],
  ['hyperreal-wasm', 'cargo', ['check', ...m('hyperreal'), '--lib', '--target', 'wasm32-unknown-unknown']],
  ['hyperreal-fmt', 'cargo', ['fmt', '--manifest-path', path.join(root, 'hyperreal/Cargo.toml'), '--all', '--', '--check']],
  ['cache-tests-fmt', 'rustfmt', ['--edition', '2024', '--check', path.join(root, 'hyperreal/src/computable/node/cache_rescale_tests.rs')]],
  ...['hyperlattice', 'hyperlimit', 'hypertri', 'hypersolve'].map(name => [name, 'cargo', ['test', ...m(name)]]),
  ['hypercurve-lib', 'cargo', ['test', ...m('hypercurve'), '--lib']],
];
const records = [];
for (const [i, [name, command, args]] of commands.entries()) {
  verify();
  console.log(JSON.stringify({ started: i, name, command, args, evidence }));
  const begin = new Date().toISOString();
  const result = await new Promise(resolve => {
    const child = spawn(command, args, { cwd: root, env, stdio: ['ignore', 'pipe', 'pipe'] });
    let stdout = '', stderr = '', error = null;
    child.stdout.setEncoding('utf8').on('data', s => { stdout += s; });
    child.stderr.setEncoding('utf8').on('data', s => { stderr += s; });
    child.on('error', e => { error = e.message; });
    child.on('close', (status, signal) => resolve({ status, signal, error, stdout, stderr }));
  });
  const record = { name, command, args, begin, end: new Date().toISOString(), ...result };
  const file = String(i).padStart(2, '0') + '-' + name + '.json';
  fs.writeFileSync(path.join(evidence, file), JSON.stringify(record, null, 2) + '\n', { flag: 'wx' });
  const counts = [...result.stdout.matchAll(/test result: ok\. (\d+) passed;/g)].map(m => Number(m[1]));
  console.log(JSON.stringify({ finished: i, name, status: result.status, passed: counts.reduce((a, b) => a + b, 0), summaries: result.stdout.split('\n').filter(s => s.startsWith('test result:')) }));
  assert.equal(result.status, 0, JSON.stringify({ ...record, stdout: result.stdout.slice(-24000), stderr: result.stderr.slice(-24000) }));
  assert.equal(result.error, null);
  assert.equal(result.signal, null);
  verify();
  records.push([file, sha(path.join(evidence, file))]);
}
fs.writeFileSync(path.join(evidence, 'complete.json'), JSON.stringify({ records, sourceHashes, baselineHash: sha(path.join(dir, 'retention-baseline.json')), scriptHash: sha(fileURLToPath(import.meta.url)), verifiedBaselineFiles: Object.values(baseline.repositories).reduce((n, r) => n + r.files.length, 0) }, null, 2) + '\n', { flag: 'wx' });
console.log(JSON.stringify({ complete: true, evidence, commands: records.length, sourceHashes }));
