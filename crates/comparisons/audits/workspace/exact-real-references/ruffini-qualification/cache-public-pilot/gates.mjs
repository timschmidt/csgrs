import fs from 'node:fs';
import path from 'node:path';
import crypto from 'node:crypto';
import assert from 'node:assert/strict';
import { spawn } from 'node:child_process';
import { fileURLToPath } from 'node:url';
const dir = path.dirname(fileURLToPath(import.meta.url));
const root = path.resolve(dir, '../../..');
const after = JSON.parse(fs.readFileSync(path.join(dir, 'after.json')));
const snapshot = JSON.parse(fs.readFileSync(path.join(dir, 'snapshot.json')));
const evidence = fs.mkdtempSync(path.join(dir, 'gates-'));
const sha = p => crypto.createHash('sha256').update(fs.readFileSync(p)).digest('hex');
function verify() {
  for (const [p, hash] of after.sources) assert.equal(sha(path.join(dir, p)), hash, p);
  for (const [p, hash] of snapshot.files) {
    assert.equal(sha(path.join(root, 'hyperreal', p)), hash, p);
    const changed = after.changed.find(c => c[0] === p);
    assert.equal(sha(path.join(dir, 'source/hyperreal', p)), changed?.[1] ?? hash, p);
  }
}
verify();
const env = { ...process.env, TMPDIR: path.join(root, '.audit-coefficient-tmp.XBlc5e'), CARGO_TARGET_DIR: path.join(dir, 'target'), CARGO_INCREMENTAL: '0', CARGO_PROFILE_DEV_DEBUG: '0', CARGO_PROFILE_TEST_DEBUG: '0' };
const manifest = ['--offline', '--locked', '--manifest-path', 'source/hyperreal/Cargo.toml'];
const commands = [
  ['cargo', ['test', ...manifest]],
  ['cargo', ['test', ...manifest, '--release', '--all-features']],
  ['cargo', ['clippy', ...manifest, '--all-features', '--all-targets', '--', '-D', 'warnings']],
  ['cargo', ['check', ...manifest, '--lib', '--target', 'wasm32-unknown-unknown']],
  ['cargo', ['fmt', '--manifest-path', 'source/hyperreal/Cargo.toml', '-p', 'hyperreal', '--', '--check']],
];
const records = [];
for (const [i, [command, args]] of commands.entries()) {
  console.log(JSON.stringify({ started: i, command, args, evidence }));
  const begin = new Date().toISOString();
  const result = await new Promise(resolve => {
    const child = spawn(command, args, { cwd: dir, env, stdio: ['ignore', 'pipe', 'pipe'] });
    let stdout = '', stderr = '', error = null;
    child.stdout.setEncoding('utf8').on('data', s => { stdout += s; });
    child.stderr.setEncoding('utf8').on('data', s => { stderr += s; });
    child.on('error', e => { error = e.message; });
    child.on('close', (status, signal) => resolve({ status, signal, error, stdout, stderr }));
  });
  const record = { command, args, begin, end: new Date().toISOString(), ...result };
  const file = String(i).padStart(2, '0') + '.json';
  fs.writeFileSync(path.join(evidence, file), JSON.stringify(record, null, 2) + '\n', { flag: 'wx' });
  assert.equal(result.status, 0, JSON.stringify(record));
  assert.equal(result.error, null);
  assert.equal(result.signal, null);
  verify();
  records.push([file, sha(path.join(evidence, file))]);
  console.log(JSON.stringify({ finished: i, status: result.status, summaries: result.stdout.split('\n').filter(s => s.startsWith('test result:')) }));
}
fs.writeFileSync(path.join(evidence, 'complete.json'), JSON.stringify({ records, verifiedSourceFiles: snapshot.files.length, scriptHash: sha(fileURLToPath(import.meta.url)), afterHash: sha(path.join(dir, 'after.json')) }, null, 2) + '\n', { flag: 'wx' });
console.log(JSON.stringify({ complete: true, evidence, commands: records.length }));
