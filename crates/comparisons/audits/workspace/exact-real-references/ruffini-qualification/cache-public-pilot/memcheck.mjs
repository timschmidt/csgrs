import fs from 'node:fs';
import path from 'node:path';
import crypto from 'node:crypto';
import assert from 'node:assert/strict';
import { spawnSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
const dir = path.dirname(fileURLToPath(import.meta.url));
const after = JSON.parse(fs.readFileSync(path.join(dir, 'after.json')));
const evidence = path.join(dir, 'memcheck-v2');
fs.mkdirSync(evidence);
const sha = p => crypto.createHash('sha256').update(fs.readFileSync(p)).digest('hex');
for (const [p, h] of after.sources) assert.equal(sha(path.join(dir, p)), h, p);
for (const [p, h] of after.binaries) assert.equal(sha(path.join(dir, after.evidence, p)), h, p);
const unit = path.join(dir, 'target/release/deps/hyperreal-c3155383c66788f0');
fs.copyFileSync(unit, path.join(evidence, 'cache-tests'), fs.constants.COPYFILE_EXCL);
const commands = [
  [path.join(evidence, 'cache-tests'), 'cache_rescale_', '--test-threads=1'],
  [path.join(dir, after.evidence, 'release'), 'verify-extreme'],
];
const records = [];
for (const [i, args] of commands.entries()) {
  const fullArgs = ['--tool=memcheck', '--vgdb=no', '--error-exitcode=97', '--leak-check=no', ...args];
  const begin = new Date().toISOString();
  const r = spawnSync('valgrind', fullArgs, { cwd: dir, encoding: 'utf8', timeout: 60000, maxBuffer: 8 * 1024 * 1024 });
  const result = { args: fullArgs, begin, end: new Date().toISOString(), status: r.status, signal: r.signal, error: r.error?.message ?? null, stdout: r.stdout, stderr: r.stderr };
  const file = String(i) + '.json';
  fs.writeFileSync(path.join(evidence, file), JSON.stringify(result, null, 2) + '\n', { flag: 'wx' });
  assert.equal(r.error, undefined, JSON.stringify(result));
  assert.equal(r.status, 0, JSON.stringify(result));
  assert.match(r.stderr, /ERROR SUMMARY: 0 errors/);
  if (i === 0) assert.match(r.stdout, /5 passed; 0 failed/);
  else assert.deepEqual(JSON.parse(r.stdout), { mode: 'verify', extreme: true, checked: 180 });
  records.push([file, sha(path.join(evidence, file))]);
  console.log(JSON.stringify({ command: i, status: r.status, zeroMemcheckErrors: true }));
}
const report = { records, testBinaryHash: sha(path.join(evidence, 'cache-tests')), publicBinaryHash: sha(path.join(dir, after.evidence, 'release')), scriptHash: sha(fileURLToPath(import.meta.url)), leakChecking: false };
fs.writeFileSync(path.join(evidence, 'complete.json'), JSON.stringify(report, null, 2) + '\n', { flag: 'wx' });
