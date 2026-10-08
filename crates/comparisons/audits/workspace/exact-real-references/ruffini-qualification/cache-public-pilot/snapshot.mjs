import fs from 'node:fs';
import path from 'node:path';
import crypto from 'node:crypto';
import assert from 'node:assert/strict';
import { execFileSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
const dir = path.dirname(fileURLToPath(import.meta.url));
const root = path.resolve(dir, '../../..');
const sha = p => crypto.createHash('sha256').update(fs.readFileSync(p)).digest('hex');
const previous = JSON.parse(fs.readFileSync(path.join(dir, '../square-root-pilot/snapshot.json')));
for (const [p, hash] of previous.files) assert.equal(sha(path.join(root, p)), hash, p);
const repo = path.join(root, 'hyperreal');
const paths = [...new Set(execFileSync('git', ['ls-files', '-z', '--cached', '--others', '--exclude-standard'], { cwd: repo, encoding: 'utf8' }).split('\0').filter(Boolean))].sort();
const records = [];
for (const p of paths) {
  assert(!path.isAbsolute(p) && !p.split('/').includes('..') && !p.startsWith('target/'));
  const from = path.join(repo, p);
  assert(fs.lstatSync(from).isFile(), p);
  const to = path.join(dir, 'source/hyperreal', p);
  fs.mkdirSync(path.dirname(to), { recursive: true });
  fs.copyFileSync(from, to, fs.constants.COPYFILE_EXCL);
  records.push([p, sha(from), fs.statSync(from).size]);
  assert.equal(sha(from), sha(to));
}
const result = {
  created: new Date().toISOString(),
  head: execFileSync('git', ['rev-parse', 'HEAD'], { cwd: repo, encoding: 'utf8' }).trim(),
  status: execFileSync('git', ['status', '--short'], { cwd: repo, encoding: 'utf8' }),
  previous397SnapshotHash: sha(path.join(dir, '../square-root-pilot/snapshot.json')),
  files: records,
};
fs.writeFileSync(path.join(dir, 'snapshot.json'), JSON.stringify(result, null, 2) + '\n', { flag: 'wx' });
console.log(JSON.stringify({ files: records.length, bytes: records.reduce((s, r) => s + r[2], 0), head: result.head, sourceDrift: [] }));
