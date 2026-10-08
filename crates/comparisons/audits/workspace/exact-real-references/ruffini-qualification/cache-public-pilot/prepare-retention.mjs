import fs from 'node:fs';
import path from 'node:path';
import crypto from 'node:crypto';
import assert from 'node:assert/strict';
import { execFileSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
const dir = path.dirname(fileURLToPath(import.meta.url));
const root = path.resolve(dir, '../../..');
const sha = p => crypto.createHash('sha256').update(fs.readFileSync(p)).digest('hex');
const original = JSON.parse(fs.readFileSync(path.join(dir, 'snapshot.json')));
const after = JSON.parse(fs.readFileSync(path.join(dir, 'after.json')));
for (const [p, h] of original.files) assert.equal(sha(path.join(root, 'hyperreal', p)), h, p);
for (const [p, h] of after.changed) assert.equal(sha(path.join(dir, 'source/hyperreal', p)), h, p);
const control = path.join(dir, 'before-source');
fs.mkdirSync(control);
const repositories = {};
for (const name of ['hyperreal', 'hyperlattice', 'hyperlimit', 'hypertri', 'hypersolve', 'hypercurve']) {
  const repo = path.join(root, name);
  const git = args => execFileSync('git', args, { cwd: repo, encoding: 'utf8' });
  const files = [...new Set(git(['ls-files', '-z', '--cached', '--others', '--exclude-standard']).split('\0').filter(Boolean))].sort();
  const records = [];
  for (const p of files) {
    assert(!path.isAbsolute(p) && !p.split('/').includes('..') && !p.startsWith('target/'));
    const from = path.join(repo, p), to = path.join(control, name, p);
    assert(fs.lstatSync(from).isFile(), from);
    fs.mkdirSync(path.dirname(to), { recursive: true });
    fs.copyFileSync(from, to, fs.constants.COPYFILE_EXCL);
    assert.equal(sha(from), sha(to));
    records.push([p, sha(from), fs.statSync(from).size]);
  }
  repositories[name] = { head: git(['rev-parse', 'HEAD']).trim(), status: git(['status', '--short']), files: records };
}
const evidence = ['before.json', 'after.json', 'paired-v1/analysis.json', 'focused-v1/analysis.json', 'gates-ddSPOg/complete.json', 'memcheck-v2/complete.json'].map(p => [p, sha(path.join(dir, p))]);
fs.writeFileSync(path.join(dir, 'retention-baseline.json'), JSON.stringify({ created: new Date().toISOString(), control, repositories, evidence }, null, 2) + '\n', { flag: 'wx' });
console.log(JSON.stringify({ control, files: Object.values(repositories).reduce((n, r) => n + r.files.length, 0), bytes: Object.values(repositories).reduce((n, r) => n + r.files.reduce((n, f) => n + f[2], 0), 0), heads: Object.fromEntries(Object.entries(repositories).map(([n, r]) => [n, r.head])) }));
