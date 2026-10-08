// Mechanically preserve a user-worktree baseline without editing that worktree.
import { execFileSync } from 'node:child_process';
import { createHash } from 'node:crypto';
import { readFileSync, writeFileSync, mkdirSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';

const here = dirname(fileURLToPath(import.meta.url));
const source = resolve(here, '../../../../hyperreal');
const target = resolve(here, 'baseline-hyperreal');
const git = (...args) => execFileSync('git', ['-C', source, ...args], {
  encoding: 'utf8', maxBuffer: 16 * 1024 * 1024,
});
const sha = bytes => createHash('sha256').update(bytes).digest('hex');
const commit = git('rev-parse', 'HEAD').trim();
const status = git('status', '--porcelain=v1');
const diff = git('diff', '--binary', 'HEAD');
const paths = [...new Set(git('ls-files', '-z', '--cached', '--others', '--exclude-standard')
  .split('\0').filter(Boolean))].sort();
const extending = process.argv[2] === 'extend';
let existing = [];
if (extending) {
  const previous = JSON.parse(readFileSync(resolve(here, 'baseline-hyperreal.json'), 'utf8'));
  assert.equal(previous.commit, commit, 'Cannot extend a different HEAD');
  assert.equal(previous.status, status, 'Cannot extend a different worktree');
  assert.equal(readFileSync(resolve(here, 'baseline-hyperreal.patch'), 'utf8'), diff);
  existing = previous.files;
  for (const file of existing) {
    assert.equal(sha(readFileSync(resolve(target, file.path))), file.sha256, 'Frozen file drift');
    assert.equal(sha(readFileSync(resolve(source, file.path))), file.sha256, 'Live file drift');
  }
} else {
  mkdirSync(target); // Refuse replacing any earlier snapshot.
}
const files = paths.map(path => {
  const bytes = readFileSync(resolve(source, path));
  const destination = resolve(target, path);
  mkdirSync(dirname(destination), { recursive: true });
  if (!existing.some(f => f.path === path)) writeFileSync(destination, bytes, { flag: 'wx' });
  return { path, bytes: bytes.length, sha256: sha(bytes) };
});
for (const file of files)
  assert.equal(sha(readFileSync(resolve(source, file.path))), file.sha256, `Source drift: ${file.path}`);
assert.equal(git('rev-parse', 'HEAD').trim(), commit, 'HEAD drift');
assert.equal(git('status', '--porcelain=v1'), status, 'Worktree status drift');
assert.equal(git('diff', '--binary', 'HEAD'), diff, 'Worktree diff drift');
writeFileSync(resolve(here, 'baseline-hyperreal.json'), JSON.stringify({
  note: 'Includes pre-existing uncommitted user work; not attributed to this audit.',
  commit, status, files,
}, null, 2) + '\n', { flag: extending ? 'w' : 'wx' });
if (!extending) writeFileSync(resolve(here, 'baseline-hyperreal.patch'), diff, { flag: 'wx' });
console.log(JSON.stringify({ commit, files: files.length, bytes: files.reduce((n, f) => n + f.bytes, 0) }));
