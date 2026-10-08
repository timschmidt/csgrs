import { readFileSync, mkdirSync, copyFileSync, symlinkSync, constants } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here = dirname(fileURLToPath(import.meta.url));
const m = JSON.parse(readFileSync(resolve(here, 'retained-monic.json'), 'utf8'));
const sha = p => createHash('sha256').update(readFileSync(p)).digest('hex');
const destination = resolve(here, 'rank-dominance-trial'); mkdirSync(destination);
symlinkSync(resolve(here, 'root-exp-reuse-trial-hyperreal'), resolve(destination, 'hyperreal'), 'dir');
let copied = 0;
for (const [path, hash] of Object.entries(m.liveSources)) {
  const source = resolve(here, m.frozenSnapshot, path);
  assert.equal(sha(source), hash, path);
  if (path.startsWith('hyperreal/')) continue;
  const target = resolve(destination, path); mkdirSync(dirname(target), { recursive: true });
  copyFileSync(source, target, constants.COPYFILE_EXCL | constants.COPYFILE_FICLONE);
  assert.equal(sha(target), hash, path); copied++;
}
assert.equal(copied, 774);
console.log(JSON.stringify({ copied, source: m.frozenSnapshot, changed: 'none; isolated rank edit follows' }));
