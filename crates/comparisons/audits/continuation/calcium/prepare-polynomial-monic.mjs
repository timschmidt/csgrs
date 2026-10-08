import { readFileSync, mkdirSync, copyFileSync, symlinkSync, constants } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here = dirname(fileURLToPath(import.meta.url));
const manifest = JSON.parse(readFileSync(resolve(here, 'retained-polynomial-facts.json')));
const sha = p => createHash('sha256').update(readFileSync(p)).digest('hex');
const destination = resolve(here, 'polynomial-monic-trial');
mkdirSync(destination);
symlinkSync(resolve(here, 'root-exp-reuse-trial-hyperreal'), resolve(destination, 'hyperreal'), 'dir');
let copied = 0;
for (const [path, hash] of Object.entries(manifest.liveSources)) {
  if (!['hyperlattice', 'hyperlimit', 'hypersolve'].includes(path.split('/')[0])) continue;
  const source = resolve(here, manifest.frozenSnapshot, path), target = resolve(destination, path);
  assert.equal(sha(source), hash); mkdirSync(dirname(target), { recursive: true });
  copyFileSync(source, target, constants.COPYFILE_EXCL | constants.COPYFILE_FICLONE);
  assert.equal(sha(target), hash); copied++;
}
assert.equal(copied, 339);
console.log(JSON.stringify({ copied, baseline: 'retained fact-first polynomial source; isolated monic experiment' }));
