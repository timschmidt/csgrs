import { readFileSync, mkdirSync, copyFileSync, symlinkSync, constants } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here = dirname(fileURLToPath(import.meta.url));
const manifest = JSON.parse(readFileSync(resolve(here, 'polynomial-closure-experiment.json')));
const sha = p => createHash('sha256').update(readFileSync(p)).digest('hex');
const destination = resolve(here, 'polynomial-monic-qualified-trial');
mkdirSync(destination);
symlinkSync(resolve(here, 'root-exp-reuse-trial-hyperreal'), resolve(destination, 'hyperreal'), 'dir');
let copied = 0;
for (const [path, hash] of Object.entries(manifest.candidateSources)) {
  const source = resolve(here, 'polynomial-monic-trial', path), target = resolve(destination, path);
  assert.equal(sha(source), hash); mkdirSync(dirname(target), { recursive: true });
  copyFileSync(source, target, constants.COPYFILE_EXCL | constants.COPYFILE_FICLONE);
  assert.equal(sha(target), hash); copied++;
}
assert.equal(copied, 339);
console.log(JSON.stringify({ copied, source: 'immutable checkpoint17 monic candidate; qualification tests to be added separately' }));
