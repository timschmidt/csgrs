// Freeze one consumer experiment without modifying prior retained snapshots.
import { readFileSync, writeFileSync, mkdirSync, copyFileSync, symlinkSync, constants } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here = dirname(fileURLToPath(import.meta.url));
const workspace = resolve(here, '../../../..');
const prior = JSON.parse(readFileSync(resolve(here, 'reuse-consumers.json')));
const sha = p => createHash('sha256').update(readFileSync(p)).digest('hex');
const selected = prior.crates.filter(c => ['hypersolve', 'hyperlimit', 'hyperlattice'].includes(c.crate));
for (const { crate, files } of selected) for (const file of files) {
  assert.equal(sha(resolve(here, 'reuse-consumers', crate, file.path)), file.sha256);
  assert.equal(sha(resolve(workspace, crate, file.path)), file.sha256);
}
const root = resolve(here, 'polynomial-decision-trial');
mkdirSync(root);
// Copy all selected crates to preserve one unified Cargo dependency path.
symlinkSync(resolve(here, 'root-exp-reuse-trial-hyperreal'), resolve(root, 'hyperreal'), 'dir');
for (const { crate, files } of selected) for (const file of files) {
  const target = resolve(root, crate, file.path);
  mkdirSync(dirname(target), { recursive: true });
  copyFileSync(resolve(here, 'reuse-consumers', crate, file.path), target,
    constants.COPYFILE_EXCL | constants.COPYFILE_FICLONE);
  assert.equal(sha(target), file.sha256);
}
writeFileSync(resolve(here, 'polynomial-decision-baseline-sources.json'),
  JSON.stringify({ schema: 1, crates: selected,
    scalar: 'root-exp-reuse-trial-hyperreal',
    scalarManifest: 'retained-exp-proof.json',
    note: 'These existing files match live bytes at preparation; no new live-file inventory claim.' }, null, 2) + '\n', { flag: 'wx' });
console.log(JSON.stringify({ copiedFiles: selected.flatMap(c => c.files).length }));
