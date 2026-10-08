import { readFileSync, mkdirSync, copyFileSync, constants } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here = dirname(fileURLToPath(import.meta.url)), workspace = resolve(here, '../../../..');
const prior = JSON.parse(readFileSync(resolve(here, 'reuse-consumers.json')));
const sha = path => createHash('sha256').update(readFileSync(path)).digest('hex');
let copied = 0;
for (const { crate, files } of prior.crates.filter(c => ['hypertri', 'hypercurve'].includes(c.crate))) {
  for (const file of files) {
    const source = resolve(here, 'reuse-consumers', crate, file.path);
    assert.equal(sha(source), file.sha256);
    assert.equal(sha(resolve(workspace, crate, file.path)), file.sha256);
    const target = resolve(here, 'polynomial-facts-trial', crate, file.path);
    mkdirSync(dirname(target), { recursive: true });
    copyFileSync(source, target, constants.COPYFILE_EXCL | constants.COPYFILE_FICLONE);
    assert.equal(sha(target), file.sha256); copied++;
  }
}
console.log(JSON.stringify({ copied, note: 'Additional frozen consumer files match corresponding live bytes; new-file inventory is not established.' }));
