import { readFileSync, mkdirSync, copyFileSync, symlinkSync, constants } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here = dirname(fileURLToPath(import.meta.url));
const baseline = JSON.parse(readFileSync(resolve(here, 'polynomial-decision-baseline-sources.json')));
const prior = JSON.parse(readFileSync(resolve(here, 'polynomial-decision-experiment.json')));
const sha = path => createHash('sha256').update(readFileSync(path)).digest('hex');
const destination = resolve(here, 'polynomial-facts-trial');
mkdirSync(destination);
symlinkSync(resolve(here, 'root-exp-reuse-trial-hyperreal'), resolve(destination, 'hyperreal'), 'dir');
let copied = 0;
for (const { crate, files } of baseline.crates) for (const file of files) {
  const relative = `polynomial-decision-trial/${crate}/${file.path}`;
  const source = resolve(here, relative), target = resolve(destination, crate, file.path);
  const expected = prior.files[relative] ?? file.sha256;
  assert.equal(sha(source), expected);
  mkdirSync(dirname(target), { recursive: true });
  copyFileSync(source, target, constants.COPYFILE_EXCL | constants.COPYFILE_FICLONE);
  assert.equal(sha(target), expected); copied++;
}
console.log(JSON.stringify({ copied, source: 'immutable polynomial v1; separate fact-scheduling experiment' }));
