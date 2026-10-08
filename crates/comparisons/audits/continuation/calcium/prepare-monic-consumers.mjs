import { readFileSync, mkdirSync, copyFileSync, constants } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here = dirname(fileURLToPath(import.meta.url));
const manifest = JSON.parse(readFileSync(resolve(here, 'retained-polynomial-facts.json')));
const sha = p => createHash('sha256').update(readFileSync(p)).digest('hex');
let copied = 0;
for (const [path, hash] of Object.entries(manifest.liveSources)) {
  if (!['hypertri', 'hypercurve'].includes(path.split('/')[0])) continue;
  const source = resolve(here, manifest.frozenSnapshot, path);
  const target = resolve(here, 'polynomial-monic-qualified-trial', path);
  assert.equal(sha(source), hash); mkdirSync(dirname(target), { recursive: true });
  copyFileSync(source, target, constants.COPYFILE_EXCL | constants.COPYFILE_FICLONE);
  assert.equal(sha(target), hash); copied++;
}
assert.equal(copied, 434);
console.log(JSON.stringify({ copied, source: 'retained checkpoint16 consumers, for monic downstream qualification' }));
