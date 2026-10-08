import { createHash } from 'node:crypto';
import { readFileSync, writeFileSync, mkdirSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';

const here = dirname(fileURLToPath(import.meta.url));
const manifest = JSON.parse(readFileSync(resolve(here, 'baseline-hyperreal.json'), 'utf8'));
const name = process.argv[2] ?? 'trial-hyperreal';
assert(/^[a-z][a-z0-9-]*trial[a-z0-9-]*$/.test(name), 'Expected a distinct trial directory name');
const target = resolve(here, name);
mkdirSync(target); // Never overwrite a trial or the immutable baseline.
for (const file of manifest.files) {
  const bytes = readFileSync(resolve(here, 'baseline-hyperreal', file.path));
  assert.equal(createHash('sha256').update(bytes).digest('hex'), file.sha256, file.path);
  const out = resolve(target, file.path);
  mkdirSync(dirname(out), { recursive: true });
  writeFileSync(out, bytes, { flag: 'wx' });
}
console.log(`Prepared ${manifest.files.length} hash-verified trial files`);
