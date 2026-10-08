import fs from 'node:fs';
import path from 'node:path';
import crypto from 'node:crypto';
import assert from 'node:assert/strict';
import { fileURLToPath } from 'node:url';
const dir = path.dirname(fileURLToPath(import.meta.url));
const root = path.resolve(dir, '../../..');
const evidence = path.join(dir, 'evidence-v1');
const prepared = JSON.parse(fs.readFileSync(path.join(evidence, 'prepared.json')));
const sha = p => crypto.createHash('sha256').update(fs.readFileSync(p)).digest('hex');
for (const [p, hash] of prepared.snapshot.files) {
  const source = path.join(root, p);
  assert.equal(sha(source), hash, p);
  if (path.dirname(source) === dir) fs.copyFileSync(source, path.join(evidence, path.basename(p)), fs.constants.COPYFILE_EXCL);
}
fs.copyFileSync(path.join(dir, 'analyze.mjs'), path.join(evidence, 'analyze.mjs'), fs.constants.COPYFILE_EXCL);
console.log('Preserved exact v1 pilot sources and analyzer; no numerical result reclassified.');
