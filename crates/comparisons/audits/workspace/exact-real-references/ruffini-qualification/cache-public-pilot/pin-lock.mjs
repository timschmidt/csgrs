import fs from 'node:fs';
import path from 'node:path';
import assert from 'node:assert/strict';
import { execFileSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
const dir = path.dirname(fileURLToPath(import.meta.url));
const root = path.resolve(dir, '../../..');
const lock = path.join(dir, 'Cargo.lock');
const original = fs.readFileSync(path.join(root, 'hyperreal/Cargo.lock'), 'utf8');
fs.copyFileSync(lock, path.join(dir, 'setup-generated-Cargo.lock'), fs.constants.COPYFILE_EXCL);
// This is an owned, unqualified private lockfile. Keep the original generated
// version above, then seed resolution from the production lock, not the index.
fs.copyFileSync(path.join(root, 'hyperreal/Cargo.lock'), lock);
const metadata = execFileSync('cargo', ['metadata', '--offline', '--format-version', '1'], { cwd: dir, encoding: 'utf8', maxBuffer: 8 * 1024 * 1024 });
fs.writeFileSync(path.join(dir, 'resolved-metadata.json'), metadata, { flag: 'wx' });
const entries = text => text.split('[[package]]').slice(1).map(s => {
  const get = k => s.match(new RegExp('^' + k + ' = "([^"]+)"', 'm'))?.[1];
  return { name: get('name'), version: get('version'), checksum: get('checksum') };
});
const before = entries(original);
const after = entries(fs.readFileSync(lock, 'utf8'));
for (const entry of after.filter(e => e.checksum)) assert(before.some(e => JSON.stringify(e) === JSON.stringify(entry)), `dependency drift: ${JSON.stringify(entry)}`);
assert(after.some(e => e.name === 'ruffini_cache_public_pilot'));
console.log(JSON.stringify({ registryPackages: after.filter(e => e.checksum).length, allMatchProduction: true }));
