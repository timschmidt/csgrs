// Mechanical snapshots for controlled consumer testing; no live-tree edits.
import { spawn } from 'node:child_process';
import { readFileSync, writeFileSync, mkdirSync, copyFileSync, symlinkSync, lstatSync, constants } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here = dirname(fileURLToPath(import.meta.url));
const workspace = resolve(here, '../../../..');
const destination = resolve(here, 'sign-consumers');
const crates = ['hyperlattice', 'hyperlimit', 'hypersolve', 'hypertri', 'hypercurve'];
const sha = p => createHash('sha256').update(readFileSync(p)).digest('hex');
async function git(crate, args) {
  return await new Promise((resolveOutput, reject) => {
    const child = spawn('git', ['-C', resolve(workspace, crate), ...args], { stdio: ['ignore', 'pipe', 'pipe'] });
    const out = [], err = [];
    child.stdout.on('data', b => out.push(b)); child.stderr.on('data', b => err.push(b));
    child.on('error', reject);
    child.on('close', code => code === 0 ? resolveOutput(Buffer.concat(out).toString()) :
      reject(Error(`git ${crate}: ${code}: ${Buffer.concat(err)}`)));
  });
}
const manifest = { schema: 1, recorded: new Date().toISOString(), crates: [],
  scalarSources: { baseline: resolve(here, 'baseline-hyperreal'), sign: resolve(here, 'root-exp-sign-trial-hyperreal') },
  note: 'Identical consumer source snapshots; only the hyperreal symlink changes. Use --locked for builds. Snapshots include pre-existing live modifications without attributing them to this audit. No benchmark or correctness claim follows from snapshot preparation.' };
for (const crate of crates) {
  const head = (await git(crate, ['rev-parse', 'HEAD'])).trim();
  assert(/^[0-9a-f]{40}$/.test(head));
  const paths = [...new Set((await git(crate, ['ls-files', '--cached', '--others', '--exclude-standard', '-z'])).split('\0').filter(Boolean))].sort();
  assert(paths.includes('Cargo.toml') && paths.some(p => p.startsWith('src/')));
  const files = paths.map(path => {
    assert(!path.startsWith('/') && !path.split('/').includes('..'));
    const input = resolve(workspace, crate, path), stat = lstatSync(input);
    assert(stat.isFile(), `Non-regular snapshot path: ${crate}/${path}`);
    return { path, bytes: stat.size, sha256: sha(input) };
  });
  manifest.crates.push({ crate, head, files });
}
const bytes = manifest.crates.flatMap(c => c.files).reduce((n, f) => n + f.bytes, 0);
assert(bytes < 256 * 1024 * 1024, 'Review unexpectedly large source snapshot before copying');
mkdirSync(destination); // Refuse to overwrite earlier evidence.
for (const variant of ['baseline', 'sign']) {
  const root = resolve(destination, variant);
  mkdirSync(root);
  symlinkSync(manifest.scalarSources[variant], resolve(root, 'hyperreal'), 'dir');
  for (const { crate, files } of manifest.crates) for (const file of files) {
    const original = resolve(workspace, crate, file.path), target = resolve(root, crate, file.path);
    assert.equal(sha(original), file.sha256, 'Live source changed during preparation');
    mkdirSync(dirname(target), { recursive: true });
    copyFileSync(original, target, constants.COPYFILE_EXCL | constants.COPYFILE_FICLONE);
    assert.equal(sha(target), file.sha256);
  }
}
writeFileSync(resolve(here, 'sign-consumers.json'), JSON.stringify(manifest, null, 2) + '\n', { flag: 'wx' });
console.log(JSON.stringify({ crates: crates.length, filesPerVariant: manifest.crates.reduce((n, c) => n + c.files.length, 0), bytesPerVariant: bytes }));
