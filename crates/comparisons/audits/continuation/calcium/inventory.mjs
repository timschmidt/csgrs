// Inventory is not proof of reading. Human-reviewed ranges live in coverage.json.
import { createHash } from 'node:crypto';
import { execFileSync } from 'node:child_process';
import { readFileSync, writeFileSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';

const here = dirname(fileURLToPath(import.meta.url));
const workspace = resolve(here, '../../../..');
const modules = ['ca', 'ca_ext', 'ca_field', 'ca_mat', 'ca_poly', 'ca_vec',
  'calcium', 'fexpr', 'fexpr_builtin', 'fmpz_mpoly_q', 'qqbar', 'nf', 'nf_elem'];
const roots = [
  ['calcium', '8dbb16fc4fe92eaf3ebbc7478d629e994d39f944'],
  ['flint', 'e269d38061d7a42070ddcffe6eb114466ed4aa7e'],
];
const git = (root, args) => execFileSync('git', ['-C', root, ...args], {
  encoding: 'utf8', maxBuffer: 32 * 1024 * 1024,
});
function scope(repo, path) {
  if (repo === 'calcium') return 'target';
  if (modules.some(m => path.startsWith(`src/${m}/`) ||
      path === `src/${m}.h` || path === `src/${m}_types.h` ||
      path === `doc/source/${m}.rst`)) return 'target';
  if (/^src\/gr\/(?:test\/t-)?(?:ca|qqbar|fexpr|fmpz_mpoly_q)\.c$/.test(path))
    return 'target';
  if (/^doc\/source\/(?:introduction_calcium|examples_calcium)\.rst$/.test(path) ||
      /^examples\/.*(?:ca|qqbar|fexpr).*\.c$/.test(path)) return 'target';
  return 'support-not-yet-selected';
}
const inventory = { schema: 1, note: 'Inventory does not credit reading.', sources: [] };
for (const [repo, commit] of roots) {
  const root = resolve(workspace, 'exact-real-references', repo);
  assert.equal(git(root, ['rev-parse', 'HEAD']).trim(), commit, `${repo} pin changed`);
  const files = git(root, ['ls-files', '-z']).split('\0').filter(Boolean).map(path => {
    const bytes = readFileSync(resolve(root, path));
    const text = !bytes.includes(0);
    const lines = text && bytes.length ? bytes.toString('utf8').split('\n').length -
      (bytes.at(-1) === 10 ? 1 : 0) : 0;
    return { path, bytes: bytes.length, lines, text, scope: scope(repo, path),
      sha256: createHash('sha256').update(bytes).digest('hex') };
  });
  inventory.sources.push({ repo, commit, files });
}
const command = process.argv[2];
const inventoryPath = resolve(here, 'inventory.json');
if (command === 'scan') {
  // Mechanical generation only; this command never modifies read coverage.
  writeFileSync(inventoryPath, JSON.stringify(inventory, null, 2) + '\n', { flag: 'wx' });
} else if (command === 'verify') {
  assert.deepEqual(inventory, JSON.parse(readFileSync(inventoryPath, 'utf8')),
    'Snapshot contents changed; investigate before replacing inventory');
} else {
  throw Error('Usage: node inventory.mjs scan|verify');
}
const coverage = JSON.parse(readFileSync(resolve(here, 'coverage.json'), 'utf8'));
const seen = new Set();
for (const entry of coverage) {
  const key = `${entry.repo}:${entry.path}`;
  assert(!seen.has(key), `Duplicate coverage entry: ${key}`);
  seen.add(key);
  const file = inventory.sources.find(s => s.repo === entry.repo)?.files.find(f => f.path === entry.path);
  assert(file?.text, `Unknown or non-text coverage path: ${key}`);
  assert(entry.note?.length, `Missing review note: ${key}`);
  let previous = 0;
  for (const [first, last] of entry.ranges) {
    assert(Number.isSafeInteger(first) && Number.isSafeInteger(last) && first > previous &&
      last >= first && last <= file.lines, `Invalid/overlapping read range: ${key}`);
    previous = last;
  }
}
for (const source of inventory.sources) {
  const selected = source.files.filter(f => f.scope === 'target');
  const reviewed = coverage.filter(c => c.repo === source.repo);
  const readLines = reviewed.reduce((n, c) => n + c.ranges.reduce((s, [a, b]) => s + b - a + 1, 0), 0);
  console.log(JSON.stringify({ repo: source.repo, commit: source.commit,
    trackedFiles: source.files.length, selectedFiles: selected.length,
    selectedTextLines: selected.reduce((n, f) => n + f.lines, 0),
    reviewedFiles: reviewed.length, readLines,
    completeFiles: reviewed.filter(c => {
      const f = source.files.find(f => f.path === c.path);
      return c.ranges.reduce((n, [a, b]) => n + b - a + 1, 0) === f.lines;
    }).length }));
}
