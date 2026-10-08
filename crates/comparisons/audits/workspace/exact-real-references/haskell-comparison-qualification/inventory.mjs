import fs from 'node:fs';
import path from 'node:path';
import crypto from 'node:crypto';
import assert from 'node:assert/strict';
import { execFileSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
const dir = path.dirname(fileURLToPath(import.meta.url));
const references = path.resolve(dir, '..');
const repo = path.join(references, 'haskell-reals-comparison');
const pin = '7c53d4e4259f633606b18d655cdbbf813866ab81';
const git = args => execFileSync('git', args, { cwd: repo, encoding: 'utf8' });
assert.equal(git(['rev-parse', 'HEAD']).trim(), pin);
assert.equal(git(['status', '--porcelain']), '');
// Explicit credit only after an untruncated complete file read and comparison.
const READ = new Set([
  '.gitignore', 'LICENSE', 'README.md', 'Setup.hs',
  'benchmarks/README-charts.txt', 'benchmarks/runBench.sh', 'benchmarks/all.js', 'results.html',
  'haskell-reals-comparison.cabal', 'package.yaml', 'old/IRealOps.hs',
  'src/Main.hs', 'src/Tasks/PreludeOps.hs', 'src/Tasks/MixedTypesNumOps.hs',
]);
const tree = git(['ls-tree', '-rz', '--full-tree', 'HEAD']).split('\0').filter(Boolean);
const records = [];
for (const entry of tree) {
  const m = entry.match(/^(\d+) (\S+) ([0-9a-f]+)\t(.+)$/);
  assert(m);
  const [, mode, type, object, file] = m;
  assert.equal(type, 'blob');
  assert(['100644', '100755'].includes(mode));
  const bytes = fs.readFileSync(path.join(repo, file));
  const hash = crypto.createHash('sha256').update(bytes).digest('hex');
  let text = null;
  try { text = new TextDecoder('utf-8', { fatal: true }).decode(bytes); } catch {}
  const lines = text === null ? null : (text.length === 0 ? 0 : text.split('\n').length - Number(text.endsWith('\n')));
  const kind = text === null ? 'BINARY_ASSET' : file.endsWith('.svg') ? 'SVG_ASSET' : file.endsWith('.log') ? 'STORED_LOG' : 'TEXT';
  const status = READ.has(file) ? 'READ' : kind.endsWith('ASSET') ? 'UNREVIEWED_ASSET' : 'UNREAD';
  if (READ.has(file)) assert(kind === 'TEXT' || kind === 'STORED_LOG');
  records.push({ file, mode, object, bytes: bytes.length, physicalLines: lines, sha256: hash, kind, status, range: status === 'READ' ? `1-${lines}` : '-' });
}
for (const file of READ) assert(records.some(r => r.file === file));
const columns = ['file', 'mode', 'object', 'bytes', 'physicalLines', 'sha256', 'kind', 'status', 'range'];
const table = columns.join('\t') + '\n' + records.map(r => columns.map(k => r[k] ?? '-').join('\t')).join('\n') + '\n';
fs.writeFileSync(path.join(references, 'HASKELL_COMPARISON_FILE_INVENTORY.tsv'), table);
fs.writeFileSync(path.join(references, 'HASKELL_COMPARISON_READ_COVERAGE.tsv'), ['file\tstatus\trange\tsha256', ...records.map(r => [r.file, r.status, r.range, r.sha256].join('\t'))].join('\n') + '\n');
const summary = { pin, files: records.length, bytes: records.reduce((s, r) => s + r.bytes, 0), kinds: {}, readFiles: READ.size, readLines: records.filter(r => r.status === 'READ').reduce((s, r) => s + r.physicalLines, 0) };
for (const r of records) { const group = summary.kinds[r.kind] ??= { files: 0, physicalLines: 0 }; group.files++; group.physicalLines += r.physicalLines ?? 0; }
fs.writeFileSync(path.join(dir, 'inventory.json'), JSON.stringify({ summary, records }, null, 2) + '\n');
console.log(JSON.stringify(summary));
for (const r of records.filter(r => r.kind === 'TEXT')) console.log(JSON.stringify({ file: r.file, bytes: r.bytes, physicalLines: r.physicalLines, status: r.status }));
