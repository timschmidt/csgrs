import fs from 'node:fs';
import path from 'node:path';
import crypto from 'node:crypto';
import assert from 'node:assert/strict';
import { execFileSync } from 'node:child_process';
import { fileURLToPath } from 'node:url';
const dir = path.dirname(fileURLToPath(import.meta.url));
const root = path.resolve(dir, '../../..');
const read = p => JSON.parse(fs.readFileSync(path.join(dir, p)));
const sha = p => crypto.createHash('sha256').update(fs.readFileSync(p)).digest('hex');
const baseline = read('retention-baseline.json');
const after = read('after.json');
const gatesPath = 'retained-gates-DKveZ9';
const gates = read(gatesPath + '/complete.json');
assert.equal(gates.baselineHash, sha(path.join(dir, 'retention-baseline.json')));
assert.equal(gates.scriptHash, sha(path.join(dir, 'retained-gates.mjs')));
for (const [p, h] of baseline.evidence) assert.equal(sha(path.join(dir, p)), h, p);
for (const [p, h] of gates.sourceHashes) assert.equal(sha(path.join(root, 'hyperreal', p)), h, p);
const expectedCounts = [756, 863, 0, 0, 0, 0, 202, 348, 7, 796, 910];
assert.equal(gates.records.length, expectedCounts.length);
const totals = [];
for (const [i, [file, h]] of gates.records.entries()) {
  const full = path.join(dir, gatesPath, file);
  assert.equal(sha(full), h, file);
  const r = JSON.parse(fs.readFileSync(full));
  assert.equal(r.status, 0); assert.equal(r.signal, null); assert.equal(r.error, null);
  const summaries = [...r.stdout.matchAll(/test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored;/g)];
  const passed = summaries.reduce((n, m) => n + Number(m[1]), 0);
  assert.equal(passed, expectedCounts[i]);
  assert.equal(summaries.reduce((n, m) => n + Number(m[2]), 0), 0);
  assert.equal(summaries.reduce((n, m) => n + Number(m[3]), 0), i === 10 ? 1 : 0);
  totals.push({ name: r.name, passed, status: r.status });
}
let sourceCount = 0;
for (const [name, repo] of Object.entries(baseline.repositories)) {
  for (const [p, hash] of repo.files) {
    assert.equal(sha(path.join(baseline.control, name, p)), hash, `baseline ${name}/${p}`);
    const expected = name === 'hyperreal' ? gates.sourceHashes.find(r => r[0] === p)?.[1] ?? hash : hash;
    assert.equal(sha(path.join(root, name, p)), expected, `retained ${name}/${p}`);
    sourceCount++;
  }
  const current = execFileSync('git', ['ls-files', '-z', '--cached', '--others', '--exclude-standard'], { cwd: path.join(root, name), encoding: 'utf8' }).split('\0').filter(Boolean);
  const expected = repo.files.map(r => r[0]);
  if (name === 'hyperreal') expected.push('src/computable/node/cache_rescale_tests.rs');
  assert.deepEqual([...new Set(current)].sort(), expected.sort());
  execFileSync('git', ['diff', '--check'], { cwd: path.join(root, name) });
}
assert.equal(sourceCount, 947);
const originalFacade = fs.readFileSync(path.join(baseline.control, 'hyperreal/src/computable/node.rs'), 'utf8');
assert.equal(fs.readFileSync(path.join(root, 'hyperreal/src/computable/node.rs'), 'utf8'), originalFacade + 'include!("node/cache_rescale_tests.rs");\n');
const canonical = p => execFileSync('rustfmt', ['--edition', '2024', '--emit', 'stdout'], { input: fs.readFileSync(p), encoding: 'utf8' });
assert.equal(canonical(path.join(root, 'hyperreal/src/computable/node/cache_rescale_tests.rs')), canonical(path.join(dir, 'cache_rescale_tests.rs')));
assert.equal(sha(path.join(root, 'hyperreal/src/computable/node/representation.rs')), after.changed.find(r => r[0].endsWith('/representation.rs'))[1]);
for (const stage of ['before', 'after']) {
  const info = read(stage + '.json');
  for (const [p, h] of info.sources) assert.equal(sha(path.join(dir, p)), h, p);
  for (const [p, h] of info.binaries) assert.equal(sha(path.join(dir, info.evidence, p)), h, p);
}
let rawCount = 0;
for (const folder of ['paired-v1', 'focused-v1']) {
  const analysis = read(folder + '/analysis.json');
  const inputs = read(folder + '/inputs.json');
  assert.equal(analysis.inputsHash, sha(path.join(dir, folder, 'inputs.json')));
  assert.equal(analysis.measurementsHash, sha(path.join(dir, folder, 'measurements.json')));
  assert.equal(inputs.scriptHash, sha(path.join(dir, folder === 'paired-v1' ? 'benchmark.mjs' : 'focused.mjs')));
  const data = read(folder + '/measurements.json');
  const rows = [...data.rows, ...(data.memory ?? [])];
  assert.equal(rows.length, analysis.rawHashes.length);
  const hashes = new Map(analysis.rawHashes);
  assert.equal(hashes.size, rows.length);
  for (const row of rows) {
    const file = path.join(dir, folder, row.file);
    assert.equal(sha(file), hashes.get(row.file));
    const raw = JSON.parse(fs.readFileSync(file));
    assert.equal(raw.status, 0); assert.equal(raw.error, null); assert.equal(raw.signal, null); assert.equal(raw.stderr, '');
    const { round, variant, file: name, ...result } = row;
    assert.deepEqual(result, JSON.parse(raw.stdout));
    const stage = read((variant.startsWith('before') ? 'before' : 'after') + '.json');
    assert.deepEqual(raw.args, ['-c', '6', path.join(dir, stage.evidence, row.memory ? 'memory' : 'release'), 'bench', row.family, row.negative ? 'negative' : 'positive', row.mode, String(row.bits), String(row.gap), String(row.reps)]);
    rawCount++;
  }
}
assert.equal(rawCount, 3140);
const mem = read('memcheck-v2/complete.json');
assert.equal(mem.scriptHash, sha(path.join(dir, 'memcheck.mjs')));
assert.equal(mem.testBinaryHash, sha(path.join(dir, 'memcheck-v2/cache-tests')));
for (const [p, h] of mem.records) {
  assert.equal(sha(path.join(dir, 'memcheck-v2', p)), h);
  const r = read('memcheck-v2/' + p);
  assert.equal(r.status, 0); assert.equal(r.error, null); assert.match(r.stderr, /ERROR SUMMARY: 0 errors/);
}
const result = { scope: 'Retained Ruffini cache transfer checkpoint, not completion of the full exact-real ecosystem goal.', totals, verifiedBaselineFiles: sourceCount, newTestFiles: 1, verifiedRawMeasurements: rawCount, retainedGateHash: sha(path.join(dir, gatesPath, 'complete.json')), sourceHashes: gates.sourceHashes, verifierHash: sha(fileURLToPath(import.meta.url)), evidence: ['retention-baseline.json', 'paired-v1/analysis.json', 'focused-v1/analysis.json', 'memcheck-v2/complete.json'].map(p => [p, sha(path.join(dir, p))]) };
const encoded = JSON.stringify(result, null, 2) + '\n';
const output = path.join(dir, 'retained-checkpoint.json');
if (fs.existsSync(output)) assert.equal(fs.readFileSync(output, 'utf8'), encoded); else fs.writeFileSync(output, encoded, { flag: 'wx' });
console.log(JSON.stringify({ totals, verifiedBaselineFiles: sourceCount, newTestFiles: 1, verifiedRawMeasurements: rawCount, retainedGateHash: result.retainedGateHash, checkpointHash: sha(output) }));
