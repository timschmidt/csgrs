import './verify-reuse-cost-checkpoint.mjs';
import { readFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here = dirname(fileURLToPath(import.meta.url)), workspace = resolve(here, '../../../..');
const read = p => readFileSync(resolve(here, p), 'utf8');
const json = p => JSON.parse(read(p));
const hash = p => createHash('sha256').update(readFileSync(resolve(here, p))).digest('hex');
const manifest = json('retained-exp-proof.json');
for (const [p, h] of Object.entries({ ...manifest.sourceHashes, ...manifest.evidenceHashes })) assert.equal(hash(p), h, p);
const baseline = json(manifest.baselineSnapshot);
assert.equal(Object.keys(manifest.liveSourceHashes).length, 180);
assert.equal(manifest.changed.length, 2); assert.equal(manifest.added.length, 4);
for (const [p, h] of Object.entries(manifest.liveSourceHashes)) {
  assert.equal(hash(`${manifest.frozenSourceSnapshot}/${p}`), h, p);
  const previous = baseline.files.find(f => f.path === p);
  if (previous && !manifest.changed.includes(p)) assert.equal(h, previous.sha256, p);
  if (process.argv.includes('--live')) assert.equal(hash(resolve(workspace, 'hyperreal', p)), h, `live ${p}`);
}
function capture(tag) {
  const r = json(`results/${tag}.json`); assert.equal(r.code, 0, tag); assert.equal(r.signal, null, tag);
  assert.equal(r.cwd, resolve(workspace, 'hyperreal')); return read(`results/${tag}.stdout`);
}
function suites(text) {
  const result = []; let current;
  for (const line of text.split('\n')) {
    const start = /^running (\d+) tests?$/.exec(line);
    if (start) { assert(!current); current = { expected: +start[1], names: [] }; }
    const test = /^test (.+) \.\.\. (ok|ignored)(?:, (.*))?$/.exec(line);
    if (test) { assert(current); current.names.push([test[1], test[2], test[3] || '']); }
    const end = /^test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;/.exec(line);
    if (end) {
      assert(current); const [passed, failed, ignored, measured, filtered] = end.slice(1).map(Number);
      assert.deepEqual([failed, ignored, measured, filtered], [0, 0, 0, 0]);
      assert.equal(passed, current.expected); assert.equal(current.names.length, passed);
      assert.equal(new Set(current.names.map(n => n[0])).size, passed);
      result.push({ passed, names: current.names.sort((a, b) => a[0].localeCompare(b[0])) }); current = undefined;
    }
  }
  assert(!current); return result;
}
const metadata = JSON.parse(capture('retained-exp-proof-metadata'));
const packageInfo = metadata.packages.find(p => p.name === 'hyperreal'); assert(packageInfo);
assert.equal(packageInfo.manifest_path, resolve(workspace, 'hyperreal/Cargo.toml'));
const expectedTargets = packageInfo.targets.filter(t => t.kind.includes('lib') || t.kind.includes('test')).map(t => t.name).sort();
const profiles = [];
for (const profile of ['debug', 'release']) {
  const tag = `retained-exp-proof-${profile}`, results = suites(capture(tag));
  assert.equal(results.length, 14); assert.equal(results[0].passed, 780);
  assert.equal(results.reduce((n, s) => n + s.passed, 0), 855);
  const frozen = suites(read(`results/root-exp-reuse-${profile}.stdout`));
  assert.deepEqual(results[0], frozen[0]);
  const ran = ['hyperreal', ...[...read(`results/${tag}.stderr`).matchAll(/^\s+Running tests\/([^ ]+)\.rs /gm)].map(m => m[1])].sort();
  assert.deepEqual(ran, expectedTargets); profiles.push(results);
}
assert.deepEqual(profiles[0], profiles[1], 'debug/release test membership');
for (const phase of ['clippy', 'fmt', 'wasm']) capture(`retained-exp-proof-${phase}`);
const clippy = json('results/retained-exp-proof-clippy.json').args;
for (const flag of ['--all-features', '--all-targets', '-D', 'warnings']) assert(clippy.includes(flag));
assert(json('results/retained-exp-proof-wasm.json').args.includes('wasm32-unknown-unknown'));
console.log(JSON.stringify({ checkpoint: 'retained cache-preserving exponential proofs',
  libraryTestsPerProfile: 780, integrationTestsPerProfile: 75, suitesPerProfile: 14,
  qualifiedSourceFiles: 180, preservedBaselineFiles: 174, liveBytesChecked: process.argv.includes('--live'),
  status: manifest.status, acceptedCosts: manifest.acceptedCosts, limits: manifest.limits }));
