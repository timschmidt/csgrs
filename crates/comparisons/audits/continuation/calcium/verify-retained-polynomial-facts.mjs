import './verify-polynomial-facts.mjs';
import { readFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here = dirname(fileURLToPath(import.meta.url)), workspace = resolve(here, '../../../..');
const read = p => readFileSync(resolve(here, p), 'utf8');
const json = p => JSON.parse(read(p));
const sha = p => createHash('sha256').update(readFileSync(resolve(here, p))).digest('hex');
const manifest = json('retained-polynomial-facts.json');
for (const [path, hash] of Object.entries(manifest.files)) assert.equal(sha(path), hash, path);
assert.deepEqual(manifest.changed, ['hypersolve/src/resultant.rs']);
assert.equal(Object.keys(manifest.liveSources).length, 953);
for (const [path, hash] of Object.entries(manifest.liveSources)) {
  assert.equal(sha(`${manifest.frozenSnapshot}/${path}`), hash, path);
  if (process.argv.includes('--retained-live')) assert.equal(sha(resolve(workspace, path)), hash, `live ${path}`);
}
function capture(gate) {
  const tag = `retained-polynomial-facts-${gate}`, r = json(`results/${tag}.json`);
  assert.equal(r.code, 0); assert.equal(r.signal, null); assert.equal(r.cwd, resolve(workspace, 'hypersolve'));
  return read(`results/${tag}.stdout`);
}
assert.deepEqual(manifest.gates, ['debug', 'release', 'clippy', 'fmt', 'wasm', 'metadata']);
const metadata = JSON.parse(capture('metadata'));
const packageInfo = metadata.packages.find(p => p.name === 'hypersolve'); assert(packageInfo);
assert.equal(packageInfo.manifest_path, resolve(workspace, 'hypersolve/Cargo.toml'));
const targets = packageInfo.targets.filter(t => t.kind.includes('lib') || t.kind.includes('test')).map(t => t.name).sort();
const testNames = text => [...text.matchAll(/^test (.+) \.\.\. (ok|ignored|FAILED).*$/gm)].map(m => `${m[1]}:${m[2]}`).sort();
for (const profile of ['debug', 'release']) {
  const text = capture(profile), frozen = read(`results/polynomial-facts-tests-${profile}.stdout`);
  assert.deepEqual(testNames(text), testNames(frozen)); assert.equal(testNames(text).length, 800);
  const suites = [...text.matchAll(/test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;/g)];
  assert.equal(suites.length, 7); assert.equal(suites.reduce((n,m) => n + Number(m[1]), 0), 800);
  assert(suites.every(m => m.slice(2).every(v => v === '0')));
  const ran = ['hypersolve', ...[...read(`results/retained-polynomial-facts-${profile}.stderr`).matchAll(/^\s+Running tests\/([^ ]+)\.rs /gm)].map(m => m[1])].sort();
  assert.deepEqual(ran, targets);
  const args = json(`results/retained-polynomial-facts-${profile}.json`).args;
  for (const flag of ['--offline', '--locked', '--all-features', '--lib', '--tests']) assert(args.includes(flag));
  assert.equal(args.includes('--release'), profile === 'release');
}
for (const phase of ['clippy', 'fmt', 'wasm']) capture(phase);
const clippy = json('results/retained-polynomial-facts-clippy.json').args;
for (const flag of ['--all-features', '--all-targets', '-D', 'warnings']) assert(clippy.includes(flag));
assert(json('results/retained-polynomial-facts-wasm.json').args.includes('wasm32-unknown-unknown'));
assert.deepEqual(json('results/retained-polynomial-facts-fmt.json').args, ['fmt', '--all', '--', '--check']);
console.log(JSON.stringify({ checkpoint: 'retained fact-first polynomial decisions', sourceFiles: 953,
  changed: manifest.changed, liveBytesChecked: process.argv.includes('--retained-live'), testsPerProfile: 800,
  status: manifest.status, acceptedCosts: manifest.acceptedCosts, limits: manifest.limits }));
