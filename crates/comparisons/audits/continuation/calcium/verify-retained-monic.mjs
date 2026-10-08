import './verify-matrix-pivots.mjs';
import { readFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here = dirname(fileURLToPath(import.meta.url)), workspace = resolve(here, '../../../..');
const read = p => readFileSync(resolve(here, p), 'utf8'), json = p => JSON.parse(read(p));
const sha = p => createHash('sha256').update(readFileSync(resolve(here, p))).digest('hex');
const manifest = process.argv.includes('--draft-retained-monic')
  ? (await import('./bind-retained-monic.mjs')).manifest : json('retained-monic.json');
for (const [path, hash] of Object.entries(manifest.files)) assert.equal(sha(path), hash, path);
assert.deepEqual(manifest.gates, ['debug', 'release', 'clippy', 'fmt', 'wasm', 'metadata']);
assert.equal(manifest.frozenSnapshot, 'polynomial-monic-qualified-trial');
const expected = { ...json('monic-state-experiment.json').candidateSources };
assert.equal(Object.keys(expected).length, 774);
for (const [path, hash] of Object.entries(json('retained-exp-proof.json').liveSourceHashes))
  expected[`hyperreal/${path}`] = hash;
assert.deepEqual(manifest.liveSources, expected); assert.equal(Object.keys(expected).length, 954);
const previous = json('retained-polynomial-facts.json').liveSources;
for (const path of Object.keys(previous)) assert(path in expected, path);
const changed = Object.keys(expected).filter(path => expected[path] !== previous[path]).sort();
assert.deepEqual(changed, ['hypersolve/src/root_isolation.rs', 'hypersolve/src/root_isolation_monic_tests.rs']);
assert.deepEqual(manifest.changed, changed);
for (const [path, hash] of Object.entries(expected)) {
  assert.equal(sha(`${manifest.frozenSnapshot}/${path}`), hash, path);
  if (process.argv.includes('--monic-live')) assert.equal(sha(resolve(workspace, path)), hash, `live ${path}`);
}
function capture(gate) {
  assert(manifest.gates.includes(gate));
  const tag = `retained-monic-${gate}`, r = json(`results/${tag}.json`);
  assert.equal(r.tag, tag); assert.equal(r.code, 0); assert.equal(r.signal, null);
  assert.equal(r.cwd, resolve(workspace, 'hypersolve'));
  assert(Date.parse(r.finished) >= Date.parse(r.started));
  return { ...r, stdout: read(`results/${tag}.stdout`), stderr: read(`results/${tag}.stderr`) };
}
const metadata = capture('metadata');
assert.equal(metadata.command, 'cargo');
assert.deepEqual(metadata.args, ['metadata', '--offline', '--locked', '--no-deps', '--format-version', '1']);
const pkg = JSON.parse(metadata.stdout).packages.find(p => p.name === 'hypersolve'); assert(pkg);
assert.equal(pkg.manifest_path, resolve(workspace, 'hypersolve/Cargo.toml'));
const targets = pkg.targets.filter(t => t.kind.includes('lib') || t.kind.includes('test')).map(t => t.name).sort();
assert.equal(targets.length, 7);
const names = text => [...text.matchAll(/^test (.+) \.\.\. (ok|ignored|FAILED).*$/gm)].map(m => `${m[1]}:${m[2]}`).sort();
const env = ['CARGO_TARGET_DIR=/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse',
  'CARGO_INCREMENTAL=0', 'CARGO_PROFILE_DEV_DEBUG=0', 'CARGO_BUILD_JOBS=2', 'cargo'];
for (const profile of ['debug', 'release']) {
  const g = capture(profile), frozen = read(`results/monic-qualified-tests-${profile}.stdout`);
  assert.equal(g.command, 'env');
  assert.deepEqual(g.args, [...env, 'test', '--offline', '--locked', '--all-features',
    ...(profile === 'release' ? ['--release'] : [])]);
  assert.deepEqual(names(g.stdout), names(frozen)); assert.equal(names(g.stdout).length, 803);
  const suites = [...g.stdout.matchAll(/test result: ok\. (\d+) passed; (\d+) failed; (\d+) ignored; (\d+) measured; (\d+) filtered out;/g)];
  assert.equal(suites.length, 8); assert.equal(suites.reduce((n, m) => n + +m[1], 0), 803);
  assert(suites.every(m => m.slice(2).every(v => v === '0'))); assert.equal(suites.at(-1)[1], '0');
  const ran = ['hypersolve', ...[...g.stderr.matchAll(/^\s+Running tests\/([^ ]+)\.rs /gm)].map(m => m[1])].sort();
  assert.deepEqual(ran, targets); assert.match(g.stderr, /Doc-tests hypersolve/);
}
const clippy = capture('clippy'); assert.equal(clippy.command, 'env');
assert.deepEqual(clippy.args, [...env, 'clippy', '--offline', '--locked', '--all-features', '--all-targets', '--', '-D', 'warnings']);
const fmt = capture('fmt'); assert.equal(fmt.command, 'cargo');
assert.deepEqual(fmt.args, ['fmt', '--', '--check']);
const wasm = capture('wasm'); assert.equal(wasm.command, 'env');
assert.deepEqual(wasm.args, [...env, 'check', '--offline', '--locked', '--all-features', '--lib', '--target', 'wasm32-unknown-unknown']);
console.log(JSON.stringify({ checkpoint: 'retained certified monic normalization', sourceFiles: 954,
  changed, liveBytesChecked: process.argv.includes('--monic-live'), testsPerProfile: 803, suitesPerProfile: 8,
  downstreamPassed: 1761, downstreamIgnored: 9, status: manifest.status,
  acceptedCosts: manifest.acceptedCosts, limits: manifest.limits }));
