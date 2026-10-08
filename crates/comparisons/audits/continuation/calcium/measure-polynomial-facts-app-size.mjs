import { spawn } from 'node:child_process';
import { readFileSync, writeFileSync, copyFileSync, constants, statSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here = dirname(fileURLToPath(import.meta.url));
const target = '/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse';
const snapshots = '/tmp/calcium-polynomial-facts.ZX8NqH';
const sha = path => createHash('sha256').update(readFileSync(path)).digest('hex');
async function captured(tag, cwd, cmd, args) {
  await new Promise((ok, fail) => {
    const child = spawn(process.execPath, [resolve(here, 'capture.mjs'), tag, cwd, cmd, ...args], { stdio: 'inherit' });
    child.on('error', fail); child.on('close', code => code === 0 ? ok() : fail(Error(`${tag}: ${code}`)));
  });
}
const baseline = JSON.parse(readFileSync(resolve(here, 'reuse-app-size-summary.json'))).artifacts.filter(a => a.variant === 'reuse');
assert.equal(baseline.length, 2);
for (const artifact of baseline) for (const f of artifact.files) assert.equal(sha(f.path), f.sha256);
await captured('polynomial-facts-app-build', resolve(here, 'polynomial-facts-trial/hypercurve'), 'env', [
  `CARGO_TARGET_DIR=${target}`, 'CARGO_INCREMENTAL=0', 'CARGO_PROFILE_DEV_DEBUG=0', 'CARGO_BUILD_JOBS=2',
  'cargo', 'build', '--offline', '--locked', '--release', '--example', 'basic', '--example', 'arrangement']);
const artifacts = [];
for (const example of ['basic', 'arrangement']) {
  const path = resolve(snapshots, `app-facts-${example}`), stripped = `${path}.stripped`;
  copyFileSync(resolve(target, 'release/examples', example), path, constants.COPYFILE_EXCL);
  await captured(`polynomial-facts-app-strip-${example}`, here, 'strip', ['--strip-all', '-o', stripped, path]);
  await captured(`polynomial-facts-app-size-${example}`, here, 'size', [path, stripped]);
  await captured(`polynomial-facts-app-run-${example}`, here, stripped, []);
  artifacts.push({ example, baseline: baseline.find(a => a.example === example).files,
    facts: [path, stripped].map(path => ({ path, bytes: statSync(path).size, sha256: sha(path) })) });
}
writeFileSync(resolve(here, 'polynomial-facts-app-size-summary.json'), JSON.stringify({ recorded: new Date().toISOString(), artifacts,
  limits: 'Frozen Hypercurve basic/arrangement examples, default-feature standard release profile and separately stripped copies. Baseline is the previously qualified retained scalar with unchanged Hypersolve. Not complete Alumina, all-feature/LTO/size-optimized binaries or timing benchmarks.' }, null, 2) + '\n', { flag: 'wx' });
