import { spawn } from 'node:child_process';
import { readFileSync, writeFileSync, copyFileSync, constants, statSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
const here = dirname(fileURLToPath(import.meta.url));
const config = JSON.parse(readFileSync(resolve(here, 'reuse-costs-config.json')));
async function captured(tag, cwd, cmd, args) {
  await new Promise((ok, fail) => {
    const child = spawn(process.execPath, [resolve(here, 'capture.mjs'), tag, cwd, cmd, ...args], { stdio: 'inherit' });
    child.on('error', fail); child.on('close', code => code === 0 ? ok() : fail(Error(`${tag}: ${code}`)));
  });
}
const artifacts = [];
for (const variant of ['baseline', 'sign', 'reuse']) {
  const root = resolve(here, variant === 'reuse' ? 'reuse-consumers/hypercurve' : `sign-consumers/${variant}/hypercurve`);
  await captured(`reuse-app-build-${variant}`, root, 'env', [
    `CARGO_TARGET_DIR=${config.buildTarget}`, 'CARGO_INCREMENTAL=0', 'CARGO_PROFILE_DEV_DEBUG=0', 'CARGO_BUILD_JOBS=2',
    'cargo', 'build', '--offline', '--locked', '--release', '--example', 'basic', '--example', 'arrangement']);
  for (const example of ['basic', 'arrangement']) {
    const path = resolve(config.binarySnapshots, `app-${variant}-${example}`);
    copyFileSync(resolve(config.buildTarget, 'release/examples', example), path, constants.COPYFILE_EXCL);
    const stripped = `${path}.stripped`;
    await captured(`reuse-app-strip-${variant}-${example}`, here, 'strip', ['--strip-all', '-o', stripped, path]);
    await captured(`reuse-app-size-${variant}-${example}`, here, 'size', [path, stripped]);
    await captured(`reuse-app-run-${variant}-${example}`, here, stripped, []);
    artifacts.push({ variant, example, profiles: ['default-features', 'release', 'stripped-release'],
      files: [path, stripped].map(path => ({ path, bytes: statSync(path).size,
        sha256: createHash('sha256').update(readFileSync(path)).digest('hex') })) });
  }
}
writeFileSync(resolve(here, 'reuse-app-size-summary.json'), JSON.stringify({ schema: 1,
  recorded: new Date().toISOString(), artifacts,
  limits: 'Unmodified frozen Hypercurve basic and arrangement executable examples, default features and standard Cargo release profile; GNU strip applied to separate copies. Not the full Alumina application, all-feature binaries, wasm sizes, or an optimized-size/LTO profile. Runtime assertions verify each stripped artifact; these executions are not timing benchmarks.' }, null, 2) + '\n', { flag: 'wx' });
