import { spawn } from 'node:child_process';
import { copyFileSync, constants, readFileSync } from 'node:fs';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
const here = dirname(fileURLToPath(import.meta.url));
const config = JSON.parse(readFileSync(resolve(here, 'reuse-costs-config.json')));
async function captured(tag, command, args) {
  await new Promise((ok, fail) => {
    const p = spawn(process.execPath, [resolve(here, 'capture.mjs'), tag, here, command, ...args], { stdio: 'inherit' });
    p.on('error', fail); p.on('close', c => c === 0 ? ok() : fail(Error(`${tag}: ${c}`)));
  });
}
for (const mode of ['cpu', 'alloc']) {
  // Complete every build before starting any clock campaign; never overlap
  // compilation, another benchmark, or a memory checker with CPU observations.
  for (const phase of ['numeric', 'first-touch']) for (const variant of ['baseline', 'sign', 'reuse']) {
    const name = `calcium-reuse-${phase}-${variant}`;
    await captured(`reuse-cost-build-${phase}-${mode}-${variant}`, 'env', [
      `CARGO_TARGET_DIR=${config.buildTarget}`, 'CARGO_INCREMENTAL=0', 'CARGO_PROFILE_DEV_DEBUG=0', 'CARGO_BUILD_JOBS=2',
      'cargo', 'build', '--offline', '--locked', '--release', '--manifest-path',
      resolve(here, `reuse-${phase}-${variant}/Cargo.toml`), ...(mode === 'alloc' ? ['--features', 'allocation-count'] : [])]);
    copyFileSync(resolve(config.buildTarget, 'release', name),
      resolve(config.binarySnapshots, `${phase}-${mode}-${variant}`), constants.COPYFILE_EXCL);
  }
  for (const phase of ['numeric', 'first-touch'])
    await captured(`run-reuse-cost-${phase}-${mode}`, process.execPath, [resolve(here, 'run-reuse-cost-bench.mjs'), phase, mode]);
}
