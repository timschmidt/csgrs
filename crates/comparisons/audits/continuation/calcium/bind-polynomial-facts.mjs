import { readFileSync, writeFileSync, readdirSync, copyFileSync, constants } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here = dirname(fileURLToPath(import.meta.url));
const read = p => JSON.parse(readFileSync(resolve(here, p)));
const sha = p => createHash('sha256').update(readFileSync(p)).digest('hex');
const files = {}, support = ['prepare-polynomial-facts.mjs', 'prepare-polynomial-facts-consumers.mjs',
  'run-polynomial-facts-cpu.mjs', 'polynomial-facts-cpu-summary.json', 'results/polynomial-facts-cpu.jsonl',
  'polynomial-facts-state.rs', 'polynomial-facts-allocation.rs', 'run-polynomial-facts-allocation.mjs',
  'results/polynomial-facts-allocation.jsonl', 'run-polynomial-facts-allocation-bounded.mjs',
  'results/polynomial-facts-allocation-bounded.jsonl', 'polynomial-facts-allocation-bounded-summary.json',
  'flint-polynomial-arithmetic-probe.c', 'measure-polynomial-facts-app-size.mjs', 'polynomial-facts-app-size-summary.json',
  'polynomial-facts-read-selection.json', 'polynomial-facts-trial/hypersolve/src/resultant.rs',
  'verify-polynomial-facts.mjs', 'bind-polynomial-facts.mjs'];
for (const project of ['polynomial-facts-probe', 'polynomial-facts-bench', 'polynomial-facts-state',
  'polynomial-allocation-baseline', 'polynomial-allocation-trial', 'polynomial-allocation-facts'])
  support.push(`${project}/Cargo.toml`, `${project}/Cargo.lock`);
for (const path of support) files[path] = sha(resolve(here, path));
const gates = readdirSync(resolve(here, 'results')).filter(p => /^polynomial-facts-.*\.json$/.test(p)).map(p => {
  const r = read(`results/${p}`); assert.equal(r.tag + '.json', p);
  assert.equal(r.code, r.tag === 'polynomial-facts-allocation-run' ? 1 : 0, p);
  for (const ext of ['json', 'stdout', 'stderr']) {
    const path = `results/${r.tag}.${ext}`; files[path] = sha(resolve(here, path));
  }
  return r.tag.replace('polynomial-facts-', '');
}).sort();
assert(gates.includes('consumer-hypercurve-debug'));
const binaries = {};
for (const summary of ['polynomial-facts-cpu-summary.json', 'polynomial-facts-allocation-bounded-summary.json'])
  for (const b of Object.values(read(summary).binaries)) { assert.equal(sha(b.path), b.sha256); binaries[b.path] = b.sha256; }
for (const a of read('polynomial-facts-app-size-summary.json').artifacts)
  for (const b of [...a.baseline, ...a.facts]) { assert.equal(sha(b.path), b.sha256); binaries[b.path] = b.sha256; }
const root = '/tmp/calcium-polynomial-facts.ZX8NqH';
for (const name of ['probe', 'state']) {
  const to = resolve(root, `polynomial-facts-${name}`);
  copyFileSync(`/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse/release/calcium-polynomial-facts-${name}`, to, constants.COPYFILE_EXCL);
  binaries[to] = sha(to);
}
const native = resolve(root, 'flint-polynomial-arithmetic-probe'); binaries[native] = sha(native);
const coverage = read('coverage.json');
const reads = read('polynomial-facts-read-selection.json').map(s => {
  const entry = coverage.find(c => c.repo === s.repo && c.path === s.path); assert(entry); return entry;
});
writeFileSync(resolve(here, 'polynomial-facts-experiment.json'), JSON.stringify({ schema: 1, recorded: new Date().toISOString(),
  files, binaries, gates, reads, status: 'Qualified frozen v2 candidate; live transfer not part of this manifest.',
  exclusions: ['Initial450-row allocation run stopped on pointer-dependent live-byte occupancy; it is not the passing648-row bounded campaign.',
    'Nine ignored Hypercurve tests were not run. Different proof outcomes and instrumented allocation clocks are not equal-work CPU evidence.'] }, null, 2) + '\n', { flag: 'wx' });
console.log(JSON.stringify({ boundFiles: Object.keys(files).length, gates: gates.length, binaries: Object.keys(binaries).length, reads: reads.length }));
