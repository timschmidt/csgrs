// Mechanical, exclusive evidence binding; never edits prior checkpoints.
import { readFileSync, writeFileSync, readdirSync, copyFileSync, constants } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here = dirname(fileURLToPath(import.meta.url));
const read = path => JSON.parse(readFileSync(resolve(here, path)));
const sha = path => createHash('sha256').update(readFileSync(path)).digest('hex');
const files = {};
const support = ['prepare-polynomial-decision-trial.mjs', 'polynomial-decision-baseline-sources.json',
  'polynomial-decision-probe.rs', 'polynomial-decision-bench.rs', 'flint-polynomial-decision-probe.c',
  'run-polynomial-decision-bench.mjs', 'verify-polynomial-decision.mjs', 'bind-polynomial-decision.mjs',
  'polynomial-decision-read-selection.json', 'polynomial-decision-cpu-summary.json',
  'results/polynomial-decision-cpu.jsonl', 'polynomial-decision-trial/hypersolve/src/resultant.rs'];
for (const project of ['polynomial-decision-baseline', 'polynomial-decision-trial-probe',
  'polynomial-decision-baseline-bench', 'polynomial-decision-trial-bench'])
  support.push(`${project}/Cargo.toml`, `${project}/Cargo.lock`);
for (const path of support) files[path] = sha(resolve(here, path));
const gates = readdirSync(resolve(here, 'results')).filter(p => /^polynomial-decision-.*\.json$/.test(p)).map(p => {
  const r = read(`results/${p}`);
  assert.equal(r.tag + '.json', p);
  assert.equal(r.code, 0, p);
  for (const ext of ['json', 'stdout', 'stderr']) {
    const path = `results/${r.tag}.${ext}`;
    files[path] = sha(resolve(here, path));
  }
  return r.tag.replace('polynomial-decision-', '');
}).sort();
const coverage = read('coverage.json');
const reads = read('polynomial-decision-read-selection.json').map(s => {
  const entry = coverage.find(c => c.repo === s.repo && c.path === s.path);
  assert(entry); return entry;
});
const binaries = {};
for (const b of Object.values(read('polynomial-decision-cpu-summary.json').binaries)) binaries[b.path] = sha(b.path);
const root = '/tmp/calcium-polynomial-decision.ItisYX';
for (const variant of ['baseline', 'trial']) {
  const from = `/tmp/calcium-consumer-builds.X7kU2Y/calcium-reuse/release/calcium-polynomial-decision-${variant}`;
  const to = resolve(root, `polynomial-public-${variant}`);
  copyFileSync(from, to, constants.COPYFILE_EXCL); binaries[to] = sha(to);
}
const native = resolve(root, 'flint-polynomial-decision-probe'); binaries[native] = sha(native);
writeFileSync(resolve(here, 'polynomial-decision-experiment.json'), JSON.stringify({ schema: 1,
  recorded: new Date().toISOString(), files, gates, binaries, reads,
  status: 'Isolated candidate; no production transfer.',
  exclusions: ['The first48 CPU observations overlap formatting and are not performance evidence.',
    'trial-tests-release omitted --release and is a debug run; corrected release command is separate.'] }, null, 2) + '\n', { flag: 'wx' });
console.log(JSON.stringify({ boundFiles: Object.keys(files).length, gates: gates.length, binaries: Object.keys(binaries).length, reads: reads.length }));
