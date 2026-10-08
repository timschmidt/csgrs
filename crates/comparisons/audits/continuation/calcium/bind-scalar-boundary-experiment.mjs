// Freeze this probe and evidence without changing any donor or prior checkpoint.
import { readFileSync, writeFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
const here = dirname(fileURLToPath(import.meta.url));
const hash = p => createHash('sha256').update(readFileSync(resolve(here, p))).digest('hex');
const sources = ['flint-scalar-boundary-probe.c', 'bind-scalar-boundary-experiment.mjs', 'verify-scalar-boundary-checkpoint.mjs'];
const evidence = ['scalar-boundary-compile', 'scalar-boundary-native', 'scalar-boundary-memcheck']
  .flatMap(tag => ['json', 'stdout', 'stderr'].map(ext => `results/${tag}.${ext}`));
const record = {
  schema: 1, recorded: new Date().toISOString(),
  donorCommit: 'e269d38061d7a42070ddcffe6eb114466ed4aa7e',
  phase: { controls: 30, failures: 8 }, binary64: { controls: 10013, failures: 0 },
  rounding: { controls: 84, failures: 0 },
  sourceHashes: Object.fromEntries(sources.map(p => [p, hash(p)])),
  evidenceHashes: Object.fromEntries(evidence.map(p => [p, hash(p)])),
  limits: 'Current FLINT native build executed; archived code has the same phase mechanism but was not executed. Phase oracle uses documented principal range, rational pi fractions and special-state contracts. Binary64 oracle uses independent GMP exact conversion, not decimal printing. Rounding covers simple exact dyadics and finite imaginary components only. Memcheck success is not mathematical correctness. No donor patch, external report, Hyper production change or performance improvement claimed.',
};
writeFileSync(resolve(here, 'scalar-boundary-experiment.json'), JSON.stringify(record, null, 2) + '\n', { flag: 'wx' });
console.log(JSON.stringify({ sources: sources.length, evidenceFiles: evidence.length, phase: record.phase }));
