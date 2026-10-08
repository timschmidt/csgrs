// Bind completed gates only. Ongoing Hypercurve logs are intentionally excluded.
import { readFileSync, writeFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';
const here = dirname(fileURLToPath(import.meta.url));
const read = p => readFileSync(resolve(here, p));
const hash = p => createHash('sha256').update(read(p)).digest('hex');
const gates = ['hyperlimit', 'hypersolve', 'hypertri'].flatMap(crate => ['baseline', 'sign'].map(variant =>
  ({ crate, variant, profile: 'debug', tag: `sign-consumer-${crate}-${variant}-debug` })))
  .concat(['baseline', 'sign'].map(variant => ({ crate: 'hyperlattice', variant,
    profile: 'release', tag: `sign-consumer-hyperlattice-${variant}-release-unsandboxed` })));
for (const gate of gates) assert.equal(JSON.parse(read(`results/${gate.tag}.json`)).code, 0);
const otherTags = ['expression-roundtrip-compile', 'expression-roundtrip-native',
  'expression-roundtrip-compile-workspace-tmp', 'expression-roundtrip-native-built', 'expression-roundtrip-memcheck',
  'factor-association-compile', 'factor-association-native', 'factor-association-memcheck',
  'sign-consumer-probe-sign-memcheck',
  ...['baseline', 'sign'].flatMap(v => [`sign-consumer-metadata-${v}`, `sign-consumer-metadata-all-features-${v}`,
    `sign-consumer-probe-${v}-build`, `sign-consumer-probe-${v}-build-unified-path`,
    `sign-consumer-probe-${v}-release`, `sign-consumer-hyperlattice-${v}-release`])];
const sources = ['prepare-sign-consumers.mjs', 'sign-consumers.json', 'sign-consumer-probe.rs',
  'flint-expression-roundtrip-probe.c', 'flint-factor-association-probe.c',
  'bind-sign-consumer-experiment.mjs', 'verify-sign-consumer-checkpoint.mjs',
  ...['baseline', 'sign'].flatMap(v => ['Cargo.toml', 'Cargo.lock'].map(p => `sign-consumer-probe-${v}/${p}`))];
const evidence = [...gates.map(g => g.tag), ...otherTags].flatMap(t => ['json', 'stdout', 'stderr'].map(e => `results/${t}.${e}`));
assert.equal(new Set(evidence).size, evidence.length);
const snapshot = JSON.parse(read('sign-consumers.json'));
const manifest = { schema: 1, recorded: new Date().toISOString(),
  status: 'Four completed paired consumer test gates and focused public qualification; candidate not retained. Hypercurve integration suites ongoing and not bound here.',
  gates, filesPerVariant: snapshot.crates.reduce((n, c) => n+c.files.length, 0),
  bytesPerVariant: snapshot.crates.flatMap(c => c.files).reduce((n, f) => n+f.bytes, 0),
  corpus: { rows: 224, baselineDecided: 8, signDecided: 160, newDecisions: 152, beyondCapUnknown: 64 },
  donorRoundTrip: { cases: 48, parseFailures: 0, lostValue: 6 },
  donorFactors: { cases: 90, initialFailures: 0, updatedFailures: 48 },
  sourceHashes: Object.fromEntries(sources.map(p => [p, hash(p)])),
  evidenceHashes: Object.fromEntries(evidence.map(p => [p, hash(p)])),
  limits: 'No consumer timing comparison, full CI, all-profile consumer qualification or ecosystem completion. Separate scalar benchmark evidence remains bound by root-exp-sign-experiment.json. Snapshot preparation and inventory do not credit source reading. Native failures are mathematical controls, not memory-check errors. No donor patch or external report. Opaque structural comparison work remains input-sized, not wholly capped by the128 collector visits.' };
writeFileSync(resolve(here, 'sign-consumer-experiment.json'), JSON.stringify(manifest, null, 2)+'\n', { flag: 'wx' });
console.log(JSON.stringify({ sources: sources.length, evidenceFiles: evidence.length, gates: gates.length }));
