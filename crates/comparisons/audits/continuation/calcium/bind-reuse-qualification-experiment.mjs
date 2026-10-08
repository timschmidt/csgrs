import { readFileSync, writeFileSync } from 'node:fs';
import { createHash } from 'node:crypto';
import { dirname, resolve } from 'node:path';
import { fileURLToPath } from 'node:url';
import assert from 'node:assert/strict';

const here = dirname(fileURLToPath(import.meta.url));
const read = p => readFileSync(resolve(here, p));
const json = p => JSON.parse(read(p));
const hash = p => createHash('sha256').update(read(p)).digest('hex');
const tags = {
  'reuse-consumer-hypercurve-debug': 0,
  'reuse-consumer-hyperlimit-debug': 0,
  'reuse-consumer-hypersolve-debug': 0,
  'reuse-consumer-hypertri-debug': 0,
  'reuse-consumer-hyperlattice-release': 101,
  'reuse-consumer-hyperlattice-release-unsandboxed': 0,
  'reuse-consumer-metadata-all-features': 0,
  'reuse-consumer-probe-build': 0,
  'reuse-consumer-probe-release': 0,
  'reuse-consumer-probe-memcheck': 0,
  'reuse-memory-build': 0,
  'reuse-memory-baseline-build': 0,
  'reuse-memory-sign-build': 0,
  'reuse-memory-pilot': 0,
  'run-reuse-memory': 0,
  'reuse-memory-memcheck': 0,
  'reuse-proof-state-debug': 101,
  'reuse-proof-state-release': 0,
  'reuse-proof-state-public-debug': 0,
  'reuse-proof-state-public-release': 0,
  'reuse-proof-state-memcheck': 99,
  'reuse-thread-control-build': 0,
  'reuse-thread-control-memcheck': 99,
};
for (const profile of ['debug', 'release'])
  for (const phase of ['oracle', 'state', 'public', 'tiny'])
    tags[`root-exp-reuse-${phase}-${profile}`] = 0;
for (const variant of ['baseline', 'sign', 'reuse']) tags[`reuse-memory-elf-${variant}`] = 0;
for (const [tag, code] of Object.entries(tags)) {
  const r = json(`results/${tag}.json`);
  assert.equal(r.code, code, tag); assert.equal(r.signal, null, tag);
}
const crates = ['root-exp-reuse-qualification', 'root-exp-reuse-probe',
  'root-exp-reuse-public-trial', 'sign-consumer-probe-reuse', 'reuse-proof-state',
  ...['baseline', 'sign', 'reuse'].map(v => `reuse-live-memory-${v}`)];
const sources = ['root-exp-reuse-experiment.json', 'prepare-reuse-consumers.mjs',
  'reuse-consumers.json', 'run-reuse-scalar-gates.mjs', 'run-reuse-memory.mjs',
  'reuse-live-memory.rs', 'reuse-proof-state.rs', 'scoped-thread-control.rs',
  'root-exp-qualification.rs', 'root-exp-probe.rs', 'root-exp-sign-probe.rs',
  'sign-consumer-probe.rs', 'bind-reuse-qualification-experiment.mjs',
  'verify-reuse-qualification-checkpoint.mjs',
  ...crates.flatMap(c => ['Cargo.toml', 'Cargo.lock'].map(f => `${c}/${f}`))];
const evidence = Object.keys(tags).flatMap(t => ['json', 'stdout', 'stderr'].map(e => `results/${t}.${e}`))
  .concat(['results/reuse-memory.jsonl', 'reuse-memory-summary.json']);
const coverage = json('coverage.json');
const manifest = {
  schema: 1, recorded: new Date().toISOString(),
  status: 'V3 passes additional scalar/consumer correctness gates; isolated pending remaining cost and production qualification.',
  sourceHashes: Object.fromEntries(sources.map(p => [p, hash(p)])),
  evidenceHashes: Object.fromEntries(evidence.map(p => [p, hash(p)])),
  expectedCaptureCodes: tags,
  scalarChecksPerProfile: { mpfr: 15050, state: 261, public: 96, tiny: 144 },
  concurrentChecks: { signs: 1732, enclosures: 3456, workersPerCase: 8 },
  consumerTests: { hyperlattice: 203, hyperlimit: 361, hypersolve: 797, hypertri: 187, hypercurve: 1761 },
  ignoredConsumerTests: { hypercurve: 9 },
  consumerQueries: { total: 224, known: 160, unknown: 64 },
  donorTestCoverage: coverage.filter(c => /^(src\/)?ca\/test\/[^/]+\.c$/.test(c.path) || c.repo === 'flint' && c.path === 'src/ca.h'),
  coverage: { calcium: { complete: 172, partial: 1, lines: 19886 }, flint: { complete: 176, partial: 6, lines: 21512 } },
  memory: { observations: 162, maxRequestedRetainedBytesPerWorker: 2304,
    maxRequestedRetainedBlocksPerWorker: 32, requestedPostExitDelta: 0,
    linkedStaticTlsByteDelta: 400 },
  dispositions: {
    compilerCache: 'Original Hyperlattice sandbox run fails101 with ccache read-only filesystem; unchanged approved rerun passes.',
    privateApi: 'First debug proof-state harness fails101 on private shift_right; public multiplication replacement passes. First release was queued on Cargo lock until after correction and also passes; later release duplicate is preserved.',
    memcheck: 'Concurrent proof-state and standalone std-only scoped-thread control both return99 for one 48-byte possibly-lost Thread::new/current::init_current record. No suppression used; not a clean Memcheck claim. Separate consumer and live-worker probes have zero errors.',
    memoryCapture: 'run-reuse-memory console captures are empty despite code0. Complete162 raw rows, matching summary rows and binary hashes, not code0 alone, establish campaign coverage.',
  },
  limits: 'No production transfer or full ecosystem completion. Consumer tests use debug except Hyperlattice release, not full CI/both profiles; overlapping clocks are not performance evidence. Memory measures requested Rust heap and separately linked ELF TLS on this platform, not RSS, allocator overhead or complete thread storage. Native TLS increases400 bytes even for threads that never query this proof path. Worst-case uncached structural comparison, other sharing/collision patterns, cold-TLS latency, numerical-kernel costs and application-wide size remain open.',
};
writeFileSync(resolve(here, 'reuse-qualification-experiment.json'), JSON.stringify(manifest, null, 2) + '\n', { flag: 'wx' });
console.log(JSON.stringify({ sources: sources.length, evidenceFiles: evidence.length, status: manifest.status }));
