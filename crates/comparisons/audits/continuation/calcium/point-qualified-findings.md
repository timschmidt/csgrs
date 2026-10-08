# Guarded point-image qualification — checkpoint 54

The isolated repair now passes the guarded consumer and bounded native/WASM
qualification below. It is **not retained yet**. Nonrational/Unknown endpoint
costs, broader cache histories and WASM performance remain open. The separate
power-sum optimization and the full original ecosystem audit remain open too.

## Source and regression boundary

One complete workspace snapshot copies 956 files / 45,463,105 bytes from the
retained baseline, substituting only the byte-identical guarded
`hypersolve/src/algebraic_binary.rs` from checkpoint 53. Original manifests and
all other scalar/solver/consumer sources are unchanged. Both full snapshots and
all 956 retained live hashes are checked. Previous snapshots, failed gates and
executables remain intact; no production or donor file was edited.

- Guarded Hypercurve release, all features: 1,764 passed, zero failed, nine
  pre-existing ignored; 46 suites including the zero-test documentation suite.
  Capture ends 2026-09-10T20:40:47.732Z, code 0 / null signal. Its 507.123 seconds
  include build/test work and are not a benchmark.
- Guarded Hypercurve Clippy, all targets/all features, warnings denied: passes
  at 20:45:43.300Z.
- Guarded Hypercurve WASM library, default features plus triangulation, SVG and
  Hershey: builds at 20:45:54.655Z. This is a consumer **build**, not execution of
  the full consumer test suite on WASM.
- Prior guarded Hypersolve qualification remains 811 all-feature tests in each
  debug/release profile. The earlier 1,764-test debug consumer gate belongs to
  the unstrengthened v2 source, not this final guard. No full guarded debug
  consumer, full CI or every-feature/platform claim is made here.

## Full-value cross-platform execution

The unchanged checkpoint-52 public constructors and wire format feed the same
6,400 arithmetic queries, 40 cost-case controls and one terminal record. Every
source operand's full public record is checked unchanged around the query.
Dedicated native and WASM collectors run both the retained baseline and guard.
Each WASM execution uses a fresh Node/V8 instance with no imports, exports full
JSON bytes through linear memory, and preserves all 6,441 records, not a checksum.

A second mechanically derived collector changes exactly two call sites from
STRICT queries to `APPROXIMATE_512`, leaving every constructor, endpoint,
coefficient, operation and oracle unchanged. These rational controls exercise
the source-defined approximate-policy point-image replay branch on both
platforms; this is not instrumented branch coverage or unresolved-endpoint
qualification. The independent source derivation is checked again by the verifier.

For each policy and platform:

- Exactly 825 old lost-witness rejections become certified results; all 5,616
  other records remain unchanged. This is still one completeness gap, not 825
  new defects or a wrong-value repair.
- All 5,126 returned roots and 1,794 exact witnesses pass the unchanged rational
  polynomial/Sturm oracle. The 241 denominator guards, 968 degree guards,
  65 nonisolating controls and 40 zero-resultant Undecided records are preserved.
- The oracle executes 48,059 checks, **including 6,441 candidate self-pair
  checks**. Those self-pairs do not establish baseline equality; the separate
  full-record comparison establishes the 825 gains and 5,616 unchanged records.
  Reexecuting one oracle on identical outputs does not create independent
  mathematical oracles or expand the corpus to four unrelated sets of cases.

Baseline output is 4,332,578 bytes and candidate output 4,428,994 bytes. Every
baseline execution is byte-identical to the original baseline, and every
candidate execution to the checkpoint-53 guard, across both policies/platforms.
STRICT verification ends 20:44:15.752Z; approximate-policy verification ends in
the terminal `point-qualified-approx-public-check` capture. All gates exit 0.

The WASM instances report 18,612,224 linear-memory bytes after collection.
This includes the full JSON output, is not per-query peak memory or RSS, and
does not replace the separate checkpoint-53 allocator/Memcheck observations.

## Representative size observations

Prior retained-baseline example binaries are reused only after matching the
956-file source map, original build configuration and binary hashes. They and
new candidate copies are run; outputs match, and built-in example assertions
pass. Source/configuration identity does not eliminate build-path/linker-layout
effects. In particular, the scalar example has no changed algorithm yet changes
size, so none of these differences is asserted to be a purely algorithmic cost.

| Artifact / metric | Baseline bytes | Candidate bytes | Difference |
| --- | ---: | ---: | ---: |
| Scalar quickstart, stripped | 1,415,896 | 1,415,112 | −784 |
| Curve basic, stripped | 11,012,856 | 11,013,992 | +1,136 |
| Curve arrangement, stripped | 11,587,192 | 11,588,344 | +1,152 |
| STRICT collector, native release | 2,671,168 | 2,672,320 | +1,152 |
| STRICT collector, raw WASM | 1,211,477 | 1,212,037 | +560 |
| Approximate collector, native release | 2,670,952 | 2,672,032 | +1,080 |
| Approximate collector, raw WASM | 1,211,336 | 1,211,883 | +547 |

Native examples use ordinary default-feature release builds and separately
stripped copies, not full Alumina, LTO or a size-optimized profile. These examples
need not exercise the repaired operation. The collectors do exercise it, but
their binary sizes include test collection/serialization machinery. No whole-
application timing, general speedup or general binary-size reduction is claimed.

## Storage, integrity and next work

Fourteen dedicated native/WASM files total 68,928,629 bytes in three bounded
`/tmp/calcium-point-qualified-*` directories. Six are new application copies;
the six old baseline application copies are reused, not duplicated. All builds
share the existing Cargo target with incremental compilation disabled. Source
copies and numerical logs are in the workspace. Dedicated-byte totals exclude
shared-cache growth. At 20:49:16.941Z, `/tmp` had 16,642,371,584 available bytes;
that is a capacity snapshot, not a future quota guarantee.

Rust/Cargo 1.97.0, GNU binary tools 2.45.1, Node 22.22.2 / V8
12.4.254.21-node.39 are captured. No environment-variable dump, cleanup,
deletion, commit, push, donor patch or external issue/report submission occurred.
There are 45 new terminal successful captures; prior failed mathematical,
test and bookkeeping captures remain preserved and distinct.

`point-qualified-manifest.json` and `verify-point-qualified.mjs` bind/replay the
source footprint, exact command gates, full-value mathematical checks, policy
derivation, binary identities, sizes and resource accounting. The verifier
imports checkpoint 53's chain through 48; it does not rerun the full historical
47-chain check, whose latest full capture remains 04:56:34.285Z.

There are no new donor-read lines: 1,415 complete files / 20 partial files /
183,653 uniquely counted lines remains the selected Calcium/FLINT coverage,
not whole-ecosystem coverage. The five retained continuation transfers and
956-file live source map are unchanged.

Next: qualify nonrational and genuinely Unknown endpoint behavior/costs, with
fresh/retained inputs and broader cache histories; measure relevant WASM costs;
then make an explicit retention decision. Only afterward resume the independent
power-sum optimization's matched performance work. Do not substitute donor
qualification, this rational corpus or a build-duration comparison for those gates.
