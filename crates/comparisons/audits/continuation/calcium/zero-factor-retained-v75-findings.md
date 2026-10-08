# Certified divisor zero-factor removal retained — checkpoint 75

The exact candidate qualified in checkpoints 70–74 is promoted to live
Hypersolve. This is the seventh retained Calcium/FLINT continuation transfer,
subject to the accompanying final sealed verification. It does not close the
full ecosystem audit or qualify any unrelated current changes.

## Change and proof boundary

Only algebraic_binary.rs and the new algebraic_binary/zero_factor_tests.rs
change relative to checkpoint 67's recorded map. The new map has 957 files;
the other 955 prior identities, including the existing resultant/root-isolation
edits, are unchanged. Both promoted files are byte-identical to the qualified
176-file solver. The frozen 355-file consumer and 1,279 historical artifacts
are also rechecked. No dependency/lockfile change or broad source copy.

After the existing oversized square-free reduction, division may remove exact
rational zero coefficients from the start of the divisor polynomial only when
the remaining polynomial is nonconstant and STRICT proves its selected interval
excludes zero. A borrowed suffix avoids allocation on the ordinary path;
owned square-free storage is reused. Source witnesses/intervals remain intact.
The old signed primitive resultant orientation is preserved when nonzero using
Res(P, x^k S) = ((-1)^deg(P) P(0))^k Res(P,S). All later image, vanishing and
uniqueness checks are unchanged; no approximate sign is accepted as a proof.

The main-file change is 56 added / one removed lines, including documentation
and module wiring; the separate 260-line module adds seven tests. No source-size
reduction is claimed. The independent power-sum constructor is not included.

## Fresh live qualification

- Default: 817 passed; all-feature debug and release: 818 each. Each has eight
  suites, zero failed/ignored/filtered tests, and exact name/outcome agreement
  with the frozen checkpoint-70 candidate. The sole feature-only test is the
  existing opaque-coefficient tensor pruning test. Six parser corruption
  controls reject changed outcomes or missing success summaries.
- All-target/all-feature Clippy with warnings denied, formatting, and the
  all-feature release wasm32-unknown-unknown library build pass. This live WASM
  gate is compilation; the actual execution evidence belongs to checkpoint 72.
- Complete live/candidate Cargo metadata matches after only the two intended
  solver/scalar path normalizations: 139 packages and 139 nodes, with exactly
  one hyperlattice, hyperlimit, hyperreal and hypersolve. Lockfiles match.
- Before/after Rust, Cargo, strip, size, Node/V8, platform/architecture and kernel
  identity match the recorded checkpoint-74 environment. Runs use the existing
  nonincremental cache, two build jobs and two test threads.
- The approved driver finishes 2026-09-11T22:07:12.984Z, code 0 / null signal;
  the independent evidence check finishes 22:07:33.772Z, code 0 / null signal.
  The original environment check failed with sandbox spawnSync rustc EPERM
  before any tests ran, also failing its outer driver. Both failed captures,
  original runner and pre-run origin are preserved. The approved rerun has
  separate captures/origin and does not overwrite them.

## Inherited qualification and accepted costs

These results are inherited through exact source/configuration identity, not
relabeled as fresh live-path measurements:

- Checkpoint 70: 184 newly answered divisions and 16,508 unchanged full records
  across 16,692 paired queries; 128,652 independent polynomial/interval checks;
  384 unchanged nonrational/history records and three clean native Memcheck runs.
  Repeated policies/scales are not independent defects.
- Checkpoint 71: all 196 same-result bounded-deflation groups run at
  0.210–0.809 times baseline, with lower allocation requests/bytes/peak; a
  selected quotient drops from 8.944 to 4.957 microseconds. A roughly 7.5 ns
  early-guard cost and substantial new-answer work are accepted and disclosed.
- Checkpoints 72–73: 38,280 full WASM observations match native qualification;
  all 196 same-result bounded-deflation groups per instance mode run at
  0.216–0.790 times baseline. Six bypass combinations show small slowdowns up
  to paired 3.25%. Corrected intervals remain conditional on sampling assumptions
  and are not multiplicity-adjusted. Linear-memory capacity is not peak/RSS.
- Checkpoint 74: the frozen Hypercurve consumer passes 1,764 tests with nine
  pre-existing ignored tests, all 1,773 names/outcomes unchanged. Representative
  stripped basic/arrangement examples each grow 5,744 bytes (about 0.05%);
  unstripped growth is 9,680/9,752 bytes. Both examples and preserved baselines
  execute. Linker/path/dead-code effects prevent isolated function-size claims.

Retain for certified completeness and targeted performance/allocation gains,
accepting the documented control/new-answer/size costs under the user's ordered
priorities. No universal speed, memory or code-size improvement is claimed.

## Reproduction and open scope

From this directory, use the post-adoption entry point:

```sh
node verify-zero-factor-retained-v75.mjs
```

The verifier binds the current 957-file map and rechecks fresh gates plus frozen
historical artifacts. Old checkpoint-67 and 68–74 dynamic live-source checks
refer to older source maps; they are not current-state entry points. Their
recorded results remain historical evidence. Their algorithms/statistics are
not silently rerun against changed source through this new verifier.

No dedicated binary snapshot, cleanup, commit or push. Post-build /tmp capacity
is 14,060,937,216 bytes, versus 14,190,510,080 before the approved driver; shared
cache/other process changes are not a per-query memory measurement. No new donor
reading credit: continuation coverage remains 1,415 complete files, 20 partial
files and 183,653 uniquely read lines. Remaining supporting kernels, formal/
symbolic/historical references, unresolved transfers and inventory reconciliation
remain open. Any future power-sum qualification must account for this new baseline.
