# Plume performance-variant qualification

This is a separate numerical/performance checkpoint for
Plume/versioned/perform, not closure of the whole Plume report/source audit.
All36 files/4365 physical lines are read. The separate performance.tar.gz copies
are not credited by deduplication. Original174 inventoried file hashes remain
unchanged. See ../PLUME_FILE_NOTES.md and ../PLUME_READ_COVERAGE.tsv.

## Build and isolation

GHC9.6.7, Node22.22.2, Rug1.30/MPFR4.2.2, Linux x86_64. Hyperreal is unchanged
at bd92d87f0e107fda0b31b93463cc8e0cc5e26d7c. Scratch:
workspace .audit-plume-perform.LVoj1D. Use workspace TMPDIR because the user's
/tmp quota was reached in an earlier pass.

Native source does not build on this GHC: obsolete fromInt/toInt, hidden
Lookint constructor, mismatched module names and an Int/Lookint raw formatter
boundary. instrumented-native-build.log and
instrumented-decimal-before-bridge.log preserve failures. The isolated copy has
exactly instrumented-compatibility.patch applied. This changes conversions,
exports/module names and the unused raw-output type bridge, not numerical
recurrences. The FIG header edit is a separate rendering-only bridge.

Old untagged IReal requires SBinStream/SBinFloat missing from the performance
directory; those two dependencies are taken from the already-qualified main
v1.2 compatibility copy. The build log confirms IReal itself comes from perform.
Neither original donor source nor main numerical probe is modified.

From the workspace root:

```sh
env CCACHE_DISABLE=1 TMPDIR=/home/tim/Documents/GitHub/workspace/.audit-plume-perform.LVoj1D /tmp/aern2-stack.ryVCFK/programs/x86_64-linux/ghc-tinfo6-9.6.7/bin/ghc --make -O2 -rtsopts -i/home/tim/Documents/GitHub/workspace/.audit-plume-perform.LVoj1D/compat -i/home/tim/Documents/GitHub/workspace/.audit-plume.SZHOs8/compat exact-real-references/plume-qualification/InstrumentedProbe.hs -outputdir .audit-plume-perform.LVoj1D/probe-build -o .audit-plume-perform.LVoj1D/instrumented-probe
```

Repeat with -O0, outputdir debug-build and binary instrumented-probe-debug for
the independent optimization-mode run. Final probe source and frozen
InstrumentedProbe.bench-final.hs SHA-256:
af10ffbec882d11aa60a2290a3e803f187e0833b88cd2d53d101b756b3bdfd2b.
Final optimized binary SHA-256:
8daa06604a19127a7e783f7e867c0a16cedeb573e993e26d1fc8d0a069980ff0.

An initial probe indentation error was in the audit harness, not donor code.
The final build succeeds. No donor numerical repair is hidden in a native pass.

## Commands and evidence

```sh
node exact-real-references/plume-qualification/run-instrumented.mjs grids
node exact-real-references/plume-qualification/run-instrumented.mjs
node exact-real-references/plume-qualification/run-instrumented.mjs boundaries
node exact-real-references/plume-qualification/run-instrumented.mjs bench
.audit-targets/ireal-derivative-18555/release/plume_oracle donor exact-real-references/plume-qualification/instrumented-elementary-results.tsv
env CARGO_TARGET_DIR=/home/tim/Documents/GitHub/workspace/.audit-targets/ireal-derivative-18555 cargo run --offline --release --manifest-path exact-real-references/plume-qualification/Cargo.toml --bin instrumented_hyper_controls
env CARGO_TARGET_DIR=/home/tim/Documents/GitHub/workspace/.audit-targets/ireal-derivative-18555 cargo test --offline --manifest-path hyperreal/Cargo.toml --all-features --lib linear_demand
env CARGO_TARGET_DIR=/home/tim/Documents/GitHub/workspace/.audit-targets/ireal-derivative-18555 cargo test --offline --manifest-path hyperreal/Cargo.toml --all-features --lib computable::format::tests
valgrind --tool=memcheck --error-exitcode=77 --leak-check=no .audit-plume-perform.LVoj1D/instrumented-probe extra +RTS -M512m -RTS
node exact-real-references/plume-qualification/analyze-instrumented.mjs
```

Both Hyper test filters also run with --release. The additional controls parse
formatted decimal text directly using independent integer arithmetic, not
Hyper's decimal parser. An earlier oracle erroneously passed Hyper's mixed
fraction display through GMP's whitespace-ignoring fraction parser. That
harness error is preserved in instrumented-hyper-decimal-oracle-error.log; the
corrected1170 checks pass. It was not a Hyper formatting failure.

The Node child spawner initially reported EPERM even alongside valid child
output. Approved reruns outside the sandbox have no EPERM; earlier records are
under sandbox-run/ and excluded from final results. Grids record commands,
exit status, output-file hashes and bytes; other runners preserve stdout/stderr.
No HTTP service or archived SPARC binary was launched.

- Arithmetic200,017 checks:3841 failures, zero exceptions. This includes870
  failures in17,982 representation-sensitive divisions despite540 simpler
  canonical/scaled division controls passing.
- Extra1498 checks:266 failures, zero exceptions.195 valid decimal imports
  pass;96/585 decimal outputs fail. Signed logistic variants and dyadic-float
  maps pass; dyadic-stream logistic map does not compute the same recurrence
  and additionally suffers subtraction errors. Older IReal repeats the negative
  limit defect. Three selected LookInt ordering controls overflow.
-152,700 demand rows, all numerically checked:486 multiplication tags too low,
  all other measured maximum tags equal the simultaneous-prefix demand. Each
  input has an infinite spine and index tags; only digit VALUES past a cutoff
  are undefined. Binary search finds the smallest cutoff that permits all
  requested numeric output digits. This measures forced values, not spine
  matching, output-tag computation, execution time or a universal bound.
  Demands1,3,8,81 representations per operand; add/sub only inside their domains.
  The cutoff search has ceiling128; no unresolved row remains. All O0/O2 grid,
  extra, demand and example output files are byte-identical.
-108 elementary requests:100 finite prefixes; six zero cases hit256MB, two
  atan(+1) requests hit3s. Directed8192-bit MPFR rejects20 returned prefixes
  (sin9, atan6, ln5). Every input is a subset of the129 previously checked
  Hyper cases, including donor non-results. A timeout is not a nontermination
  proof. Main and instrumented numerical outputs remain separate evidence.
- Boundary probes: zero float integer divisor and initially zero upper limit
  exhaust256MB; dyadic normalization with cap2 consumes20 zero digits.
- Hyper:1170 new exact decimal/history controls plus28 tests each in debug and
  release pass. This does not claim a new full-crate/downstream run.
- Memcheck on the extra grid reports zero memory errors. The266 numerical
  failures remain failures; this is not a leak-free or full-runtime claim.

## Benchmark scope

216 fresh-process CPU6 observations; discard24 round0 warmups, leaving192,
eight per operation/family/demand, four in each pair order. Compare parity-
sensitive sbAv with carry-based sbAv3 at32/64/128/256 output digits, using256
distinct operand pairs per process in dense, sparse and terminating families.
Both input values and tags are equally pre-forced past demand. Only fresh
numeric output construction/folding is timed; exact Rational verification is
after the timer. Every benchmark output passes. Input generation, output-tag
forcing and retained-RSS are not measured. Allocation is cumulative GHC bytes
between GC/stat snapshots, not peak/live memory. No binary-size claim.

On dense inputs, parity-sensitive average takes1.18--1.30x median CPU and
1.35--1.43x allocation despite lower digit lookahead. Sparse/terminating groups
are mixed. Several times are submillisecond and pair spread is substantial;
full min/max/mean/median and paired ratios are kept, without significance or
universal speed claims. Expensive input evaluation may change the tradeoff.
These instrumented Haskell kernels are not a Hyper throughput comparison.

Hyper's demand hint schedules operands but preserves their requested precision;
it is not a purported complete dependency trace. Current scalar formatting uses
integer magnitude/sign rather than the defective stream carry rules. No new
production code, runtime metadata, stream representation or kernel is retained.
