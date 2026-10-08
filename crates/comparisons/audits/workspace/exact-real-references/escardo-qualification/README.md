# Escardo qualification and sqrt-square transfer

See `../ESCARDO_FILE_NOTES.md`, the file/read inventories and the workspace-root
progress ledger for findings, scope and current production-gate status. This
directory preserves qualification code and raw evidence, not a replacement
exact-real implementation. All original donor files remain unchanged at
971ff20d35a4af4be595364c704ad118c4bf3c37. `Escardo.hs` differs from fun2011.hs
only by an explicit export-module header. The CC0 license is in the original
repository. `EscardoVariants.hs` is an explicitly corrected/alternative audit
control and is never counted as native donor success.

## Native reproduction

GHC9.6.7 was used, with -O2 -fno-full-laziness -fno-cse for the probes. The
native main also builds unchanged with -O2. MPFR is4.2.2 through Rug1.30.0;
the independent oracle manifest builds offline. From this directory, set
`ESCARDO_GHC` to the compiler executable and `ESCARDO_BUILD` to a fresh build
directory (prefer workspace storage: the tmpfs user quota was reached).

```sh
"$ESCARDO_GHC" -O2 -fno-full-laziness -fno-cse -rtsopts -i. -main-is EscardoProbe EscardoProbe.hs -outputdir "$ESCARDO_BUILD/probe" -o "$ESCARDO_BUILD/probe-run"
"$ESCARDO_GHC" -O2 -fno-full-laziness -fno-cse -rtsopts -i. -main-is EscardoNumerics EscardoNumerics.hs -outputdir "$ESCARDO_BUILD/numerics" -o "$ESCARDO_BUILD/numerics-run"
"$ESCARDO_GHC" -O2 -fno-full-laziness -fno-cse -rtsopts -i. -main-is EscardoBench EscardoBench.hs -outputdir "$ESCARDO_BUILD/bench" -o "$ESCARDO_BUILD/bench-run"
"$ESCARDO_BUILD/probe-run"
"$ESCARDO_BUILD/probe-run" functionals
"$ESCARDO_BUILD/probe-run" small-roots
"$ESCARDO_BUILD/probe-run" witnesses
"$ESCARDO_BUILD/numerics-run" extra
"$ESCARDO_BUILD/numerics-run" elementary
bash run-bench.sh "$ESCARDO_BUILD/bench-run"
bash run-normalizer-repeat.sh "$ESCARDO_BUILD/bench-run"
cargo build --release --offline --manifest-path Cargo.toml
```

Use `escardo_oracle donor TABLE`, `escardo_oracle hyper TABLE`, and
`escardo_oracle decimal NATIVE_STDOUT` on the resulting files. The normalizer,
division and elementary suites deliberately report native failures; exit0
means the probe completed, NOT that the numerical suite passed. Every n-digit
output is compared to an independent exact Rational / directed8192-bit MPFR
enclosure with the full2^-n signed-digit tail allowance.

Final expected row counts are243 kernel rows,
162 repeated-normalizer rows and455 elementary rows. The initial elementary
process timed out90s into a2048-bit Takano demand; its455 completed rows are
identical to the final intentionally bounded table. The initial and final
tables are both preserved. Three declared partiality probes and naive bigMid
each exceeded3s. Native BBP main prints500 characters (3. plus498 digits);
all decimal digits are independently verified. Memcheck uses `--leak-check=no`
and does not constitute full leak/runtime qualification.

## Hyper A/B reproduction and scope

Baseline is21e76ead8351f5f770f9c97165606ec969310cf9. The isolated numerical
candidate changes only the sqrt evaluator and adds an internal square-operand
query; the two unit regressions and performance note are separate. Pinned
worktrees and client manifests are in workspace `.audit-escardo.A6Oxth/`:
`hyperreal-before`, `hyperreal-abs`, `abs-before`, `abs-after`. The original
`/tmp/escardo-audit.A6Oxth` path is a symlink to this directory; no data was
deleted during quota recovery. Initially the baseline client referenced the
then-clean production worktree; after applying the change, its manifest was
redirected to the newly frozen, identical21e76ea worktree so rebuilds cannot
silently benchmark the candidate twice.

Both clients use the same `sqrt_square_controls.rs`, compiler/profile and
dependency versions. Build using the existing shared
`.audit-targets/ireal-derivative-18555` target and workspace TMPDIR. The client builds use serde;
the production gates independently cover all features. `check` verifies966
MPFR/exact/history/serde cases and one public Real::abs lossy-zero boundary.
Raw JSON shapes prove the target calls actually retain Sqrt(Square(_)).

`run-abs-bench.sh RELEASE_DIRECTORY` produces480 final rows:432 timing samples
(two variants, six cases, four demands, nine alternating-order seeds) plus48
separate allocation-count observations. All measured graphs are freshly built
and outputs forced; seven varying inputs prevent a one-value cache benchmark.
Both variants are independently checked against the mathematical oracle after
each timing sample. No bit-for-bit equality of valid approximations is assumed.
CPU is pinned to6; compilation/gates run on other cores. Time uses process CPU;
wall time is retained. Allocation increments are disabled during timing and
enabled separately. The checksum allocator wrapper still checks the flag.

The336-row short pilot is preserved but excluded from final performance claims.
Median time/rep and cumulative allocated bytes/rep are the reported quantities;
managed live-heap data apply only to the separate GHC benchmarks, not Rust.
The serde/oracle driver has368 fewer text bytes, unchanged data and384 more
BSS bytes; node layout and serialization do not change. This is not a general
binary-size claim. Hyper production retains no signed-stream implementation,
general quantifier, arbitrary continuous-function closure or new node type.

The integration/modulus/quantifier capability is broader than Hyper's current
scalar API. Its representation-invariance, totality and convergence obligations
are not satisfied by turning PredicateOutcome::Unknown into a lazy Boolean.
The donor's one-digit normalization and zero-prefix optimizations are already
matched in purpose by Hyper's magnitude-aware demand and dyadic offsets; the
new retained candidate is the narrow existing absolute-value evaluation path.
