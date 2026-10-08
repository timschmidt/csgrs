# Square-root diagonal caller pilot

Status: qualified private experiment, NOT RETAINED in production. Full ecosystem audit is
ACTIVE/OPEN. Frozen source contains397files from Hyperreal, Hyperlattice,
Hyperlimit and Hypersolve; Hyperreal HEADda26e961 and Hypersolve HEAD42f2bdb.
The baseline square-root function and seven helpers are mechanically extracted,
unchanged. `diagonal.rs` changes only its backward-loop known-coefficient sum.

At step p, coefficients0..p of the tentative degree-D root are still exact zero.
Only products with indices p+1..D-1 can contribute to coefficient D+p. The new
loop keeps their original ascending accumulation order. It reduces intermediate
products from D(D+1)^2 to D(D-1)/2; this is an operation-count argument, not a
whole-application speed claim. Leading-root validation, strict inverse/domain
checks and the final full-square identity certificate are unchanged.

## Correctness and measurement

V2 passes1,956checks each debug/release, identical output:652semantic cases
across baseline, identical-code control and diagonal. Rational expected
convolution and exported coefficients use independent BigRational arithmetic,
not Hyper equality. Cases cover squares, altered nonsquares, odd degrees,
padding, empty/zero/negative-leading boundaries, radicals and a log identity.
The original draft had a wrong symbolic test coefficient (5 instead of
3+2sqrt2). Algebraic review caught it before execution; draft source/binaries
remain intact and unqualified. V2 includes unchanged numerical and timing code.

The sandboxed /tmp linker crash and subsequent explicit host /tmp quota failure
are preserved. Unchanged sources built under the private workspace directory
`.audit-square-root.C42NO5BC`; no user files were removed to free space.

Isolated timing:48processes,1,152observations,864measured batches,
67,353,600verified measured calls, shortest CPU batch0.190104seconds. Eight
degree/cost/density/acceptance families, fresh/reused ownership separated,
three seeded processes each. Linux process CPU and wall timers; CPU6 affinity
on a shared, non-exclusive host;0.25CPU-second warmup/calibration, two rotated
warmup and six measured rounds. All identical-code controls stay within10%.
Fresh input pools are built/destroyed outside timing; input Vec clone, result
verification and destruction are included. Timed expected-result equality is
Hyper equality, backed by separate coefficient-export checks before timing.

Selected diagonal/baseline CPU medians (ranges across three process medians,
not confidence intervals): degree2/32bit dense fresh0.853(0.845–0.858),
degree8/32bit dense fresh0.424(0.423–0.430), degree16/256bit dense fresh
0.266(0.265–0.268), degree32/32bit dense fresh0.119(0.118–0.119).
These are helper-level results, not application speedups.

Separate allocation instrumentation passes96cases/288profiles, with identical
baseline/control metrics and all measured owners releasing their allocations.
Total allocated bytes never rise in this grid, but call counts and peak live
bytes can rise on reused sparse inputs. Degree16/256bit dense fresh falls from
722,456allocated bytes to219,800 and peak75,328to71,304. Reused sparse at the same
degree/cost falls from35,032to14,872allocated bytes, but peak rises8,032to10,208
and calls53to108. This cache-lifetime tradeoff must not be hidden.

## Public caller gate

Private `hypersolve-diagonal` copies all152frozen Hypersolve files. Validation
allows only Cargo package/dependency-path substitutions and the exact tested
loop replacement. Both implementations pass81public cases each debug/release:
both parameter orientations, multiple degrees/costs, two equation layouts,
independent rational factor/residual replay, and a full symbolic coefficient
identity/replay. Debug execution probes confirm one square-root helper entry in
each selected public fixture; they are separate from all timing runs.

Release public binaries have .text1,367,991baseline versus1,367,863candidate
bytes; total reported sections are equal2,266,559bytes. File bytes grow
2,826,536to2,830,856 with differing package/symbol identities. No production
binary-size saving is claimed. Candidate source adds one line overall.

Public timing completed108processes,864observations,648measured batches and
3,876,864verified measured calls; shortest CPU batch0.478316seconds. All
same-binary controls remain within10%. No owned heavy work ran concurrently.
Its seed-dependent order uses only two of the three cyclic orders, so this run
is exploratory, not fully counterbalanced. Public report comparison and
destruction are inside timing, while fresh input construction/destruction are
outside; this measures that stated caller workflow, not solver time alone.

Diagonal/baseline public CPU median ratios across three seed groups:

| Root degree / coefficient bits / layout | Fresh | Reused |
| --- | ---: | ---: |
| 1 / 8 / terminal | 1.0050 | 0.9824 |
| 2 / 8 / terminal | 0.9871 | 0.9864 |
| 4 / 8 / terminal | 1.0140 | 1.0100 |
| 8 / 32 / terminal | 1.0076 | 0.9962 |
| 1 / 8 / sampled | 1.0027 | 0.9845 |
| 4 / 32 / sampled | 1.0152 | 1.0231 |

Individual seed-group ratios span0.9527–1.0954; same-code controls span
0.9461–1.0445. These descriptive ranges are not confidence intervals. Wall
results corroborate the absence of a clear caller-level win in this grid.

Decision: do not transfer this candidate at this checkpoint. Large isolated
gains do not establish worthwhile public-caller improvement, and peak-memory
tradeoffs are mixed. No claim is made that every possible caller is unaffected.
Revisit only if profiling identifies a substantial square-root extraction cost
in a representative workload; then require fully counterbalanced caller timing,
current-source tests and explicit memory/size qualification. No production or
downstream test run is needed to deploy this rejected experiment: it is not
being deployed and no production file was changed by it.

`checkpoint-final-02.json` validates the complete isolated/public evidence,
397frozen files,152candidate files, current-source equivalence, and Ruffini's
234/245file read ledger. The preceding sandbox subprocess EPERM attempt remains
preserved as `checkpoint-final-01.json`; the authorized host rerun passes.
The numerical-stage analysis status strings describe their individual gates;
the no-transfer decision is recorded here and in the checkpoint/root ledger.
All owned processes are terminal; all other unaudited references remain in scope.
