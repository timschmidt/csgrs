# Few Digits qualification artifacts

The original 11-file archive is in ../FewDigits, separate from these probes.
All scope, numerical findings and transfer dispositions are recorded in
../FEW_DIGITS_FILE_NOTES.md. No production Hyper code changed in this pass.

## Builds and reproduction

GHC 9.6.7, QuickCheck 2.14.3, MPFR 4.2.2, Rust release
profile debug=0, codegen-units=16, rug 1.30.0 were used. Native donor builds
fail; `compat/` and fewdigits-compat.diff preserve the compatibility-only edits.
The originals' numerical formulas, dispatch and bounds are unchanged there.

Frozen working directory: /tmp/fewdigits-audit.hKnqR2. GHC executable:
/tmp/aern2-stack.ryVCFK/programs/x86_64-linux/ghc-tinfo6-9.6.7/bin/ghc.
QuickCheck package database:
/tmp/aern2-stack.ryVCFK/snapshots/x86_64-linux-tinfo6/a9cdb111ba43c34a9894161df33edb1b290c1c61f1b004418a8411cbdf470c15/9.6.7/pkgdb.

For each Haskell Main, select a separate output directory and compile with
`-O2 -fno-full-laziness -fno-cse -XUndecidableInstances -rtsopts -i./compat
-package-db <database> -outputdir <directory> <Main.hs> -o <binary>`.
CCACHE_DISABLE=1 was set. Probes are FewProbe.hs (few-probe), FewSemantic.hs
(few-semantic), FewMPFR.hs (few-mpfr), FewSumBench.hs (few-sum-bench) and
FewCompressionBench.hs (three variants described below). Commands in the
four shell drivers use the frozen paths and CPU6; update paths when relocating.

Rust sources are standalone bins; the included manifest targets the workspace
Hyperreal. The qualified snapshot is clean 21e76ead8351f5f770f9c97165606ec969310cf9.
The original run linked a pristine copy at /tmp/haskell-creal-audit.rafb8h/
hyperreal-before. Use the existing shared target directory
/home/tim/Documents/GitHub/workspace/.audit-targets/ireal-derivative-18555.
The polynomial bin compiles the unchanged self-contained Hypersolve symbolic
module directly, not the whole crate. Source SHA-256:
9bac6fbd6130fa47784d9c87e695aed13750c6d1d2109ba73e2e50825e9eb0eb.

## Numerical logs and limitations

Reporting probes can exit zero while reporting failed checks. In particular,
the semantic log has 12 failures, and some native QuickCheck properties fail.
The empty timeout logs are NOT passes: sqrt-zero, sqrt-perturbed, asin-one
and acos-one each ended at three seconds (exit124). Zero derivative/lift
ended with tail [] (exit1). Random intervalSqrt property ended with stack
overflow. Memcheck completed the semantic workload with zero memory errors;
that does not overturn its numerical failures or qualify leaks/full safety.

The elementary oracle uses exact dyadic inputs and directed MPFR outputs.
The compression oracle uses directed input bounds for non-dyadic Fibonacci
ratios, monotonic sine on (0,1), and reserves eps/65536 for the untimed dyadic
report projection. Thus its 54 checks constrain the ORIGINAL result, not just
the rounded report. Hyper interior/history checks use composed exp formulas
for sinh/cosh because Computable has no named methods; no claim that Real's
separate optimized hyperbolic dispatcher was exercised by those rows.

## Bench columns and policy controls

- fewdigits-sum-bench.tsv: algorithm, family, count, bit parameter, seed,
  process CPU microseconds, managed allocated bytes, result denominator digits.
  180 rows; original donor balanced LCM, sequential LCM reconstruction, strict
  left addition, and balanced addition. Each result equals an exact Rational
  left-fold oracle. Input construction and oracle execution are outside timing.
- fewdigits-hyper-sum-bench.tsv: algorithm, family, count, bit parameter, seed,
  monotonic wall microseconds, cumulative allocation/reallocation requested
  bytes, allocation/reallocation calls. 180 rows, each checked against GMP.
  This is not live memory or RSS. External sequential and balanced controls
  share Hyper's public wide-GCD kernel; current mean also has private structural
  and mixed-width shortcuts. These are scoped schedule controls, not a patched
  Hyper A/B binary. All four include final rational normalization.
- fewdigits-hyper-sum-time.tsv: same columns, 48 costly-case rows with counter
  increments disabled (zero allocation columns). Allocation still traverses
  the same forwarding allocator. This separates counter overhead from the
  schedule comparison; it is not a claim about entirely uninstrumented code.
- fewdigits-compression-bench.tsv: variant, Fibonacci index, requested bits,
  seed, process CPU microseconds, managed allocated bytes, input numerator,
  input denominator, report numerator, report denominator, ORIGINAL result
  denominator decimal digits. 54 rows, all independently MPFR-qualified.

Input lists/ratios are fully forced before timing; results are forced within
the timer. GHC allocated_bytes snapshots bracket full GCs, with post-operation
GC outside the CPU timer. Three paired seeds alternate algorithm order.
The equal-denominator *input family* can acquire divisors during canonical
reduction; it is not a promise that all reduced denominators remain equal.
Nested and dyadic bit parameters index exponents, not a guaranteed bit count.

Compression variants start from compat/Data/Real/CReal.hs:

1. `compressed` is unchanged `approxBase (x (eps/2)) (eps/2)`.
2. `rational-compressed` imports Data.Ratio(approxRational) and substitutes
   `approxRational (x (eps/2)) (eps/2)`. This reconstructs the older paper's
   compression policy, not its entire old implementation.
3. `uncompressed` substitutes `x eps`. A Fibonacci2395/2396 sine at128bits
   did not finish within ten seconds (exit124). No achieved-accuracy or finite
   timing claim is made for that unfinished variant; it is not retained.

Frozen compressed/rational-compressed binaries have text/data/bss bytes
3893414/197376/20632 and 3909926/198016/20632 respectively. These are linked
probe sizes, not library-only sizes. SHA-256:
d75b981ad80793737a800bd2cb26e39837b98d30fe1a5296a66b70a23dc6b22a,
561b5847e3222af71bfc5b3306a3f682a500db5f168820d220c70650c3aa2542.

## Corrected harness issue

The initial-partial Hyper table is explicitly superseded and is not counted
in the 180 qualified rows. The initial GMP oracle consumed Hyper Display,
but Hyper formats 35/4 as `8 3/4`, which GMP parses as 83/4 after ignoring
whitespace. This caused a false apparent mean failure. Direct signed
numerator/denominator interchange fixed the HARNESS; no Hyper defect or fix
is claimed. All final arithmetic and timing rows were rerun after correction.
