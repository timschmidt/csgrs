# Algebraic-integer quotient separation certificate — 2026-09-06

Constructible's exact norms and Hypersolve's retained rational functions
suggested avoiding repeated integer-norm formation in Hyperreal's cold scalar
certificate. No donor code, new backend, dependency, scalar field, cache field,
or public API is added. Only the meaning and reciprocal propagation of existing
AlgebraicSeparation metadata change. Original scalar graph/approximation code
and inverse nonzero guard remain unchanged.

## Exact invariant and proof

Retain an existential representation alpha=beta/gamma in one number field K,
where beta and gamma are algebraic integers and gamma!=0. Hn and Hd are
nonnegative integer upper bounds on log2 of every conjugate magnitude of beta
and gamma. The generator-degree product D bounds [K:Q].

If alpha!=0, beta!=0 and its field norm is a nonzero integer. For d=[K:Q],

    1 <= |Norm_K/Q(beta)| <= |beta| * 2^((d-1)*Hn)
    |alpha| = |beta| / |gamma| >= 2^(-Hd-(D-1)*Hn).

Thus the existing separation formula remains valid. The existing approximation
test at p<=-(bound+2) has |approx_integer|<=1 and error<=2^p, hence
|alpha|<=2^(p+1)<2^-bound, contradicting nonzero. It certifies exact zero, not
approximate equality.

Inductive preservation:

- Rational p/q: beta=p, gamma=q, integral with q!=0.
- Addition: (beta1*gamma2+beta2*gamma1)/(gamma1*gamma2); the existing max+1
  numerator bound and summed denominator bound apply unchanged.
- Multiplication/square: multiply/square numerator and denominator bounds.
- Negation and binary scaling: existing integral coefficient propagation.
- Inverse: the unchanged parser requires a proved nonzero input. Therefore
  beta!=0 and gamma/beta is valid; swap Hn and Hd, retaining K. The previous
  algorithm instead used Norm(beta) and its adjugate, multiplying bounds by
  the degree before a later zero test multiplied by degree again.
- nth root y with y^n=beta/gamma: gamma*y is integral, since its nth power is
  beta*gamma^(n-1), which is integral. The denominator remains gamma; its sign
  need not be positive. Every embedding gives the same magnitude bound through
  this nth-power identity. No unproved principal-root identity for gamma*y is
  used. Existing root-generator bookkeeping bounds the enlarged field.

All existing generator/node/bit/degree caps and fail-closed unsupported branches
remain. Both numerator and denominator use the same two u64 fields and Vec as
before; no representation storage increase. General computable-real equality
remains partial.

## Prototype and qualification so far

Private source-only mirror `.audit-fractional-separation.qYmpNk`, copied from
tracked current Hyperreal working-tree files, includes all earlier owned scalar
repairs. Baseline structural_analysis.rs SHA256
92d2e3c7fb956d3ee81a2f33243b1934e40440f04ed73e34254fef887ed81346;
candidate SHA256930e28d3400a59a6764402261f09e1e3c687c1f9b2166fbad8161b97c565163e.

Initial public field corpus:1,324/1,324 at-2048, versus baseline1,310/1,324 with
14UNKNOWN and no wrong answers. Candidate's depth-five28/28 also pass at-512,
whereas baseline needed-16384 to resolve the whole group.

Existing four selected algebraic tests pass. Four new tests include84 inverse
identities at depths1–6,336 freshly reconstructed signed perturbations at
128/512bits,245 directed-MPFR negative-denominator root cases, reciprocal-bound
involution and invalid/unproved-denominator rejection. Depthwise maximum bounds
are13,28,56,112,224,448bits. The roots test checks a conservative separation
inequality and a256bit approximation against1024bitdirectedMPFR. It does not
infer exact nonzeroness from an interval containing zero.

Private full default/debug and release/all-feature suites pass, including
doctests. Tests were subsequently copied into a separate included test module
and formatted in production; semantics match the private tests. Production
structural source matches the candidate byte-for-byte.

## Before/after prototype benchmark

Same frozen field benchmark source; timed fresh-file ingestion, parsing,
construction and exact comparison together. Both use -16384, so the baseline
must finish the same decisions. Seven original families plus940 independently
generated nonzero perturbations each at2^-20 and2^-60. No prior exact object or
comparison is shared between repetitions.

162 terminal observations,144afterwarmup,72measuredpairs,4,508,640verified
comparisons. CPU6,alternatingorder,eightpostwarmuppairs/family. No unresolved
answers, wrong results or60scaps. `analyze-bench.mjs` validates exact checks,
rawoutputs,binaryhashes and computes20,000resample percentile intervals.

| Family | After/before CPU | 95% interval |
| --- | ---: | ---: |
| quadratic | .9921 | .9761–1.0062 |
| tower1 | 1.0075 | .9575–1.0277 |
| tower2 | .9525 | .9128–.9887 |
| tower3 | .7427 | .7374–.7640 |
| tower4 | .4827 | .4655–.5002 |
| tower5 | .3037 | .2976–.3129 |
| independent | .9981 | .9787–1.0302 |
| transverse20 | 1.0186 | .9665–1.0346 |
| transverse60 | 1.0075 | .9887–1.0434 |

These are one-host measurements, not universal no-regression guarantees.
The private harness package/bin name differs from the original harness, so
final retained-source/identical-harness runs are still required. Standalone
prototype file size-256B but text+580B and bss+3504B illustrate why that initial
link-layout comparison is NOT claimed as a library/binary-size win. RSS is
whole-process, not allocation accounting; final memory checks remain pending.

Frozen original benchmark binary:
`.audit-constructible-build.l4UDoe/hyper-field-bench-release`,
SHA25625ed91a0df3f22d1f4b363b72381d4774f7c8c331c3aa3b0acc0e4d695fbb00c.
Private candidate binary:
`.audit-constructible-build.l4UDoe/fractional-bench-prototype-release`,
SHA256939975fa628c1075ba3314b35d62a205442754a3d56474d8f8185eb63a94fc3a.

## Production qualification checkpoint

The small change is retained in the working tree after full retained-source
and downstream qualification below. Earlier repairs and their frozen evidence
are unchanged. No commit/push. New source-only six-crate baseline mirror
`.audit-fractional-control.pXHyjP` is45MB and contains no build caches. It
preserves current user Hypercurve edits for any necessary before/after failure
isolation; it does not use the older coefficient-control snapshot.

Hypercurve launch snapshot: HEAD149bea52c9c16797001fee2a95cf1dc739fe82e5,
offset f37d3795ad1bead2de3e4bf7c20fb3d6fee60526c961c3d7978c1768b406af9f,
region81f75094cf225e904f53463d649ecf87971a7c97d2eba3bf25ef61406e0c7ee4,
policy6f9ede2c7e288e38b0fff1ff56e3d1fcf32e16fa980ba39e682480c8046f3d56.
All source snapshots are distinct from earlier-turn qualification; later user
edits, if any, require revalidation. Full goal remains ACTIVE and incomplete.

## Final retained results

Public corpus at-512: baseline1,296PASS/28UNKNOWN/0WRONG; retaineddebug and
release1,324PASS/0UNKNOWN/0WRONG. The14-unknown baseline at-2048 is a separate
earlier observation, not conflated with the512bit comparison.

Retained full suites:750default/debug,857release/allfeatures. Full downstream
default tests(including integration/doctests where present):Hyperlattice202,
Hyperlimit348,Hypertri7,Hypersolve796, allpass. CurrentHypercurve's complete
library suite902pass/0fail/1ignored in632.74s; allthree user-edited source hashes
above remained unchanged at completion. Previous-turn Hypercurve failures
belonged to a different user-source snapshot and are not claimed repaired by
this comparison.

Clippyallfeatures/alltargetsDwarnings, WASMlibcheck, fmtcheck and explicit
rustfmtcheck for the included new test file pass. Fuzz-target check initially
failed in C++ ccache's read-only cache; an approved retry withCCACHE_DISABLE=1
passes. The extra cargo test--all-targets--all-features gate also exits0, including
the million-bit integer benchmark smoke cases. Its generated benchmarks.md and
dispatch_trace.md were archived in generated-benchmark-reports.tar and restored
exactly to pre-run bytes; no unrelated report churn is retained.

Final identical-harness run: paired-retained-v1,162observations,72measuredpairs,
4,508,640verified comparisons. After/before CPU medians and95%bootstrap intervals:

| Family | Ratio | 95% interval |
| --- | ---: | ---: |
| quadratic | .9951 | .9722–1.0001 |
| tower1 | .9901 | .9729–1.0189 |
| tower2 | .9693 | .9537–.9834 |
| tower3 | .7339 | .7015–.7603 |
| tower4 | .4865 | .4786–.5030 |
| tower5 | .3009 | .2975–.3058 |
| independent | .9855 | .9821–1.0298 |
| transverse20 | 1.0218 | .9746–1.0374 |
| transverse60 | 1.0055 | .9900–1.0280 |

The small-control intervals overlap1; no universal no-regression assertion.
Final binary is the same package/bin/source harness as baseline. File size
1,814,128→1,813,968B; text1,284,137→1,283,965; data218,704unchanged;
bss1,048→1,208, total loaded sections-12B. Size is not the primary retention
argument and these are standalone-harness, not universal library-size numbers.

Separate counting-allocator measurements:54observations/19,224checked
comparisons, allthree repetitions identical. Depth5 allocation calls
115,983→48,239; cumulative requested bytes10,919,353→2,147,976(-80.33%);
peak requested live bytes57,265→22,728(-60.31%). Depth4 cumulativebytes
2,753,033→1,239,928; depth3 957,481→773,720. Shallow/nonzero controls have
identical calls and only aone-byteargument-path-length difference in bytes;
that byte is not a substantive memory saving. Timing from these instrumented
executables is not used. See analyze-memory.mjs and memory-analysis.json.

Retained structural sourceSHA256930e28d3400a59a6764402261f09e1e3c687c1f9b2166fbad8161b97c565163e;
nodefacade81578c53491c2c75cf445d8c7a9394ff8928a49bcbcfb069a02e4cd84b0bab5b;
formattedtestmodulea6aafd580f1afd4de6bd5e26a7f741a5bda9f3148cca61f7254c00737e883e26.
Finalbenchbinarybe460e417fd5b44c013c5e8198415c3dc84dd2e9e9a1e7dffb96aa418c12ed04;
fielddebugcc56da89151781bcb2f1464197c2fde1e66acbe4082491cf5bc73746d38067d9;
fieldrelease629aae20d1f530bf2c79c0e6336f2a8101ff5201e8b61bb3f8f139effc42182a.

Decision: KEEP for stronger bounded exact decisions, lower deep proof cost and
lower allocation, with unchanged representation/API/caps. This closes the
declared Constructible transfer question, not the full ecosystem audit.
