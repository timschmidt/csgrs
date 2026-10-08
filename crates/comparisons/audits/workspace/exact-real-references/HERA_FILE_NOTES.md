# HERA 0.2 per-file audit — targeted transfer comparison complete

Official source: https://www2.arnes.si/~abizja4/hera/files/HERA-0.2.tar.gz

Archive SHA-256:
`b4dcd62c876c5cb4cc5bb42465dfcc7dfe71695934b9887f9e7acbf29015eae9`.
The archive has 14 regular text files, no symlinks or unsafe extraction paths.
All 14 files / 2,849 physical lines have been read; inventory and full-range
coverage TSVs reconcile hashes and lengths. Original files are unchanged.
Source dates are September 2008, not a current release date.

Source coverage, independent qualification and targeted Hyper transfer
comparison are complete. The original full native build and exhaustive
runtime safety are NOT qualified. No Hyper production change is retained.

| File | Source findings and current comparison |
| --- | --- |
| `C/hsmpfr.c` | MPFR custom storage and macro wrappers. The zero predicate calls the infinity predicate; numerical probe confirms zero is not identified. Allocator/FFI coupling is not a Hyper transfer. |
| `C/hsmpfr.h` | C prototypes for the custom/legacy MPFR interface; read against Haskell declarations and installed MPFR header. ABI width/alignment issues are isolated before numerical execution, not exercised as unsafe probes. |
| `Data/Number/Ball.hs` | Arbitrary-precision center, 32-bit-significand outward radius, half-ULP correction from MPFR ternary flags, specialized small-radius division/sqrt. Addition/subtraction and abs pass initial exact endpoint checks. Multiplication applies abs to crossing-zero inputs then restores only the centers' sign, losing possible signs; 1,620/3,136 enclosure failures. Four of 1,792 division cases fail. exp/log lower endpoints both use center+radius; exact exp(0)=1/log(1)=0 witnesses confirm exclusions. `fromWord` routes through signed Int and fails two upper-half Word cases. Fixed radius alone is not blamed for these formula defects; its possible performance/storage tradeoff remains to compare. |
| `Data/Number/Dyadic.hs` | Reexports MPFR as Dyadic, plus binary-power constructor. Hyper already has exact binary-offset representation. |
| `Data/Number/DyadicInterval.hs` | Maybe Ball, with Nothing interpreted as wholly unknown rather than empty. Intersections with Nothing return the other input; disjoint finite intersections also produce Nothing, so consistency failures are erased as unknown. Arithmetic is strict around Ball wrappers; invalid-domain output remains explicit. No unconditional transfer selected. |
| `Data/Number/FFIhelper.hsc` | Full FFI declaration file read. Original build fails on removed rounding tags; current-host alignment and MPFR exponent/string-size types require compatibility repairs. Some obsolete unused exports are removed only in the isolated test build. No original ABI success is claimed. |
| `Data/Number/MPFR.hs` | Allocates fresh ForeignPtr mantissas for pure wrappers, uses ternary flags, exact-sized dyadic arithmetic and copied nextBelow. Precision sizing uses Double log2; general huge/zero integer edge cases still need checks. Three non-underscore constant wrappers call pi instead of their named constants, and all nine bounded-value controls fail. compose divides by the decomposed exponent rather than multiplying; 15/15 round trips fail. Formatting/large exponent and finer arithmetic controls remain pending. |
| `Data/Number/Real.hs` | CReal closure plus IORef caching only the last exact requested stage, allowing coarser requests to evict finer work. Full-stage reevaluation, partial Eq/order, bounded approximation search, limits/series with caller-provided error estimates. Fast Int/Word imports freeze a 32-bit rounded point; string import similarly freezes a finite dyadic approximation to decimal input. Initial controls confirm all six large Int/Word error-bound failures and two of three decimal cases. limRec initially intersects against the seed's point enclosure even though it is not a limit enclosure: an eventually constant sequence with valid shrinking errors produces a false Less result at stage 1. General lim/series and cache behavior still need qualification. |
| `Data/Order.hs` | Explicit partial ordering/Boolean result enums. Hyper already carries certified decisions and bounded unknown outcomes. |
| `HERA.cabal` | GHC 6.8.2/6.8.3, legacy flat Cabal syntax, hsc2hs and MPFR C linkage; seven modules, no declared automated test suite. Configure succeeds with warnings on the current host; original build fails during preprocessing. |
| `LICENSE` | BSD-3-Clause text read. No implementation copied into production Hyper. |
| `README` | Minimal MPFR/Cabal build guidance and historical external build link, not numerical evidence. |
| `Setup.hs` | Standard Simple Cabal setup. |
| `demo/Demo.hs` | Harmonic/alternating sums, Chudnovsky, Borwein/Gauss-Legendre pi, e, limits and an Erdos–Borwein series. Iteration counts based on Double logarithms are not standalone tail proofs. Partial CLI parsing and obsolete catch usage; no benchmark harness with independent achieved-accuracy validation. The separately linked official Demo.hs was also read in the browser and differs in whitespace/line count; its file is not silently substituted for the archive version. |

## Qualification so far

All artifacts are in `/tmp/hera-audit.UOikl9`.

- `native/`: unchanged archive copy. Configure succeeds; build fails on
  removed GMP_RND_MAX/GMP_RNDNA constants. Logs:
  `/tmp/hera-native-configure.log`, `/tmp/hera-native-build.log`.
- `compat/`: GHC 9.6.7 compatibility copy. Changes are limited to obsolete
  unused rounding tags/random2 declaration, modern FFI constructor imports,
  actual struct alignment, correctly sized MPFR exponent/string arguments,
  unsafePerformIO import location and MonadFail constraints. Arithmetic,
  constant selection, radius formulas, cache, limits and imports are unchanged.
  `/tmp/hera-compat-source.diff` records all source differences. Seven-module
  build passes with historical/deprecation/style warnings; this is not a
  pristine native pass or a warnings-clean production gate.
- `HeraProbe.hs` compiles against that compatibility copy and stock MPFR.
  It checks exact rational ball endpoints, plus independent mathematical
  identities exp(0)=1 and log(1)=0. This avoids relying on donor Ball.contains
  as its own oracle. Run closes successfully; test failures are reported data,
  not silently treated as a passing numerical suite.
- Stock MPFR is 4.2.2. A supplementary `HeraImportProbe.hs` run records
  all nine import requests explicitly returning Right, so the inaccurate
  outputs above are not merely valid Left/limited-accuracy warnings.
  `/tmp/hera-import-probe.log` preserves the exact mantissa/exponent outputs.
- The first ordinary-workload Memcheck attempt remained at startup output
  while consuming one CPU for over three minutes. It was interrupted with
  exit 130 and its process was confirmed gone. No error summary or completed
  numerical output was produced: this is INCONCLUSIVE, not a zero-error pass.
  Investigate runtime/tool compatibility before any retry; bound future runs.
  `/tmp/hera-probe-memcheck.log` and `-memcheck-output.log` preserve the attempt.
- `/tmp/hera-probe.log`: ball add and sub each pass 3,136; mul fails
  1,620/3,136; division fails 4/1,792; abs passes 112. Known-zero exp
  enclosures fail 30/36, known-one log enclosures 6/9. Named constants fail
  9/9, isZero 1/1, compose/decompose 15/15, Ball.fromWord 2/5.
- Large Real.fromInt and Real.fromWord each fail all three 3/8/16-decimal
  absolute-error checks for 2^40+1. Decimal fromString("0.1") returns 51/512
  as a point and fails its 8/16-decimal requests. These exact-Rational controls
  test the retained real's contract, not merely display rounding.
- The limRec witness has a0=0, ak=10 for k>=1 and valid error bound
  10*2^(1-k). At stage 1 the returned comparison against exact 10 is Less;
  later tested stages are Incomparable. The first strict result is false even
  though the supplied error sequence is valid and tends to zero.

## Supporting sources and remaining work

The official project, download index, Real Haddock page and 2008 announcement
were read. The Ball Haddock page returned a browser cache miss; its complete
source documentation has been read locally. The linked 2007 RZ paper and both
slide decks were located, but their full text/figures have NOT been credited
as read. Paper browser output currently covers only its initial portion.

- https://www2.arnes.si/~abizja4/hera/
- https://www2.arnes.si/~abizja4/hera/doc/Data-Number-Real.html
- https://math.andrej.com/2008/09/03/exact-real-arithmetic-in-haskell/
- https://math.andrej.com/wp-content/uploads/2007/04/rzreals.pdf
- https://math.andrej.com/wp-content/uploads/2007/09/cca2007-slides.pdf
- https://math.andrej.com/wp-content/uploads/2007/09/domains8-slides.pdf

The opening per-file table and qualification notes above are chronological:
their pending items are resolved or explicitly bounded below. This closure
does not turn the original build, every API input or every memory path into a
passing implementation.

## Completed follow-up qualification

- `HeraKernel.hs`: 684,162 independent exact-Rational controls pass across
  dyadic extraction, directed add/sub/mul/div, rounding-indicator direction,
  directed bracketing, neg/abs/sqr/set, FMA, signed binary scaling, sqrt via
  exact rational squaring, and the supposedly exact Num add/sub/mul operators.
  Inputs cover signs, zero, exponents -127 through 129 and requested
  precisions 2 through 256. Nearest results are bracketed and their ternary
  sign is checked; this is not an exhaustive nearest-tie oracle.
- Small-radius ball division fails 2,484/17,496 exact endpoint controls.
  This is not confined to low precision: at p=64, numerator ball
  [-1-2^-36,-1+2^-36] and denominator [1-2^-36,1+2^-36] produce
  [-1-3*2^-64,-1+3*2^-64], excluding true quotient endpoints. The signed
  numerator center cancels the radius term in divSmall; an absolute-value
  bound is required. The 16/32/64/128-bit witnesses are separately logged.
  All 162 additional ball-sqrt exact-square enclosure controls pass, spanning
  the small-radius and endpoint branches. No division formula is transferred.
- General `lim`, `limRat`, `infSum`, and `infSumRec` each pass seven
  no-false-strict-comparison and five requested-accuracy controls for valid
  geometric sequences/series. The earlier limRec seed counterexample remains.
  Both series APIs time out after three seconds on the finite exact series
  a0=3, an=0 for n>0, with exact zero remainders. Source examination explains
  nontermination: rac<=ler remains 0<=0 forever. This is an exact-tail
  completeness failure, not evidence that arbitrary slow limits are invalid.
- Num.fromInteger fails 32/160 integer-grid checks. At and beyond the
  2^1024 boundary, Double-based bit-length sizing reaches infinity and chooses
  only 32 bits on this host; nearby integers are frozen incorrectly. Exact
  powers themselves may still fit, which is why the grid includes offsets.
  This is distinct from the earlier fixed-32-bit Int/Word constructors.
- A bounded Valgrind runtime-only check succeeds. Two ordinary Memcheck
  workloads then complete with zero errors: a 1001-bit integer conversion and
  a qualified radical-sum mixed-precision workload. Neither is a full leak
  check, and both leave GHC runtime allocations at exit. The original larger
  interrupted attempt remains inconclusive; no unsupported runtime workaround
  or full-suite memory-safety claim is made. Formatting/legacy unused API
  safety is not exercised as a vulnerability reproduction.

## Cache and compact-radius measurements

The donor compatibility build's cache is compared with an isolated one-line
experimental variant (`n' == n` becomes `n' >= n`). It changes no numerical
kernel. This is a scoped experiment, not a proposed upstream patch or proof
that arbitrary nonmonotone user stages should use this precise policy.

Both variants pass 156 additional independent directed-MPFR radical-sum/history
checks, covering every seed used by the timing driver. Each expression sums 32
positive square roots. Exact dyadic conversion and a 256-guard-bit directed
oracle verify the achieved decimal error before/after measurement. Cold
checksums match between variants; mixed-query checksums need not match because
returning a finer valid center is allowed. Measurements use process CPU time,
CPU 6 affinity, six alternating-order repetitions, optimized builds with CSE
and full-laziness disabled, forced results, and GC-flushed allocation accounting
outside timing. The 108 complete observations are in hera-cache-bench.tsv.

| Decimal digits | Same-accuracy query: last / finest, ns | Alternating 8/fine digits: last / finest, ns | Cold: last / finest, ns |
| --- | ---: | ---: | ---: |
| 32 | 523,557 / 2,192 | 512,269 / 4,114 | 529,654 / 532,600 |
| 256 | 536,045 / 3,316 | 522,400 / 5,022 | 544,329 / 556,354 |
| 1024 | 645,127 / 8,388 | 569,311 / 9,551 | 662,736 / 665,927 |

These are medians, not a Hyper-versus-Haskell language speed comparison.
Even same-accuracy public requests repeatedly restart at a coarser stage before
finding their previous successful finer stage, so the last-stage cache defeats
warm reuse. Finest reuse reduces cumulative allocation demand from roughly
1.95--2.14 MB/query to 7.8--115.8 KB/query in the same-accuracy workloads.
Cold allocation demand is effectively unchanged; cold time is 0.5--2.2% higher
in the variant. Managed live heaps remain similar (about 254--334 KB).
Both linked cache drivers have identical `size` totals: text 2,784,057 B,
data 131,504 B, bss 22,152 B. No binary-size reduction is claimed.

A separate radius experiment compares untouched Ball.add with a clearly
labelled full-radius reconstruction of the same formula. It runs 128-term
signed dyadic sums from preconstructed inputs; each mathematical source ball
is enclosed after radius rounding, and all 336 independent exact endpoint
controls pass. There are 48 pinned alternating-order measurements, six per
variant/precision; allocation accounting and forcing follow the cache harness.

| Center bits | Fixed-32 / full radius, ns per sum | Fixed-32 / full allocated B per sum | Fixed-32 / full managed live B |
| --- | ---: | ---: | ---: |
| 64 | 33,229 / 32,786 | 220,910 / 221,606 | 1,443,816 / 1,443,744 |
| 256 | 34,816 / 35,726 | 223,317 / 226,395 | 2,019,784 / 2,033,816 |
| 1024 | 36,554 / 40,631 | 236,456 / 252,602 | 4,435,408 / 4,532,612 |
| 4096 | 44,647 / 55,458 | 288,977 / 357,435 | 5,226,952 / 5,673,392 |

Compact radii give a real high-precision ball benefit (about 1.24x at 4096 bits,
19% less cumulative allocation), with no useful 64-bit speed win. Live figures
include retained inputs and GHC bookkeeping and are not total RSS, peak memory,
or per-ball layout sizes. The full-radius control is not a native HERA API.

## Supporting semantics review and Hyper disposition

The 21-page RZ paper is now fully read (all 1,135 extracted lines, including
every axiom appendix). Both slide decks are read completely (14 pages / 251
lines and 15 pages / 215 lines); the limit formula and representation-spectrum
diagram are also visually inspected. Official PDFs and extracted text are
preserved beside this note. SHA-256:

- rzreals.pdf: 327ae6ae0cbe62fd201d41de727242e146d587d4e0bf5cf536c6b93611dde270
- cca2007-slides.pdf: 4d2eb835d3865f1ded4955415f26e0a1587654eb68e792b670b93b00916d6804
- domains8-slides.pdf: 2c978020d7235f2d62c60a557304503f8c86b943c5ac5e4cd393eb29cf8dbe05

RZ separates realizability specifications from hand implementation: axioms in
generated comments are obligations, not a machine-checked certificate that
HERA satisfies them. Partial equivalence relations, non-extensional witnesses,
partial comparison, interval-domain completion, normalization tolerances and
limits without supplied convergence moduli are useful semantic distinctions.
The paper's ~40x and later slides' ~10x iRRAM comparisons concern historical
OCaml Era, not a verified contemporary HERA benchmark. Reading these materials
does not close the later independent RZ repository audit.

- REJECT last-request eviction. Hyper's synchronized ApproximationCache already
  preserves one finest result, rounds for coarser requests, and rejects stale
  coarser publication. It avoids the measured donor pathology without a patch.
- DO NOT replace Hyper's scalar approximation with a ball. Hyper already stores
  one integer plus its scale, with a certified implicit +/-1 error unit; there
  is no second arbitrary-precision radius to compress. The useful donor radius
  tradeoff is genuine but applies to explicit variable-width ball arithmetic,
  not this representation. Prior interval/compact-bound audits remain relevant
  to any future validated-function layer.
- General HERA limit/series constructors are an expressiveness difference, not
  something falsely claimed to be fully subsumed by Hyper's elementary tower.
  They require caller-supplied mathematical error promises and have observed
  implementation failures. No current consumer plus checked convergence/domain,
  cancellation, serialization and exact-fact contract has been established for
  importing an opaque callback node. Keep this as a future function-space/API
  design issue, not an unchecked scalar-core addition. Specialized Hyper series
  already have local proven tails and strict termination thresholds.
- Hyper's structural exact decisions, bounded unknown outcomes and multivalued
  near-integer operation already respect the relevant partiality distinctions.
  Whole-expression uniform-stage retry would discard its independently chosen
  child precision and reuse advantages; no replacement is justified here.
- Unchanged Hyperreal 21e76ea passes 227 new matching controls: 120 exact import/
  approximation-history checks, 72 independently directed-MPFR radical-sum/
  history checks, and 35 exact zero/one elementary identities. Parsing is also
  compared to exact rational Real values. Sources and the frozen driver are
  retained; no production patch or new full cross-crate gate is required.

No worthwhile new Hyper change survived this target. The earlier retained
scheduling/derivative fixes remain. The overall ecosystem audit is still active.
