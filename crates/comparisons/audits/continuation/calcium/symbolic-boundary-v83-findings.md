# Checkpoint 83 — complex components and symbolic conversion

The pinned current FLINT symbolic importer has two confirmed false-success
families and a confirmed temporary-integer leak. These are donor defects, not
Hyper defects. No production change is retained in this checkpoint. Hyper's
corresponding angle/enclosure checks pass; some exact complex-norm equalities
remain Unknown. The full requested ecosystem audit is still open.

## Source coverage

Both pinned versions were read independently, including comments, failure paths,
tests and cleanup. Exact ranges and hashes are in
[read records](symbolic-boundary-read-records-v83.json) and
[origin](symbolic-boundary-origin-v83.json).

| File under qqbar (current: src/qqbar) | Calcium lines | FLINT lines |
|---|---:|---:|
| abs.c | 52 | 52 |
| im.c | 45 | 45 |
| re.c | 35 | 35 |
| re_im.c | 27 | 27 |
| sgn.c | 40 | 40 |
| get_fexpr.c | 1067 | 1072 |
| set_fexpr.c | 1126 | 1128 |
| test/t-abs.c | 67 | 58 |
| test/t-abs2.c | 58 | 49 |
| test/t-re_im.c | 70 | 61 |
| test/t-sgn.c | 57 | 48 |
| test/t-get_fexpr.c | 96 | 87 |
| test/t-get_fexpr_formula.c | 146 | 140 |

Total: 26 complete new files, 5,728 lines (2,886 archive; 2,842 current).
Cumulative continuation coverage: Calcium 613 complete / 66,370 lines; FLINT
925 complete, 20 partial / 129,259 lines. Combined: 1,538 complete, 20 partial,
195,629 uniquely read lines. This is not whole-ecosystem completion. Current
donor/library execution does not imply the archived or randomized upstream tests
were run. Previously read documentation, fexpr views and Hyper source were
reread without duplicate coverage credit.

## Confirmed exactness defects

1. **Missing Pi accepted as Pi.** `_fexpr_get_rational_arg_pi` permits a nonzero
   rational with no Pi factor. The public dispatcher can consequently accept
   `Sin(1)` as zero, `Cos(1)` as minus one and `Exp(i)` as minus one. The input is
   not in the documented algebraic subset, so honest rejection is acceptable;
   a false successful value is not. Independent 32-term alternating Taylor sums
   with exact rational remainder bounds separate these values from their actual
   radian values. This argument proves inequality, not transcendence.
2. **Skipped evaluation reported successful.** The Exp/Sin/Cos/Tan/Cot/Sec/Csc
   branches guard immediate-integer arguments but leave `success` unchanged if
   a numerator/denominator is a multiprecision integer. Explicit rational-Pi
   arguments such as `Sin(2^80 * Pi)` then retain a destination initialized to
   7 or -11 while returning success. The exact result is zero. These inputs are
   within the documented form. Period reduction or honest rejection would be
   sound; stale success is not.

The 308 angle/control cases plus terminal record are identical natively and
under Memcheck (48,305 bytes each). The independent checker makes 2,124 checks
and rejects eight corrupted records/streams. It classifies 148 correct rows,
112 stale successes, 24 missing-Pi successes, 20 rejected poles and four rejected
unsupported-radian forms. The 136 wrong rows exercise two defect families, not
136 independent bugs. Failed destination values are never consumed.

The source sites are current `set_fexpr.c:425` and `set_fexpr.c:749` onward.
The test corpus spans both signs and coefficient exponents 0, 30, 60, 61, 62,
63, 80 and 256; the source guard starts skipping at exponent 62 on this build.
The upstream formula tests mostly generate small arguments and do not assert
the importer return flag. The exact polynomial and both selected component
enclosures, not donor equality or approximate overlap, form the value oracle.

## Confirmed memory defect and causal control

The Pow branch initializes p and q but never clears them. Pow(1,2^80) and
Pow(1,1/2^80) legitimately fail the implementation's exponent budget, yet leak.
At 1, 32 and 256 calls, each rejected variant gives Memcheck exit 97: one lost
69,632-byte fmpz pool plus 16 indirect retained bytes per call. This is **not**
a 69 KiB per-call allocation. Pow(1,2) controls are clean.

An audit-only copy of that one translation unit, with only two fmpz_clear calls
added and the public function renamed for linkage, removes the leak. At 256
calls, all three control streams match the originals byte-for-byte; all exit
zero with zero errors and live blocks. Allocations/frees are respectively
6,390 / 4,343 / 4,600 and requested cumulative bytes 214,608 / 208,504 / 212,672.
The source copy retains its license header. No donor, linked library or Hyper
source was edited, and this control is not a production retention.

## Components, aliases and serialization

The 96 inputs are four exact rational scales times exp(pi*i*k/12), k=0..23.
Native testing covers 960 separate component/in-place operations, 576 combined
real/imaginary outputs with three alias layouts, and 960 expression conversions.
Every component and every successfully generated/parsed expression has the
correct primitive minimal polynomial and certified selected embedding. There
are 15,456 mathematical checks and ten rejected corruptions. Of 960 conversion
queries, 672 generate and round-trip correctly; 288 formula queries decline.
Internal serialization, indexed roots, nearest roots, cyclotomic formulas and
all-flags formulas succeed on all 96 inputs. Other flags have narrower coverage.

**Memory qualification for this collector failed.** It aborts in
`fmpz_lll_is_reduced_d` under Memcheck. An isolated public
abs(-7/3 * exp(pi*i/12)) call succeeds natively, but aborts under both Valgrind
`--tool=none` and Memcheck. This narrows the reproduction without assigning a
cause to FLINT, compiler code generation or Valgrind. No clean-memory claim;
abort-time live allocations are not classified as normal-exit leaks. The
original partial stream and both isolated aborts remain preserved.

Architectural observations:

- Algebraic abs/sign use exact axis/root-of-unity shortcuts before conjugate
  products and roots; complex sgn is a phase, not an ordering.
- re_im chooses projection order when an output aliases its input. Its upstream
  test exercises separate projections, not the combined wrapper/alias paths.
- Serialized polynomial/enclosure input is explicitly trusted and unchecked by
  contract. Malformed payloads are not used to manufacture a supported-input bug.
- Nearest-root presentation certifies the chosen decimal point; indexed roots
  enumerate conjugates. Formula synthesis is optional and exactly validated.
  Cubic/quartic/quintic flags are documented but unimplemented here.
- The nested-Pow arity check and builtin-dispatch assumptions merit trust-boundary
  attention, but were not executed on malformed serialization. No demonstrated
  defect is claimed for those source-only concerns.

## Hyper comparison and transfer decision

Hyper retains separate rational-radian and rational-Pi paths, and its large-Pi
reduction succeeds on the corresponding corpus. Debug/release streams match all
585 records, including complete equality-certificate text. Independent checking
passes 3,536 checks and 14 corruption controls: 304 Pi-angle rows, 56 nonzero
radian rows, 32 pole errors and 192 complex norm/magnitude rows. All returned
enclosures are correctly ordered, sufficiently narrow and contain the exact
field value or the independent rational Taylor enclosure.

Norm equality proves 144 rows and returns Unknown on 48. Those Unknowns are 24
inputs repeated at two policies: all three nonzero scales with k=1,5,7,11,13,17,
19,23. Their norm and magnitude enclosures are nonetheless correct. A narrowly
scoped complementary-angle sum-of-squares certificate is a possible completeness
candidate; it is not implemented or benchmarked here. The broad isolated field
proof candidate remains unretained because its previously measured query costs
are too high. Do not conflate these norm cases with a newly qualified candidate.

HyperComplex's pair-of-Real representation avoids algebraic projection work.
It already fuses the isolated positive-product norm and shares inverse work;
reciprocal/division deliberately use separately tuned kernels. Total algebraic
sign/equality cannot be imported as a total computable-real decision procedure.
Derived serde representation was inspected, but no malformed Hyper serde input
was tested and no deserialization defect is established in this checkpoint.

Fmt, Clippy and offline metadata pass for the 22-package public probe graph.
Only its default Hyper feature configuration is qualified; no fresh full-stack
regression, WASM test, matched speed benchmark or product-size claim is made.
Seven continuation retentions and the 957 live / 184 isolated-candidate file
maps remain unchanged.

## Evidence and resource accounting

Original compiler/driver failures from four misleading-indentation warnings are
preserved; the corrected C diagnostic changes whitespace only. The complex driver
fails on the donor assertion and is not relabeled successful. Six expected leak
exits, three SIGABRT captures and all partial/empty failure outputs are retained.
No empty successful numerical capture is accepted. One truncated ad-hoc coverage
inspection failed before editing; its bounded retry published the intended records.

Reuse pinned libraries and the existing shared Rust target. Four C executables
total 87,608 bytes; two Rust executables total 5,884,352 bytes; combined 5,971,960
bytes. These are audit artifacts, not representative product-size deltas. The
single donor source copy is a documented deviation from the initial no-copy
protocol; there is no broad source copy, library rebuild or cleanup. Observed
/tmp available after builds: 13,888,749,568 bytes. Whole-collector angle Memcheck
is clean: 13,768 allocations/frees, 438,556 cumulative requested bytes, zero live
blocks/errors; this says nothing about the aborted complex collector's leaks.

No external issue, message, commit or push was made. Remaining source/support
references, scoped transfer hypotheses and full-inventory reconciliation remain
required before claiming the original audit complete.
