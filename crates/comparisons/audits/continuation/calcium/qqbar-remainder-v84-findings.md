# Checkpoint 84 — remaining qqbar implementations and generic support

Three donor defect families are reproduced: a large-exponent evaluation abort,
an overflowing root degree bound, and a leaked root vector on budget failure.
Normal-input checks pass; no Hyper production change is retained. The whole
requested ecosystem audit remains incomplete.

## Source coverage

Read the remaining 16 archived and 19 current qqbar files, the delegated generic
multivariate evaluator and its test, and the unread middle of gr/qqbar.c. This
adds 37 complete files and completes one formerly partial file: 5,324 new unique
lines. Exact file/range records and snapshot hashes are in
[read records](qqbar-remainder-read-records-v84.json) and
[origin](qqbar-remainder-origin-v84.json). Both versions were read independently.

All inventoried text files under the two qqbar directories are now fully read:
149 archived files / 14,405 lines and 153 current files / 14,086 lines. This
does not imply all FLINT, Calcium, support libraries, or upstream tests are
qualified. Cumulative continuation coverage is 1,576 complete files, 19 partial
files, and 200,953 unique lines. The current gr/qqbar.c adapter is fully read;
the wider generic-ring implementation is not.

## Confirmed donor defects

1. The public qqbar multivariate wrapper dispatches a single term to an evaluator
   requiring machine-sized exponents without checking their width. At x=1,
   x^(2^64), x^(2^80), and x^(2^256) abort with an exponent-vector conversion
   exception. The generic dispatcher returns the exact result 1, and the
   two-term public versions return 2. Twenty-one successful cases have the
   correct minimal polynomial and selected real/imaginary enclosures. Exponent
   2^63 already uses 128-bit packing but still fits ulong and succeeds; packing
   width and exponent value width must not be conflated.
2. roots_poly_squarefree.c:158 multiplies two signed dimensions without checking
   overflow. The squarefree input sqrt(2)*(x^(d+1)-1)/(x-1) has d distinct roots.
   For d=57..61, the coefficient-conjugate count fits signed 64 bits but its
   product with d does not. Five compiler-instrumented cases trap; a debugger
   locates the degree-57 trap at that exact multiplication. Degrees 62 and 63,
   and degree 61 with limit 200, reject earlier. No enormous uninstrumented
   calculation was attempted. The installed reporting UBSan runtime is missing;
   GCC trap instrumentation of the unmodified translation unit supplies the
   evidence, not a fabricated UBSan report or a whole-library sanitizer run.
3. The gr/qqbar.c public root wrapper allocates croots and breaks on solver
   failure without clearing it. The first x^2-sqrt(2) cases at limits 1 and 2
   reject before this allocation and are clean. x^3-sqrt(2), limit 4, reaches
   it and leaks on every call: 360 direct plus 48 indirect bytes, four blocks,
   per call on this build. At 1/32/128 calls, normal-exit live bytes are
   408/13,056/52,224; all three Memcheck captures exit 97 with stacks in the
   root wrapper. The limit-8 success controls are clean. This is a failure-path
   ownership defect, not an objection to returning GR_UNABLE under a budget.

These are three families, not eleven independent bugs. No donor source, linked
library or Hyper source was edited; no upstream issue was filed.

## Independent execution qualification

The normal collector has 244 cases plus a terminal record: 180 multivariate
evaluations across three monomial orders, ten polynomials, three destination
alias layouts and two dispatchers; 48 root/eigenvalue outputs; and 16 triangular
matrix cases including dimensions 0..3 and repeated roots. All 252 returned
algebraic values have independent exact minimal-polynomial and selected-embedding
checks in Q(sqrt(2),sqrt(3),i). Native and Memcheck streams match. Memcheck reports
659,711 allocations/frees, 32,441,800 cumulative requested bytes, zero errors and
zero live blocks. These are collector totals, not per-call or peak costs.

The repeated root controls cover 805 queries and return 805 root values in the
successful subset. Their x^(2d)-2 minimal polynomials are irreducible by
Eisenstein at 2. Exact rational endpoint powers and signs select the correct
fourth/sixth roots and cubic-root complex components, not merely overlapping
intervals. Across those roots, 21 monomial controls and the normal collector,
1,078 algebraic values pass independent validation. Fourteen deliberately
corrupted records/streams are rejected. Failed outputs are not consumed.

Hyper runs the matching monomial cases at bases -1, 0 and 1, nested/direct
multivariate evaluations, and polynomial lengths around leaf/balancing boundaries
0,1,2,7,8,9,63,64,65,127,128,129. All 160 enclosures in 116 cases plus terminal
are independently correct. Debug/release streams match all 33,952 bytes, including
certificates. Thirty-eight equality queries prove Equal; six return Unknown:
lengths 127/128/129 at two policies. No false equality/inequality. Twelve
corruption controls pass. Fmt, Clippy and the 21-package offline dependency graph
pass after correcting one unnecessary cast in the audit harness. The original
and corrected numerical streams are identical.

## Architectural transfer decisions

- Preserve full exponent types across dispatch boundaries. Hyper's public powi
  already accepts BigInt and the tested special bases stay exact; no new wrapper
  guard is needed in that API.
- Check every derived allocation/degree dimension, not just intermediate factors.
  Hyper's independent-root degree product and integer Sylvester/quotient-ring
  matrix dimensions already use checked arithmetic in the inspected paths.
- Preserve shared coefficient fields. The donor root routine takes an independent
  Cartesian product of coefficient conjugates, ignoring correlations; its own
  TODO recognizes this. Hyper already uses source-degree quotient-ring norms
  for supported images/fibers. No wholesale Cartesian-product solver is proposed.
- The donor's survivor-count root certification is sound only with squarefree
  input and a complete candidate universe: exactly d surviving distinct roots
  then identifies the full set. A bare count heuristic is not a transferable
  substitute for those premises.
- Iterative multivariate Horner scheduling avoids recursion proportional to
  terms but materializes exponent/stack storage. Hyper's scalar evaluator already
  handles rational arguments specially and balances long polynomial expressions;
  the solver's bounded quadratic form is not a general multivariate CAS. A new
  generic evaluator needs a concrete workload and matched cost evidence.
- The six long-polynomial Unknowns suggest bounded algebraic relation retention
  as a completeness experiment, not a proved need for full factorization or an
  approved broad field rewrite. Earlier rounding, inverse-angle and complementary
  norm proof hypotheses remain open. No new implementation or speed claim.
- Numerator-only rational-root views avoid copying a harmless common denominator.
  Matrix wrappers add characteristic-polynomial construction, not a distinct
  root scheduler. Decimal print/write methods are presentation, not exact
  serialization. Random generators and metamorphic tests are useful sampling
  tools, not independent or exhaustive mathematical qualification.

## Reproducibility and limits

Pinned Calcium 8dbb16fc4fe92eaf3ebbc7478d629e994d39f944 and FLINT
e269d38061d7a42070ddcffe6eb114466ed4aa7e are unchanged. All 957 live Hyper files
and the 184-file isolated prior candidate remain bound to their prior snapshots.
No new full-stack test, all-Hyper-features run, WASM execution, matched benchmark,
allocation comparison or product-size delta is claimed. Seven continuation
transfers remain retained.

Preserved failures include the original const mismatch, donor signedness warning
under Werror, unavailable UBSan runtime, omitted audit-only fmpq_poly header,
incorrect checker assumption about exponent packing, and unnecessary Rust cast.
Only that donor warning is demoted for trap compilation; its stderr is retained.
The missing runtime needs no installation. The pre-lint Rust source is preserved
separately and hash-checked; removing its redundant cast changes no output.

Existing libraries and the shared Cargo target are reused. Four small C programs
and the two probe executables are the new build artifacts; no source-tree copy,
library rebuild, broad cleanup, commit or push. Observed /tmp available after
builds: 13,883,142,144 bytes. The complete original ecosystem inventory and open
transfer experiments still require reconciliation before a final completion claim.
