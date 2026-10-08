# Multivariate rational functions — checkpoint 46

Source closure: all 36 current files in src/fmpz_mpoly_q/, its public header and
manual, totaling 3,564 lines. The archived 201-line manual is also read, but not
its implementation. Supporting reads add 87 fmpz_mpoly.h lines, 66 test-driver
header lines and the complete 37-line multiplier helper. Total: 40 read records,
3,955 new lines, 38 new complete files and two new partial files. No navigation
search, truncated output, nonexistent path or prior reread receives credit.

Architecture and transfer disposition:

- The representation is a pair of expanded integer multivariate polynomials,
  canonical under polynomial GCD one and positive denominator leading term.
  Equality compares that canonical pair. It is a formal rational-function
  field, not an arbitrary computable real or an authored partial function.
- Add/sub use zero, equal/unit denominator and scalar shortcuts. With denominator
  GCD g, cross products use the divided denominators and only factors of g need
  cancellation afterward. Scalar-denominator cases use integer content instead
  of general multivariate GCD. Multiply pre-cancels against opposite denominators;
  division reuses reciprocal multiplication with alias/sign handling.
- These growth-control patterns are useful, but Hyperreal already cross-cancels
  integer rational products and restricts mean reduction to the proved possible
  divisor. Hypersolve's local field already recognizes identical denominators,
  stores denominator one without a vector, and reduces products modulo the
  retained root polynomial. Its coefficients can be exact Reals with partial
  decisions, so a total Q[x] GCD routine is not a drop-in replacement.
- Formal cancellation can erase a factor vanishing at a specialization. Hyper's
  fiber-reduction API explicitly leaves the selected-root denominator obligation
  with the caller, and rational-image tests already certify cancellation of a
  common factor outside the isolated root. No operation here removes that
  obligation or justifies heuristic cancellation of an undecided factor.
- Ball evaluation checks the denominator enclosure before the numerator, returning
  indeterminate when it contains zero. Its broad ball/backend support is not
  newly numerically qualified. String conversion delegates to the generic-ring
  wrapper, still a separate source requirement. The private used-variable helper
  comment says AND, but the source merges flags by OR; this is a comment mismatch,
  not a proved arithmetic failure. Zero content's returned pair is one/one by
  implementation convention and is checked as such, not as a general definition
  of polynomial content.
- No nonredundant production change with justified exactness/completeness or
  performance benefit is selected. Further selected-field cancellation would
  require its own nonzero certificates and matched measurements; this checkpoint
  neither implements nor claims to disprove all such future optimizations.

Test-source findings: all 15 registered upstream tests use lexicographic contexts.
Their references share polynomial/GCD kernels. General arithmetic randomly picks
one public output alias; scalar-wrapper tests do not explicitly alias that
wrapper's output to its input. Valid generated string roundtrips do not qualify
malformed inputs or failure outputs. These are scope limitations, not donor bugs.

The new independent corpus uses twenty formal fractions in each of nine contexts:
1, 2 and 4 variables crossed with lexicographic, degree-lexicographic and
degree-reverse-lexicographic orders. Fixtures include zero/one/signs, unit/scalar/
shared denominators, common linear factors and integer contents 2^65+7 and
2^129+7 (66/130 bits). Directly constructed valid polynomial pairs are explicitly
canonicalized before arithmetic. No zero polynomial divisor, invalid context or
memory contract, excessive exponent, malformed parser input or crash probe runs.

The native harness emits complete numerator/denominator coefficients, four value
properties, numerator/denominator/union variable masks and content. It checks
canonicality using the library and exercises all binary output aliases, unary
in-place operations and rational/integer/signed-word scalar wrapper aliases.
Final input emissions prove preservation, not just internal equality assertions.

Independent BigInt regenerates all raw inputs and operation targets, then proves
cross-polynomial identities coefficient by coefficient. Because every corpus
denominator factors into known primitive linear factors and a scalar, exact
long division completely factors it, rejects any shared numerator factor and
checks integer-content GCD one. Gauss's lemma and linear irreducibility justify
this corpus-specific canonicality certificate without calling FLINT's GCD or
copying its GCD algorithm. Every alias is compared in full. The 10,044 primary
identity/canonicality certificates cover all 23,841 values, including 13,797
aliases. This is not a proof of the general multivariate GCD implementation or
merely agreement at sampled evaluation points.

All 15 upstream tests also run without filters at explicit FLINT_TEST_MULTIPLIER=1
and pass. The read no-argument driver and multiplier helper confirm the scope.
This adds random valid-input/shared-backend tests, including generated string
roundtrips, to the separate independent corpus; it does not make their references
independent. Upstream tests run natively, not under the focused Memcheck.

The first harness compile failed because GMP types were used without their
header. Original source and failed gate are preserved. The corrected source
adds only the required standard-I/O/GMP include prelude and one trailing newline;
derivation is mechanically verified. There was no executable or numerical run
from the failed build, and no donor patch. Corrected native and Memcheck output
are byte-identical (3,087,444 bytes each). Focused Memcheck reports zero errors,
suppressed contexts or live blocks; all 2,797,045 allocations are freed,
75,163,303 cumulative bytes including checks/setup. These are not donor-only
allocation, peak demand, RSS or matched performance measurements.

The independent and upstream executables are 24,280 and 36,560 bytes, totaling
60,840 bytes in /tmp/calcium-mpoly-rational.Hmk8Is. Paired numerical logs total
6,174,888 workspace bytes. Existing libraries are reused. No Rust/whole-native
rebuild, source copy, cleanup/deletion, production/donor edit, commit or push.
All 956 live scalar/consumer hashes match retained checkpoint43 and its five
continuation transfers. Inherited regression evidence remains bound to those
unchanged sources; no new Hyper speed/size claim is made.

Next: archived rational-function implementation/tests, generic-ring/fexpr bridges,
recursive polynomial/GCD and remaining scalar helpers, then all original formal,
symbolic and historical references plus unresolved transfers and full inventory
reconciliation. This checkpoint advances the full audit; it does not close it.
