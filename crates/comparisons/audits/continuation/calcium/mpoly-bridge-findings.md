# Checkpoint 47 — archived rational functions and expression bridges

Status: source and bounded numerical progress, not ecosystem completion.
No production/donor edit or new retained transfer. The five retained continuation
changes and all 956 live source hashes remain unchanged from checkpoint46.

## Source coverage

Read all 44 archived Calcium rational-function implementation/test/header files,
3,564 lines. Its 201-line manual was read at46, so the complete inventoried
45-file archived rational-function slice is now read. This does not include
recursive polynomial/GCD dependencies or unrelated Calcium subsystems.

Also read both generations of fexpr rational-function conversion (four files),
the current generic-ring adapter/test (two files), and current arithmetic-head
recognition, leaf collection, expanded normalization and polynomial expression
emission (four files). These 54 complete new files total 4,881 lines. The
96-line formal-expression manual excerpt and 52 generic-header lines bring the
checkpoint to 56 records/5,029 new lines. Exact records and inventory hashes are
in mpoly-bridge-read-records.json. Coverage is append-only:

- Calcium: 421 complete, one partial, 41,035 uniquely read lines.
- FLINT: 735 complete, 21 partial, 104,761 uniquely read lines.
- Combined: 1,156 complete, 22 partial, 145,796 uniquely read lines.

Initial broad filename and later Hyper symbol searches were truncated and count
only as navigation. A draft recording script omitted the inv.c note and stopped
before either output write; adding that already-read file's note allowed the
single successful append. No read range, numerical fixture or failed donor gate
was removed.

## Architecture and transfer decisions

The archive already has denominator-GCD staged addition/subtraction, restriction
of the residual cancellation to the previous GCD, coefficient-content shortcuts,
opposite-side multiplication cancellation and denominator leading-sign repair.
Its separate scalar files become consolidated current implementation files.
Archived multiplication explicitly leaves integer-denominator specialization as
a TODO; current FLINT implements those paths. This is an evolution of the same
formal fraction-field representation, not a new constructive-real architecture.

Hyperreal's rational addition in ops.rs:1019–1078 already computes denominator
GCD, forms reduced cross-products and restricts final reduction to that GCD.
Hypersolve's LocalFieldElement in algebraic_fiber.rs:3060–3118 represents unit
denominators without an allocation and recognizes shared denominators. Its
local-field implementation at3260–3346 propagates undecided predicates and
reduces residues modulo retained-root evidence. The public fiber reduction
contract at1138–1157 explicitly leaves the authored denominator's nonzero value
at the selected root as a caller obligation. These current excerpts are bound.

The fexpr manual explicitly states that conversion treats terminal expressions
as independent formal indeterminates and can implicitly divide by zero when
they have algebraic relations. Expanded normalization collects and sorts these
leaves, evaluates a formal fraction, and emits its canceled numerator/denominator.
It preserves the original expression on reported conversion failure, but does
not prove independence or recover the original domain after cancellation.
Consequently a normalization result alone is not evidence for a total Real
rewrite, equality decision, or selected-root denominator certificate.

The generic adapter distinguishes out-of-domain division/inversion from inability
to compute a polynomial power. Exact predicates are justified by its canonical
formal field, not by arbitrary exact-real decidability. Its balanced-addition
string hook, shared polynomial context helpers and default methods require their
own reads; registering a method does not qualify its internals. Commented-out
square-root/factor implementations are not active functionality. No general
thread-safety claim was tested.

The archived ball evaluator explicitly sums monomials, recomputes variable
powers and supports word exponents, with zero/constant shortcuts. The rational
evaluator checks the denominator enclosure first. Neither code provides a new
demand-driven or cached evaluation strategy to transfer. Large-exponent and
archived-runtime behavior remain unqualified; no failure reproduction attempted.

The 14 archived arithmetic tests are lex-only and use shared arithmetic/GCD
references; scalar wrappers do not explicitly test their own in-place output.
They are source-read, not newly executed. The current generic test delegates ten
random contexts to gr_test_ring; neither the complete generic suite nor its
support is claimed qualified by the bounded corpus below.

Decision: no new production candidate or matched performance claim justified
by this slice. Avoid an unchecked formal normalizer in Hyper. Field-specific
optimizations remain possible, but require independent domain proofs and matched
runtime/allocation/size measurements before retention. This is not a claim that
all future polynomial optimizations are exhausted.

## Bounded numerical qualification

Current FLINT only, native64. Reused the checkpoint46 twenty finite fraction
recipes and exact emitter as a mechanically extracted prefix, and independently
reused its BigInt polynomial/canonicality mathematics as another exact prefix
with exports. Original files are unchanged; derivations are checked byte-for-byte.

Nine contexts cover 1/2/4 variables and all three monomial orders. Explicit
symbol vectors match context lengths. Valid generated expressions only; powers
are -3,-1,0,1,2,3 and zero bases never receive negative powers. Zero to power zero
is checked as the formal polynomial convention one, not as a Hyper domain rule.
No malformed expressions, unsupported arities, huge exponents, invalid sizes,
concurrency probes or earlier failure reproduction.

The 2,052 primary full-polynomial certificates check 7,308 complete values:

- 1,062 power rows: signed generic powers (distinct/in-place), direct fexpr
  conversion, expanded-normal-form conversion, and unsigned generic powers
  (distinct/in-place) for nonnegative exponents.
- 360 numerator/denominator extraction rows, each distinct and in-place.
- 180 input-expression roundtrips, each direct and expanded-normal-form.
- 90 independently authored expression identities: cancellation, difference of
  squares, reciprocal products, signs, variadic Add/Mul, negative powers and
  polynomial-zero identities, each direct and expanded-normal-form.
- 180 input and 180 final input-preservation rows.

All 2,142 generic-operation aliases match full primary values. Independent
BigInt arithmetic regenerates inputs and mathematical targets, checks every
coefficient of the cross-polynomial identity, proves complete denominator
factorization into the corpus's known primitive irreducible linear factors,
rejects common numerator factors and checks integer-content coprimality.
Term order, leading sign, zero/unit/scalar flags, content and variable masks are
also checked. This is not a point-sampling oracle or a general multivariate GCD
proof. Part targets use input polynomials only after independent input identity
and canonicality certification.

Five captured gates pass: compile, native, linked libraries, focused Memcheck,
and independent output checker. Native/Memcheck stdout is byte-identical,
864,384 bytes each (1,728,768 paired workspace bytes). Memcheck reports zero
errors/suppressed/live blocks; all 603,105 allocations freed and 11,974,118
cumulative allocated bytes, including setup and checking. No donor-only cost,
peak/RSS, timing benchmark, all-target or archived qualification follows.

One 28,488-byte executable in /tmp/calcium-mpoly-bridge.7pQl1d reuses the five
existing libraries/configuration. No Rust/whole-native rebuild, library copy,
cleanup/deletion, commit, push or external report. All older failed numerical,
FENV, LLL and thread-memory experiments remain preserved and unrerun.

## Follow-up

Read recursive polynomial/GCD support and remaining generic/default conversion
helpers, fexpr and algebraic-number paths. Continue every original uncompleted
formal/symbolic/historical reference, unresolved transfer, and full inventory
reconciliation. Complete source reads, domain proofs and workload-specific
qualification are separate requirements; this checkpoint closes none globally.
