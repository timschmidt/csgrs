# Verus verification of Hyperreal

The goal is a Verus proof of **all Hyperreal behavior**. It is not complete.
The current proof target verifies ninety-four production contracts (eighty-six
functions and eight constants) and one hundred eighteen arithmetic model theorems. The rest
of the crate is still awaiting implementation refinement proofs. A passing
Verus job must not be described as 100% verification of Hyperreal.

The entire verification change must also meet the fixed
[pre-Verus acceptance baseline](BASELINE.md), in the order exactness,
completeness, performance, memory use, binary size and code size. Proof
milestones do not move that baseline, and proof success alone does not
establish performance or resource parity.

## Run the proofs

On Linux x86_64, with Python 3.11+, curl, unzip support in Python, and rustup:

```sh
python3 scripts/verify_verus.py install
python3 scripts/verify_verus.py verify
python3 verification/test_rejections.py
python3 verification/test_acceptance.py
python3 verification/test_dependency_identity.py
```

The installer downloads the official release pinned in
[`toolchain.json`](toolchain.json), checks its SHA-256, and installs its required
Rust toolchain without changing the default compiler. Verus itself lives in
`target/verus/`. A local archive can be supplied with `install --archive PATH`;
it receives the same checksum check. Set `VERUS=/absolute/path/to/verus` to use
an existing installation of the exact pinned version and commit.

The proof runner always passes `--no-cheating` and verifies the entire
[`lib.rs`](lib.rs) proof target. It accepts no options to skip modules, lifetime
checking, or proof checking. It checks the JSON result and the presence of all
listed kernel contracts and model theorems, and records output and source
hashes under `target/verification/`. If sources change during verification, it
rejects the result. These are proof-target checks, not a whole-crate coverage
measurement.

The explicit completion gate is:

```sh
python3 scripts/verify_verus.py verify --require-complete
```

This currently exits unsuccessfully after checking the proofs because the full
objective's obligation groups remain open and baseline qualification is pending.
[`scope.json`](scope.json) records
those obligations; it is a work ledger, not itself evidence that any requirement
is proved. There is no percentage derived from test counts, source hashes, or
the number of lemmas. Completing an obligation also requires auditing the
actual source, contracts, caller preconditions, and verifier results.

The completion gate also checks [`acceptance.json`](acceptance.json) against the
immutable baseline and priority order. Acceptance requires no unresolved items,
assessments for all six priorities with explanations and evidence references,
matching hashes of the current implementation/proof/test/workload/build inputs,
and intact repository-local evidence artifacts. Generate the input inventory
with `python3 scripts/check_verification_acceptance.py --sources`; the command
without `--sources` checks empirical acceptance independently of the proofs.
Frozen fuzz corpora and consumer snapshots belong in the hashed evidence.
The gate validates the recorded assessment and its source binding; it does not
derive performance parity from JSON fields, function counts or a single run.
The measurement and source audits in [BASELINE.md](BASELINE.md) remain required.
The assessment record is excluded from its own input hash to avoid a circular
reference; the Verus evidence separately hashes that record and the checker.
Guard tests reject stale sources, altered workloads, changed artifacts, a moved
baseline, reordered priorities and incomplete assessments.

## Current executable proof boundary

The library and proof target import **the same** files in
[`src/verified`](../src/verified). Functions retain ordinary Rust bodies;
Verus's `verus_spec` attributes attach their contracts. Normal Cargo builds
erase only the proof annotations and `proof!` blocks. There is no copied model
implementation standing in for a production body, and no new runtime
dependency on Verus. The annotation mechanism is described in the upstream
[attribute syntax guide](https://verus-lang.github.io/verus/guide/exec_attr.html).

The proof target also includes the production `computable/node/bounds.rs`
directly. Seventeen constructor, transformation and query contracts preserve the
typed bound state, exponent calculations and the distinction between zero and
an unavailable exponent. Eighteen denotation lemmas connect these checked contracts
to sign, nonzero and exact real-binade certificates, conditional on valid input
bounds. They do not yet establish the denotation of an incoming bound or cache.
Rational import, production mapping callbacks, constants and concurrent caches
remain outside this added boundary. Normal builds retain every operation.

The production `computable/node/representation.rs` file also supplies three
pure cache codecs. Bound packing preserves every typed state, sign, optional
signed exponent and exactness flag, with unused bits zero. Exact-sign packing
and decoding preserve invalid, unknown, negative, zero and positive states;
the decoder requires a valid sign tag. This does not establish the cache's
atomic operations, storage invariant, memory ownership or caller preconditions.
Word-level lemmas also establish that the construction expression retains all
four fields, that a bound update preserves the sign and construction hints,
and that a sign update preserves the bound and other fields. These lemmas use
the production constants and checked encoder contracts. They do not verify the
atomic constructor or compare-exchange loops.

The public carriers in `structural.rs` contribute eleven checked accessors and
mask operations plus seven checked mask constants. Certificate queries preserve
every known answer and every unknown state. Mask operations cover all sixteen
bits, including bits outside the named families. A closed ghost view preserves
the mask field's private visibility. The pinned verifier reports the associated
constants under `structural::impl&%3`; the scope inventory checks those exact
symbols. These contracts do not prove that certificate producers report sound
mathematical facts, or verify every generated trait implementation.

The verifier builds `verification/dependencies` with its pinned Rust compiler
and checks every registry dependency against the production lockfile. A
transparent type declaration imports the actual `num::bigint::Sign` variants;
it assumes no equality or arithmetic implementation. Dependency source and
artifact hashes are checked before and after proof checking and retained in
the evidence. Derived sign equality and unsupported standard-library methods
remain proof obligations. Conditional method markers use the pinned Verus
attribute implementation and disappear from normal builds.

| Executable kernel | Checked behavior | Production caller |
| --- | --- | --- |
| `BoundInfo::with_sign`, `BoundInfo::with_sign_msd` | Complete typed constructor result, including nonzero signs with unavailable exponents. | Computable structural bound construction |
| `BoundInfo::map_msd` | Preserves every non-magnitude field and maps exactly the available exponent through the callback's contract; zero, unknown and unavailable exponents retain their distinct states. | Binary-offset bound propagation |
| `BoundInfo::negate`, `BoundInfo::inverse`, `BoundInfo::sqrt`, `negate_sign` | Exact sign/metadata transformation; inverse exponent overflow remains unavailable; square-root exponents use floor division. | Computable bound propagation |
| `BoundInfo::square`, `BoundInfo::multiply`, `BoundInfo::add` | Complete typed propagation, including overflow, zero, absent exponents and unknown signs. Conditional denotation lemmas prove nonzero/sign facts and retained exact certificates. Addition's opposite-sign shortcut requires distinct exact binades; all new magnitude estimates remain explicitly inexact. | Computable bound propagation; producer and graph invariants remain open |
| `BoundInfo::known_msd`, `BoundInfo::planning_msd`, `BoundInfo::known_sign` | Zero sentinel iff the bound is `Zero`, and returned exponents/signs match the stored metadata. | Computable magnitude/sign queries |
| `BoundInfo::magnitude_bits`, `public_sign`, `private_sign` | Complete public magnitude export with the exactness flag preserved, and lossless conversion of negative, zero and positive signs. Conditional denotation lemmas preserve valid certificates without certifying an inexact magnitude. | Public structural facts and private sign import |
| `BoundInfo::certified_sign_and_msd` | Preserves every known sign and exact magnitude while declining inexact magnitude hints. Returned zero and binade certificates are sound conditional on valid incoming bounds. | Precision selection in reciprocal, addition, multiplication and square root; graph and approximation-kernel proofs remain open |
| `AtomicFacts::encode_bound` | Exact wire layout; the round-trip theorem recovers every typed bound, including both signed exponent limits and unavailable magnitudes. | Atomic bound-cache publication |
| `AtomicFacts::encode_exact_sign`, `AtomicFacts::decode_exact_sign` | Lossless exact-sign encoding, zero unused bits, valid encoded tags, and exact decoding for every valid sign tag. | Atomic exact-sign cache |
| Public certificate accessors | Exact extraction of known sign, ordering and equality answers; unknown stays `None`, and knownness matches the variant. | Predicate and solver certificate consumers |
| `SymbolicDependencyMask` methods and constants | Exact raw-bit import/export, subset query, emptiness and union for every `u16`; named constants retain their specified bit positions. | Structural dependency planning |
| `approximation_scale_plan` | Exact direction and shift count for every `i32` precision, including the `2^31` left shift at `i32::MIN`. | Integer-leaf approximation |
| `decode_f32` | All 32-bit patterns decode to the IEEE sign, significand and binary exponent, or the correct nonfinite class; field bounds are proved. | `TryFrom<f32> for Rational` |
| `decode_f64` | The corresponding result for all 64-bit patterns, including subnormals and both zero signs. | `TryFrom<f64> for Rational` |
| `compare_products` | A returned ordering matches exact unbounded cross products; `None` occurs exactly when a product exceeds `u128`. | Rational word-magnitude comparison |
| `signed_add` | Exact signed sum, canonical zero sign, and overflow iff the mathematical magnitude exceeds `u128`. | Rational signed-word aggregates |
| `checked_shift_left` | Exact multiplication by mathematical `2^shift`, or `None` for an unrepresentable factor or product. | Exact dyadic word scaling |
| `floor_half` | Exact floor division by two for every signed 32-bit exponent, including negative odd values, with overflow safety. | Square-root magnitude metadata |
| `checked_bit_length_msd` | Exact bit-length difference minus the comparison correction plus a signed offset; returns `None` iff the final mathematical exponent is outside `i32`. Full-width inputs and intermediate exponents outside `i32` are covered. | Rational and scaled-real magnitude metadata |
| `gcd_remainder` | Exact Euclidean remainder without overflow/underflow for `left >= right > u64::MAX`, including the high-limb quotient estimate. | `reduce_gcd_to_word` |
| `reduce_gcd_to_word` | For every pair of `u128`s, returns an ordered pair with a word-sized second operand and the same mathematical GCD; loop termination and remainder-call preconditions are proved. | Rational two-limb GCD |
| `small_gcd`, `fill_small_gcd_table`, `build_small_gcd_table`, `SMALL_GCD_TABLE` | All 4,096 compile-time table entries equal mathematical GCD; recursive initialization terminates and preserves entries outside the filled range. | `gcd_u64` |
| `binary_gcd` | Exact binary GCD for nonzero words, including removal and restoration of powers of two, subtraction, swap, overflow safety and termination. | `gcd_u64` |
| `gcd_u64` | Exact GCD for every pair of 64-bit inputs, through zero, table and binary paths. | Rational normalization and exponent GCD |
| `trailing_zeros_u128` | Exact trailing-zero count using the two 64-bit halves; the mathematical factorization theorem proves a positive odd quotient for nonzero inputs. | `gcd_u128`, `gcd_small_wide` |
| `gcd_small_wide` | Exact GCD for a nonzero word and two-limb value, including the power-of-two shortcut. | `gcd_u128` |
| `gcd_u128` | Exact GCD for every pair of 128-bit inputs; all dispatch paths, helper preconditions and final common-factor shifts are proved. | `Rational::gcd_word` |
| `fraction::reduce` | Exact GCD quotients with a positive denominator, coprime parts, unchanged fraction and canonical `0/1`; division safety and the GCD helper's contract are proved. | General word reduction, cross-cancellation, normal-component reduction and scaled word quotients |
| `fraction::checked_factors_product`, `fraction::cross_cancelled_product` | Complete numerator-major cancellation traversal with unit shortcuts, fraction preservation, positive denominators, coprime products and canonical zero. Failure iff an ordered post-cancellation numerator or denominator prefix exceeds `u128`, including overflow before a later zero. Array bounds, termination and helper preconditions are proved. | `Rational::product_term_words_cross_cancelled` |
| `limbs::compare` | Ordering agrees with the unbounded integer denoted by little-endian limb arrays. | Fixed-buffer GCD |
| `limbs::subtract_word`, `limbs::subtract` | Exact subtraction with borrow propagation; a no-larger subtrahend cannot leave a final borrow. | Fixed-buffer GCD |
| `limbs::shift_right`, `limbs::shift_left` | Right shift is division by the corresponding power of two; left shift is exact multiplication when the result fits. In-place traversal preserves unread limbs. | Fixed-buffer GCD |
| `limbs::trailing_zeros` | Zero returns the full buffer width; every nonzero array factors into the reported power of two times a positive odd integer. | Fixed-buffer GCD |
| `limbs::to_u128` | Returns the exact integer iff it fits in 128 bits. | Fixed-buffer GCD scalar exit |
| `fixed_gcd::gcd_fixed` | Exact GCD and termination for generic bounded limb arrays, including zero inputs, scalar exits, comparison, subtraction, normalization and restoration of common factors. | `Rational::gcd_fixed::<4/8>` |
| `lehmer::coefficient_fits`, `lehmer::coefficient_magnitude`, `lehmer::row_coefficients` | Exact word magnitudes and sum/difference selection for signed coefficients, rejecting precisely those larger than `u64::MAX`. | `Rational::apply_lehmer_gcd_matrix` |
| `lehmer::matrix` | Exact endpoint-agreement batch selection on ordered leading 62-bit values, including `None` cases, coefficient bounds, determinant ±1, strict reduction, checked-arithmetic safety and loop/counter bounds. | `Rational::lehmer_gcd_matrix` |
| `product::multiply_accumulate` | Exact low/high limbs of a limb product plus an existing digit and carry; the complete sum cannot overflow `u128`. | `product::multiply` |
| `product::multiply` | Exact schoolbook product for arbitrary fixed input lengths when the output has at least their combined length, including both carry loops, zero-length inputs, unused high limbs, bounds and termination. | Dyadic accumulators and compact/wide line-parameter carriers |
| `product::split_u128`, `product::multiply_u128` | Exact decomposition of all `u128` values into two limbs and exact four-limb products, with the generic multiplier's precondition discharged. | Compact dyadic products and stack accumulators |
| `accumulate::highest_nonzero`, `accumulate::aligned_digit`, `accumulate::add_word` | Exact highest occupied limb, shifted digit and addition with carry for all admissible inputs. | Direct shifted accumulation |
| `accumulate::add_shifted` | Exact in-place addition of a product times `2^shift`; `None` iff the mathematical sum exceeds the buffer. An oversized addend leaves the buffer unchanged; carry overflow leaves the sum modulo the buffer width. Zero products, arbitrary `u64` shifts, bounds and termination are covered. | Dyadic product accumulators |
| `accumulate::add_product`, `accumulate::add_wide_product` | Composition of multiplication, alignment and accumulation, including overflow and resulting buffer contents, with helper preconditions discharged. The wide wrapper retains both scalar-width paths. | `DyadicStackAccumulator` |
| `limbs::resize` | Exact conversion between limb widths, including zero extension and empty buffers; `None` iff the input cannot fit the output width. | Wide dyadic result conversion |
| `dyadic::difference` | Exact sign and magnitude of the positive accumulator minus the negative accumulator; `None` iff they cancel to zero. | `dyadic::finish` |
| `dyadic::normalize` | Exact removal of the common binary factor, preserving a positive magnitude and producing an integer or odd numerator. The resulting numerator and power-of-two denominator have GCD one. | `dyadic::finish` |
| `dyadic::finish` | Composition of difference and reduction with helper preconditions discharged: exact zero/sign classification, bounded exponent reduction, coprimality and preservation of the signed fraction by cross multiplication. | `Rational::finish_dyadic_stack_sum` |
| `aggregate::plan` | Exact per-product exponents and their maximum, including empty inputs and inactive terms; `None` iff an exponent exceeds `u64::MAX`. | Stack product-sum planning |
| `aggregate::add_product` | Both word and wide factor variants add the exact shifted product, with overflow iff the unsigned sum exceeds the buffer. Oversized addends leave it unchanged; carry overflow leaves the sum modulo its width. | `aggregate::sum_products` |
| `aggregate::sum_products` | Complete planning, signed-term dispatch, accumulation and reduction loops for arbitrary term counts and bounded buffer widths. Failure iff an exponent or either unsigned subtotal overflows; success preserves the exact signed fraction with canonical zero, sign and coprime numerator/denominator. All helper preconditions and loop termination are proved. | Narrow and wide-to-narrow stack product sums |
| `checked_shift_left_u64` | Exact native alignment with the complete `u64` metadata range checked before narrowing; failure iff the factor or result cannot fit a word, including zero magnitudes. | Native dyadic paths |
| `dyadic::normalize_word` | Direct `u128` reduction with exact factor removal, coprimality, fraction preservation, canonical activity/sign and zero exponent, including maximal metadata. | Native differences and two-product sums |
| `dyadic::difference_word` | Exact signed difference and canonical fraction; fallback iff either raw alignment or the signed difference exceeds the native range. Both raw alignments are checked even for inactive inputs. | `Rational::difference_dyadic_words` |
| `aggregate::native_product` | Inactive terms produce zero; active terms require a word factor, representable alignment, and both raw/scaled products to fit. Success has the exact signed value and canonical zero sign. | Native product-sum fast path |
| `aggregate::sum_products_word` | Complete two-product planning, native multiplication/alignment, signed addition and direct reduction, with exact fallback conditions and helper preconditions discharged. | `Rational::product_sum2_dyadic_words_word` |

The GCD specification is proved to preserve exactly the positive common
divisors, to be positive away from `(0, 0)`, and to be the greatest common
divisor. Further lemmas prove symmetry, subtraction, scaling, odd-operand
power-of-two cancellation, trailing-zero factorizations and exact bounded
shifts. The limb model defines the positional integer value of an array and
proves bounds, splitting, prefix equality and the characterization of zero.
The Lehmer model proves that an integer matrix with determinant ±1 preserves
GCD for arbitrary input magnitudes after taking absolute row values. A second
theorem proves the sum/absolute-difference identity used by unsigned row
application. The executable builder refines a bounded recursive specification
over mathematical integers; coefficient growth proves the original `u8` step
counter stays below 129, so the model's recursion budget covers every step.
The product proof tracks the integer represented by all processed rows and the
pending carry. A positional-update lemma connects each array write to its
integer contribution. This proves the final carry slot is zero before each
write, including the slot where the former dyadic loops had an overflow guard.
The alignment theorems characterize the highest shifted limb and prove that
the assembled digits denote multiplication by the corresponding power of two.
Accumulation then preserves the complete integer sum plus the pending carry;
its final overflow decision and partially written failure state are proved.
Dyadic reduction lemmas prove exact division by a prefix of a known binary
factor, preservation of the signed fraction, and coprimality of a canonical
dyadic numerator and denominator. The finishing kernel composes these with
the existing limb comparison, subtraction, trailing-zero and shift proofs.
The aggregate model relates positive/negative subtotals to the signed sum of
aligned products. It proves that each alignment preserves its original
fraction and that raising the common denominator rescales the entire numerator
exactly. Monotonic unsigned subtotals establish the complete loop's failure
condition even when a later cancellation would make the final difference fit.
Native-word normalization reuses the proved two-limb trailing-zero count and
exact factor-division theorem. Its word-value model treats inactive inputs as
zero and proves fraction-preserving alignment. Native differences and product
sums keep their distinct raw-alignment and inactive-term rules while proving
the complete success/fallback boundary and canonical results.
[`rational_model.rs`](rational_model.rs) defines signed, unbounded
fractions with positive denominators, without assuming canonical reduction.
Its twenty-one theorems establish equivalence and ordering laws, rescaling,
arithmetic closure, ring identities, reciprocal correctness, and respect for
equivalent representations. GCD cancellation proves that normalization yields
a canonical signed fraction with the same value, including zero. The native
fraction reducer uses those same exact quotient identities. Its eight production
call sites retain their existing branch selection and arithmetic order; their
positive-denominator preconditions and surrounding algorithms still need caller
proofs. These model theorems do not yet establish that
all `Rational` methods implement the model.

Bézout witnesses are constructed by induction over the mathematical Euclidean
recurrence. They establish Euclid's divisibility lemma, preservation of
coprimality under products and exact quotient cancellation, and uniqueness of
signed reduced fractions with positive denominators. Normalization is therefore
idempotent, and comparing normalized parts characterizes rational equivalence
even for unreduced inputs. The native cross-cancellation traversal now refines
these mathematical foundations. Rational equality and the surrounding callers
still require refinement proofs. The mathematical theorems are erased from
normal Rust builds.

Factor-sequence theorems prove that dividing a factor divides every containing
prefix, cross-cancelling any pair preserves the complete fraction, and pairwise
cross-coprimality produces coprime numerator and denominator products. Reducing
the next pair also preserves coprimality of every pair already processed by
the numerator-major traversal, including zero numerators. These theorems
include empty products and zero numerators. Prefix order remains explicit:
a native checked product can overflow before encountering a later zero.
The executable kernel preserves the original numerator-major pair order and
unit shortcuts, then checks numerator and denominator prefixes in their original
order. The adapter that imports `BigUint` parts and establishes positive native
denominators remains outside this proof boundary. The new loop form also needs
runtime qualification against the fixed baseline; verification alone does not
establish code-generation or performance parity.

The `floor_half` contract covers the integer exponent calculation.
[`bound_denotation.rs`](bound_denotation.rs) proves the positive-root binade
identity and connects it to the actual `BoundInfo::sqrt` contract, including
negative odd exponents. Constructor, negation and reciprocal contracts preserve
valid sign and exact-magnitude certificates. A separated approximation yields
a sound sign/nonzero bound, and bound queries return sound certificates.
Each lemma names the checked production method through `call_ensures`; no
copied implementation supplies those contracts. Inexact planning magnitudes
have no claimed error bound here. Establishing valid inputs in the expression
graph, rational import, cache publication and all callers remains open.
Binary scaling shifts exact real binades by the same mathematical exponent.
The mapping refinement connects this fact to `map_msd` when its callback returns
the checked exponent sum; unrepresentable results retain valid sign and nonzero
certificates without asserting a magnitude. The production callback closures
and the expression graph's incoming certificates still need caller proofs.
The checked bit-length helper proves the final exponent range calculation.
Its shared exponent specification is connected to five theorems in
[`magnitude_model.rs`](magnitude_model.rs). Bit-length bounds and one aligned
comparison determine the unique interval `2^msd <= |n/d| < 2^(msd+1)`.
Aligned equality characterizes an exact power-of-two scale even for unreduced
parts. Signed binary scaling moves that interval by exactly its exponent.
These theorems support arbitrary mathematical exponents; the executable helper
checks only the final result against `i32`.
Rational queries retain full-width lengths and shifts, and real queries add
the computable offset before narrowing. The `BigUint` bit-length/shifted
comparison semantics and the full real-magnitude certificates still require
refinement proofs.

Four theorems in [`bit_length_model.rs`](bit_length_model.rs) connect the
standard-library leading-zero specification and normalized limb denotation to
exact mathematical bit lengths. They prove bit-length growth under arbitrary
binary shifts and ordering when exact widths differ. Normalization of actual
`BigUint` storage, its digit iterator, machine-width length arithmetic and the
runtime borrowed comparison still require implementation proofs.

Three theorems in [`integer_approximation_model.rs`](integer_approximation_model.rs)
prove signed nearest-integer rounding, with halfway values toward positive
infinity, for the production precision plan. They also prove that coarsening a
rational-centered cached approximation preserves its one-unit error bound.
The shift plan itself is an executable verified kernel covering every `i32`
precision. The `BigInt` shifts/addition, leaf dispatch, related-constant cache
identities and concurrent cache implementation still need refinement proofs.
The cache theorem assumes its input error bound; it does not certify arbitrary
cached values or the kernels producing them.

Fifteen theorems in [`real_approximation_model.rs`](real_approximation_model.rs)
extend the error model to the verifier's exact mathematical reals. Signed binary
units compose, and the integer precision plan gives a valid real approximation.
Negation, exact binary offsets, addition with two guard bits and cache
coarsening preserve the one-unit error bound. Signed integer scaling adds at
most half a unit. The asymmetric multiplication schedule preserves one-unit
accuracy with three guard bits, including its zero shortcut; cached integer
bit lengths provide the upper magnitude bounds that schedule needs. Its
operand approximations and magnitude bounds remain explicit premises. These
results do not assume that a cached integer and exact real share a binade.
When the sampled other operand lies strictly inside its upper binade bound,
the product error is strictly less than one unit. A two-integer approximation
gap then certifies strict ordering given strict input error bounds. Inclusive
one-unit bounds alone are insufficient for that comparison rule; the production
graph must still establish the stronger input property on every relevant path.
A quarter-unit approximation
followed by rounding returns either the floor or ceiling, as required by
`near_integer`; an approximation separated from zero by more than one unit
certifies its sign. These are compositional mathematical results, conditional
on each input approximation's error bound. They do not establish the production
graph's real denotation, its precision arithmetic, dependency operations,
transcendental error bounds, domain validity, abort semantics or convergence.

The float proofs stop at decomposition into a signed integer times a power of
two. The `BigUint` construction, reduction, retained-fact cache, and public
conversion wrappers are not yet verified. The scalar and fixed-buffer GCD
kernels are complete, but their `BigUint` import/construction boundary,
arbitrary-precision GCD tiers, rational normalization and other callers still
need formal refinement proofs. The fixed-buffer kernel requires between 2 and
`u32::MAX / 64` limbs; the production wrapper enforces this bound with a const
assertion and currently instantiates 4- and 8-limb buffers.
The Lehmer builder and row-coefficient selection are now verified, but the
leading-bit extraction and `BigUint` row multiplication/addition/subtraction
still need implementation proofs. The matrix theorem establishes the intended
GCD identity; it does not silently certify those dependency operations, the
wide GCD dispatch loop, or the recursive half-GCD tier.
Dyadic multiplication uses the same verified generic kernel for 2×2, 4×1,
4×2 and 4×4 limbs. The product accumulators also use verified alignment and
addition without allocating another full-width buffer. Their array lengths
must not exceed `u32::MAX / 64`; production uses at most six limbs. Signed
differences, normalization and the result-width checks now use verified
kernels too. Narrow and wide-to-narrow stack product sums now use verified
planning and signed-term loops, retaining both word/wide multiplication paths.
Planning includes inactive terms, as in the previous carrier implementation,
and checked exponent addition returns a fallback on overflow. The adapters
that assemble product inputs and convert signs/results remain unverified.
Native dyadic differences and two-product sums now use verified kernels too,
with native `u128` multiplication and shifts retained. The native and stack
product-sum paths share their input adapter. Other scalar-only and
`BigUint`-backed aggregate paths, `BigUint` materialization,
parameter comparisons and surrounding geometric algorithms also still need
implementation proofs. These kernels alone do not establish the full dyadic
or geometry API contracts.

The table uses a bounded-depth recursive traversal during const evaluation;
its values are computed by the same verified functions in both builds. The
128-bit trailing-zero helper and power-of-two predicate use ordinary shifts
and bit operations because upstream vstd does not yet specify those Rust
intrinsics. The limb subtractor similarly uses wrapping subtraction and
explicit borrow comparisons in place of unspecified overflowing intrinsics.
The accumulator similarly uses wrapping addition and explicit carry
comparisons. Its shifted target index stays in `u64` until the buffer check
proves conversion to `usize` is safe, including on narrower targets.
No project assumptions were added to cover these operations.

`test_rejections.py` copies the checked production sources and mathematical models
into temporary directories, first verifies the unmodified proofs, and then
requires rejection of incorrect implementations, false mathematical claims and
an attempted assumption bypass. The model guards reject reversed comparison
corrections, overlapping adjacent binary intervals, invalid approximation bounds
and false sign, reciprocal, square-root and zero certificates. This guards
against a proof job that silently stops checking executable behavior. Runtime
oracle tests compare canonical numerator and denominator values against
`num::BigRational`, so Hyperreal's own equality cannot hide an error. GCD tests
also cover every pair of trailing-zero positions across the 64-bit boundary,
all table entries, quotient-estimate boundaries and random full-width inputs.
Fixed-buffer tests compare with `BigUint` GCD across borrow chains, zero cases,
every trailing-zero position, whole-limb factor restoration and randomized
256-bit operands.
Lehmer tests check known batch/fallback results, Fibonacci chains approaching
the 62-bit limit, signed coefficient limits, determinant and row bounds, and
GCD preservation with discarded low limbs. Mutations also require rejection of
an always-`None` batch builder, corrupt coefficients and reversed row signs.
Product tests compare against `BigUint` across full carry chains, empty inputs,
asymmetric lengths, padded output buffers, and both scalar paths through the
wide dyadic accumulator. Further mutations omit carry, return a zero product,
misplace the high half of a word and multiply by the wrong operand.
Shifted accumulation tests cover every bit alignment across the tested buffer
widths, empty inputs and outputs, zero padding, `u64::MAX` shifts, exact sums,
and both failure-state cases against `BigUint`. Mutations reject lost carry,
incorrect zero/fallback results, broken digit extraction and ignored shifts
in both product wrappers.
Finishing tests compare signed numerators and canonical denominators directly
with `BigRational`, exercising every trailing-zero position across the stack,
borrow chains, cancellation, both signs and maximal exponent metadata. Width
conversion tests check every single-bit boundary, padding, truncation and
empty buffers. Mutations reject wrong signs, unreduced output exponents,
vacuous zero results and corrupt narrowed values.
Product-sum tests compare unsigned subtotal capacity decisions with `BigUint`
and canonical results with `BigRational`. They cover every stack alignment,
inactive terms, cancellation after an oversized subtotal, mixed factor widths,
empty inputs/buffers and maximal exponent metadata. Both production carrier
adapters are checked against independently constructed signed fractions.
Mutations reject incorrect scale plans, omitted or reversed signs, corrupt
products, swapped subtotals, always-fallback results and negative zero.
Native-word oracle tests check every trailing-bit position, shift boundaries,
raw product and signed-sum overflow, inactive terms, negative zero and maximal
metadata. They compare canonical numerators and denominators with `BigRational`
and independently compute the fast paths' representability decisions. Added
mutations reject truncated shifts, omitted normalization, incorrect zero/sign
classification, broken native multiplication, ignored scales and unconditional
fallbacks.

## Remaining work and completion criteria

1. Connect the rational model to the actual signed-magnitude representation,
   normalization, caches, every arithmetic dispatch path, and public error
   behavior. Prove the remaining GCD tiers, roots/powers, checked aggregates, dyadic
   carriers, Barrett division, Toom multiplication, and NTT/CRT operations.
2. Establish an implementation proof boundary for `num-bigint` and other
   behavior-bearing dependencies. A trusted assertion about BigUint arithmetic
   is not a completed proof of that implementation.
3. Define the denotation of every `Real`/`Computable` representation and prove
   exact symbolic rewrites, partial-function domains, approximation error,
   convergence where promised, and bounded refinement/uncertainty behavior.
4. Prove elementary, trigonometric, hyperbolic, probability, and special
   functions, together with the precision and resource limits they document.
5. Prove structural facts, sign/equality/order certificates, enclosures, exact
   linear algebra, and floating-point filters against those denotations.
6. Cover primitive imports and exports, parsing, formatting, the `simple`
   language and CLI, and JSON/CBOR behavior under every feature combination.
7. Verify cache validity, synchronization, unsafe memory and ownership,
   cancellation, and all advertised termination/resource contracts. Every
   unverified caller must discharge the contracts it relies on.
8. Audit all production sources and dependencies against the public API and
   documented guarantees in the crate README and source guides. Each remaining
   obligation needs direct verifier evidence; passing tests supplement it.
   The production crate itself must be covered before declaring 100%.

The trusted verification infrastructure currently includes Rust's compiler and
standard-library specifications, Verus and its macro erasure, vstd's imported
proofs/specifications, Z3, and the execution environment. Project proofs add no
`assume`, `admit`, `external_body`, or `assume_specification` escape hatches.
Unverified Hyperreal code and dependency implementations are outstanding proof
work, not silently counted as verified infrastructure.
