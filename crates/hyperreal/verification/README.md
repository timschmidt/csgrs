# Verus verification of Hyperreal

The goal is a Verus proof of **all Hyperreal behavior**. It is not complete.
The current proof target verifies forty-two production contracts (forty-one
functions and one constant) and forty-four arithmetic model theorems. The rest
of the crate is still awaiting implementation refinement proofs. A passing
Verus job must not be described as 100% verification of Hyperreal.

## Run the proofs

On Linux x86_64, with Python 3.11+, curl, unzip support in Python, and rustup:

```sh
python3 scripts/verify_verus.py install
python3 scripts/verify_verus.py verify
python3 verification/test_rejections.py
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
objective's obligation groups remain open. [`scope.json`](scope.json) records
those obligations; it is a work ledger, not itself evidence that any requirement
is proved. There is no percentage derived from test counts, source hashes, or
the number of lemmas. Completing an obligation also requires auditing the
actual source, contracts, caller preconditions, and verifier results.

## Current executable proof boundary

The library and proof target import **the same** files in
[`src/verified`](../src/verified). Functions retain ordinary Rust bodies;
Verus's `verus_spec` attributes attach their contracts. Normal Cargo builds
erase only the proof annotations and `proof!` blocks. There is no copied model
implementation standing in for a production body, and no new runtime
dependency on Verus. The annotation mechanism is described in the upstream
[attribute syntax guide](https://verus-lang.github.io/verus/guide/exec_attr.html).

| Executable kernel | Checked behavior | Production caller |
| --- | --- | --- |
| `decode_f32` | All 32-bit patterns decode to the IEEE sign, significand and binary exponent, or the correct nonfinite class; field bounds are proved. | `TryFrom<f32> for Rational` |
| `decode_f64` | The corresponding result for all 64-bit patterns, including subnormals and both zero signs. | `TryFrom<f64> for Rational` |
| `compare_products` | A returned ordering matches exact unbounded cross products; `None` occurs exactly when a product exceeds `u128`. | Rational word-magnitude comparison |
| `signed_add` | Exact signed sum, canonical zero sign, and overflow iff the mathematical magnitude exceeds `u128`. | Rational signed-word aggregates |
| `checked_shift_left` | Exact multiplication by mathematical `2^shift`, or `None` for an unrepresentable factor or product. | Exact dyadic word scaling |
| `gcd_remainder` | Exact Euclidean remainder without overflow/underflow for `left >= right > u64::MAX`, including the high-limb quotient estimate. | `reduce_gcd_to_word` |
| `reduce_gcd_to_word` | For every pair of `u128`s, returns an ordered pair with a word-sized second operand and the same mathematical GCD; loop termination and remainder-call preconditions are proved. | Rational two-limb GCD |
| `small_gcd`, `fill_small_gcd_table`, `build_small_gcd_table`, `SMALL_GCD_TABLE` | All 4,096 compile-time table entries equal mathematical GCD; recursive initialization terminates and preserves entries outside the filled range. | `gcd_u64` |
| `binary_gcd` | Exact binary GCD for nonzero words, including removal and restoration of powers of two, subtraction, swap, overflow safety and termination. | `gcd_u64` |
| `gcd_u64` | Exact GCD for every pair of 64-bit inputs, through zero, table and binary paths. | Rational normalization and exponent GCD |
| `trailing_zeros_u128` | Exact trailing-zero count using the two 64-bit halves; the mathematical factorization theorem proves a positive odd quotient for nonzero inputs. | `gcd_u128`, `gcd_small_wide` |
| `gcd_small_wide` | Exact GCD for a nonzero word and two-limb value, including the power-of-two shortcut. | `gcd_u128` |
| `gcd_u128` | Exact GCD for every pair of 128-bit inputs; all dispatch paths, helper preconditions and final common-factor shifts are proved. | `Rational::gcd_word` |
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
[`rational_model.rs`](rational_model.rs) defines signed, unbounded
fractions with positive denominators, without assuming canonical reduction.
Its seventeen theorems establish equivalence and ordering laws, rescaling,
arithmetic closure, ring identities, reciprocal correctness, and respect for
equivalent representations. These model theorems do not yet establish that
all `Rational` methods implement the model.

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
kernels too. The loops that plan scales and dispatch signed terms, the sign
type adapters, `BigUint` materialization, parameter comparisons and surrounding
geometric algorithms still need implementation proofs. These kernels alone do
not establish the full dyadic or geometry API contracts.

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

`test_rejections.py` copies the production kernels into a temporary directory,
first verifies the unmodified bodies, and then requires rejection of forty-two
incorrect implementations and an attempted assumption bypass. This guards
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
