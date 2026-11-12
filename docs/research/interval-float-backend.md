# Strict finite float backend admission design

This records the integer IEEE algorithm for phase 4 of
[the interval plan](../INTERVAL_REFACTOR_PLAN.md), implemented in Hyperinterval's
`float-enclosure` feature. The [progress record](../../INTERVAL_REFACTOR_PROGRESS.md)
separates measured qualification from the remaining plan scope. The full plan
and its acceptance gates remain authoritative.

## Authority and ownership

Hyperinterval owns the finite scalar sets and their directed arithmetic.
Hyperlimit and Hypersolve continue to own decisions and certificates. A finite
IEEE value enters as its exact binary rational value. An enclosure's bounds
are exact IEEE endpoints, not approximate samples of another source.
No proposal converts into a certified carrier. A caller must replay the
original exact-source operation to validate a proposed computation.

The backend uses integer operations on IEEE encodings. Native float
arithmetic is unnecessary for add, subtract, multiply or divide. This avoids
relying on ambient rounding, FTZ or DAZ for the certified path. Only bit transfer
through `to_bits`/`from_bits` crosses the primitive float interface. Compiler,
ABI and integer-operation semantics still require qualification; this is not
a promise about arbitrary GPU or custom targets. Public nonempty ingress is
gated to x86_64; ARM/WASM compilation does not qualify their execution.

Source-level integer logic alone was insufficient: optimized LLVM code replaced
an inlined magnitude-zero bit test with `ucomisd`, misclassifying subnormals
under DAZ. The shared `Format::finite` boundary is therefore `inline(never)`.
Release-mode isolated MXCSR tests and emitted-code inspection are required
alongside mathematical oracles. This also fixes proposal ingress, whose prior
normal-environment tests did not establish DAZ independence. The code scanner
covers emitted strict kernel/client functions and wide-division helpers, not
unrelated runtime or panic code; compiler qualification is not a universal
proof for arbitrary build flags or future optimizers.

Use the existing private IEEE validation/ordering helper and `Bounds` storage.
A private finite-bit wrapper with integer-based `Eq`/`Ord` can let `Bounds`
reuse its set-relation methods without native floating comparisons. The public
families remain concrete and do not expose a user-implementable proof trait.
The independent `float-enclosure` feature must add neither allocator nor
Hyperreal dependency, and must not silently enable proposal arithmetic.

## Finite encoding and directed conversion

For precision fraction width F (23 or 52), decode a finite nonzero value as
`(-1)^sign * m * 2^e`, where m has at most F+1 bits. Subnormals use the minimum
subnormal grid exponent, respectively -149 or -1074. Exact zero has canonical
positive-zero bits; both input zero signs denote the same real set point.
Reject NaN and infinity before order or arithmetic. Their encodings are not
sentinels for empty, exhaustion or unbounded output.

The point-rounding kernel takes a sign, a u128 magnitude m and an i32 exponent e.
For nonzero m, let `top = e + floor(log2(m))` and
`grid = max(minimum_subnormal_exponent, top - F)`. Integer floor/ceiling division
by `2^(grid-e)` produces adjacent outward grid coefficients; negative shifts
are exact left shifts with proved width bounds. A discarded nonzero remainder
increments the upper magnitude. Zero and shifts of at least 128 need explicit
branches rather than an overflowing machine shift. Encode each resulting
coefficient exactly, including a carry into the next binade. For negative
results, exchange the magnitude bounds and negate them.

Any required endpoint outside the finite IEEE range returns Overflow. Do not
saturate to the largest finite value and claim containment. Canonical empty
results remain valid. Extended unbounded intervals are a separate later API.

## Basic point arithmetic

Products have at most 106 magnitude bits and fit u128 exactly. Add source
exponents, multiply integer significands, combine signs, then round once.

For addition, normalize each nonzero operand's magnitude to F+1 bits, adjusting
its exponent exactly. After ordering exponents, an exponent gap at most 74 lets
the shifted magnitude and its sum/difference fit u128 (at most 127 shifted bits,
with room for addition). Use signed-magnitude integer arithmetic and round the
exact result. Handle zero operands before normalization.

For a gap greater than 74, the smaller operand is less than the spacing on
either side of the larger, exactly representable operand. The exact sum lies
strictly between that operand and its adjacent representable value in the
smaller operand's direction. Return those two adjacent values. This includes
binade boundaries; the smaller magnitude is also below the half-sized spacing
on the lower side of a power of two. Endpoint stepping uses integer encodings.
An outward step beyond the finite range is Overflow. Subtraction negates the
second source encoding and reuses addition.

For division, the ratio of nonzero input magnitudes has a binary order determined
by their bit-length difference and one exact shifted comparison. Add the source
exponent difference to get the quotient's top exponent, then choose its output
grid as above. Normal quotients need at most 106 bits in the shifted numerator.
A very small subnormal quotient can require an enormous shifted denominator;
if that denominator's bit length exceeds u128 while the numerator has at most
53 bits, the positive floor is zero and the ceiling is one. Otherwise use exact
u128 quotient and remainder. Negation exchanges the directed bounds.
Zero numerators are exact zero; zero denominators fail before arithmetic.

These bounds must be checked in implementation and exercised at every boundary.
They are an algorithm outline, not a substitute for implementation review or
independent tests.

## Interval surface and composition

Reuse named set relations, signs, hull/intersection and explicit empty state.
Nonfinite point queries must fail even for an empty interval. Validate a
zero-containing denominator before applying an empty-numerator shortcut.
Use monotone endpoint addition/subtraction and exact four-corner point bounds
for multiplication/division. Pointwise square has zero as its minimum when
the input contains zero; it must not be implemented as an independent-set
self-product across zero.

Nonnegative integer powers use at most 32 squaring and 32 product stages per
source endpoint, evaluating both directed bounds, and apply parity afterward.
Document accumulated outward
rounding and intermediate overflow. Exponent zero maps nonempty input to one;
empty remains empty. Do not claim a tight image without proving it.

Fused affine images compute the exact product (at most 106 magnitude bits)
before adding the offset. Both dyadic operands have at most 106 bits. A common
work grid places the dominant top bit at position 126, leaving one carry bit.
The dominant operand is exact on that grid; only the smaller operand can lose
a fractional tail. A lost tail requires top-bit separation of at least 21,
so opposite signs cannot cause severe cancellation. Consequently the result's
IEEE spacing is coarser than the work grid. The tail, whose magnitude is less
than one work-grid unit, changes the directed endpoints only when the retained
sum lies exactly on an IEEE endpoint; then step one endpoint in the tail's
signed direction. Otherwise its floor/ceiling are unchanged. Without a lost
tail, cancellation is computed exactly in u128. Intermediate product overflow
is not an error when the final image is finite: `MAX * 2 - MAX` returns MAX.
Apply this point kernel to both interval endpoints and take their hull.

Expose checked adjacent-value expansion. Never introduce NaN/infinity through
next-up/down at the finite extrema. Native replay should encode IEEE bits and
rebuild through validated constructors on a supported backend. No stored flag
or cache substitutes for validating the represented finite set.

## Required acceptance evidence

Before accepting the implementation:

- Record the owning property/domain and failing or missing witness before edits.
- Exhaust a reduced IEEE format against independent integer-rational expectations.
- Test f32/f64 normal, subnormal, zero, cancellation, binade and overflow edges,
  plus random bit pairs against exact GMP/Rug rational arithmetic and directed
  rounding. Check actual enclosure and domain outcomes, not checksums alone.
- Test finite interval images, mixed sign cases, empty/invalid precedence,
  output-as-input chains, outward expansion and validated bit replay.
- Verify rejection of proposal-to-enclosure conversion at compile time.
- On x86_64, use an isolated test process to exercise rounding modes and FTZ/DAZ.
  Prepare independent expected bit patterns before changing MXCSR and restore
  it afterward. The library itself must not read or modify ambient state.
- Inspect generated code for native floating arithmetic/comparison instructions
  in the strict kernel. Establish explicit target gates and fail closed on
  targets not covered by the backend contract; compilation alone is not runtime
  hardware qualification.
- Run MSRV, feature isolation, docs, strict lint, sanitizer and downstream checks.
  Preserve original failures and compare completed workload costs and sizes.
- Keep any absent arithmetic, unsupported target, unbounded output, overflow,
  or undecided result explicit. Do not mark phase 4 complete from this document.
