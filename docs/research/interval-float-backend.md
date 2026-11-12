# Strict finite float backend admission design

This is the next implementation design for phase 4 of
[the interval plan](../INTERVAL_REFACTOR_PLAN.md). It is not an implemented
capability or qualification claim. The proposal wrappers already exist; no
`F32Enclosure` or `F64Enclosure` implementation is present at this checkpoint.
The full plan and its acceptance gates remain authoritative.

## Authority and ownership

Hyperinterval owns the finite scalar sets and their directed arithmetic.
Hyperlimit and Hypersolve continue to own decisions and certificates. A finite
IEEE value enters as its exact binary rational value. An enclosure's bounds
are exact IEEE endpoints, not approximate samples of another source.
No proposal converts into a certified carrier. A caller must replay the
original exact-source operation to validate a proposed computation.

The candidate backend uses integer operations on IEEE encodings. Native float
arithmetic is unnecessary for add, subtract, multiply or divide. This avoids
relying on ambient rounding, FTZ or DAZ for the certified path. Only bit transfer
through `to_bits`/`from_bits` crosses the primitive float interface. Compiler,
ABI and integer-operation semantics still require qualification; this is not
a promise about arbitrary GPU or custom targets.

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

Nonnegative integer powers can use at most 32 squarings and 32 products per
positive endpoint enclosure and apply parity afterward. Document accumulated outward
rounding and intermediate overflow. Exponent zero maps nonempty input to one;
empty remains empty. Do not claim a tight image without proving it. An initial
affine operation may compose directed multiply and add, explicitly documenting
separate rounding and intermediate representability. A fused affine kernel
requires its own width/rounding argument and should be measured before adoption.

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
