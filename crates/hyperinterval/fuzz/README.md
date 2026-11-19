# Interval campaigns

The initial target exercises checked fixed/integer constructors, set relations,
signs and adversarial JSON ingress. It does not qualify arithmetic that has
not been implemented. `fixed_arithmetic` checks the tight Q2.30 and Q31.32 arithmetic images with exact rational
cross-products. Each endpoint must enclose all extrema and moving it one lattice
unit inward must exclude an attainable value. Zero-containing divisors and
unrepresentable results require their respective typed errors. The first 16 bytes
retain the original four Q2.30 endpoints. With 32 bytes the same input also
supplies four full-width Q31.32 endpoints.

```sh
mkdir -p /tmp/hyperinterval-corpus
cargo +nightly fuzz run constructors /tmp/hyperinterval-corpus -- -runs=10000 -max_len=1024 -max_total_time=30
mkdir -p /tmp/hyperinterval-arithmetic
cargo +nightly fuzz run fixed_arithmetic /tmp/hyperinterval-arithmetic -- -runs=100000 -max_len=32 -max_total_time=30
```

Run from this crate's directory. Cargo-fuzz enables ASan by default. Preserve
the seed, toolchain, raw log and minimized failures outside the source tree;
promote each failure to a named deterministic regression before closing it.

`conversions` exercises rational/dyadic conversion with full-width signed
numerators and nonzero unsigned denominators, ordinary grids from -128 to 128,
and explicit refusal at extreme grids. It also checks fixed-point endpoints
at 30, 32 and 63 fractional bits against independent GMP fractions after
conversion to either host backend. Independent GMP directed bounds check
tightness and exact-grid admission. Generated empty, singleton and interval
results pass through serialization, exact rational replay and another grid;
exact work limits, exhausted attempts and fresh-budget retries are checked.
Arbitrary binary64 bits also exercise exact finite ingress, including subnormals,
and typed rejection of NaN/infinity before budget use. These finite campaigns do
not establish arbitrary coefficient-size coverage or total-resource bounds.

```sh
mkdir -p /tmp/hyperinterval-conversions
cargo +nightly fuzz run conversions --features rational /tmp/hyperinterval-conversions -- -runs=100000 -max_len=64 -max_total_time=30 -seed=20261125
```

`atanh` draws dyadic endpoints around −1, 0 and 1 with up to 60 fractional
bits. It checks both open domain limits, local coverage, empty-domain errors
and every finite endpoint against directed MPFR bounds. The deterministic
owner test also exercises intervals one grid step to either side of each pole.

```sh
mkdir -p /tmp/hyperinterval-atanh
cargo +nightly fuzz run atanh /tmp/hyperinterval-atanh -- -runs=10000 -max_len=8 -max_total_time=30 -seed=20261126
```

`integer_affine` checks signed and unsigned 128-bit affine hulls against GMP,
including cancellation after an overflowing product, range reversal and final
endpoint overflow.

```sh
mkdir -p /tmp/hyperinterval-integer-affine
cargo +nightly fuzz run integer_affine /tmp/hyperinterval-integer-affine -- -runs=100000 -max_len=64 -max_total_time=30 -seed=20261127
```

`strict_float` checks binary32 and binary64 products and quotients of two
finite intervals. It evaluates all four corners in GMP rationals and rounds
the extrema outward with MPFR. It compares both endpoint bit patterns and
checks typed overflow and zero-containing divisors. It also converts the hull
of the first two points to the other IEEE format, checks its directed endpoint
bits or overflow against MPFR, and converts successful results back to
check source containment. Four little-endian
64-bit words supply the endpoints. Binary32 uses each word's low 32 bits.
Existing 24-byte inputs retain their point, affine, dyadic and replay checks.
