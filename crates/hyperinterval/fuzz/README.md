# Interval campaigns

The initial target exercises checked fixed/integer constructors, set relations,
signs and adversarial JSON ingress. It does not qualify arithmetic that has
not been implemented. `fixed_arithmetic` independently checks rational
cross-products, denominator-domain handling and exact overflow admission.

```sh
mkdir -p /tmp/hyperinterval-corpus
cargo +nightly fuzz run constructors /tmp/hyperinterval-corpus -- -runs=10000 -max_len=1024 -max_total_time=30
mkdir -p /tmp/hyperinterval-arithmetic
cargo +nightly fuzz run fixed_arithmetic /tmp/hyperinterval-arithmetic -- -runs=100000 -max_len=16 -max_total_time=30
```

Run from this crate's directory. Cargo-fuzz enables ASan by default. Preserve
the seed, toolchain, raw log and minimized failures outside the source tree;
promote each failure to a named deterministic regression before closing it.

`conversions` exercises rational/dyadic conversion with full-width signed
numerators and nonzero unsigned denominators, ordinary grids from -128 to 128,
and explicit refusal at extreme grids. Independent GMP directed bounds check
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
