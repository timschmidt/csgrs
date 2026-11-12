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
