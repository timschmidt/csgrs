# Hyperlimit fuzzing

`predicate_invariants` exercises the public 2D/3D predicate families, scalar
and batch agreement, retained evidence, and metamorphic laws.
`hyperreal_representations` crosses all eight public Hyperreal structural kinds
against each other through orientation, sidedness, sign, and ordering.

```sh
cargo check --manifest-path fuzz/Cargo.toml --bins
cargo +nightly fuzz run hyperreal_representations --fuzz-dir fuzz -- -max_total_time=30
```
