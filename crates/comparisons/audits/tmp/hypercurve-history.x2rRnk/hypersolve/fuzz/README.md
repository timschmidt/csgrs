# Hypersolve fuzzing

The suite covers algebraic transforms, exact linear algebra, curve resultants
and substitutions, active-set regeneration, failed constraints, and the
public sketch constraint families. `hyperreal_representations` crosses all
eight public Hyperreal structural kinds against each other through symbolic
facts, evaluation, variables, residuals, and elementary expression nodes.

```sh
cargo check --manifest-path fuzz/Cargo.toml --bins
cargo +nightly fuzz run hyperreal_representations --fuzz-dir fuzz -- -max_total_time=30
```
