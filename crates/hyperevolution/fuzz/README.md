# Hyperevolution fuzzing

`evolution_invariants` crosses every public Hyperreal structural representation
against every other representation through GP evaluation, fitness ordering,
mutation, crossover, diversity, selection, archive, and replay carriers.

```sh
cargo check --manifest-path fuzz/Cargo.toml --bins
cargo +nightly fuzz run evolution_invariants --fuzz-dir fuzz -- -max_total_time=30
```
