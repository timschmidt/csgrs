# Hyperparts fuzzing

The suite covers EDA authoring intake plus exact assertion and electrical
envelope carriers. `hyperreal_representations` crosses every pair of the eight
public Hyperreal structural kinds through scalar, interval, and overlap APIs.

```sh
cargo check --manifest-path fuzz/Cargo.toml --bins
cargo +nightly fuzz run hyperreal_representations --fuzz-dir fuzz -- -max_total_time=30
```
