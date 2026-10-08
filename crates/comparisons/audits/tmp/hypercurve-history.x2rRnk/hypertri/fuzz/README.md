# Hypertri Fuzz Targets

These harnesses are opt-in and excluded from normal builds. They are run with
`cargo-fuzz` from the `hypertri` repository root:

```sh
cargo install cargo-fuzz
cargo +nightly fuzz run topology_invariants
cargo +nightly fuzz run hyperreal_representations
```

The targets generate exact inputs and call public `hypertri` APIs. They do not
use `earcutr`; that crate remains a dev-only differential oracle in ordinary
integration tests.

`hyperreal_representations` executes all 22 optimized finite Hyperreal
certificate classes on every input, adds four variable-depth computable-opaque
DAG values, and rotates the ordered pairings selected by the byte stream. Each
value crosses earcut, ordinary and spatial Delaunay, crossing-constraint CDT,
D-dimensional Delaunay, and runtime polygon selection. The companion
integration suite also fails if the 22 private certificate tags or the eight
public structural kinds drift without a new fixture.

For a bounded sanitizer smoke run:

```sh
ASAN_OPTIONS=detect_leaks=0 cargo +nightly fuzz run hyperreal_representations \
  --fuzz-dir fuzz -- -runs=128
```

When a minimized input is found, convert it into the closest focused regression
test in `tests/adversarial.rs` or `tests/fuzz_properties.rs`.
