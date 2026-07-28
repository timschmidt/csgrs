# Hypertri UI

This unpublished egui application is a visual test and debugging surface for
Hypertri's public triangulation families.

Its views cover:

- polygon triangulation for concave, holed, and adversarial inputs;
- Delaunay triangulation for point clouds, perturbed grids, and degenerate
  cases;
- constrained Delaunay triangulation for holes, crossings, and open PSLGs;
- runtime backend comparison over the same retained polygon.

Scene state can be encoded into a shareable URL. The editor and renderer use
finite display coordinates, while Hypertri owns triangulation predicates and
topology.

## Run

Native:

```sh
cargo run --manifest-path examples/hypertri_ui/Cargo.toml
```

WebAssembly with Trunk:

```sh
trunk serve examples/hypertri_ui/index.html
```

## Validation

```sh
cargo test --manifest-path examples/hypertri_ui/Cargo.toml
cargo clippy --manifest-path examples/hypertri_ui/Cargo.toml --all-targets -- -D warnings
trunk build examples/hypertri_ui/index.html --release
```

This package is `publish = false` and follows Hypertri's Apache-2.0 license.
