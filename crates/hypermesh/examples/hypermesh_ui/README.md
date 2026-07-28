# Hypermesh UI

This unpublished egui application is a visual test and debugging surface for
Hypermesh. It constructs two exact-coordinate cubes, runs the public certified
mesh Boolean API, and renders inputs and output through Hypergraphics.

The operation selector covers union, intersection, both directed differences,
and symmetric difference. The side panels expose mesh/display controls and
topology statistics so regressions can be inspected without introducing a
second geometry implementation.

## Run

Native:

```sh
cargo run --manifest-path examples/hypermesh_ui/Cargo.toml
```

WebAssembly with Trunk:

```sh
trunk serve examples/hypermesh_ui/index.html
```

The UI and GPU boundaries use finite display coordinates. Hypermesh remains
the authority for topology, predicates, and Boolean results.

## Validation

```sh
cargo test --manifest-path examples/hypermesh_ui/Cargo.toml
cargo clippy --manifest-path examples/hypermesh_ui/Cargo.toml --all-targets -- -D warnings
trunk build examples/hypermesh_ui/index.html --release
```

This package is `publish = false` and follows Hypermesh's Apache-2.0 license.
