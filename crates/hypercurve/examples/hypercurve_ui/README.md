# hypercurve UI Test Article

This is a small egui test article modeled after `cavalier_contours_ui`, but all
geometry operations are routed through `hypercurve`.

Run natively:

```text
cargo run --manifest-path examples/hypercurve_ui/Cargo.toml
```

Run for WebAssembly with Trunk:

```text
trunk serve examples/hypercurve_ui/index.html
```

If Trunk 0.21 rejects `NO_COLOR=1` in the local environment, run the same
command as `env -u NO_COLOR trunk serve examples/hypercurve_ui/index.html`.

The app keeps a Cavalier-style bulge vertex editor because that is convenient
for UI testing. The editor data is converted to `hypercurve` curve strings,
contours, and regions at the operation boundary.
