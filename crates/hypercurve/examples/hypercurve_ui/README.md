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

The root `.github/workflows/deploy-ui-pages.yml` workflow builds this app with
Trunk on pushes to `main` and publishes the resulting `dist` directory through
GitHub Pages Actions. Set the repository's Pages source to GitHub Actions before
the first deployment.

The app keeps a Cavalier-style bulge vertex editor because that is convenient
for UI testing. The editor data is converted to `hypercurve` curve strings,
contours, and regions at the operation boundary.

The editor and plotting layers store coordinates as `f64` because egui, Geo,
and rendering APIs require primitive floats. Core `hypercurve` operations lift
finite UI coordinates into hyperreal-backed Real values before topology decisions;
Geo boolean output is used only as an interactive display fallback when the
exact curve kernel reports an unresolved case.
