# hyperdrc Examples

This folder contains small user-facing examples for configuring `hyperdrc`.

## Contents

- [`hyperdrc-config.json`](hyperdrc-config.json) demonstrates rule thresholds,
  KiCad copper-layer selection, generated-output freshness, package-profile
  selection, assembly-profile thresholds for SMT, fixture, hand, soldering,
  press-fit, and coating processes, required production artifacts/layers,
  stackup process/material metadata, fabrication capability thresholds, and
  net-class constraints for width, clearance, current, voltage, reference-plane,
  approximate length/skew, differential-pair, and impedance-control
  target/tolerance intent that can be loaded with `--config`.

## Usage

Run `hyperdrc` with the example config:

```sh
cargo run -- \
  --config examples/hyperdrc-config.json \
  --kicad-pcb board.kicad_pcb
```

CLI flags override values from the JSON config. Keep example files compact and
reviewable so users can copy a small starting point into their own repositories.

Return to the [repository README](../README.md).
