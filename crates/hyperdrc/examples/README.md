# hyperdrc Examples

This folder contains small user-facing examples for configuring `hyperdrc`.

## Contents

- [`hyperdrc-config.json`](hyperdrc-config.json) demonstrates rule thresholds
  and KiCad copper-layer selection that can be loaded with `--config`.

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
