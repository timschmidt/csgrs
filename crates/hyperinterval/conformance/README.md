# Alumina differential client

`alumina_q30.rs` compares the shared kernel with the actual, unchanged
Alumina FOC implementation. It runs 20,736 endpoint-boundary combinations,
100,000 deterministic random cases and 81 extreme i64 ratios. Every result
endpoint or mapped typed error must agree. Independent mathematical enclosure
tests remain in Hyperinterval; agreement with a legacy implementation alone
is insufficient.

Run from the repository root:

```sh
python3 crates/hyperinterval/scripts/alumina_conformance.py \
  --firmware ../alumina-firmware --output /tmp/interval-alumina
```

This creates an isolated dev-only Cargo workspace under the selected output
directory. Firmware is not a production or normal dev dependency of the
published Hyperinterval package. The explicit output directory retains the
manifest, lockfile and log for evidence. By default the client uses Rust 1.88.

Add `--bench` to collect 30 paired alternating samples for five Q30 operations,
with exact output checks and separate warmup. The probe measures out-of-line
calls on four successful input cases, including construction and materialized
endpoints. It is not a full FOC, embedded-cycle or HIL benchmark. Raw samples
stay in the chosen output directory; do not compare a successful row with an
overflow/invalid row.
