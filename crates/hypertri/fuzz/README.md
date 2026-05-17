# Hypertri Fuzz Targets

These harnesses are opt-in and excluded from normal builds. They are run with
`cargo-fuzz` from the `hypertri` repository root:

```sh
cargo install cargo-fuzz
cargo +nightly fuzz run topology_invariants
```

The targets generate exact rational inputs and call public `hypertri` APIs.
They do not use `earcutr`; that crate remains a dev-only differential oracle in
ordinary integration tests.

When a minimized input is found, convert it into the closest focused regression
test in `tests/adversarial.rs` or `tests/fuzz_properties.rs`.
