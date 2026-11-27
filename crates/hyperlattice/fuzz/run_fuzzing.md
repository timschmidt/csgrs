From `crates/hyperlattice`, use
`cargo +nightly fuzz run <fuzz_target> --fuzz-dir fuzz` with the desired libFuzzer
resource options. See [README.md](README.md) for targets, build/private-doc
commands, input domains, and oracle limitations. Reachable representation pairings
are not evidence that a fuzz campaign exercised every pair.
