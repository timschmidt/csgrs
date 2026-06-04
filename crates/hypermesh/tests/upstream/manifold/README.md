# Imported Manifold Test Suite

This directory contains the upstream Manifold test suite and relevant test
artifacts imported from:

- Repository: <https://github.com/elalish/manifold>
- Commit: `beb681b08048b56ec282e9097d74547d1bff53ee`
- Commit date: 2026-05-24
- License: Apache License 2.0, copied in `LICENSE-APACHE-2.0`

The imported C++ sources and fixture files are intentionally kept in their
upstream layout under `test/` with the `samples/` support sources required by
Manifold's sample-backed tests. Hypermesh does not compile these C++ gtest
files directly. Instead, `tests/manifold_upstream.rs` validates the imported
suite inventory, source-to-fixture references, OBJ fixture parseability, and
polygon corpus structure so the upstream corpus is a testable artifact
boundary inside the Rust crate.
