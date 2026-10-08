# Third-party dependency boundary

The root comparison harness contains independently written tests, benchmarks,
analysis scripts, and recorded reports. It does **not** contain or redistribute
exactCorelib source code.

The `audits/` archive additionally preserves historical Hyper snapshots and
compatibility copies of other audited implementations. Those files retain
their original licenses and notices; the root MIT/Apache-2.0 license does not
relicense them. `audits/migration.json` identifies their original paths and
exact digests. `audits/dependencies.json` records the external donor checkouts,
commit pins, origins, and top-level license locations. Archived source trees
include their accompanying license and copyright files. The original exactCorelib
checkout and the active Hyper crates remain external dependencies.

The build consumes an external exactCorelib checkout through its public C++
headers and compiles the source files from that checkout in place. By default,
`build.rs` expects `../../../exactCorelib-main/trunk`; set `EXACTCORE_ROOT` to another
exactCorelib `trunk` directory when needed. exactCorelib is separately licensed
under the terms supplied by its authors. This repository's MIT/Apache-2.0
license does not relicense exactCorelib.

The Delaunay comparison uses an independently written exhaustive
empty-circumcircle implementation in `cpp/empty_circle_complex.h`. It relies on
exactCorelib only for public exact scalar types and predicates; no exactCorelib
demo source is included.

The active Hyper crates are sibling local path dependencies in the monorepo. GMP,
MPFR, Rust crates from `Cargo.lock`, and all other external dependencies remain
subject to their respective licenses.
