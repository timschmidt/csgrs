# Hyperspatial directed inputs

The `forward_local` target checks positive rational sphere/oblate/prolate
surfaces at exact special angles, closest-point inversion with certified
latitude/height/forward-residual enclosures, both local-axis conventions,
orthonormality, and repeated source/evidence-preserving composition. It is a separate,
unpublished development package; libFuzzer is not a production dependency.
The `normal_offset` target uses Hyperreal's 22 shared representation recipes
for positive axes and heights, checks the independently constructed normal foot,
then consumes the inverse in fresh forward evaluation and local vector operations.
The `mercator` target uses the same scalar inventory for independently
parameterized latitudes and longitudes, all three longitude policies, false
origins, scales, signed heights and oblate/prolate/spherical source ellipsoids.
It checks fresh inverse/forward arithmetic and downstream ECEF/local use.
The `inverse_isometric` target independently parameterizes the native scalar
inverse and consumes it in geodetic/ECEF/local operations. The
`ellipsoidal_mercator` target exercises the prepared ellipsoidal projection
with the same 22-family inventory, independent oblate/prolate/spherical
northings, fresh inverse arithmetic, retained relation replay, and local use.
The `transverse_mercator` target exercises the spherical transverse chart,
huge-coordinate inverses, pole/edge conventions and angular UTM selection.
The `elliptic` target checks the native RF/RD prerequisites with ordinary,
one-zero, extreme-scale and near-singular inputs, independent identities and
consumption of their outputs in ellipsoid/ECEF construction. It also checks
complete/incomplete Legendre F/E, unwrapped periods, odd symmetry, circular
endpoints, and separate spatial consumption of the Legendre outputs.
The `jacobi` target evaluates the three native components, consumes their
exact unit norm and positive dn in an ellipsoid/ECEF operation, and preserves
qualified input provenance. Its regimes include circular parameters, both
signs of m, quarter-period shifts and small positive complements.

From the repository root:

```sh
cargo check --locked --manifest-path crates/hyperspatial/fuzz/Cargo.toml --bins
python3 crates/hyperspatial/scripts/directed.py
python3 crates/hyperspatial/scripts/directed.py --workload mercator --output target/hyperspatial-directed-mercator
python3 crates/hyperspatial/scripts/directed.py --workload inverse_isometric --output target/hyperspatial-directed-isometric
python3 crates/hyperspatial/scripts/directed.py --workload ellipsoidal_mercator --output target/hyperspatial-directed-ellipsoidal
python3 crates/hyperspatial/scripts/directed.py --workload transverse_mercator --output target/hyperspatial-directed-transverse
python3 crates/hyperspatial/scripts/directed.py --workload elliptic --output target/hyperspatial-directed-elliptic
python3 crates/hyperspatial/scripts/directed.py --workload inverse_legendre --output target/hyperspatial-directed-inverse-legendre
python3 crates/hyperspatial/scripts/directed.py --workload jacobi --output target/hyperspatial-directed-jacobi
python3 crates/hyperspatial/scripts/fuzz-smoke.py --selected-corpus target/hyperspatial-directed/corpus --mercator-corpus target/hyperspatial-directed-mercator/corpus --isometric-corpus target/hyperspatial-directed-isometric/corpus --ellipsoidal-corpus target/hyperspatial-directed-ellipsoidal/corpus --transverse-corpus target/hyperspatial-directed-transverse/corpus --elliptic-corpus target/hyperspatial-directed-elliptic/corpus --inverse-legendre-corpus target/hyperspatial-directed-inverse-legendre/corpus --jacobi-corpus target/hyperspatial-directed-jacobi/corpus
```

Retain every crash/timeout and its minimized counterpart, then promote the
failure to a deterministic regression without weakening the original case.
The input generator is bounded; arbitrary native scalar allocations and time
inside a scalar call remain unbounded. The libFuzzer containment flags are
therefore part of the campaign command. Install nightly Rust and the pinned
development tool with `cargo install --locked cargo-fuzz --version 0.13.2`.
The smoke driver supplies 918 initial forward/local cases, 44 normal-offset
cases, 44 spherical projection cases, 44 inverse-scalar cases, 44 ellipsoidal
projection cases, 176 transverse cases, 176 elliptic cases, and any cases
selected by the directed driver. The inverse-Legendre target adds 176 cases
covering the same 22 families, four regimes, and cold/warm inputs. The Jacobi
target adds another 176 seeds with the same family/regime/warming layout. All nine
targets use ASan, a 10-second per-input timeout, and a 2048 MiB RSS limit.
The existing forward/local, normal-offset and spherical projection targets
have a 60-second libFuzzer budget. The inverse scalar, ellipsoidal projection,
transverse chart, elliptic prerequisites and inverse integrals each have 900
seconds because replaying their full seeds is more expensive. Jacobi has 1800
seconds: the measured 900-second campaign completed initialization without
reaching mutation, so its budget includes additional mutation time.
`--targets` permits focused reruns without changing any target's seed set. Initialization can consume or extend the time budget; the raw log
distinguishes corpus replay from mutation and any newly discovered paths.

The directed driver runs two generations of four candidates in each of 22
independent first-scalar-family buckets. Exact lexicase selection shuffles the
criteria (checked workflow time, dispatch count, work checkpoints), retains the
largest observed score at each step and breaks remaining ties randomly. The
second generation retains and mutates the selected parent without changing its
family. Failed/undecided/timed-out cases are retained and fail the campaign;
they cannot win as fast successes. These are cost-discovery samples, not a
statistical performance comparison. JSON reports and compressed raw dispatch
observations record all inputs, outcomes, scores, seeds, source digests and
tool configuration under `target/hyperspatial-directed` and
`target/hyperspatial-asan`. Override destinations with `--output`.

The [directed-testing admission](../docs/admission/e1-directed-testing.md) records
actual executed evidence and remaining limits. A build alone establishes
neither an ASan campaign nor coverage.
