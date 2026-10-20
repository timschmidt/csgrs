# E1 native implementation admission

Baseline: `1bbddf42f`, branch `hyperspatial`. The baseline exposes no numerical
API. This extension is owned by Hyperspatial; scalar operations, predicates and
Cartesian carriers remain owned by Hyperreal, Hyperlimit and Hyperlattice.

## Contract before implementation

Relevant requirements: P1–P11, P18–P20, P26 and P35–P39. First missing acceptance
cases are exact angular ingress, invalid latitude/ellipsoid rejection,
geodetic/ECEF axis identities, ellipsoid-normal local bases, explicit anchor
and reference-frame mismatch, and inherited approximate decisions.

The forward domain is positive equatorial and polar radii (sphere, oblate or
prolate), latitude in [-pi/2, pi/2], explicitly wrapped or unrolled longitude,
and real ellipsoidal height. Height may be negative. Cartesian outputs use
Hyperlattice and retain source coordinates and construction evidence. A forward
map does not imply uniqueness of its inverse in the deep interior.

Local Cartesian operations retain a specific prepared anchor, convention,
reference-frame identity and coordinate epoch. Pole longitude explicitly
chooses the local meridian. Positions translate; free vectors only rotate.
An ellipsoid is a reference surface, not a datum transformation.

Resource accounting initially bounds Hyperspatial work checkpoints, batch
length and retained evidence depth. It does not bound arbitrary input Real
graphs, native scalar allocations, or elapsed time within one scalar/predicate
call. Cancellation is cooperative between checkpoints; retained scalar values
must not acquire a cancellation signal that can later change their meaning.
Contain adversarial processes externally until the deeper bounds are admitted.

Tests combine exact independently derived cases, representation-aware generated
cases and published PROJ conversion fixtures. Round trips supplement those
checks. Sources: [PROJ cartesian conversion](https://proj.org/en/stable/operations/conversions/cart.html),
[PROJ topocentric conversion](https://proj.org/en/stable/operations/conversions/topocentric.html),
and [NGA WGS84 definition](https://earth-info.nga.mil/index.php?dir=wgs84&action=wgs84).

## Implementation and validation

Retained: exact units and named longitude handling; policy-validated latitude
and positive spheroids; native forward conversion; ENU/NED preparation;
reference/epoch and anchor checks; point/vector distinctions; atomic forward
batches; immutable source and evidence graphs; explicit input accuracy;
cooperative orchestration limits. General ECEF inversion remains open. No
existing downstream crate depends on Hyperspatial yet.

The initial missing-API test was reproduced on the scaffold: `cargo test
--locked --offline -p hyperspatial --test units` failed with unresolved imports.
Adding native dependencies required fetching the already locked scalar and
development-test dependencies. No normal dependency is an external transform
implementation.

Validation on Rust/Cargo 1.99.0, x86_64-unknown-linux-gnu:

| Check | Result |
| --- | --- |
| `cargo test --locked --offline -p hyperspatial --all-features` | 20 integration tests and one README doctest pass. The generated invariant test executes 64 cases. |
| `cargo test --locked --offline -p hyperspatial --all-targets --all-features` | Forward/local integration suites and benchmark executable pass; the later three scalar-family/concurrency/qualification tests also pass in the all-feature run above. |
| `cargo clippy --locked --offline -p hyperspatial --all-targets --all-features -- -D warnings` | Pass after the final test additions. |
| `RUSTDOCFLAGS="-D warnings" cargo doc --locked --offline -p hyperspatial --no-deps --all-features` | Pass after correcting a README license link exposed by rustdoc. |
| `cargo fmt --all -- --check` | Pass for the workspace. |
| Separate `/tmp/hyperspatial-e1-consumer-6mzsp8cm` project | The README workflow builds and runs using an explicit path dependency, its own workspace and no workspace patches. This is local source-consumer validation, not a published distribution. |
| `cargo bench --locked --offline -p hyperspatial --bench forward_local` | Five trials of 100 preparations and 100 forward/local/display workflows complete with exact predicate certainty. |

Coverage includes analytical equator/pole/negative-height results on sphere,
oblate and prolate surfaces; PROJ's rounded GRS80 Cartesian/topocentric examples;
independently checked surface equations and orthogonal bases; all eight public
Hyperreal structural families in the height workflow; shared-source cache warming
on four threads; sticky approximate/input qualification; distinct reference
epochs and anchors; empty and exhausted atomic batches; and cancellation/recovery.
This does not establish exhaustive scalar dispatch or instrumented coverage.

The retained [cost rows](../../benchmarks/2026-10-18-forward-local.csv) are a
focused probe, not a speedup or competing-library claim. Preparation and
evaluation are separate; evaluation includes evidence retention, a return to
ECEF and explicit binary64 display materialization checked against the fixture.
The probe used the workspace optimized profile, Rust 1.99.0 and an Intel Xeon
E5-2680 v2 CPU. [Process resource observations](../../benchmarks/2026-10-18-smoke-resources.json)
record executable size and one fresh-process peak RSS; RSS is not an allocation
count or proof of a heap bound. `/usr/bin/time` was unavailable, so the resource
observation used Python's child-process `getrusage` after executing the probe.

Open: ECEF inversion and its branch evidence, stronger scalar work/memory
contracts, full representation/dispatch and instrumented coverage, directed
fuzz/ASan campaigns, MSRV/WASM/distribution checks, and the rest of the
development plan (E2–E4 and A1–A5). Dedicated testing infrastructure follows
this increment. No push or full repository pre-push campaign was performed.
