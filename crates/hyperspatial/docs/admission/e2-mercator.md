# E2 spherical Mercator admission

Baseline: `ce66e164f`. Owner: Hyperspatial projection semantics, over native
Hyperreal functions and Hyperlimit predicates. Properties P1–P11, P18–P20,
P26 and P35–P39. Missing acceptance: no projection constructor or projected
coordinate can currently enter the geodetic/ECEF/local workflow.

Admit spherical Mercator and the WGS84 pseudo-Mercator parameterization first.
Use the source ellipsoid's equatorial radius `a`; the mathematical projection
is spherical even when the source latitude belongs to a nonspherical ellipsoid.
This is an exact represented projection model, not a physical accuracy or
ellipsoidal conformality claim. It performs no datum or height conversion.
Parameters and coordinates are Real-backed. Cartesian storage reuses native
Hyperlattice points. No external runtime projection dependency is permitted.

For positive scale `k`, false easting/northing `E0,N0`, central longitude `L0`,
and latitude `phi` strictly between the poles, define
`E = E0 + ak(L-L0)` and `N = N0 + ak asinh(tan(phi))`.
The inverse uses `phi = atan(sinh((N-N0)/(ak)))` and
`L = L0 + (E-E0)/(ak)`. These native stable forms are documented by
[PROJ Mercator](https://proj.org/en/stable/operations/projections/merc.html).
[PROJ Web Mercator](https://proj.org/en/stable/operations/projections/webmerc.html)
defines the pseudo-Mercator use of the source ellipsoid's equatorial radius and
provides the independent rounded example (longitude 2°, latitude 49°).

Longitude policy is mandatory: preserve unrolled turns, reject outside the
central-meridian-relative half-open strip `[-pi,pi)`, or explicitly wrap into
that strip. The latter is many-to-one; inverse longitude is canonical in the
chosen strip, while the original unrolled source remains inspectable. Projected
ingress under either strip policy rejects eastings outside the inverse domain.
Poles fail typed, without clipping. The usual Web tile latitude boundary is not
the mathematical singularity, and display clipping is a separate boundary.

Prepared projections retain source ellipsoid, frame/epoch, parameters and
validation evidence. Clones share an immutable identity; consuming a point
from a different prepared projection fails explicitly. Source frame/epoch and
ellipsoid compatibility use policy-bearing predicates. Height passes through
unchanged. Inverse results retain their projected source and must feed ECEF and
local transforms without losing qualification. Certified source identities may
avoid expression growth on replay; approximate evidence cannot justify them.

Reuse existing work checkpoints, evidence-depth and atomic batch limits.
Move the identical E1 atomic-batch orchestration into a private Context helper
and reuse it for projection batches, preserving its checkpoint sequence.
Cancellation and domain/refinement/resource failures publish no complete
partial result. These remain orchestration bounds, not scalar heap/time bounds.
Finite inputs may remain undecided under a bounded predicate budget; do not
silently replace that outcome with a pole, clipped coordinate or float result.

Required tests: independently derived equator and `sinh(ln 2)=3/4` examples;
published PROJ rounded coordinates; pole/seam endpoints and exact offsets;
explicit wrapping/unrolling; false origin and scale; below-surface heights;
frame, epoch, ellipsoid and prepared-identity mismatches; qualified evidence;
empty/failed atomic batches; small budgets and cancellation; repeated projected
→ geodetic → ECEF/local consumption; scalar representations and fresh inverse
evaluation so identity shortcuts cannot conceal paired mistakes. Extend the
existing development checks, coverage, probes and sanitizer corpus to the new
surface. Ellipsoidal Mercator, TM/UTM and geodesics remain later E2 increments.

Add pinned development-only `proj4rs` 0.2.1 comparisons on matched spherical
equations. Its [documented API](https://docs.rs/proj4rs/latest/proj4rs/) uses
`Proj::from_proj_string`, in-place `transform`, and geographic radians. Keep
binary64 fixture ingress/egress explicit and qualified; it supplies no native
predicate decision, frame evidence or exactness claim. Use identical sphere
definitions on both oracle sides to exclude datum changes from this comparison.

## Implementation and acceptance

The new public surface is `SphericalMercator`, `MercatorParameters`,
`LongitudePolicy` and `ProjectedPosition<P>`. Projection data reuse `Point3`
with easting/northing/unchanged ellipsoidal height in metres. Inverse geodetic
values retain `GeodeticSource::SphericalMercator`. Prepared identity, source
ellipsoid and frame/epoch mismatches have distinct errors; poles return
`ProjectionSingularity`. Both batch directions use the same private atomic
Context helper as the existing E1 batches, with the original checkpoint order.

The initial acceptance test failed to compile against `ce66e164f` because these
APIs were absent. The final corpus has **46 integration tests and one README
doctest**, including eight projection contract tests, all 484 ordered
scale/latitude recipe pairs, and 63 pinned `proj4rs` grid comparisons. All 22
recipes come from the scalar owner's shared inventory. The new matrix and
directed/fuzz workloads evaluate fresh inverse and forward expressions to keep
source-identity reuse from concealing a paired mistake.

The oracle consumes the same represented binary64 angular inputs and exact
radius as the native spherical equations. Forward display agreement is checked
within `3e-7` m; native inverse error against the original represented input is
certified below `3e-14` rad. Oracle inverse checks use the same angular bound.
The independently derived `sinh(ln 2)=3/4` fixture, downstream Cartesian
coordinates, and published PROJ example are separate checks. This comparison
does not establish physical datum equivalence. `proj4rs` 0.2.1 is pinned with
default features disabled and carries `MIT OR Apache-2.0` package metadata.
It is development-only; no external crate was added to the production graph.

The first large-longitude test incorrectly assumed `10^310` degrees would
overflow binary64 after conversion to radians. The converted value is about
`1.745e308`, within that range. The original case remains, with its correct
display expectation, alongside `10^320` degrees, whose easting is outside
binary64. Both preserve the exact longitude and height on inversion. No
previously accepted geometry or numerical bound was weakened.

Clippy reproduced a duplicate-module error when the combined trace probe loaded
the scalar recipe source through both workflows. Each binary/test now imports
the shared inventory once; both workflows consume that module. No warning
suppression was added.

## Executed validation

All checks below passed on x86_64 Linux. Raw outputs, coverage and campaign
records are retained in
[the verification directory](../../verification/2026-10-18-e2-mercator/).

| Check | Result |
| --- | --- |
| `bash crates/hyperspatial/scripts/checks.sh`, Rust 1.99 stable and Rust 1.95 | Formatting, default/all-feature tests and targets, README doctest, strict Clippy/rustdoc, separate README consumer, native dependency boundary and all three fuzz builds/lints pass. |
| `bash crates/hyperspatial/scripts/representation-checks.sh` | All 46 integration tests and all-target Clippy pass in all four scalar cache feature configurations. |
| Updated Hyperspatial command in `scripts/adversarial.sh` | All affected adversarial, inverse, representation, projection and oracle suites pass. Unchanged workspace suites were not rerun as a full repository campaign. |
| WASM library Clippy, all features, `-D warnings` | Passes for `wasm32-unknown-unknown`; no JS runtime/binding claim. |
| `python3 crates/hyperspatial/scripts/coverage.py` | 1,073/1,091 production lines (98.35%), 1,468/1,567 regions (93.68%), 126/126 functions. Stable branch coverage is unmeasured; 100% line/region targets remain open. |
| `cargo bench --locked -p hyperspatial --benches` | All E1 and projection probes complete every row with Exact certainty. |
| `zizmor --pedantic --strict-collection .`, version 1.31.1 | No findings in default offline audit mode; online-only audits were not executed. |

The projection probe includes forward construction, independently evaluated
inverse, ECEF/local composition, evidence checks and display materialization.
Five optimized trials of 100 workflows took about 101–102 ms each on an Intel
Xeon E5-2680 v2 @ 2.80 GHz. This is a native cost observation under concurrent
validation, not an optimization or cross-library performance result.
A separate fresh-process smoke recorded 16,620 KiB peak RSS and a 2,712,424-byte
executable; its [resource record](../../benchmarks/2026-10-18-mercator-resources.json)
retains the actual command, compiler and source/binary digests. These numbers
are not allocation counts or native heap/time bounds.

## Directed and sanitized campaigns

Both normal-offset and projection lexicase campaigns completed 176/176 strict
checked workflows, in 22 independent representation-family buckets with two
generations of four candidates. They selected 37 and 40 distinct replay cases,
respectively. The projection workload mixes latitude and scale recipes, all
three longitude policies, turns, false origins, positive scales, signed heights
and sphere/oblate/prolate source surfaces. Its observed native dispatch counts
ranged from 1,154 to 2,816 and work checkpoints from 56 to 72. Selection scores,
all raw paths, failure outcomes (none), tool flags and source digests are in the
two reports and compressed observation files. `source_revision` names the base
checkout; the recorded digests identify this increment's measured files.

Nightly Rust 1.101.0 (`8006091eb`), cargo-fuzz 0.13.2, ASan, seed `20261018`,
60-second campaign budgets, 10-second per-input timeouts, maximum six-byte
inputs and 2048 MiB RSS limits were used. All three targets completed with
coverage-increasing mutations and no new crash, timeout or sanitizer diagnostic:

| Target | Initial files | Executions / elapsed | Final counters / features | Reported peak RSS |
| --- | --- | --- | --- | --- |
| `forward_local` | 918 | 1,834 / 61 s | 8,209 / 24,749 | 627 MiB |
| `normal_offset` | 81 | 527 / 61 s | 9,414 / 24,054 | 688 MiB |
| `mercator` | 84 | 2,275 / 61 s | 9,763 / 29,294 | 622 MiB |

These counters cover the instrumented fuzz binaries, not production line or
branch coverage. RSS includes sanitizer overhead. The historical empty
LeakSanitizer sandbox artifact remains explicitly preexisting; there is no new
native failure to minimize. The ASan report retains exact before/after corpus
bytes, source digests, commands, tools and artifact inventories. CI runs both
selection workloads and all three targets, retaining evidence even on failure.
Hosted artifact upload has not been run locally. No push or publication occurred.

Open: ellipsoidal Mercator, Transverse Mercator/UTM, geodesics, explicit display
clipping and nonlinear geometry images; formal verification, infinite-domain
and 100% coverage, scalar memory/work bounds, release packaging and actual
binding consumers; datum/data operations and the astronomy stages. The full
development goal remains active.
