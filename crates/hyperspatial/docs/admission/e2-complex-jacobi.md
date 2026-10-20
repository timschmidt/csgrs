# Complex Jacobi prerequisites and spatial consumption

Baseline `5261beba6`, branch `hyperspatial`. Owners are Hyperlattice for
complex continuation, Hyperreal for scalar relations and range reduction,
and Hyperspatial development tooling for downstream acceptance. Properties
P1–P7, P17–P20 and P35–P39 apply. The missing boundary is a native complex
Jacobi API whose norm identities remain useful to subsequent strict geometry.
There is no new production Hyperspatial coordinate operation in this increment.

Broader native validation reproduces a downstream inventory failure:
Hyperlimit's private-node predicate matrix still contains 60 recipes while
Hyperreal exposes 70 nodes. Hyperlimit development tests own this P18/P35/P38
gap. Preserve the complete original inventory and add valid inverse-isometric,
Carlson, Legendre, inverse-Legendre and Jacobi recipes, including serialized
value replay and the existing strict translation predicates. Run this guard
in native CI and all four primitive-cache configurations so future scalar
extensions cannot silently omit their predicate consumers.

The broader all-target Clippy check also rejects the existing Hyperlattice
and Hyperlimit allocation examples under Rust 1.99: `AtomicUsize::fetch_update`
is deprecated, while its renamed API is newer than the declared MSRV. The
development allocation counters own this compatibility issue (P36/P38).
Use the equivalent relaxed compare-exchange retry loop, preserving saturating
subtraction and the existing measurement boundaries. Compare the complete
one-iteration output with the pre-edit examples and check both compilers;
do not suppress the warning or change the measurement contract.

The [complex contract](../../../hyperlattice/docs/admission-complex-jacobi.md)
defines finite z and admitted real m < 1, positive/negative/zero parameter
branches, quarter-period continuation and distinct invalid/domain/pole errors.
The [scalar contract](../../../hyperreal/docs/admission-jacobi-rational-relations.md)
defines optional exact proofs, inverse guards, work limits and retained caches.
Production expressions remain native `Real`; MPFR is development-only.

## Consumer matrix

The shared `dev/complex_jacobi.rs` workload constructs x, y and m from the
22 scalar certificate recipes and checks their actual public kinds. For each
finite triplet it refines all six components, requires exact rational proofs
of sn²+cn²=1, dn²+m*sn²=1 and dn²−m*cn²=1−m, then consumes the first norm
as ellipsoid axis, geodetic height and latitude. Strict equatorial ECEF must
have x=4 exactly, and its receipt must retain the input's qualified accuracy.

Keep all 484 ordered pairs at each of positive and negative m, and all 176 family/regime/display
cases, including negative m, zero m, quarter-period translations and parity.
Retain the minimized logarithm stack-overflow pair `[7,19,2,2,0,0]` alongside
the complete matrix. The scalar owner also retains minimized quadratic
inverse, ellipsoid-axis composition, exact-zero trig and logarithm-planning
regressions. No difficult pair or exact-success assertion is removed.
Keep the sanitizer timeout `[21,6,127,39,1,4]`, its equivalent reduced encoding
`[21,6,0,0,1,4]`, and slow unit `[12,12,66,201,7,1]` as deterministic and fuzz
inputs. The [Carlson enclosure admission](../../../hyperreal/docs/admission-carlson-mean-enclosure.md)
records the scalar bound introduced after that timeout.
Retain directed negative-quadratic input `[0,120,113,108,1,0]` and its reduced
encoding `[0,10,0,0,1,0]` in both deterministic and sanitizer fixtures.

All four primitive-cache configurations execute this full matrix and the
public Hyperlattice complex tests. Stable and MSRV native checks keep their
full unoptimized spatial tests. Serialized scalar components independently
replay their values and domain evidence before the exact relations are queried;
the wire format does not retain evaluation/proof caches. This does not add
Hyperspatial serialization or certify external observational accuracy.

## Independent oracle and validation gates

The local MPFR oracle integrates s'=c*d, c'=−s*d and d'=−m*s*c from (0,1,1)
using outward interval coefficients and complex Horner evaluation. For
A=max(1,|m|), the convolution recurrence bounds coefficient n by A^n.
At q=A*(|x|+|y|) <= 3/4, a 512-degree polynomial has an absolute complex
tail at most q^513/(1−q). Each component receives that tail bound. The
oracle shares no rectangular, inverse-amplitude or elliptic-integral algorithm
with production. Tests require overlapping native/reference enclosures, each
narrower than 2^-200; benchmark rows require width below 2^-96. These are
comparison contracts, not native equality tolerances or physical error claims.

Coverage adds separate 95% production line floors for the complex API and
the five scalar relation-helper files. All previous denominators and floors
remain independent. Shared polynomial arithmetic, generic trig/logarithm
construction and structural matching are outside these two new subsets;
their affected regressions and native suites still run. Stable branch coverage
and the 100% target remain open.
The complete instrumented corpus uses test optimization level one with debug
assertions and overflow checks, matching the cache matrix. Separate stable and
MSRV checks retain the full unoptimized debug corpus. This changes execution
cost, not the fixture domain, assertions, numerical widths or coverage floors.

The directed selector adds a `complex_jacobi` workload in 22 independent
family buckets, retaining failures and costly valid cases. ASan uses the same
six-byte workload with all 176 original seeds, the retained regressions, and
selected cases. Its 1800-second
campaign budget, 10-second input timeout and 2048 MiB RSS ceiling are external
containment, not native resource guarantees. Raw logs must demonstrate mutation
after seed initialization before a mutation campaign is credited.

The new benchmark materializes all six component enclosures and checks every
row after timing. Cold rows reconstruct output expressions; inputs and scalar
global caches may be warm. Warm rows prime all six enclosures. Independent
MPFR rows have the stated finite-enclosure contract but a different evaluation
algorithm. Concurrent validation can affect timing; no speed ratio is claimed.

The [verification record](../../verification/2026-10-19-e2-complex-jacobi/README.md)
records final commands, hashes, completed test counts, all eight coverage gates,
directed selection and the completed sanitizer campaign. It preserves diagnostic
failures, the MSRV storage-recovery execution and the limits of these checks.
Thompson/Lee rectification, its certified inverse,
ellipsoidal Transverse Mercator/UTM and the remaining Earth/sky stages stay open.
