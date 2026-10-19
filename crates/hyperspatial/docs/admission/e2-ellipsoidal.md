# Ellipsoidal Mercator admission

Target revision before implementation: `5fb8fc570`. The owning layer is
Hyperspatial's coordinate composition, with the refinable inverse equation
owned by Hyperreal. P1, P6, P8–P11 and P17–P20 apply. This is an API extension,
not a performance-only change.

The missing acceptance case is a prepared `EllipsoidalMercator` that maps a
geodetic latitude on an oblate, spherical or prolate surface to its isometric
latitude and back, using `Real` throughout. The first test supplies independent
analytic northings with latitude atan(3/4), sine 3/5, cosine 4/5. Fresh projected
ingress must force the inverse equation rather than replay an original point.
The inverse must remain consumable by ECEF and local-frame operations.

For positive axes a,b and scale k, retain x=E0+ak(λ−λ0), y=N0+akψ, z=h.
Longitude treatment is explicitly unrolled, rejected outside [-π,π), or wrapped
to that strip relative to λ0. Both poles are singular. Heights, terrestrial
realization, coordinate epoch, input qualification and model identity survive.
There is no datum, vertical, registry or tile-clipping operation.

With β=b/a, the forward isometric latitude is asinh(tan φ) minus
e atanh(e sin φ) for e²=1−β²>0, unchanged for β=1, and plus
c atan(c sin φ) for c²=β²−1>0. The inverse is the exact represented
`Real::inverse_isometric_latitude` equation, not a rounded Newton iterate.
See the [native scalar admission](../../../hyperreal/docs/admission-inverse-isometric-latitude.md)
for its proof, domain and resource limitations. The oblate formula and parameter
conventions agree with the [PROJ Mercator definition](https://proj.org/en/stable/operations/projections/merc.html)
(checked 2026-10-18). External projection implementations remain dev-only oracles.

The inverse's construction proves −π/2<φ<π/2. Retain that proof and its defining
(ψ,β) relation privately in `Latitude`; arbitrary latitude ingress still needs
the existing predicate validation. A cloned latitude retains this evidence.
Reprojection may reuse ψ only when the represented β has an exact structural
match. A matching relation does not erase approximate ancestry or input
qualification. Raw scalar extraction and reconstruction deliberately relinquish
that relation; no general scalar range oracle is promised here.

Prepared identities prevent unprojecting with a different definition. Shared
Mercator orchestration must enforce the same longitude, reference, surface,
source and batch rules for both families. Existing spherical tests are retained.
The spherical inverse formula is not being replaced by this extension.

`Context` policy remains explicit. Unknown predicates, invalid input, scalar
domain errors, singular poles, mismatches, cancellation and orchestration
exhaustion stay typed. Batch publication is atomic. Checkpoints, batch length
and receipt depth do not bound individual scalar time, heap bytes or expression
growth; external containment remains necessary for adversarial testing.

Acceptance evidence will include analytic and dev-oracle checks, false origins,
longitude seams, shape/reference mismatches, approximate ancestry, independent
fresh ingress, huge finite northings with retained range evidence, repeated and
downstream composition, the native representation corpus, sanitizer fuzzing,
coverage, source-consumer compilation and affected build/lint configurations.
No serialization or binding API is added. The full development plan remains
open beyond this projection family.

The public addition is `EllipsoidalMercator` and its
`GeodeticSource::EllipsoidalMercator` variant. The unpublished enum's exhaustive
consumers must handle the new variant. Spherical and ellipsoidal families share
private orchestration, while their prepared identities and equations remain
distinct. Certified forward-source replay now retains the original `Latitude`
evidence instead of extracting and revalidating its scalar. Receipt operation
names reflect that retained evidence; this is not an artifact-preserving
performance-only refactor.

## Execution evidence

The [evidence directory](../../verification/2026-10-18-e2-ellipsoidal/)
retains the missing-API compiler failure, analytic/domain tests, representation
and oracle tests, command logs, production coverage, directed traces, sanitizer
reports and measured benchmark output. `source_revision` in development reports
identifies the base checkout; `source_sha256` identifies the measured working
tree. No runtime dependency was added.

Eight projection tests cover positive/negative analytic oblate, sphere and
prolate inverses; fresh ingress and independent forward evaluation; false
origins; all longitude policies and exact seam offsets; pole singularities;
shape, frame/epoch and prepared-definition mismatches; approximate ancestry;
atomic batches; cancellation and recovery; receipt/work limits; repeated use;
and ECEF/local consumption. They retain the original spherical tests as well.

The extreme-northing test uses ψ=±10^320 and ±10^-80 on axis ratios 3/5, 1 and
7/5. It refines each latitude to a certified rational interval at binary
precision -160 and projects a cloned latitude using a new prepared scale and
false origin. The retained relation avoids numerically re-proving the polar
domain or evaluating an enormous tangent. A separate test changes the target
axis ratio and verifies the appropriate different isometric latitude. This
does not promise completion of arbitrary downstream predicates near a pole.

The new representation test exercises all 484 ordered latitude/scale recipe
pairs from the shared 22-family scalar inventory. The dev-only `proj4rs 0.2.1`
oracle grid supplies 63 WGS84 inputs, comparing forward coordinates within
3×10^-7 metres and fresh native inverses within 3×10^-14 radians. Exact analytic
fixtures provide the stronger, independent mathematical checks.

The benchmark retains longitude 2°, latitude 49°, height 100 m with identical
binary64 angle ingress to both implementations. Native evaluation keeps its
strict 3×10^-14-radian assertion and exact represented model. The comparator's
inverse misses that criterion: residual 2.3314683517128287×10^-13 radians.
The initial assertion failure is retained in
[`oracle-benchmark-bound-failure.log`](../../verification/2026-10-18-e2-ellipsoidal/oracle-benchmark-bound-failure.log).
The final report records zero accepted comparator completions at that bound;
it does not loosen the native criterion or claim the comparator passed it.
This is consistent with the pinned oracle's `math/phi2.rs` stopping its
iteration when the last increment is at most `EPS_10` (10^-10 radians), which
does not establish the native bound. The 63-point grid's passing inputs are
not a universal accuracy claim about that oracle.

Five release trials of 20 native operations took 32.75–33.32 ms per trial;
five trials of 10,000 oracle operations took 5.39–5.70 ms, with the above
accuracy failure in every oracle sample. The operations have different
certainty and receipt contracts, and the native inverse consumes exact
forward coordinates while the oracle consumes its rounded coordinates.
These are cost observations, not an exact-contract speed ranking or an
optimization claim. Other validation ran concurrently. A fresh process
(preparation, warmup and one measured operation per implementation) took
5.35 ms wall time, reached 16,328 KiB peak RSS, and used a 2,772,792-byte
executable. The [manifest](../../verification/2026-10-18-e2-ellipsoidal/benchmark-manifest.json)
records compiler, inputs, source/executable digests and measurement scope.

Production coverage is 1,303/1,323 lines (98.49%), 1,728/1,840 regions
(93.91%) and 158/158 functions. The new ellipsoidal module has 120/120 lines;
shared Mercator orchestration has 169/173. The separately measured native
inverse remains 136/140 lines (97.14%). The existing 95% gates are unchanged.
Branch coverage, 100% whole-crate coverage and formal proof remain open.

Final directed runs completed 176/176 cases per projection family across
22 independent buckets, with 44 selections and 39 distinct selected inputs
per run. Ellipsoidal cases used 1,296–59,436 traced dispatches, 51–67
orchestration checkpoints and receipt depth 8. The script's warming metadata
was corrected to identify projection byte 5 bit 2; the initial metadata had
incorrectly described bit 1. This changes the report description, not the
workload. Both initial and final traces are retained. ASan consumes the
initial selected corpus, whose actual bytes are recorded in its report.

The ASan runs used nightly 1.101.0, cargo-fuzz 0.13.2, seed 20261018,
six-byte inputs, a 10-second per-input timeout and a 2048 MiB RSS limit.
Both progressed beyond corpus initialization into coverage-increasing mutations.
Neither produced a crash, timeout, sanitizer diagnostic or new failure artifact.

| ASan workload | Seed files | Executions / elapsed | Counters / features | Peak RSS |
| --- | --- | --- | --- | --- |
| Ellipsoidal Mercator | 84 | 3,972 / 902 s | 11,316 / 39,032 | 747 MiB |
| Shared spherical Mercator | 80 | 2,214 / 61 s | 9,840 / 29,167 | 618 MiB |

The final ellipsoidal live corpus had 980 entries; its output directory had
989 files. The spherical counts were 651 and 662. Runtime corpus accounting
and retained directory files are distinct; reports preserve the actual bytes.
All final directed and ASan source digests match the retained implementation.
The two 900-second inverse campaigns plus the existing targets are accommodated
by a 65-minute CI job; no seeds, timeouts, policies or validation bounds were
reduced. Existing 60-second target budgets remain intact.

| Executed validation | Result |
| --- | --- |
| Complete Hyperspatial check script, stable Rust 1.99 and MSRV 1.95 | Passed on both; 116 test executions each, plus all-target checks/benches, strict Clippy, docs, independent README consumer and fuzz-manifest checks |
| Four native cache configurations | Passed: 228 integration-test executions and all-target strict Clippy |
| All release benchmarks, stable and MSRV | Native assertions passed; the comparator's zero accepted accuracy samples remain reported |
| Native WASM library, all features, Clippy `-D warnings` | Passed; compilation/lint only |
| Coverage and final workspace/fuzz formatting | Passed; 95% production-line floors unchanged |
| `zizmor --pedantic --strict-collection .` | Passed with offline audits |
| Changed shell/Python drivers and local documentation links | Syntax/links checked; directed/fuzz drivers also executed |

The [command ledger](../../verification/2026-10-18-e2-ellipsoidal/commands.json)
records actual commands and exit codes, including the initial expected missing
API failure and the retained comparator accuracy failure. No push or full
repository pre-push matrix is claimed. Context counters still do not bound
scalar work, heap bytes or elapsed time. The package remains unpublished;
WASM compilation does not establish a JS binding, and hosted artifact upload
is not tested here. Transverse Mercator/UTM, geodesics, E3–E4 and A1–A5 remain
active plan work.
