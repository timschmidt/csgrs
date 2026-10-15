# Planar constructor failures and reusable empty results

This records local acceptance under `properties.md` and `docs/admission.md`.
The base is `f6ddbc74b` plus the original 35 pending native files. Those files
remain unchanged and await separate admission. This is worktree evidence;
clean-checkout, external-package, release and MSRV qualification remain open.
`9d1f2fb` preserves the baseline/failed trials and storage manifests;
`fd83df307` contains the implementation, callers, regressions and API migration
documentation.

## Contract and ownership

CSGRS owns parameter validation and composition of the existing native
Hypercurve constructors. Hypercurve owns contour and region validity. Scalar
adapters, Gerber import, Hypercircuit materialization/handoff and bindings must
retain the resulting diagnostic. Relevant properties are P1, P3–P4, P6, P8–P9,
P13, P17–P18, P20, P25, P28, P30, P35–P37 and P39; this slice does not establish
that any of those properties is satisfied throughout the stack.

This is an explicit Rust API migration. Rectangle, square, circle, ellipse,
right triangle, regular polygon, polygon/polygon_points and rounded rectangle
return `Result<CurveRegion2, ValidationError>`. Their context variants and
existing compound constructors retain `GeometryOutcome` with the same error
carrier. Native contour/region causes survive instead of becoming strings or
successful empty regions. C/C++/Python/Go signatures are unchanged; errors
reach their existing status/exception/error mechanisms. The affected WASM
constructors throw `GeometryError` with a machine-readable category.

| Constructor | Declared input domain |
| --- | --- |
| Rectangle, square, right triangle | Nonnegative dimensions; zero area is canonical empty after all dimensions are validated. |
| Circle, ellipse, regular polygon | Nonnegative dimensions/radius, at least three samples and representable primary buffer capacity, including for zero area. |
| Polygon and polygon_points | A valid native ring with at least three vertices. Explicit `empty()` represents absence; an empty or malformed ring is an error. |
| Rounded rectangle | Nonnegative dimensions/radius and checked corner-sample capacity. Existing radius clamp and sampling schedule remain; zero corner samples select a sharp rectangle. |
| Keyhole | Positive circle radius, handle width and handle height; at least three circle samples. |
| Reuleaux | Odd side count at least three, positive diameter, at least six circle samples and checked doubled side count. |
| Ring | Positive inner diameter and thickness; at least three samples. |
| Crescent | Positive inner radius, strictly larger outer radius, arbitrary exact offset and at least six samples. |
| Circle with keyway | Positive radius, key width and key depth; key dimensions less than the diameter; at least three samples. |
| Circle with flat(s) | Positive radius, distance in `[0,radius)`, at least three samples; two flats at distance zero yield canonical empty. |

Unresolved decisions remain typed failures under strict policy. Selecting
approximate policy alone does not taint an exact answer; consuming terminal
evidence does. These receipts concern the operation's decisions. They do not
establish end-to-end provenance, analytic fidelity or arbitrary downstream
topology. Sampled circles and ellipses represent polygons.

The shared ring cache stores only successful strict constructions, with owned
structurally exact coordinate keys and cloned native outputs. Approximate
requests may reuse those proofs but never populate this cache. Failures are
not cached. It is a per-thread 64-entry FIFO, cleared on thread destruction;
tests also clear it directly. There is no public disable/reset switch or byte
budget. Returned transformations cannot mutate retained geometry.

Hypercircuit rectangle materialization consumes the selected policy's
certainty and returns `GeometryMaterializationError::Construction` with its
cause. Invalid keepout/component rings produce the existing qualified handoff
with a typed `ProfileConstruction` omission. That omission is not a successful
empty envelope. The two diagnostic enums retain `PartialEq` and lose `Eq`
because their native causes do not implement it.

## Semantic evidence

The nine baseline regressions all failed before implementation: negative and
undecided dimensions became empty successes, zero dimensions hid invalid
companions/resolutions, malformed polygons and compound ranges passed as
empty, capacity overflow became empty, and wrappers flattened native causes.

The expanded tests distinguish invalid input, undecided decisions and checked
resource exhaustion through the public adapter. They feed canonical zero-area
results through union accumulation, subtraction to empty and extrusion. They
independently derive rectangle area `35/6`, triangle area `35/12`, unit sampled
circle area `2`, sampled ellipse area `4`, and height-three extrusion volumes.
Signed triangle area, signed volume and closed manifold topology are separate
assertions. Strict→approximate→strict sequences check evidence isolation.

Cache tests cover cold/warm success, failures, FIFO eviction, explicit test
reset, transformed output isolation and thread-local isolation. The primitive
resolution and capacity checks run before empty shortcuts. Hypercircuit tests
exercise both typed materialization causes and public keepout handoff recovery.
Binding consumers exercise errors, empty composition and subsequent recovery.

Existing strict shape/gear fixtures, README construction, adversarial inputs,
and sampling resolutions are retained. Call sites propagate errors or assert
the validity of fixed fixtures. Fuzz callers end an invalid generated request
explicitly instead of feeding an accidental empty success downstream.

## Trials

Executed runners and trials are immutable. Each retains commands, source
hashes/patch/archive, stdout/stderr, timing/resources and terminal result.

| Trial | Result |
| --- | --- |
| baseline-v1 | Nine of nine failure regressions failed on the target revision. |
| candidate-v1 | All 14 expanded regressions and 48 curve-only library tests passed. |
| candidate-v2 | Formatting passed; Clippy stopped at the temporary-filesystem quota before changed-crate diagnostics. |
| candidate-v3 | Formatting passed; Clippy exposed a missing `Point2` import in the new Hypercircuit tests. |
| native-v1 | All 19 native qualification commands passed. |
| bindings-v1 | All 39 commands passed, including real C/C++/Python/Go and generated Node/WASM/TypeScript consumers. |
| cost-v1 | Probe harness compilation failed on an unmatched JSON-output format brace, before any samples ran. |
| cost-v2 | Both builds and all 60 samples passed: 61,440 measured successful operations, plus warmup. |

Native qualification includes root/fuzz formatting, strict all-target and
all-feature Clippy for CSGRS, FFI, Hypercircuit and HyperDRC, CSGRS
library/integration tests, affected downstream tests, minimal/curve feature
configurations, fuzz-target Clippy, warning-free docs/doctests and the FFI
release build. The all-feature CSGRS suite has 253 passing tests and one
existing ignored rendering test. Hypercircuit has 24 passing targeted tests,
HyperDRC 35, FFI 10, minimal CSGRS 38 and curve-only CSGRS 70. Counts overlap
across feature configurations; no ignore was added. Four doctests passed.
Bindings cover six consumer fixtures each for C, C++ and Python, three Go
tests, strict WASM Clippy and the generated Node/TypeScript consumer.

## Focused cost probe

The same harness links the preserved baseline and candidate curve-only release
libraries. Each of five fresh-process samples performs 128 warmup operations,
then 1,024 measured constructions with independently checked exact polygon
area and output destruction. Warm cases repeat one ring; miss cases cycle 128
widths through the 64-entry FIFO. Other scalar/native caches may be warm. The
allocation wrapper counts Rust global allocations and contributes timing cost;
it does not account for every native allocation. Raw samples, library/input
hashes, flags, toolchain and hardware are retained in `cost-v2/`.

| Workload | Baseline median µs/op | Candidate median µs/op | Allocations/op, both medians |
| --- | ---: | ---: | ---: |
| Rectangle, warm | 1.277 | 1.549 | 2 |
| Rectangle, FIFO miss | 30.890 | 29.893 | 315.875 |
| Four-sample circle, warm | 6.671 | 5.957 | 92 |
| Four-sample circle, FIFO miss | 68.131 | 66.032 | 685.059 |
| Polygon, warm | 1.856 | 1.843 | 3 |
| Polygon, FIFO miss | 31.145 | 31.100 | 316.875 |

All measured operations completed under strict policy. Median requested bytes,
retained-byte deltas and additional peak live bytes also match for each pair.
Warm rectangle time increased about 21% (0.272 µs/op); that unfavorable result
is retained. The probe executable grew from 21,130,456 to 21,131,696 bytes, and
the CSGRS rlib from 4,394,348 to 4,422,696 bytes. These are measurements on a
shared host with no frequency control, not a broad performance qualification.
The change is retained to restore explicit failure and evidence handling, with
these observed costs; no performance-only improvement is claimed.

## Resource limits and preserved artifacts

Validation uses offline resolution, two Cargo jobs and serial commands, each
with 1800-second external wall containment and an 8-GiB per-process address
space limit. Node/WASM omits the address-space limit for virtual reservations;
wall containment remains. GNU time measures command-level resources including
compilation. Internal total work, scalar/coefficient growth, secondary native
allocation, retained heap bytes and cancellation remain unbounded or unmeasured.

The deletion manifest maps 2,107 non-executable duplicate compiler files
(1,682,918,575 bytes) to preserved identical originals. Subsequent consolidation
hard-links 1,205 temporary-filesystem paths (1,008,409,001 duplicate bytes) and
58,738 repository-filesystem paths (50,210,030,048 duplicate bytes). Every
hard-linked path and byte remains available. These closed targets are
immutable: new phases copy seeds into separate inodes and never resume Cargo
in a consolidated directory. Executed artifacts, unique compiler outputs,
source snapshots and logs are retained.

## Open work

Other infallible sampled shapes, gears, text constructors and legacy transforms
still need typed-failure migration. Complete provenance, analytic fidelity,
internal resource/cancellation bounds, directed fuzz/sanitizer campaigns,
full CI/pre-push coverage, regenerated images and shipped external package
validation remain open. The full properties plan remains active. No push is
part of this qualification.
