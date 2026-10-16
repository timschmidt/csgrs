# Polyhedral construction progress

On `67a3daae8`, a cyclic rotation of an ordinary concave face could reverse its
triangulation and make a loft fail the next Boolean. Crossed faces and repeated
sections could publish successful incomplete or degenerate meshes. The retained
baseline and candidate replay reproduce the behavior and its correction.

Hypertri now validates a simple ring through shared Hyperlimit predicates,
reuses its existing earcut implementation, and returns input winding with the
oriented triangles and aggregate certainty. Explicit closure and forward
collinear boundary vertices remain supported. CSGRS polyhedron faces retain
authored winding; loft orientation consumes whole-ring winding, consecutive
equal sections coalesce with their certainty, and the completed boundary passes
native closed-PWN validation. Native causes survive both public API forms.
Owned CSGRS vector reservations use the existing checked-capacity helpers.

The migration and domain are in [implementation-contract-v1.md](implementation-contract-v1.md).
Open faces and points-only structural buffers do not acquire a solid-topology
claim. New simple-ring validation rejects other zero-length edges, crossings,
backtracking and nonadjacent repeated coordinates. General earcut's existing
normalization and hole behavior is unchanged. Closed PWN does not imply
embeddedness or fidelity to an unspecified continuous swept surface.

All **40 qualification commands pass**. Results by configuration are:

| Configuration | Passed | Existing ignored |
| --- | ---: | ---: |
| Hypertri, all features | 193 | 0 |
| Hypermesh, all features | 245 | 7 |
| CSGRS, minimal | 121 | 0 |
| CSGRS, curve | 277 | 0 |
| CSGRS, all features | 394 | 1 |
| FFI | 10 | 0 |
| Root / mesh / triangulation doctests | 1 / 0 / 4 | 0 |

These totals overlap across feature configurations. Formatting, warning-denied
Clippy, documentation, original benchmark consumers, C/C++, Python, Go, and
JavaScript/WASM with TypeScript checks also pass. The new binding consumers
reject crossed faces and recover with a concave face of independently checked
signed area 5. A later one-byte README table-separator correction passes final
README tests, docs and formatting in `cost-build-v1`; all production and test
sources remain identical to the 40-command qualification.

Independent acceptance includes every cyclic/reversed L face and prism,
positive loft volume 10 and an empty notch, native topology, empty union,
complete subtraction and recovery. Axis permutations and negative travel,
collinear boundaries, redundant sections, invalid rings, native predicate
causes and strict→approximate→strict qualification are covered. Original
tetrahedron, square-loft and complete README construction fixtures retain their
independently derived volumes. Full README render regeneration was not run.

All **32 paired cost children** pass, following sixteen completed pre-edit
baseline children. Original tetrahedron and square-loft output bytes agree in
every cost replay. The independent diagnostic replay also preserves the
tetrahedron, square-loft and cube-sum artifacts byte for byte. The timing
boundary includes the candidate's internal validation; see
[cost-clarification-v1.md](cost-clarification-v1.md).

| Completed construction | Baseline median µs | Candidate median µs | Ratio |
| --- | ---: | ---: | ---: |
| Tetrahedron, cold | 82.015 | 89.934 | 1.097 |
| Tetrahedron, warm | 9.589 | 15.399 | 1.606 |
| Square loft, cold | 81.874 | 92.884 | 1.134 |
| Square loft, warm | 8.680 | 16.769 | 1.932 |

Each median has three retained samples. Tetrahedron construction adds 32
allocations and 2,640 requested bytes; loft adds 27 allocations and 4,692
requested bytes, under both temperatures. Peak requested bytes change by +24
and -304 respectively; retained deltas are unchanged. These are small fixed
probes, not application-wide estimates or an optimization claim. The correctness
repair is retained with its added cost visible. Library, driver and source sizes,
all raw samples and the interrupted/failed diagnostic phases remain in the
[summary](qualification-summary-v1.json) and its linked raw records.

Limits remain explicit: quadratic edge-pair validation, native triangulation,
scalar growth, total work, heap bytes and cancellation have no complete internal
bound. Tests use external 1800-second/8-GiB containment; Node/WASM has no virtual
address-space bound and cost children additionally have 120-second walls.
Allocation counters exclude allocator overhead.

The evidence closure uses a lossless hash-pinned parent audit plus additions,
as documented in [artifact-format-v1.md](artifact-format-v1.md), to avoid another
full copy of the inherited audit on the nearly full repository filesystem.
It still verifies every available inherited artifact and preserves every
expectation, including all 27,093 historical missing paths. The 35 pending
Hypercurve/Hyperdrc files remain unchanged and separately unadmitted. Only the
ledger's raw `current_work` value is updated.

The next reproduced semantic failure is general Minkowski sum: disconnected
inputs are convexified, unused points change the set, malformed operands can be
hidden by empty input, and native hull errors are flattened. These failures
remain present in the candidate replay and require a separate native capability
and explicit public migration. End-to-end provenance, complete internal resource
contracts, clean external package/release validation, full CI/formal/fuzz/coverage
and the rest of the properties plan remain open. This is a local qualified
implementation milestone, not full-plan completion or release admission.
