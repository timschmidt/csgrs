# Mesh distribution and concatenation acceptance contract

Target revision: `00e612307`, including the unchanged 35 pending native files.
The previous goal turn made verified progress on typed text construction; it
did not complete the properties plan or admit those pending native changes.

Reproduce the current behavior before implementation. The review candidates
are strict distribution wrappers replacing errors with empty geometry, native
Boolean causes converted to strings, unchecked copy/buffer capacities, and
single-copy/disjoint/zero-count shortcuts bypassing input validation. Mesh
concatenation also needs checked index rebasing and explicit failure.

CSGRS owns distribution layout, modeling composition, argument validation and
the public result/certainty surface. Hypermesh owns triangle index validity,
closed-PWN validity and native buffer/index composition. Reuse its validation
and topology machinery; do not introduce an adapter topology test or reject
balanced nonmanifold PWN inputs by requiring manifoldness. Concatenation is a
separate structural operation: it preserves open meshes and winding
multiplicity, and does not claim regularized union or closed topology.

Relevant properties: P4–P6, P8–P11, P17–P27, P35–P39. Missing acceptance tests
must distinguish valid canonical empty results, invalid sources, undecided
predicates and capacity exhaustion through both strict wrappers and context
entry points. Source validation must survive zero-copy, one-copy and disjoint
shortcuts. Linear direction must be provably nonzero; preserve signed spacing,
signed arc radius and the existing midpoint placement for a single arc copy.
Do not silently change valid coordinates, order, topology or certainty.

Use independent placement, membership, signed-volume and topology expectations,
including overlapping/touching copies, accumulation from empty and subsequent
subtraction to empty. Preserve the original full benchmark/query workloads:
eight linear unit cubes at spacing 2; a 4-by-4 grid at spacing 2; twelve unit
cubes at radius 10 and angles 0 through 330 degrees; and the feature benchmark's
22-copy concatenation. Record every represented
coordinate and triangle row separately from semantic assertions.

Exercise cold/warm/disabled/evicted/mutated cache states and strict → approximate
→ strict. A retained proof may strengthen an answer; a retained approximate
result cannot answer a strict request. Failures must retain native causes and
must not enter caches as successful geometry. Input-controlled capacities must
be checked before allocation and reported typed. Existing internal expression,
total-work, heap-byte and cancellation limitations remain explicit; dangerous
probes use external containment and are not evidence of internal bounds.

The API migration must update all affected Rust tests, benchmark consumers and
documentation. Run affected native/modeling tests, feature combinations, lint,
docs and actual benchmark consumers after the final code edit. Compare original
full-workload outputs and retain all cost trials, including adverse results;
this is a correctness extension, not a performance-only claim. Only
`current_work` in the protected ledger may change. Full CI, clean external
release validation and all remaining properties obligations stay open.
