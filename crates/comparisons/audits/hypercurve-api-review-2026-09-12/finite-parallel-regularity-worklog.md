# Explicit finite parallel regularity

Parent: Hypercurve 7b15b7826c4fc72603fd42d223c642bbad5d397e. Goal remains active.

The public singularity API now requires an exact finite range; all production/test/benchmark callers are updated with no compatibility method. The existing finite-domain root authority clips speed roots, poles, and cusp candidates against the original endpoint identities. The exact quadratic formula uses the same domain predicate. The report retains its requested range. Generic range construction and the unit range become public, without adding another range representation.

Retained parallel composition and chord tangency certification use their actual range. Fragment validation passes its admitted range, but its exterior admission guard is deliberately still present until related pruning premises are repaired. Derivative and approximation callers explicitly use their authored unit domains. No claim of complete analytic region closure or improved performance is made.

New regressions use P(t)=(t,(t-2)^2) and its equivalent P(4s) chart. The source is everywhere regular and the positive distance-1/2 parallel has its only cusp at t=2 (s=1/2), since |P'|^2=1+4(t-2)^2 and curvature numerator=2. Negative displacement shares the squared equation but not its selected sheet. Quadratic, elevated cubic, and negative-gauge rational carriers, closed endpoints, reversals, and excluded subranges are checked. Separate stationary and pole fixtures distinguish retained source roots from undefined rational support; the selected-fiber range test uses 1+sqrt(2) as an unreduced endpoint authority and checks both cusp and pole admission.

The first final-source no-default all-target check passes. Release builds and focused qualification are in progress; source files are frozen.

Both release builds and no-default all-target checks pass. All five focused checks and all 28 frozen public matrices pass, as do the three fuzz/UI caller checks. Normal library SHA 3f204f6dcef0133f415650348226026d9a964d24eae3cd2fbd33cc53b4667deb; HC libtest SHA 2e35669386918133a2c969c85f3c6ad5f70254c318647db1001de076a0f4f2a5. The full 49-target qualification is running on unchanged sources. The next contact-pruning counterexample is derived in finite-parallel-pruning-next.md; it has not yet been executed as a parent regression.

Final qualification: 2,280 passes (2,048 Hypercurve, 232 HyperBREP) across 49 targets; five unchanged known failures, nine ignored, eight existing expensive exclusions, no new failures/timeouts. Five focused checks, 28 public matrices, three callers, formatting and 396 source hashes pass. Committed 2c16c30bb1aba38f08456bed462b5ea058bc5c71; all source hashes unchanged after commit and all 30 repositories clean. No failed candidate build/test attempts in this stage. The original goal remains active.
