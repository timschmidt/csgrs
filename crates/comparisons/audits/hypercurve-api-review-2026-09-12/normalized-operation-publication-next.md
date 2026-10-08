# Normalize operation publication after raw factory removal

Continue after the raw region factories become crate-private. Do not mark public `CurveRegion2` publication closed yet: corner replacement and the XOR composition fallback still return raw intermediate loops.

## Public corner-edit witness to bind before editing production

Use the normalized region with material square `[0, 8]^2` and a hole `[1, 3]^2`, admitted through general closed paths. Locate the material corner `(0, 0)` by exact incidence on the normalized boundary. Chamfer it with setbacks 5, or fillet it with radius 5, under both policies.

The corner cut intersects the hole. The intended result has one regularized boundary: the surviving void opens through the edited outer boundary. In particular `(1, 2)` lies outside after the chamfer, while the old hole boundary at that point must no longer claim boundary ownership. For the fillet use `(1, 1)` as a removed old hole-boundary point and check exact intersection points `(1, 2)` and `(2, 1)` separately. Confirm the sample oracle and actual baseline before writing production changes. Compare publication against explicit normalization and an independent point/loop-count oracle; do not count a blocked operation as exact closure.

`with_corner_chain_replaced` at `src/bezier_region.rs` retains contact connectivity through `try_new_from_certified_connected_chain`, then attaches roles, fill rules, and filled sides from the source. These are necessary inputs, not a normalization certificate. Normalize that result using the existing unary arrangement authority and preserve the caller's operation tag. Do not independently reconstruct selected contact coordinates or claim a local geometric certificate without proving absence of intersections with every other boundary.

The existing kernel may expose missing evidentiary reuse once normalization becomes immediate. Preserve the intended set and selected contact authority rather than suppressing such failures or reverting to raw publication. Qualify affected corner/editing tests and repeated-operation regressions, with parent comparisons for the five already recorded promotion failures and the two unresolved analytic timeouts.

## XOR fallback

`compose_xor_from_exact_regions` at `src/curve_region_boolean.rs` combines normalized union and intersection boundaries, flips the intersection's filled sides, strips colliding provenance indices, and returns a raw region. The set identity is sound, but coincident and hidden seams must be removed before publication. Unary normalization is independent of the binary XOR fallback, so calling it does not inherently recurse into XOR. Preserve the Boolean error tag and consumed decision requirements.

Qualify the fallback directly in crate-private tests using independently known overlapping/identical/nested filled sets. Assert normalized boundary ownership and reusable result certificates, not just membership away from boundaries. Keep public Boolean compositions in integration coverage. Do not require a public case to fail merely to exercise a private fallback deterministically.

## Remaining audit

Review complete affine, certified-segmentation, and offset producer return paths. `regularized_exact_offset_band_arrangement` already normalizes its intermediate band, so its raw builder is not itself a publication defect. Keep `regularized_region` and its cache until all producers satisfy the invariant. Afterward remove redundant normalization state and wrappers based on actual ownership of topology evidence.

Hyperreal working changes remain owned by the other session. The mapped-point inverse working changes in `src/bezier_offset.rs` remain unqualified and separate.
