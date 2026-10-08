# Normalized path and native region admission

Hypercurve commit: `9f7a63686a9ee1a40fcedaba5e822234bd960f42`.

Path and explicit native-contour constructors now publish the exact regularized set. Coincident material/hole cancellation, nested material seams, winding cancellation, and normalized filled-left ownership are handled before the object is returned. The separate signed-composition constructor and state flag, plus the single-contour regularization wrapper, are removed. Public caller migrations preserve consumed terminal certainty and choose geometric corners instead of depending on authored boundary indices.

SVG retains authored loops privately until its selected fill rule is applied. Segmentation and native line/arc views accept represented straight retained fragments without inventing a native parameter map. Unary arrangement uses the existing conservative AABB scheduling authority, including complete fallback for unavailable bounds and the original surviving contact order. Native admission compacts only strictly certified codirected collinear runs.

The immutable parent public fixture exposed 12 construction-invariant failures across 16 cases; explicit regularization passed all 36 point replays. The migrated public fixture passes all 8 current API cases and 18 point replays. The two removed routes are not retained as compatibility interfaces.

Final broad qualification ran 49 Hypercurve/HyperBREP targets: 2,316 passes, the same five known promotion failures, nine ignored tests, eight prior expensive exclusions, no new failures, and no timeouts. Four minimal-feature/all-target and release/all-feature builds pass. All 396 working-source and 1,007 isolated-source hashes, all 49 archived executables, and all 13 staged and committed files were verified. The actual public probe and full-run normal library have matching SHA-256 `fd1203e4eb3e6fa064be46977d35d871e0185d2eed992d795a23ca191d56afdf`.

An earlier candidate caused a required PCB fixture to time out at 300 seconds. That regression was repaired, not excluded. Final PCB regression target time is 11.13 seconds for two tests. The larger fixture reduces from 35,840 authored edges to 2,560 certified line runs. These measurements concern this fixture; they do not establish general throughput.

Evidence: `normalized-path-admission-qualification.json`, `normalized-path-admission-post-commit.json`, `normalized-path-admission-full2-full-results.json`, `normalized-path-admission-full2-libraries/`, `normalized-region-admission-baseline.json`, and `normalized-region-admission-completed.json`. Earlier candidates and the full1 timeout remain archived.

Hyperreal uses the pinned `a2da8e2b5de9a1a4653d3662f2f05cb6533fd7ef` snapshot. The other session's Hyperreal files are neither edited nor build inputs. The unfinished mapped-point inverse in working `src/bezier_offset.rs` is excluded from this commit and every admission snapshot; its required failing normal-sheet test remains recorded separately.

The full goal remains active. Public retained-loop and traversal constructors still admit raw inputs. Corner replacement, certified segmentation, and Boolean fallback producers need the same publication audit. Global authored winding across SVG subpaths, compact mapped-point inverse proofs, source-cusp frames, the known failures/exclusions, and broader representation/evidence consolidation remain required.

Downstream migration is being qualified separately. Hypercircuit now collects positive compound operands into one normalized construction instead of maintaining a second bounds/union-find/sequential-union implementation. A new two-policy regression checks holes, islands, duplicate operands, and seam removal against sequential union. Two stale scalar-only endpoint consumers in Hypercircuit and CSGrs are migrated to exact retained-point comparison. These downstream changes are not yet included in the qualified Hypercurve commit.


Downstream migration is now committed: Hypercircuit `ad421497ce6e88fe936818ec198c8a1b779776c7` and CSGrs `4e5b9ebb59127a559ecda9779d1b8af05ed40fcb`. Hypercircuit's geometry/interchange all-target check and 29 selected tests pass; all fifteen CSGrs native-geometry tests pass. Endpoint comparison and exact boundary consumers are migrated directly, without compatibility APIs. The additional strict rack decision gap is repaired in Hypercurve `2d26b0ae4d6178700d5c26ccdab5f8ac25a1f787`, whose separate full qualification records 2,317 core passes with the same five known failures and no new failures/timeouts. See `opposed-endpoint-contact-plan.md` and the bound core/caller qualification records. No owned build or test handle remains live.
