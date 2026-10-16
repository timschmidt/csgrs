# Modeling acceptance after stronger scalar proofs

`qualification-v3` passes scalar/default/all-features, foundations, native mesh
and their Clippy checks, then reproduces two root-curve unit failures. Both
must be resolved without dropping their original geometry or using a weaker
policy. The owning production layers remain scalar proof and native curve
admission; CSGRS owns these modeling acceptance tests, P2/P3/P4/P8/P9/P17/P18/
P24/P25/P27/P35/P39.

`curve-semantics-v2` replays the complete original four-tooth/eight-sample rack
and 16-segment crescent against both preserved 045621eba and E12 coherent
graphs. Both pass independent contact, signed-displacement and set-membership
checks. `curve-semantics-v1` is retained as a compile failure: the diagnostic
initially imported Hyperlattice's Point2 instead of Hypercurve's native Point2.

The rack has 132 edges, zero stored pitch-line vertices, but eight pitch-line
crossings. For the first sample, write `r=2(theta-sin(theta))`,
`s=2(1-cos(theta))`, `theta=pi/24`. At the left pitch contact `q=-pi/4`, its
neighbors are `(q-5*r/4,-5*s/4)` and `(q+r,s)`. The pitch contact `(q,0)` lies
between them on the same line. The right side is mirrored; every tooth is
translated by an integer multiple of pi. The native region constructor already
documents discarding authored subdivision vertices when strict collinearity is
proved (`bezier_region.rs`, `try_from_native_contours_with_policy`).

Therefore the geometric contract is eight crossings at `tooth*pi +/- pi/4`,
not eight indispensable stored vertices. Keep the complete fixture and existing
tip/root-level checks. Count half-open edge crossings, require all eight exact
contacts to classify as certified boundary, and require y displacements of
`-1e-12` and `+1e-12` to classify as certified inside/outside. This strengthens
set-level evidence while permitting the documented representation compaction.
Fewer redundant vertices can alter newly constructed serialized artifacts;
this completeness extension is not a claim of byte-preserving optimization.

The original crescent now completes with Certified certainty under both
construction policies, and strict finite-profile projection completes with
Certified certainty. Require these results instead of permanent strict failure
or an approximate receipt merely because the approximate policy was selected.
Preserve all original dimensions/resolution; check independent membership at
(-1,0), (1,0), and (3,0): respectively inside, inside the removed unit polygon,
and outside the outer polygon. Existing separately unresolved predicate tests
continue to require typed Unknown and qualified terminal certainty.

No production geometry implementation, tolerance, fixture resolution, or policy
is changed. Rerun the full downstream matrix after these final test edits.
