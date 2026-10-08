# Read-only next API audit

RationalBezierOverlapOrientation2 is already the public orientation of generic CurveIntersectionOverlap2 and of analytic parallel overlap evidence. Its Same/Reversed semantics carry no rational-coefficient or Bezier-parameter theorem. Consider moving the definition to the shared curve-intersection vocabulary as CurveOverlapOrientation2, updating every controlled caller and deleting the old export without an alias. This is a naming/ownership migration, not a completeness or performance fix. Verify identifier-only caller edits and compile all affected targets; do not add tests mirroring a mechanical rename.

The parallel/rational candidate system has a superficially similar raw overlap field to the removed parallel-pair projection field, but it also stores native rational-image correspondences with no selected component certificate. That field cannot simply be deleted by repeating V694. No second mirror-removal candidate is proposed.

This artifact is not an active driver input. V697 owns the production, mirror, archive, driver and V696 candidate until exact reaping; the proposed API audit remains unimplemented. Prefer a demonstrated closure or computational issue if one emerges.
