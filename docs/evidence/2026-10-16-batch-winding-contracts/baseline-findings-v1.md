# Current-revision reproduction

`baseline-v1` recompiles the unchanged complete diagnostic from the preceding
milestone against the immutable libraries qualified for
`f74a7848805398eb62dddb167b3997662e6f90d5`. All 2,594 library inputs match their
previously retained hashes. The three runner commands pass as a diagnostic;
their zero exit codes do not make the two failed geometry operations successful.

The 26-operand native union and the checked merged-shell native union both return
`SurfaceArrangementFailed { reason: "surface cell winding propagation is inconsistent" }`.
The full original binary accumulation succeeds with 636 triangles, exact
six-volume 744 (volume 124), and a Certified closed-PWN validation receipt.
No production source changed before this reproduction.

The next diagnostic may minimize this configuration to locate the inconsistent
links or transitions. The original complete fixture remains required. Successful
ordinary Boolean controls must be frozen before implementation for semantic,
deterministic-artifact and cost comparisons; failed completion is never a faster
successful operation.
