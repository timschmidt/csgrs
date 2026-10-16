# Locate the first inconsistent cell merge

`baseline-probes-v1` passes six commands and all 24 frozen cost trials. The
ordinary overlap, three-coincident and face-touching controls certify their
analytic volumes and deterministic artifacts. Production remains unchanged.
The reducer retains the complete transcript and finds a deletion-minimal subset
of original box indices `[12,17,20,21,24]`; both batch and merged forms still
fail. Deleting any one of these five makes the batch union succeed. This is
deletion minimal, not a claim of global minimization.

The five-box union has volume 46 independently: its horizontal area is 14 for
`0<z<1`, 16 for `1<z<2`, and 8 for `2<z<4`. The complete fixture and all earlier
phases remain required. Minimization does not replace the original volume-124
workload.

Add temporary, explicitly enabled diagnostic output in the native radial-cell
implementation. Capture the already computed sheet transitions, arrangement
points/facets and each cell-side merge, then the existing propagation failure.
Reconstruct the resulting integer difference constraints independently in a
diagnostic script. A sheet requires `w(back)-w(front)=delta`; a cell merge
requires equal winding. Report the first contradictory cycle, if present,
with its source sheets and radial edge. This diagnoses topology; it is not an
independent set oracle or a proposed production repair.

The temporary output is externally contained, excluded from performance claims,
and must be removed before final qualification. Run both traced five-box forms
and the original full case without tracing on the same frozen source. Preserve
every diagnostic build and transcript. No error is converted to geometry, no
transition is changed and no strict assertion is weakened by this instrumented
phase. A final repair must satisfy the original admission contract and its
downstream checks.
