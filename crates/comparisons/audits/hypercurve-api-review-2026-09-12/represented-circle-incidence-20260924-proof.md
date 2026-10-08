# Retained-point incidence on represented circles

Parent Hypercurve: 1cf070a5d1ec2ebf2796177da784a2d5a0494ee2.

The represented-circle predicate already handles stored Cartesian points,
native selected rational expressions and analytic parallel points. Its other
retained point variants use independent Cartesian enclosures alone. Those
enclosures can prove strict radial separation, but generally cannot certify an
exact boundary point. Fillet offset admission calls this predicate for an
authored circular support, so this representation distinction can block a
later exact operation.

The test-only V1 adds two regressions, with all production bytes verified
identical to the parent. The geometric oracle selects 2*t^2=1 with 0<t<1 on
the diagonal P=(t,t). A similarity with linear part [[1,-1],[s,s]], s=+/-1,
and center C=(3,-2) gives Q-C=(0,2*s*t). Its squared norm is therefore 2.
The represented-circle residual must be zero for squared radius 2, positive
for 1, and negative for 3. Both policies, rotation/reflection and the lazy
endpoint view of a finite chord must retain certified decisions. The first
query is the boundary query on the retained similarity image, before any
endpoint view or independent Cartesian projection.

The second regression uses a translated represented point at unit distance
from its center and a squared radius perturbed by the opaque scalar identity
sin(e)^2+cos(e)^2-1. STRICT must remain uncertain; APPROXIMATE_512 may decide
terminal equality only with Approximate512Consumed; a later forced-strict
query must remain uncertain. The sequence repeats to check that an approximate
terminal cannot become cached exact evidence.

The initial physical snapshot preparation hit EDQUOT before any build or test.
All owned processes were reaped. The adjacent cache-cleanup inventory records
763 obsolete Cargo release artifacts, reclaiming 7,322 MiB while protecting
all current artifacts named by the parent's successful build receipts.
Completed source snapshots, logs and copied executables were not changed;
all parent's qualified executable hashes were rechecked. The incomplete V1
copy was then completed and all 2,044 physical input bindings verified.

V1 is a diagnostic baseline, not a qualified implementation. Its driver skips
Clippy and runs the new geometric regression against freshly compiled parent
production code. Source bytes remain frozen until the driver is terminal and
its process handle has been reaped.

V1 reproduced the defect in the first strict boundary query: direction=-1,
endpoint=false, squared radius 2 returned Uncertain(Ordering) instead of
Decided(Zero). The test finished in 0.00 library seconds. Its driver exited
101 for the case, recorded all source bindings and reaped its child; the
outer driver exited 1 and its process handle was also reaped before editing.

V2 preserves both regressions byte-for-byte. In the represented-circle
predicate, the existing native dispatch and refinement steps 0,2,4 remain.
Before step 8, one optional strict replay calls the existing shared projective
radial residual helper. Its squared common nonzero denominator preserves the
affine sign and its selected-field relations can prove exact zero. A decided
sign returns immediately; an unavailable or uncertain replay retains the
existing refinement sequence and explicit 512-step approximate terminal.
The strict pass preserves retained-object policy authority while suppressing
approximate equality in the optional shortcut. No representation, cache,
forwarding overload or alternate predicate implementation is introduced.

V2 is frozen over 2,044 physical inputs. Formatting and all-feature all-target
Clippy pass; the remaining checks and focused regressions are in progress.

V2 focused qualification passes: formatting, both all-target Clippy feature
configurations with warnings denied, and all eight focused cases. Both new
regressions pass, including the previously failing strict boundary query.
All 2,044 source hashes and the copied release executable are bound to the
terminal receipt, and the focused driver handle is reaped. The affected
qualification now runs fillet/circle/cusp/chord/retained-point/corner library
cases and all 225 cases from nine public targets against the same sources.
The three previously isolated long compositions retain their original limits.

Final V2 qualification: all eight focused cases pass; both new regressions
complete below the library runner's 0.01-second display resolution. Formatting
and both all-target Clippy feature configurations pass. Across 626 affected
and public cases, 620 pass, three are ignored and three unchanged 75-second
timeouts remain. All 225 public integration cases pass. The isolated long
compositions complete in 37.72, 148.40 and 61.72 wall seconds under unchanged
limits. The parent's full 1,465-case qualification remains the broader baseline;
the five unselected chamfer timeouts remain outstanding too.

Every input, copied executable, index and HEAD binding is verified, and all
owned processes are reaped. Committed as Hypercurve c46890e2694b50d8ed5dc0baddd43023e757dc3e.
All thirty repositories are clean after commit; nothing was pushed. The full
implementation goal remains active. The V2 qualification, staged and post-commit
receipts record these bindings.
