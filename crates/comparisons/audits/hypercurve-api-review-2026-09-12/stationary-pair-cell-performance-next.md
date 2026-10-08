# Source-cell proof reuse after V532

Current pending integration V537 passes76 focused cases/six checks. V536 broad394-case qualification is running, exact outer35300: no input edits until reaped. Ordinary joined continuous-family workload is5.471s versus3.017s atV527; nonlinear/linear is3.969s, near baseline after ordinary cut evidence was restored. These are whole-test timings, not per-fillet latency.

Concrete duplicate work: PreparedFilletCarrier2::parallel computes original-source singularity_analysis and regular_subranges under an optional bounded scheduling pass, then discards the cells. Each signed radius choice creates another FilletOffsetCarrier2::Parallel; parallel_pair_centers recomputes both original analyses, cells, and finite-cell derivative signs. The source, original displacement, finite range, cut side and requested mode do not change with center-support distance. Source pole/speed incident barriers are also independent of center distance, but verify the incident-domain construction before sharing that result.

Candidate consolidation: a private lazy successful source-cell preparation owned by PreparedFilletCarrier2, borrowed by each FilletOffsetCarrier2. Reuse it in the line/parallel and parallel/pair branches, where cell/ray/seam preparation currently duplicates. Preserve optional/demand-driven direction scheduling: do not make the prepared constructor eagerly reject a case the downstream exact solver could previously decide; a cheap contradictory center must still short-circuit. A per-operation OnceCell can keep exact decided cell/sign evidence; do not turn uncertainty into a global negative cache. Original contact-curve orientation is distinct from center-support derivative orientation and must stay distinct.

For finite cells, a constraint's expected original derivative sign is already proved everywhere on the owned cell. Component reconstruction currently recomputes signs at selected roots; consider reusing that proof through the existing internal fillet preparation. Keep CurveParameterComponent2 generic: do not add a new public family/frame variant or copy original curves into it merely to cache a fillet sign.

Separately BezierParallelSourceData2 already shares polynomial power basis, differential, and PH speed. A success-only primitive hodograph factor/field cache belongs on that existing source owner. Today source_oriented_regularized_tangent_field_at_interior recomputes GCD and exact divisions for every range. Preserve the arbitrary exact-coefficient rank-one fallback and source-sheet orientation; monic normalization failure is not a geometric rejection. Benchmark independently before combining cache changes with the source-cell consolidation.

Current new point/frame path and finite partitions add actual supported stationary cuts; removing those partitions merely to restore a timing would drop needed one-sided cusp ownership. Preserve closure while reducing repeated proofs.

Verified source inspection: incident_domain_from_parameter builds its anchor bridge and barrier only from the source weight and speed polynomials, so its geometry does not depend on center-support distance. It still depends on the source endpoint authority and outward direction. BezierParallelIncidentDomain2 retains a first-source-pole/speed-barrier contract, but parameter_ray() erases that provenance into the generic ray. The new same-sheet check consequently re-isolates a raw/primitive tangent dot product. Future evidence reuse should retain a source identity/certificate with the existing incident owner before eliminating this check; a generic hand-built ray does not carry the same theorem.

Source-cell preparation has only two production PreparedFilletCarrier2::new call sites in solve_carrier_fillet_corner. FilletOffsetCarrier2 already borrows prepared data through its second lifetime; a lazy borrowed domain proof needs no new public type or compatibility path. Update the existing direct construction tests with any private representation change. The line/parallel path deliberately computes singularities only after Boundary incidence; preserve that demand behavior when sharing preparation.

A smaller independent prerequisite is a success-only primitive-hodograph cache on BezierParallelSourceData2. Store a proven constant GCD as cached None; otherwise retain the exact common factor and the primitive x/y field. Choose its sign at the requested branch interior. Retain the negative field lazily if requested, so repeated same-sheet frames share the same Arc. Do not cache unavailable monic GCD as None: arbitrary exact coefficient rank-one fallback must remain reachable, and uncertainty may improve. This cache shares clone/with_distance work without a new public type or changing source ownership. Validate both opposite cusp branches, re-use across offset distances, and the existing arbitrary-exact rank-one cases; compare full family and reoffset workloads before attributing a speedup.

Do not conflate a prepared original finite range and its expanded incident bridge in a single unkeyed analysis cache. TrimOnly scheduling analyzes the original finite range; the pair solver expands it through the certified endpoint-to-anchor bridge for TrimOrExtend. The native line/parallel branch currently uses finite incidence plus open-ray incidence separately. Inspection suggests a possible gap for contacts strictly inside an algebraic endpoint bridge; this needs an independent counterexample before changing its domain. Its incident method explicitly isolates beyond the anchor and does not itself visit the bridge.

Independent bridge probe design (unrun): P(t)=(t,t²), retained previous endpoint alpha=positive selected root of2t²−1 with isolator upper anchor1; next support is the horizontal line y=1/2 traversed leftwards. Pick intended extended contact t=3/4 (strictly between alpha and1). For clockwise fillet, radius r=√13/[16(√13+2)], center=(3/4+3r/√13,1/2+r), next contact=(center.x,1/2). All are exact; the radial vector to P(3/4) is r*(3/√13,−2/√13), perpendicular to tangent(1,3/2), and the line radial is(0,r). Both retained ends extend, and the bridge contact must be owned once. Construct the endpoint as selected parameter evidence rather than a represented Real sqrt, which would make it its own anchor and miss the bridge. Assert its actual anchor exceeds3/4 before testing. The suspected native line/parallel finite+ray gap has not yet been executed, so it is not a claimed failing regression.

Cache ownership refinement: BezierParallelDifferential2 is already retained exactly once in BezierParallelSourceData2, is distance-independent, and has one production initializer. Keeping the successful primitive factor/field cache on that differential directly associates it with the polynomial evidence it factors, avoiding a second source-level owner. The first implementation should preserve interior selection and rank-one fallback unchanged; benchmark GCD/division reuse separately before adding other shortcuts. Hypersolve's existing GCD/division functions explicitly use STRICT coefficient predicates, so successful factor evidence is independent of an approximate caller policy.

V539 exact tangent-pair replay passes the stationary quartic family in33.306s (20 constrained requests plus radius-only checks across both policies/path directions). V540 broad is running on the same source. V542 now has an UNAPPLIED/UNTESTED candidate source file, a unified diff and pinned source hash. It moves successful monic GCD/division proof into the source differential and lazily shares positive/negative primitive fields. The opaque rank-one fallback is unchanged. Do not copy it over a changed bezier_offset.rs: verify primitive-tangent-reuse-v542-base.json first and adapt the diff if another closure fix intervenes. Test V541 region composition before starting this performance experiment.

V545 source inspection corrects the earlier cache-owner claim: there are six temporary BezierParallelDifferential2 initializers for regularized frames, in addition to the retained source initializer. V542 UNAPPLIED candidate now stores the primitive-factor cache on BezierParallelSourceData2 (one initializer), so temporary differential views remain unchanged and carry no unused cache. This supersedes the differential-owner proposal above. Candidate still uncompiled/unmeasured and deferred behind the V541 offset composition failure; its driver also needs retargeting to the eventual successful closure baseline.

V556 candidate driver prepared, UNRUN, retargeted to successful/committed V555 finite-field qualification. It uses the unchanged V542 source-factor candidate/base SHA, and now reuses the promoted stationary-family composition test instead of appending V541 twice. Fifteen focused cases compare against the new closure baseline; do not launch before exact V555 outer58041 is reaped and the production commit is sealed. Decide necessity from V555 performance evidence; no cache production change is pending.


V568 continuation (28 September): the rebased primitive tangent cache passes
16 copied-source regressions. Exact outer 2653 and formatter 55917 are reaped 0.
It is applied as the sole pending source change, under normal V571 qualification
(exact outer 30604). Strict/approximate full reoffsets pass 116.221/115.671s;
qualification is still running. Do not claim a cache commit yet. Focused
whole-operation timings are essentially unchanged: joined family 5.471→5.521s,
quartic family 34.012→33.557s, independent oblique family 21.895→21.492s.
The prior joined-family performance regression remains open.

The algebraic endpoint bridge failure discussed above was independently fixed
and committed as 8c013bc7; the stationary region composition is fixed by
9baa94d8; algebraic image point constraints are fixed by 18976738. Their active
status is in implementation.md. Old V542/V556 whole-file cache candidates are
stale and must not overwrite the current source.

The next source-cell investigation has bounded phase counters prepared in
source-cell-measurement-v574.rs. Combined copied-source probe V575 is unrun and
requires the planned V573 interval migration to be committed. It also probes a
remaining recursive point-constraint representation; that completeness check
precedes optional performance work if it demonstrates a failure. No repeated
source-cell cost has yet been measured directly.
