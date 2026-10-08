# Nonlinear finite circle domains after finite fiber promotion

The discovery source `finite-circle-nonlinear-next.rs` is preserved from Hypercurve 384f667. On 63dc97a its separate replay artifact certifies all eight point replays on four complete unit queries; four equivalent exterior queries remain explicitly incomplete. Do not overwrite the historical baseline.

Finite fiber promotion now removes a prerequisite limitation: common selected, resultant, quotient, and local-norm root schedules accept an explicit CurveParameterRange2 and share CurveParameterDomain2 envelopes. Local selected-fiber filtering preserves arbitrary selected boundary identities. Scalar promotion uses its own finite isolator rather than a unit chart or incident ray.

The next geometric migration must pass active ranges through circle/rational dispatch and each frame-specific kernel, together with target weight validation and overlap partitioning. Merely removing Pair::require_unit_domain is unsound:

- rational_intersections_internal has a unit support-bounds disjointness shortcut for selected chord-normal/radial frames.
- selected_parallel_normal_rational_system, selected_radial_rational_system, represented_rational_system, represented_rational_component_system, and chord_normal_projective_rational_system validate or orient target denominators using unit_weight_sign.
- selected_parallel_normal_rational_selected_fiber_intersections and its overlap replay seed roots and endpoint boundaries in [0,1].
- selected-radial, recursive-radial, represented and chord-normal contact/overlap authorities instantiate SelectedThirdAxisDomain2::Finite(unit).
- exact_line_image_selected_radial_rational_intersections and the general rational-frame replay retain separate unit parameter discovery assumptions.

Root envelopes schedule candidates; original exact ranges own admission. A denominator nonzero on a selected active range can have a pole elsewhere in a coarse outer envelope. Isolated candidates outside the active range must be rejected before pole-dependent geometric evaluation. Positive-dimensional replay must either refine to a pole-free enclosing chart or retain pole boundaries/open ownership before clipping; an envelope endpoint cannot become an authored endpoint by accident. Preserve positive-dimensional correspondences, residual isolated contacts, retracing, source parameter charts, and selected tangent identities.

Update controlled callers directly. Keep optional represented/rational image fast paths subordinate to complete exact replay. Consolidate duplicate discovery/publication APIs where the retained parameter map can remain demand-driven. Do not add a new public curve or point carrier solely to cover a wider parameter interval.

Qualification should compare equivalent unit, exterior, negative and reversed source charts; selected bounds; active/inactive poles and signed weights; tangent/transverse contacts; coincident circular rational components and retracing; both operand orders and policies; split/re-intersection and region evidence reentry. The five known promotion failures, eight previously unqualified expensive cases, normalized region admission and the wider architecture goal remain separate outstanding work.

The mixed-component API prerequisite is committed in Hypercurve `e99dfbd6902caf5b1bee89617d36ceeca4165082`: mapped and selected-fiber results carry contacts and overlap cells together, with split/re-intersection regressions. Closed singleton clipping is now qualified and committed in `5c47fb199ddcd687ef030811221acdf76db8a624`. The executed [1/2,3/4] adapter regression reproduced a false empty result and now retains the visit at 3/4. Common public dispatch and the adapter share endpoint replay through the original correspondence, preserving the circle parameter evidence and avoiding duplicate contacts at endpoints owned by positive cells. The independent 1/4 visit remains distinct whenever its parameter is admitted.

Proceed with the explicit finite-domain migration above. Keep positive-length range restriction separate from closed point-contact discovery, both using the shared correspondence. The common `singleton_contact` replay requires that positive clipping has already returned no span. Do not reintroduce a private positive-only clipper or treat a zero-length range as overlap evidence. The rational-component adapter's selected-fiber branch still forwards original components for consumer-side clipping; widening discovery must give mapped and selected authorities the same explicit active-domain contract.
