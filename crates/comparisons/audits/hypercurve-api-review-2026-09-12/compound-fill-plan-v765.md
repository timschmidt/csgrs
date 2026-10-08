## V766 compound-fill implementation under qualification

V764 baseline probe exact outer44832 reaped0, reproducing one wrong nested NonZero fill and four rejected overlapping subpath imports, with three passing controls. All are exact rational rectangles. The SVG global-fill contract is W3C SVG2 painting section13.4.2, https://www.w3.org/TR/SVG2/painting.html#FillRuleProperty: all subpaths contribute to one winding rule. Source and binary identities pinned.

V765 candidate makes FillRule explicit on CurveRegion2::try_from_boundary_paths and migrates104 existing calls across HC/HB/CS directly to EvenOdd, preserving their previous intent. SVG imports call that constructor directly rather than inferring raw roles and duplicating fill-rule vectors. A unary-context-only fill selection uses the sum of retained signed winding. Published region representation is unchanged. Boundary-side ray helpers now return winding evidence only; one context selector combines that evidence for both local-side and propagated-face decisions. Existing explicit Material/Hole selection stays unchanged, as does unordered endpoint assembly's documented/tested parity composition.

Five new regression functions cover SVG nesting/overlap/recursive islands/cancellation, orientation/order, winding2 minus winding1, circular algebraic intersections, nonuniform rational and generated selected-endpoint boundaries, normalization replay, offset and Boolean reentry. Both policies covered in exact-core tests. Existing assertions retained.

V766 formatter/promotion outer27623 reaped0;30files across3repos promoted. Normal qualification exact outer67842 ACTIVE; immutable production/candidate/mirror/archive/driver until exact reap.12targets,91selected HC library cases, full affected integration inventories, existing50HB+17CS cases,8checks. Reviewer/seal notyetprepared. Full goal remains active. NextunusedV767.

