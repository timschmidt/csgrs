# Retain normalized material components through downstream extrusion and DRC

Parents: Hypercurve `28fd4d6e909686ce0cead19f5a64b176d6bb4206`, HyperBREP `878fcb441b281ab1b44051ca4420b28092c3c2ed`, HyperDRC `5c9d3a509be606f1a0825dcfb31f8b178f335cd1`.

`CurveRegion2::material_components` decomposes the regularized filled set into one material exterior plus its owned holes per result. Recursive islands are separate components; isolated boundary contacts do not imply one material interior. Results have the material loop first and retain filled-left orientation. This is a geometry operation needed by two callers, not a compatibility adapter for a removed constructor.

Normalize once, resolve exact hole ownership once, and retain the certified boundary fragments directly. Subsetting complete material components preserves noncrossing filled-left topology. Each new component inherits source normalization and ownership decision requirements. A single component already in material-first order reuses the original shared data and its caches. Native line contours need no synthetic arrangement indices; exact represented endpoints already supply their line geometry. Selected carriers keep their original support and parameter evidence.

HyperBREP consumes those components for extrusion and planar support, removing its manual role/side reconstruction and raw-region construction. HyperDRC consumes the same component operation for its exact-backed finite conversion cache. Neither caller must reconstruct endpoint connectivity or guess nested ownership.

Validation: independent exact areas and membership for two levels of holes/islands plus a disjoint material; pairwise intersections and union/XOR recomposition; selected-root components after affine transport and union; repeated single-component data sharing; existing B-rep corpus; affected DRC geometry tests; all-target compile checks in all three repositories. Bind actual source manifests and executables. Exclude the unfinished offset inverse file and use pinned Hyperreal, leaving the other session's edits untouched.

The direct raw-loop constructors still need removal and caller migration inside Hypercurve tests/benchmarks/fuzz. This increment removes their two external production consumers. Operation-output normalization, the mapped normal-sheet inverse, and the broader unresolved goal remain outstanding.
