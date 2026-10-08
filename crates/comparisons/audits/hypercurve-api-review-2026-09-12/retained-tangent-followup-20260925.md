# Retained tangent parameter migration (read-only design)

`CurveTangent2::RetainedParallel` in `bezier_region.rs` still stores a native
`BezierParameter2` plus an optional selected-fiber parameter. Its producer
`exact_parallel_region_endpoint_tangent` promotes a selected fiber solely to
populate that native field, then stores the original selection alongside it.
This is an internal construction-history distinction already covered by
`CurveParameter2` and should be removed rather than given a compatibility
adapter. Recursive parameters currently fall back to an oriented tangent chord.

The coherent migration replaces both fields with one `CurveParameter2`, updates
the parallel tangent/vector and tangent/pair predicates to consume retained
scalar authority, and updates all constructors and pattern matches directly.
Native polynomial kernels remain optional execution paths through
`as_bezier_parameter`; they must not become admission requirements. Selected
identity and recursive polynomial sign replay should use the existing common
parameter evidence instead of projecting a global eliminant for a tangent.

Keep `parallel` versus `source_parallel` until their distinct roles are handled:
a round join is centered at the unoffset source point, while the offset boundary
owns the composed support. The existing recursive fallback explicitly avoids
moving the join center twice. A simplification that merges these supports without
that distinction would repeat the earlier offset defect.

Required validation includes generated selected and recursive endpoints, reversed
traversal, both policies, round joins at source-carrier changes, and empty global
projection caches for locally decidable tangent work. Cross-only consumers should
not acquire mandatory dot/normalization work. No production edit is made here;
the current contact candidate remains subject to its unchanged qualification.
