# Shared overlap evidence and restriction

Committed Hypercurve `dfc8ff21c6f86f4feb8e75f90da010cd5c0f3bf2`. Qualified against the documented regression baseline. The original architecture goal remains active.

## Corrected authority boundary

Region publication previously copied clipped ranges and orientation but discarded `RegionPairOverlapSource::Correspondence` and `AlgebraicChordRational`. Its optional public `source()` described whether the native Bezier dispatcher had produced the overlap, rather than whether the result retained exact transport evidence.

`CurveRegionIntersectionOverlap2` now owns the same `CurveIntersectionOverlap2` used by common curve and path reports. Its `overlap()` exposes the active paired domains, endpoint inclusion and public restriction operation. The region-only overlap struct, source enum, native-only accessor and duplicated range/orientation accessors are removed. Production consumers and regression callers are updated directly.

Correspondence is mandatory in the common overlap value. Native lines and retained injective lineage carry their certified affine transport in the existing projective authority; native circular overlaps retain rational chart correspondence. Region chord/chord, chord/rational and analytic component producers retain the corresponding existing kernel authority. The source chart of a correspondence is never replaced with its clipped output domain.

`CurveIntersectionOverlap2::restrict` takes two pairs of exact local bounds. It intersects both limits with the currently active component before transporting cuts, retains traversal and original open endpoint ownership, and cannot widen an earlier restriction. Singleton intersections return no positive-length component; this operation does not replace isolated-contact discovery for open curve intersections. Unchanged domains clone existing evidence without replaying a map. No compatibility interface is added.

## Shared finite chord inverse

The shared chord/rational correspondence now clips either operand using its certified monotone source branch. The inverse first preserves original endpoint and zero-distance source-point identities. It reuses retained selected/recursive parameters when the source and zero displacement prove the same construction, then uses existing incidence or finite-domain local root isolation for other cuts. Exact represented target points can isolate the unique inverse over the actual finite source range, including exterior intervals. A region-only forward adapter and the older common source-only adapter are removed.

Three focused regressions cover nonlinear and reversed maps, finite exterior inverse roots, and eight repeated restrictions retaining an unprojected degree 135 selected parameter. Two new native regressions cover distinct line/arc charts, traversal, original open endpoints, singleton/disjoint clipping and repeated wider limits. Existing selected nonlinear and published region chord regressions now exercise public restriction and evidence lifetime after source contexts are dropped.

## Qualification record

The first focused inverse run passed all three new tests. The first compile migration found retired accessors, followed by explicit-conversion and removed-getter errors; all are corrected. Attempt 5 built all 49 Hypercurve/HyperBREP targets and passed the six focused filters (nine test cases). Its exact sources, build logs and focused results are retained. The final API accepts endpoint pairs directly and adds explicit singleton-limit coverage. Its separate qualification has 2,227 passes (1,995 Hypercurve and 232 HyperBREP) across 49 targets, the same five known failures, nine ignored tests and eight previously unqualified expensive exclusions, with no new failures or timeouts. Both all-target checks, formatting, whitespace and all 396 frozen source hashes pass. Locked offline Hypercurve fuzz/UI and HyperBREP fuzz checks pass; a tracked-source audit of the other 29 repositories finds no additional users of the retired region-overlap API.

Historical public probes remain unchanged. The region carrier matrix has a separately named copy adapted to `overlap()`. A new independently compiled replay matrix exercises public restrictions from new intersection parameters, repeated wider limits, singleton cuts, both operand orders and reversed traversals, including region reports used after their input regions are dropped. It certifies 420 intersection queries, 60 restrictions, 160 point replays and 160 repeated restrictions. All eight preserved earlier probes/matrices also pass against the final normal-library SHA `ac087dcc452fada528c2564ba2c289aa0b5bd7d1eeeabc4b5e9db6e2ce224d75`.

## Remaining original goal

This migration does not establish all finite exterior or retraced-domain intersections, normalized public region construction, the five existing promotion failures, the eight previously unqualified expensive tests, or the broader shared algebraic authority and computational-closure work. No general performance, allocation or memory improvement is claimed. Qualification remains relative to the recorded regression baseline until those outstanding cases are resolved.
