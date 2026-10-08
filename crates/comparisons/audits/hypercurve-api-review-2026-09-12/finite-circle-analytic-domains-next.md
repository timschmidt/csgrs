# Finite selected-circle / analytic-parallel domains

Audit of the candidate after finite ordinary analytic pairs. The executed public baseline is now recorded in `finite-circle-analytic-domains-baseline.json`: 16 native cases complete and 16 equivalent exterior cases return Unsupported. The retained-circle fixture and its independent transverse-contact proof are in `finite-circle-analytic-domains-certificate.md`. The earlier rational-contact construction below produced an ordinary arc and remains archived only as an unsuccessful fixture attempt.

The remaining Pair::circle_parallel unit_domain_covers guard cannot simply be removed. It optionally maps rational PH images through the range-aware rational-circle authority, then calls parallel_intersections_in_range only when endpoints fit BezierParameter2; selected endpoints otherwise fall back to the native whole-source worker. The new common finite pair authority does not own selected-circle coefficient fields and should not replace them with independently materialized scalars.

The existing private selected-circle interface has three wrappers (parallel_intersections, parallel_intersections_in_range, parallel_intersections_with_incident_ray) over parallel_intersections_with_domain(range: Option<&BezierParameterRange2>, incident: Option<...>). Several lower workers inherit that optional/narrow range. They should share the explicit finite CurveParameterRange2 and existing optional extension domain, preserving the native unit interval as a caller-supplied value, rather than adding another exterior-only wrapper.

Concrete range-losing sites from source inspection:

- parallel_parameters_with_incident_domain always enumerates SelectedThirdAxisDomain2::Finite(unit) and only afterwards clips to its optional range. This shared helper schedules represented, recursive and selected coefficient-field projections and is a useful consolidation point.
- parallel_intersections_with_domain projects its selected-root and shared-cusp-witness paths on CurveParameterRange2::unit even when range is Some(exterior).
- The same worker's conservative-bounds early rejection uses other.conservative_bounds without the active range. It must require range coverage or reuse the finite bounds authority.
- Finite rational materialization uses exact_rational_parallel_component (native normal sheet) in several branches; it must own the consumed range before substitution.
- Exact-center finite circle_incidence and source_circle_incidence use native-domain root discovery. Incident-ray variants add exterior roots but do not automatically make an arbitrary finite range complete.
- CurveParameter2 selected endpoints must retain their policy/evidence identity through optional rational attempts and lower projection; passing a fresh strict counterpart can discard that authority. Use strict predicate scope with the original policy for retained geometric evidence.

Build a public independent baseline first: a genuinely non-PH parabola parallel with known tangent contact at its vertex, in both exterior and equivalent native charts, intersected with selected circle arcs. Cover both orders, reversals, both policies, native/exterior poles, selected endpoints, tangency and transverse branches. Preserve public location replay and overlap correspondence; add Boolean/trim reentry once the ordinary pair authority is qualified. Rational PH-only success is insufficient evidence for this stage. One-sided source-cusp endpoints remain a separate frame requirement and must never be silently dropped.

Dependency builds stay pinned. The other session's Hyperreal sources and verification files remain outside the edit/stage/commit scope.

## Shared projection migration — 72d831c46241

The scalar projection layer now consumes `CurveParameterDomain2` directly. Seven optional/narrow finite-range plus geometric-incident signatures are replaced by that existing borrowed finite/ray domain; represented, recursive, cached-univariate and direct-pair workers share it. The common scheduler isolates the requested finite range, reuses the projected equation on the optional ray, and gives the finite interval ownership of shared roots. The old unit-root allocation/copy/filter pass is removed. The three upper geometric workers still convert their current narrow optional ranges at the boundary; their migration, the selected-circle entry-point consolidation and the public guard removal remain unfinished.

The common selected-fiber interval worker now performs exact isolation and clipping in a strict predicate scope while retaining the original context for endpoint replay. Both defects were reproduced against parent `8228394f174be2ae607694682defb167f030f540`: the old scheduler found zero roots instead of three in the first exterior factored-polynomial case, and the old selected-fiber worker rejected a boundary whose policy came from an earlier terminal in the same composite operation. See `finite-circle-projection-parent-results.json`. This bounded implementation step is qualified with 2,296 passes, eight focused checks, 34 passing public probes, unchanged known failures/exclusions and verified source/artifact hashes. The separate public circle matrix still has 16 native successes and 16 exterior blockers; no public exterior-circle completion claim follows from this step.

The existing `CurveSupport2::certified_outer_bounds` already provides range-owned analytic bounds, including unused native poles. The next circle entry should reuse that authority or move its common parallel implementation lower, rather than duplicating another finite-bounds algorithm. Positive-dimensional mapped overlap storage and inverse replay still use narrow ordinary endpoints and require a separate careful migration; avoid turning inability to promote selected endpoints in an optional rational shortcut into an invalid-input error.

## Independently derived non-PH circle tangency fixture

An unexecuted exact fixture for the next baseline is P(t)=(t,(t-2)^2), signed distance -1/4, restricted to t in [3/2,5/2]. Its equivalent native source is P(3/2+u). The displaced vertex is V=(2,-1/4), tangent horizontal. Use a selected circle with center C=(2,3/4), radius 1, and a retained half/arc that contains V.

Write v=(t-2)^2 and s=sqrt(1+4v). Then Qx=2+(t-2)*(1+1/(2s)), Qy=v-1/(4s), and |Q-C|^2-1 = v^2-v/2-3/8+(v/2+3/8)/sqrt(1+4v). It is zero at v=0. For 0<v<=1/4, sqrt(1+4v)>=1+3v/2; multiplying the resulting upper bound by its positive denominator gives v*(3v^2/2+v/4-9/16), which is strictly negative throughout this interval. Thus the unique supporting-circle contact in the requested range is the exact tangent visit t=2. Do not reuse the wider [1,3] pair fixture: it also has two transverse circle crossings. The public selected-circle construction should reuse an existing generated-frame fixture, not bypass the public API with private constructors.

To exercise the native-bounds premise as well, shift the fixture's vertex to t=3: P(t)=(t,(t-3)^2), restricted to [5/2,7/2], circle center (3,3/4). Its native control points are (0,9),(1/2,6),(1,4). The unused native span lies above the circle, so its native bounds can incorrectly reject the finite tangent contact if the coverage premise is removed. The same v=(t-3)^2 proof applies. This stronger fixture remains unexecuted.

## Public construction candidate for the selected circle

Reuse the existing public fillet route rather than private semicircle constructors. Before scaling, the path is the line (-10,0)->(0,0), followed by P(t)=(t^2,2t), t in [0,1]. Radius 15/4 has its next contact at t=3/4: P=(9/16,3/2), unit tangent (3/5,4/5), and left center C=(-39/16,15/4). The incoming contact is (-39/16,0). The center's y-coordinate equation 2t+(15/4)*t/sqrt(1+t^2)=15/4 is strictly increasing, proving uniqueness on the retained next span.

Apply x'=4x/15+73/20, y'=4y/15-1/4 to the authored inputs directly, then request a radius-one TrimOnly fillet. All controls are rational:

- incoming line (59/60,-1/4)->(73/20,-1/4);
- next quadratic controls (73/20,-1/4), (73/20,1/60), (47/12,17/60).

The expected circle center is (3,3/4), incoming tangent contact (3,-1/4), and next contact (19/5,3/20). The generated arc contains the unique analytic-parallel tangency derived above. Inspect the returned circle carrier before treating this as evidence for the selected-circle guard: exact rational contacts might be materialized by a lower shortcut. This construction was subsequently executed and materialized to an ordinary native arc; see `finite-circle-analytic-domains-native-arc-attempt1/`. The retained-circle transverse fixture in the independent certificate replaces it for this migration. The arithmetic recipe was derived independently, not taken from Hypercurve output.


## Geometric entry migration — 3eabc047e4e8

The explicit generic finite-range migration, one selected-circle/parallel entry, native shortcut coverage premises, public exterior guard removal, generic mapped overlap storage and finite inverse replay are committed and qualified. All 32 independently derived public polynomial cases pass, including the 16 exterior cases that the parent blocked. See `finite-circle-domains-worklog.md` and `finite-circle-domains-qualification.json`. Lower equation builders still assume whole-native source finiteness/regularity; `finite-circle-native-poles-next.md` records that next audit. This stage does not complete general selected-circle/rational-source or region-operation closure.
