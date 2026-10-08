# Parameter construction cleanup, V588 (unrun)

Do not edit production during V587. First reap exact outer 99449, qualify and
commit the current generated-point incidence repair. The full implementation
goal remains active. This is a subsequent coherent API cleanup, not permission
to mix edits into the live source snapshot.

Remove BezierParameter2::exact and ::algebraic and migrate every caller directly
to Exact and Algebraic variants. The existing public enum already admits both
values; the exact wrapper alone imposed a hidden [0,1] admission rule. No
compatibility alias or replacement forwarding API is needed. Updated inventory
is parameter-construction-uses-v588.json. Also audit the three Self::exact
importer calls and any helper policy parameter made unused by the migration.

Preserve unit admission at the owning operations:
1. real_coefficient_rational_image_parameter: after denominator certification,
   explicit in_closed_unit_interval -> Some(Exact), None or Ordering uncertainty.
2. exact_rational_parameter_image: same check only when unit_domain is true.
3. supporting_line_contact_evidence_impl: finite line parameter is optional,
   so reject exterior parameters as None; affine witness stays unbounded.
4. Root importer: exact-point, represented upper-endpoint and deflated linear
   branches each keep current native-unit validation. A local admission closure
   belongs to this importer and can share those three identical checks. The
   nonlinear interval check already exists and must remain.

Tests/benches/fuzz values are already authored in [0,1]. Replace their wrapper
unpacking and impossible uncertainty branches, and remove policy-only helper
arguments when now unnecessary. Do not merely capitalize the name and leave
Result/Classification wrappers around an enum constructor. All three affected
fuzz targets must compile (bezier_arrangement, bezier_region,
bezier_split_materialization), plus all-target feature configurations.

Strengthen the existing unit-import regression for exterior exact point,
owned upper endpoint and deflated linear root, including negative/exterior
values under both policies. Preserve the current generic root importer for
arbitrary exact values. Existing public image/domain/pole regressions and
full operation-composition coverage remain relevant.

Correct Hypersolve AlgebraicRootRepresentation.interval documentation from
'Certified unit isolating interval' to a finite isolating interval. This
owning-layer representation already supports exterior and exact-Real roots;
no implementation or compatibility change is required there.

V594 unrun fixture is represented-parameter-domain-tests-v594.rs. It covers
exact point, owned upper endpoint and deflated linear imports for -1, 2,
-sqrt(2), sqrt(2) and pi/4 under both policies. The native importer must
admit only the selected value in its actual unit domain; an exact interior
root may be recovered even when an upstream interval extends outside that
domain. This fixture has not been built or run and must first prove valid
on the current baseline before it is used to judge a constructor migration.
