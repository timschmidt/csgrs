# Pending selected-chamfer monotonicity investigation

The homogeneous-composition work is committed as Hypercurve `5af1dfe` and
`07cb100`. A separate v1 candidate now uses opposite certified endpoint signs to
decline the optional monotonicity proof and evaluates the final Bernstein control
directly at the endpoint. Its focused qualification is terminal: both all-target
Clippy feature configurations pass with warnings denied, and four focused tests
pass. The original selected-chamfer and extended-fillet cases still time out at
75 seconds. All owned candidate processes were reaped. V1 is not qualified for
commit until the separate domain defect below is resolved.

The existing `nonlinear_selected_corner_chamfers_through_retained_fixed_distance_image`
test constructs a nonpromotable selected endpoint on the zero-distance parallel
of P(t)=(2t²,t), adds a vertical chord and a closing chord, then chamfers by 1/10
under both policies. It requires a certified local selected-fiber cut and explicitly
rejects the global-center fallback in the fixed-distance kernel. The current
75-second timeout is on the pre-existing nonpass list.

The bounded GDB capture uses the immutable final homogeneous-composition libtest
SHA a2f463f8739237b40b0a8364e687a3a004482af52c16a579e5ed51a72b41927c.
Its terminal report and bounded stack are in
`nonlinear-selected-chamfer-20260924-stack1-{terminal.json,log}`. The debugger
killed and reaped its own inferior. At eight seconds the test is in:

```
chamfer -> with_corner_chain_replaced -> unary regularization
-> algebraic_chord_parallel_pair_result
-> parallel_tangent_cross_sign_on_region_range
-> recursive_projective_endpoints_with_direction
-> recursive_projective_point
-> promoted_bezier_parameter_complete
-> bivariate resultant -> Bareiss determinant
```

Thus the sampled work is an optional monotonicity certificate requesting complete
global point projection. It is not yet evidence of a defect in Bareiss arithmetic
or the local fixed-distance solver. The stack does not identify which incident
chord was selected, so that detail remains unproven.

The monotonicity helper already constructs conservative degree-at-most-two
Bernstein cross controls and certified endpoint signs before attempting complete
point-field projection. A possible improvement is to decline strict monotonicity
as soon as the first and last **endpoint values** have opposite certified signs.
Continuity then proves an interior zero. Interior Bernstein control signs alone
would not justify this conclusion. The authoritative incidence solver must still
run; declining this optional certificate must not exclude a curve or contact.

Its last Bernstein control is currently expanded as q0+q1+q2, using interval
delta=end-start and correlated repeated parameters. Evaluating the same polynomial
directly at the endpoint interval is a smaller valid enclosure and avoids the q2
temporary. This algebraic identity should be checked alongside the opposite-sign
guard, with reversal, endpoint-zero, and same-sign controls. The original selected
chamfer regression must retain all geometry, candidate, exactness, and local-field
assertions. No reduction of the supported domain or added projection limit is
authorized by this hypothesis.

## Domain review before accepting v1

Code review of the remaining fallback found three authored-unit assumptions:
the represented coefficient Bernstein proof, the recursive-field Bernstein
proof, and the fallback recursive-polynomial root enumeration all use `[0,1]`
even though this predicate accepts a general finite retained range. All three
must respect the queried range. This needs a direct executable counterexample
and a coherent repair before v1 can be qualified.

An independent exact-fraction calculation supplies the candidate counterexample:
P(t)=(t,15t/4-2t²+t³/3), chord direction (1,0). The tangent cross is
Q(t)=15/4-4t+t²=(t-3/2)(t-5/2). Its unit Bernstein controls
`[15/4,7/4,3/4]` are all positive. On the extended range `[1,3]`, however, its
controls are `[3/4,-5/4,3/4]`, it has two interior roots, and Q(2)=-1/4 while
both endpoint values are 3/4. Thus endpoint-sign opposition alone cannot detect
this case. A unit proof cannot certify strict monotonicity on the extended range,
and enumerating only unit roots would miss both relevant roots. The exact
calculation is retained in `nonlinear-selected-chamfer-20260924-exterior-polynomial.json`.
This calculation establishes the mathematical witness. The control executable
then confirmed the defect against unchanged committed production code: the new
test failed with `Decided(Positive)` for `[1,3]`. It completed in 0.274 seconds
including process startup. Its SHA is
4483e90fe8e118800bce87e8a81903c3d3c13dbfe92032df5460659694a2759a.
`extended-monotonicity-20260924-control-terminal.json` records return code 101,
unchanged source bindings, and reaped processes.

## V1 remaining sampled cost

`nonlinear-selected-chamfer-20260924-stack2-{terminal.json,log}` binds a second
eight-second stack to the v1 executable SHA
ee6e84eb3d6482dba5e080b8b4cf8366eca8d3fe38a7db52ac56fc659604c51b.
The optional monotonicity call is absent from this stack. The authoritative
chord/rational intersection kernel now requests recursive projective endpoints,
which still promotes the selected analytic point through the complete global
parameter resultant. The debugger killed and reaped its own inferior. This is
progress past one unnecessary projection, not a completed selected-chamfer fix.

The isolated parent control was built from committed `07cb100` production
bytes with only the new retained-range regression added. Its source manifest is
`extended-monotonicity-20260924-control-sources.json`; the exact test addition is
`extended-monotonicity-20260924-regression.rs`. Existing immutable snapshots and
workspace production sources remained unchanged throughout that build.

## V2 domain repair under qualification

Both unit-Bernstein accelerators now require the existing exact finite-domain
containment certificate. The recursive root fallback receives the actual
retained range. Its existing isolation authority already handles arbitrary
finite ranges and preserves original parameter identity; no extra chart or
coordinate projection is introduced. The trace label now describes a retained
range sign rather than claiming every answer came from unit Bernstein controls.

The regression checks the two-interior-root witness, positive unit and exterior
subranges, a negative subrange, and the range with roots at both endpoints. It
checks both range orientations, both chord directions, polynomial and positive/
negative homogeneous presentations, and both policies. These cases distinguish
strict interior-root exclusion from permitted endpoint tangency.

The two opposite-endpoint and extended-range regressions plus 231 existing cases
are selected for v2 qualification, including all five public Boolean/arrangement/
path/PCB suites used by the preceding change. Both all-target Clippy feature
configurations passed with warnings denied. Release tests remain pending.

## Qualified and committed range repair

Hypercurve `20f1580` contains the coherent range repair and opposite-endpoint
guard. All 233 selected release attempts are terminal: 230 pass, zero are ignored,
and exactly three match the existing baseline nonpasses (the selected-chamfer and
extended-fillet 75-second timeouts, and the PH corner test's obsolete one-interval
storage assertion). All 121 public integration tests pass. The two new tests pass
in 0.016 and 0.004 seconds including startup. Both all-target Clippy configurations
and the modified file's Rustfmt check pass.

The 2,044 source inputs, copied executables, unchanged parent counterexample,
candidate runs, staged content, and committed HEAD are bound by
`extended-monotonicity-20260924-{qualification,staged,post-commit}.json` and their
referenced artifacts. Every owned process was reaped before staging. All thirty
workspace repositories are clean; nothing was pushed. This fixes a demonstrated
certificate correctness defect, not the outstanding selected-chamfer performance
problem. The full implementation goal remains active.
