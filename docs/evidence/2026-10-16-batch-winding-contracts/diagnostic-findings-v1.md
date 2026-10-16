# A triangulation disagreement caused the apparent winding contradiction

The pre-edit libraries at `f74a7848805398eb62dddb167b3997662e6f90d5`
reject both the original 26-operand union and its checked merged-shell form.
The retained binary accumulation succeeds with exact volume 124. The diagnostic
exit codes record successful reproduction, not successful batch geometry.

`reduce_v1.rs` retains the original case and finds a deletion-minimal five-box
case with source indices `[12,17,20,21,24]`. Each single deletion succeeds;
the five-box batch and merged-shell forms both fail. Its independently derived
volume is 46: horizontal section areas 14, 16 and 8 over heights 1, 1 and 2.
This is deletion minimal for that reduction, not a proof of a globally smallest
counterexample.

The temporary traces in `diagnostic-v1` and `diagnostic-v2` locate a cycle that
joins opposite sides of source face 45 without its winding transition. The
first inconsistent merge is 640, joining nodes 690 and 492 at edge `[23,54]`.
Node 690 crosses sheet 345 to 691 with winding delta +1; merge 639 joins 691
to 495, and merge 613 joins 495 to 492. `analyze_trace_v1.py` retains this
cycle. The contradiction is real in the assembled topology; ignoring it would
publish an unjustified result.

Three coincident faces in the x=3 plane have the same protected cell but two
different valid triangle sets. In their yz projection, P=(4,2), T=(4,1),
Q=(3,1), R=(4,4/3) and S=(17/5,8/5), with boundary P-R-T-Q-S-P.
Faces 33 and 57 use a Q-centered fan. Face 45 uses an S-centered fan. Scaling
by 30 gives the exact integer fixture in `fixtures/common-cell-v1.json`.
Both fans are local minima for the old decreasing single-flip rule: one flip
would pass through the mandatory collinear vertex R; the other first increases
the endpoint pair. The rule therefore did not establish the canonicalization
property on which coincident-sheet bundling depends.

All temporary mesh tracing was removed before `triangulation-baseline-v1`.
The private five-point and public three-face regressions both fail against the
unchanged Hypertri algorithm in that phase. The repair belongs in Hypertri's
topology-only constrained triangulation, using its existing exact constraint
recovery to protect admissible diagonals in endpoint order. Hypermesh's winding
propagation and consistency rejection remain unchanged. The separate reasoning
record explains the combinatorial invariant and its assumptions.

`candidate-focused-v1` passes both triangulation regressions but retains a
test-compilation failure: the new mesh assertion initially used `min`/`max`
instead of the existing `mins`/`maxs` bounds fields. Correcting those field names
does not change production behavior or acceptance requirements.
`candidate-focused-v2` passes all five commands, including both native box
regressions, the expanded independent triangulation oracle, the public modeling
composition regression, and formatting. Broader qualification and resource
evidence are recorded separately; this diagnostic record does not establish
release readiness, internal resource bounds, or the full properties plan.
