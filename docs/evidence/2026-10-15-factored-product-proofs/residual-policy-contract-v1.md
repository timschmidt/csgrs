# Dense residual replay acceptance

`qualification-v1/foundations.stdout` reproduces an outdated expectation in
`dense_replay_obeys_terminal_equality_policy_and_retains_certainty`: the original
two-row system now returns exact zero residuals under `STRICT`, where the test
expected `UnknownResidual`. This is not yet attributed to E12: replay the same
public API against preserved E11 and current coherent dependency graphs first.

The owning implementation layers remain Hyperreal's symbolic identities and
Hyperlimit's policy resolver. Hypersolve owns the report's row and aggregate
certainty. Relevant properties are P2/P3/P4/P8/P9/P35/P39. No solver production
change is proposed.

For the original matrix `[[e, pi], [pi, e]]`, right hand side `[1, 2]`, and
candidate `[(e-2*pi)/(e^2-pi^2), (2*e-pi)/(e^2-pi^2)]`, ordinary distribution
gives numerators `e^2-pi^2` and `2*(e^2-pi^2)`. Both residuals are exactly zero.
The denominator is nonzero since `e < 3 < pi`. Retain this complete original
fixture, requiring exact acceptance under both policies, including fresh
approximate-policy inputs and strict/approximate/strict reuse.

Retain terminal-equality coverage with the existing Hypersolve test helper
`terminal_zero`: `sin(e)^2 + cos(e)^2 - 1`. Its zero value follows independently
from the trigonometric identity, outside the admitted polynomial relationships
between opaque sine/cosine atoms. Replay it alongside an exactly rational zero
row in both orders. Require strict `UnknownResidual`, approximate zero with
approximate certainty on exactly the terminal row and aggregate report, and
strict `UnknownResidual` again after approximate reuse. This is additional
coverage, not removal or simplification of the original system.

The standalone probe is externally contained by the recorded runner. It does
not establish a total-work/heap bound or constitute a benchmark. After the test
change, rerun the affected tests and complete downstream qualification on a
fresh independently copied target; retain the failed qualification unchanged.
