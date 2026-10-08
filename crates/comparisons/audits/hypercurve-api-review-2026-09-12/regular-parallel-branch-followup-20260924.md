# Regular parallel branch replay (read-only follow-up)

The V12/V13 quartic stack reaches parallel_derivative_scale_sign_from_polynomials
at a local contact. parallel_intersections_on_regular_range currently computes
one strict interior sample, but reuses its derivative-scale sign only when a
regularized source tangent frame exists. Ordinary source frames repeat the
higher-degree squared cusp polynomial query at each selected contact.

A possible next step is to consume an existing regular-branch certificate. With
no pole, source singularity or parallel cusp on a connected closed range, the
continuous scalar multiplying the homogeneous source tangent is nonzero and has
one sign throughout that range. One exact interior evaluation then owns every
contact's derivative orientation. Existing singularity_analysis can certify
these conditions without joining endpoint coefficient fields. If regularity
is not already carried by the range, a bounded optional analysis can authorize
reuse and leave per-contact replay intact when it declines. Simply assuming
constant sign from one sample would be unsound on a range crossing a cusp.

A concrete adversarial oracle uses P(t)=(t,t^2), left offset d=1 and the horizontal
chord (-1,1) to (1,1). On [0,1] the contacts are t=0 and
sqrt((7-sqrt(17))/8). The first has negative derivative scale and zero cross;
the second has positive scale and positive cross. A cusp lies between them.
This follows from u=t^2 and u*(4*u^2-7*u+2)=0 plus the positive-speed sheet
condition 1-u>0. The regular interval [1/2,1] owns only the second contact and
has positive scale. Reversal must flip both chord tangent signs. This example
can test both certificate admission and refusal independently of representation.

No production change for this proposal has been made. Native affine-chart
identity and duplicate source-axis membership remain separate open hypotheses.
