# Incident cusp continuation audit (unimplemented)

V605 broad qualification is active (exact outer 60112); production and its mirror/archive inputs are frozen. V600 independent two-case probe remains unrun and must follow the sealed V605 commit. This audit proposes no outcome before that probe.

The original parallel can have zero derivative with nonzero rational source tangent. In the independent PH fixture, Q has a cusp at t=1, source speed 1+t^2 is positive everywhere, and the surviving t<1 tangent points down. The required numeric side is determined by the cut: previous XOR reversed retains the lower side. It must not come from the center support's orientation.

Current boundary points:

- PreparedFilletCarrier2::accepts_offset_contact (curve.rs) calls the pointwise original derivative and blocks on zero. All three production/test call sites have the cut role available.
- FilletParallelSource2::support_reverses_source_at independently compares pointwise original and center derivatives; its six production call sites also know the cut role. The center derivative must remain pointwise: a center-locus cusp does not provide a nonzero contact tangent, while the original curve's surviving side can.
- parallel_derivative_scale_sign_on_regular_range (bezier_offset.rs) already owns the exact first-nonzero Taylor coefficient theorem and asks for the endpoint side only when the pointwise derivative vanishes. Reuse this theorem rather than isolating every cusp on an incident ray.
- Native line center enumeration attaches a regular source frame only when finite incidence reports a source singularity. Original-offset cusps and all incident-ray contacts may carry no frame. Point evidence remains valid at an original-offset cusp with regular source tangent; determine whether just retaining its orientation suffices before constructing a heavier frame.
- BezierParallelDerivativeConstraint2 and its transformed polynomial selector choose original normal sheets of continuous components. Pointwise zero currently rejects a cusp. Any side proof added there must transport through swapped operands, negative affine scale and decreasing incident charts. Source poles/zero normal fields stay excluded, and finite/open boundary ownership stays with the existing parameter domains.
- regular_parallel_pair_center uses finite range endpoint sides. For an extension contact beyond those bounds, that range cannot select the surviving local side. Do not let a finite-cell orientation silently certify the entire extension.

The existing polynomial-line fixture is collinear and may use a promoted line path; it does not demonstrate a nonlinear pair path. A separate exact nonlinear cubic can preserve the independently known contact L and tangent 2(L-J): R(u)=J+2uD+u(u-1/2)^2 N, D=L-J, N=(-D_y,D_x). Its Bernstein controls are J, J+(2D+N/4)/3, J+(4D-N/2)/3, J+2D+N/4. Python Fraction verified R(1/2)=L, R'(1/2)=2D, noncollinear controls, circle incidence and tangency. Such a case is a possible later probe; it has not been compiled or run and may require a separate bounded elimination investigation.

Preserve the ordinary nonstationary fast path. A local nonzero sign is reusable proof, uncertainty is not a permanent exclusion, and a collapsed support must not acquire an arbitrary tangent or representative.
