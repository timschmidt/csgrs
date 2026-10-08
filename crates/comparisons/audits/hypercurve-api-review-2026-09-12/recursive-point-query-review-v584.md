# Retained point incidence review (unrun follow-up, V584)

V583 production and archive are frozen until exact outer 18886 is reaped.
Three newly selected test names use the wrong module; the functions actually
belong to conversion_tests. The replacement focused driver must use the actual
post-build --list output before invoking exact test names.

The production repair uses the existing Cartesian theorem in the point field:
delta=(X*W-Nx*D, Y*W-Ny*D). For d != 0, gcd(delta dot T,
delta dot delta-d^2*W^2*D^2) owns all unsigned contacts; the sign of
(delta_y*Tx-delta_x*Ty)*d*W*D chooses the authored normal. Candidate roots
retain the GCD authority without rebuilding either defining equality. For
zero d, gcd(delta_x,delta_y) needs no source normal. An empty GCD denotes
both polynomial identities, never a coprime pair. Whole-domain incidence
proves nonzero W and speed on finite and incident components before using
continuity of the nonzero normal orientation. Isolated contacts check their
own speed; unrelated stationary parameters do not invalidate them.

Follow-up strengthening to consider after V583 is reaped:
- Run the three new conversion_tests cases under their actual names.
- Add a quintic selected source t^5+t-1 to the independent oblique chord
  point fixture, in addition to the quadratic root, to exercise a field
  without a simple radical scalar representation.
- Reuse strict_tangent_chord_query_point for an independently known (1,0)
  AlgebraicCuspChord query on a simple parallel.
- Reuse selected_fiber_rational_quarter_overlap for a mapped derived point;
  compare its incidence against the source quarter at zero displacement.
- Include all existing recursive polynomial replay/isolation regressions,
  since the root enumerator now consumes finite or incident domains.

The existing native algebraic-image and Cartesian fast paths remain unchanged
apart from the shared source-domain predicate. Any subsequent consolidation
must compare exact output and total time, not just count code lines.
