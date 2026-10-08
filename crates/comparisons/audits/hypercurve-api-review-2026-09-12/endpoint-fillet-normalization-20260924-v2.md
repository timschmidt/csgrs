# Admissible exterior endpoint fillet branch

The first oracle selected a negative contact whose line contact lies before O.
That branch is not admissible: algebraic_chord_corner_cut_from_support_point
admits trimming inside the finite chord and extension only beyond the edited
end. V20 correctly has no result matching that oracle. The earlier derivation
is retained as a rejected test assumption, not a production failure.

For P(t)=(t^2,t), alpha=sqrt(1/2), r=1/10, use instead the circle
on the left of both incident supports. With v=(1,sqrt(2))/sqrt(3) and
n=(-sqrt(2),1)/sqrt(3), C=P(t)+r*(-1,2t)/sqrt(1+4*t^2) and n.C=r.
The exact contact equation is

F(t)=t-sqrt(2)*t^2+r*(sqrt(2)+2*t)/sqrt(1+4*t^2)-r*sqrt(3)=0.

The adjacent Fraction calculation proves F(69/100)>0 and F(7/10)<0.
F'(t)=(1-2*sqrt(2)*t)*(1+2*r/(1+4*t^2)^(3/2))<0 there, so the
contact is unique. t<7/10<alpha extends the outgoing incident end.
The line contact is L=C-r*n. Certified rational bounds give L_y>71/100>alpha,
so the incoming chord extends beyond its edited end, too.

The circle is exterior to the parabola and has no other parabola contact:

|P(u)-C|^2-r^2=(u-t)^2*((u+t)^2+1+2*r/sqrt(1+4*t^2)).

The CCW circle continuation is the major arc because the incoming line tangent
angle exceeds the outgoing parabola tangent angle. The original clockwise
region survives, and the exterior CCW lobe meets it at P(alpha). Both windings
are nonzero; the boundaries need not be stored in the same loop.

Five exact witnesses identify and certify this admissible result:

- (9/20,3/4) lies strictly inside the new disk: both coordinate differences
  from C have absolute value <1/25, so squared distance <2/625<1/100.
- P(7/10)=(49/100,7/10) is on the exposed added parabola boundary.
- P(3/4)=(9/16,3/4) remains on the original parabola boundary.
- (1/2,3/5) remains inside the original region.
- (197/400,7/10) lies in the exterior gap between x=y^2 and x=y/sqrt(2).

The exact Fraction bounds are in endpoint-fillet-normalization-20260924-v2-algebra.json.
These witnesses replace the inadmissible negative-contact oracle; all previous
support, fillet, normalized topology, policy and candidate assertions remain.
