# Independent nonlinear endpoint fillet branch

The fixture follows O=(0,0), the line to P(alpha), the parabola
P(t)=(t^2,t) from alpha=sqrt(1/2) to 1, and the line from (1,1)
to O. The circle radius is r=1/10. This calculation concerns one
TrimOrExtend branch, independently of the implementation's fragment layout.

Let v=(1,sqrt(2))/sqrt(3) be the incoming line direction and
n=(-sqrt(2),1)/sqrt(3) its left unit normal. A circle tangent on the
parabola's left has center C=P(t)+r*(-1,2t)/sqrt(1+4*t^2).
Choose the line offset n.C=-r. The contact equation is

F(t)=t-sqrt(2)*t^2+r*(sqrt(2)+2*t)/sqrt(1+4*t^2)+r*sqrt(3)=0.

It has one root in (-1/4,-1/5). With sqrt(2) in (7/5,10/7),
sqrt(3) in (12/7,7/4), sqrt(1+4*(1/5)^2)<11/10 and speed>1,
exact rational bounds give F(-1/4)<-39/560<0 and
F(-1/5)>2/385>0. Throughout that interval,
F'(t)=(1-2*sqrt(2)*t)*(1+2*r/(1+4*t^2)^(3/2))>0.

C_y=t+2*r*t/sqrt(1+4*t^2)<-1/5, so the entire circle lies below
y=-1/10. Its line contact L=C+r*n has negative y and hence negative
line parameter. The incoming segment from O to L traverses -v; the circle's
CCW continuation matches that direction and the increasing parabola tangent
at P(t). The circle has no other parabola contact because

|P(u)-C|^2-r^2=(u-t)^2*((u+t)^2+1+2*r/sqrt(1+4*t^2)).

After the circular continuation and negative-parameter parabola span, the
path reaches O again and then traverses the full positive parabola cap.
Nonzero regularization retains the lower lobe and upper cap, meeting at O;
it must not require both components' carriers to appear in boundary loop zero.
The upper cap has negative winding and the lower lobe positive winding;
nonzero fill retains both. All circular boundary is below y=-1/10.

Exact independent witnesses for this branch are:

- P(1/2)=(1/4,1/2): upper boundary.
- (1/4,2/5): upper interior, between y=x and y=sqrt(x).
- P(-1/16)=(1/256,-1/16): lower boundary.
- (0,-1/16): lower interior, between x=y/sqrt(2) and x=y^2.
- (1/64,-1/16): exterior, to the right of that lower parabola.

The current focused candidate V19 checks normalized topology, the exact
parabola identity and fillet survival across every boundary component.
The fixed witnesses above are an additional semantic oracle to qualify after
that focused result. No inference is made from fragment order or enum spelling.

Correction after V20: the negative-contact branch above violates finite
incoming-chord cut admission because L lies before O. It is not a required
fillet result. The admissible independent oracle and preserved rejection
analysis are in endpoint-fillet-normalization-20260924-v2.md.
