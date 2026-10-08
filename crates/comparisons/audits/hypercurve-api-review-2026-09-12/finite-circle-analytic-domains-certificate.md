# Independent selected-circle / non-PH parallel crossing certificate

The original rational-contact fillet was materialized to an ordinary arc and did not exercise the intended worker. Its exact source/executable/result are preserved in `finite-circle-analytic-domains-native-arc-attempt1/`. The revised public fixture uses the previously exercised retained fillet from the line (-4,0)->(0,0) to P(u)=(u^2,2u), radius r=1/4, TrimOnly. The fixture explicitly checks that its circle is retained.

The selected contact u is the unique zero on [0,1] of g(u)=2u+u/(4*sqrt(1+u^2))-1/4. Its derivative is 2+1/(4*(1+u^2)^(3/2))>0. At u=1/10 the value is strictly below -1/40; at u=1/8 it is positive. Thus 1/10<u<1/8. The center is C=(c,1/4), with c=u^2-1/(4*sqrt(1+u^2)). Since sqrt(1+u^2)<=1+u^2/2<=129/128, we have -1/4<c<1/64-32/129=-1919/8256<-7/32. The retained counterclockwise arc starts at (c,0) and ends at (u^2,2u) in the lower-right circle quadrant.

The other source is A(t)=(-1/8+t^2,t), 0<=t<=1/8, with signed left displacement d=1/64. Writing s=sqrt(1+4t^2), its exact parallel is Q(t)=(-1/8+t^2-d/s, t+2dt/s). It is genuinely non-PH: its tangent norm sqrt(1+4t^2) is not a polynomial square. The native compact chart is A(v/8), v in [0,1]; the exterior chart is A(w-2), w in [2,17/8]. Their Bernstein controls are independently expanded in the public fixture. Their affine chart slopes are positive and the normal displacement is unchanged.

Throughout the retained interval, -9/64<=Qx<=-7/64, 0<=Qy<=33/256, 5/64<Qx-c<9/64, Qy-1/4<=-31/256, 0<=Qx'<=33/128, and Qy'>=1. For f(t)=|Q(t)-C|^2-1/16:

- f(0)=(Qx(0)-c)^2>25/4096>0;
- f(1/8)<=(9/64)^2+(1/8)^2-1/16=-111/4096<0;
- f'(t)<=2*(9/64)*(33/128)-2*(31/256)=-695/4096<0.

These rational arithmetic bounds were checked independently with Python fractions. Continuity and strict monotonicity prove exactly one circle contact, strictly inside the parallel's active interval, and that contact is transverse. Qx>c and Qy<1/4 select the lower-right circle branch. Qx<0<u^2 puts the contact before the retained arc's endpoint, so it belongs to the authored fillet arc, not only its supporting circle. Its natural oriented tangent cross product is positive for circle followed by parallel, since the circle tangent (-[Qy-1/4],Qx-c) crossed with Q' equals -f'/2>0. Reversing one traversal or swapping operands reverses that sign.

The expected contact count and branch are therefore independent of Hypercurve's answer. Exact returned points are replayed through both public source locations. This proof does not presume that the implementation completes the query; runtime blockers and timeouts are recorded separately.
