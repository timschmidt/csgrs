# Independent finite fillet certificate

Let P(t)=(-1/8+t²,t), S=1+4t², N=(-1,2t)/sqrt(S), d=1/64 and b=1/8. The four frozen charts in `finite-selected-point-domains-public.rs` describe Q=P+dN for 0<=t<=b, exactly as in the fixed-distance fixture. The rational exterior chart has t=(s-2)/(2s-1), s in [2,5/2], with a genuine unused source pole at s=1/2.

Set r=1/32, C=(-9/64,-r), and A=(-11/64,-r). A clockwise quarter-circle of radius r about C runs from A to Q(0)=(-9/64,0), with a rightward terminal tangent. Append Q, whose initial tangent points upward. Request a trim-only fillet of radius r at this corner.

For the left-offset candidate, the first circle's offset has radius 2r about C. The second offset is F=P+(d+r)N=P+(3/64)N. Write Delta=F-C. On [0,b],

- Delta_x=t²+1/64-3/(64 sqrt(S)) >= -1/32=-r;
- Delta_x <= 1/32-3/(16 sqrt(17)) < 0, since sqrt(17)<6;
- Delta_y=t+3t/(32 sqrt(S))+r >= t+r;
- F'=h(2t,1), where h=1+3/(32 S^(3/2))>0.

Thus (|Delta|²)'=2h(2t Delta_x+Delta_y) >= 2h(15t/16+r)>0. At t=0, |Delta|²=2r²<4r². At t=b, Delta_y>=b+r=5/32>2r. There is exactly one selected t in (0,b) with |F-C|=2r.

Its contact on the original circle is (F+C)/2. Delta has negative x and positive y, so this contact lies strictly inside the authored clockwise quarter-circle. The contact Q(t) lies strictly inside the other finite curve. The positive dot product (2t Delta_x+Delta_y) also orders the corresponding tangent directions: the connecting counterclockwise fillet has a positive sweep strictly below pi/2. Both contacts preserve the source traversal directions.

For the other handedness, the first offset collapses to C. The second is G=P+(d-r)N=P-(1/64)N. Its y-coordinate is t(1-1/(32 sqrt(S))) >=0 throughout [0,b], whereas C_y=-r<0. C is not on this finite offset. Consequently this branch contributes neither a discrete solution nor a coincident family, and the requested fillet has exactly one solution. Reversing the complete path preserves the same geometric construction with reversed traversal.

These count and domain conclusions come from the equations above, independently of any implementation output. The executed parent baseline separately records native/exterior chart behavior. This certificate does not claim the next implementation or full closure goal is complete.
