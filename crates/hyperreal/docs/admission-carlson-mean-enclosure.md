# A tighter certified Carlson RF enclosure

Owner: Hyperreal. Properties P1/P3/P5/P20/P35/P36. The complex Jacobi
consumer `[21,6,127,39,1,4]` exceeds the existing ten-second ASan input
limit while evaluating Carlson RF inside inverse-amplitude refinement.
Replacing duplicate integer division alone leaves the diagnostic replay
above that limit. Keep the complete workload and containment limits.

The proposed change tightens the bound on the remaining RF integral during
duplication. It neither truncates the represented function nor changes its
domain. RD retains its existing bound. Input enclosures, directed rounding,
cancellation, grid restarts and the final output-width check still apply.
The missing acceptance is a sharper-bound regression with independent
expectations, followed by the original sanitizer workload.

## Derivation

Starting from the [defining RF integral](https://dlmf.nist.gov/19.16.E1),
let A=(x+y+z)/3, delta_i=1-x_i/A, r=max |delta_i|<1 and
G=sum(delta_i²)/(4(1-r))<1. Substitute u=A/(A+t). The normalized integrand
is F(u)=product(1-delta_i*u)^(-1/2), with weight u^(-1/2)/2 on [0,1].
The deltas sum to zero. Concavity of log gives F>=1. The absolutely
convergent logarithm series gives

    0 <= log F(u) <= G*u².

Indeed each term beyond the canceled linear term is bounded in absolute
value by delta_i²*u²/(4(1-r)), using 1/n<=1/2 for n>=2.
For 0<=v<=G<1, exp(v)<=1/(1-v), hence

    1 <= F(u) <= 1 + G*u²/(1-G).

The weight integrates to one and its u² moment is 1/5. Therefore

    1/sqrt(A) <= RF(x,y,z)
               <= (1 + G/(5(1-G)))/sqrt(A).

This is a bound on the entire integral, with quadratic dependence on the
relative spread. It is not a fixed-order correction treated as exact.

## Integer implementation and acceptance

For integer grid coordinates L_i with physical scale 2^q, set S=sum L_i,
D_i=|3L_i-S|, D=max D_i, N=sum D_i² and H=4S(S-D). Use the sharper
upper bound only if D<S and N<H. Then G=N/H and the correction factor
is (5H-4N)/(5(H-N)). Also 1/sqrt(S/3)=3/sqrt(3S), so integer square
root floors/ceilings and directed division suffice. No float conversion is
introduced. RF decreases in each input: evaluate the lower bound using the
three upper input endpoints and the upper bound using the lower endpoints.
If the sharper bound is unavailable, retain the existing min/max bound.
RF evaluates these stopping enclosures every fourth duplication step to
amortize their integer square roots and divisions. This is a work schedule;
every successful return still requires the full enclosure-width certificate.
RD keeps its existing per-step stopping checks.

Test the new helper against analytically known diagonal and repeated-argument
RF values and against independent integral-series references. Include uncertain
input intervals, signed grid/output scales, near-diagonal spreads and declined
wide/zero cases. Existing Carlson, Legendre, inverse, Jacobi, complex and spatial
tests must still pass. Approximation midpoints may change within the existing
strict error contract; exact mathematical values, output domains, qualified
receipts and serialized expression definitions remain unchanged. Record that
evaluation change explicitly rather than promising identical midpoint bits.
Measure the original complete consumer, including all six enclosures and exact
downstream identities; a faster incomplete evaluation is not acceptance.

The [verification record](../../hyperspatial/verification/2026-10-19-e2-complex-jacobi/README.md)
retains the original timeout and slow unit, unfavorable optimization probes,
final complete-consumer measurements, native suites and the passing mutation
campaign under the unchanged ten-second input limit. These measurements do
not establish a general speed ratio or a native wall-time guarantee.
