# Native Legendre elliptic integrals

Baseline: `8b4acb6eb`. Owner: Hyperreal, P1–P7 and P17–P20; downstream
consumer: Hyperspatial E2 rectification. The baseline supplies RF/RD but lacks
complete and incomplete Legendre APIs. The initial acceptance test is
`tests/legendre_integrals.rs`, which must fail to compile on that baseline.

The extension represents F(phi|m), E(phi|m), K(m), and complete E(m) for real
amplitude phi and real **parameter m < 1** (m is the squared modulus).
Incomplete integrals start at zero and retain the unwrapped amplitude.
Their derivatives are respectively `1/sqrt(cos²(phi)+(1-m)*sin²(phi))` and
`sqrt(cos²(phi)+(1-m)*sin²(phi))`. Complete integrals end at pi/2.
The common domain excludes m=1, including the finite second-kind endpoint;
complex continuation is not part of this extension. Invalid parameters return
`Problem::NotANumber`; unresolved parameter-domain proofs return
`Problem::Exhausted`. Validate before zero/circular identities.

Every production scalar remains Real-backed. No external runtime dependency,
floating-point seed, rounded amplitude, or finite-series model is introduced.
Outputs must support arithmetic, strict predicates, native serialization,
repeated refinement, cancellation recovery, and spatial composition.
Existing precision, allocation, and abort contracts apply; no new total-work,
heap-byte, or wall-time bound is promised. Cache warming cannot change meaning.

## Representation and branch proof

Append incomplete-integral nodes with payload `[phi, ln(1-m)]`, so every finite
deserialized payload has a valid parameter domain. The amplitude's sign is the
integral's sign. Complete integrals are positive Carlson combinations. The
circular identities K(0)=E_complete(0)=pi/2 and F(phi|0)=E(phi|0)=phi remain
symbolic. Zero amplitudes remain zero after parameter validation.

Let c=1-m>0. For an amplitude r in (-pi/2, pi/2), put S=sin(r), C=cos(r),
Y=C²+c*S². The principal formulas are

```
F(r|m) = S*RF(C²,Y,1)
E(r|m) = S*C/sqrt(Y) + c*S³/3 * (RD(C²,Y,1)+RD(C²,1,Y)).
K(m) = RF(0,c,1)
E_complete(m) = c/3 * (RD(0,c,1)+RD(0,1,c)).
```

The second-kind formula adds terms of the same sign and avoids subtracting
large nearly equal first-kind terms. It follows from the Carlson cyclic
identities and the [Legendre relations](https://dlmf.nist.gov/19.25).
Specifically, for positive x,y,z, the cyclic sum of RD is `3/sqrt(x*y*z)`
and the sum weighted by the respective distinguished argument is `3*RF`.
Set x=C², y=Y, z=1 and substitute into `S*RF-m*S³*RD(x,y,1)/3`.
Here y-x=c*S² and 1-x=S², giving the positive expression above.

During evaluation, approximate phi/(pi/2) at absolute error strictly below one
to choose an integer n. Then r=phi-n*pi/2 is strictly inside the principal
chart. This multivalued choice needs no decision at an exact seam. For even n,
add n complete integrals to the principal integral at r with parameter m.
For odd n use complement 1/c instead, multiplying the local F by 1/sqrt(c)
or local E by sqrt(c). The substitution t=n*pi/2+u in the defining integral
proves these formulas for either sign of n. The represented value retains
phi, not a particular reduction choice. Abort checks must prevent an interrupted
phase approximation from being consumed as a branch proof.

## Required acceptance

- Complete/circular/equatorial identities, odd symmetry, quarter and full
  periods, exact seams and offsets of 1e-320, and large unwrapped amplitudes.
- Independent MPFR integrand-series and AGM comparisons, oblate and prolate
  parameters, extreme positive complements, and all 22 scalar families.
- Typed invalid/unresolved domains before identities; low/high/low refinement,
  cold/warm/disabled caches, concurrent reads, cancellation and recovery.
- JSON/CBOR replay, appended-node inventory, structural sign/identity, and
  downstream strict spatial consumption with qualification retained.
- Completion-checked benchmarks, representation-bucket selection, ASan fuzz
  replay, production coverage, stable/MSRV/WASM and affected downstream checks.

The tests compare independent binomial expansions of the defining integrands
at 1024-bit MPFR precision, with 2048 terms and a bounded analytical tail below
2^-840 for the chosen corpus. Native comparisons use 512-bit acceptance.
Complete values also use an independent
[AGM and square-sum oracle](https://dlmf.nist.gov/19.8#i), including complements
1e-160 and 1e160; these comparisons use a wider 192-bit margin (normalizing
the large second-kind result). Oracle agreement is not a proof of the native
implementation or of MPFR's finite-precision error.

Executed checks and retained evidence are in the
[verification bundle](../../hyperspatial/verification/2026-10-19-e2-legendre/README.md).
Carlson and Legendre retain independent 95% production-line gates; Hyperspatial
and the inverse-isometric subset keep their existing independent gates.
This prerequisite
does not itself implement inverse Legendre or Jacobi functions, ellipsoidal
Transverse Mercator, or the later Earth/sky stages.
