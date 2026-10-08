# V802 finite-source authority audit

Baseline: Hypercurve69b8de16, homogeneous-projection-v800. Read-only audit; no universal closure claim.

- General Curve2 evaluation checks a common-sign unit-domain certificate or the exact weight at the selected parameter before publishing a lazy rational point.
- The implicit-conic route now checks the original source denominator at every selected root (V796).
- Circle/rational discovery starts with finite_discovery_envelope, which proves a pole-free enclosure or excludes exterior isolator poles against exact source bounds. Selected parallel-normal, pair-radial, chord-normal and represented circle systems additionally require denominator_sign(range). Recursive selected-radial polynomial-sign replay checks its target weight at the selected parameter.
- The source-related chord shortcut is called in production only after rational_intersections proves its requested range finite; its unit-source shortcut additionally requires a finite unit source. General recursive chord queries use the same outer gate. Affine-line construction has constant unit weight. Monotone overlap transport retains and checks its admitted source range.
- Rational mapped point sources originate from admitted circle contacts/overlaps. Their similarity transports preserve parameter/weight. Algebraic-to-rational parallel component materialization is an optional view of a finite owned point; exact_center materialization does not itself publish an arbitrary new selected point.

The audit did not expose a second missing finite-domain gate in these remaining lazy source constructors. This is a source/call-chain audit, not an exhaustive independent test of every private constructor. Preserve geometry-owned finite proofs; do not force coordinate reconstruction on every point.

A separate public API gap was reproduced in V804: polynomial point and endpoint images report XImageFailed for sqrt(2)/pi coefficients at the positive root of2t²-1, while identical unit-weight rational curves produce Transformed points (eight cases, both policies). V803 inventories all callers of the redundant polynomial point carrier and endpoint sum type across2048source files. Next candidate V805 shares the existing rational point machinery and directly migrates these callers.
