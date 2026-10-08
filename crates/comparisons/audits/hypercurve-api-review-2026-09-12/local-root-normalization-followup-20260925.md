# Normalize retained local-root coefficients once (read-only finding)

The V25 late stack reaches BezierRecursivePolynomialParameterAuthority2::
defining_sign_at_real, scaling dense rational coefficients during contact
refinement. Its stored defining row keeps the original rational gauge.

Correction to the initial hypothesis: Hypersolve's ordered-field context exposes
normalize_positive_scale, but the Bernstein isolator currently never calls that
hook. Only root_sign.rs's Sturm-Tarski replay normalizes its local polynomial
copies. Isolation and the retained Hypercurve authority both begin with the
unnormalized row; this is not merely loss of a normalized isolation output.

There are four production struct construction sites for the local polynomial
authority, including independent-field sign replay, affine parameter transforms,
local Bernstein roots, and certified opposite-sign brackets. A single constructor
could apply the existing optional positive-content normalization once and retain
the normalized row. Local isolation should consume that same row before the
selected root is published. Positive scale preserves every root, selected sheet,
endpoint sign and sign-remainder query. Arbitrary exact nonrational coefficients
must continue unchanged when rational-content extraction declines. Existing
normalization code already preserves shared value nodes and rescales warm scalar
witnesses; no alternate coefficient representation is needed.

Validate wide positive/negative rational gauges through root isolation, repeated
refinement, affine charts, local sign replay and lazy projection. This is not yet
implemented, and its performance impact is not measured. It is a general replay
normalization opportunity independent of the optional regular-range derivative
certificate currently under qualification.

Implemented in the V28 candidate (not yet qualified): one private constructor now normalizes all four retained-authority construction sites, and local Bernstein isolation consumes that same row. Positive normalization preserves signs copied during an exact coefficient-field embedding. The existing local projection/chart regression now covers positive/negative 1025-bit rational scales and reciprocals, with local root and sign replay checked before projection. No coefficient payload restriction or secondary representation was introduced.
