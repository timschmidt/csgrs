# ADR 0002: Keep exact canonical coordinates

Status: accepted.

Internal geometry, centroids, rotations, and panel transforms use
`hyperreal::Real` and typed coordinate frames. Units, axes, handedness, board
side, view convention, and rotation convention are explicit. Lossy projection
to decimal Gerber, drill, SVG, or CSV coordinates occurs only at the external
format boundary and is covered by round-trip evidence.

Adapters must not infer bottom-side orientation or panel transforms from
rendered graphics.
