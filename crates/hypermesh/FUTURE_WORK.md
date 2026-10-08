# Future work

## Convex hulls of sampled spheres

`solid::convex_hull` of a csgrs `solid::sphere` stops with `ConvexHullPredicate { stage: "face outside linear query" }` for most sample counts.

**Why.** The four corners of each quad between two latitude rings form an isosceles trapezoid, which is exactly planar. The hull's `orient3d` against such a face is therefore an exact zero. With trigonometric coordinates that zero has no structural proof, and refinement cannot certify it.

**What already decides.** Exact sine and cosine are available at multiples of π/2, π/3, π/4, π/6 and π/12. The last adds the 15-degree values `(sqrt(6) ∓ sqrt(2))/4`. Spheres whose segment and stack counts divide those turns have coordinates in `Q(sqrt(2), sqrt(3))`. Their coplanarity zeros are multiquadratic identities, and the quadratic tower decides them: `sphere(8, 24, 12)` gives a certified hull with 528 triangles.

**What does not.** Other sample counts still produce opaque `SinPi` computables or nested radicals. Examples are the 32 × 16 sphere used by csgrs's `kernel_comparison` bench, and any count with an angle such as π/16 or π/5. Their hulls stay undecidable.

**Options:**
- **Hull from mesh structure.** The quads come from the sampled grid, so the generator knows which four vertices are coplanar. Carrying that combinatorial certificate into the hull's `memberships` would decide the zeros without arithmetic.
- **Larger exact trig tables.** Add closed forms where they are biquadratic or one nested radical deep, such as π/8 and π/5. These would only help where the quadratic tower already covers the field.
- **Hull of an exact-rational sphere.** Rational points on the sphere (for example from rational stereographic parameters) make every predicate rational, but they change the sampled geometry.
