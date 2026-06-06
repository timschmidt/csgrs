# hypergraphics

`hypergraphics` extracts the small desktop 3D renderer shape from CopperForge's
`render3d` module and adapts it to the hyperreal geometry stack.

Scene geometry is owned as `hyperreal::Real` coordinates through
`hyperlattice::Point3`. Exact topology and predicate decisions use
`hypertri` and `hyperlimit`. Primitive floats are only produced by explicit
projection/export APIs for render backends.

The OpenGL backend mirrors CopperForge's unlit `xyz rgb` mesh pipeline:

- one VAO/VBO pair per colored mesh,
- a single MVP uniform shader,
- `glow` upload/draw calls,
- conversion to `f32` only at the OpenGL upload/uniform boundary.

CopperForge credits this renderer pattern to `alumina-interface` by Timothy
Schmidt, MIT licensed. `hypergraphics` keeps the same renderer boundary while
replacing CopperForge's float-owned geometry with hyperreal-owned scene data.
