"""Python consumer of the packaged csgrs-ffi library through bindings/python.

Drills a 12-unit cube with a 4 x 4 bar in the f64 family and checks the exact
bounds and the closed edge pairing of the exported buffers.
"""

import sys

import csgrs

csgrs.load(sys.argv[1])
body = csgrs.TriangleMeshF64.cube(12.0)
drill = csgrs.TriangleMeshF64.cuboid(4.0, 4.0, 16.0).translate(4.0, 4.0, -2.0)
part = body.difference(drill)

bounds = part.bounding_box()
assert bounds is not None, "a drilled cube has bounds"
mins = (bounds.mins.x, bounds.mins.y, bounds.mins.z)
maxs = (bounds.maxs.x, bounds.maxs.y, bounds.maxs.z)
assert mins == (0.0, 0.0, 0.0) and maxs == (12.0, 12.0, 12.0), (mins, maxs)

buffers = part.vertices_and_indices()
raw = buffers.raw
triangles = [(raw.indices[i].a, raw.indices[i].b, raw.indices[i].c) for i in range(raw.index_len)]
directed = {(t[e], t[(e + 1) % 3]) for t in triangles for e in range(3)}
unpaired = sum((b, a) not in directed for (a, b) in directed)
assert triangles and unpaired == 0, (len(triangles), unpaired)
print(f"python: {len(triangles)} triangles, closed, exact bounds")
