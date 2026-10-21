# Future work

## Earcut cost grows quadratically with hole count

A planar region with many holes is a natural result of successive 2D
Booleans, and extrusion triangulates its caps with `earcut`. Hole elimination
bridges each hole into the outer ring in turn, so the work grows with the
number of holes times the ring length.

**Evidence.** A focused release probe triangulated a rectangle minus k exact
squares with `csgrs::curve::try_triangulate`. It took 0.33, 0.82, 3.5 and 18 ms
for k = 4, 8, 16 and 32, giving 26, 50, 98 and 194 triangles. Each doubling of
the output cost 2.5 to 5.1 times more. This is a single timer-scale run, not a
benchmark.

**Options.** Use the constrained Delaunay path for regions with many holes,
or order and index hole bridging spatially. Either must keep the exact
predicate cascade and the certified output contract.
