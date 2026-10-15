"""Bounds absence through the live Python/C ABI path."""
from pathlib import Path
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'bindings/python'))
import csgrs

cube = csgrs.TriangleMeshF64.cube(1)
empty = cube.difference(cube).center().floating()
assert empty.bounding_box() is None
restored = empty.union(cube)
bounds = restored.bounding_box()
assert (bounds.mins.x, bounds.maxs.x) == (0, 1)
centered = cube.center().floating()
bounds = centered.bounding_box()
assert (bounds.mins.x, bounds.maxs.x, bounds.mins.z, bounds.maxs.z) == (-0.5, 0.5, 0, 1)
square = csgrs.CurveRegionF64.square(1)
removed = square.difference(square)
assert removed.bounding_box() is None
del removed, square, centered, restored, empty, cube
print('Python consumer: absent bounds and exact positioning passed')
