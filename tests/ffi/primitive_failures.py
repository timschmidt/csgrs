"""Actual ctypes consumer: exceptions preserve constructor failure categories."""
from pathlib import Path
import ctypes
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'bindings/python'))
import csgrs

for construct, status in [
    (lambda: csgrs.TriangleMeshF64.cube(-1), 4),
    (lambda: csgrs.TriangleMeshF64.sphere(0, 2, 4), 4),
    (lambda: csgrs.TriangleMeshF64.sphere(1, ctypes.c_size_t(-1).value, 2), 6),
]:
    try:
        construct()
    except csgrs.CsgrsError as error:
        assert error.status == status
        assert str(error)
    else:
        raise AssertionError('failed construction published successful geometry')

empty = csgrs.TriangleMeshF64.cube(0)
cube = csgrs.TriangleMeshF64.cube(1)
restored = empty.union(cube)
assert restored.bounding_box().maxs.x == 1
del restored, cube, empty
print('Python primitive errors and recovery passed')
