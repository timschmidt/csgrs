"""Actual ctypes consumer: transform errors retain their failure category."""
from pathlib import Path
import ctypes
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'bindings/python'))
import csgrs

cube = csgrs.TriangleMeshF64.cube(1)
shifted = cube.translate(-0.5, 0, 0)
matrix = csgrs.Matrix4F64((ctypes.c_double * 16)(0, 0, 0, 1, 0, 1, 0, 0, 0, 0, 1, 0, 1, 0, 0, 0))
for operation, status in [
    (lambda: cube.scale(0, 1, 1), 4),
    (lambda: shifted.transform(matrix), 9),
    (lambda: cube.transform(matrix), 9),
]:
    try:
        operation()
    except csgrs.CsgrsError as error:
        assert error.status == status
        assert str(error)
    else:
        raise AssertionError('failed transform published successful geometry')
restored = shifted.translate(0.5, 0, 0)
assert restored.bounding_box().maxs.x == 1
del restored, shifted, cube
print('Python typed transform errors and recovery passed')
