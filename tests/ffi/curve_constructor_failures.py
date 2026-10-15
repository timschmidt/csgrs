"""Actual ctypes consumer: failed planar constructors never publish a model."""
from pathlib import Path
import ctypes
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'bindings/python'))
import csgrs

Region = csgrs.CurveRegionF64
for construct, status in [
    (lambda: Region.square(-1), 4),
    (lambda: Region.rectangle(0, -1), 4),
    (lambda: Region.circle(0, 2), 4),
    (lambda: Region.circle(1, ctypes.c_size_t(-1).value), 6),
    (lambda: Region.polygon([]), 4),
]:
    try:
        construct()
    except csgrs.CsgrsError as error:
        assert error.status == status and str(error)
    else:
        raise AssertionError('failed constructor published successful geometry')
empty = Region.square(0)
assert empty.bounding_box() is None
restored = empty.union(Region.square(1))
assert restored.bounding_box().maxs.x == 1
del restored, empty
print('Python planar errors, empty composition, and recovery passed')
