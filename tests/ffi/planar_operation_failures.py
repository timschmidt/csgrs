"""Actual ctypes consumer for planar transform diagnostics and recovery."""
from pathlib import Path
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'bindings/python'))
import csgrs

Region = csgrs.CurveRegionF64
square = Region.square(1)
empty = Region.square(0)
for source in [square, empty]:
    for operation, status in [
        (lambda: source.scale(0, 1, 1), 4),
        (lambda: source.translate(0, 0, 1), 7),
        (lambda: source.rotate(90, 0, 0), 7),
        (lambda: source.translate(0, 0, float('nan')), 3),
    ]:
        try:
            operation()
        except csgrs.CsgrsError as error:
            assert error.status == status and str(error)
        else:
            raise AssertionError('failed transform published geometry')
reflected = square.rotate(180, 0, 0)
bounds = reflected.bounding_box()
assert bounds.mins.y == -1 and bounds.maxs.y == 0 and bounds.maxs.x == 1
restored = empty.union(reflected)
assert restored.bounding_box().mins.y == -1
del restored, reflected, empty, square
print('Python planar errors, half-turn meaning, empty validation, and recovery passed')
