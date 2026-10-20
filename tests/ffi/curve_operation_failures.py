"""Real ctypes consumer: undecided curve operations are distinct from invalid input."""
from pathlib import Path
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'bindings/python'))
import csgrs


def fails(action, status, diagnostic):
    try:
        action()
    except csgrs.CsgrsError as error:
        assert error.status == status and diagnostic in str(error)
    else:
        raise AssertionError('failed operation published geometry')


Region = csgrs.CurveRegionF64
square = Region.square(2)
restored = square.rotate(0, 0, 13).rotate(0, 0, -13)
for operation in [square.union, square.difference, square.intersection, square.xor]:
    fails(lambda: operation(restored), 5, 'blocked by Ordering')
fails(lambda: square.revolve(360, 1), 4, 'segments must not be less than 2')
large = Region.square(sys.float_info.max).scale(2, 2, 1)
fails(large.region_profiles, 4, 'finite projection point is not finite')
empty = square.difference(square)
assert empty.bounding_box() is None
recovered = empty.union(square)
bounds = recovered.bounding_box()
assert bounds.mins.x == 0 and bounds.mins.y == 0 and bounds.maxs.x == 2 and bounds.maxs.y == 2
del recovered, empty, large, restored, square
print('Python curve categories, diagnostics, empty composition and recovery passed')
