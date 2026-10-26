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
assert square.difference(restored).bounding_box() is None
assert square.xor(restored).bounding_box() is None
for same in [square.union(restored), square.intersection(restored)]:
    bounds = same.bounding_box()
    assert bounds.mins.x == 0 and bounds.mins.y == 0 and bounds.maxs.x == 2 and bounds.maxs.y == 2
# The two-rotation identity is now certified. Keep it as a success control;
# this independently reconstructed three-rotation zero still has no proof.
opaque = square.rotate(0, 0, 13.1).rotate(0, 0, 0.5).rotate(0, 0, -(13.1 + 0.5))
for operation in [square.union, square.difference, square.intersection, square.xor]:
    fails(lambda: operation(opaque), 5, 'blocked by Predicate')
fails(lambda: square.revolve(360, 1), 4, 'segments must not be less than 2')
# R01: signed meridians survive disconnected composition and repeated coverage
# through the real ctypes/C ABI path, including its finite bounds projection.
negative = Region.square(1).translate(-2, 0, 0)
positive = Region.square(1).translate(1, 2, 0)
mixed = negative.union(positive)
for segments in [3, 5, 6]:
    for angle in [-720, 360, 1080]:
        mesh = mixed.revolve(angle, segments)
        assert mesh.bounding_box().mins.x == -2
        del mesh
fails(lambda: mixed.revolve(450, 3), 4, 'invalid arguments')
del mixed, positive, negative
large = Region.square(sys.float_info.max).scale(2, 2, 1)
fails(large.region_profiles, 4, 'finite projection point is not finite')
empty = square.difference(square)
assert empty.bounding_box() is None
recovered = empty.union(square)
bounds = recovered.bounding_box()
assert bounds.mins.x == 0 and bounds.mins.y == 0 and bounds.maxs.x == 2 and bounds.maxs.y == 2
del recovered, empty, large, opaque, restored, square
print('Python curve categories, diagnostics, empty composition and recovery passed')
