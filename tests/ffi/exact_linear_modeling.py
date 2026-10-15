"""Exact integer profiles survive the actual Python/C extrusion boundary."""
from pathlib import Path
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'bindings/python'))
import csgrs

width = 2**53 + 1
region = csgrs.CurveRegionI128.rectangle(width, 1)
try:
    region.extrude(0)
except csgrs.CsgrsError as error:
    assert error.status == 4 and 'nonzero' in str(error)
else:
    raise AssertionError('zero extrusion published a successful model')
solid = region.extrude(2)
bounds = solid.bounding_box()
assert (bounds.maxs.x.to_int(), bounds.maxs.y.to_int(), bounds.maxs.z.to_int()) == (width, 1, 2)
del solid, region
print('Python exact linear extrusion and failure recovery passed')
