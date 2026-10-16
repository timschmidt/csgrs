"""Real ctypes consumer: undecided Booleans retain status and publish no model."""
from pathlib import Path
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'bindings/python'))
import csgrs

cube = csgrs.TriangleMeshF64.cube(1)
rotated = cube.rotate(0, 0, 13)
restored = rotated.rotate(0, 0, -13)
structured = 0
for operation in ['union', 'difference', 'intersection', 'xor']:
    try:
        getattr(cube, operation)(restored)
    except csgrs.CsgrsError as error:
        assert 'predicate could not be decided' in str(error)
        structured += error.status == 5
    else:
        raise AssertionError('failed Boolean published geometry')
print(f'Python undecided Boolean statuses: {structured}/4', flush=True)
assert structured == 4
empty = cube.difference(cube)
assert empty.bounding_box() is None
recovered = empty.union(cube)
bounds = recovered.bounding_box()
assert (bounds.mins.x, bounds.maxs.x) == (0, 1)
del recovered, empty, restored, rotated, cube
print('Python typed Boolean failures, empty result and recovery passed')
