"""Exercise the public Python wrapper against CSGRS_LIBRARY, not a mock."""
from pathlib import Path
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'bindings/python'))
import csgrs


def check_empty(mesh):
    buffers = mesh.vertices_and_indices()
    assert buffers.raw.vertex_len == 0 and buffers.raw.index_len == 0
    del buffers


def check_box(mesh):
    bounds = mesh.bounding_box()
    assert (bounds.mins.x, bounds.mins.y, bounds.mins.z) == (0, 0, 0)
    assert (bounds.maxs.x, bounds.maxs.y, bounds.maxs.z) == (1, 1, 1)


cube = csgrs.TriangleMeshF64.cube(1)
empty = cube.difference(cube)
check_empty(empty)
opened = csgrs.TriangleMeshF64.polyhedron([(0, 0, 0), (1, 0, 0), (0, 1, 0)], [[0, 1, 2]])
for operation in ['union', 'intersection', 'difference', 'xor']:
    for empty_on_left in [False, True]:
        left, right = (empty, cube) if empty_on_left else (cube, empty)
        result = getattr(left, operation)(right)
        if operation == 'intersection' or (operation == 'difference' and empty_on_left):
            check_empty(result)
            check_box(result.union(cube))
        else:
            check_box(result)
            check_empty(result.difference(cube))
        del result
        left, right = (empty, opened) if empty_on_left else (opened, empty)
        try:
            getattr(left, operation)(right)
        except csgrs.CsgrsError as error:
            assert str(error) == f'input mesh {int(empty_on_left)} has 3 boundary edges'
        else:
            raise AssertionError('malformed input became successful geometry')
del left, right, opened, empty, cube
print('Python consumer: empty composition and visible failures passed')
