"""Real consumer of polyhedron diagnostics and explicit graphics projection."""
from pathlib import Path
import sys

sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'bindings/python'))
import csgrs

Mesh = csgrs.TriangleMeshF64
try:
    Mesh.polyhedron([(0,0,0), (3,3,0), (0,2,0), (2,0,0)], [[0,1,2,3]])
except csgrs.CsgrsError as error:
    assert error.status == 4 and 'simple ring edges cross' in str(error)
else:
    raise AssertionError('malformed ring published a model')

mesh = Mesh.polyhedron([(0,0,0), (1,0,0), (0,3,4)], [[0,1,2]])
for _ in range(2):
    graphics = mesh.graphics_mesh()
    assert graphics.raw.vertex_len == graphics.raw.index_len == 3
    for index in range(3):
        normal = graphics.raw.vertices[index].normal
        assert (normal.x, normal.y, normal.z) == (0, -0.8, 0.6)
    del graphics

for exponent in [100, 400, 1000, 1074]:
    thin = Mesh.polyhedron([(0,0,0), (1,0,0), (0,2.0**-exponent,0)], [[0,1,2]])
    graphics = thin.rotate(0, 0, 13).graphics_mesh()
    assert graphics.raw.vertex_len == graphics.raw.index_len == 3
    del graphics, thin

empty = Mesh.cube(0).graphics_mesh()
assert empty.raw.vertex_len == empty.raw.index_len == 0
del empty, mesh
print('Python polyhedron diagnostics, normal values, thin rotated surfaces and empty recovery passed')
