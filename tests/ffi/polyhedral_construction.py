"""Exercise the shipped Python wrapper against the candidate native library."""
from pathlib import Path
import sys
sys.dont_write_bytecode = True
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'bindings/python'))
import csgrs

try:
    csgrs.TriangleMeshF64.polyhedron([(0,0,0),(3,3,0),(0,2,0),(2,0,0)], [[0,1,2,3]])
except csgrs.CsgrsError as error:
    assert error.status == 4
    assert 'simple ring' in str(error)
else:
    raise AssertionError('crossed face became successful geometry')

face = csgrs.TriangleMeshF64.polyhedron([(3,1,0),(1,1,0),(1,3,0),(0,3,0),(0,0,0),(3,0,0)], [[0,1,2,3,4,5]])
buffers = face.vertices_and_indices()
raw = buffers.raw
twice_area = 0
for i in range(raw.index_len):
    triangle = raw.indices[i]
    a,b,c = [raw.vertices[j] for j in [triangle.a, triangle.b, triangle.c]]
    twice_area += (b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x)
assert twice_area == 10
print('Python: typed face failure and exact concave orientation passed')
