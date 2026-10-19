import sys
sys.dont_write_bytecode = True
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'bindings/python'))
import csgrs
cube = csgrs.TriangleMeshF64.cube(1)
moved = cube.translate(3, 5, 7)
removed = moved.difference(moved)
for mesh in [cube, moved, removed]:
    evidence = mesh.construction_evidence()
    assert not evidence.source_history_missing and not evidence.approximate_512_consumed
sphere = csgrs.TriangleMeshF64.sphere(1, 8, 4)
evidence = sphere.inverse().construction_evidence()
assert evidence.source_sampled and not evidence.source_history_missing
del sphere
assert evidence.source_sampled
print('Python native construction evidence passed')
