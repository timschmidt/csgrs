
"""Real ctypes wrapper: zero-volume sources preserve the regularized identities."""
import sys
sys.dont_write_bytecode=True
sys.path.insert(0,sys.argv[1])
import csgrs
cube=csgrs.TriangleMeshF64.cube(1)
def check(mesh,empty):
    owner=mesh.vertices_and_indices();raw=owner.raw
    if empty:
        assert raw.vertex_len==raw.index_len==0
    else:
        volume=0
        for i in range(raw.index_len):
            t=raw.indices[i];a,b,c=[raw.vertices[j] for j in [t.a,t.b,t.c]]
            assert all(0<=v<=1 for p in [a,b,c] for v in [p.x,p.y,p.z])
            assert any(getattr(a,axis)==getattr(b,axis)==getattr(c,axis) and getattr(a,axis) in [0,1] for axis in ['x','y','z'])
            volume+=a.x*(b.y*c.z-b.z*c.y)+a.y*(b.z*c.x-b.x*c.z)+a.z*(b.x*c.y-b.y*c.x)
        assert volume==6
for copies in [1,2,3]:
    for z in [0.5,1,2]:
        flat=csgrs.TriangleMeshF64.polyhedron([(-2,-2,z),(4,-2,z),(-2,4,z)],[[0,1,2],[2,1,0]]*copies)
        for operation in ['union','intersection','difference','xor']:
            for reverse in [False,True]:
                left,right=(flat,cube) if reverse else (cube,flat)
                output=getattr(left,operation)(right)
                empty=operation=='intersection' or (operation=='difference' and reverse)
                check(output,empty)
                check(output.union(cube) if empty else output.difference(cube),not empty)
print('Python cancelled sheets: 72 Boolean cases and downstream compositions passed')
