"""Add real downstream consumers for the cancelled-sheet arrangement extension."""
from pathlib import Path
W=Path(__file__).resolve().parent;ROOT=W.parents[2];P=W/'ffi-probes-v1';P.mkdir(exist_ok=False)
base=(ROOT/'tests/ffi/empty_composition.c').read_text().split('int main(void)')[0]
(P/'cancelled_flat.c').write_text(base+r'''
int main(void) {
  csgrs_triangle_mesh_t *cube = NULL;
  assert(csgrs_triangle_mesh_f64_cube(1, &cube) == CSGRS_STATUS_OK);
  const boolean_fn operations[] = {csgrs_triangle_mesh_f64_union,
    csgrs_triangle_mesh_f64_intersection, csgrs_triangle_mesh_f64_difference,
    csgrs_triangle_mesh_f64_xor};
  const double levels[] = {0.5, 1, 2};
  for (size_t copies=1; copies<=3; ++copies) {
    for (size_t plane=0; plane<3; ++plane) {
      const double z=levels[plane];
      const csgrs_vec3_f64_t points[] = {{-2,-2,z},{4,-2,z},{-2,4,z}};
      size_t indices[18], offsets[7];
      const size_t pair[] = {0,1,2,2,1,0};
      for (size_t i=0; i<6*copies; ++i) indices[i]=pair[i%6];
      for (size_t i=0; i<=2*copies; ++i) offsets[i]=3*i;
      csgrs_triangle_mesh_t *flat=NULL;
      assert(csgrs_triangle_mesh_f64_polyhedron(points,3,indices,6*copies,offsets,2*copies+1,&flat)==CSGRS_STATUS_OK);
      for (size_t operation=0; operation<4; ++operation) {
        for (int reverse=0; reverse<2; ++reverse) {
          csgrs_triangle_mesh_t *output=NULL,*continued=NULL;
          assert(operations[operation](reverse?flat:cube,reverse?cube:flat,&output)==CSGRS_STATUS_OK);
          if (operation==1 || (operation==2 && reverse)) {
            check_empty(output);
            assert(csgrs_triangle_mesh_f64_union(output,cube,&continued)==CSGRS_STATUS_OK);
            check_unit_box(continued);
          } else {
            check_unit_box(output);
            assert(csgrs_triangle_mesh_f64_difference(output,cube,&continued)==CSGRS_STATUS_OK);
            check_empty(continued);
          }
          csgrs_triangle_mesh_free(continued);csgrs_triangle_mesh_free(output);
        }
      }
      csgrs_triangle_mesh_free(flat);
    }
  }
  csgrs_triangle_mesh_free(cube);
  puts("C cancelled sheets: 72 Boolean cases and downstream compositions passed");
}
''')
(P/'cancelled_flat.cpp').write_text(r'''
#include "csgrs.hpp"
#include <cassert>
#include <iostream>
using Mesh = csgrs::TriangleMesh<csgrs::F64>;
static void check(const Mesh &mesh, bool empty) {
  const auto b=mesh.vertices_and_indices();
  if (empty) { assert(b.vertex_len==0 && b.index_len==0); }
  else {
    double volume=0;
    for (size_t i=0;i<b.index_len;++i) {
      const auto t=b.indices[i]; const auto a=b.vertices[t.a],c=b.vertices[t.b],d=b.vertices[t.c];
      volume+=a.x*(c.y*d.z-c.z*d.y)+a.y*(c.z*d.x-c.x*d.z)+a.z*(c.x*d.y-c.y*d.x);
    }
    assert(volume==6);
  }
  csgrs::F64::mesh_buffers_free(b);
}
int main() {
  const auto cube=Mesh::cube(1);
  const auto flat=Mesh::polyhedron({{-2,-2,0.5},{4,-2,0.5},{-2,4,0.5}},{0,1,2,2,1,0},{0,3,6});
  check(flat.union_with(cube),false);check(cube.difference(flat),false);
  check(flat.intersection(cube),true);check(flat.difference(cube),true);
  check(flat.union_with(cube).difference(cube),true);
  std::cout << "C++ cancelled-sheet composition passed\n";
}
''')
(P/'cancelled_flat.py').write_text(r'''
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
''')
(P/'contract.json').write_text('''{
  "scope": "Real C, C++ and Python consumers of the changed native arrangement through the existing scalar mesh ABI. Existing empty, typed-Boolean and polyhedral tests also run.",
  "oracle": "A triangle and its opposite have zero winding. Repeated copies at z=1/2,1,2 represent empty; all four Boolean operations with a unit cube obey exact empty identities. Dyadic retained cube vertices make the exact volume-six sum representable in binary64; plane support and downstream composition are checked separately.",
  "certainty_limit": "These existing C-family adapters expose status and errors, not an end-to-end certainty receipt. This validation does not certify the missing receipt/provenance interface or a published distribution.",
  "source": "Same frozen final candidate-source-v3 used by native and WASM qualification. No runtime or repository source changes."
}
''')
