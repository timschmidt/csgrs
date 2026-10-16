use csgrs::{GeometryContext, Real, TriangleMesh, solid::{self, SolidExt}};
use hypermesh::{BooleanOp, BooleanProgram};
fn r(n: i32) -> Real { n.into() }
fn report(label: &str, result: Result<TriangleMesh, impl std::fmt::Debug>) {
    match result {
        Err(error) => println!("{label}: error {error:?}"),
        Ok(mesh) => {
            let six = mesh.triangles.iter().fold(r(0),|s,t| {
                let [a,b,c] = t.indices().map(|i| mesh.positions[i].to_vector());
                s+a.dot(&b.cross(&c))
            });
            println!("{label}: triangles={} six_volume={:?} topology={:?}",
                mesh.triangles.len(), six.exact_rational_normal_form(),
                mesh.validate_closed_pwn(&GeometryContext::STRICT.mesh_context()));
        }
    }
}
fn main() {
    let mut cells=Vec::new();
    for (x,a) in [(0,2),(1,3),(3,2)] {
        for (y,b) in [(0,2),(1,3),(3,2)] {
            for (z,c) in [(0,2),(1,3),(3,2)] {
                if (x,y,z)==(1,1,1) { continue; }
                cells.push(solid::cuboid(r(a),r(b),r(c)).unwrap().translated(r(x),r(y),r(z)).unwrap());
            }
        }
    }
    let context=GeometryContext::STRICT.mesh_context();
    let views=cells.iter().map(TriangleMesh::as_ref).collect::<Vec<_>>();
    report("batch-26", hypermesh::boolean(&context,&views,BooleanProgram::Operation(BooleanOp::Union))
        .and_then(|out|out.value.into_triangle_meshes()).map(|mut meshes|meshes.remove(0)));
    let merged=solid::merge(&cells).unwrap();
    report("merged-26", hypermesh::boolean(&context,&[merged.as_ref()],BooleanProgram::Operation(BooleanOp::Union))
        .and_then(|out|out.value.into_triangle_meshes()).map(|mut meshes|meshes.remove(0)));
    report("incremental-26",cells.iter().try_fold(solid::empty(),|acc,cell|acc.try_union(cell)));
}
