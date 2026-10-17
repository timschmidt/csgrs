"""Retain additional lower-dimensional and symbolic controls before changing arrangement."""
import shutil
from pathlib import Path
work=Path(__file__).resolve().parent
out=work/'prototype-v3'
shutil.copytree(work/'prototype-v2',out)
p=out/'tests/containment_winding.rs'
s=p.read_text()+r'''

fn flat_source(copies: usize, z: i32) -> TriangleMesh {
    let positions = vec![point([-4, -4, 2*z]), point([4, -4, 2*z]), point([0, 4, 2*z])];
    let mut triangles = Vec::new();
    for _ in 0..copies {
        triangles.extend([Triangle::new(0, 1, 2), Triangle::new(2, 1, 0)]);
    }
    TriangleMesh::new(positions, triangles)
}

fn regularize(context: &GeometryContext, sources: &[&TriangleMesh], op: BooleanOp) -> TriangleMesh {
    let refs: Vec<_> = sources.iter().map(|source| source.as_ref()).collect();
    let result = hypermesh::boolean(&context.mesh_context(), &refs, BooleanProgram::Operation(op)).unwrap();
    assert_eq!(result.certainty, MeshCertainty::Certified);
    let mut meshes = result.value.into_triangle_meshes().unwrap();
    assert_eq!(meshes.len(), 1);
    meshes.remove(0)
}

#[test]
fn cancelled_flat_sheets_regularize_to_canonical_empty() {
    for copies in [1, 2, 3] {
        let source = flat_source(copies, 0);
        for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512, GeometryContext::STRICT] {
            assert_eq!(source.validate_closed_pwn(&context.mesh_context()).unwrap().certainty, MeshCertainty::Certified);
            let output = regularize(&context, &[&source], BooleanOp::Union);
            assert!(output.positions.is_empty());
            assert!(output.triangles.is_empty());
            assert!(!solid::contains_point_with_context(&output, &point([0, 0, 0]), &context).unwrap().value);
            let continued = solid::union_with_context(&output, &source, &context).unwrap();
            assert_eq!(continued.certainty, GeometryCertainty::Certified);
        }
    }
}

fn assert_unit_box_boundary(output: &TriangleMesh) {
    assert_eq!(output.validate_closed_pwn(&GeometryContext::STRICT.mesh_context()).unwrap().certainty, MeshCertainty::Certified);
    let mut six_volume = Real::zero();
    for triangle in &output.triangles {
        let [a, b, c] = triangle.indices.map(|i| &output.positions[i]);
        let coords = |p: &Point3| [&p.x, &p.y, &p.z];
        assert!([a,b,c].into_iter().all(|p| coords(p).into_iter().all(|v| *v >= Real::from(-1) && *v <= Real::one())));
        assert!((0..3).any(|axis| [-1,1].into_iter().any(|edge| [a,b,c].into_iter().all(|p| *coords(p)[axis] == Real::from(edge)))));
        six_volume = six_volume + a.x.clone() * (b.y.clone()*c.z.clone()-b.z.clone()*c.y.clone())
            + a.y.clone() * (b.z.clone()*c.x.clone()-b.x.clone()*c.z.clone())
            + a.z.clone() * (b.x.clone()*c.y.clone()-b.y.clone()*c.x.clone());
    }
    assert_eq!(six_volume, Real::from(48));
}

#[test]
fn cancelled_flat_sheets_compose_with_a_closed_volume() {
    let boxes = [box_input([-1,-1,-1],[1,1,1],1)];
    let cube = mesh(&boxes);
    let points: Vec<_> = (-3..=3).flat_map(|x| (-3..=3).flat_map(move |y| (-3..=3).map(move |z| [x,y,z]))).collect();
    for copies in [1,2] {
        for z in [0,1,3] {
            let flat = flat_source(copies,z);
            for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
                for op in [BooleanOp::Union, BooleanOp::Intersection, BooleanOp::Difference, BooleanOp::SymmetricDifference] {
                    for reverse in [false,true] {
                        let sources = if reverse { [&flat,&cube] } else { [&cube,&flat] };
                        let output = regularize(&context, &sources, op);
                        let empty = op == BooleanOp::Intersection || (op == BooleanOp::Difference && reverse);
                        if empty {
                            assert!(output.positions.is_empty());
                            assert!(output.triangles.is_empty());
                        } else {
                            assert_unit_box_boundary(&output);
                            check_queries(&output,&boxes,&points,|p|p);
                        }
                    }
                }
            }
        }
    }
}

#[test]
fn symbolic_translation_retains_certified_boundary_and_interior_queries() {
    fn translate(p: Point3) -> Point3 { Point3::new(p.x + Real::pi(), p.y, p.z) }
    let boxes = [box_input([0,0,0],[2,2,2],1)];
    let original = mesh(&boxes);
    let source = TriangleMesh::new(original.positions.iter().cloned().map(translate).collect(),original.triangles.to_vec());
    check_queries(&source,&boxes,&[[1,1,1],[0,1,1],[0,0,1],[0,0,0],[5,1,1]],translate);
}
'''
# Use the native indexed Triangle representation.
s=s.replace('triangle.indices.map(|i| &output.positions[i])','triangle.0.map(|i| &output.positions[i])')
p.write_text(s)
p=out/'contract-v1.md'
p.write_text(p.read_text()+'''\nAdditional acceptance controls reproduce cancelled flat closed-PWN sources in\nthe native arrangement before changing it. Unary normalization must yield\ncanonical empty; mixed Boolean operations with a solid must satisfy the exact\nempty-set identities under both policies. Coincident multiplicities, crossing,\ntouching and disconnected flat sheets remain in the matrix. Verify output\nsource-plane support, exact signed volume, topology and independent grid\nmembership separately. A symbolic pi translation must preserve earlier\ncertified inside, boundary and outside queries. An open sheet remains invalid.\n''')
