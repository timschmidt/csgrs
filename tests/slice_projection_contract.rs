//! Exact slice/projection semantics and visible failures through public modeling APIs.
#![cfg(feature = "curve")]

use csgrs::errors::{ValidationError, ValidationErrorKind};
use csgrs::{GeometryCertainty, GeometryContext, Real, TriangleMesh, curve, solid};
use hypercurve::{BooleanOp, CurveRegion2};
use hyperlattice::{Matrix4, Point3};
use solid::cache::{self, CachePolicy};

// A baseline empty fallback is observed as success, never reinterpreted as error.
trait Attempt<T> {
    fn attempt(self) -> Result<T, ValidationError>;
}
impl Attempt<CurveRegion2> for CurveRegion2 {
    fn attempt(self) -> Result<CurveRegion2, ValidationError> {
        Ok(self)
    }
}
impl Attempt<solid::SliceResult> for solid::SliceResult {
    fn attempt(self) -> Result<solid::SliceResult, ValidationError> {
        Ok(self)
    }
}
impl<T, E: Into<ValidationError>> Attempt<T> for Result<T, E> {
    fn attempt(self) -> Result<T, ValidationError> {
        self.map_err(Into::into)
    }
}
fn unknown_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    &sine * &sine + &cosine * &cosine - Real::one()
}
fn half() -> Real {
    (Real::one() / Real::from(2)).unwrap()
}
fn triangle(points: Vec<Point3>) -> TriangleMesh {
    let cube = solid::cube(1.into()).unwrap();
    let mut face = cube.triangles[0];
    face.v0 = 0;
    face.v1 = 1;
    face.v2 = 2;
    TriangleMesh::new(points, vec![face])
}
fn volume_six(mesh: &TriangleMesh) -> Real {
    mesh.triangles.iter().fold(Real::zero(), |sum, face| {
        let [a, b, c] = face.indices().map(|i| mesh.positions[i].to_vector());
        sum + a.dot(&b.cross(&c))
    })
}
fn hollow_prism() -> TriangleMesh {
    let outer = curve::square(4.into()).unwrap();
    let hole =
        curve::translated(&curve::square(2.into()).unwrap(), 1.into(), 1.into()).unwrap();
    let profile = outer.boolean_region(&hole, BooleanOp::Difference).unwrap();
    assert_eq!(profile.filled_area().unwrap(), Some(12.into()));
    let mesh = curve::try_extrude(&profile, 2.into(), &GeometryContext::STRICT).unwrap();
    assert_eq!(mesh.certainty, GeometryCertainty::Certified);
    assert_eq!(volume_six(&mesh.value), Real::from(144));
    assert!(
        mesh.value
            .is_closed_manifold_geometry(&GeometryContext::STRICT.mesh_context())
            .unwrap()
            .value
    );
    mesh.value
}
fn assert_hollow(region: &CurveRegion2) {
    assert_eq!(
        region.filled_area().unwrap(),
        Some(12.into()),
        "a section filled the hole"
    );
    assert_eq!(
        curve::contains_xy(region, 2.into(), 2.into()).unwrap(),
        Some(false)
    );
    assert_eq!(
        curve::contains_xy(region, half(), half()).unwrap(),
        Some(true)
    );
    let mesh = curve::try_extrude(region, 3.into(), &GeometryContext::STRICT).unwrap();
    assert_eq!(mesh.certainty, GeometryCertainty::Certified);
    assert_eq!(volume_six(&mesh.value), Real::from(216));
    assert!(
        mesh.value
            .is_closed_manifold_geometry(&GeometryContext::STRICT.mesh_context())
            .unwrap()
            .value
    );
}

#[test]
fn malformed_slice_is_an_error_not_empty_geometry() {
    let error = solid::slice_z(&triangle(Vec::new()), 0.into())
        .attempt()
        .unwrap_err();
    assert_eq!(error.kind(), ValidationErrorKind::InvalidInput);
}
#[test]
fn malformed_projection_is_an_error_not_empty_geometry() {
    let error = solid::flatten(&triangle(Vec::new())).attempt().unwrap_err();
    assert_eq!(error.kind(), ValidationErrorKind::InvalidInput);
}
#[test]
fn undecided_slice_is_an_error_not_empty_geometry() {
    let mesh = solid::cube(1.into()).unwrap();
    let error = solid::slice_z(&mesh, unknown_zero()).attempt().unwrap_err();
    assert_eq!(error.kind(), ValidationErrorKind::Undecided);
}
#[test]
fn undecided_projection_is_an_error_not_empty_geometry() {
    let mesh = triangle(vec![
        Point3::origin(),
        Point3::new(1.into(), 0.into(), 0.into()),
        Point3::new(0.into(), unknown_zero(), 1.into()),
    ]);
    let error = solid::flatten(&mesh).attempt().unwrap_err();
    assert_eq!(error.kind(), ValidationErrorKind::Undecided);
}
#[test]
fn hollow_interior_slice_keeps_hole_and_composes() {
    let (region, strings, paths) =
        solid::slice_z(&hollow_prism(), 1.into()).attempt().unwrap();
    assert_hollow(&region);
    assert!(strings.is_empty() && paths.is_empty());
}
#[test]
fn coplanar_bottom_slice_keeps_hole_and_composes() {
    let (region, strings, paths) =
        solid::slice_z(&hollow_prism(), 0.into()).attempt().unwrap();
    assert_hollow(&region);
    assert!(strings.is_empty() && paths.is_empty());
}
#[test]
fn coplanar_top_slice_keeps_hole_and_composes() {
    let (region, strings, paths) =
        solid::slice_z(&hollow_prism(), 2.into()).attempt().unwrap();
    assert_hollow(&region);
    assert!(strings.is_empty() && paths.is_empty());
}
#[test]
fn face_permutation_and_global_reversal_preserve_hollow_set() {
    let mesh = hollow_prism();
    let mut faces = mesh.triangles.to_vec();
    faces.reverse();
    for face in &mut faces {
        std::mem::swap(&mut face.v1, &mut face.v2);
    }
    let reversed = TriangleMesh::new(mesh.positions.to_vec(), faces);
    assert_hollow(&solid::slice_z(&reversed, 1.into()).attempt().unwrap().0);
}
#[test]
fn nested_material_shells_do_not_become_parity_holes() {
    let outer = solid::cube(4.into()).unwrap();
    let inner = solid::transform(
        &solid::cube(2.into()).unwrap(),
        &Matrix4::affine_translation([1.into(), 1.into(), 1.into()]),
    )
    .unwrap();
    let mesh = solid::merge(&[outer, inner]);
    let (region, strings, paths) = solid::slice_z(&mesh, 2.into()).attempt().unwrap();
    assert_eq!(region.filled_area().unwrap(), Some(16.into()));
    assert_eq!(
        curve::contains_xy(&region, 2.into(), 2.into()).unwrap(),
        Some(true)
    );
    assert!(strings.is_empty() && paths.is_empty());
}
#[test]
fn duplicate_material_shells_do_not_cancel() {
    let cube = solid::cube(2.into()).unwrap();
    let mesh = solid::merge(&[cube.clone(), cube]);
    let (region, strings, paths) = solid::slice_z(&mesh, 1.into()).attempt().unwrap();
    assert_eq!(region.filled_area().unwrap(), Some(4.into()));
    assert_eq!(
        curve::contains_xy(&region, 1.into(), 1.into()).unwrap(),
        Some(true)
    );
    assert!(strings.is_empty() && paths.is_empty());
}
#[test]
fn disconnected_hollow_components_preserve_both_voids() {
    let first = hollow_prism();
    let second = solid::transform(
        &first,
        &Matrix4::affine_translation([6.into(), 0.into(), 0.into()]),
    )
    .unwrap();
    let mesh = solid::merge(&[first, second]);
    let (region, strings, paths) = solid::slice_z(&mesh, 1.into()).attempt().unwrap();
    assert_eq!(region.filled_area().unwrap(), Some(24.into()));
    for x in [2, 8] {
        assert_eq!(
            curve::contains_xy(&region, x.into(), 2.into()).unwrap(),
            Some(false)
        );
    }
    assert!(strings.is_empty() && paths.is_empty());
}
#[test]
fn canonical_empty_and_outside_slices_remain_consumable() {
    let empty = TriangleMesh::new(Vec::new(), Vec::new());
    let cube = solid::cube(1.into()).unwrap();
    for mesh in [&empty, &cube] {
        let (region, strings, paths) = solid::slice_z(mesh, 2.into()).attempt().unwrap();
        assert!(region.is_empty() && strings.is_empty() && paths.is_empty());
        let square = curve::square(1.into()).unwrap();
        assert_eq!(
            region
                .boolean_region(&square, BooleanOp::Union)
                .unwrap()
                .filled_area()
                .unwrap(),
            Some(1.into())
        );
        assert!(
            curve::try_extrude(&region, 1.into(), &GeometryContext::STRICT)
                .unwrap()
                .value
                .triangles
                .is_empty()
        );
    }
    assert!(solid::flatten(&empty).attempt().unwrap().is_empty());
}
#[test]
fn open_triangle_intersection_remains_an_open_wire() {
    let mesh = triangle(vec![
        Point3::new(0.into(), 0.into(), (-1).into()),
        Point3::new(2.into(), 0.into(), 1.into()),
        Point3::new(0.into(), 2.into(), 1.into()),
    ]);
    let (region, strings, paths) = solid::slice_z(&mesh, 0.into()).attempt().unwrap();
    assert!(region.is_empty() && paths.is_empty());
    assert_eq!(strings.len(), 1);
    assert_eq!(strings[0].segments().len(), 1);
    let segment = &strings[0].segments()[0];
    let points = [segment.start(), segment.end()].map(|p| [p.x().clone(), p.y().clone()]);
    assert!(points.contains(&[1.into(), 0.into()]));
    assert!(points.contains(&[0.into(), 1.into()]));
}
#[test]
fn projection_of_hollow_prism_keeps_void() {
    assert_hollow(&solid::flatten(&hollow_prism()).attempt().unwrap());
}
#[test]
fn cache_states_and_mutation_preserve_section_meaning() {
    let mesh = hollow_prism();
    for policy in [
        CachePolicy::DISABLED,
        CachePolicy {
            slices: 1,
            flattenings: 1,
            ..CachePolicy::DEFAULT
        },
    ] {
        cache::with_policy(policy, || {
            cache::clear();
            for _ in 0..2 {
                assert_hollow(&solid::slice_z(&mesh, 1.into()).attempt().unwrap().0);
            }
            assert!(
                solid::slice_z(&mesh, 3.into())
                    .attempt()
                    .unwrap()
                    .0
                    .is_empty()
            );
            assert_hollow(&solid::slice_z(&mesh, 1.into()).attempt().unwrap().0);
            let mut changed = mesh.clone();
            changed.positions = Vec::new().into();
            assert!(solid::slice_z(&changed, 1.into()).attempt().is_err());
            assert!(solid::flatten(&changed).attempt().is_err());
            assert_hollow(&solid::slice_z(&mesh, 1.into()).attempt().unwrap().0);
        });
    }
}
#[test]
fn approximate_slice_never_certifies_a_later_strict_request() {
    let mesh = solid::cube(1.into()).unwrap();
    let z = unknown_zero();
    for context in [
        GeometryContext::STRICT,
        GeometryContext::APPROXIMATE_512,
        GeometryContext::STRICT,
    ] {
        let result = solid::slice_z_with_context(&mesh, z.clone(), &context);
        if context == GeometryContext::STRICT {
            assert_eq!(result.unwrap_err().kind(), ValidationErrorKind::Undecided);
        } else {
            let result = result.unwrap();
            assert_eq!(result.certainty, GeometryCertainty::Approximate512Consumed);
            assert_eq!(result.value.0.filled_area().unwrap(), Some(1.into()));
        }
    }
}

fn reversed(mesh: &TriangleMesh) -> TriangleMesh {
    let mut faces = mesh.triangles.to_vec();
    for face in &mut faces {
        std::mem::swap(&mut face.v1, &mut face.v2);
    }
    TriangleMesh::new(mesh.positions.to_vec(), faces)
}

#[test]
fn opposite_shells_cancel_interior_and_coplanar_sections() {
    let cube = solid::cube(2.into()).unwrap();
    let mesh = solid::merge(&[cube.clone(), reversed(&cube)]);
    for z in [0, 1, 2] {
        let (region, strings, paths) = solid::slice_z(&mesh, z.into()).attempt().unwrap();
        assert!(region.is_empty(), "opposite winding at Z={z}");
        assert!(strings.is_empty() && paths.is_empty());
    }
}

#[test]
fn signed_multiplicity_is_preserved_interior_and_coplanar() {
    let cube = solid::cube(2.into()).unwrap();
    let mesh = solid::merge(&[cube.clone(), cube.clone(), reversed(&cube)]);
    for z in [0, 1, 2] {
        let (region, strings, paths) = solid::slice_z(&mesh, z.into()).attempt().unwrap();
        assert_eq!(region.filled_area().unwrap(), Some(4.into()));
        assert!(strings.is_empty() && paths.is_empty());
    }
}

#[test]
fn shared_on_plane_edges_do_not_change_hole_winding() {
    let lower = hollow_prism();
    let upper = solid::transform(
        &lower,
        &Matrix4::affine_translation([0.into(), 0.into(), 2.into()]),
    )
    .unwrap();
    let mesh = solid::merge(&[lower, upper]);
    let (region, strings, paths) = solid::slice_z(&mesh, 2.into()).attempt().unwrap();
    assert_hollow(&region);
    assert!(strings.is_empty() && paths.is_empty());
}

#[test]
fn vertex_touching_components_retain_both_material_regions() {
    let first = solid::cube(2.into()).unwrap();
    let second = solid::transform(
        &first,
        &Matrix4::affine_translation([2.into(), 2.into(), 0.into()]),
    )
    .unwrap();
    let mesh = solid::merge(&[first, second]);
    let (region, strings, paths) = solid::slice_z(&mesh, 1.into()).attempt().unwrap();
    assert_eq!(region.filled_area().unwrap(), Some(8.into()));
    for (x, y, inside) in [(1, 1, true), (3, 3, true), (1, 3, false), (3, 1, false)] {
        assert_eq!(
            curve::contains_xy(&region, x.into(), y.into()).unwrap(),
            Some(inside)
        );
    }
    assert!(strings.is_empty() && paths.is_empty());
}

#[test]
fn contextual_invalid_indices_preserve_the_native_diagnostic() {
    let mesh = triangle(Vec::new());
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        for error in [
            solid::flatten_with_context(&mesh, &context).unwrap_err(),
            solid::slice_z_with_context(&mesh, 0.into(), &context).unwrap_err(),
        ] {
            assert!(matches!(error, ValidationError::Mesh(_)));
            assert_eq!(error.kind(), ValidationErrorKind::InvalidInput);
            assert!(std::error::Error::source(&error).is_some());
        }
    }
}

#[test]
fn interleaved_opposite_faces_preserve_winding_after_permutation() {
    let cube = solid::cube(2.into()).unwrap();
    for copies in [1, 2] {
        let mut operands = vec![cube.clone(); copies];
        operands.push(reversed(&cube));
        let mesh = solid::merge(&operands);
        let n = cube.triangles.len();
        let mut faces = (0..n)
            .flat_map(|i| (0..operands.len()).map(move |j| j * n + i))
            .map(|i| mesh.triangles[i])
            .collect::<Vec<_>>();
        for shift in 0..n {
            faces.rotate_left(1);
            let permuted = TriangleMesh::new(mesh.positions.to_vec(), faces.clone());
            for z in [0, 1, 2] {
                let (region, strings, paths) = solid::slice_z(&permuted, z.into())
                    .attempt()
                    .unwrap_or_else(|error| {
                        panic!("copies={copies}, shift={shift}, z={z}: {error:?}")
                    });
                if copies == 1 {
                    assert!(region.is_empty());
                } else {
                    assert_eq!(region.filled_area().unwrap(), Some(4.into()));
                }
                assert!(strings.is_empty() && paths.is_empty());
            }
        }
    }
}

#[test]
fn thread_local_cache_modes_preserve_hollow_sections() {
    let workers = [CachePolicy::DEFAULT, CachePolicy::DISABLED].map(|policy| {
        std::thread::spawn(move || {
            cache::with_policy(policy, || {
                let mesh = hollow_prism();
                assert_hollow(&solid::slice_z(&mesh, 1.into()).attempt().unwrap().0);
            })
        })
    });
    for worker in workers {
        worker.join().unwrap();
    }
}

#[test]
fn independent_position_rows_preserve_the_same_section() {
    let source = hollow_prism();
    let mut positions = Vec::new();
    let faces = source
        .triangles
        .iter()
        .map(|face| {
            let start = positions.len();
            positions.extend(face.indices().map(|i| source.positions[i].clone()));
            let mut output = *face;
            output.v0 = start;
            output.v1 = start + 1;
            output.v2 = start + 2;
            output
        })
        .collect();
    let mesh = TriangleMesh::new(positions, faces);
    for z in [0, 1, 2] {
        let (region, strings, paths) = solid::slice_z(&mesh, z.into()).attempt().unwrap();
        assert_hollow(&region);
        assert!(strings.is_empty() && paths.is_empty());
    }
}

#[test]
fn shuffled_coincident_shell_faces_preserve_signed_set() {
    let cube = solid::cube(2.into()).unwrap();
    let source = solid::merge(&[cube.clone(), reversed(&cube)]);
    let mut faces = source.triangles.to_vec();
    let mut state = 0x1734_abcd_u64;
    for case in 0..32 {
        for i in (1..faces.len()).rev() {
            state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
            faces.swap(i, (state % (i as u64 + 1)) as usize);
        }
        let mesh = TriangleMesh::new(source.positions.to_vec(), faces.clone());
        let (region, strings, paths) = solid::slice_z(&mesh, 1.into())
            .attempt()
            .unwrap_or_else(|error| panic!("case={case}: {error:?}"));
        assert!(region.is_empty() && strings.is_empty() && paths.is_empty());
    }
}
