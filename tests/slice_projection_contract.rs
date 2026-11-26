//! Exact slice/projection semantics and visible failures through public modeling APIs.
#![cfg(feature = "curve")]

#[path = "support/uncertainty.rs"]
mod uncertainty;

use csgrs::errors::{ValidationError, ValidationErrorKind};
use csgrs::{GeometryCertainty, GeometryContext, Real, TriangleMesh, curve, solid};
use hypercurve::{BooleanOp, CurveRegion2};
use hyperlattice::{Matrix4, Point3};
use solid::cache::{self, CachePolicy};

/// Adapts historical infallible and current fallible results to the same assertions.
///
/// An empty fallback from an infallible API remains successful, so a regression
/// cannot satisfy an assertion that expects an explicit error.
trait Attempt<T> {
    /// Preserves successful geometry and converts only an existing error.
    fn attempt(self) -> Result<T, ValidationError>;
}
impl Attempt<CurveRegion2> for CurveRegion2 {
    /// Wraps a region, including an empty region, as success.
    fn attempt(self) -> Result<CurveRegion2, ValidationError> {
        Ok(self)
    }
}
impl Attempt<solid::SliceResult> for solid::SliceResult {
    /// Wraps all three section components as success without inspecting them.
    fn attempt(self) -> Result<solid::SliceResult, ValidationError> {
        Ok(self)
    }
}
impl<T, E: Into<ValidationError>> Attempt<T> for Result<T, E> {
    /// Converts the error type while preserving the original success or failure.
    fn attempt(self) -> Result<T, ValidationError> {
        self.map_err(Into::into)
    }
}
/// Obtains a mathematically zero expression that remains unresolved by the strict policy.
///
/// The shared helper checks the legacy trigonometric identity before returning
/// its independent terminal-zero witness.
fn unknown_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    uncertainty::terminal_zero(&sine * &sine + &cosine * &cosine - Real::one())
}
/// Returns the exact rational one half for interior membership probes.
fn half() -> Real {
    (Real::one() / Real::from(2)).unwrap()
}
/// Builds a single face with indices 0, 1, 2 over arbitrary fixture positions.
///
/// Deliberately skips validation so callers can construct malformed inputs.
fn triangle(points: Vec<Point3>) -> TriangleMesh {
    let cube = solid::cube(1).unwrap();
    let mut face = cube.triangles[0];
    face.v0 = 0;
    face.v1 = 1;
    face.v2 = 2;
    TriangleMesh::new(points, vec![face])
}
/// Sums exact oriented tetrahedron determinants, equal to six times signed volume.
///
/// This algebraic sum alone does not establish closure or the represented set.
fn volume_six(mesh: &TriangleMesh) -> Real {
    mesh.triangles.iter().fold(Real::zero(), |sum, face| {
        let [a, b, c] = face.indices().map(|i| mesh.positions[i].to_vector());
        sum + a.dot(&b.cross(&c))
    })
}
/// Extrudes a 4-by-4 square with a centered 2-by-2 hole through height two.
///
/// Checks area 12, certified construction, signed volume 24 and geometric
/// manifold admission before returning the fixture.
fn hollow_prism() -> TriangleMesh {
    let outer = curve::square(4).unwrap();
    let hole = curve::translated(&curve::square(2).unwrap(), 1, 1).unwrap();
    let profile = outer.boolean_region(&hole, BooleanOp::Difference).unwrap();
    assert_eq!(profile.filled_area().unwrap(), Some(12.into()));
    let mesh = curve::try_extrude(&profile, 2.into(), GeometryContext::STRICT).unwrap();
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
/// Checks the hollow section through area, two membership probes and re-extrusion.
///
/// Height-three extrusion must be certified, have signed volume 36 and pass
/// geometric manifold admission. Area and probes alone are not set equality.
fn assert_hollow(region: &CurveRegion2) {
    assert_eq!(
        region.filled_area().unwrap(),
        Some(12.into()),
        "a section filled the hole"
    );
    assert_eq!(curve::contains_xy(region, 2, 2).unwrap(), Some(false));
    assert_eq!(
        curve::contains_xy(region, half(), half()).unwrap(),
        Some(true)
    );
    let mesh = curve::try_extrude(region, 3.into(), GeometryContext::STRICT).unwrap();
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
/// Requires invalid-index slicing to return invalid input rather than empty success.
fn malformed_slice_is_an_error_not_empty_geometry() {
    let error = solid::slice_z(&triangle(Vec::new()), 0)
        .attempt()
        .unwrap_err();
    assert_eq!(error.kind(), ValidationErrorKind::InvalidInput);
}
#[test]
/// Requires invalid-index projection to return invalid input rather than empty success.
fn malformed_projection_is_an_error_not_empty_geometry() {
    let error = solid::flatten(&triangle(Vec::new())).attempt().unwrap_err();
    assert_eq!(error.kind(), ValidationErrorKind::InvalidInput);
}
#[test]
/// Requires an unresolved section height to remain an explicit strict-policy failure.
fn undecided_slice_is_an_error_not_empty_geometry() {
    let mesh = solid::cube(1).unwrap();
    let error = solid::slice_z(&mesh, unknown_zero()).attempt().unwrap_err();
    assert_eq!(error.kind(), ValidationErrorKind::Undecided);
}
#[test]
/// Requires an unresolved projected triangle orientation to remain undecided.
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
/// Checks an interior hollow section and its extrusion, with no residual open components.
fn hollow_interior_slice_keeps_hole_and_composes() {
    let (region, strings, paths) = solid::slice_z(&hollow_prism(), 1).attempt().unwrap();
    assert_hollow(&region);
    assert!(strings.is_empty() && paths.is_empty());
}
#[test]
/// Checks the bottom coplanar cap retains the hollow section and composes by extrusion.
fn coplanar_bottom_slice_keeps_hole_and_composes() {
    let (region, strings, paths) = solid::slice_z(&hollow_prism(), 0).attempt().unwrap();
    assert_hollow(&region);
    assert!(strings.is_empty() && paths.is_empty());
}
#[test]
/// Checks the top coplanar cap retains the hollow section and composes by extrusion.
fn coplanar_top_slice_keeps_hole_and_composes() {
    let (region, strings, paths) = solid::slice_z(&hollow_prism(), 2).attempt().unwrap();
    assert_hollow(&region);
    assert!(strings.is_empty() && paths.is_empty());
}
#[test]
/// Reverses face order and every triangle winding, then checks the hollow interior section.
fn face_permutation_and_global_reversal_preserve_hollow_set() {
    let mesh = hollow_prism();
    let mut faces = mesh.triangles.to_vec();
    faces.reverse();
    for face in &mut faces {
        std::mem::swap(&mut face.v1, &mut face.v2);
    }
    let reversed = TriangleMesh::new(mesh.positions.to_vec(), faces);
    assert_hollow(&solid::slice_z(&reversed, 1).attempt().unwrap().0);
}
#[test]
/// Checks nested positive shells retain area 16 and material at their common center.
fn nested_material_shells_do_not_become_parity_holes() {
    let outer = solid::cube(4).unwrap();
    let inner = solid::transform(
        &solid::cube(2).unwrap(),
        &Matrix4::affine_translation([1.into(), 1.into(), 1.into()]),
    )
    .unwrap();
    let mesh = solid::merge(&[outer, inner]).expect("valid concatenation fixture");
    let (region, strings, paths) = solid::slice_z(&mesh, 2).attempt().unwrap();
    assert_eq!(region.filled_area().unwrap(), Some(16.into()));
    assert_eq!(curve::contains_xy(&region, 2, 2).unwrap(), Some(true));
    assert!(strings.is_empty() && paths.is_empty());
}
#[test]
/// Checks coincident positive shells retain a filled square under nonzero winding.
fn duplicate_material_shells_do_not_cancel() {
    let cube = solid::cube(2).unwrap();
    let mesh = solid::merge(&[cube.clone(), cube]).expect("valid concatenation fixture");
    let (region, strings, paths) = solid::slice_z(&mesh, 1).attempt().unwrap();
    assert_eq!(region.filled_area().unwrap(), Some(4.into()));
    assert_eq!(curve::contains_xy(&region, 1, 1).unwrap(), Some(true));
    assert!(strings.is_empty() && paths.is_empty());
}
#[test]
/// Checks two disconnected hollow sections have total area 24 and two empty centers.
fn disconnected_hollow_components_preserve_both_voids() {
    let first = hollow_prism();
    let second = solid::transform(
        &first,
        &Matrix4::affine_translation([6.into(), 0.into(), 0.into()]),
    )
    .unwrap();
    let mesh = solid::merge(&[first, second]).expect("valid concatenation fixture");
    let (region, strings, paths) = solid::slice_z(&mesh, 1).attempt().unwrap();
    assert_eq!(region.filled_area().unwrap(), Some(24.into()));
    for x in [2, 8] {
        assert_eq!(curve::contains_xy(&region, x, 2).unwrap(), Some(false));
    }
    assert!(strings.is_empty() && paths.is_empty());
}
#[test]
/// Checks empty sections from empty and missed solids compose with union and extrusion.
///
/// Also checks projecting the canonical empty mesh returns an empty region.
fn canonical_empty_and_outside_slices_remain_consumable() {
    let empty = TriangleMesh::new(Vec::new(), Vec::new());
    let cube = solid::cube(1).unwrap();
    for mesh in [&empty, &cube] {
        let (region, strings, paths) = solid::slice_z(mesh, 2).attempt().unwrap();
        assert!(region.is_empty() && strings.is_empty() && paths.is_empty());
        let square = curve::square(1).unwrap();
        assert_eq!(
            region
                .boolean_region(&square, BooleanOp::Union)
                .unwrap()
                .filled_area()
                .unwrap(),
            Some(1.into())
        );
        assert!(
            curve::try_extrude(&region, 1.into(), GeometryContext::STRICT)
                .unwrap()
                .value
                .triangles
                .is_empty()
        );
    }
    assert!(solid::flatten(&empty).attempt().unwrap().is_empty());
}
#[test]
/// Checks an open triangle produces one segment with exact unordered endpoints.
///
/// The filled region and path list remain empty.
fn open_triangle_intersection_remains_an_open_wire() {
    let mesh = triangle(vec![
        Point3::new(0.into(), 0.into(), (-1).into()),
        Point3::new(2.into(), 0.into(), 1.into()),
        Point3::new(0.into(), 2.into(), 1.into()),
    ]);
    let (region, strings, paths) = solid::slice_z(&mesh, 0).attempt().unwrap();
    assert!(region.is_empty() && paths.is_empty());
    assert_eq!(strings.len(), 1);
    assert_eq!(strings[0].segments().len(), 1);
    let segment = &strings[0].segments()[0];
    let points = [segment.start(), segment.end()].map(|p| [p.x().clone(), p.y().clone()]);
    assert!(points.contains(&[1.into(), 0.into()]));
    assert!(points.contains(&[0.into(), 1.into()]));
}
#[test]
/// Checks projection of the hollow prism retains its void and supports extrusion.
fn projection_of_hollow_prism_keeps_void() {
    assert_hollow(&solid::flatten(&hollow_prism()).attempt().unwrap());
}
#[test]
/// Checks slice reuse and eviction under disabled and one-entry cache policies.
///
/// Invalidated position storage must fail slicing and projection, while the
/// unchanged source still yields the hollow section.
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
                assert_hollow(&solid::slice_z(&mesh, 1).attempt().unwrap().0);
            }
            assert!(solid::slice_z(&mesh, 3).attempt().unwrap().0.is_empty());
            assert_hollow(&solid::slice_z(&mesh, 1).attempt().unwrap().0);
            let mut changed = mesh.clone();
            changed.positions = Vec::new().into();
            assert!(solid::slice_z(&changed, 1).attempt().is_err());
            assert!(solid::flatten(&changed).attempt().is_err());
            assert_hollow(&solid::slice_z(&mesh, 1).attempt().unwrap().0);
        });
    }
}
#[test]
/// Checks strict/approximate/strict slicing preserves undecided failures and qualification.
fn approximate_slice_never_certifies_a_later_strict_request() {
    let mesh = solid::cube(1).unwrap();
    let z = unknown_zero();
    for context in [
        GeometryContext::STRICT,
        GeometryContext::APPROXIMATE_512,
        GeometryContext::STRICT,
    ] {
        let result = solid::slice_z_with_context(&mesh, z.clone(), context);
        if context == GeometryContext::STRICT {
            assert_eq!(result.unwrap_err().kind(), ValidationErrorKind::Undecided);
        } else {
            let result = result.unwrap();
            assert_eq!(result.certainty, GeometryCertainty::Approximate512Consumed);
            assert_eq!(result.value.0.filled_area().unwrap(), Some(1.into()));
        }
    }
}

/// Copies positions and face rows, swapping each face winding for cancellation fixtures.
fn reversed(mesh: &TriangleMesh) -> TriangleMesh {
    let mut faces = mesh.triangles.to_vec();
    for face in &mut faces {
        std::mem::swap(&mut face.v1, &mut face.v2);
    }
    TriangleMesh::new(mesh.positions.to_vec(), faces)
}

#[test]
/// Checks opposite coincident shells cancel at both caps and the interior section.
fn opposite_shells_cancel_interior_and_coplanar_sections() {
    let cube = solid::cube(2).unwrap();
    let mesh =
        solid::merge(&[cube.clone(), reversed(&cube)]).expect("valid concatenation fixture");
    for z in [0, 1, 2] {
        let (region, strings, paths) = solid::slice_z(&mesh, z).attempt().unwrap();
        assert!(region.is_empty(), "opposite winding at Z={z}");
        assert!(strings.is_empty() && paths.is_empty());
    }
}

#[test]
/// Checks two positive shells minus one opposite shell retain square area at three heights.
fn signed_multiplicity_is_preserved_interior_and_coplanar() {
    let cube = solid::cube(2).unwrap();
    let mesh = solid::merge(&[cube.clone(), cube.clone(), reversed(&cube)])
        .expect("valid concatenation fixture");
    for z in [0, 1, 2] {
        let (region, strings, paths) = solid::slice_z(&mesh, z).attempt().unwrap();
        assert_eq!(region.filled_area().unwrap(), Some(4.into()));
        assert!(strings.is_empty() && paths.is_empty());
    }
}

#[test]
/// Checks the shared cap of stacked hollow prisms retains the hole and no open remnants.
fn shared_on_plane_edges_do_not_change_hole_winding() {
    let lower = hollow_prism();
    let upper = solid::transform(
        &lower,
        &Matrix4::affine_translation([0.into(), 0.into(), 2.into()]),
    )
    .unwrap();
    let mesh = solid::merge(&[lower, upper]).expect("valid concatenation fixture");
    let (region, strings, paths) = solid::slice_z(&mesh, 2).attempt().unwrap();
    assert_hollow(&region);
    assert!(strings.is_empty() && paths.is_empty());
}

#[test]
/// Checks square sections touching at one vertex retain both interiors and total area eight.
fn vertex_touching_components_retain_both_material_regions() {
    let first = solid::cube(2).unwrap();
    let second = solid::transform(
        &first,
        &Matrix4::affine_translation([2.into(), 2.into(), 0.into()]),
    )
    .unwrap();
    let mesh = solid::merge(&[first, second]).expect("valid concatenation fixture");
    let (region, strings, paths) = solid::slice_z(&mesh, 1).attempt().unwrap();
    assert_eq!(region.filled_area().unwrap(), Some(8.into()));
    for (x, y, inside) in [(1, 1, true), (3, 3, true), (1, 3, false), (3, 1, false)] {
        assert_eq!(curve::contains_xy(&region, x, y).unwrap(), Some(inside));
    }
    assert!(strings.is_empty() && paths.is_empty());
}

#[test]
/// Checks both policies retain the native mesh error and its source for malformed indices.
fn contextual_invalid_indices_preserve_the_native_diagnostic() {
    let mesh = triangle(Vec::new());
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        for error in [
            solid::flatten_with_context(&mesh, context).unwrap_err(),
            solid::slice_z_with_context(&mesh, 0.into(), context).unwrap_err(),
        ] {
            assert!(matches!(error, ValidationError::Mesh(_)));
            assert_eq!(error.kind(), ValidationErrorKind::InvalidInput);
            assert!(std::error::Error::source(&error).is_some());
        }
    }
}

#[test]
/// Checks signed multiplicity after interleaving face rows and cyclically permuting them.
///
/// One positive copy cancels; two retain area four at interior and cap heights.
fn interleaved_opposite_faces_preserve_winding_after_permutation() {
    let cube = solid::cube(2).unwrap();
    for copies in [1, 2] {
        let mut operands = vec![cube.clone(); copies];
        operands.push(reversed(&cube));
        let mesh = solid::merge(&operands).expect("valid concatenation fixture");
        let n = cube.triangles.len();
        let mut faces = (0..n)
            .flat_map(|i| (0..operands.len()).map(move |j| j * n + i))
            .map(|i| mesh.triangles[i])
            .collect::<Vec<_>>();
        for shift in 0..n {
            faces.rotate_left(1);
            let permuted = TriangleMesh::new(mesh.positions.to_vec(), faces.clone());
            for z in [0, 1, 2] {
                let (region, strings, paths) = solid::slice_z(&permuted, z)
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
/// Checks hollow-section witnesses in separate workers using default and disabled caches.
fn thread_local_cache_modes_preserve_hollow_sections() {
    let workers = [CachePolicy::DEFAULT, CachePolicy::DISABLED].map(|policy| {
        std::thread::spawn(move || {
            cache::with_policy(policy, || {
                let mesh = hollow_prism();
                assert_hollow(&solid::slice_z(&mesh, 1).attempt().unwrap().0);
            })
        })
    });
    for worker in workers {
        worker.join().unwrap();
    }
}

#[test]
/// Duplicates positions per face and checks coordinate incidence still assembles hollow sections.
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
        let (region, strings, paths) = solid::slice_z(&mesh, z).attempt().unwrap();
        assert_hollow(&region);
        assert!(strings.is_empty() && paths.is_empty());
    }
}

#[test]
/// Checks cancellation survives 32 deterministic shuffles of coincident opposite shell faces.
fn shuffled_coincident_shell_faces_preserve_signed_set() {
    let cube = solid::cube(2).unwrap();
    let source =
        solid::merge(&[cube.clone(), reversed(&cube)]).expect("valid concatenation fixture");
    let mut faces = source.triangles.to_vec();
    let mut state = 0x1734_abcd_u64;
    for case in 0..32 {
        for i in (1..faces.len()).rev() {
            state = state.wrapping_mul(6364136223846793005).wrapping_add(1);
            faces.swap(i, (state % (i as u64 + 1)) as usize);
        }
        let mesh = TriangleMesh::new(source.positions.to_vec(), faces.clone());
        let (region, strings, paths) = solid::slice_z(&mesh, 1)
            .attempt()
            .unwrap_or_else(|error| panic!("case={case}: {error:?}"));
        assert!(region.is_empty() && strings.is_empty() && paths.is_empty());
    }
}
