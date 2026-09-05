use csgrs::{GeometryCertainty, GeometryContext, Real, TriangleMesh, solid};
use hyperlattice::{Matrix4, Point3, Vector3};

const STRICT: GeometryContext = GeometryContext::STRICT;
const APPROXIMATE: GeometryContext = GeometryContext::APPROXIMATE_512;

fn unresolved_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    &sine * &sine + &cosine * &cosine - Real::one()
}

fn uncertain_bounds() -> TriangleMesh {
    let a = Real::e().sin();
    let b = &a + unresolved_zero();
    TriangleMesh::new(
        vec![
            Point3::new(a.clone(), Real::zero(), a),
            Point3::new(b.clone(), Real::one(), b),
        ],
        Vec::new(),
    )
}

#[test]
fn bounds_and_positioning_propagate_policy_and_certainty() {
    let mesh = uncertain_bounds();
    assert!(solid::try_bounding_box(&mesh).is_err());
    assert!(solid::try_center(&mesh).is_err());
    assert!(solid::try_float(&mesh).is_err());
    let bounds = solid::try_bounding_box_with_context(&mesh, &APPROXIMATE).unwrap();
    assert_eq!(bounds.certainty, GeometryCertainty::Approximate512Consumed);
    for positioned in [
        solid::try_center_with_context(&mesh, &APPROXIMATE).unwrap(),
        solid::try_float_with_context(&mesh, &APPROXIMATE).unwrap(),
    ] {
        assert_eq!(
            positioned.certainty,
            GeometryCertainty::Approximate512Consumed
        );
        assert_eq!(positioned.value.positions.len(), 2);
    }
    assert!(solid::try_bounding_box_with_context(&mesh, &STRICT).is_err());

    let adapted = csgrs::adapter::RawTriangleMesh::from_native(mesh);
    assert!(adapted.bounding_box().is_err());
    assert_eq!(
        adapted
            .bounding_box_with_context(&APPROXIMATE)
            .unwrap()
            .certainty,
        GeometryCertainty::Approximate512Consumed
    );
    assert_eq!(
        adapted.center_with_context(&APPROXIMATE).unwrap().certainty,
        GeometryCertainty::Approximate512Consumed
    );
}

#[test]
fn transformation_policy_does_not_allow_collapsed_geometry() {
    let mesh = solid::cube(Real::one());
    let zero = unresolved_zero();
    assert!(
        solid::try_scale_with_context(&mesh, zero.clone(), Real::one(), Real::one(), &STRICT)
            .is_err()
    );
    assert!(matches!(
        solid::try_scale_with_context(&mesh, zero, Real::one(), Real::one(), &APPROXIMATE),
        Err(csgrs::errors::ValidationError::InvalidArguments)
    ));
    let mirror = hypermesh::Plane::axis_aligned(0, Real::zero());
    for context in [STRICT, APPROXIMATE] {
        for outcome in [
            solid::try_transform_with_context(
                &mesh,
                &Matrix4::affine_nonuniform_scale([-Real::one(), Real::one(), Real::one()]),
                &context,
            )
            .unwrap(),
            solid::try_scale_with_context(
                &mesh,
                -Real::one(),
                Real::one(),
                Real::one(),
                &context,
            )
            .unwrap(),
            solid::try_mirror_with_context(&mesh, &mirror, &context).unwrap(),
            solid::try_rotate_with_context(
                &mesh,
                Real::zero(),
                Real::zero(),
                Real::from(90),
                &context,
            )
            .unwrap(),
        ] {
            assert_eq!(outcome.certainty, GeometryCertainty::Certified);
            assert!(
                outcome
                    .value
                    .is_closed_manifold_geometry(&context.mesh_context())
                    .unwrap()
                    .value
            );
        }
    }
}

#[test]
fn hulls_and_queries_accept_terminal_equality() {
    let cube = solid::cube(Real::one());
    let mut positions = cube.positions.to_vec();
    positions.push(Point3::new(unresolved_zero(), Real::zero(), Real::zero()));
    let mesh = TriangleMesh::new(positions, cube.triangles.to_vec());
    assert!(solid::convex_hull(&mesh).is_err());
    let hull = solid::convex_hull_with_context(&mesh, &APPROXIMATE).unwrap();
    assert_eq!(hull.certainty, GeometryCertainty::Approximate512Consumed);
    assert!(hull.value.is_closed_manifold());
    assert!(solid::minkowski_sum_with_context(&mesh, &cube, &STRICT).is_err());
    let sum = solid::minkowski_sum_with_context(&mesh, &cube, &APPROXIMATE).unwrap();
    assert_eq!(sum.certainty, GeometryCertainty::Approximate512Consumed);
    assert!(sum.value.is_closed_manifold());

    let half = (Real::one() / Real::from(2)).unwrap();
    let point = Point3::new(unresolved_zero(), half.clone(), half.clone());
    assert!(solid::contains_point(&cube, &point).is_err());
    let inside = solid::contains_point_with_context(&cube, &point, &APPROXIMATE).unwrap();
    assert_eq!(
        inside.value,
        solid::contains_point(&cube, &Point3::new(Real::zero(), half.clone(), half.clone()))
            .unwrap()
    );
    assert_eq!(inside.certainty, GeometryCertainty::Approximate512Consumed);
    let line = [
        Point3::new(Real::from(-1), half.clone(), half.clone()),
        point,
        Point3::new(Real::from(2), half.clone(), half),
    ];
    assert!(solid::polyline_intersections(&cube, &line).is_err());
    let hits = solid::polyline_intersections_with_context(&cube, &line, &APPROXIMATE).unwrap();
    assert_eq!(hits.certainty, GeometryCertainty::Approximate512Consumed);
    assert_eq!(hits.value.len(), 2);
}

#[test]
fn distributions_do_not_reuse_approximate_results_for_strict_calls() {
    let cube = solid::cube(Real::one());
    let step = Real::one() + unresolved_zero();
    assert!(
        solid::distribute_grid_with_context(&cube, 1, 2, step.clone(), Real::one(), &STRICT)
            .is_err()
    );
    for outcome in [
        solid::distribute_grid_with_context(
            &cube,
            1,
            2,
            step.clone(),
            Real::one(),
            &APPROXIMATE,
        )
        .unwrap(),
        solid::distribute_linear_with_context(
            &cube,
            2,
            Vector3::x(),
            step.clone(),
            &APPROXIMATE,
        )
        .unwrap(),
    ] {
        assert_eq!(outcome.certainty, GeometryCertainty::Approximate512Consumed);
        assert!(outcome.value.is_closed_manifold());
    }
    let replay = solid::distribute_grid_with_context(
        &cube,
        1,
        2,
        step.clone(),
        Real::one(),
        &APPROXIMATE,
    )
    .unwrap();
    assert_eq!(replay.certainty, GeometryCertainty::Approximate512Consumed);
    assert!(
        solid::distribute_grid_with_context(&cube, 1, 2, step, Real::one(), &STRICT).is_err()
    );
    let arc = solid::distribute_arc_with_context(
        &cube,
        4,
        Real::from(4),
        Real::zero(),
        Real::from(270),
        &APPROXIMATE,
    )
    .unwrap();
    assert_eq!(arc.value.triangles.len(), 48);
}

#[test]
fn scalar_mesh_booleans_expose_context_without_losing_certainty() {
    use csgrs::adapter::TriangleMeshF64;
    let body = TriangleMeshF64::cuboid(12.0, 12.0, 4.0).unwrap();
    let drill = TriangleMeshF64::cylinder(2.0, 6.0, 16)
        .unwrap()
        .translate(6.0, 6.0, -1.0)
        .unwrap();
    assert!(body.difference(&drill).is_err());
    let result = body.difference_with_context(&drill, &APPROXIMATE).unwrap();
    assert_eq!(result.certainty, GeometryCertainty::Approximate512Consumed);
    assert!(result.value.native().is_closed_manifold());
}

#[cfg(feature = "curve")]
#[test]
fn slices_and_flattening_report_uncertainty_instead_of_empty_success() {
    let cube = solid::cube(Real::one());
    let z = unresolved_zero();
    assert!(solid::slice_z_with_context(&cube, z.clone(), &STRICT).is_err());
    let sliced = solid::slice_z_with_context(&cube, z.clone(), &APPROXIMATE).unwrap();
    assert_eq!(sliced.certainty, GeometryCertainty::Approximate512Consumed);
    assert!(!sliced.value.0.is_empty());
    assert_eq!(
        solid::slice_z_with_context(&cube, z.clone(), &APPROXIMATE)
            .unwrap()
            .certainty,
        GeometryCertainty::Approximate512Consumed
    );
    assert!(solid::slice_z_with_context(&cube, z, &STRICT).is_err());
    let invalid = TriangleMesh::new(Vec::new(), vec![hypermesh::Triangle::new(0, 1, 2)]);
    assert!(solid::flatten_with_context(&invalid, &APPROXIMATE).is_err());
    let projected = solid::flatten_with_context(&cube, &APPROXIMATE).unwrap();
    assert_eq!(projected.certainty, GeometryCertainty::Certified);
    assert!(!projected.value.is_empty());
}

#[cfg(feature = "curve")]
#[test]
fn curve_queries_preserve_boundary_and_uncertainty_distinctions() {
    let square = csgrs::curve::square(Real::one());
    let half = (Real::one() / Real::from(2)).unwrap();
    assert!(
        csgrs::curve::contains_xy_with_context(
            &square,
            unresolved_zero(),
            half.clone(),
            &STRICT
        )
        .is_err()
    );
    let boundary = csgrs::curve::contains_xy_with_context(
        &square,
        unresolved_zero(),
        half.clone(),
        &APPROXIMATE,
    )
    .unwrap();
    assert_eq!(boundary.value, None);
    assert_eq!(boundary.certainty, GeometryCertainty::Approximate512Consumed);
    for context in [STRICT, APPROXIMATE] {
        let inside = csgrs::curve::contains_xy_with_context(
            &square,
            half.clone(),
            half.clone(),
            &context,
        )
        .unwrap();
        assert_eq!(inside.value, Some(true));
        assert_eq!(inside.certainty, GeometryCertainty::Certified);
    }
}

#[cfg(feature = "sdf")]
#[test]
fn sdf_meshing_consumes_the_selected_policy() {
    let point = |v| Point3::new(Real::from(v), Real::from(v), Real::from(v));
    let field = |p: &Point3| p.x.clone() + unresolved_zero();
    assert!(
        solid::sdf_with_context(field, (3, 3, 3), point(-1), point(1), Real::zero(), &STRICT)
            .is_err()
    );
    let result = solid::sdf_with_context(
        field,
        (3, 3, 3),
        point(-1),
        point(1),
        Real::zero(),
        &APPROXIMATE,
    )
    .unwrap();
    assert_eq!(result.certainty, GeometryCertainty::Approximate512Consumed);
    assert!(!result.value.triangles.is_empty());
}

#[test]
fn polyhedron_and_loft_use_context_for_planarity_and_caps() {
    let points = [
        [Real::zero(), Real::zero(), Real::zero()],
        [Real::one(), Real::zero(), Real::zero()],
        [Real::one(), Real::one(), Real::zero()],
        [Real::zero(), Real::one(), unresolved_zero()],
    ];
    assert!(solid::polyhedron_with_context(&points, &[&[0, 1, 2, 3]], &STRICT).is_err());
    let face =
        solid::polyhedron_with_context(&points, &[&[0, 1, 2, 3]], &APPROXIMATE).unwrap();
    assert_eq!(face.certainty, GeometryCertainty::Approximate512Consumed);
    assert_eq!(face.value.triangles.len(), 2);

    // Planarity is certified here; the undecided equality is inside Hypertri.
    let left = Real::from(3) + unresolved_zero();
    let triangulated_points = [
        [left.clone(), Real::zero(), Real::zero()],
        [&left + Real::one(), Real::zero(), Real::zero()],
        [&left + Real::one(), Real::one(), Real::zero()],
        [left, Real::one(), Real::zero()],
        [Real::from(3), Real::zero(), Real::zero()],
    ];
    assert!(
        solid::polyhedron_with_context(&triangulated_points, &[&[0, 1, 2, 3, 4]], &STRICT)
            .is_err()
    );
    let triangulated = solid::polyhedron_with_context(
        &triangulated_points,
        &[&[0, 1, 2, 3, 4]],
        &APPROXIMATE,
    )
    .unwrap();
    assert_eq!(
        triangulated.certainty,
        GeometryCertainty::Approximate512Consumed
    );
    assert_eq!(triangulated.value.triangles.len(), 2);
    let bottom = points
        .into_iter()
        .map(|[x, y, z]| Point3::new(x, y, z))
        .collect::<Vec<_>>();
    let top = bottom
        .iter()
        .map(|point| Point3::new(point.x.clone(), point.y.clone(), &point.z + Real::one()))
        .collect::<Vec<_>>();
    let sections = [bottom, top];
    assert!(solid::loft_with_context(&sections, &STRICT).is_err());
    let loft = solid::loft_with_context(&sections, &APPROXIMATE).unwrap();
    assert_eq!(loft.certainty, GeometryCertainty::Approximate512Consumed);
    assert_eq!(loft.value.triangles.len(), 12);
    assert!(
        loft.value
            .is_closed_manifold_geometry(&APPROXIMATE.mesh_context())
            .unwrap()
            .value
    );
}

#[cfg(feature = "curve")]
#[test]
fn composite_curve_constructors_preserve_nested_predicate_certainty() {
    let distance = unresolved_zero();
    assert!(
        csgrs::curve::circle_with_flat(Real::one(), 8, distance.clone(), &STRICT).is_err()
    );
    let clipped =
        csgrs::curve::circle_with_flat(Real::one(), 8, distance, &APPROXIMATE).unwrap();
    assert_eq!(clipped.certainty, GeometryCertainty::Approximate512Consumed);
    assert!(!clipped.value.is_empty());
    let bounds =
        csgrs::curve::try_bounding_box_with_context(&clipped.value, &APPROXIMATE).unwrap();
    assert_eq!(bounds.value.mins.y.to_f64_lossy().unwrap(), 0.0);
    assert_eq!(bounds.value.maxs.y.to_f64_lossy().unwrap(), 1.0);

    let rounded = csgrs::curve::rounded_rectangle_with_context(
        Real::from(2),
        Real::one(),
        unresolved_zero(),
        8,
        &APPROXIMATE,
    )
    .unwrap();
    assert_eq!(rounded.certainty, GeometryCertainty::Approximate512Consumed);
    assert!(!rounded.value.is_empty());
}

#[cfg(feature = "wasm")]
#[test]
fn javascript_mesh_results_expose_the_consumed_certainty() {
    use csgrs::solid::SolidExt;
    use csgrs::wasm::mesh_js::MeshJs;
    let body = MeshJs::from(solid::cuboid(Real::from(12), Real::from(12), Real::from(4)));
    let drill = MeshJs::from(solid::cylinder(Real::from(2), Real::from(6), 16).translated(
        Real::from(6),
        Real::from(6),
        Real::from(-1),
    ));
    let result = body.difference_with_context(&drill, true).unwrap();
    assert!(result.approximate_512_consumed());
    assert_eq!(result.into_mesh().triangle_count(), 144);
    let bounds = MeshJs::from(uncertain_bounds())
        .bounding_box_with_context(true)
        .unwrap();
    assert!(bounds.approximate_512_consumed());
}

#[cfg(feature = "obj-io")]
#[test]
fn obj_context_preserves_certified_concave_triangulation_and_parse_errors() {
    let source = b"v 0 0 0\nv 2 0 0\nv 2 2 0\nv 1 1 0\nv 0 2 0\nf 1 2 3 4 5\n";
    for context in [STRICT, APPROXIMATE] {
        let imported = csgrs::io::obj::from_obj_with_context(&source[..], &context).unwrap();
        assert_eq!(imported.certainty, GeometryCertainty::Certified);
        assert_eq!(imported.value.triangles.len(), 3);
        assert!(
            csgrs::io::obj::from_obj_with_context(&b"v 0 0 0\nf 1 2 3\n"[..], &context)
                .is_err()
        );
        #[cfg(feature = "attributed")]
        assert_eq!(
            csgrs::io::obj::from_obj_attributed_with_context(&source[..], &context)
                .unwrap()
                .certainty,
            GeometryCertainty::Certified
        );
    }
}

#[cfg(feature = "dxf-io")]
#[test]
fn dxf_context_preserves_exact_imports() {
    let source = csgrs::io::dxf::to_dxf(&solid::cube(Real::one())).unwrap();
    for context in [STRICT, APPROXIMATE] {
        let imported = csgrs::io::dxf::from_dxf_with_context(&source, &context).unwrap();
        assert_eq!(imported.certainty, GeometryCertainty::Certified);
        assert_eq!(imported.value.triangles.len(), 12);
    }
}

#[cfg(feature = "vrml-io")]
#[test]
fn vrml_context_reaches_transformed_polygon_triangulation() {
    let source = br"#VRML V2.0 utf8
Transform {
  rotation 1 2 3 0.37
  children [ Shape { geometry IndexedFaceSet {
    coord Coordinate { point [ 0 0 0, 2 0 0, 2 2 0, 1 1 0, 0 2 0 ] }
    coordIndex [ 0 1 2 3 4 -1 ]
  } } ]
}";
    let imported = csgrs::io::vrml::from_vrml_with_context(source, &APPROXIMATE).unwrap();
    assert!(csgrs::io::vrml::from_vrml(source).is_err());
    assert_eq!(imported.certainty, GeometryCertainty::Approximate512Consumed);
    assert_eq!(imported.value.mesh.triangles.len(), 3);
    assert_eq!(imported.value.ignored_degenerate_polygon_count, 0);
    assert_eq!(imported.value.ignored_degenerate_triangle_count, 0);
}

#[cfg(feature = "gerber-io")]
#[test]
fn gerber_context_reaches_aperture_composition_and_projection() {
    use csgrs::io::gerber::{self, GerberExportOptions};
    let source =
        b"%FSLAX46Y46*%\n%MOMM*%\n%ADD10C,2*%\nD10*\nX0Y0D03*\nX1000000Y0D03*\nM02*\n";
    let imported = gerber::import_gerber_with_context(source, &APPROXIMATE).unwrap();
    assert_eq!(imported.certainty, GeometryCertainty::Certified);
    assert!(!gerber::import_gerber(source).unwrap().0.is_empty());
    assert!(!imported.value.0.is_empty());
    let exported = gerber::export_gerber_with_options_and_context(
        &imported.value.0,
        GerberExportOptions::default(),
        &APPROXIMATE,
    )
    .unwrap();
    assert!(!exported.value.is_empty());
    assert!(
        !gerber::import_gerber_with_context(&exported.value, &APPROXIMATE)
            .unwrap()
            .value
            .0
            .is_empty()
    );
    assert!(gerber::import_gerber_with_context(b"invalid", &APPROXIMATE).is_err());
}
