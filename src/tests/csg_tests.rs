//! Unit tests for core CSG operations.

use super::support::*;

fn oriented_volume_six(mesh: &Mesh<()>) -> Real {
    mesh.polygons
        .iter()
        .flat_map(|polygon| {
            (1..polygon.vertices.len() - 1).map(move |index| {
                let a = &polygon.vertices[0].position;
                let b = &polygon.vertices[index].position;
                let c = &polygon.vertices[index + 1].position;
                a.x.clone() * (b.y.clone() * c.z.clone() - b.z.clone() * c.y.clone())
                    + a.y.clone() * (b.z.clone() * c.x.clone() - b.x.clone() * c.z.clone())
                    + a.z.clone() * (b.x.clone() * c.y.clone() - b.y.clone() * c.x.clone())
            })
        })
        .fold(Real::zero(), |volume, tetrahedron| volume + tetrahedron)
}

#[test]
fn test_csg_from_polygons_and_to_polygons() {
    let poly: Polygon<()> = Polygon::new(
        vec![
            Vertex::new(Point3::origin(), Vector3::z()),
            Vertex::new(p3(1.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(p3(0.0, 1.0, 0.0), Vector3::z()),
        ],
        (),
    );
    let csg: Mesh<()> = Mesh::from_polygons(&[poly.clone()]);
    assert_eq!(csg.polygons.len(), 1);
    assert_eq!(csg.polygons[0].vertices.len(), 3);
}

#[test]
fn rotation_at_exact_degree_landmarks_matches_finite_trigonometry() {
    let source = Polygon::new(
        vec![
            Vertex::new(p3(10.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(p3(11.0, 0.0, 0.0), Vector3::z()),
            Vertex::new(p3(10.0, 1.0, 0.0), Vector3::z()),
        ],
        (),
    );
    let mesh = Mesh::from_polygons(&[source]);

    for degrees in [120.0_f64, 239.0, 240.0, 241.0] {
        let rotated = mesh.rotate(r(0.0), r(0.0), r(degrees));
        let point = &rotated.polygons[0].vertices[0].position;
        let radians = degrees.to_radians();
        let actual_x = point.x.to_f64_lossy().expect("finite rotated x");
        let actual_y = point.y.to_f64_lossy().expect("finite rotated y");

        assert!(
            (actual_x - 10.0 * radians.cos()).abs() < 1.0e-9,
            "x mismatch at {degrees} degrees: {actual_x}"
        );
        assert!(
            (actual_y - 10.0 * radians.sin()).abs() < 1.0e-9,
            "y mismatch at {degrees} degrees: {actual_y}"
        );
    }
}

#[test]
fn test_csg_union() {
    let cube1: Mesh<()> = Mesh::cube(r(2.0), ()).translate(r(-1.0), r(-1.0), r(-1.0)); // from -1 to +1 in all coords
    let cube2: Mesh<()> = Mesh::cube(r(1.0), ()).translate(r(0.5), r(0.5), r(0.5));

    let union_csg = cube1.union(&cube2);
    assert!(
        !union_csg.polygons.is_empty(),
        "Union of two cubes should produce polygons"
    );

    // Check bounding box => should now at least range from -1 to (0.5+1) = 1.5
    let bb = bounding_box(&union_csg.polygons);
    assert!(approx_eq(&bb[0], -1.0, 1e-8));
    assert!(approx_eq(&bb[1], -1.0, 1e-8));
    assert!(approx_eq(&bb[2], -1.0, 1e-8));
    assert!(approx_eq(&bb[3], 1.5, 1e-8));
    assert!(approx_eq(&bb[4], 1.5, 1e-8));
    assert!(approx_eq(&bb[5], 1.5, 1e-8));
}

#[test]
fn test_csg_difference() {
    // Subtract a smaller cube from a bigger one
    let big_cube: Mesh<()> = Mesh::cube(r(4.0), ()).translate(r(-2.0), r(-2.0), r(-2.0)); // radius=2 => spans [-2,2]
    let small_cube: Mesh<()> = Mesh::cube(r(2.0), ()).translate(r(-1.0), r(-1.0), r(-1.0)); // radius=1 => spans [-1,1]

    let result = big_cube.difference(&small_cube);
    assert!(
        !result.polygons.is_empty(),
        "Subtracting a smaller cube should leave polygons"
    );

    // Check bounding box => should still be [-2,-2,-2, 2,2,2], but with a chunk removed
    let bb = bounding_box(&result.polygons);
    // At least the bounding box remains the same
    assert!(approx_eq(&bb[0], -2.0, 1e-8));
    assert!(approx_eq(&bb[3], 2.0, 1e-8));
}

#[test]
fn test_csg_union2() {
    let c1: Mesh<()> = Mesh::cube(r(2.0), ());
    let c2: Mesh<()> = Mesh::cube(r(1.0), ()).translate(r(3.0), r(0.0), r(0.0));
    let result = c1
        .try_union(&c2)
        .expect("disjoint cube union should run through hypermesh");
    assert!(
        !result.polygons.is_empty(),
        "Union of cube and sphere should produce polygons"
    );
}

#[test]
fn mesh_union_uses_hypermesh_without_legacy_fallback() {
    let cube: Mesh<()> = Mesh::cube(r(2.0), ());
    let other: Mesh<()> = Mesh::cube(r(1.0), ()).translate(r(3.0), r(0.0), r(0.0));

    let result = cube
        .try_union(&other)
        .expect("hypermesh union should certify");
    assert!(
        !result.polygons.is_empty(),
        "Union of cube and sphere should produce polygons"
    );
}

#[test]
fn test_csg_intersect() {
    let c1: Mesh<()> = Mesh::cube(r(2.0), ());
    let c2: Mesh<()> = Mesh::cube(r(2.0), ()).translate(r(1.0), r(1.0), r(1.0));
    let result = c1
        .try_intersection(&c2)
        .expect("overlapping cube intersection should run through hypermesh");
    assert!(
        !result.polygons.is_empty(),
        "Intersection of cube and sphere should produce polygons"
    );
}

#[test]
fn test_csg_intersect2() {
    let left: Mesh<()> = Mesh::cube(r(1.0), ());
    let right: Mesh<()> = Mesh::cube(r(1.0), ()).translate(r(3.0), r(0.0), r(0.0));
    let result = left
        .try_intersection(&right)
        .expect("disjoint cube intersection should run through hypermesh");
    assert!(result.polygons.is_empty());
}

#[test]
fn test_csg_inverse() {
    let c1: Mesh<()> = Mesh::cube(r(2.0), ());
    let inv = c1.inverse();
    // The polygons are flipped
    // We can check just that the polygon planes are reversed, etc.
    // A full check might compare the polygons, but let's do a quick check on one polygon.
    let orig_poly = &c1.polygons[0];
    let inv_poly = &inv.polygons[0];
    assert!(approx_eq(
        orig_poly.plane.normal().0[0].clone(),
        -inv_poly.plane.normal().0[0].clone(),
        tolerance()
    ));
    assert!(approx_eq(
        orig_poly.plane.normal().0[1].clone(),
        -inv_poly.plane.normal().0[1].clone(),
        tolerance()
    ));
    assert!(approx_eq(
        orig_poly.plane.normal().0[2].clone(),
        -inv_poly.plane.normal().0[2].clone(),
        tolerance()
    ));
    assert_eq!(
        c1.polygons.len(),
        inv.polygons.len(),
        "Inverse should keep the same polygon count, but flip them"
    );
}

#[test]
fn test_csg_cube() {
    let c: Mesh<()> = Mesh::cube(r(2.0), ());
    // By default, corner at (0,0,0)
    // We expect 6 faces, each 4 vertices = 6 polygons
    assert_eq!(c.polygons.len(), 6);
    // Check bounding box
    let bb = c.bounding_box();
    assert!(approx_eq(&bb.mins.x, 0.0, tolerance()));
    assert!(approx_eq(&bb.maxs.x, 2.0, tolerance()));
}

// --------------------------------------------------------
//   CSG: Basic Shape Generation
// --------------------------------------------------------

#[test]
fn test_csg_sphere() {
    // Default sphere => radius=1, slices=16, stacks=8
    let sphere: Mesh<()> = Mesh::sphere(r(1.0), 16, 8, ());
    assert!(!sphere.polygons.is_empty(), "Sphere should generate polygons");

    let bb = bounding_box(&sphere.polygons);
    // Should roughly be [-1, -1, -1, 1, 1, 1]
    assert!(approx_eq(&bb[0], -1.0, 1e-1));
    assert!(approx_eq(&bb[1], -1.0, 1e-1));
    assert!(approx_eq(&bb[2], -1.0, 1e-1));
    assert!(approx_eq(&bb[3], 1.0, 1e-1));
    assert!(approx_eq(&bb[4], 1.0, 1e-1));
    assert!(approx_eq(&bb[5], 1.0, 1e-1));

    // Two 16-triangle caps plus two triangles for each interior UV cell.
    assert_eq!(sphere.polygons.len(), 2 * 16 * (8 - 1));
    assert!(
        sphere
            .polygons
            .iter()
            .all(|polygon| polygon.vertices.len() == 3)
    );
    assert!(oriented_volume_six(&sphere) > Real::zero());
}

#[test]
fn round_primitive_vertices_are_directly_exportable() {
    let meshes = [
        Mesh::sphere(r(2.0), 16, 8, ()),
        Mesh::frustum(r(2.0), r(1.0), r(3.0), 16, ()),
        Mesh::arrow(p3(1.0, 2.0, 3.0), v3(2.0, 3.0, 4.0), 16, false, ()),
    ];
    for mesh in meshes {
        for vertex in mesh.vertices() {
            for coordinate in [
                &vertex.position.x,
                &vertex.position.y,
                &vertex.position.z,
                &vertex.normal.0[0],
                &vertex.normal.0[1],
                &vertex.normal.0[2],
            ] {
                assert!(coordinate.to_f64_lossy().is_some());
            }
        }
    }
}

#[test]
fn test_csg_cylinder() {
    // Default cylinder => from (0,0,0) to (0,2,0) with radius=1
    let cylinder: Mesh<()> = Mesh::cylinder(r(1.0), r(2.0), 16, ());
    assert!(
        !cylinder.polygons.is_empty(),
        "Cylinder should generate polygons"
    );

    let bb = bounding_box(&cylinder.polygons);
    // Expect x in [-1,1], y in [-1,1], z in [-1,1].
    assert!(approx_eq(&bb[0], -1.0, 1e-8), "min X");
    assert!(approx_eq(&bb[1], -1.0, 1e-8), "min Y");
    assert!(approx_eq(&bb[2], 0.0, 1e-8), "min Z");
    assert!(approx_eq(&bb[3], 1.0, 1e-8), "max X");
    assert!(approx_eq(&bb[4], 1.0, 1e-8), "max Y");
    assert!(approx_eq(&bb[5], 2.0, 1e-8), "max Z");

    // We have slices = 16, plus 16*2 polygons for the end caps
    assert_eq!(cylinder.polygons.len(), 48);
}

#[test]
fn frustum_family_has_outward_winding() {
    let meshes = [
        Mesh::cylinder(r(1.0), r(2.0), 16, ()),
        Mesh::frustum(r(0.0), r(1.0), r(2.0), 16, ()),
        Mesh::frustum(r(1.0), r(0.0), r(2.0), 16, ()),
    ];

    for mesh in meshes {
        assert!(
            oriented_volume_six(&mesh) > Real::zero(),
            "closed frustum-family meshes must use outward winding"
        );
    }
}

#[test]
fn solid_primitives_reject_nonpositive_dimensions() {
    assert!(Mesh::<()>::cube(Real::zero(), ()).polygons.is_empty());
    assert!(
        Mesh::<()>::cuboid(r(1.0), r(-1.0), r(1.0), ())
            .polygons
            .is_empty()
    );
    assert!(Mesh::<()>::sphere(r(-1.0), 16, 8, ()).polygons.is_empty());
    assert!(
        Mesh::<()>::ellipsoid(r(1.0), Real::zero(), r(1.0), 16, 8, ())
            .polygons
            .is_empty()
    );
    assert!(Mesh::<()>::octahedron(Real::zero(), ()).polygons.is_empty());
    assert!(Mesh::<()>::icosahedron(r(-1.0), ()).polygons.is_empty());
    assert!(
        Mesh::<()>::cylinder(r(-1.0), r(2.0), 16, ())
            .polygons
            .is_empty()
    );
    assert!(
        Mesh::<()>::frustum(r(1.0), r(-1.0), r(2.0), 16, ())
            .polygons
            .is_empty()
    );
}

#[test]
fn icosahedron_radius_is_its_circumradius() {
    let radius = r(3.25);
    let mesh = Mesh::<()>::icosahedron(radius.clone(), ());
    assert!(!mesh.polygons.is_empty());

    for vertex in mesh.polygons.iter().flat_map(|polygon| &polygon.vertices) {
        let coordinates = [
            vertex.position.x.to_f64_lossy().unwrap_or(f64::NAN),
            vertex.position.y.to_f64_lossy().unwrap_or(f64::NAN),
            vertex.position.z.to_f64_lossy().unwrap_or(f64::NAN),
        ];
        let squared_distance = coordinates.iter().map(|value| value * value).sum::<f64>();
        assert!(
            (squared_distance - 3.25_f64.powi(2)).abs() < 1.0e-9,
            "squared distance was {squared_distance}, coordinates were {coordinates:?}"
        );
    }
}

#[test]
fn tapered_frustum_side_normals_include_axial_slope() {
    let mesh = Mesh::<()>::frustum(r(2.0), r(1.0), r(2.0), 16, ());
    assert!(!mesh.polygons.is_empty());

    for side in mesh.polygons.iter().skip(2).step_by(3) {
        for vertex in &side.vertices {
            assert!(
                vertex.normal.0[2] > Real::zero(),
                "a narrowing +Z frustum must have side normals pointing partly +Z"
            );
            let squared_length = vertex
                .normal
                .0
                .iter()
                .map(|component| {
                    component
                        .to_f64_lossy()
                        .expect("frustum normal component must be approximable")
                        .powi(2)
                })
                .sum::<f64>();
            assert!((squared_length - 1.0).abs() < 1.0e-9);
        }
    }
}

#[test]
fn representative_mesh_shape_constructors_produce_topology() {
    let meshes = [
        ("cuboid", Mesh::<()>::cuboid(r(2.0), r(3.0), r(4.0), ())),
        ("sphere", Mesh::<()>::sphere(r(2.0), 12, 6, ())),
        ("cylinder", Mesh::<()>::cylinder(r(2.0), r(4.0), 12, ())),
        ("frustum", Mesh::<()>::frustum(r(2.0), r(1.0), r(4.0), 12, ())),
        (
            "ellipsoid",
            Mesh::<()>::ellipsoid(r(2.0), r(3.0), r(4.0), 12, 6, ()),
        ),
        (
            "arrow",
            Mesh::<()>::arrow(p3(1.0, 2.0, 3.0), v3(2.0, 3.0, 4.0), 12, false, ()),
        ),
        ("octahedron", Mesh::<()>::octahedron(r(2.0), ())),
        ("icosahedron", Mesh::<()>::icosahedron(r(2.0), ())),
        ("torus", Mesh::<()>::torus(r(3.0), r(1.0), 12, 8, ())),
        (
            "spur_gear_involute",
            Mesh::<()>::spur_gear_involute(r(2.0), 12, r(20.0), r(0.0), r(0.0), 4, r(2.0), ()),
        ),
        (
            "spur_gear_cycloid",
            Mesh::<()>::spur_gear_cycloid(r(2.0), 12, r(1.0), r(0.0), 4, r(2.0), ()),
        ),
        (
            "helical_involute_gear",
            Mesh::<()>::helical_involute_gear(
                r(2.0),
                12,
                r(20.0),
                r(0.0),
                r(0.0),
                4,
                r(6.0),
                r(20.0),
                4,
                (),
            ),
        ),
    ];

    for (name, mesh) in meshes {
        assert!(!mesh.polygons.is_empty(), "{name} returned empty topology");
    }
}

#[test]
fn helical_gear_twist_is_derived_from_helix_angle() {
    let mesh = Mesh::<()>::helical_involute_gear(
        r(2.0),
        12,
        r(20.0),
        r(0.0),
        r(0.0),
        4,
        r(6.0),
        r(20.0),
        4,
        (),
    );
    let side = mesh
        .polygons
        .iter()
        .find(|polygon| polygon.vertices.len() == 4)
        .expect("helical gear must have connected side faces");
    let angle = |vertex: &Vertex| {
        let x = vertex.position.x.to_f64_lossy().expect("finite x");
        let y = vertex.position.y.to_f64_lossy().expect("finite y");
        y.atan2(x)
    };
    let mut measured = angle(&side.vertices[3]) - angle(&side.vertices[0]);
    if measured > std::f64::consts::PI {
        measured -= std::f64::consts::TAU;
    } else if measured < -std::f64::consts::PI {
        measured += std::f64::consts::TAU;
    }
    let expected = 6.0 * 20.0_f64.to_radians().tan() / 12.0;
    assert!((measured - expected / 4.0).abs() < 1.0e-9);
}

#[test]
fn test_csg_polyhedron() {
    // A simple tetrahedron
    let pts = &[
        [r(0.0), r(0.0), r(0.0)], // 0
        [r(1.0), r(0.0), r(0.0)], // 1
        [r(0.0), r(1.0), r(0.0)], // 2
        [r(0.0), r(0.0), r(1.0)], // 3
    ];
    let faces: [&[usize]; 4] = [&[0, 1, 2], &[0, 1, 3], &[1, 2, 3], &[2, 0, 3]];
    let csg_tetra: Mesh<()> = Mesh::polyhedron(pts, &faces, ()).unwrap();
    // We should have exactly 4 triangular faces
    assert_eq!(csg_tetra.polygons.len(), 4);
}

#[test]
fn polyhedron_rejects_degenerate_and_nonplanar_faces() {
    let points = &[
        [r(0.0), r(0.0), r(0.0)],
        [r(1.0), r(0.0), r(0.0)],
        [r(1.0), r(1.0), r(0.0)],
        [r(0.0), r(1.0), r(1.0)],
    ];
    assert!(Mesh::<()>::polyhedron(points, &[&[0, 1]], ()).is_err());
    assert!(Mesh::<()>::polyhedron(points, &[&[0, 1, 1]], ()).is_err());
    assert!(Mesh::<()>::polyhedron(points, &[&[0, 1, 2, 3]], ()).is_err());
    assert!(Mesh::<()>::polyhedron(points, &[&[0, 1, points.len()]], ()).is_err());
}

#[test]
fn test_csg_transform_translate_rotate_scale() {
    let c: Mesh<()> = Mesh::cube(r(2.0), ()).center();
    let translated = c.translate(r(1.0), r(2.0), r(3.0));
    let rotated = c.rotate(r(90.0), r(0.0), r(0.0)); // 90 deg about X
    let scaled = c.scale(r(2.0), r(1.0), r(1.0));

    // Quick bounding box checks
    let bb_t = translated.bounding_box();
    assert!(approx_eq(&bb_t.mins.x, -1.0 + 1.0, tolerance()));
    assert!(approx_eq(&bb_t.mins.y, -1.0 + 2.0, tolerance()));
    assert!(approx_eq(&bb_t.mins.z, -1.0 + 3.0, tolerance()));

    let bb_s = scaled.bounding_box();
    assert!(approx_eq(&bb_s.mins.x, -2.0, tolerance())); // scaled by 2 in X
    assert!(approx_eq(&bb_s.maxs.x, 2.0, tolerance()));
    assert!(approx_eq(&bb_s.mins.y, -1.0, tolerance()));
    assert!(approx_eq(&bb_s.maxs.y, 1.0, tolerance()));

    // For rotated, let's just check one polygon's vertices to see if z got mapped to y, etc.
    // (A thorough check would be more geometry-based.)
    let poly0 = &rotated.polygons[0];
    for v in &poly0.vertices {
        // After a 90° rotation around X, the old Y should become old Z,
        // and the old Z should become -old Y.
        // We can't trivially guess each vertex's new coordinate but can do a sanity check:
        // The bounding box in Y might be [-1..1], but let's check we have differences in Y from original.
        assert_ne!(v.position.y, r(0.0)); // Expect something was changed if originally it was ±1 in Z
    }
}

#[test]
fn test_csg_mirror() {
    let c: Mesh<()> = Mesh::cube(r(2.0), ());
    let plane_x = Plane::from_normal(Vector3::x(), r(0.0)); // x=0 plane
    let mirror_x = c.mirror(plane_x);
    let bb_mx = mirror_x.bounding_box();
    // The original cube was from x=0..2, so mirrored across X=0 should be -2..0
    assert!(approx_eq(&bb_mx.mins.x, -2.0, tolerance()));
    assert!(approx_eq(&bb_mx.maxs.x, 0.0, tolerance()));
}

#[test]
#[cfg(feature = "chull-io")]
fn test_csg_convex_hull() {
    // If we take a shape with some random points, the hull should just enclose them
    let c1: Mesh<()> = Mesh::sphere(r(1.0), 16, 8, ());
    // The convex_hull of a sphere's sampling is basically that same shape, but let's see if it runs.
    let hull = c1.convex_hull(());
    // The hull should have some polygons
    assert!(!hull.polygons.is_empty());
}

#[test]
#[cfg(feature = "chull-io")]
fn test_csg_minkowski_sum() {
    // Minkowski sum of two cubes => bigger cube offset by edges
    let c1: Mesh<()> = Mesh::cube(r(2.0), ()).center();
    let c2: Mesh<()> = Mesh::cube(r(1.0), ()).center();
    let sum = c1.minkowski_sum(&c2, ());
    let bb_sum = sum.bounding_box();
    // Expect bounding box from -1.5..+1.5 in each axis if both cubes were centered at (0,0,0).
    assert!(approx_eq(&bb_sum.mins.x, -1.5, 0.01));
    assert!(approx_eq(&bb_sum.maxs.x, 1.5, 0.01));
}

#[test]
fn test_csg_subdivide_triangles() {
    let cube: Mesh<()> = Mesh::cube(r(2.0), ());
    // subdivide_triangles(1) => each polygon (quad) is triangulated => 2 triangles => each tri subdivides => 4
    // So each face with 4 vertices => 2 triangles => each becomes 4 => total 8 per face => 6 faces => 48
    let subdiv = cube.subdivide_triangles(1.try_into().expect("not 0"));
    assert_eq!(subdiv.polygons.len(), 6 * 8);
}

#[test]
fn test_csg_renormalize() {
    let mut cube: Mesh<()> = Mesh::cube(r(2.0), ());
    // After we do some transforms, normals might be changed. We can artificially change them:
    for poly in &mut cube.polygons {
        for v in poly.vertices_mut().iter_mut() {
            v.normal = Vector3::x(); // just set to something
        }
    }
    cube.renormalize();
    // Now each polygon's vertices should match the plane's normal
    for poly in &cube.polygons {
        for v in poly.vertices() {
            let plane_normal = poly.plane().normal();
            assert!(approx_eq(&v.normal.0[0], &plane_normal.0[0], tolerance()));
            assert!(approx_eq(&v.normal.0[1], &plane_normal.0[1], tolerance()));
            assert!(approx_eq(&v.normal.0[2], &plane_normal.0[2], tolerance()));
        }
    }
}

#[test]
fn test_csg_ray_intersections() {
    let cube: Mesh<()> = Mesh::cube(r(2.0), ()).center();
    // Ray from (-2,0,0) toward +X
    let origin = p3(-2.0, 0.0, 0.0);
    let direction = v3(1.0, 0.0, 0.0);
    let hits = cube.ray_intersections(&origin, &direction);
    // Expect 2 intersections with the cube's side at x=-1 and x=1
    assert_eq!(hits.len(), 2);
    // The distances should be 1 unit from -2.0 -> -1 => t=1, and from -2.0 -> +1 => t=3
    assert!(approx_eq(&hits[0].1, 1.0, tolerance()));
    assert!(approx_eq(&hits[1].1, 3.0, tolerance()));
}

#[test]
fn ray_intersections_and_containment_preserve_unit_offsets_beyond_f64_resolution() {
    let base = Real::from(1_i64 << 60);
    let half = (Real::one() / Real::from(2_u8)).unwrap();
    let cube: Mesh<()> =
        Mesh::cube(Real::one(), ()).translate(base.clone(), Real::zero(), Real::zero());
    let origin = Point3::new(base.clone() - Real::one(), half.clone(), half.clone());
    let hits = cube.ray_intersections(&origin, &Vector3::x());

    assert_eq!(hits.len(), 2);
    assert_eq!(hits[0].1, Real::one());
    assert_eq!(hits[1].1, Real::from(2_u8));
    assert!(cube.contains_vertex(&Point3::new(base + half.clone(), half.clone(), half,)));
}

#[test]
fn test_csg_square() {
    let sq: Profile = Profile::square(r(2.0));
    let mesh_2d: Mesh<()> = Mesh::from_profile(sq, ());
    // Single polygon, 4 vertices
    assert_eq!(mesh_2d.polygons.len(), 1);
    let poly = &mesh_2d.polygons[0];
    assert!(
        matches!(poly.vertices.len(), 4 | 5),
        "Expected 4 or 5 vertices, got {}",
        poly.vertices.len()
    );
}

#[test]
fn test_csg_circle() {
    let circle: Profile = Profile::circle(r(2.0), 32);
    let mesh_2d: Mesh<()> = Mesh::from_profile(circle, ());
    // Single polygon with 32 segments => 32 or 33 vertices if closed
    assert_eq!(mesh_2d.polygons.len(), 1);
    let poly = &mesh_2d.polygons[0];
    assert!(
        matches!(poly.vertices.len(), 32 | 33),
        "Expected 32 or 33 vertices, got {}",
        poly.vertices.len()
    );
}

#[test]
fn test_csg_extrude() {
    let sq: Profile = Profile::square(r(2.0));
    let extruded = sq.extrude(r(5.0), ());
    // We expect:
    //   bottom polygon: 2 (square triangulated)
    //   top polygon 2 (square triangulated)
    //   side polygons: 4 for a square (one per edge)
    // => total 8 polygons
    assert_eq!(extruded.polygons.len(), 8);
    // Check bounding box
    let bb = extruded.bounding_box();
    assert!(approx_eq(&bb.mins.z, 0.0, tolerance()));
    assert!(approx_eq(&bb.maxs.z, 5.0, tolerance()));
}

#[test]
fn test_csg_revolve() {
    // Default square is from (0,0) to (1,1) in XY.
    // Shift it so it's from (1,0) to (2,1) — i.e. at least 1.0 unit away from the Z-axis.
    let square: Profile = Profile::square(r(2.0)).translate(r(1.0), r(0.0), r(0.0));

    // Now revolve this translated square around the Z-axis, 360° in 16 segments.
    let revolve = square.revolve(r(360.0), 16, ()).unwrap();

    // We expect a ring-like “tube” instead of a degenerate shape.
    assert!(!revolve.polygons.is_empty());
}

#[test]
fn test_csg_bounding_box() {
    let sphere: Mesh<()> = Mesh::sphere(r(1.0), 16, 8, ());
    let bb = sphere.bounding_box();
    // center=(2,-1,3), radius=2 => bounding box min=(0,-3,1), max=(4,1,5)
    assert!(approx_eq(&bb.mins.x, -1.0, 0.1));
    assert!(approx_eq(&bb.mins.y, -1.0, 0.1));
    assert!(approx_eq(&bb.mins.z, -1.0, 0.1));
    assert!(approx_eq(&bb.maxs.x, 1.0, 0.1));
    assert!(approx_eq(&bb.maxs.y, 1.0, 0.1));
    assert!(approx_eq(&bb.maxs.z, 1.0, 0.1));
}

#[test]
fn test_csg_vertices() {
    let cube: Mesh<()> = Mesh::cube(r(2.0), ());
    let verts = cube.vertices();
    // 6 faces x 4 vertices each = 24
    assert_eq!(verts.len(), 24);
}

#[test]
#[cfg(feature = "offset")]
fn test_csg_offset_2d() {
    let square: Profile = Profile::square(r(2.0));
    let grown = square.offset(r(0.5));
    let shrunk = square.offset(r(-0.5));
    let bb_square = square.bounding_box();
    let bb_grown = grown.bounding_box();
    let bb_shrunk = shrunk.bounding_box();

    println!("Square bb: {:#?}", bb_square);
    println!("Grown bb: {:#?}", bb_grown);
    println!("Shrunk bb: {:#?}", bb_shrunk);

    // Should be bigger
    assert!(bb_grown.maxs.x > bb_square.maxs.x.clone() + r(0.4));

    // Should be smaller
    assert!(bb_shrunk.maxs.x < bb_square.maxs.x + r(0.1));
}

#[cfg(feature = "truetype-text")]
#[test]
fn test_csg_text() {
    // We can't easily test visually, but we can at least test that it doesn't panic
    // and returns some polygons for normal ASCII letters.
    let font_data = include_bytes!("../../asar.ttf");
    let text_csg: Profile = Profile::text("ABC", font_data, r(10.0));
    assert!(!text_csg.region_profiles().is_empty());
}

#[cfg(feature = "truetype-text")]
#[test]
fn test_truetype_text_spacing_and_line_breaks() {
    let font_data = include_bytes!("../../asar.ttf");

    let compact: Profile = Profile::text("AA", font_data, r(20.0));
    let spaced: Profile = Profile::text("A A", font_data, r(20.0));
    let stacked: Profile = Profile::text("A\nA", font_data, r(20.0));

    let compact_bb = compact.bounding_box();
    let spaced_bb = spaced.bounding_box();
    let stacked_bb = stacked.bounding_box();

    let compact_width = compact_bb.maxs.x - compact_bb.mins.x;
    let spaced_width = spaced_bb.maxs.x - spaced_bb.mins.x;
    let stacked_width = stacked_bb.maxs.x - stacked_bb.mins.x;
    let compact_height = compact_bb.maxs.y - compact_bb.mins.y;
    let stacked_height = stacked_bb.maxs.y - stacked_bb.mins.y;

    assert!(
        spaced_width > compact_width,
        "space glyph advance should widen text layout"
    );
    assert!(
        stacked_width < compact_width,
        "newline should reset the horizontal pen"
    );
    assert!(
        stacked_height > compact_height,
        "newline should advance to a new baseline"
    );
}

#[test]
fn test_csg_vertex_index_buffers() {
    let cube: Mesh<()> = Mesh::cube(r(2.0), ());
    let (_vertices, indices) = cube
        .try_get_vertices_and_indices()
        .expect("vertex/index buffers should build");
    assert_eq!(indices.len(), 12); // 6 faces => 2 triangles each => 12
}

#[test]
fn test_csg_mass_properties() {
    let cube: Mesh<()> = Mesh::cube(r(2.0), ()).center(); // side=2 => volume=8. If density=1 => mass=8
    let (mass, com, _frame) = cube
        .mass_properties(r(1.0))
        .expect("mass properties should build");
    println!("{:#?}", mass);
    // For a centered cube with side 2, volume=8 => mass=8 => COM=(0,0,0)
    assert!(approx_eq(mass, 8.0, 0.1));
    assert!(approx_eq(&com.x, 0.0, 0.001));
    assert!(approx_eq(&com.y, 0.0, 0.001));
    assert!(approx_eq(&com.z, 0.0, 0.001));
}

#[test]
fn test_csg_exact_mass_properties_use_hyperphysics_report() {
    let cube: Mesh<()> = Mesh::cube(r(2.0), ()).center();
    let report = cube
        .exact_mass_properties(r(1.0))
        .expect("exact mass properties should build");

    assert_eq!(report.volume, hyperphysics::Real::from(8));
    assert_eq!(report.mass, hyperphysics::Real::from(8));
    assert_eq!(report.center_of_mass, hyperphysics::Vector3::zero());
    assert_eq!(report.certificate.triangle_count, 12);
}

#[ignore = "TODO: impl Mesh::from_stl"]
#[cfg(any())]
#[test]
#[cfg(feature = "stl-io")]
fn test_csg_to_stl_and_from_stl_file() -> Result<(), Box<dyn std::error::Error>> {
    // We'll create a small shape, write to an STL, read it back.
    // You can redirect to a temp file or do an in-memory test.
    let tmp_path = "test_csg_output.stl";

    let cube: Mesh<()> = Mesh::cube(r(2.0), ());
    let res = cube.to_stl_binary("A cube");
    let _ = std::fs::write(tmp_path, res.as_ref().unwrap());
    assert!(res.is_ok());

    let stl_data: Vec<u8> = std::fs::read(tmp_path)?;
    let csg_in: Mesh<()> = Mesh::from_stl(&stl_data, ())?;
    // We expect to read the same number of triangular faces as the cube originally had
    // (though the orientation/normals might differ).
    // The default cube -> 6 polygons x 1 polygon each with 4 vertices => 12 triangles in STL.
    // So from_stl_file => we get 12 triangles as 12 polygons (each is a tri).
    assert_eq!(csg_in.polygons.len(), 12);

    // Cleanup the temp file if desired
    let _ = std::fs::remove_file(tmp_path);
    Ok(())
}
