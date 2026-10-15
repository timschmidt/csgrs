//! Topology construction and certificate regressions.

use super::*;
use hypercurve::{CircularArc2, Curve2, LineSeg2, Point2 as CurvePoint2};
use hyperlattice::Vector3;

// Assertions compare independently constructed exact representations. Keep
// production predicates strict while allowing the test oracle to finish a
// comparison when structural certification alone cannot normalize them.
fn compare_reals(
    left: &Real,
    right: &Real,
    _policy: hyperlimit::PredicatePolicy,
) -> hyperlimit::PredicateOutcome<std::cmp::Ordering> {
    hyperlimit::compare_reals(left, right, crate::TEST_ORACLE_PREDICATES)
}

fn point3_equal(
    left: &Point3,
    right: &Point3,
    _policy: hyperlimit::PredicatePolicy,
) -> hyperlimit::PredicateOutcome<bool> {
    hyperlimit::point3_equal(left, right, crate::TEST_ORACLE_PREDICATES)
}

fn r(value: i32) -> Real {
    Real::from(value)
}

fn p(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

fn uv(x: i32, y: i32) -> CurvePoint2 {
    CurvePoint2::new(r(x), r(y))
}

fn add_planar_triangle(builder: &mut ModelBuilder) -> (FaceId, [EdgeId; 3]) {
    let vertices = [
        builder.vertex(p(0, 0, 0)).unwrap(),
        builder.vertex(p(2, 0, 0)).unwrap(),
        builder.vertex(p(0, 3, 0)).unwrap(),
    ];
    let model_points = [p(0, 0, 0), p(2, 0, 0), p(0, 3, 0)];
    let parameter_points = [uv(0, 0), uv(2, 0), uv(0, 3)];
    let mut edges = Vec::new();
    let mut edge_uses = Vec::new();
    for index in 0..3 {
        let next = (index + 1) % 3;
        let curve = builder
            .curve(Curve3::line(model_points[index].clone(), model_points[next].clone()).unwrap())
            .unwrap();
        let edge = builder
            .edge(
                vertices[index],
                vertices[next],
                curve,
                ParameterDomain::unit(),
            )
            .unwrap();
        let pcurve = builder
            .pcurve(Pcurve::new(Curve2::from(
                LineSeg2::try_new(
                    parameter_points[index].clone(),
                    parameter_points[next].clone(),
                )
                .unwrap(),
            )))
            .unwrap();
        let edge_use = builder
            .edge_use(
                edge,
                Direction::Forward,
                pcurve,
                ParameterCorrespondence::identity(),
            )
            .unwrap();
        edges.push(edge);
        edge_uses.push(edge_use);
    }
    let wire = builder.wire(edge_uses).unwrap();
    let surface = builder
        .surface(Surface::plane(p(0, 0, 0), Vector3::x(), Vector3::y()).unwrap())
        .unwrap();
    let face = builder
        .face(surface, Orientation::Forward, wire, Vec::new())
        .unwrap();
    (face, edges.try_into().unwrap())
}

fn planar_circle_model() -> (Model, [EdgeUseId; 4]) {
    let mut builder = ModelBuilder::new();
    let points = [p(2, 0, 0), p(0, 2, 0), p(-2, 0, 0), p(0, -2, 0)];
    let parameters = [uv(2, 0), uv(0, 2), uv(-2, 0), uv(0, -2)];
    let vertices = points
        .iter()
        .cloned()
        .map(|point| builder.vertex(point).unwrap())
        .collect::<Vec<_>>();
    let half_pi = (Real::pi() / r(2)).unwrap();
    let mut uses = Vec::new();
    for index in 0..4 {
        let next = (index + 1) % 4;
        let start_angle = &half_pi * r(index as i32);
        let end_angle = &half_pi * r(index as i32 + 1);
        let curve = builder
            .curve(
                Curve3::circle_arc(
                    p(0, 0, 0),
                    Vector3::x(),
                    Vector3::y(),
                    r(2),
                    start_angle.clone(),
                    end_angle.clone(),
                )
                .unwrap(),
            )
            .unwrap();
        let edge = builder
            .edge(
                vertices[index],
                vertices[next],
                curve,
                ParameterDomain::new(start_angle, end_angle).unwrap(),
            )
            .unwrap();
        let pcurve = builder
            .pcurve(Pcurve::new(Curve2::from(
                CircularArc2::try_from_center(
                    parameters[index].clone(),
                    parameters[next].clone(),
                    uv(0, 0),
                    false,
                )
                .unwrap(),
            )))
            .unwrap();
        uses.push(
            builder
                .edge_use(
                    edge,
                    Direction::Forward,
                    pcurve,
                    ParameterCorrespondence::angular_sweep(),
                )
                .unwrap(),
        );
    }
    let use_ids: [EdgeUseId; 4] = uses.clone().try_into().unwrap();
    let wire = builder.wire(uses).unwrap();
    let surface = builder
        .surface(Surface::plane(p(0, 0, 0), Vector3::x(), Vector3::y()).unwrap())
        .unwrap();
    let face = builder
        .face(surface, Orientation::Forward, wire, Vec::new())
        .unwrap();
    builder.shell(vec![face]).unwrap();
    (builder.finish().unwrap(), use_ids)
}

#[test]
fn angular_sweep_correspondence_certifies_a_complete_planar_circle() {
    let (model, uses) = planar_circle_model();
    let half = (Real::one() / r(2)).unwrap();
    let first_parameter = model.edge_parameter_at(uses[0], &half).unwrap();
    let first_use = model.edge_use(uses[0]).unwrap();
    let edge = model.edge(first_use.edge()).unwrap();
    let spatial = model
        .curve(edge.curve())
        .unwrap()
        .point_at(&first_parameter)
        .unwrap();
    let face = model.faces().next().unwrap().1;
    let surface_parameter = model
        .pcurve(first_use.pcurve())
        .unwrap()
        .point_at(&half)
        .unwrap();
    let from_face = model
        .surface(face.surface())
        .unwrap()
        .point_at(&surface_parameter)
        .unwrap();
    assert_eq!(
        point3_equal(&spatial, &from_face, crate::STRICT_PREDICATES).value(),
        Some(true)
    );
    assert!(matches!(
        model.edge_use(uses[0]).unwrap().parameter_correspondence(),
        ParameterCorrespondence::AngularSweep
    ));

    let json = model.to_json().unwrap();
    assert!(json.contains("\"AngularSweep\""));
    let decoded = crate::RawModel::from_json(&json)
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(
        compare_reals(
            &decoded.edge_parameter_at(uses[0], &half).unwrap(),
            &first_parameter,
            crate::STRICT_PREDICATES
        )
        .value(),
        Some(std::cmp::Ordering::Equal)
    );
}

#[test]
fn revolution_patch_certifies_complete_meridian_and_latitude_images() {
    let profile = Curve3::line(p(2, 0, 0), p(3, 0, 1)).unwrap();
    let surface = Surface::revolution(profile, Point3::origin(), Vector3::z()).unwrap();
    let quarter = (Real::pi() / Real::from(2)).unwrap();
    let parameters = [
        Point2::new(Real::zero(), Real::zero()),
        Point2::new(quarter.clone(), Real::zero()),
        Point2::new(quarter.clone(), Real::one()),
        Point2::new(Real::zero(), Real::one()),
    ];
    let points = parameters
        .iter()
        .map(|parameter| surface.point_at(parameter).unwrap())
        .collect::<Vec<_>>();
    let mut builder = ModelBuilder::new();
    let vertices = points
        .iter()
        .cloned()
        .map(|point| builder.vertex(point).unwrap())
        .collect::<Vec<_>>();
    let curves = [
        Curve3::circle_arc(
            p(0, 0, 0),
            Vector3::x(),
            Vector3::y(),
            r(2),
            Real::zero(),
            quarter.clone(),
        )
        .unwrap(),
        Curve3::line(points[1].clone(), points[2].clone()).unwrap(),
        Curve3::circle_arc(
            p(0, 0, 1),
            Vector3::x(),
            Vector3::y(),
            r(3),
            Real::zero(),
            quarter.clone(),
        )
        .unwrap(),
        Curve3::line(points[0].clone(), points[3].clone()).unwrap(),
    ];
    let domains = [
        ParameterDomain::new(Real::zero(), quarter.clone()).unwrap(),
        ParameterDomain::unit(),
        ParameterDomain::new(Real::zero(), quarter.clone()).unwrap(),
        ParameterDomain::unit(),
    ];
    let endpoints = [(0, 1), (1, 2), (3, 2), (0, 3)];
    let edges = curves
        .into_iter()
        .zip(domains)
        .zip(endpoints)
        .map(|((curve, domain), (start, end))| {
            let curve = builder.curve(curve).unwrap();
            builder
                .edge(vertices[start], vertices[end], curve, domain)
                .unwrap()
        })
        .collect::<Vec<_>>();
    let pcurve_points = [
        (
            CurvePoint2::new(Real::zero(), Real::zero()),
            CurvePoint2::new(quarter.clone(), Real::zero()),
        ),
        (
            CurvePoint2::new(quarter.clone(), Real::zero()),
            CurvePoint2::new(quarter.clone(), Real::one()),
        ),
        (
            CurvePoint2::new(quarter.clone(), Real::one()),
            CurvePoint2::new(Real::zero(), Real::one()),
        ),
        (
            CurvePoint2::new(Real::zero(), Real::one()),
            CurvePoint2::new(Real::zero(), Real::zero()),
        ),
    ];
    let directions = [
        Direction::Forward,
        Direction::Forward,
        Direction::Reversed,
        Direction::Reversed,
    ];
    let correspondences = [
        ParameterCorrespondence::affine(quarter.clone(), Real::zero()).unwrap(),
        ParameterCorrespondence::identity(),
        ParameterCorrespondence::affine(-quarter.clone(), quarter).unwrap(),
        ParameterCorrespondence::affine(-Real::one(), Real::one()).unwrap(),
    ];
    let uses = edges
        .into_iter()
        .zip(pcurve_points)
        .zip(directions)
        .zip(correspondences)
        .map(|(((edge, (start, end)), direction), correspondence)| {
            let pcurve = builder
                .pcurve(Pcurve::new(Curve2::from(
                    LineSeg2::try_new(start, end).unwrap(),
                )))
                .unwrap();
            builder
                .edge_use(edge, direction, pcurve, correspondence)
                .unwrap()
        })
        .collect();
    let wire = builder.wire(uses).unwrap();
    let surface = builder.surface(surface).unwrap();
    let face = builder
        .face(surface, Orientation::Forward, wire, Vec::new())
        .unwrap();
    builder.shell(vec![face]).unwrap();
    builder.finish().unwrap();
}

#[test]
fn analytic_image_certificate_rejects_a_sheared_arc_between_matching_endpoints() {
    let mut builder = ModelBuilder::new();
    let vertices = [
        builder.vertex(p(1, 0, 0)).unwrap(),
        builder.vertex(p(-1, 0, 0)).unwrap(),
    ];
    let angle_domains = [(Real::zero(), Real::pi()), (Real::pi(), Real::tau())];
    let parameter_points = [(uv(1, 0), uv(-1, 0)), (uv(-1, 0), uv(1, 0))];
    let mut uses = Vec::new();
    for index in 0..2 {
        let (start_angle, end_angle) = &angle_domains[index];
        let curve = builder
            .curve(
                Curve3::circle_arc(
                    p(0, 0, 0),
                    Vector3::x(),
                    Vector3::y(),
                    Real::one(),
                    start_angle.clone(),
                    end_angle.clone(),
                )
                .unwrap(),
            )
            .unwrap();
        let edge = builder
            .edge(
                vertices[index],
                vertices[(index + 1) % 2],
                curve,
                ParameterDomain::new(start_angle.clone(), end_angle.clone()).unwrap(),
            )
            .unwrap();
        let pcurve = builder
            .pcurve(Pcurve::new(Curve2::from(
                CircularArc2::try_from_center(
                    parameter_points[index].0.clone(),
                    parameter_points[index].1.clone(),
                    uv(0, 0),
                    false,
                )
                .unwrap(),
            )))
            .unwrap();
        uses.push(
            builder
                .edge_use(
                    edge,
                    Direction::Forward,
                    pcurve,
                    ParameterCorrespondence::angular_sweep(),
                )
                .unwrap(),
        );
    }
    let wire = builder.wire(uses).unwrap();
    let sheared_plane = builder
        .surface(
            Surface::plane(
                p(0, 0, 0),
                Vector3::x(),
                Vector3::from_xyz(Real::one(), Real::one(), Real::zero()),
            )
            .unwrap(),
        )
        .unwrap();
    assert_eq!(
        builder.face(sheared_plane, Orientation::Forward, wire, Vec::new()),
        Err(BuildError::EdgeUseSupportMismatch)
    );
}

fn add_planar_wire(builder: &mut ModelBuilder, coordinates: &[(i32, i32)]) -> WireId {
    let vertices = coordinates
        .iter()
        .map(|(x, y)| builder.vertex(p(*x, *y, 0)).unwrap())
        .collect::<Vec<_>>();
    let mut uses = Vec::with_capacity(coordinates.len());
    for index in 0..coordinates.len() {
        let next = (index + 1) % coordinates.len();
        let (x0, y0) = coordinates[index];
        let (x1, y1) = coordinates[next];
        let curve = builder
            .curve(Curve3::line(p(x0, y0, 0), p(x1, y1, 0)).unwrap())
            .unwrap();
        let edge = builder
            .edge(
                vertices[index],
                vertices[next],
                curve,
                ParameterDomain::unit(),
            )
            .unwrap();
        let pcurve = builder
            .pcurve(Pcurve::new(Curve2::from(
                LineSeg2::try_new(uv(x0, y0), uv(x1, y1)).unwrap(),
            )))
            .unwrap();
        uses.push(
            builder
                .edge_use(
                    edge,
                    Direction::Forward,
                    pcurve,
                    ParameterCorrespondence::identity(),
                )
                .unwrap(),
        );
    }
    builder.wire(uses).unwrap()
}

#[test]
fn builder_commits_private_typed_arenas_with_retained_adjacency() {
    let mut builder = ModelBuilder::new();
    let (face, edges) = add_planar_triangle(&mut builder);
    let shell = builder.shell(vec![face]).unwrap();
    let model = builder.finish().unwrap();

    assert_eq!(
        model.counts(),
        ModelCounts {
            vertices: 3,
            curves: 3,
            pcurves: 3,
            surfaces: 1,
            edges: 3,
            edge_uses: 3,
            wires: 1,
            faces: 1,
            shells: 1,
            solids: 0,
        }
    );
    assert_eq!(model.shell(shell).unwrap().faces(), &[face]);
    assert_eq!(model.uses_of_edge(edges[0]).unwrap().len(), 1);
    assert_eq!(
        model
            .edges_at_vertex(model.edge(edges[0]).unwrap().start())
            .unwrap()
            .len(),
        2
    );
}

#[test]
fn builder_rejects_topologically_disconnected_wires() {
    let mut builder = ModelBuilder::new();
    let vertices = [
        builder.vertex(p(0, 0, 0)).unwrap(),
        builder.vertex(p(1, 0, 0)).unwrap(),
        builder.vertex(p(2, 0, 0)).unwrap(),
    ];
    let mut uses = Vec::new();
    for (start, end) in [(0, 1), (2, 1)] {
        let curve = builder
            .curve(Curve3::line(p(start, 0, 0), p(end, 0, 0)).unwrap())
            .unwrap();
        let edge = builder
            .edge(
                vertices[start as usize],
                vertices[end as usize],
                curve,
                ParameterDomain::unit(),
            )
            .unwrap();
        let pcurve = builder
            .pcurve(Pcurve::new(Curve2::from(
                LineSeg2::try_new(uv(start, 0), uv(end, 0)).unwrap(),
            )))
            .unwrap();
        uses.push(
            builder
                .edge_use(
                    edge,
                    Direction::Forward,
                    pcurve,
                    ParameterCorrespondence::identity(),
                )
                .unwrap(),
        );
    }
    assert_eq!(
        builder.wire(uses),
        Err(BuildError::DisconnectedWire { at: 0 })
    );
}

#[test]
fn finish_reports_orphan_topology_instead_of_publishing_invalid_model() {
    let mut builder = ModelBuilder::new();
    let start = builder.vertex(p(0, 0, 0)).unwrap();
    let end = builder.vertex(p(1, 0, 0)).unwrap();
    let curve = builder
        .curve(Curve3::line(p(0, 0, 0), p(1, 0, 0)).unwrap())
        .unwrap();
    let edge = builder
        .edge(start, end, curve, ParameterDomain::unit())
        .unwrap();
    let pcurve = builder
        .pcurve(Pcurve::new(Curve2::from(
            LineSeg2::try_new(uv(0, 0), uv(1, 0)).unwrap(),
        )))
        .unwrap();
    let edge_use = builder
        .edge_use(
            edge,
            Direction::Forward,
            pcurve,
            ParameterCorrespondence::identity(),
        )
        .unwrap();

    let report = builder.finish().unwrap_err();
    assert_eq!(report.errors(), &[BuildError::OrphanEdgeUse(edge_use)]);
}

#[test]
fn rational_line_image_certificate_rejects_collinear_backtracking() {
    let monotone = Curve3::rational_bezier(
        vec![p(0, 0, 0), p(1, 0, 0), p(3, 0, 0)],
        vec![Real::one(), r(2), r(5)],
    )
    .unwrap();
    assert!(certified_monotone_line_curve_image(&monotone).unwrap());

    let backtracking = Curve3::rational_bezier(
        vec![p(0, 0, 0), p(3, 0, 0), p(2, 0, 0)],
        vec![Real::one(), r(2), r(5)],
    )
    .unwrap();
    assert!(!certified_monotone_line_curve_image(&backtracking).unwrap());
}

#[test]
fn extrusion_area_certifies_constant_and_line_image_laws_and_rejects_variable_speed_profiles() {
    let profiles = [
        Curve3::rational_bezier(
            vec![p(0, 0, 0), p(1, 0, 0), p(2, 0, 0)],
            vec![Real::one(), r(2), r(3)],
        )
        .unwrap(),
        Curve3::nurbs(
            2,
            vec![p(0, 0, 0), p(1, 0, 0), p(2, 0, 0)],
            vec![Real::one(), r(2), r(3)],
            vec![r(2), r(2), r(2), r(5), r(5), r(5)],
        )
        .unwrap(),
    ];
    for profile in profiles {
        let (model, face) =
            crate::builder::extrusion_patch(profile, Vector3::z(), Real::zero(), r(3)).unwrap();
        let area = model.face_area(face).unwrap();
        assert_eq!(
            compare_reals(&area, &r(6), crate::STRICT_PREDICATES).value(),
            Some(std::cmp::Ordering::Equal)
        );
        let replayed = crate::RawModel::from_json(&model.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(
            compare_reals(
                &replayed.face_area(face).unwrap(),
                &r(6),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
    }

    let planar_profiles = [
        Curve3::rational_bezier(
            vec![p(0, 0, 0), p(2, 1, 0), p(0, 2, 0)],
            vec![Real::one(), r(2), r(3)],
        )
        .unwrap(),
        Curve3::nurbs(
            2,
            vec![p(0, 0, 0), p(2, 1, 0), p(0, 2, 0)],
            vec![Real::one(), r(2), r(3)],
            vec![r(2), r(2), r(2), r(5), r(5), r(5)],
        )
        .unwrap(),
    ];
    for profile in planar_profiles {
        assert!(certified_monotone_planar_extrusion_image(&profile, &Vector3::x()).unwrap());
        let (model, face) =
            crate::builder::extrusion_patch(profile, Vector3::x(), Real::zero(), r(3)).unwrap();
        assert_eq!(
            compare_reals(
                &model.face_area(face).unwrap(),
                &r(6),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
        let replayed = crate::RawModel::from_json(&model.to_json().unwrap())
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(
            compare_reals(
                &replayed.face_area(face).unwrap(),
                &r(6),
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
    }

    let curved = Curve3::rational_bezier(
        vec![p(0, 0, 0), p(1, 1, 0), p(2, 0, 0)],
        vec![Real::one(), r(2), r(3)],
    )
    .unwrap();
    let (model, face) =
        crate::builder::extrusion_patch(curved, Vector3::z(), Real::zero(), r(3)).unwrap();
    assert_eq!(
        model.face_area(face),
        Err(QueryError::Geometry(GeometryError::UnsupportedMeasurement))
    );

    let backtracking = Curve3::rational_bezier(
        vec![p(0, 0, 0), p(2, 3, 0), p(0, 2, 0)],
        vec![Real::one(), r(2), r(3)],
    )
    .unwrap();
    assert!(!certified_monotone_planar_extrusion_image(&backtracking, &Vector3::x()).unwrap());
    let (backtracking_model, backtracking_face) =
        crate::builder::extrusion_patch(backtracking, Vector3::x(), Real::zero(), r(3)).unwrap();
    assert_eq!(
        backtracking_model.face_area(backtracking_face),
        Err(QueryError::Geometry(GeometryError::UnsupportedMeasurement))
    );

    let circle = Curve3::circle_arc(
        p(0, 0, 0),
        Vector3::x(),
        Vector3::y(),
        r(2),
        Real::zero(),
        (Real::pi() / r(2)).unwrap(),
    )
    .unwrap();
    let (normal_model, normal_face) =
        crate::builder::extrusion_patch(circle.clone(), Vector3::z(), Real::zero(), r(3)).unwrap();
    assert_eq!(
        compare_reals(
            &normal_model.face_area(normal_face).unwrap(),
            &(r(3) * Real::pi()),
            crate::STRICT_PREDICATES
        )
        .value(),
        Some(std::cmp::Ordering::Equal)
    );
    let (oblique_model, oblique_face) = crate::builder::extrusion_patch(
        circle,
        Vector3::from_xyz(Real::one(), Real::zero(), Real::one()),
        Real::zero(),
        r(3),
    )
    .unwrap();
    assert_eq!(
        oblique_model.face_area(oblique_face),
        Err(QueryError::Geometry(GeometryError::UnsupportedMeasurement))
    );

    let ellipse = Curve3::ellipse_arc(
        p(0, 0, 0),
        Vector3::x(),
        Vector3::y(),
        r(2),
        Real::one(),
        Real::zero(),
        (Real::pi() / r(2)).unwrap(),
    )
    .unwrap();
    let (ellipse_model, ellipse_face) =
        crate::builder::extrusion_patch(ellipse, Vector3::z(), Real::zero(), r(3)).unwrap();
    assert_eq!(
        ellipse_model.face_area(ellipse_face),
        Err(QueryError::Geometry(GeometryError::UnsupportedMeasurement))
    );
}

#[test]
fn canonical_model_and_geometry_are_send_and_sync() {
    fn assert_send_sync<T: Send + Sync>() {}

    assert_send_sync::<Curve3>();
    assert_send_sync::<Pcurve>();
    assert_send_sync::<Surface>();
    assert_send_sync::<Model>();
}

#[test]
fn planar_face_certifies_inner_wire_orientation_nesting_and_area() {
    let mut builder = ModelBuilder::new();
    let outer = add_planar_wire(&mut builder, &[(0, 0), (10, 0), (10, 10), (0, 10)]);
    let inner = add_planar_wire(&mut builder, &[(3, 3), (3, 7), (7, 7), (7, 3)]);
    let surface = builder
        .surface(Surface::plane(p(0, 0, 0), Vector3::x(), Vector3::y()).unwrap())
        .unwrap();
    let face = builder
        .face(surface, Orientation::Forward, outer, vec![inner])
        .unwrap();
    builder.shell(vec![face]).unwrap();
    let model = builder.finish().unwrap();
    assert_eq!(
        compare_reals(
            &model.face_area(face).unwrap(),
            &Real::from(84),
            crate::STRICT_PREDICATES
        )
        .value(),
        Some(std::cmp::Ordering::Equal)
    );

    let mut wrong_orientation = ModelBuilder::new();
    let outer = add_planar_wire(
        &mut wrong_orientation,
        &[(0, 0), (10, 0), (10, 10), (0, 10)],
    );
    let inner = add_planar_wire(&mut wrong_orientation, &[(3, 3), (7, 3), (7, 7), (3, 7)]);
    let surface = wrong_orientation
        .surface(Surface::plane(p(0, 0, 0), Vector3::x(), Vector3::y()).unwrap())
        .unwrap();
    assert_eq!(
        wrong_orientation.face(surface, Orientation::Forward, outer, vec![inner]),
        Err(BuildError::InconsistentWireOrientation(inner))
    );

    let mut outside = ModelBuilder::new();
    let outer = add_planar_wire(&mut outside, &[(0, 0), (10, 0), (10, 10), (0, 10)]);
    let inner = add_planar_wire(&mut outside, &[(12, 2), (12, 4), (14, 4), (14, 2)]);
    let surface = outside
        .surface(Surface::plane(p(0, 0, 0), Vector3::x(), Vector3::y()).unwrap())
        .unwrap();
    assert_eq!(
        outside.face(surface, Orientation::Forward, outer, vec![inner]),
        Err(BuildError::InnerWireOutside(inner))
    );
}

#[cfg(test)]
#[test]
fn homogeneous_pcurve_validation_requires_a_nonzero_projective_scale() {
    use hypercurve::HomogeneousControl2;
    let expected = [
        HomogeneousControl2::new(Real::one(), Real::zero(), Real::one()),
        HomogeneousControl2::new(Real::zero(), Real::one(), Real::zero()),
        HomogeneousControl2::new(-Real::one(), Real::zero(), Real::one()),
    ];
    for prefix in [
        Vec::new(),
        vec![HomogeneousControl2::new(
            Real::zero(),
            Real::zero(),
            Real::zero(),
        )],
        vec![HomogeneousControl2::new(
            Real::one(),
            Real::one(),
            Real::zero(),
        )],
    ] {
        let expected: Vec<_> = prefix.into_iter().chain(expected.iter().cloned()).collect();
        for scale in [Real::pi(), -Real::pi()] {
            let scaled: Vec<_> = expected
                .iter()
                .map(|control| {
                    HomogeneousControl2::new(
                        control.x() * &scale,
                        control.y() * &scale,
                        control.weight() * &scale,
                    )
                })
                .collect();
            validate_homogeneous_pcurve_controls(&scaled, &expected).unwrap();
        }
        let zero = vec![
            HomogeneousControl2::new(Real::zero(), Real::zero(), Real::zero());
            expected.len()
        ];
        assert!(validate_homogeneous_pcurve_controls(&zero, &expected).is_err());
        assert!(validate_homogeneous_pcurve_controls(&expected, &zero).is_err());
    }
}
