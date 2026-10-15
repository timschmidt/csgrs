use std::cmp::Ordering;

use hyperbrep::{
    Curve3, Curve3Kind, CurveParameterLocation, Direction, GeometricPoint3, GeometryError, Matrix4,
    Model, ModelBuilder, Parameter, ParameterCorrespondence, ParameterRange, Point3, Real,
    SolidPointLocation, TopologyEditError, Vector3, builder,
};

fn point(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(Real::from(x), Real::from(y), Real::from(z))
}

fn rational(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn root() -> Parameter {
    // The selected unit-interval root of 32t^5 - 2t - 1 has no stored Real.
    let curve = Curve3::rational_bezier(
        [-5, -7, -9, -11, -13, 145]
            .map(|n| Point3::new(rational(n, 5), Real::zero(), Real::zero()))
            .to_vec(),
        vec![Real::one(); 6],
    )
    .unwrap();
    let CurveParameterLocation::Parameters(mut parameters) =
        curve.parameters_of(&Point3::origin()).unwrap()
    else {
        panic!("expected a finite selected root");
    };
    assert_eq!(parameters.len(), 1);
    let parameter = parameters.remove(0);
    assert!(parameter.try_real().is_none());
    parameter
}

fn check_correspondences(model: &Model) {
    for edge in model.edges() {
        for edge_use in edge.uses() {
            let pcurve = edge_use.pcurve();
            let endpoints = match edge_use.direction() {
                Direction::Forward => [edge.domain().start(), edge.domain().end()],
                Direction::Reversed => [edge.domain().end(), edge.domain().start()],
            };
            for ((local, spatial), expected_point) in
                [pcurve.domain_start().unwrap(), pcurve.domain_end().unwrap()]
                    .into_iter()
                    .zip(endpoints)
                    .zip([pcurve.curve().start(), pcurve.curve().end()])
            {
                assert_eq!(
                    edge_use
                        .edge_parameter_at(&local)
                        .unwrap()
                        .compare(spatial)
                        .unwrap(),
                    Ordering::Equal,
                    "edge {} use {}: local={local:?}, spatial={spatial:?}, map={:?}, native={}",
                    edge.index(),
                    edge_use.index(),
                    edge_use.parameter_correspondence(),
                    pcurve.curve().geometry().is_some()
                );
                assert_eq!(
                    edge_use
                        .pcurve_parameter_at(spatial)
                        .unwrap()
                        .compare(&local)
                        .unwrap(),
                    Ordering::Equal
                );
                assert!(
                    pcurve
                        .point_at(&local)
                        .unwrap()
                        .coincides_with(&expected_point)
                        .unwrap()
                );
            }
        }
    }
}

#[test]
fn selected_split_preserves_both_incident_uses_and_shared_spatial_point() {
    let source = builder::cuboid(point(0, 0, 0), point(2, 3, 4)).unwrap();
    let parameter = root();
    let edge = source.model().edges().next().unwrap();
    let expected = edge.curve().point_at(&parameter).unwrap();
    let split = edge.split(parameter.clone()).unwrap();
    let details = split.details();
    assert_eq!(
        split.model().counts().vertices,
        source.model().counts().vertices + 1
    );
    assert_eq!(
        split.model().counts().edges,
        source.model().counts().edges + 1
    );
    assert_eq!(details.edge_uses().len(), 2);
    assert_eq!(details.vertex().edges().len(), 2);
    assert!(details.vertex().point().try_coordinates().is_none());
    assert!(details.vertex().point().coincides_with(&expected).unwrap());
    assert_eq!(
        details.first().domain().end().compare(&parameter).unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        details
            .second()
            .domain()
            .start()
            .compare(&parameter)
            .unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        details.first().curve().index(),
        details.second().curve().index()
    );
    assert!(
        source
            .model()
            .vertices()
            .all(|v| v.point().try_coordinates().is_some())
    );
    check_correspondences(split.model());
    split.model().bounds().unwrap().unwrap();
    let solid = split.model().solids().next().unwrap();
    assert_eq!(
        solid.classify_point(&point(1, 1, 1)).unwrap(),
        SolidPointLocation::Inside
    );
    assert_eq!(
        solid.classify_point(&point(3, 1, 1)).unwrap(),
        SolidPointLocation::Outside
    );
    assert_eq!(solid.volume().unwrap(), Real::from(24));
}

#[test]
fn selected_split_composes_and_rejects_repeated_endpoint() {
    let source = builder::cuboid(point(0, 0, 0), point(2, 3, 4)).unwrap();
    let parameter = root();
    let first = source
        .model()
        .edges()
        .next()
        .unwrap()
        .split(parameter.clone())
        .unwrap();
    assert!(matches!(
        first.details().first().split(parameter.clone()),
        Err(TopologyEditError::Geometry(
            GeometryError::InvalidParameterDomain
        ))
    ));
    let second = first
        .details()
        .first()
        .split(rational(1, 2).into())
        .unwrap();
    check_correspondences(second.model());
    assert!(
        second
            .model()
            .vertices()
            .any(|v| v.point().try_coordinates().is_none())
    );
    let third = first
        .details()
        .second()
        .split(rational(3, 4).into())
        .unwrap();
    check_correspondences(third.model());
}

#[test]
fn selected_vertices_and_pcurves_survive_affine_transforms_and_reflection() {
    let source = builder::cuboid(point(0, 0, 0), point(2, 3, 4)).unwrap();
    let split = source
        .model()
        .edges()
        .next()
        .unwrap()
        .split(root())
        .unwrap();
    for entries in [
        [1, 1, 0, 3, 0, 1, 0, 4, 0, 0, 2, 5, 0, 0, 0, 1],
        [-1, 0, 0, 3, 0, 1, 0, 4, 0, 0, 1, 5, 0, 0, 0, 1],
    ] {
        let transform = Matrix4::from_row_major(entries.map(Real::from));
        let expected = split
            .details()
            .vertex()
            .point()
            .transformed(&transform)
            .unwrap();
        let moved = split.model().transformed(&transform).unwrap();
        let vertex = moved
            .vertices()
            .find(|v| v.point().try_coordinates().is_none())
            .unwrap();
        assert!(vertex.point().coincides_with(&expected).unwrap());
        check_correspondences(&moved);
        let restored = moved.transformed(&transform.inverse().unwrap()).unwrap();
        let vertex = restored
            .vertices()
            .find(|v| v.point().try_coordinates().is_none())
            .unwrap();
        assert!(
            vertex
                .point()
                .coincides_with(split.details().vertex().point())
                .unwrap()
        );
        check_correspondences(&restored);
        let continued = moved
            .edges()
            .next()
            .unwrap()
            .split(rational(1, 2).into())
            .unwrap();
        check_correspondences(continued.model());
    }
}

#[test]
fn selected_spline_trims_keep_their_authored_parameter_charts() {
    for nurbs in [false, true] {
        let controls = vec![point(0, 0, 0), point(1, -1, 0), point(2, 0, 0)];
        let weights = vec![Real::one(), Real::from(2), Real::one()];
        let (profile, parameter, kind) = if nurbs {
            let selector = Curve3::nurbs(
                5,
                [-5, -7, -9, -11, -13, 145]
                    .map(|n| Point3::new(rational(n, 5), Real::zero(), Real::zero()))
                    .to_vec(),
                vec![Real::one(); 6],
                [vec![Real::from(2); 6], vec![Real::from(5); 6]].concat(),
            )
            .unwrap();
            let CurveParameterLocation::Parameters(parameters) =
                selector.parameters_of(&Point3::origin()).unwrap()
            else {
                panic!("expected finite selected knot parameter");
            };
            (
                Curve3::nurbs(
                    2,
                    controls,
                    weights,
                    [2, 2, 2, 5, 5, 5].map(Real::from).to_vec(),
                )
                .unwrap(),
                parameters[0].clone(),
                Curve3Kind::Nurbs,
            )
        } else {
            (
                Curve3::rational_bezier(controls, weights).unwrap(),
                root(),
                Curve3Kind::RationalBezier,
            )
        };
        let expected = profile.point_at(&parameter).unwrap();
        let source =
            builder::extrusion_patch(profile, Vector3::z(), Real::zero(), Real::one()).unwrap();
        let edge = source
            .model()
            .edges()
            .find(|edge| edge.curve().kind() == kind)
            .unwrap();
        let split = edge.split(parameter.clone()).unwrap();
        assert!(
            split
                .details()
                .vertex()
                .point()
                .coincides_with(&expected)
                .unwrap()
        );
        assert!(split.details().first().domain().end().try_real().is_none());
        check_correspondences(split.model());
        let continued = split
            .details()
            .first()
            .split(
                if nurbs {
                    rational(7, 2)
                } else {
                    rational(1, 2)
                }
                .into(),
            )
            .unwrap();
        check_correspondences(continued.model());
    }
}

#[test]
fn selected_cylinder_axial_split_preserves_solid_queries() {
    let source = builder::cylinder(Real::from(2), Real::from(3)).unwrap();
    let edge = source
        .model()
        .edges()
        .find(|edge| edge.curve().kind() == Curve3Kind::Line)
        .unwrap();
    let split = edge.split(root()).unwrap();
    check_correspondences(split.model());
    let solid = split.model().solids().next().unwrap();
    assert_eq!(
        solid.volume().unwrap(),
        source.bodies().next().unwrap().volume().unwrap()
    );
    assert_eq!(
        solid.classify_point(&point(0, 0, 1)).unwrap(),
        SolidPointLocation::Inside
    );
    assert_eq!(
        solid.classify_point(&point(3, 0, 1)).unwrap(),
        SolidPointLocation::Outside
    );
}

#[test]
fn arbitrary_edits_cannot_reuse_subdivision_certificates() {
    let source = builder::cuboid(point(0, 0, 0), point(2, 3, 4)).unwrap();
    let split = source
        .model()
        .edges()
        .next()
        .unwrap()
        .split(root())
        .unwrap();
    let mut edit = split.model().edit();
    edit.replace_vertex(split.details().vertex(), point(9, 9, 9).into())
        .unwrap();
    assert!(edit.commit().is_err());

    let edge_use = source.model().edge_uses().next().unwrap();
    let mut edit = source.model().edit();
    edit.replace_edge_use(
        edge_use,
        edge_use.edge(),
        edge_use.direction(),
        edge_use.pcurve(),
        ParameterCorrespondence::affine(Real::from(2), Real::zero()).unwrap(),
    )
    .unwrap();
    assert!(edit.commit().is_err());
}

#[test]
fn remaining_scalar_boundaries_fail_explicitly_without_mutating_the_source() {
    let source = builder::cuboid(point(0, 0, 0), point(2, 3, 4)).unwrap();
    let split = source
        .model()
        .edges()
        .next()
        .unwrap()
        .split(root())
        .unwrap();
    assert!(matches!(
        split.model().to_raw(),
        Err(hyperbrep::PersistenceError::Build(
            hyperbrep::BuildError::Geometry(GeometryError::UnrepresentablePoint)
        ))
    ));
    let replayed = split.model().edit().commit().unwrap();
    check_correspondences(&replayed);
    assert!(
        replayed
            .vertices()
            .any(|vertex| vertex.point().try_coordinates().is_none())
    );

    let cylinder = builder::cylinder(Real::from(2), Real::from(3)).unwrap();
    let json = cylinder.model().to_json().unwrap();
    let circular = cylinder
        .model()
        .edges()
        .find(|edge| edge.curve().kind() == Curve3Kind::CircleArc)
        .unwrap();
    assert!(matches!(
        circular.split(root()),
        Err(TopologyEditError::Geometry(
            GeometryError::UnsupportedSelectedEvaluation
        ))
    ));
    assert_eq!(cylinder.model().to_json().unwrap(), json);
}

#[test]
fn selected_rational_pcurve_reversal_keeps_source_parameters_and_endpoint_images() {
    let curve = hypercurve::RationalBezier2::try_new(
        vec![
            hypercurve::Point2::new(Real::zero(), Real::zero()),
            hypercurve::Point2::new(Real::one(), -Real::one()),
            hypercurve::Point2::new(Real::from(2), Real::zero()),
        ],
        vec![Real::one(), Real::from(2), Real::one()],
    )
    .unwrap();
    let pcurve = hyperbrep::Pcurve::new(curve.into());
    let (first, _) = pcurve.split_at(&root()).unwrap();
    let reversed = first.reversed().unwrap();
    assert_eq!(
        first
            .domain_start()
            .unwrap()
            .compare(&reversed.domain_end().unwrap())
            .unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        first
            .domain_end()
            .unwrap()
            .compare(&reversed.domain_start().unwrap())
            .unwrap(),
        Ordering::Equal
    );
    assert!(
        reversed
            .point_at(&reversed.domain_start().unwrap())
            .unwrap()
            .coincides_with(&first.curve().end())
            .unwrap()
    );
    assert!(
        reversed
            .point_at(&reversed.domain_end().unwrap())
            .unwrap()
            .coincides_with(&first.curve().start())
            .unwrap()
    );
    assert!(reversed.domain_start().unwrap().try_real().is_none());
}

#[test]
fn planar_selected_rational_pcurves_survive_model_reflection() {
    use hypercurve::{Curve2, CurvePath2, LineSeg2, Point2, RationalBezier2};

    let uv = |x, y| Point2::new(Real::from(x), Real::from(y));
    let curved = RationalBezier2::try_new(
        vec![uv(0, 0), uv(1, -1), uv(2, 0)],
        vec![Real::one(), Real::from(2), Real::one()],
    )
    .unwrap();
    let boundary = CurvePath2::try_new(vec![
        curved.into(),
        Curve2::from(LineSeg2::try_new(uv(2, 0), uv(2, 2)).unwrap()),
        Curve2::from(LineSeg2::try_new(uv(2, 2), uv(0, 2)).unwrap()),
        Curve2::from(LineSeg2::try_new(uv(0, 2), uv(0, 0)).unwrap()),
    ])
    .unwrap();
    let source =
        builder::planar_face(&boundary, &[], Point3::origin(), Vector3::x(), Vector3::y()).unwrap();
    let reflection =
        Matrix4::from_row_major([-1, 0, 0, 3, 0, 1, 0, 4, 0, 0, 1, 5, 0, 0, 0, 1].map(Real::from));
    check_correspondences(&source.model().transformed(&reflection).unwrap());
    let edge = source
        .model()
        .edges()
        .find(|edge| edge.curve().kind() == Curve3Kind::RationalBezier)
        .unwrap();
    let split = edge.split(root()).unwrap();
    let reflected = split.model().transformed(&reflection).unwrap();
    check_correspondences(&reflected);
    let expected = split
        .details()
        .vertex()
        .point()
        .transformed(&reflection)
        .unwrap();
    let actual = reflected
        .vertices()
        .find(|vertex| vertex.point().try_coordinates().is_none())
        .unwrap();
    assert!(actual.point().coincides_with(&expected).unwrap());
    let continued = reflected
        .edges()
        .next()
        .unwrap()
        .split(rational(1, 2).into())
        .unwrap();
    check_correspondences(continued.model());
}

#[test]
fn selected_trim_range_orders_exactly_and_builder_checks_endpoint_incidence() {
    let parameter = root();
    let range = ParameterRange::new(Real::zero().into(), parameter.clone()).unwrap();
    assert!(range.contains(&rational(1, 2).into()).unwrap());
    assert!(!range.contains(&rational(3, 4).into()).unwrap());
    assert!(range.try_real_endpoints().is_none());
    assert!(matches!(
        ParameterRange::new(parameter.clone(), parameter.clone()),
        Err(GeometryError::InvalidParameterDomain)
    ));
    assert!(matches!(
        ParameterRange::new(parameter.clone(), Real::zero().into()),
        Err(GeometryError::InvalidParameterDomain)
    ));

    let curve = Curve3::line(point(0, 0, 0), point(1, 0, 0)).unwrap();
    for correct in [true, false] {
        let mut builder = ModelBuilder::new();
        let start = builder.vertex(point(0, 0, 0).into()).unwrap();
        let end = builder
            .vertex(if correct {
                curve.point_at(&parameter).unwrap()
            } else {
                GeometricPoint3::from_coordinates(point(1, 0, 0))
            })
            .unwrap();
        let carrier = builder.curve(curve.clone()).unwrap();
        assert_eq!(
            builder.edge(&start, &end, &carrier, range.clone()).is_ok(),
            correct
        );
    }
}
