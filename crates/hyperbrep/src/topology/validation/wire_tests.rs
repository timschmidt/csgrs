use crate::topology::*;

fn point(x: i32, y: i32) -> Point3 {
    Point3::new(x.into(), y.into(), Real::zero())
}

fn native_sheet(curves: Vec<Curve3>) -> Result<ArenaModel, BuildError> {
    let plane = Surface::plane(point(0, 0), Vector3::x(), Vector3::y())?;
    let mut builder = ArenaModelBuilder::new();
    let surface = builder.surface(plane.clone())?;
    let vertices = curves
        .iter()
        .map(|curve| builder.vertex(curve.start()?.into()))
        .collect::<Result<Vec<_>, BuildError>>()?;
    let mut uses = Vec::new();
    for (i, curve) in curves.iter().enumerate() {
        let id = builder.curve(curve.clone())?;
        let edge = builder.edge(
            vertices[i],
            vertices[(i + 1) % vertices.len()],
            id,
            curve.domain().clone().into(),
        )?;
        let trace = SurfaceIntersectionCurve::on_plane(curve.clone(), &plane)?;
        let materialized = trace.first_pcurve().materialize()?;
        let correspondence = match materialized.correspondence() {
            SurfacePcurveCorrespondence::Affine { scale, offset } => {
                ParameterCorrespondence::affine(scale.clone(), offset.clone())?
            }
            SurfacePcurveCorrespondence::AngularSweep { .. } => {
                ParameterCorrespondence::angular_sweep()
            }
        };
        let pcurve = builder.pcurve(Pcurve::new(materialized.curve().clone()))?;
        uses.push(builder.edge_use(edge, Direction::Forward, pcurve, correspondence)?);
    }
    let wire = builder.wire(uses)?;
    let face = builder.face(surface, Orientation::Forward, wire, vec![])?;
    builder.shell(vec![face])?;
    builder.finish().map_err(|report| report.errors[0].clone())
}

fn arc(radius: i32, first_quarter: i32, last_quarter: i32) -> Curve3 {
    let quarter = (Real::pi() / Real::from(2)).unwrap();
    Curve3::circle_arc(
        point(0, 0),
        Vector3::x(),
        Vector3::y(),
        radius.into(),
        &quarter * Real::from(first_quarter),
        quarter * Real::from(last_quarter),
    )
    .unwrap()
}

fn replay_area(model: &ArenaModel, expected: &Real) {
    let json = model.to_json().unwrap();
    let replay =
        crate::persistence::ImportedModel::from_json(&json, &crate::ArchiveLimits::default())
            .unwrap()
            .validate()
            .unwrap();
    assert_eq!(replay.to_json().unwrap(), json);
    for model in [model, &replay] {
        let face = model.faces().next().unwrap().0;
        assert_eq!(
            compare_reals(
                &model.face_area(face).unwrap(),
                expected,
                crate::STRICT_PREDICATES
            )
            .value(),
            Some(std::cmp::Ordering::Equal)
        );
    }
}

#[test]
fn circular_wire_specialization_falls_back_for_distinct_supports() {
    let model = native_sheet(vec![
        arc(7, 0, 1),
        arc(7, 1, 2),
        Curve3::line(point(-7, 0), point(-3, 0)).unwrap(),
        arc(3, 1, 2).reversed().unwrap(),
        arc(3, 0, 1).reversed().unwrap(),
        Curve3::line(point(3, 0), point(7, 0)).unwrap(),
    ])
    .unwrap();
    replay_area(&model, &(Real::from(20) * Real::pi()));
}

#[test]
fn circular_wire_specialization_falls_back_for_exterior_chords() {
    let model = native_sheet(vec![
        arc(2, 0, 1),
        arc(2, 1, 2),
        Curve3::line(point(-2, 0), point(-3, -3)).unwrap(),
        Curve3::line(point(-3, -3), point(3, -3)).unwrap(),
        Curve3::line(point(3, -3), point(2, 0)).unwrap(),
    ])
    .unwrap();
    replay_area(&model, &(Real::from(15) + Real::from(2) * Real::pi()));
    let crossing = native_sheet(vec![
        arc(2, 0, 1),
        arc(2, 1, 2),
        Curve3::line(point(-2, 0), point(3, -3)).unwrap(),
        Curve3::line(point(3, -3), point(-3, -3)).unwrap(),
        Curve3::line(point(-3, -3), point(2, 0)).unwrap(),
    ]);
    assert!(
        matches!(crossing, Err(BuildError::SelfIntersectingWire(_))),
        "{crossing:?}"
    );
}

#[test]
fn circular_segment_area_preserves_its_certified_sign() {
    let radius = (Real::one() / Real::from(20)).unwrap();
    let start = (Real::pi() / Real::from(6)).unwrap();
    let middle = (Real::pi() / Real::from(2)).unwrap();
    let end = Real::from(5) * &start;
    let first = Curve3::circle_arc(
        point(0, 0),
        Vector3::x(),
        Vector3::y(),
        radius.clone(),
        start,
        middle.clone(),
    )
    .unwrap();
    let second =
        Curve3::circle_arc(point(0, 0), Vector3::x(), Vector3::y(), radius, middle, end).unwrap();
    let chord = Curve3::line(second.end().unwrap(), first.start().unwrap()).unwrap();
    let model = native_sheet(vec![first, second, chord]).unwrap();
    let expected = (Real::pi() / Real::from(1200)).unwrap()
        - (Real::from(3).sqrt().unwrap() / Real::from(1600)).unwrap();
    replay_area(&model, &expected);
}

#[test]
fn single_circular_arc_and_chord_is_simple() {
    let angle = (Real::pi() / Real::from(6)).unwrap();
    let arc = Curve3::circle_arc(
        point(0, 0),
        Vector3::x(),
        Vector3::y(),
        Real::one(),
        Real::zero(),
        angle,
    )
    .unwrap();
    let chord = Curve3::line(arc.end().unwrap(), arc.start().unwrap()).unwrap();
    let model = native_sheet(vec![arc, chord]).unwrap();
    replay_area(
        &model,
        &((Real::pi() / Real::from(12)).unwrap() - (Real::one() / Real::from(4)).unwrap()),
    );
}

#[test]
fn circular_segment_area_replays_split_angular_source() {
    let angle = (Real::pi() / Real::from(6)).unwrap();
    let arc = Curve3::circle_arc(
        point(0, 0),
        Vector3::x(),
        Vector3::y(),
        Real::one(),
        Real::zero(),
        angle.clone(),
    )
    .unwrap();
    let second = Curve3::circle_arc(
        point(0, 0),
        Vector3::x(),
        Vector3::y(),
        Real::one(),
        angle.clone(),
        Real::from(2) * &angle,
    )
    .unwrap();
    let chord = Curve3::line(second.end().unwrap(), arc.start().unwrap()).unwrap();
    let model = native_sheet(vec![arc, second, chord]).unwrap();
    let expected = (Real::pi() / Real::from(6)).unwrap()
        - (Real::from(3).sqrt().unwrap() / Real::from(4)).unwrap();
    replay_area(&model, &expected);
    let first = model.edges().next().unwrap().0;
    let (split, _) = model
        .split_edge(first, (angle / Real::from(2)).unwrap().into())
        .unwrap();
    replay_area(&split, &expected);
}

#[test]
fn split_circular_segment_area_survives_irrational_similarities() {
    let center = point(5, 0);
    let sixth = (Real::pi() / Real::from(6)).unwrap();
    let cuts = [
        Real::from(5) * &sixth,
        Real::pi(),
        Real::from(9) * &sixth,
        Real::tau(),
        Real::from(13) * &sixth,
    ];
    let mut curves = cuts
        .windows(2)
        .map(|range| {
            Curve3::circle_arc(
                center.clone(),
                Vector3::x(),
                Vector3::y(),
                Real::from(2),
                range[0].clone(),
                range[1].clone(),
            )
            .unwrap()
        })
        .collect::<Vec<_>>();
    curves.push(
        Curve3::line(
            curves.last().unwrap().end().unwrap(),
            curves[0].start().unwrap(),
        )
        .unwrap(),
    );
    let model = native_sheet(curves).unwrap();
    let expected = Real::from(8) * &sixth * Real::from(2) + Real::from(3).sqrt().unwrap();
    replay_area(&model, &expected);
    let q = |n, d| (Real::from(n) / Real::from(d)).unwrap();
    let map = |scale: Real| {
        Matrix4::from_row_major([
            Real::zero(),
            Real::zero(),
            scale.clone(),
            Real::from(3),
            q(-3, 5) * &scale,
            q(-4, 5) * &scale,
            Real::zero(),
            Real::from(-2),
            q(-4, 5) * &scale,
            q(3, 5) * &scale,
            Real::zero(),
            Real::from(7),
            Real::zero(),
            Real::zero(),
            Real::zero(),
            Real::one(),
        ])
    };
    let transformed = model
        .transformed(&map(Real::from(2).sqrt().unwrap()))
        .unwrap();
    replay_area(&transformed, &(&expected * Real::from(2)));
    let first = transformed.edges().next().unwrap().0;
    let (split, _) = transformed
        .split_edge(first, (q(11, 2) * sixth).into())
        .unwrap();
    replay_area(&split, &(&expected * Real::from(2)));
    let final_model = split.transformed(&map(q(1, 1000))).unwrap();
    replay_area(&final_model, &(expected * q(2, 1_000_000)));
}
