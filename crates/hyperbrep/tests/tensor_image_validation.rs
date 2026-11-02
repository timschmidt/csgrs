//! Whole-image validation of generated tensor-image edges against their
//! retained tensor pcurves, through replay and affine transport.
//!
//! The generated curves are the branches of the perpendicular cylinder pair
//! x^2 + y^2 = 1, y^2 + z^2 = 2 on an exact rational patch of the second.

use hyperbrep::*;

fn q(n: i64, d: i64) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn p(x: Real, y: Real, z: Real) -> Point3 {
    Point3::new(x, y, z)
}

fn cylinder_patch() -> Surface {
    let arc = [
        (q(7, 5), q(1, 5), Real::one()),
        (q(5, 4), q(5, 4), q(4, 5)),
        (q(1, 5), q(7, 5), Real::one()),
    ];
    Surface::rational_bezier(
        arc.iter()
            .map(|(y, z, _)| {
                vec![
                    p((-2).into(), y.clone(), z.clone()),
                    p(2.into(), y.clone(), z.clone()),
                ]
            })
            .collect(),
        arc.iter()
            .map(|(_, _, w)| vec![w.clone(), w.clone()])
            .collect(),
    )
    .unwrap()
}

fn branches() -> Vec<SurfaceIntersectionCurve> {
    let cylinder = Surface::cylinder(
        Point3::origin(),
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        1.into(),
    )
    .unwrap();
    match cylinder.intersect_surface(&cylinder_patch()).unwrap() {
        SurfaceSurfaceIntersection::Curves(curves) => curves,
        other => panic!("two branches required, found {other:?}"),
    }
}

/// Stages one edge per branch on the patch with the retained tensor pcurves,
/// optionally pairing each edge with another branch's pcurve.
fn validate(
    sections: &[SurfaceIntersectionCurve],
    surface: &Surface,
    pcurve_of: impl Fn(usize) -> usize,
) -> Vec<Result<(), BuildError>> {
    let mut builder = ModelBuilder::new();
    let support = builder.surface(surface.clone()).unwrap();
    let mut results = Vec::new();
    for (index, section) in sections.iter().enumerate() {
        let curve = section.curve();
        let a = builder
            .vertex(curve.point_at(curve.domain().start()).unwrap())
            .unwrap();
        let b = builder
            .vertex(curve.point_at(curve.domain().end()).unwrap())
            .unwrap();
        let source = builder.curve(curve.clone()).unwrap();
        let edge = builder
            .edge(&a, &b, &source, curve.domain().clone())
            .unwrap();
        let materialized = sections[pcurve_of(index)]
            .second_pcurve()
            .materialize()
            .unwrap();
        let (scale, offset) = materialized.correspondence().affine_coefficients().unwrap();
        let pcurve = builder
            .pcurve(Pcurve::new(materialized.curve().clone()))
            .unwrap();
        results.push(builder.validate_edge_image(
            &edge,
            Direction::Forward,
            &pcurve,
            ParameterCorrespondence::affine(scale.clone(), offset.clone()).unwrap(),
            &support,
        ));
    }
    results
}

#[test]
fn generated_branches_validate_against_their_tensor_pcurves() {
    let sections = branches();
    for result in validate(&sections, &cylinder_patch(), |index| index) {
        result.unwrap();
    }
}

/// Exchanged pcurves or a different tensor are whole-image mismatches, even
/// though the branches share their event endpoint (0, 1, 1).
#[test]
fn mismatched_pcurves_and_tensors_are_rejected() {
    let sections = branches();
    for result in validate(&sections, &cylinder_patch(), |index| 1 - index) {
        assert!(result.is_err());
    }
    let shifted = Surface::rational_bezier(
        [
            (q(7, 5), q(1, 5), Real::one()),
            (q(5, 4), q(5, 4), q(4, 5)),
            (q(1, 5), q(7, 5), Real::one()),
        ]
        .iter()
        .map(|(y, z, _)| {
            vec![
                p((-2).into(), y.clone(), z.clone() + Real::one()),
                p(2.into(), y.clone(), z.clone() + Real::one()),
            ]
        })
        .collect(),
        vec![
            vec![Real::one(); 2],
            vec![q(4, 5), q(4, 5)],
            vec![Real::one(); 2],
        ],
    )
    .unwrap();
    for result in validate(&sections, &shifted, |index| index) {
        assert!(result.is_err());
    }
}

/// A validated model with both branch edges and their tensor pcurves replays
/// cold, and its transformed image validates again on the transformed patch.
#[test]
fn validated_edges_replay_and_transform() {
    let sections = branches();
    let mut builder = ModelBuilder::new();
    let support = builder.surface(cylinder_patch()).unwrap();
    for section in &sections {
        let curve = section.curve();
        let a = builder
            .vertex(curve.point_at(curve.domain().start()).unwrap())
            .unwrap();
        let b = builder
            .vertex(curve.point_at(curve.domain().end()).unwrap())
            .unwrap();
        let source = builder.curve(curve.clone()).unwrap();
        let edge = builder
            .edge(&a, &b, &source, curve.domain().clone())
            .unwrap();
        let materialized = section.second_pcurve().materialize().unwrap();
        let (scale, offset) = materialized.correspondence().affine_coefficients().unwrap();
        let pcurve = builder
            .pcurve(Pcurve::new(materialized.curve().clone()))
            .unwrap();
        builder
            .validate_edge_image(
                &edge,
                Direction::Forward,
                &pcurve,
                ParameterCorrespondence::affine(scale.clone(), offset.clone()).unwrap(),
                &support,
            )
            .unwrap();
    }
    let model = builder.finish().unwrap();
    let json = model.to_json().unwrap();
    let cold = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(cold.to_json().unwrap(), json);
    // Uniform scale 2, rotation by (3/5, 4/5) about z, translation (1, 2, 3).
    let transform = Matrix4::from_row_major([
        q(6, 5),
        q(-8, 5),
        0.into(),
        1.into(),
        q(8, 5),
        q(6, 5),
        0.into(),
        2.into(),
        0.into(),
        0.into(),
        2.into(),
        3.into(),
        0.into(),
        0.into(),
        0.into(),
        1.into(),
    ]);
    let mapped = cold.transformed(&transform).unwrap();
    let surface = mapped.surfaces().next().unwrap().geometry().clone();
    let mapped_sections = mapped
        .curves()
        .zip(&sections)
        .map(|(curve, section)| (curve.geometry().clone(), section))
        .collect::<Vec<_>>();
    let mut builder = ModelBuilder::new();
    let support = builder.surface(surface).unwrap();
    for (curve, section) in mapped_sections {
        let a = builder
            .vertex(curve.point_at(curve.domain().start()).unwrap())
            .unwrap();
        let b = builder
            .vertex(curve.point_at(curve.domain().end()).unwrap())
            .unwrap();
        let domain = curve.domain().clone();
        let source = builder.curve(curve).unwrap();
        let edge = builder.edge(&a, &b, &source, domain).unwrap();
        let materialized = section.second_pcurve().materialize().unwrap();
        let (scale, offset) = materialized.correspondence().affine_coefficients().unwrap();
        let pcurve = builder
            .pcurve(Pcurve::new(materialized.curve().clone()))
            .unwrap();
        builder
            .validate_edge_image(
                &edge,
                Direction::Forward,
                &pcurve,
                ParameterCorrespondence::affine(scale.clone(), offset.clone()).unwrap(),
                &support,
            )
            .unwrap();
    }
}
