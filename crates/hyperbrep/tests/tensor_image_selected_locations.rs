//! Retained tensor images consume independent selected queries and survive edits.
use hyperbrep::*;
use hypercurve::{Curve2, LineSeg2, NurbsCurve2, Point2 as Uv};
use std::cmp::Ordering;
fn q(n: i64, d: i64) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
fn selector() -> Curve3 {
    Curve3::rational_bezier(
        [-5, -7, -9, -11, -13, 145]
            .map(|n| Point3::new(q(n, 5), 0.into(), 0.into()))
            .to_vec(),
        vec![Real::one(); 6],
    )
    .unwrap()
}
fn alpha() -> Parameter {
    let CurveParameterLocation::Parameters(mut roots) =
        selector().parameters_of(Point3::origin()).unwrap()
    else {
        panic!("one root")
    };
    assert_eq!(roots.len(), 1);
    let root = roots.remove(0);
    assert!(root.try_real().is_none());
    root
}
fn native_parabola() -> Curve3 {
    Curve3::rational_bezier(
        vec![
            Point3::new(0.into(), (-1).into(), 1.into()),
            Point3::new(0.into(), 0.into(), (-1).into()),
            Point3::new(0.into(), 1.into(), 1.into()),
        ],
        vec![Real::one(); 3],
    )
    .unwrap()
}
fn paraboloid() -> Surface {
    Surface::rational_bezier(
        (0..3)
            .map(|i| {
                (0..3)
                    .map(|j| {
                        let a = [1, -1, 1];
                        let xs = [-1, 0, 1];
                        Point3::new(xs[i].into(), xs[j].into(), (a[i] + a[j]).into())
                    })
                    .collect()
            })
            .collect(),
        vec![vec![Real::one(); 3]; 3],
    )
    .unwrap()
}
fn section(patch: &Surface) -> SurfaceIntersectionCurve {
    let plane = Surface::plane(Point3::origin(), Vector3::y(), Vector3::z()).unwrap();
    match patch.intersect_surface(&plane).unwrap() {
        SurfaceSurfaceIntersection::Curve(c) => *c,
        other => panic!("one complete section: {other:?}"),
    }
}
fn params(curve: &Curve3, point: &GeometricPoint3) -> Vec<Parameter> {
    match curve.parameters_of(point).unwrap() {
        CurveParameterLocation::None => Vec::new(),
        CurveParameterLocation::Parameters(found) => {
            for t in &found {
                assert!(curve.point_at(t).unwrap().coincides_with(point).unwrap());
            }
            for pair in found.windows(2) {
                assert_eq!(pair[0].compare(&pair[1]).unwrap(), Ordering::Less);
            }
            found
        }
        CurveParameterLocation::EntireDomain => panic!("finite locations"),
    }
}
#[track_caller]
fn reload(model: &Model) -> Model {
    let json = model.to_json().unwrap();
    let cold = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(cold.to_json().unwrap(), json);
    cold
}
fn transform() -> Matrix4 {
    Matrix4::from_row_major([1, 2, 0, 3, 0, -1, 0, 4, 0, 0, 2, 5, 0, 0, 0, 1].map(Real::from))
}
#[test]
fn generated_tensor_section_consumes_independent_selected_query() {
    let section = section(&paraboloid());
    assert_eq!(section.curve().kind(), Curve3Kind::TensorImage);
    let a = alpha();
    let native = native_parabola();
    let query = native.point_at(&a).unwrap();
    assert!(query.try_coordinates().is_none());
    let roots = params(&native, &query);
    assert_eq!(roots.len(), 1);
    assert_eq!(roots[0].compare(&a).unwrap(), Ordering::Equal);
    assert!(
        section
            .curve()
            .point_at(&a)
            .unwrap()
            .coincides_with(&query)
            .unwrap()
    );
    let found = params(section.curve(), &query);
    assert_eq!(found.len(), 1);
    assert_eq!(found[0].compare(&a).unwrap(), Ordering::Equal);
    let reversed = section.curve().reversed().unwrap();
    let found = params(&reversed, &query);
    assert_eq!(found.len(), 1);
    assert_eq!(
        found[0]
            .compare(&a.affine_image(&(-1).into(), &1.into()).unwrap())
            .unwrap(),
        Ordering::Equal
    );
    let excluded = section.curve().subcurve(&0.into(), &q(1, 2)).unwrap();
    assert!(params(&excluded, &query).is_empty());
    let kept = section.curve().subcurve(&q(1, 2), &1.into()).unwrap();
    let found = params(&kept, &query);
    assert_eq!(found.len(), 1);
    assert_eq!(
        found[0]
            .compare(&a.affine_image(&2.into(), &(-1).into()).unwrap())
            .unwrap(),
        Ordering::Equal
    );
    let moved = Curve3::affine_image(section.curve().clone(), transform()).unwrap();
    assert_eq!(
        params(&moved, &query.transformed(&transform()).unwrap()).len(),
        1
    );
}
fn paraboloid_face() -> (Model, Surface) {
    let patch = paraboloid();
    let mut builder = ModelBuilder::new();
    let surface = builder.surface(patch.clone()).unwrap();
    let vertices = [(-1, -1), (-1, 1), (1, 1), (1, -1)].map(|(x, y)| {
        builder
            .vertex(Point3::new(x.into(), y.into(), 2.into()).into())
            .unwrap()
    });
    let mut uses = Vec::new();
    for (axis, constant, a, b, from, to, direction) in [
        (
            SurfaceIsoAxis::U,
            0,
            0,
            1,
            (0, 0),
            (1, 0),
            Direction::Forward,
        ),
        (
            SurfaceIsoAxis::V,
            1,
            1,
            2,
            (1, 0),
            (1, 1),
            Direction::Forward,
        ),
        (
            SurfaceIsoAxis::U,
            1,
            3,
            2,
            (0, 1),
            (1, 1),
            Direction::Reversed,
        ),
        (
            SurfaceIsoAxis::V,
            0,
            0,
            3,
            (0, 0),
            (0, 1),
            Direction::Reversed,
        ),
    ] {
        let curve = patch.iso_curve(axis, &Real::from(constant)).unwrap();
        let domain = curve.domain().clone();
        let curve = builder.curve(curve).unwrap();
        let edge = builder
            .edge(&vertices[a], &vertices[b], &curve, domain)
            .unwrap();
        let (from, to, map) = if direction == Direction::Forward {
            (from, to, ParameterCorrespondence::identity())
        } else {
            (
                to,
                from,
                ParameterCorrespondence::affine((-1).into(), 1.into()).unwrap(),
            )
        };
        let pcurve = builder
            .pcurve(Pcurve::new(Curve2::from(
                LineSeg2::try_new(Uv::from_values(from.0, from.1), Uv::from_values(to.0, to.1))
                    .unwrap(),
            )))
            .unwrap();
        uses.push(builder.edge_use(&edge, direction, &pcurve, map).unwrap());
    }
    let wire = builder.wire(&uses).unwrap();
    let face = builder
        .face(&surface, Orientation::Forward, &wire, &[])
        .unwrap();
    builder.shell(&[face]).unwrap();
    (builder.finish().unwrap(), patch)
}
fn continue_replayed_model(model: &Model) {
    let a = alpha();
    let query = native_parabola().point_at(&a).unwrap();
    let edge = model
        .edges()
        .find(|edge| {
            edge.curve().kind() == Curve3Kind::TensorImage
                && edge
                    .domain()
                    .start()
                    .compare(&Real::zero().into())
                    .unwrap()
                    .is_eq()
                && edge.domain().end().try_real().is_none()
        })
        .unwrap();
    assert_eq!(edge.uses().len(), 2);
    let found = params(&edge.curve(), &query);
    assert_eq!(found.len(), 1);
    assert_eq!(
        found[0].compare(edge.domain().end()).unwrap(),
        Ordering::Equal
    );
    let edit = edge
        .split(found[0].affine_image(&q(1, 2), &0.into()).unwrap())
        .unwrap();
    eprintln!("cold second selected split");
    let cold = reload(edit.model());
    assert_eq!(cold.faces().count(), 2);
    let moved = model.transformed(&transform()).unwrap();
    eprintln!("cold affine transformed split");
    let moved = reload(&moved);
    let edge = moved
        .edges()
        .find(|edge| {
            edge.curve().kind() == Curve3Kind::TensorImage
                && edge.domain().end().try_real().is_none()
        })
        .unwrap();
    assert_eq!(
        params(&edge.curve(), &query.transformed(&transform()).unwrap()).len(),
        1
    );
}
#[test]
fn tensor_inverse_splits_both_face_uses_replays_fresh_and_splits_again() {
    const INPUT: &str = "HYPERBREP_TENSOR_INVERSE_REPLAY";
    if let Some(path) = std::env::var_os(INPUT) {
        let json = std::fs::read_to_string(path).unwrap();
        let model = RawModel::from_json(&json, &ArchiveLimits::default())
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(model.to_json().unwrap(), json);
        continue_replayed_model(&model);
        return;
    }
    let (model, patch) = paraboloid_face();
    eprintln!("cold source");
    let model = reload(&model);
    let cut = section(&patch);
    let split = model
        .faces()
        .next()
        .unwrap()
        .split_by_surface_curve(&cut, SurfaceIntersectionOperand::First)
        .unwrap();
    assert_eq!(split.model().faces().count(), 2);
    eprintln!("cold face partition");
    let partition = reload(split.model());
    let edge = partition
        .edges()
        .find(|edge| edge.curve().kind() == Curve3Kind::TensorImage)
        .unwrap();
    assert_eq!(edge.uses().len(), 2);
    let query = native_parabola().point_at(&alpha()).unwrap();
    let found = params(&edge.curve(), &query);
    assert_eq!(found.len(), 1);
    let edit = edge.split(found[0].clone()).unwrap();
    eprintln!("cold first selected split");
    let model = reload(edit.model());
    continue_replayed_model(&model);
    let path = std::env::temp_dir().join(format!(
        "hyperbrep-tensor-inverse-{}.json",
        std::process::id()
    ));
    std::fs::write(&path, model.to_json().unwrap()).unwrap();
    let child = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "tensor_inverse_splits_both_face_uses_replays_fresh_and_splits_again",
            "--nocapture",
        ])
        .env(INPUT, &path)
        .output()
        .unwrap();
    std::fs::remove_file(path).unwrap();
    assert!(
        child.status.success(),
        "{}{}",
        String::from_utf8_lossy(&child.stdout),
        String::from_utf8_lossy(&child.stderr)
    );
}
#[test]
fn native_tensor_sources_keep_all_visits_spline_charts_and_third_coordinate_checks() {
    let a = alpha();
    let source =
        Curve2::from(LineSeg2::try_new(Uv::from_values(0, 0), Uv::from_values(1, 0)).unwrap());
    let patch = Surface::rational_bezier(
        (0..2)
            .map(|v| {
                (0..=3)
                    .map(|i| {
                        Point3::new(
                            if i < 2 {
                                0.into()
                            } else {
                                q((i * (i - 1)) as i64, 6)
                            } + Real::from(v),
                            if i == 3 { 1.into() } else { 0.into() },
                            q(i as i64, 3),
                        )
                    })
                    .collect()
            })
            .collect(),
        vec![vec![Real::one(); 4]; 2],
    )
    .unwrap();
    let curve = Curve3::tensor_image(patch, source.clone()).unwrap();
    let native = Curve3::rational_bezier(
        vec![
            Point3::origin(),
            Point3::new(0.into(), 0.into(), q(1, 3)),
            Point3::new(q(1, 3), 0.into(), q(2, 3)),
            Point3::new(1.into(), 1.into(), 1.into()),
        ],
        vec![Real::one(); 4],
    )
    .unwrap();
    let query = native.point_at(&a).unwrap();
    assert_eq!(params(&curve, &query).len(), 1);
    let shift = Matrix4::from_row_major([
        Real::one(),
        0.into(),
        0.into(),
        0.into(),
        0.into(),
        Real::one(),
        0.into(),
        0.into(),
        0.into(),
        0.into(),
        Real::one(),
        q(1, 1_i64 << 60),
        0.into(),
        0.into(),
        0.into(),
        Real::one(),
    ]);
    assert!(params(&curve, &query.transformed(&shift).unwrap()).is_empty());
    let patch = Surface::rational_bezier(
        (0..2)
            .map(|v| {
                [1, -1, 1]
                    .map(|x| Point3::new(x.into(), 0.into(), v.into()))
                    .to_vec()
            })
            .collect(),
        vec![vec![Real::one(); 3]; 2],
    )
    .unwrap();
    let query = Curve3::rational_bezier(
        vec![
            Point3::origin(),
            Point3::origin(),
            Point3::new(1.into(), 0.into(), 0.into()),
        ],
        vec![Real::one(); 3],
    )
    .unwrap()
    .point_at(&a)
    .unwrap();
    let spline = Curve2::from(
        NurbsCurve2::try_new(
            1,
            vec![
                Uv::from_values(0, 0),
                Uv::new(q(1, 2), 0.into()),
                Uv::from_values(1, 0),
            ],
            vec![Real::one(); 3],
            vec![(-2).into(), (-2).into(), 1.into(), 4.into(), 4.into()],
        )
        .unwrap(),
    );
    for source in [source, spline] {
        let curve = Curve3::tensor_image(patch.clone(), source).unwrap();
        let found = params(&curve, &query);
        assert_eq!(found.len(), 2);
        for (t, scale) in found.iter().zip([-1, 1]) {
            let expected = a.affine_image(&q(scale, 2), &q(1, 2)).unwrap();
            assert_eq!(t.compare(&expected).unwrap(), Ordering::Equal);
        }
        let zero = selector().point_at(&a).unwrap();
        let seam = params(&curve, &zero);
        assert_eq!(seam.len(), 1);
        assert_eq!(seam[0].compare(&q(1, 2).into()).unwrap(), Ordering::Equal);
    }
}
#[test]
fn constant_projection_does_not_hide_axial_preimages_or_whole_collapse() {
    let a = alpha();
    let source = Curve2::from(
        LineSeg2::try_new(Uv::new(0.into(), q(1, 2)), Uv::new(1.into(), q(1, 2))).unwrap(),
    );
    let patch = Surface::rational_bezier(
        (0..2)
            .map(|v| {
                (0..2)
                    .map(|u| Point3::new(v.into(), 0.into(), u.into()))
                    .collect()
            })
            .collect(),
        vec![vec![Real::one(); 2]; 2],
    )
    .unwrap();
    let curve = Curve3::tensor_image(patch, source.clone()).unwrap();
    let query = Curve3::line(
        Point3::new(q(1, 2), 0.into(), 0.into()),
        Point3::new(q(1, 2), 0.into(), 1.into()),
    )
    .unwrap()
    .point_at(&a)
    .unwrap();
    assert_eq!(params(&curve, &query).len(), 1);
    let patch = Surface::rational_bezier(
        (0..2)
            .map(|v| {
                (0..2)
                    .map(|_| Point3::new(v.into(), 0.into(), 0.into()))
                    .collect()
            })
            .collect(),
        vec![vec![Real::one(); 2]; 2],
    )
    .unwrap();
    let constant = Curve3::tensor_image(patch, source).unwrap();
    let point = Point3::new(q(1, 2), 0.into(), 0.into());
    let query = Curve3::rational_bezier(vec![point.clone(); 2], vec![Real::one(); 2])
        .unwrap()
        .point_at(&a)
        .unwrap();
    assert!(query.try_coordinates().is_none());
    assert!(matches!(
        constant.parameters_of(&query).unwrap(),
        CurveParameterLocation::EntireDomain
    ));
    assert!(matches!(
        constant.parameters_of(point).unwrap(),
        CurveParameterLocation::EntireDomain
    ));
    assert!(params(&constant, &Point3::new(q(1, 2), 0.into(), 1.into()).into()).is_empty());
}
#[test]
fn tensor_inverse_resource_refusal_preserves_a_larger_retry() {
    let curve = section(&paraboloid());
    let a = alpha();
    let query = native_parabola().point_at(&a).unwrap();
    let denied =
        OperationContext::strict(WorkBudget::ZERO).evaluate(|| curve.curve().parameters_of(&query));
    let exhausted = denied.result.unwrap_err();
    assert_eq!(exhausted.limit, 0);
    assert_eq!(exhausted.used, 0);
    assert!(exhausted.requested > 0);
    let retry = OperationContext::strict(WorkBudget {
        refinements: 100_000,
        promotions: 10_000,
    })
    .evaluate(|| curve.curve().parameters_of(&query));
    let CurveParameterLocation::Parameters(found) = retry.result.unwrap().unwrap() else {
        panic!("finite inverse")
    };
    assert_eq!(found.len(), 1);
    assert_eq!(found[0].compare(&a).unwrap(), Ordering::Equal);
}

#[test]
fn mixed_projection_components_recover_all_finite_spatial_visits() {
    // S(u,v)=(u(u-v),v(v-u),uv); the two UV axes and diagonal map
    // to three distinct spatial axes. Every planar projection collapses
    // one pair of source spans, although the spatial inverse is finite.
    let patch = Surface::rational_bezier(
        (0..3)
            .map(|j| {
                (0..3)
                    .map(|i| {
                        let uv = q(i * j, 4);
                        Point3::new(
                            Real::from(if i == 2 { 1 } else { 0 }) - &uv,
                            Real::from(if j == 2 { 1 } else { 0 }) - &uv,
                            uv,
                        )
                    })
                    .collect()
            })
            .collect(),
        vec![vec![Real::one(); 3]; 3],
    )
    .unwrap();
    let source = Curve2::from(
        NurbsCurve2::try_new(
            1,
            vec![
                Uv::from_values(0, 0),
                Uv::from_values(1, 0),
                Uv::from_values(0, 0),
                Uv::from_values(0, 1),
                Uv::from_values(0, 0),
                Uv::from_values(1, 1),
                Uv::from_values(0, 0),
            ],
            vec![Real::one(); 7],
            vec![
                0.into(),
                0.into(),
                1.into(),
                2.into(),
                3.into(),
                4.into(),
                5.into(),
                6.into(),
                6.into(),
            ],
        )
        .unwrap(),
    );
    let curve = Curve3::tensor_image(patch, source).unwrap();
    let query = selector().point_at(&alpha()).unwrap();
    assert!(query.try_coordinates().is_none());
    // The six spans have squared linear nonzero coordinates. Only the
    // four returns lambda=0,2,4,6 hit zero; the spatial chart is lambda/6.
    let expected = [Real::zero(), q(1, 3), q(2, 3), Real::one()];
    for t in &expected {
        assert!(
            curve
                .point_at(&t.clone().into())
                .unwrap()
                .coincides_with(&query)
                .unwrap()
        );
    }
    let CurveParameterLocation::Parameters(found) = curve.parameters_of(&query).unwrap() else {
        panic!("complete finite spatial inverse")
    };
    assert_eq!(found.len(), expected.len());
    for (actual, expected) in found.iter().zip(expected) {
        assert_eq!(actual.compare(&expected.into()).unwrap(), Ordering::Equal);
    }
    let trimmed = curve.subcurve(&q(1, 6), &q(5, 6)).unwrap();
    let visits = params(&trimmed, &query);
    assert_eq!(visits.len(), 2);
    for (visit, expected) in visits.iter().zip([q(1, 4), q(3, 4)]) {
        assert_eq!(visit.compare(&expected.into()).unwrap(), Ordering::Equal);
    }
    let reversed = trimmed.reversed().unwrap();
    let moved = Curve3::affine_image(reversed, transform()).unwrap();
    let visits = params(&moved, &query.transformed(&transform()).unwrap());
    assert_eq!(visits.len(), 2);
    for (visit, expected) in visits.iter().zip([q(1, 4), q(3, 4)]) {
        assert_eq!(visit.compare(&expected.into()).unwrap(), Ordering::Equal);
    }
}
