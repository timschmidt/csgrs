//! Solids capped by a rational Bézier tensor graph.
//!
//! A closed shell of a tensor top, a planar base and planar walls parallel to
//! the base normal is admitted when the top is certified as a graph strictly
//! above the base; points are classified exactly against it.

use hyperbrep::*;
use hypercurve::{Curve2, LineSeg2, Point2 as Uv, RationalBezier2};

fn q(n: i64, d: i64) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
fn pi(x: i64, y: i64, z: i64) -> Point3 {
    Point3::new(x.into(), y.into(), z.into())
}
fn uv(u: i64, v: i64) -> Uv {
    Uv::from_values(u, v)
}
fn line2(a: Uv, b: Uv) -> Pcurve {
    Pcurve::new(Curve2::from(LineSeg2::try_new(a, b).unwrap()))
}
fn quad2(p: [(i64, i64); 3]) -> Pcurve {
    Pcurve::new(Curve2::from(
        RationalBezier2::try_new(
            p.iter().map(|&(a, b)| Uv::from_values(a, b)).collect(),
            vec![Real::one(); 3],
        )
        .unwrap(),
    ))
}

/// The paraboloid top z = x^2 + y^2 over [-1, 1]^2 (x = 2u - 1, y = 2v - 1,
/// so the tensor normal points up), the base z = -1 and four planar walls.
/// Each wall carries its top edge as the exact quadratic Bézier pcurve.
fn bump() -> Model {
    let xs = [-1, 0, 1];
    let a = [1, -1, 1];
    let controls: Vec<Vec<Point3>> = (0..3)
        .map(|i| (0..3).map(|j| pi(xs[j], xs[i], a[i] + a[j])).collect())
        .collect();
    let top = Surface::rational_bezier(controls.clone(), vec![vec![Real::one(); 3]; 3]).unwrap();
    let mut b = ModelBuilder::new();
    let [va, vb, vc, vd] = [pi(-1, -1, -1), pi(1, -1, -1), pi(1, 1, -1), pi(-1, 1, -1)]
        .map(|p| b.vertex(p.into()).unwrap());
    let [ta, tb, tc, td] = [pi(-1, -1, 2), pi(1, -1, 2), pi(1, 1, 2), pi(-1, 1, 2)]
        .map(|p| b.vertex(p.into()).unwrap());
    let edge = |b: &mut ModelBuilder, s, e, c: Curve3| {
        let id = b.curve(c.clone()).unwrap();
        b.edge(s, e, &id, c.domain().clone()).unwrap()
    };
    let line = |p: Point3, q: Point3| Curve3::line(p, q).unwrap();
    let e_ab = edge(&mut b, &va, &vb, line(pi(-1, -1, -1), pi(1, -1, -1)));
    let e_bc = edge(&mut b, &vb, &vc, line(pi(1, -1, -1), pi(1, 1, -1)));
    let e_cd = edge(&mut b, &vc, &vd, line(pi(1, 1, -1), pi(-1, 1, -1)));
    let e_da = edge(&mut b, &vd, &va, line(pi(-1, 1, -1), pi(-1, -1, -1)));
    let v_a = edge(&mut b, &va, &ta, line(pi(-1, -1, -1), pi(-1, -1, 2)));
    let v_b = edge(&mut b, &vb, &tb, line(pi(1, -1, -1), pi(1, -1, 2)));
    let v_c = edge(&mut b, &vc, &tc, line(pi(1, 1, -1), pi(1, 1, 2)));
    let v_d = edge(&mut b, &vd, &td, line(pi(-1, 1, -1), pi(-1, 1, 2)));
    let bez = |pts: Vec<Point3>| Curve3::rational_bezier(pts, vec![Real::one(); 3]).unwrap();
    let col = |j: usize| {
        controls
            .iter()
            .map(|row| row[j].clone())
            .collect::<Vec<_>>()
    };
    let t_row0 = edge(&mut b, &ta, &tb, bez(controls[0].clone()));
    let t_col2 = edge(&mut b, &tb, &tc, bez(col(2)));
    let t_row2 = edge(&mut b, &td, &tc, bez(controls[2].clone()));
    let t_col0 = edge(&mut b, &ta, &td, bez(col(0)));
    let id = ParameterCorrespondence::identity();
    let rev = || ParameterCorrespondence::affine(-Real::one(), Real::one()).unwrap();
    let face = |b: &mut ModelBuilder,
                surface: Surface,
                uses: Vec<(&EdgeKey, Direction, Pcurve, ParameterCorrespondence)>| {
        let s = b.surface(surface).unwrap();
        let uses = uses
            .into_iter()
            .map(|(e, d, p, c)| {
                let p = b.pcurve(p).unwrap();
                b.edge_use(e, d, &p, c).unwrap()
            })
            .collect::<Vec<_>>();
        let w = b.wire(&uses).unwrap();
        b.face(&s, Orientation::Forward, &w, &[]).unwrap()
    };
    use Direction::{Forward as F, Reversed as R};
    let plane = |o: Point3, u: Vector3, v: Vector3| Surface::plane(o, u, v).unwrap();
    let f_top = face(
        &mut b,
        top,
        vec![
            (&t_row0, F, line2(uv(0, 0), uv(1, 0)), id.clone()),
            (&t_col2, F, line2(uv(1, 0), uv(1, 1)), id.clone()),
            (&t_row2, R, line2(uv(1, 1), uv(0, 1)), rev()),
            (&t_col0, R, line2(uv(0, 1), uv(0, 0)), rev()),
        ],
    );
    let f_bottom = face(
        &mut b,
        plane(pi(0, 0, -1), Vector3::y(), Vector3::x()),
        vec![
            (&e_da, R, line2(uv(-1, -1), uv(1, -1)), rev()),
            (&e_cd, R, line2(uv(1, -1), uv(1, 1)), rev()),
            (&e_bc, R, line2(uv(1, 1), uv(-1, 1)), rev()),
            (&e_ab, R, line2(uv(-1, 1), uv(-1, -1)), rev()),
        ],
    );
    let f_ym = face(
        &mut b,
        plane(pi(0, -1, 0), Vector3::x(), Vector3::z()),
        vec![
            (&e_ab, F, line2(uv(-1, -1), uv(1, -1)), id.clone()),
            (&v_b, F, line2(uv(1, -1), uv(1, 2)), id.clone()),
            (&t_row0, R, quad2([(1, 2), (0, 0), (-1, 2)]), rev()),
            (&v_a, R, line2(uv(-1, 2), uv(-1, -1)), rev()),
        ],
    );
    let f_xp = face(
        &mut b,
        plane(pi(1, 0, 0), Vector3::y(), Vector3::z()),
        vec![
            (&e_bc, F, line2(uv(-1, -1), uv(1, -1)), id.clone()),
            (&v_c, F, line2(uv(1, -1), uv(1, 2)), id.clone()),
            (&t_col2, R, quad2([(1, 2), (0, 0), (-1, 2)]), rev()),
            (&v_b, R, line2(uv(-1, 2), uv(-1, -1)), rev()),
        ],
    );
    let f_yp = face(
        &mut b,
        plane(pi(0, 1, 0), -Vector3::x(), Vector3::z()),
        vec![
            (&e_cd, F, line2(uv(-1, -1), uv(1, -1)), id.clone()),
            (&v_d, F, line2(uv(1, -1), uv(1, 2)), id.clone()),
            (&t_row2, F, quad2([(1, 2), (0, 0), (-1, 2)]), id.clone()),
            (&v_c, R, line2(uv(-1, 2), uv(-1, -1)), rev()),
        ],
    );
    let f_xm = face(
        &mut b,
        plane(pi(-1, 0, 0), -Vector3::y(), Vector3::z()),
        vec![
            (&e_da, F, line2(uv(-1, -1), uv(1, -1)), id.clone()),
            (&v_a, F, line2(uv(1, -1), uv(1, 2)), id.clone()),
            (&t_col0, F, quad2([(1, 2), (0, 0), (-1, 2)]), id.clone()),
            (&v_d, R, line2(uv(-1, 2), uv(-1, -1)), rev()),
        ],
    );
    let shell = b.shell(&[f_top, f_bottom, f_ym, f_xp, f_yp, f_xm]).unwrap();
    b.solid(&shell, &[]).unwrap();
    b.finish().unwrap()
}

/// Expected locations follow from z = x^2 + y^2 over [-1, 1]^2 above z = -1.
fn expected_locations() -> Vec<(Point3, SolidPointLocation)> {
    vec![
        (pi(0, 0, 0), SolidPointLocation::Boundary),
        (
            Point3::new(0.into(), 0.into(), q(-1, 2)),
            SolidPointLocation::Inside,
        ),
        (
            Point3::new(q(1, 2), q(1, 2), q(1, 4)),
            SolidPointLocation::Inside,
        ),
        (
            Point3::new(q(1, 2), q(1, 2), q(1, 2)),
            SolidPointLocation::Boundary,
        ),
        (
            Point3::new(q(1, 2), q(1, 2), q(3, 4)),
            SolidPointLocation::Outside,
        ),
        (pi(2, 0, 0), SolidPointLocation::Outside),
        (pi(0, 0, -1), SolidPointLocation::Boundary),
        (pi(0, 0, -2), SolidPointLocation::Outside),
        (
            Point3::new(1.into(), 0.into(), q(1, 2)),
            SolidPointLocation::Boundary,
        ),
        (
            Point3::new(1.into(), 0.into(), q(3, 2)),
            SolidPointLocation::Outside,
        ),
        (
            Point3::new(q(9, 10), q(-9, 10), q(161, 100)),
            SolidPointLocation::Inside,
        ),
    ]
}

#[test]
fn tensor_graph_solids_are_admitted_and_classified_exactly() {
    let model = bump();
    assert_eq!(model.faces().count(), 6);
    let solid = model.solids().next().unwrap();
    for (point, expected) in expected_locations() {
        assert_eq!(solid.classify_point(&point).unwrap(), expected, "{point:?}");
    }
}

#[test]
fn tensor_graph_solids_replay_cold_and_transform() {
    let model = bump();
    let json = model.to_json().unwrap();
    let cold = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(cold.to_json().unwrap(), json);
    let solid = cold.solids().next().unwrap();
    for (point, expected) in expected_locations() {
        assert_eq!(solid.classify_point(&point).unwrap(), expected);
    }
    // A scaled rotation about z with a translation maps each probe point.
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
    let mapped = model.transformed(&transform).unwrap();
    let solid = mapped.solids().next().unwrap();
    for (point, expected) in expected_locations() {
        let image = Point3::new(
            q(6, 5) * &point.x - q(8, 5) * &point.y + Real::one(),
            q(8, 5) * &point.x + q(6, 5) * &point.y + Real::from(2),
            Real::from(2) * &point.z + Real::from(3),
        );
        assert_eq!(solid.classify_point(&image).unwrap(), expected);
    }
}

/// The cuboid x <= 1/2 cuts the tensor top along the iso section x = 1/2
/// (z = 1/4 + y^2) and the walls y = +/-1 along lines ending on their Bézier
/// edges at (1/2, +/-1, 5/4). The intersection keeps x <= 1/2 and the
/// difference keeps x >= 1/2; each is again a tensor-graph solid (six faces)
/// that classifies exactly and replays cold. The union's shell is outside
/// every admitted class, a typed refusal.
#[test]
fn booleans_split_tensor_graph_solids_exactly() {
    let body = BodySet::from_solid(bump().solids().next().unwrap());
    let cut = builder::cuboid(pi(-2, -2, -2), Point3::new(q(1, 2), 2.into(), 3.into())).unwrap();
    let common = boolean::intersection(&body, &cut).unwrap();
    assert_eq!(common.model().faces().count(), 6);
    let reloaded = RawModel::from_json(
        &common.model().to_json().unwrap(),
        &ArchiveLimits::default(),
    )
    .unwrap()
    .validate()
    .unwrap();
    for model in [common.model(), &reloaded] {
        let solid = model.solids().next().unwrap();
        for (point, expected) in [
            (pi(0, 0, 0), SolidPointLocation::Boundary),
            (
                Point3::new(q(1, 4), 0.into(), 0.into()),
                SolidPointLocation::Inside,
            ),
            (
                Point3::new(q(3, 4), 0.into(), 0.into()),
                SolidPointLocation::Outside,
            ),
            (
                Point3::new(q(1, 2), 0.into(), 0.into()),
                SolidPointLocation::Boundary,
            ),
            (
                Point3::new(q(1, 2), 0.into(), q(1, 2)),
                SolidPointLocation::Outside,
            ),
            (
                Point3::new(q(-1, 2), 0.into(), q(-1, 2)),
                SolidPointLocation::Inside,
            ),
        ] {
            assert_eq!(solid.classify_point(&point).unwrap(), expected, "{point:?}");
        }
    }
    let remainder = boolean::difference(&body, &cut).unwrap();
    assert_eq!(remainder.model().faces().count(), 6);
    let solid = remainder.bodies().next().unwrap();
    for (point, expected) in [
        (
            Point3::new(q(3, 4), 0.into(), 0.into()),
            SolidPointLocation::Inside,
        ),
        (
            Point3::new(q(1, 4), 0.into(), 0.into()),
            SolidPointLocation::Outside,
        ),
        (
            Point3::new(q(1, 2), 0.into(), 0.into()),
            SolidPointLocation::Boundary,
        ),
        (
            Point3::new(q(3, 4), 0.into(), q(1, 2)),
            SolidPointLocation::Inside,
        ),
        (
            Point3::new(q(3, 4), 0.into(), q(3, 4)),
            SolidPointLocation::Outside,
        ),
    ] {
        assert_eq!(solid.classify_point(&point).unwrap(), expected, "{point:?}");
    }
    assert!(boolean::union(&body, &cut).is_err());
}
