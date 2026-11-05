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
/// `|value - expected| < 10^-30`, decided exactly (no rounding).
fn assert_within(value: &Real, expected: &Real) {
    let scale = (0..30).fold(Real::one(), |value, _| value * Real::from(10));
    let tolerance = (Real::one() / scale).unwrap();
    let difference = (value.clone() - expected.clone()).abs();
    assert_eq!(
        difference.partial_cmp(&tolerance),
        Some(std::cmp::Ordering::Less),
        "{:?} vs {:?}",
        value.to_f64_lossy(),
        expected.to_f64_lossy()
    );
}

/// `91/12 + 25 pi/32 - (25/8) asin(2 sqrt(5)/5)`: the integral of
/// `min(x² + y², 5/4) + 1` over [-1,1]², derived in closed form.
fn slab_cut_volume() -> Real {
    let angle = (Real::from(2) * Real::from(5).sqrt().unwrap() / Real::from(5))
        .unwrap()
        .asin()
        .unwrap();
    q(91, 12) + q(25, 32) * Real::pi() - q(25, 8) * angle
}

fn quad2(p: [(i64, i64); 3]) -> Pcurve {
    quad2_over(p.map(|(a, b)| (Real::from(a), b)))
}

fn quad2_over(p: [(Real, i64); 3]) -> Pcurve {
    Pcurve::new(Curve2::from(
        RationalBezier2::try_new(
            p.into_iter().map(|(a, b)| Uv::new(a, b.into())).collect(),
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

/// A tensor-graph solid over [-1,1]² with base z = -1 and quadratic top
/// controls `h[i][j]` at (x_j, y_i), x, y in {-1, 0, 1}; the same topology as
/// the bump, with walls following the top's boundary Bézier rows.
fn graph_fixture(h: [[i64; 3]; 3]) -> Model {
    graph_fixture_over([-1, 0, 1].map(Real::from), h)
}

/// The graph fixture whose column controls sit at the abscissas `xs`
/// (from -1 to 1), so a nonuniform `xs` makes `x(u)` quadratic.
fn graph_fixture_over(xs: [Real; 3], h: [[i64; 3]; 3]) -> Model {
    let ys = [-1, 0, 1];
    let at = |i: usize, j: usize| Point3::new(xs[j].clone(), ys[i].into(), h[i][j].into());
    let controls: Vec<Vec<Point3>> = (0..3).map(|i| (0..3).map(|j| at(i, j)).collect()).collect();
    let top = Surface::rational_bezier(controls.clone(), vec![vec![Real::one(); 3]; 3]).unwrap();
    let mut b = ModelBuilder::new();
    let [va, vb, vc, vd] = [pi(-1, -1, -1), pi(1, -1, -1), pi(1, 1, -1), pi(-1, 1, -1)]
        .map(|p| b.vertex(p.into()).unwrap());
    let corner = [
        pi(-1, -1, h[0][0]),
        pi(1, -1, h[0][2]),
        pi(1, 1, h[2][2]),
        pi(-1, 1, h[2][0]),
    ];
    let [ta, tb, tc, td] = corner.clone().map(|p| b.vertex(p.into()).unwrap());
    let edge = |b: &mut ModelBuilder, s, e, c: Curve3| {
        let id = b.curve(c.clone()).unwrap();
        b.edge(s, e, &id, c.domain().clone()).unwrap()
    };
    let line = |p: Point3, q: Point3| Curve3::line(p, q).unwrap();
    let e_ab = edge(&mut b, &va, &vb, line(pi(-1, -1, -1), pi(1, -1, -1)));
    let e_bc = edge(&mut b, &vb, &vc, line(pi(1, -1, -1), pi(1, 1, -1)));
    let e_cd = edge(&mut b, &vc, &vd, line(pi(1, 1, -1), pi(-1, 1, -1)));
    let e_da = edge(&mut b, &vd, &va, line(pi(-1, 1, -1), pi(-1, -1, -1)));
    let v_a = edge(&mut b, &va, &ta, line(pi(-1, -1, -1), corner[0].clone()));
    let v_b = edge(&mut b, &vb, &tb, line(pi(1, -1, -1), corner[1].clone()));
    let v_c = edge(&mut b, &vc, &tc, line(pi(1, 1, -1), corner[2].clone()));
    let v_d = edge(&mut b, &vd, &td, line(pi(-1, 1, -1), corner[3].clone()));
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
    let up = |u: i64, z: i64| uv(u, z);
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
            (&v_b, F, line2(uv(1, -1), up(1, h[0][2])), id.clone()),
            (
                &t_row0,
                R,
                quad2_over([
                    (xs[2].clone(), h[0][2]),
                    (xs[1].clone(), h[0][1]),
                    (xs[0].clone(), h[0][0]),
                ]),
                rev(),
            ),
            (&v_a, R, line2(up(-1, h[0][0]), uv(-1, -1)), rev()),
        ],
    );
    let f_xp = face(
        &mut b,
        plane(pi(1, 0, 0), Vector3::y(), Vector3::z()),
        vec![
            (&e_bc, F, line2(uv(-1, -1), uv(1, -1)), id.clone()),
            (&v_c, F, line2(uv(1, -1), up(1, h[2][2])), id.clone()),
            (
                &t_col2,
                R,
                quad2([(1, h[2][2]), (0, h[1][2]), (-1, h[0][2])]),
                rev(),
            ),
            (&v_b, R, line2(up(-1, h[0][2]), uv(-1, -1)), rev()),
        ],
    );
    let f_yp = face(
        &mut b,
        plane(pi(0, 1, 0), -Vector3::x(), Vector3::z()),
        vec![
            (&e_cd, F, line2(uv(-1, -1), uv(1, -1)), id.clone()),
            (&v_d, F, line2(uv(1, -1), up(1, h[2][0])), id.clone()),
            (
                &t_row2,
                F,
                quad2_over([
                    (-xs[0].clone(), h[2][0]),
                    (-xs[1].clone(), h[2][1]),
                    (-xs[2].clone(), h[2][2]),
                ]),
                id.clone(),
            ),
            (&v_c, R, line2(up(-1, h[2][2]), uv(-1, -1)), rev()),
        ],
    );
    let f_xm = face(
        &mut b,
        plane(pi(-1, 0, 0), -Vector3::y(), Vector3::z()),
        vec![
            (&e_da, F, line2(uv(-1, -1), uv(1, -1)), id.clone()),
            (&v_a, F, line2(uv(1, -1), up(1, h[0][0])), id.clone()),
            (
                &t_col0,
                F,
                quad2([(1, h[0][0]), (0, h[1][0]), (-1, h[2][0])]),
                id.clone(),
            ),
            (&v_d, R, line2(up(-1, h[2][0]), uv(-1, -1)), rev()),
        ],
    );
    let shell = b.shell(&[f_top, f_bottom, f_ym, f_xp, f_yp, f_xm]).unwrap();
    b.solid(&shell, &[]).unwrap();
    b.finish().unwrap()
}

/// The ramp: top z = x² + y + 1 (controls a_j + b_i with a = [1, -1, 1] and
/// b = [0, 1, 2]) over [-1,1]², above z = -1; its volume is 28/3.
fn ramp() -> Model {
    let a = [1, -1, 1];
    let b = [0, 1, 2];
    graph_fixture([0, 1, 2].map(|i| [0, 1, 2].map(|j| a[j] + b[i])))
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
    // Volume: integral over [-1, 1]^2 of (x^2 + y^2 + 1) = 4/3 + 4/3 + 4.
    assert_eq!(solid.volume().unwrap(), q(20, 3));
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
    // The linear part has determinant (36/25 + 64/25) * 2 = 8.
    assert_eq!(solid.volume().unwrap(), q(160, 3));
    assert_eq!(cold.solids().next().unwrap().volume().unwrap(), q(20, 3));
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
    // Volume of x <= 1/2: integral_{-1}^{1/2} (2x^2 + 8/3) dx = 3/4 + 4.
    assert_eq!(common.bodies().next().unwrap().volume().unwrap(), q(19, 4));
    let remainder = boolean::difference(&body, &cut).unwrap();
    // The complement: 20/3 - 19/4.
    assert_eq!(
        remainder.bodies().next().unwrap().volume().unwrap(),
        q(23, 12)
    );
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

/// Boolean output as input: the x <= 1/2 intersection, replayed cold, is
/// intersected with y <= 1/2. The second cut crosses the first cut's wall and
/// iso section, and the corner solid has volume
/// integral_{-1}^{1/2} integral_{-1}^{1/2} (x^2 + y^2 + 1) = 9/8 + 9/4.
#[test]
fn chained_booleans_on_tensor_graph_solids_stay_exact() {
    let body = BodySet::from_solid(bump().solids().next().unwrap());
    let cut_x = builder::cuboid(pi(-2, -2, -2), Point3::new(q(1, 2), 2.into(), 3.into())).unwrap();
    let first = boolean::intersection(&body, &cut_x).unwrap();
    let cold = RawModel::from_json(&first.model().to_json().unwrap(), &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    let first = BodySet::from_solid(cold.solids().next().unwrap());
    let cut_y = builder::cuboid(pi(-2, -2, -2), Point3::new(2.into(), q(1, 2), 3.into())).unwrap();
    let corner = boolean::intersection(&first, &cut_y).unwrap();
    assert_eq!(corner.len(), 1);
    assert_eq!(corner.model().faces().count(), 6);
    let solid = corner.bodies().next().unwrap();
    assert_eq!(solid.volume().unwrap(), q(27, 8));
    for (point, expected) in [
        (
            Point3::new(q(-1, 2), q(-1, 2), 0.into()),
            SolidPointLocation::Inside,
        ),
        (
            Point3::new(q(3, 4), 0.into(), 0.into()),
            SolidPointLocation::Outside,
        ),
        (
            Point3::new(0.into(), q(3, 4), 0.into()),
            SolidPointLocation::Outside,
        ),
        (
            Point3::new(q(1, 2), q(1, 2), q(1, 4)),
            SolidPointLocation::Boundary,
        ),
        (
            Point3::new(q(1, 2), q(1, 2), q(3, 4)),
            SolidPointLocation::Outside,
        ),
    ] {
        assert_eq!(solid.classify_point(&point).unwrap(), expected, "{point:?}");
    }
    let replayed = RawModel::from_json(
        &corner.model().to_json().unwrap(),
        &ArchiveLimits::default(),
    )
    .unwrap()
    .validate()
    .unwrap();
    assert_eq!(
        replayed.solids().next().unwrap().volume().unwrap(),
        q(27, 8)
    );
}

/// The plane z = 5/4 meets the bump top z = x² + y² in the circle of radius
/// √5/2, which leaves the unit square: four corner arcs end on the square's
/// sides at (±1, ±1/2) and (±1/2, ±1). Each arc is a similarity-mapped
/// polynomial branch whose ends are selected points; the unit cap face
/// splits along all four into four corners and a centre, and replays cold.
#[test]
fn horizontal_bump_sections_partition_unit_caps() {
    let top = bump()
        .faces()
        .find(|face| face.surface().kind() == SurfaceKind::RationalBezier)
        .unwrap()
        .surface()
        .geometry()
        .clone();
    let model = builder::cuboid(pi(-1, -1, -2), Point3::new(1.into(), 1.into(), q(5, 4)))
        .unwrap()
        .model()
        .clone();
    let face = model
        .faces()
        .find(|face| matches!(face.surface().geometry().plane_origin(), Some(o) if o.z == q(5, 4)))
        .unwrap();
    let pieces = match face.surface().geometry().intersect_surface(&top).unwrap() {
        SurfaceSurfaceIntersection::Curves(curves) => curves,
        other => panic!("four corner arcs required, found {other:?}"),
    };
    assert_eq!(pieces.len(), 4);
    let half = q(1, 2);
    let expected = [
        (Real::one(), half.clone()),
        (Real::one(), -half.clone()),
        (-Real::one(), half.clone()),
        (-Real::one(), -half.clone()),
        (half.clone(), Real::one()),
        (half.clone(), -Real::one()),
        (-half.clone(), Real::one()),
        (-half.clone(), -Real::one()),
    ];
    let mut seen = [0usize; 8];
    for piece in &pieces {
        assert_eq!(piece.curve().kind(), Curve3Kind::PlanarImage);
        for end in [piece.curve().start().unwrap(), piece.curve().end().unwrap()] {
            let index = expected
                .iter()
                .position(|(x, y)| end == Point3::new(x.clone(), y.clone(), q(5, 4)))
                .expect("arc ends on the unit square's sides");
            seen[index] += 1;
        }
    }
    assert_eq!(seen, [1; 8]);
    let split = face
        .split_by_surface_curves(&pieces, SurfaceIntersectionOperand::First)
        .unwrap();
    assert_eq!(split.model().faces().count(), 10);
    let caps = split
        .model()
        .faces()
        .filter(
            |face| matches!(face.surface().geometry().plane_origin(), Some(o) if o.z == q(5, 4)),
        )
        .collect::<Vec<_>>();
    assert_eq!(caps.len(), 5);
    let mut edge_counts = caps
        .iter()
        .map(|face| face.outer().unwrap().edge_uses().count())
        .collect::<Vec<_>>();
    edge_counts.sort_unstable();
    // Each corner: one arc and two side pieces; the centre: four arcs and
    // four side pieces.
    assert_eq!(edge_counts, vec![3, 3, 3, 3, 8]);
    let json = split.model().to_json().unwrap();
    let cold = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(cold.to_json().unwrap(), json);
    assert_eq!(cold.faces().count(), 10);
}

/// Cutting the bump solid by the slab z <= 5/4 keeps the tensor top where
/// x² + y² <= 5/4 and replaces the four corners by planar caps at 5/4. The
/// result is a capped tensor-graph solid: points under, on and over both
/// tops classify exactly, including on the curved seam where both tops
/// meet; it replays cold and transforms, and its volume matches the closed
/// form 91/12 + 25 pi/32 - (25/8) asin(2 sqrt(5)/5).
#[test]
fn slab_cuts_through_tensor_tops_leave_capped_graph_solids() {
    let body = BodySet::from_solid(bump().solids().next().unwrap());
    let slab = builder::cuboid(pi(-2, -2, -2), Point3::new(2.into(), 2.into(), q(5, 4))).unwrap();
    let result = boolean::intersection(&body, &slab).unwrap();
    assert_eq!(result.len(), 1);
    let model = result.model().clone();
    // Base, four walls, the central tensor piece and four corner caps.
    assert_eq!(model.faces().count(), 10);
    assert_eq!(
        model
            .faces()
            .filter(|face| face.surface().kind() == SurfaceKind::RationalBezier)
            .count(),
        1
    );
    // (11/13)² + (19/26)² = 845/676 = 5/4: a rational seam point.
    let (sx, sy) = (q(11, 13), q(19, 26));
    let p = |x: Real, y: Real, z: Real| Point3::new(x, y, z);
    let n = |n: i64, d: i64| q(n, d);
    let cases = vec![
        // Under, on and over the tensor top x² + y² at the centre.
        (p(0.into(), 0.into(), n(-1, 2)), SolidPointLocation::Inside),
        (
            p(0.into(), 0.into(), 0.into()),
            SolidPointLocation::Boundary,
        ),
        (p(0.into(), 0.into(), n(1, 2)), SolidPointLocation::Outside),
        (p(n(1, 2), n(1, 2), n(1, 4)), SolidPointLocation::Inside),
        (p(n(1, 2), n(1, 2), n(1, 2)), SolidPointLocation::Boundary),
        // Under, on and over a corner cap, still below the removed tensor.
        (p(n(9, 10), n(9, 10), n(6, 5)), SolidPointLocation::Inside),
        (p(n(9, 10), n(9, 10), n(5, 4)), SolidPointLocation::Boundary),
        (
            p(n(9, 10), n(9, 10), n(13, 10)),
            SolidPointLocation::Outside,
        ),
        // On the curved seam where both tops have height 5/4.
        (
            p(sx.clone(), sy.clone(), 1.into()),
            SolidPointLocation::Inside,
        ),
        (
            p(sx.clone(), sy.clone(), n(5, 4)),
            SolidPointLocation::Boundary,
        ),
        (
            p(sx.clone(), sy.clone(), n(3, 2)),
            SolidPointLocation::Outside,
        ),
        // Walls and base.
        (
            p(1.into(), n(9, 10), 1.into()),
            SolidPointLocation::Boundary,
        ),
        (
            p(n(11, 10), 0.into(), 0.into()),
            SolidPointLocation::Outside,
        ),
        (
            p(0.into(), 0.into(), (-1).into()),
            SolidPointLocation::Boundary,
        ),
    ];
    let solid = model.solids().next().unwrap();
    for (point, expected) in &cases {
        assert_eq!(&solid.classify_point(point).unwrap(), expected, "{point:?}");
    }
    // The corner caps and the tensor centre are trimmed by circular arcs;
    // their exact areas and Green integrals give the closed form.
    assert_within(&solid.volume().unwrap(), &slab_cut_volume());
    let json = model.to_json().unwrap();
    let cold = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(cold.to_json().unwrap(), json);
    let cold_solid = cold.solids().next().unwrap();
    for (point, expected) in &cases {
        assert_eq!(&cold_solid.classify_point(point).unwrap(), expected);
    }
    // A translation carries the caps and the tensor together.
    let transform = Matrix4::from_row_major([
        1.into(),
        0.into(),
        0.into(),
        3.into(),
        0.into(),
        1.into(),
        0.into(),
        (-2).into(),
        0.into(),
        0.into(),
        1.into(),
        1.into(),
        0.into(),
        0.into(),
        0.into(),
        1.into(),
    ]);
    let mapped = model.transformed(&transform).unwrap();
    let mapped_solid = mapped.solids().next().unwrap();
    for (point, expected) in &cases {
        let image = Point3::new(
            &point.x + Real::from(3),
            &point.y - Real::from(2),
            &point.z + Real::one(),
        );
        assert_eq!(&mapped_solid.classify_point(&image).unwrap(), expected);
    }
}

/// A block [-1/2, 1/2]² × [-1/2, 1] pierces the bump top (height
/// x² + y² <= 1/2 over the block). The union gains a planar cap at z = 1 and
/// step walls; the difference gains a pocket floor at z = -1/2. Both are
/// capped tensor-graph solids with straight trims, so their volumes are
/// exact: the block's intersection with the bump is
/// integral (x² + y² + 1/2) = 1/6 + 1/2 = 2/3, the union is
/// 20/3 + 3/2 - 2/3 = 15/2 and the difference is 20/3 - 2/3 = 6.
#[test]
fn blocks_piercing_tensor_tops_union_and_pocket_exactly() {
    let body = BodySet::from_solid(bump().solids().next().unwrap());
    let block = builder::cuboid(
        Point3::new(q(-1, 2), q(-1, 2), q(-1, 2)),
        Point3::new(q(1, 2), q(1, 2), 1.into()),
    )
    .unwrap();
    let union = boolean::union(&body, &block).unwrap();
    let difference = boolean::difference(&body, &block).unwrap();
    let common = boolean::intersection(&body, &block).unwrap();
    assert_eq!(
        (
            union.model().faces().count(),
            difference.model().faces().count(),
            common.model().faces().count()
        ),
        (11, 11, 6)
    );
    assert_eq!(union.bodies().next().unwrap().volume().unwrap(), q(15, 2));
    assert_eq!(
        difference.bodies().next().unwrap().volume().unwrap(),
        Real::from(6)
    );
    assert_eq!(common.bodies().next().unwrap().volume().unwrap(), q(2, 3));
    // The same block authored at half size and scaled by 2 about the origin:
    // its cap's plane frame is no longer the base frame, so the cap's area is
    // carried by a Jacobian of 4 and the union volume is unchanged.
    let stretch = Matrix4::from_row_major([
        2.into(),
        0.into(),
        0.into(),
        0.into(),
        0.into(),
        2.into(),
        0.into(),
        0.into(),
        0.into(),
        0.into(),
        2.into(),
        0.into(),
        0.into(),
        0.into(),
        0.into(),
        1.into(),
    ]);
    let narrow = builder::cuboid(
        Point3::new(q(-1, 4), q(-1, 4), q(-1, 4)),
        Point3::new(q(1, 4), q(1, 4), q(1, 2)),
    )
    .unwrap()
    .model()
    .transformed(&stretch)
    .unwrap();
    let stretched = BodySet::from_solid(narrow.solids().next().unwrap());
    let stretched_union = boolean::union(&body, &stretched).unwrap();
    assert_eq!(
        stretched_union.bodies().next().unwrap().volume().unwrap(),
        q(15, 2)
    );
    let p = |x: Real, y: Real, z: Real| Point3::new(x, y, z);
    let n = |n: i64, d: i64| q(n, d);
    let union_cases = vec![
        (p(0.into(), 0.into(), n(9, 10)), SolidPointLocation::Inside),
        (
            p(0.into(), 0.into(), 1.into()),
            SolidPointLocation::Boundary,
        ),
        (
            p(0.into(), 0.into(), n(11, 10)),
            SolidPointLocation::Outside,
        ),
        (p(n(1, 2), 0.into(), n(4, 5)), SolidPointLocation::Boundary),
        (p(n(3, 5), 0.into(), n(3, 10)), SolidPointLocation::Inside),
        (p(n(3, 5), 0.into(), n(9, 25)), SolidPointLocation::Boundary),
        (p(n(3, 5), 0.into(), n(2, 5)), SolidPointLocation::Outside),
        (
            p(0.into(), 0.into(), (-1).into()),
            SolidPointLocation::Boundary,
        ),
    ];
    let difference_cases = vec![
        (p(0.into(), 0.into(), 0.into()), SolidPointLocation::Outside),
        (
            p(0.into(), 0.into(), n(-1, 2)),
            SolidPointLocation::Boundary,
        ),
        (p(0.into(), 0.into(), n(-3, 4)), SolidPointLocation::Inside),
        (p(n(1, 2), 0.into(), n(-1, 4)), SolidPointLocation::Boundary),
        (p(n(3, 4), 0.into(), n(1, 2)), SolidPointLocation::Inside),
        (p(n(3, 4), 0.into(), n(9, 16)), SolidPointLocation::Boundary),
    ];
    for (result, cases) in [(&union, &union_cases), (&difference, &difference_cases)] {
        let solid = result.bodies().next().unwrap();
        for (point, expected) in cases {
            assert_eq!(&solid.classify_point(point).unwrap(), expected, "{point:?}");
        }
        let json = result.model().to_json().unwrap();
        let cold = RawModel::from_json(&json, &ArchiveLimits::default())
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(cold.to_json().unwrap(), json);
        let cold_solid = cold.solids().next().unwrap();
        for (point, expected) in cases {
            assert_eq!(&cold_solid.classify_point(point).unwrap(), expected);
        }
        assert_eq!(cold_solid.volume().unwrap(), solid.volume().unwrap());
    }
    // The block authored at half width and stretched by 2 along x only: its
    // wall planes' frames are no longer similar to the patch chart, so the
    // section pcurves carry exact non-similarity affine maps. Every result
    // keeps its volume and classification.
    let x_stretch = Matrix4::from_row_major([
        2.into(),
        0.into(),
        0.into(),
        0.into(),
        0.into(),
        1.into(),
        0.into(),
        0.into(),
        0.into(),
        0.into(),
        1.into(),
        0.into(),
        0.into(),
        0.into(),
        0.into(),
        1.into(),
    ]);
    let slim = builder::cuboid(
        Point3::new(q(-1, 4), q(-1, 2), q(-1, 2)),
        Point3::new(q(1, 4), q(1, 2), 1.into()),
    )
    .unwrap()
    .model()
    .transformed(&x_stretch)
    .unwrap();
    let slim = BodySet::from_solid(slim.solids().next().unwrap());
    let slim_union = boolean::union(&body, &slim).unwrap();
    for (result, volume) in [
        (&slim_union, q(15, 2)),
        (&boolean::difference(&body, &slim).unwrap(), Real::from(6)),
        (&boolean::intersection(&body, &slim).unwrap(), q(2, 3)),
    ] {
        assert_eq!(result.bodies().next().unwrap().volume().unwrap(), volume);
    }
    let slim_solid = slim_union.bodies().next().unwrap();
    for (point, expected) in &union_cases {
        assert_eq!(&slim_solid.classify_point(point).unwrap(), expected);
    }
    // Mirroring the stretch (x -> -2x) makes those maps reverse orientation.
    let mirror = Matrix4::from_row_major([
        (-2).into(),
        0.into(),
        0.into(),
        0.into(),
        0.into(),
        1.into(),
        0.into(),
        0.into(),
        0.into(),
        0.into(),
        1.into(),
        0.into(),
        0.into(),
        0.into(),
        0.into(),
        1.into(),
    ]);
    let mirrored = builder::cuboid(
        Point3::new(q(-1, 4), q(-1, 2), q(-1, 2)),
        Point3::new(q(1, 4), q(1, 2), 1.into()),
    )
    .unwrap()
    .model()
    .transformed(&mirror)
    .unwrap();
    let mirrored = BodySet::from_solid(mirrored.solids().next().unwrap());
    let mirrored_union = boolean::union(&body, &mirrored).unwrap();
    assert_eq!(
        mirrored_union.bodies().next().unwrap().volume().unwrap(),
        q(15, 2)
    );
    assert_eq!(
        boolean::difference(&body, &mirrored)
            .unwrap()
            .bodies()
            .next()
            .unwrap()
            .volume()
            .unwrap(),
        Real::from(6)
    );
    // A scaled rotation about z with a translation; the linear part has
    // determinant (36/25 + 64/25) * 2 = 8, which scales the cap terms too.
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
    let mapped = union.model().transformed(&transform).unwrap();
    let mapped_solid = mapped.solids().next().unwrap();
    assert_eq!(mapped_solid.volume().unwrap(), Real::from(60));
    for (point, expected) in &union_cases {
        let image = Point3::new(
            q(6, 5) * &point.x - q(8, 5) * &point.y + Real::one(),
            q(8, 5) * &point.x + q(6, 5) * &point.y + Real::from(2),
            Real::from(2) * &point.z + Real::from(3),
        );
        assert_eq!(&mapped_solid.classify_point(&image).unwrap(), expected);
    }
}

/// A block [-1,1]² × [-1,3] shares the bump's base and all four walls and
/// contains it. The common walls are coincident faces whose curved Bézier
/// edges both operands trace, and the walls cut the tensor top exactly
/// along its own boundary: the intersection is the bump again (six faces,
/// volume 20/3, same classification), the difference is empty and the union
/// is the block (volume 16).
#[test]
fn shared_wall_blocks_containing_tensor_graph_solids_intersect_exactly() {
    let body = BodySet::from_solid(bump().solids().next().unwrap());
    let block = builder::cuboid(pi(-1, -1, -1), Point3::new(1.into(), 1.into(), 3.into())).unwrap();
    let common = boolean::intersection(&body, &block).unwrap();
    assert_eq!(common.len(), 1);
    assert_eq!(common.model().faces().count(), 6);
    let solid = common.bodies().next().unwrap();
    assert_eq!(solid.volume().unwrap(), q(20, 3));
    for (point, expected) in expected_locations() {
        assert_eq!(solid.classify_point(&point).unwrap(), expected, "{point:?}");
    }
    let json = common.model().to_json().unwrap();
    let cold = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(cold.to_json().unwrap(), json);
    assert!(boolean::difference(&body, &block).unwrap().is_empty());
    // The union is the block: each shared wall keeps its two coplanar
    // pieces, stitched along the bump's own Bézier edge.
    let union = boolean::union(&body, &block).unwrap();
    assert_eq!(union.len(), 1);
    let solid = union.bodies().next().unwrap();
    assert_eq!(solid.volume().unwrap(), Real::from(16));
    for (point, expected) in [
        (pi(0, 0, 2), SolidPointLocation::Inside),
        (pi(0, 0, 3), SolidPointLocation::Boundary),
        (pi(1, 0, 2), SolidPointLocation::Boundary),
        (pi(1, 0, 0), SolidPointLocation::Boundary),
        (pi(0, 0, 4), SolidPointLocation::Outside),
        (pi(2, 0, 0), SolidPointLocation::Outside),
    ] {
        assert_eq!(solid.classify_point(&point).unwrap(), expected, "{point:?}");
    }
    let json = union.model().to_json().unwrap();
    let cold = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(cold.to_json().unwrap(), json);
}

/// A block [-1,1]² × [-1,5/4] shares the bump's base and walls and cuts the
/// top in the four corner arcs of x² + y² = 5/4, which end on the shared
/// walls at selected points. The intersection keeps the tensor centre and
/// four corner caps (min(x² + y², 5/4)); the union keeps a central disk cap
/// and the tensor corners (max(x² + y², 5/4)). Both classify exactly, also
/// on the rational seam point (11/13, 19/26), and replay cold.
#[test]
fn shared_wall_blocks_cut_tensor_tops_into_capped_solids() {
    let body = BodySet::from_solid(bump().solids().next().unwrap());
    let block = builder::cuboid(pi(-1, -1, -1), Point3::new(1.into(), 1.into(), q(5, 4))).unwrap();
    let common = boolean::intersection(&body, &block).unwrap();
    let union = boolean::union(&body, &block).unwrap();
    assert_eq!((common.len(), union.len()), (1, 1));
    assert_eq!(common.model().faces().count(), 10);
    let (sx, sy) = (q(11, 13), q(19, 26));
    let p = |x: Real, y: Real, z: Real| Point3::new(x, y, z);
    let n = |n: i64, d: i64| q(n, d);
    let common_cases = vec![
        (p(0.into(), 0.into(), n(-1, 2)), SolidPointLocation::Inside),
        (
            p(0.into(), 0.into(), 0.into()),
            SolidPointLocation::Boundary,
        ),
        (p(0.into(), 0.into(), n(1, 2)), SolidPointLocation::Outside),
        (p(n(9, 10), n(9, 10), n(6, 5)), SolidPointLocation::Inside),
        (p(n(9, 10), n(9, 10), n(5, 4)), SolidPointLocation::Boundary),
        (
            p(n(9, 10), n(9, 10), n(13, 10)),
            SolidPointLocation::Outside,
        ),
        (
            p(sx.clone(), sy.clone(), 1.into()),
            SolidPointLocation::Inside,
        ),
        (
            p(sx.clone(), sy.clone(), n(5, 4)),
            SolidPointLocation::Boundary,
        ),
        (
            p(sx.clone(), sy.clone(), n(3, 2)),
            SolidPointLocation::Outside,
        ),
    ];
    let union_cases = vec![
        (p(0.into(), 0.into(), n(6, 5)), SolidPointLocation::Inside),
        (p(0.into(), 0.into(), n(5, 4)), SolidPointLocation::Boundary),
        (
            p(0.into(), 0.into(), n(13, 10)),
            SolidPointLocation::Outside,
        ),
        (p(n(9, 10), n(9, 10), n(3, 2)), SolidPointLocation::Inside),
        (
            p(n(9, 10), n(9, 10), n(81, 50)),
            SolidPointLocation::Boundary,
        ),
        (
            p(n(9, 10), n(9, 10), n(17, 10)),
            SolidPointLocation::Outside,
        ),
        (
            p(sx.clone(), sy.clone(), 1.into()),
            SolidPointLocation::Inside,
        ),
        (
            p(sx.clone(), sy.clone(), n(5, 4)),
            SolidPointLocation::Boundary,
        ),
        (
            p(sx.clone(), sy.clone(), n(3, 2)),
            SolidPointLocation::Outside,
        ),
    ];
    // min and max of x² + y² and 5/4 sum to x² + y² + 5/4, so the volumes
    // sum to 8/3 + 5 + 8 = 47/3; the intersection is the slab-cut closed form.
    let common_volume = common.bodies().next().unwrap().volume().unwrap();
    let union_volume = union.bodies().next().unwrap().volume().unwrap();
    assert_within(&common_volume, &slab_cut_volume());
    assert_within(&(common_volume + union_volume), &q(47, 3));
    for (result, cases) in [(&common, &common_cases), (&union, &union_cases)] {
        let solid = result.bodies().next().unwrap();
        for (point, expected) in cases {
            assert_eq!(&solid.classify_point(point).unwrap(), expected, "{point:?}");
        }
        let json = result.model().to_json().unwrap();
        let cold = RawModel::from_json(&json, &ArchiveLimits::default())
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(cold.to_json().unwrap(), json);
        let cold_solid = cold.solids().next().unwrap();
        for (point, expected) in cases {
            assert_eq!(&cold_solid.classify_point(point).unwrap(), expected);
        }
    }
}

/// A block [-1,1]² × [-1,1/2] shares the bump's base and walls; its top cuts
/// the tensor in the closed circle x² + y² = 1/2 inside the square. The
/// intersection (min(x² + y², 1/2)) keeps a tensor disk and a cap with a
/// circular hole; the union (max(x² + y², 1/2)) keeps a disk cap inside a
/// holed tensor face. Both classify exactly, also on the rational seam point
/// (1/2, 1/2), and replay cold.
#[test]
fn shared_wall_blocks_cut_closed_sections_into_holed_caps() {
    let body = BodySet::from_solid(bump().solids().next().unwrap());
    let block = builder::cuboid(pi(-1, -1, -1), Point3::new(1.into(), 1.into(), q(1, 2))).unwrap();
    let common = boolean::intersection(&body, &block).unwrap();
    let union = boolean::union(&body, &block).unwrap();
    assert_eq!((common.len(), union.len()), (1, 1));
    assert_eq!(common.model().faces().count(), 7);
    assert_eq!(union.model().faces().count(), 11);
    let p = |x: Real, y: Real, z: Real| Point3::new(x, y, z);
    let n = |n: i64, d: i64| q(n, d);
    let half = n(1, 2);
    let common_cases = vec![
        (p(0.into(), 0.into(), n(-1, 2)), SolidPointLocation::Inside),
        (
            p(0.into(), 0.into(), 0.into()),
            SolidPointLocation::Boundary,
        ),
        (p(0.into(), 0.into(), n(1, 10)), SolidPointLocation::Outside),
        (p(n(9, 10), 0.into(), n(2, 5)), SolidPointLocation::Inside),
        (
            p(n(9, 10), 0.into(), half.clone()),
            SolidPointLocation::Boundary,
        ),
        (p(n(9, 10), 0.into(), n(3, 5)), SolidPointLocation::Outside),
        (
            p(half.clone(), half.clone(), n(2, 5)),
            SolidPointLocation::Inside,
        ),
        (
            p(half.clone(), half.clone(), half.clone()),
            SolidPointLocation::Boundary,
        ),
        (
            p(half.clone(), half.clone(), n(3, 5)),
            SolidPointLocation::Outside,
        ),
    ];
    let union_cases = vec![
        (p(0.into(), 0.into(), n(2, 5)), SolidPointLocation::Inside),
        (
            p(0.into(), 0.into(), half.clone()),
            SolidPointLocation::Boundary,
        ),
        (p(0.into(), 0.into(), n(3, 5)), SolidPointLocation::Outside),
        (p(n(9, 10), 0.into(), n(7, 10)), SolidPointLocation::Inside),
        (
            p(n(9, 10), 0.into(), n(81, 100)),
            SolidPointLocation::Boundary,
        ),
        (p(n(9, 10), 0.into(), n(9, 10)), SolidPointLocation::Outside),
        (
            p(half.clone(), half.clone(), n(2, 5)),
            SolidPointLocation::Inside,
        ),
        (
            p(half.clone(), half.clone(), half.clone()),
            SolidPointLocation::Boundary,
        ),
        (
            p(half.clone(), half.clone(), n(3, 5)),
            SolidPointLocation::Outside,
        ),
    ];
    // The disk x² + y² <= 1/2 lies inside the square: the union adds
    // integral over the disk of (1/2 - x² - y²) = pi/4 - pi/8 to 20/3, and
    // the intersection is 38/3 minus that.
    let eighth_pi = (Real::pi() / Real::from(8)).unwrap();
    assert_within(
        &union.bodies().next().unwrap().volume().unwrap(),
        &(q(20, 3) + eighth_pi.clone()),
    );
    assert_within(
        &common.bodies().next().unwrap().volume().unwrap(),
        &(Real::from(6) - eighth_pi),
    );
    for (result, cases) in [(&common, &common_cases), (&union, &union_cases)] {
        let solid = result.bodies().next().unwrap();
        for (point, expected) in cases {
            assert_eq!(&solid.classify_point(point).unwrap(), expected, "{point:?}");
        }
        let json = result.model().to_json().unwrap();
        let cold = RawModel::from_json(&json, &ArchiveLimits::default())
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(cold.to_json().unwrap(), json);
        let cold_solid = cold.solids().next().unwrap();
        for (point, expected) in cases {
            assert_eq!(&cold_solid.classify_point(point).unwrap(), expected);
        }
    }
}

/// The bump stretched by 2 along x has top z = x²/4 + y²; the slab z <= 5/4
/// cuts it in elliptical corner arcs whose plane pcurves carry exact
/// non-similarity affine maps. Each elliptical cap is an affine image of a
/// circular region in the section's source chart, so the volume is exactly
/// twice the unstretched closed form, and stretched probes classify alike.
#[test]
fn stretched_slab_cuts_measure_elliptical_caps_exactly() {
    let stretch = Matrix4::from_row_major([
        2.into(),
        0.into(),
        0.into(),
        0.into(),
        0.into(),
        1.into(),
        0.into(),
        0.into(),
        0.into(),
        0.into(),
        1.into(),
        0.into(),
        0.into(),
        0.into(),
        0.into(),
        1.into(),
    ]);
    let wide = bump().transformed(&stretch).unwrap();
    let body = BodySet::from_solid(wide.solids().next().unwrap());
    let slab = builder::cuboid(pi(-3, -3, -2), Point3::new(3.into(), 3.into(), q(5, 4))).unwrap();
    let result = boolean::intersection(&body, &slab).unwrap();
    assert_eq!((result.len(), result.model().faces().count()), (1, 10));
    let solid = result.bodies().next().unwrap();
    assert_within(
        &solid.volume().unwrap(),
        &(Real::from(2) * slab_cut_volume()),
    );
    let p = |x: Real, y: Real, z: Real| Point3::new(x * Real::from(2), y, z);
    let n = |n: i64, d: i64| q(n, d);
    for (point, expected) in [
        (p(0.into(), 0.into(), n(-1, 2)), SolidPointLocation::Inside),
        (
            p(0.into(), 0.into(), 0.into()),
            SolidPointLocation::Boundary,
        ),
        (p(0.into(), 0.into(), n(1, 2)), SolidPointLocation::Outside),
        (p(n(9, 10), n(9, 10), n(6, 5)), SolidPointLocation::Inside),
        (p(n(9, 10), n(9, 10), n(5, 4)), SolidPointLocation::Boundary),
        (
            p(n(9, 10), n(9, 10), n(13, 10)),
            SolidPointLocation::Outside,
        ),
    ] {
        assert_eq!(solid.classify_point(&point).unwrap(), expected, "{point:?}");
    }
    let json = result.model().to_json().unwrap();
    let cold = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(cold.to_json().unwrap(), json);
}

/// Removing the shared-wall block [-1,1]² × [-1,5/4] from the bump leaves
/// four corner solids over the planar base z = 5/4, each bounded by a
/// corner arc where the tensor meets that base. The tensor dips below the
/// base elsewhere on its patch, but it is certified above the base over each
/// corner's own profile: every plane/tensor section lies on that profile's
/// boundary or wholly outside it, and an interior point fixes the sign. Each
/// corner has volume (20/3 - slab cut volume) / 4.
#[test]
fn shared_wall_differences_leave_corner_graph_solids() {
    let body = BodySet::from_solid(bump().solids().next().unwrap());
    let block = builder::cuboid(pi(-1, -1, -1), Point3::new(1.into(), 1.into(), q(5, 4))).unwrap();
    let corners = boolean::difference(&body, &block).unwrap();
    assert_eq!(corners.len(), 4);
    let each = ((q(20, 3) - slab_cut_volume()) / Real::from(4)).unwrap();
    for solid in corners.bodies() {
        assert_within(&solid.volume().unwrap(), &each);
    }
    let locate = |model: &Model, point: &Point3| {
        let inside = model
            .solids()
            .filter(|solid| solid.classify_point(point).unwrap() == SolidPointLocation::Inside)
            .count();
        let boundary = model
            .solids()
            .filter(|solid| solid.classify_point(point).unwrap() == SolidPointLocation::Boundary)
            .count();
        (inside, boundary)
    };
    let n = |n: i64, d: i64| q(n, d);
    for (point, expected) in [
        (Point3::new(n(9, 10), n(9, 10), n(3, 2)), (1, 0)),
        (Point3::new(n(-9, 10), n(9, 10), n(3, 2)), (1, 0)),
        (Point3::new(n(9, 10), n(9, 10), n(5, 4)), (0, 1)),
        (Point3::new(n(9, 10), n(9, 10), n(81, 50)), (0, 1)),
        (Point3::new(n(9, 10), n(9, 10), n(6, 5)), (0, 0)),
        (Point3::new(n(9, 10), n(9, 10), n(17, 10)), (0, 0)),
        (Point3::new(0.into(), 0.into(), 1.into()), (0, 0)),
    ] {
        assert_eq!(locate(corners.model(), &point), expected, "{point:?}");
    }
    let json = corners.model().to_json().unwrap();
    let cold = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(cold.to_json().unwrap(), json);
    assert_eq!(
        locate(&cold, &Point3::new(n(9, 10), n(9, 10), n(3, 2))),
        (1, 0)
    );
}

/// Removing the shared-wall block [-1,1]² × [-1,1/2] from the bump leaves a
/// ring over the base z = 1/2 whose base face has a circular hole, bounded
/// by the closed section where the tensor meets that base. The base region
/// keeps its hole, the tensor is certified above the base over that holed
/// profile, and the volume is 20/3 - (6 - pi/8) = 2/3 + pi/8.
#[test]
fn shared_wall_differences_leave_holed_ring_graph_solids() {
    let body = BodySet::from_solid(bump().solids().next().unwrap());
    let block = builder::cuboid(pi(-1, -1, -1), Point3::new(1.into(), 1.into(), q(1, 2))).unwrap();
    let ring = boolean::difference(&body, &block).unwrap();
    assert_eq!((ring.len(), ring.model().faces().count()), (1, 6));
    let solid = ring.bodies().next().unwrap();
    assert_within(
        &solid.volume().unwrap(),
        &(q(2, 3) + (Real::pi() / Real::from(8)).unwrap()),
    );
    let p = |x: Real, y: Real, z: Real| Point3::new(x, y, z);
    let n = |n: i64, d: i64| q(n, d);
    let half = n(1, 2);
    let cases = vec![
        (p(n(9, 10), 0.into(), n(7, 10)), SolidPointLocation::Inside),
        (
            p(n(9, 10), 0.into(), half.clone()),
            SolidPointLocation::Boundary,
        ),
        (
            p(n(9, 10), 0.into(), n(81, 100)),
            SolidPointLocation::Boundary,
        ),
        (p(n(9, 10), 0.into(), n(9, 10)), SolidPointLocation::Outside),
        (p(n(9, 10), 0.into(), n(2, 5)), SolidPointLocation::Outside),
        (
            p(0.into(), 0.into(), half.clone()),
            SolidPointLocation::Outside,
        ),
        (p(0.into(), 0.into(), n(3, 5)), SolidPointLocation::Outside),
        (
            p(half.clone(), half.clone(), half.clone()),
            SolidPointLocation::Boundary,
        ),
    ];
    for (point, expected) in &cases {
        assert_eq!(&solid.classify_point(point).unwrap(), expected, "{point:?}");
    }
    let json = ring.model().to_json().unwrap();
    let cold = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(cold.to_json().unwrap(), json);
    let cold_solid = cold.solids().next().unwrap();
    for (point, expected) in &cases {
        assert_eq!(&cold_solid.classify_point(point).unwrap(), expected);
    }
}

/// A Boolean result reused as the next operand: the holed ring left by the
/// z <= 1/2 shared-wall difference, cut by the slab z <= 5/4, keeps its holed
/// base, gains four corner caps, and its tensor still dips below the base
/// inside the hole. The profile-only certificate admits it with caps, and the
/// volume is the integral of min(x² + y², 5/4) - 1/2 outside the disk,
/// slab cut volume - 6 + pi/8.
#[test]
fn capped_rings_chain_profile_certified_tensor_graphs() {
    let body = BodySet::from_solid(bump().solids().next().unwrap());
    let block = builder::cuboid(pi(-1, -1, -1), Point3::new(1.into(), 1.into(), q(1, 2))).unwrap();
    let ring = boolean::difference(&body, &block).unwrap();
    let slab = builder::cuboid(pi(-2, -2, -2), Point3::new(2.into(), 2.into(), q(5, 4))).unwrap();
    let capped = boolean::intersection(&ring, &slab).unwrap();
    assert_eq!((capped.len(), capped.model().faces().count()), (1, 10));
    let solid = capped.bodies().next().unwrap();
    assert_within(
        &solid.volume().unwrap(),
        &(slab_cut_volume() - Real::from(6) + (Real::pi() / Real::from(8)).unwrap()),
    );
    let p = |x: Real, y: Real, z: Real| Point3::new(x, y, z);
    let n = |n: i64, d: i64| q(n, d);
    for (point, expected) in [
        (p(n(9, 10), 0.into(), n(7, 10)), SolidPointLocation::Inside),
        (p(n(9, 10), n(9, 10), n(6, 5)), SolidPointLocation::Inside),
        (p(n(9, 10), n(9, 10), n(5, 4)), SolidPointLocation::Boundary),
        (
            p(n(9, 10), n(9, 10), n(13, 10)),
            SolidPointLocation::Outside,
        ),
        (p(n(9, 10), 0.into(), n(2, 5)), SolidPointLocation::Outside),
        (p(0.into(), 0.into(), n(3, 5)), SolidPointLocation::Outside),
    ] {
        assert_eq!(solid.classify_point(&point).unwrap(), expected, "{point:?}");
    }
    let json = capped.model().to_json().unwrap();
    let cold = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(cold.to_json().unwrap(), json);
}

/// The slab z <= 1 cuts the ramp z = x² + y + 1 along the parabola
/// y = -x², a polynomial graph rather than a conic: the tensor keeps the
/// part below it and a planar cap covers the part above. Polynomial-graph
/// trims integrate exactly, giving the rational volume 112/15.
#[test]
fn slab_cuts_along_polynomial_graph_trims_measure_exactly() {
    let body = BodySet::from_solid(ramp().solids().next().unwrap());
    assert_eq!(body.bodies().next().unwrap().volume().unwrap(), q(28, 3));
    let slab = builder::cuboid(pi(-2, -2, -2), Point3::new(2.into(), 2.into(), 1.into())).unwrap();
    let cut = boolean::intersection(&body, &slab).unwrap();
    assert_eq!(cut.len(), 1);
    let solid = cut.bodies().next().unwrap();
    assert_eq!(solid.volume().unwrap(), q(112, 15));
    let p = |x: Real, y: Real, z: Real| Point3::new(x, y, z);
    let n = |n: i64, d: i64| q(n, d);
    for (point, expected) in [
        (p(0.into(), n(-1, 2), n(1, 4)), SolidPointLocation::Inside),
        (p(0.into(), n(-1, 2), n(1, 2)), SolidPointLocation::Boundary),
        (p(0.into(), n(-1, 2), n(3, 4)), SolidPointLocation::Outside),
        (p(0.into(), n(1, 2), n(9, 10)), SolidPointLocation::Inside),
        (p(0.into(), n(1, 2), 1.into()), SolidPointLocation::Boundary),
        (p(0.into(), n(1, 2), n(11, 10)), SolidPointLocation::Outside),
    ] {
        assert_eq!(solid.classify_point(&point).unwrap(), expected, "{point:?}");
    }
    let json = cut.model().to_json().unwrap();
    let cold = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(cold.to_json().unwrap(), json);
    assert_eq!(cold.solids().next().unwrap().volume().unwrap(), q(112, 15));
}

/// Flat rational Bézier faces with curved (non-affine) parameterizations
/// measure exactly: the Green integral of the plane Jacobian over the trims,
/// carried by the plane frame's area element. A non-uniform square has area
/// 4; the same patch tilted into z = x has area 4 sqrt(2); bulging the
/// bottom edge's middle control to (1/2, -3/2) gives 13/3 (derived with
/// SymPy; the Jacobian keeps one sign).
#[test]
fn flat_tensor_faces_with_curved_parameterizations_measure_exactly() {
    let patch = |lift: &dyn Fn(Real, Real) -> Point3, bulge: bool| {
        let xs = [Real::from(-1), q(1, 2), Real::one()];
        let ys = [Real::from(-1), Real::zero(), Real::one()];
        let controls = (0..3)
            .map(|i| {
                (0..3)
                    .map(|j| {
                        if bulge && i == 0 && j == 1 {
                            lift(q(1, 2), q(-3, 2))
                        } else {
                            lift(xs[j].clone(), ys[i].clone())
                        }
                    })
                    .collect()
            })
            .collect();
        let sheet =
            builder::rational_bezier_patch(controls, vec![vec![Real::one(); 3]; 3]).unwrap();
        sheet.model().faces().next().unwrap().area().unwrap()
    };
    let flat = |x: Real, y: Real| Point3::new(x, y, Real::zero());
    let tilted = |x: Real, y: Real| Point3::new(x.clone(), y, x);
    assert_eq!(patch(&flat, false), Real::from(4));
    assert_eq!(
        patch(&tilted, false),
        Real::from(4) * Real::from(2).sqrt().unwrap()
    );
    assert_eq!(patch(&flat, true), q(13, 3));
    // A folded flat patch (x controls -1, 3, 1 turn back at t = 2/3): its
    // Jacobian changes sign, so |integral J| is not its area and the
    // measurement stays typed.
    let controls = (0..3)
        .map(|i| {
            [-1, 3, 1]
                .into_iter()
                .map(|x| pi(x, i - 1, 0))
                .collect::<Vec<_>>()
        })
        .collect::<Vec<_>>();
    if let Ok(sheet) = builder::rational_bezier_patch(controls, vec![vec![Real::one(); 3]; 3]) {
        assert!(matches!(
            sheet.model().faces().next().unwrap().area(),
            Err(QueryError::Geometry(GeometryError::UnsupportedMeasurement))
        ));
    } else {
        panic!("the folded patch is a valid sheet to measure");
    }
}

/// Nonuniform x(u)=-1+3u-u², y(v)=2v-1 and z=(2u-1)²+y².
/// The horizontal section is a circle in UV and a nonlinear rational image
/// in the cap chart; the generated cap must keep that exact image on replay.
#[test]
fn nonlinear_tensor_cap_sections_replay_exactly() {
    let cases = [
        (
            Point3::new(q(1, 4), 0.into(), q(-1, 2)),
            SolidPointLocation::Inside,
        ),
        (
            Point3::new(q(1, 4), 0.into(), 0.into()),
            SolidPointLocation::Boundary,
        ),
        (
            Point3::new(q(1, 4), 0.into(), q(1, 4)),
            SolidPointLocation::Outside,
        ),
        (
            Point3::new(q(-71, 100), 0.into(), q(1, 4)),
            SolidPointLocation::Inside,
        ),
        (
            Point3::new(q(-71, 100), 0.into(), q(1, 2)),
            SolidPointLocation::Boundary,
        ),
        (
            Point3::new(q(-71, 100), 0.into(), q(3, 4)),
            SolidPointLocation::Outside,
        ),
        (
            Point3::new(q(-5, 16), q(1, 2), q(1, 2)),
            SolidPointLocation::Boundary,
        ),
    ];
    if let Some(path) = std::env::var_os("HYPERBREP_NONLINEAR_CAP_REPLAY") {
        let json = std::fs::read_to_string(path).unwrap();
        let model = RawModel::from_json(&json, &ArchiveLimits::default())
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(model.to_json().unwrap(), json);
        let solid = model.solids().next().unwrap();
        for (point, expected) in &cases {
            assert_eq!(&solid.classify_point(point).unwrap(), expected);
        }
        return;
    }
    let model = graph_fixture_over(
        [Real::from(-1), q(1, 2), Real::one()],
        [[2, 0, 2], [0, -2, 0], [2, 0, 2]],
    );
    let body = BodySet::from_solid(model.solids().next().unwrap());
    assert_eq!(body.bodies().next().unwrap().volume().unwrap(), q(20, 3));
    let slab = builder::cuboid(pi(-2, -2, -2), Point3::new(2.into(), 2.into(), q(1, 2))).unwrap();
    let graph =
        boolean::intersection_graph(body.bodies().next().unwrap(), slab.bodies().next().unwrap())
            .unwrap();
    assert_eq!(graph.unresolved_trim_pairs(), 0);
    assert_eq!(graph.unsupported_pairs(), 0);
    let capped = boolean::intersection(&body, &slab).unwrap();
    assert_eq!(capped.len(), 1);
    assert_eq!(capped.model().faces().count(), 7);

    let json = capped.model().to_json().unwrap();
    let cold = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(cold.to_json().unwrap(), json);
    for model in [capped.model(), &cold] {
        let solid = model.solids().next().unwrap();
        for (point, expected) in &cases {
            assert_eq!(&solid.classify_point(point).unwrap(), expected, "{point:?}");
        }
    }
    let nonce = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap()
        .as_nanos();
    let path = std::env::temp_dir().join(format!(
        "hyperbrep-nonlinear-cap-{}-{nonce}.json",
        std::process::id()
    ));
    std::fs::write(&path, &json).unwrap();
    let output = std::process::Command::new(std::env::current_exe().unwrap())
        .args(["--exact", "nonlinear_tensor_cap_sections_replay_exactly"])
        .env("HYPERBREP_NONLINEAR_CAP_REPLAY", &path)
        .output()
        .unwrap();
    std::fs::remove_file(path).unwrap();
    assert!(
        output.status.success(),
        "{}",
        String::from_utf8_lossy(&output.stderr)
    );
}

/// Full next-operation target retained alongside the admitted cap/replay case.
#[test]
fn nonlinear_tensor_caps_support_transverse_recuts() {
    fn verify(cut: &BodySet) {
        assert_eq!(cut.len(), 1);
        let solid = cut.bodies().next().unwrap();
        use SolidPointLocation::{Boundary, Inside, Outside};
        // At u=1/4, x=-5/16 and the uncut roof is 1/4+y^2.
        // At u=1/3, x=-1/9 and the uncut roof is 1/9+y^2.
        // Material also obeys -1<=x<=0, |y|<=1 and -1<=z<=1/2.
        let samples = [
            (Point3::new(q(-5, 16), 0.into(), 0.into()), Inside),
            (Point3::new(q(-5, 16), 0.into(), q(3, 8)), Outside),
            (Point3::new(q(-5, 16), q(3, 4), q(3, 8)), Inside),
            (Point3::new(q(-5, 16), q(3, 4), q(5, 8)), Outside),
            (Point3::new(q(-5, 16), q(1, 2), q(1, 2)), Boundary),
            (Point3::new(q(-1, 9), 0.into(), q(1, 18)), Inside),
            (Point3::new(q(-1, 9), 0.into(), q(1, 6)), Outside),
            (Point3::new(q(1, 4), 0.into(), q(-1, 2)), Outside),
            (Point3::new(0.into(), 0.into(), q(-1, 2)), Boundary),
            (Point3::new(0.into(), 0.into(), q(1, 4)), Outside),
            (Point3::new(q(-5, 16), q(3, 4), (-1).into()), Boundary),
            (Point3::new(q(-5, 16), 0.into(), q(-9, 8)), Outside),
            (Point3::new(q(-5, 16), q(9, 8), 0.into()), Outside),
        ];
        for (point, expected) in samples {
            assert_eq!(
                solid.classify_point(&point).unwrap(),
                expected,
                "independent material witness {point:?}"
            );
        }
        // These are exact offsets from a known roof value, not a tolerance.
        let epsilon = (Real::one() / Real::from(1_u64 << 60)).unwrap();
        for (z, expected) in [(q(1, 4) - &epsilon, Inside), (q(1, 4) + &epsilon, Outside)] {
            let point = Point3::new(q(-5, 16), 0.into(), z);
            assert_eq!(solid.classify_point(&point).unwrap(), expected);
        }
        for edge in cut.model().edges() {
            assert_eq!(edge.uses().count(), 2);
        }
        assert!(
            cut.model()
                .vertices()
                .any(|vertex| vertex.point().try_coordinates().is_none())
        );
    }
    fn continue_after_replay(cut: &BodySet) {
        let slab =
            builder::cuboid(pi(-2, -2, -2), Point3::new(2.into(), 2.into(), q(-1, 2))).unwrap();
        let lower = boolean::intersection(cut, &slab).unwrap();
        assert_eq!(lower.len(), 1);
        assert_eq!(
            lower.bodies().next().unwrap().volume().unwrap(),
            Real::one()
        );
        let expected =
            builder::cuboid(pi(-1, -1, -1), Point3::new(0.into(), 1.into(), q(-1, 2))).unwrap();
        assert!(boolean::difference(&lower, &expected).unwrap().is_empty());
        assert!(boolean::difference(&expected, &lower).unwrap().is_empty());
        let text = lower.model().to_json().unwrap();
        RawModel::from_json(&text, &ArchiveLimits::default())
            .unwrap()
            .validate()
            .unwrap();
        let empty = boolean::difference(&lower, &lower).unwrap();
        assert!(empty.is_empty());
        assert_eq!(boolean::union(&empty, &lower).unwrap().len(), 1);
    }
    const INPUT: &str = "HYPERBREP_NONLINEAR_RECUT_REPLAY";
    if let Some(path) = std::env::var_os(INPUT) {
        let text = std::fs::read_to_string(path).unwrap();
        let model = RawModel::from_json(&text, &ArchiveLimits::default())
            .unwrap()
            .validate()
            .unwrap();
        assert_eq!(model.to_json().unwrap(), text);
        let cut = BodySet::from_solid(model.solids().next().unwrap());
        verify(&cut);
        continue_after_replay(&cut);
        return;
    }
    let model = graph_fixture_over(
        [Real::from(-1), q(1, 2), Real::one()],
        [[2, 0, 2], [0, -2, 0], [2, 0, 2]],
    );
    let body = BodySet::from_solid(model.solids().next().unwrap());
    let slab = builder::cuboid(pi(-2, -2, -2), Point3::new(2.into(), 2.into(), q(1, 2))).unwrap();
    let capped = boolean::intersection(&body, &slab).unwrap();
    let half = builder::cuboid(pi(-2, -2, -2), pi(0, 2, 3)).unwrap();
    let graph = boolean::intersection_graph(
        capped.bodies().next().unwrap(),
        half.bodies().next().unwrap(),
    )
    .unwrap();
    assert_eq!(graph.unresolved_trim_pairs(), 0);
    assert_eq!(graph.unsupported_pairs(), 0);
    for pair in graph.intersections() {
        let detail = match pair.relation() {
            boolean::FacePairRelation::Exact(SurfaceSurfaceIntersection::Curve(curve)) => format!(
                "{:?}, pcurves available: {}/{}",
                curve.curve().kind(),
                curve.first_pcurve().materialize().is_ok(),
                curve.second_pcurve().materialize().is_ok()
            ),
            relation => format!("{relation:?}"),
        };
        assert!(
            !matches!(pair.trim(), boolean::FacePairTrim::NotAvailable),
            "missing trim: faces {}/{}; {detail}",
            pair.first_face().index(),
            pair.second_face().index()
        );
    }
    let cut = boolean::intersection(&capped, &half).unwrap();
    assert_eq!(cut.len(), 1);
    verify(&cut);
    let text = cut.model().to_json().unwrap();
    let model = RawModel::from_json(&text, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(model.to_json().unwrap(), text);
    let cold = BodySet::from_solid(model.solids().next().unwrap());
    verify(&cold);
    let path = std::env::temp_dir().join(format!(
        "hyperbrep-nonlinear-recut-{}.json",
        std::process::id()
    ));
    std::fs::write(&path, &text).unwrap();
    let child = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "nonlinear_tensor_caps_support_transverse_recuts",
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
fn nonlinear_cap_line_trims_keep_selected_endpoints_and_replay() {
    const INPUT: &str = "HYPERBREP_SELECTED_CAP_LINE_REPLAY";
    const PARTITION_INPUT: &str = "HYPERBREP_SELECTED_CAP_PARTITION_REPLAY";
    fn verify(curves: &[&Curve3]) {
        assert_eq!(curves.len(), 2);
        let mut inner_signs = Vec::new();
        let mut outer_signs = Vec::new();
        for &curve in curves {
            let frame = curve.planar_geometry().expect("retained plane line");
            assert_eq!(frame.source.family(), hypercurve::CurveFamily2::Line);
            assert!(curve.domain().try_real_endpoints().is_none());
            let affine = |origin: &Real, u: &Real, v: &Real| {
                hypersolve::BivariatePolynomial::new(vec![
                    vec![origin.clone(), v.clone()],
                    vec![u.clone()],
                ])
            };
            let world_xy = hypercurve::RationalMap2::new(
                [
                    affine(&frame.origin.x, &frame.u.0[0], &frame.v.0[0]),
                    affine(&frame.origin.y, &frame.u.0[1], &frame.v.0[1]),
                ],
                hypersolve::BivariatePolynomial::new(vec![vec![Real::one()]]),
            );
            let mut signs = Vec::new();
            let mut outer = 0;
            for end in [curve.domain().start(), curve.domain().end()] {
                let point = curve.point_at(end).unwrap();
                assert_eq!(
                    point
                        .compare_linear_form(&Vector3::x(), &Real::zero())
                        .unwrap(),
                    std::cmp::Ordering::Equal
                );
                assert_eq!(
                    point.compare_linear_form(&Vector3::z(), &q(1, 2)).unwrap(),
                    std::cmp::Ordering::Equal
                );
                let sign = point
                    .compare_linear_form(&Vector3::y(), &Real::zero())
                    .unwrap();
                signs.push(sign);
                if [Real::from(-1), Real::one()].iter().any(|y| {
                    point.compare_linear_form(&Vector3::y(), y).unwrap()
                        == std::cmp::Ordering::Equal
                }) {
                    outer_signs.push(sign);
                    outer += 1;
                } else {
                    inner_signs.push(sign);
                    let uv = frame
                        .source
                        .point_at(
                            &end.affine_image(frame.source_scale, frame.source_offset)
                                .unwrap()
                                .curve_parameter(),
                        )
                        .unwrap();
                    let xy = uv.rational_image(&world_xy).unwrap();
                    // x=0 selects u=(3-sqrt(5))/2. The inner end has
                    // y²=4sqrt(5)-17/2, hence 4y⁴+68y²-31=0.
                    assert_eq!(
                        xy.rational_form(
                            hypersolve::BivariatePolynomial::new(vec![vec![
                                (-31).into(),
                                0.into(),
                                68.into(),
                                0.into(),
                                4.into()
                            ]]),
                            hypersolve::BivariatePolynomial::new(vec![vec![Real::one()]]),
                        )
                        .unwrap()
                        .compare_to_real(&Real::zero())
                        .unwrap(),
                        std::cmp::Ordering::Equal
                    );
                }
            }
            assert_eq!(outer, 1);
            assert_eq!(signs[0], signs[1]);
            let level = if signs[0] == std::cmp::Ordering::Less {
                q(-3, 4)
            } else {
                q(3, 4)
            };
            let expected = Point3::new(0.into(), level.clone(), q(1, 2));
            let cutter = Surface::plane(
                Point3::new(0.into(), level, 0.into()),
                Vector3::x(),
                Vector3::z(),
            )
            .unwrap();
            let CurveSurfaceIntersection::Points(points) = cutter.intersect_curve(curve).unwrap()
            else {
                panic!("one transverse point");
            };
            assert_eq!(points.len(), 1);
            assert!(
                curve
                    .point_at(&points[0].parameter)
                    .unwrap()
                    .coincides_with(&expected.into())
                    .unwrap()
            );
            let a = curve
                .restrict_source(
                    ParameterRange::new(
                        curve.domain().start().clone(),
                        points[0].parameter.clone(),
                    )
                    .unwrap(),
                )
                .unwrap();
            let b = curve
                .restrict_source(
                    ParameterRange::new(points[0].parameter.clone(), curve.domain().end().clone())
                        .unwrap(),
                )
                .unwrap();
            assert!(
                a.point_at(a.domain().end())
                    .unwrap()
                    .coincides_with(&b.point_at(b.domain().start()).unwrap())
                    .unwrap()
            );
        }
        inner_signs.sort();
        outer_signs.sort();
        assert_eq!(
            inner_signs,
            vec![std::cmp::Ordering::Less, std::cmp::Ordering::Greater]
        );
        assert_eq!(outer_signs, inner_signs);
    }
    fn verify_partition(model: &Model) {
        use hypercurve::{CurvePath2, CurveRegion2, FillRule, RegionPointLocation};
        let faces = model.faces().filter(|face| {
            matches!(face.surface().geometry().plane_origin(), Some(o) if o.z == q(1, 2))
        }).collect::<Vec<_>>();
        assert_eq!(faces.len(), 2);
        let mut regions = Vec::new();
        for face in &faces {
            assert_eq!(face.inner().len(), 0);
            let path = CurvePath2::try_new(
                face.outer()
                    .unwrap()
                    .edge_uses()
                    .map(|usage| usage.pcurve().geometry().curve().clone())
                    .collect(),
            )
            .unwrap();
            regions
                .push(CurveRegion2::try_from_boundary_paths(&[path], FillRule::NonZero).unwrap());
        }
        // On the cap, |y|=3/4 lies outside the removed disk for every x.
        // The two partitions cover opposite x sides, share x=0 only on the
        // material intervals, and both exclude the original central hole.
        let classify = |x: Real, y: Real| {
            faces
                .iter()
                .zip(&regions)
                .map(|(face, region)| {
                    let surface = face.surface().geometry();
                    let o = surface.plane_origin().unwrap();
                    let (u, v) = surface.plane_directions().unwrap();
                    let (dx, dy) = (&x - &o.x, &y - &o.y);
                    let det = &u.0[0] * &v.0[1] - &u.0[1] * &v.0[0];
                    let uv = Uv::new(
                        ((&dx * &v.0[1] - &dy * &v.0[0]) / &det).unwrap(),
                        ((&u.0[0] * &dy - &u.0[1] * &dx) / &det).unwrap(),
                    );
                    region.classify_point(&uv.into()).unwrap()
                })
                .collect::<Vec<_>>()
        };
        let left = classify(q(-3, 4), q(3, 4));
        let right = classify(q(3, 4), q(3, 4));
        assert!(matches!(
            left.as_slice(),
            [RegionPointLocation::Inside, RegionPointLocation::Outside]
                | [RegionPointLocation::Outside, RegionPointLocation::Inside]
        ));
        assert_eq!(right, left.into_iter().rev().collect::<Vec<_>>());
        assert_eq!(
            classify(0.into(), 0.into()),
            vec![RegionPointLocation::Outside; 2]
        );
        for y in [q(-3, 4), q(3, 4)] {
            assert_eq!(
                classify(0.into(), y),
                vec![RegionPointLocation::Boundary; 2]
            );
        }
        let first = faces[0].outer().unwrap().edge_uses().collect::<Vec<_>>();
        let second = faces[1].outer().unwrap().edge_uses().collect::<Vec<_>>();
        let shared = first
            .iter()
            .filter_map(|a| {
                second
                    .iter()
                    .find(|b| a.edge() == b.edge())
                    .map(|b| (*a, *b))
            })
            .collect::<Vec<_>>();
        assert_eq!(shared.len(), 2);
        for (a, b) in &shared {
            assert_ne!(a.direction(), b.direction());
        }
        verify(
            &shared
                .iter()
                .map(|(a, _)| a.edge().curve().geometry())
                .collect::<Vec<_>>(),
        );
    }
    if let Some(path) = std::env::var_os(PARTITION_INPUT) {
        let text = std::fs::read_to_string(path).unwrap();
        let model = RawModel::from_json(&text, &ArchiveLimits::default())
            .unwrap()
            .validate()
            .unwrap();
        verify_partition(&model);
        assert_eq!(model.to_json().unwrap(), text);
        return;
    }
    if let Some(path) = std::env::var_os(INPUT) {
        let text = std::fs::read_to_string(path).unwrap();
        let model = RawModel::from_json(&text, &ArchiveLimits::default())
            .unwrap()
            .validate()
            .unwrap();
        let curves = model
            .curves()
            .map(|curve| curve.geometry())
            .collect::<Vec<_>>();
        verify(&curves);
        assert_eq!(model.to_json().unwrap(), text);
        return;
    }
    let source = graph_fixture_over(
        [(-1).into(), q(1, 2), 1.into()],
        [[2, 0, 2], [0, -2, 0], [2, 0, 2]],
    );
    let body = BodySet::from_solid(source.solids().next().unwrap());
    let slab = builder::cuboid(pi(-2, -2, -2), Point3::new(2.into(), 2.into(), q(1, 2))).unwrap();
    let capped = boolean::intersection(&body, &slab).unwrap();
    let half = builder::cuboid(pi(-2, -2, -2), pi(0, 2, 3)).unwrap();
    let cap = capped
        .model()
        .faces()
        .find(|face| matches!(face.surface().geometry().plane_origin(), Some(o) if o.z == q(1, 2)))
        .unwrap();
    let wall = half.model().faces().find(|face| {
        let surface = face.surface().geometry();
        matches!((surface.plane_origin(), surface.plane_directions()), (Some(o), Some((u, v))) if o.x == Real::zero() && u.0[0] == Real::zero() && v.0[0] == Real::zero())
    }).unwrap();
    let pair = boolean::intersect_faces(cap, wall).unwrap().unwrap();
    let boolean::FacePairTrim::SurfaceCurveFragments(traces) = pair.details().trim() else {
        panic!("selected line trim: {:?}", pair.details().trim());
    };
    verify(&traces.iter().map(|trace| trace.curve()).collect::<Vec<_>>());
    for trace in traces {
        for end in [trace.curve().domain().start(), trace.curve().domain().end()] {
            let spatial = trace.curve().point_at(end).unwrap();
            for (surface, pcurve) in [
                (cap.surface().geometry(), trace.first_pcurve()),
                (wall.surface().geometry(), trace.second_pcurve()),
            ] {
                let uv = pcurve.point_at(end).unwrap();
                assert!(
                    surface
                        .point_at(&uv)
                        .unwrap()
                        .coincides_with(&spatial)
                        .unwrap()
                );
            }
        }
    }
    let partition = cap
        .split_by_surface_curves(traces, SurfaceIntersectionOperand::First)
        .unwrap();
    assert_eq!(partition.details().faces().len(), 2);
    verify_partition(partition.model());
    let partition_path = std::env::temp_dir().join(format!(
        "hyperbrep-selected-cap-partition-{}.json",
        std::process::id()
    ));
    std::fs::write(&partition_path, partition.model().to_json().unwrap()).unwrap();
    let replay = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "nonlinear_cap_line_trims_keep_selected_endpoints_and_replay",
            "--nocapture",
        ])
        .env(PARTITION_INPUT, &partition_path)
        .output()
        .unwrap();
    std::fs::remove_file(partition_path).unwrap();
    assert!(
        replay.status.success(),
        "{}{}",
        String::from_utf8_lossy(&replay.stdout),
        String::from_utf8_lossy(&replay.stderr)
    );
    let mut builder = ModelBuilder::new();
    for trace in traces {
        builder.curve(trace.curve().clone()).unwrap();
    }
    let text = builder.finish().unwrap().to_json().unwrap();
    let path = std::env::temp_dir().join(format!(
        "hyperbrep-selected-cap-line-{}.json",
        std::process::id()
    ));
    std::fs::write(&path, text).unwrap();
    let result = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "nonlinear_cap_line_trims_keep_selected_endpoints_and_replay",
            "--nocapture",
        ])
        .env(INPUT, &path)
        .output()
        .unwrap();
    std::fs::remove_file(path).unwrap();
    assert!(
        result.status.success(),
        "{}{}",
        String::from_utf8_lossy(&result.stdout),
        String::from_utf8_lossy(&result.stderr)
    );
}

#[test]
fn nonuniform_tensor_sections_retain_both_charts_on_folding_plane_projections() {
    fn verify(surface: &Surface, plane: &Surface, curve: &Curve3, first: &Curve2, second: &Curve2) {
        assert_eq!(curve.kind(), Curve3Kind::TensorImage);
        for t in [Real::zero(), q(1, 2), Real::one()] {
            let parameter: Parameter = t.clone().into();
            let spatial = curve.point_at(&parameter).unwrap();
            let uv = first.point_at(&t.clone().into()).unwrap();
            let yz = second.point_at(&t.clone().into()).unwrap();
            assert!(
                surface
                    .point_at(&uv)
                    .unwrap()
                    .coincides_with(&spatial)
                    .unwrap()
            );
            assert!(
                plane
                    .point_at(&yz)
                    .unwrap()
                    .coincides_with(&spatial)
                    .unwrap()
            );
            assert_eq!(
                spatial
                    .compare_linear_form(&Vector3::x(), &Real::zero())
                    .unwrap(),
                std::cmp::Ordering::Equal
            );
            // z-y²=c, c²-18c+1=0, with the selected small positive c.
            let equation = hypersolve::BivariatePolynomial::new(vec![
                vec![1.into(), (-18).into(), 1.into()],
                vec![],
                vec![18.into(), (-2).into()],
                vec![],
                vec![1.into()],
            ]);
            assert_eq!(
                yz.rational_form(
                    equation,
                    hypersolve::BivariatePolynomial::new(vec![vec![1.into()]])
                )
                .unwrap()
                .compare_to_real(&Real::zero())
                .unwrap(),
                std::cmp::Ordering::Equal
            );
            if t == q(1, 2) {
                assert_eq!(
                    spatial
                        .compare_linear_form(&Vector3::y(), &Real::zero())
                        .unwrap(),
                    std::cmp::Ordering::Equal
                );
                assert_eq!(
                    spatial
                        .compare_linear_form(&Vector3::z(), &Real::zero())
                        .unwrap(),
                    std::cmp::Ordering::Greater
                );
                assert_eq!(
                    spatial
                        .compare_linear_form(&Vector3::z(), &q(1, 16))
                        .unwrap(),
                    std::cmp::Ordering::Less
                );
            }
        }
        let next = Surface::plane(
            Point3::new(0.into(), 0.into(), q(1, 2)),
            Vector3::x(),
            Vector3::y(),
        )
        .unwrap();
        let CurveSurfaceIntersection::Points(points) = next.intersect_curve(curve).unwrap() else {
            panic!("two selected contacts");
        };
        assert_eq!(points.len(), 2);
        let mut signs = Vec::new();
        for point in points {
            let spatial = curve.point_at(&point.parameter).unwrap();
            assert_eq!(
                spatial
                    .compare_linear_form(&Vector3::x(), &Real::zero())
                    .unwrap(),
                std::cmp::Ordering::Equal
            );
            assert_eq!(
                spatial
                    .compare_linear_form(&Vector3::z(), &q(1, 2))
                    .unwrap(),
                std::cmp::Ordering::Equal
            );
            signs.push(
                spatial
                    .compare_linear_form(&Vector3::y(), &Real::zero())
                    .unwrap(),
            );
        }
        signs.sort();
        assert_eq!(
            signs,
            vec![std::cmp::Ordering::Less, std::cmp::Ordering::Greater]
        );
    }
    let model = graph_fixture_over(
        [(-1).into(), q(1, 2), 1.into()],
        [[2, 0, 2], [0, -2, 0], [2, 0, 2]],
    );
    let surface = model
        .faces()
        .find(|face| face.surface().geometry().kind() == SurfaceKind::RationalBezier)
        .unwrap()
        .surface()
        .geometry();
    let plane = Surface::plane(Point3::origin(), Vector3::y(), Vector3::z()).unwrap();
    let SurfaceSurfaceIntersection::Curve(trace) = surface.intersect_surface(&plane).unwrap()
    else {
        panic!("one selected section");
    };
    assert_eq!(trace.polynomial_contact_exponent(), Some(1));
    let first = trace.first_pcurve().materialize().unwrap();
    let second = trace.second_pcurve().materialize().unwrap();
    for pcurve in [&first, &second] {
        let Some((scale, offset)) = pcurve.correspondence().affine_coefficients() else {
            panic!("source chart");
        };
        assert_eq!(scale, &Real::one());
        assert_eq!(offset, &Real::zero());
    }
    verify(
        surface,
        &plane,
        trace.curve(),
        first.curve(),
        second.curve(),
    );
    let mut builder = ModelBuilder::new();
    builder.surface(surface.clone()).unwrap();
    builder.surface(plane).unwrap();
    builder.curve(trace.curve().clone()).unwrap();
    builder.pcurve(Pcurve::new(first.curve().clone())).unwrap();
    builder.pcurve(Pcurve::new(second.curve().clone())).unwrap();
    let text = builder.finish().unwrap().to_json().unwrap();
    let replay = RawModel::from_json(&text, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    let surfaces = replay.surfaces().map(|s| s.geometry()).collect::<Vec<_>>();
    let pcurves = replay
        .pcurves()
        .map(|p| p.curve().clone())
        .collect::<Vec<_>>();
    verify(
        surfaces[0],
        surfaces[1],
        replay.curves().next().unwrap().geometry(),
        &pcurves[0],
        &pcurves[1],
    );
    assert_eq!(replay.to_json().unwrap(), text);
}
