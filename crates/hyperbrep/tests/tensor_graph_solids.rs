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
/// meet; it replays cold and transforms, and its volume stays a typed
/// unsupported measurement until curved-trim integrals exist.
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
    assert!(matches!(
        solid.volume(),
        Err(QueryError::Geometry(GeometryError::UnsupportedMeasurement))
    ));
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
