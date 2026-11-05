//! Native cylinder sections by nonparallel cylinders, off-axis spheres and
//! off-axis ring tori, through an exact rational atlas of the cylinder.
//!
//! Topology is checked independently of the implementation: piece endpoints
//! are chained by exact coincidence, every junction must join exactly two
//! piece ends, and the closed loops are counted.

use hyperbrep::*;
use std::cmp::Ordering;

fn q(n: i64, d: i64) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn root(n: i64) -> Real {
    Real::from(n).sqrt().unwrap()
}

fn p(x: Real, y: Real, z: Real) -> Point3 {
    Point3::new(x, y, z)
}

fn pi(x: i64, y: i64, z: i64) -> Point3 {
    Point3::new(x.into(), y.into(), z.into())
}

/// x^2 + y^2 = r^2 about the z axis.
fn z_cylinder(radius: Real) -> Surface {
    Surface::cylinder(
        Point3::origin(),
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        radius,
    )
    .unwrap()
}

/// y^2 + z^2 = r^2 about the x axis.
fn x_cylinder(radius: Real) -> Surface {
    Surface::cylinder(
        Point3::origin(),
        Vector3::y(),
        Vector3::z(),
        Vector3::x(),
        radius,
    )
    .unwrap()
}

fn pieces(result: SurfaceSurfaceIntersection) -> Vec<SurfaceIntersectionCurve> {
    match result {
        SurfaceSurfaceIntersection::Curves(curves) => curves,
        SurfaceSurfaceIntersection::Curve(curve) => vec![*curve],
        other => panic!("curve pieces required, found {other:?}"),
    }
}

fn endpoints(piece: &SurfaceIntersectionCurve) -> [GeometricPoint3; 2] {
    let curve = piece.curve();
    [
        curve.point_at(curve.domain().start()).unwrap(),
        curve.point_at(curve.domain().end()).unwrap(),
    ]
}

/// Exact seam points of the atlas of a cylinder about an origin with frame
/// (`x`, `y`) and radius `r`, as `origin + r(c x + s y)` without the height.
struct Seams {
    origin: Point3,
    x: Vector3,
    y: Vector3,
    radius: Real,
    /// Axis whose sign separates the loops; on every seam line each sign-half
    /// of the fixture's intersection crosses exactly once (argued per test).
    sign_axis: Vector3,
}

impl Seams {
    /// The seam index of a point whose two in-plane coordinates match a seam.
    /// Only exact linear forms against represented seam values are used.
    fn index(&self, point: &GeometricPoint3) -> Option<usize> {
        seams().iter().position(|(c, s)| {
            let at = self.origin.clone()
                + self.x.clone() * (&self.radius * c)
                + self.y.clone() * (&self.radius * s);
            [&self.x, &self.y].into_iter().all(|direction| {
                point
                    .compare_linear_form(direction, &direction.dot(&Vector3::from(at.clone())))
                    .unwrap()
                    == Ordering::Equal
            })
        })
    }
}

/// Number of closed loops formed by the pieces; panics on a junction that
/// does not join exactly two piece ends.
///
/// Seam ends are identified by (seam, sign) alone: each bucket must hold
/// exactly two ends, justified by the fixture's uniqueness argument, so no
/// costly equality proof between independently selected points of adjacent
/// patches is needed. Interior event ends are matched by exact coincidence
/// only with ends of the same patch and sign.
fn loops(pieces: &[SurfaceIntersectionCurve], seams: &Seams) -> usize {
    type Key = (Option<usize>, Ordering, Option<Point3>);
    let mut nodes = Vec::<(Key, GeometricPoint3, usize)>::new();
    let mut node = |piece: &SurfaceIntersectionCurve, point: GeometricPoint3| -> usize {
        let seam = seams.index(&point);
        let sign = point
            .compare_linear_form(&seams.sign_axis, &Real::zero())
            .unwrap();
        let patch = seam.is_none().then(|| {
            piece
                .curve()
                .tensor_geometry()
                .unwrap()
                .surface
                .point_at(&hypercurve::Point2::from_values(0, 0).into())
                .unwrap()
                .try_coordinates()
                .unwrap()
                .clone()
        });
        let same_patch = |a: &Option<Point3>, b: &Option<Point3>| match (a, b) {
            (None, None) => true,
            (Some(p), Some(q)) => GeometricPoint3::from(p.clone())
                .coincides_with(&q.clone().into())
                .unwrap(),
            _ => false,
        };
        let found = nodes.iter().position(|(key, existing, _)| {
            key.0 == seam
                && key.1 == sign
                && same_patch(&key.2, &patch)
                && (seam.is_some() || existing.coincides_with(&point).unwrap())
        });
        match found {
            Some(index) => {
                nodes[index].2 += 1;
                index
            }
            None => {
                nodes.push(((seam, sign, patch), point, 1));
                nodes.len() - 1
            }
        }
    };
    let edges = pieces
        .iter()
        .map(|piece| {
            let [a, b] = endpoints(piece);
            (node(piece, a), node(piece, b))
        })
        .collect::<Vec<_>>();
    let count = nodes.len();
    assert!(
        nodes.iter().all(|(_, _, ends)| *ends == 2),
        "open or branched junction"
    );
    let mut parent = (0..count).collect::<Vec<_>>();
    fn find(parent: &mut [usize], mut i: usize) -> usize {
        while parent[i] != i {
            parent[i] = parent[parent[i]];
            i = parent[i];
        }
        i
    }
    for &(a, b) in &edges {
        let (ra, rb) = (find(&mut parent, a), find(&mut parent, b));
        parent[ra] = rb;
    }
    (0..count).filter(|&i| find(&mut parent, i) == i).count()
}

/// Unit cylinder about z, loops separated by the sign of z.
fn unit_seams() -> Seams {
    Seams {
        origin: Point3::origin(),
        x: Vector3::x(),
        y: Vector3::y(),
        radius: Real::one(),
        sign_axis: Vector3::z(),
    }
}

/// Whether some piece ends at the expected point, which has represented x and y;
/// z is compared only for endpoints with those exact x and y.
fn has_endpoint(pieces: &[SurfaceIntersectionCurve], expected: &Point3) -> bool {
    pieces.iter().any(|piece| {
        endpoints(piece).iter().any(|point| {
            point
                .compare_linear_form(&Vector3::x(), &expected.x)
                .unwrap()
                == Ordering::Equal
                && point
                    .compare_linear_form(&Vector3::y(), &expected.y)
                    .unwrap()
                    == Ordering::Equal
                && point
                    .compare_linear_form(&Vector3::z(), &expected.z)
                    .unwrap()
                    == Ordering::Equal
        })
    })
}

/// Unit-circle atlas seam points at half-angle tangents 0, ±5/12, ±3/4,
/// ±4/3, ±12/5 and infinity.
fn seams() -> Vec<(Real, Real)> {
    let mut points = vec![(q(1, 1), q(0, 1)), (q(-1, 1), q(0, 1))];
    for (x, y) in [(119_i64, 120_i64), (7, 24), (-7, 24), (-119, 120)] {
        let d = if x.abs() == 119 { 169 } else { 25 };
        points.push((q(x, d), q(y, d)));
        points.push((q(x, d), q(-y, d)));
    }
    points
}

/// The plan's decisive pair x^2 + y^2 = 1, y^2 + z^2 = 2: two disjoint smooth
/// nonplanar loops, z = ±sqrt(2 - y^2), with no rational parameterization.
#[test]
fn unequal_perpendicular_cylinders_give_two_exact_loops() {
    let all = pieces(
        z_cylinder(1.into())
            .intersect_surface(&x_cylinder(root(2)))
            .unwrap(),
    );
    for piece in &all {
        assert_eq!(piece.curve().kind(), Curve3Kind::TensorImage);
        // The atlas is internal; no operand pcurve is claimed.
        assert!(piece.first_pcurve().materialize().is_err());
        assert!(piece.second_pcurve().materialize().is_err());
    }
    assert_eq!(loops(&all, &unit_seams()), 2);
    // Each loop crosses every atlas seam (c, s) at z = ±sqrt(2 - s^2).
    for (c, s) in seams() {
        let z = (Real::from(2) - &s * &s).sqrt().unwrap();
        assert!(has_endpoint(&all, &p(c.clone(), s.clone(), z.clone())));
        assert!(has_endpoint(&all, &p(c, s, -z)));
    }
    // The loops separate by the sign of z.
    let upper = all
        .iter()
        .filter(|piece| {
            endpoints(piece)[0]
                .compare_linear_form(&Vector3::z(), &Real::zero())
                .unwrap()
                == Ordering::Greater
        })
        .cloned()
        .collect::<Vec<_>>();
    assert_eq!(loops(&upper, &unit_seams()), 1);
    assert!(upper.len() < all.len());
    // Operand exchange charts the other cylinder: the pieces differ but form
    // the same two loops. On that atlas's seam lines (fixed y, z) the loops
    // satisfy x = ±sqrt(1 - y^2), one crossing per sign of x.
    let swapped = pieces(
        x_cylinder(root(2))
            .intersect_surface(&z_cylinder(1.into()))
            .unwrap(),
    );
    let swapped_seams = Seams {
        origin: Point3::origin(),
        x: Vector3::y(),
        y: Vector3::z(),
        radius: root(2),
        sign_axis: Vector3::x(),
    };
    assert_eq!(loops(&swapped, &swapped_seams), 2);
}

/// Cutting both loops with x = 0 gives exactly the four transverse contacts
/// (0, ±1, ±1).
#[test]
fn plane_cut_of_the_native_loops_has_four_contacts() {
    let all = pieces(
        z_cylinder(1.into())
            .intersect_surface(&x_cylinder(root(2)))
            .unwrap(),
    );
    let plane = Surface::plane(Point3::origin(), Vector3::y(), Vector3::z()).unwrap();
    let mut found = Vec::<GeometricPoint3>::new();
    for piece in &all {
        match plane.intersect_curve(piece.curve()).unwrap() {
            CurveSurfaceIntersection::None => {}
            CurveSurfaceIntersection::Points(points) => {
                for contact in points {
                    assert_eq!(contact.kind, CurveSurfaceContactKind::Simple);
                    if !found
                        .iter()
                        .any(|f| f.coincides_with(&contact.point).unwrap())
                    {
                        found.push(contact.point);
                    }
                }
            }
            other => panic!("isolated contacts required, found {other:?}"),
        }
    }
    assert_eq!(found.len(), 4);
    for (y, z) in [(1, 1), (1, -1), (-1, 1), (-1, -1)] {
        assert!(
            found
                .iter()
                .any(|f| f.coincides_with(&pi(0, y, z).into()).unwrap())
        );
    }
}

/// Equal radii: z^2 = x^2, so every piece lies in z = x or z = -x.
#[test]
fn equal_radius_control_reduces_to_planar_conics() {
    let all = pieces(
        z_cylinder(1.into())
            .intersect_surface(&x_cylinder(1.into()))
            .unwrap(),
    );
    assert!(!all.is_empty());
    let plus = Vector3::new([1.into(), 0.into(), (-1).into()]);
    let minus = Vector3::new([1.into(), 0.into(), 1.into()]);
    for piece in &all {
        for point in endpoints(piece) {
            let on_plus =
                point.compare_linear_form(&plus, &Real::zero()).unwrap() == Ordering::Equal;
            let on_minus =
                point.compare_linear_form(&minus, &Real::zero()).unwrap() == Ordering::Equal;
            assert!(on_plus || on_minus);
        }
    }
}

/// The sphere |p - (1,0,0)| = 1 meets the unit cylinder where z^2 = 2x - 1:
/// one closed loop through the seams with x >= 1/2.
#[test]
fn off_axis_sphere_gives_one_loop_through_known_seams() {
    let sphere = Surface::sphere(
        pi(1, 0, 0),
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        1.into(),
    )
    .unwrap();
    let all = pieces(sphere.intersect_surface(&z_cylinder(1.into())).unwrap());
    assert_eq!(loops(&all, &unit_seams()), 1);
    assert!(has_endpoint(&all, &pi(1, 0, 1)));
    assert!(has_endpoint(&all, &pi(1, 0, -1)));
    let z = (root(69) / Real::from(13)).unwrap();
    assert!(has_endpoint(&all, &p(q(119, 169), q(120, 169), z.clone())));
    assert!(has_endpoint(&all, &p(q(119, 169), q(-120, 169), -z)));
}

/// A thin cylinder about the y-directed line through (3, 0, 0) pierces the
/// tube of the R = 3, r = 1 torus twice.
#[test]
fn off_axis_torus_gives_two_loops() {
    let torus = Surface::torus(
        Point3::origin(),
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        3.into(),
        1.into(),
    )
    .unwrap();
    let thin = Surface::cylinder(
        pi(3, 0, 0),
        Vector3::z(),
        Vector3::x(),
        Vector3::y(),
        q(1, 2),
    )
    .unwrap();
    let all = pieces(thin.intersect_surface(&torus).unwrap());
    // Seams lie on lines (3 + s/2, y, c/2). There the outer tube radius
    // 3 + sqrt(1 - z^2) >= 3.87 exceeds |x| <= 3.5 while the inner radius
    // 3 - sqrt(1 - z^2) <= 2.13 is below x >= 2.5, so each sign of y has
    // exactly one crossing.
    let thin_seams = Seams {
        origin: pi(3, 0, 0),
        x: Vector3::z(),
        y: Vector3::x(),
        radius: q(1, 2),
        sign_axis: Vector3::y(),
    };
    assert_eq!(loops(&all, &thin_seams), 2);
}

#[test]
fn distant_carriers_give_complete_empty_results() {
    let far = Surface::sphere(
        pi(9, 0, 0),
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        1.into(),
    )
    .unwrap();
    assert!(matches!(
        far.intersect_surface(&z_cylinder(1.into())).unwrap(),
        SurfaceSurfaceIntersection::None
    ));
    // Skew axes (z, and x through (0, 5, 0)) at distance 5 with radii 1, 2.
    let skew = Surface::cylinder(
        pi(0, 5, 0),
        Vector3::y(),
        Vector3::z(),
        Vector3::x(),
        2.into(),
    )
    .unwrap();
    assert!(matches!(
        z_cylinder(1.into()).intersect_surface(&skew).unwrap(),
        SurfaceSurfaceIntersection::None
    ));
}
