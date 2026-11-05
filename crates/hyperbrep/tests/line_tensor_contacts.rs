//! Complete contacts between finite lines and rational Bézier tensors.
//!
//! Expectations come from the defining equations stated beside each witness.

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

fn line(start: Point3, end: Point3) -> Curve3 {
    Curve3::line(start, end).unwrap()
}

/// z = x^2 + y^2 over [-1, 1]^2 with x = 2v - 1, y = 2u - 1.
fn paraboloid() -> Surface {
    let a = [1, -1, 1];
    let xs = [-1, 0, 1];
    Surface::rational_bezier(
        (0..3)
            .map(|i| (0..3).map(|j| pi(xs[i], xs[j], a[i] + a[j])).collect())
            .collect(),
        vec![vec![Real::one(); 3]; 3],
    )
    .unwrap()
}

/// The arc of y^2 + z^2 = 2 from (7/5, 1/5) to (1/5, 7/5) with middle weight
/// 4/5, swept along x in [-2, 2].
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

fn contacts(surface: &Surface, curve: &Curve3) -> Vec<CurveSurfacePoint> {
    match surface.intersect_curve(curve).unwrap() {
        CurveSurfaceIntersection::Points(points) => {
            for point in &points {
                assert!(
                    point
                        .point
                        .coincides_with(&curve.point_at(&point.parameter).unwrap())
                        .unwrap()
                );
            }
            points
        }
        other => panic!("isolated contacts required, found {other:?}"),
    }
}

fn assert_contacts(
    points: &[CurveSurfacePoint],
    expected: &[(Real, Point3, CurveSurfaceContactKind)],
) {
    assert_eq!(points.len(), expected.len());
    for (point, (parameter, position, kind)) in points.iter().zip(expected) {
        assert_eq!(
            point
                .parameter
                .compare(&Parameter::from_real(parameter.clone()))
                .unwrap(),
            Ordering::Equal
        );
        assert!(
            point
                .point
                .coincides_with(&position.clone().into())
                .unwrap()
        );
        assert_eq!(point.kind, *kind);
    }
}

use CurveSurfaceContactKind::{Repeated, Simple};

/// The vertical line x = 1/2, y = 0 meets z = x^2 + y^2 at z = 1/4, which is
/// t = 5/12 on z = -1 + 3t.
#[test]
fn vertical_line_pierces_the_paraboloid_once() {
    let points = contacts(
        &paraboloid(),
        &line(
            p(q(1, 2), 0.into(), (-1).into()),
            p(q(1, 2), 0.into(), 2.into()),
        ),
    );
    assert_contacts(
        &points,
        &[(q(5, 12), p(q(1, 2), 0.into(), q(1, 4)), Simple)],
    );
}

/// At height 1/4 the paraboloid section is x = ±1/2 (t = 3/8, 5/8 on
/// x = -2 + 4t); at height 0 the line is tangent at the origin; at height 1/2
/// the contacts are x = ±sqrt(2)/2, t = 1/2 ∓ sqrt(2)/8.
#[test]
fn horizontal_lines_give_secant_tangent_and_irrational_contacts() {
    let at = |z: Real| {
        line(
            p((-2).into(), 0.into(), z.clone()),
            p(2.into(), 0.into(), z),
        )
    };
    assert_contacts(
        &contacts(&paraboloid(), &at(q(1, 4))),
        &[
            (q(3, 8), p(q(-1, 2), 0.into(), q(1, 4)), Simple),
            (q(5, 8), p(q(1, 2), 0.into(), q(1, 4)), Simple),
        ],
    );
    assert_contacts(
        &contacts(&paraboloid(), &at(Real::zero())),
        &[(q(1, 2), Point3::origin(), Repeated)],
    );
    let half = (root(2) / Real::from(2)).unwrap();
    let offset = (root(2) / Real::from(8)).unwrap();
    assert_contacts(
        &contacts(&paraboloid(), &at(q(1, 2))),
        &[
            (
                q(1, 2) - &offset,
                p(-half.clone(), 0.into(), q(1, 2)),
                Simple,
            ),
            (q(1, 2) + &offset, p(half, 0.into(), q(1, 2)), Simple),
        ],
    );
    // Below the vertex there is no contact.
    assert!(matches!(
        paraboloid().intersect_curve(&at((-1).into())).unwrap(),
        CurveSurfaceIntersection::None
    ));
}

/// The diagonal x = y = -1 + 2t at height 1 meets z = 2x^2 where
/// x = ±sqrt(2)/2, at t = 1/2 ∓ sqrt(2)/4.
#[test]
fn oblique_line_contacts_are_selected_parameters_that_split_and_replay() {
    let curve = line(pi(-1, -1, 1), pi(1, 1, 1));
    let half = (root(2) / Real::from(2)).unwrap();
    let offset = (root(2) / Real::from(4)).unwrap();
    let points = contacts(&paraboloid(), &curve);
    assert_contacts(
        &points,
        &[
            (
                q(1, 2) - &offset,
                p(-half.clone(), -half.clone(), 1.into()),
                Simple,
            ),
            (q(1, 2) + &offset, p(half.clone(), half, 1.into()), Simple),
        ],
    );
    // Reversal maps t to 1 - t.
    let reversed = contacts(&paraboloid(), &curve.reversed().unwrap());
    assert_eq!(reversed.len(), 2);
    assert!(reversed[0].point.coincides_with(&points[1].point).unwrap());
    let mut builder = ModelBuilder::new();
    let a = builder.vertex(pi(-1, -1, 1).into()).unwrap();
    let b = builder.vertex(pi(1, 1, 1).into()).unwrap();
    let id = builder.curve(curve.clone()).unwrap();
    builder.edge(&a, &b, &id, curve.domain().clone()).unwrap();
    let model = builder.finish().unwrap();
    let split = model
        .edges()
        .next()
        .unwrap()
        .split(points[0].parameter.clone())
        .unwrap();
    assert!(
        split
            .details()
            .vertex()
            .point()
            .coincides_with(&points[0].point)
            .unwrap()
    );
    let json = split.model().to_json().unwrap();
    let cold = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(cold.to_json().unwrap(), json);
    assert_eq!(cold.edges().count(), 2);
    assert_eq!(
        contacts(&paraboloid(), cold.curves().next().unwrap().geometry()).len(),
        2
    );
}

/// On the rational cylinder patch, the line x = 0, z = 1 meets y^2 + 1 = 2 at
/// y = 1, t = 1/2 on y = 2t.
#[test]
fn line_meets_a_rational_cylinder_patch() {
    let points = contacts(&cylinder_patch(), &line(pi(0, 0, 1), pi(0, 2, 1)));
    assert_contacts(&points, &[(q(1, 2), pi(0, 1, 1), Simple)]);
}

#[test]
fn coplanar_tensors_and_budgets_remain_typed() {
    let flat = Surface::rational_bezier(
        vec![
            vec![pi(-1, -1, 0), pi(-1, 1, 0)],
            vec![pi(1, -1, 0), pi(1, 1, 0)],
        ],
        vec![vec![Real::one(); 2]; 2],
    )
    .unwrap();
    // A line in the tensor's own plane meets it in a range; the shared UV
    // component is a typed limitation (an unsupported relation, a topology
    // blocker, or Hypercurve's coincident-branch blocker), never isolated
    // contacts.
    assert!(matches!(
        flat.intersect_curve(&line(pi(-2, 0, 0), pi(2, 0, 0))),
        Err(GeometryError::UnsupportedIntersection
            | GeometryError::PolynomialTopology(_)
            | GeometryError::PlanarCurve(_))
    ));
    let curve = line(
        p(q(1, 2), 0.into(), (-1).into()),
        p(q(1, 2), 0.into(), 2.into()),
    );
    let denied = OperationContext::strict(WorkBudget::ZERO)
        .evaluate(|| paraboloid().intersect_curve(&curve));
    assert_eq!(denied.result.unwrap_err().limit, 0);
    assert_eq!(contacts(&paraboloid(), &curve).len(), 1);
}
