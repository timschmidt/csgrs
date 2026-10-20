//! Circle-image admission with independently authored equivalent angular spans.
use hyperbrep::*;
use hypercurve::{Curve2, LineSeg2, Point2 as Uv};

pub fn proofs() {
    let q = |n, d| (Real::from(n) / Real::from(d)).unwrap();
    let spatial = q(3, 5).asin().unwrap();
    let uv = q(3, 4).atan().unwrap();
    let gap = q(1, 1_000_000) * q(1, 1_000_000);
    for divisor in [1, 2, 4, 16, 256] {
        let spatial = (&spatial / Real::from(divisor)).unwrap();
        let uv = (&uv / Real::from(divisor)).unwrap();
        for direction in [Direction::Forward, Direction::Reversed] {
            for adjustment in [Real::zero(), Real::tau(), -Real::tau(), gap.clone()] {
                let mut builder = ModelBuilder::new();
                let curve = Curve3::circle_arc(
                    Point3::origin(),
                    Vector3::x(),
                    Vector3::y(),
                    5.into(),
                    Real::zero(),
                    spatial.clone(),
                )
                .unwrap();
                let start = builder
                    .vertex(Point3::new(5.into(), 0.into(), 0.into()).into())
                    .unwrap();
                let end = builder.vertex(curve.end().unwrap().into()).unwrap();
                let curve = builder.curve(curve).unwrap();
                let edge = builder
                    .edge(
                        &start,
                        &end,
                        &curve,
                        ParameterDomain::new(Real::zero(), spatial.clone())
                            .unwrap()
                            .into(),
                    )
                    .unwrap();
                let (a, b, scale, offset) = if direction == Direction::Forward {
                    (
                        Real::zero(),
                        &uv + &adjustment,
                        spatial.clone(),
                        Real::zero(),
                    )
                } else {
                    (
                        &uv + &adjustment,
                        Real::zero(),
                        -spatial.clone(),
                        spatial.clone(),
                    )
                };
                let pcurve = builder
                    .pcurve(Pcurve::new(Curve2::from(
                        LineSeg2::try_new(Uv::new(a, Real::zero()), Uv::new(b, Real::zero()))
                            .unwrap(),
                    )))
                    .unwrap();
                let surface = builder
                    .surface(
                        Surface::cylinder(
                            Point3::origin(),
                            Vector3::x(),
                            Vector3::y(),
                            Vector3::z(),
                            5.into(),
                        )
                        .unwrap(),
                    )
                    .unwrap();
                let map = ParameterCorrespondence::affine(scale, offset).unwrap();
                for _ in 0..2 {
                    let result = builder.validate_edge_image(
                        &edge,
                        direction,
                        &pcurve,
                        map.clone(),
                        &surface,
                    );
                    if adjustment == Real::zero() {
                        result.unwrap();
                    } else {
                        assert!(
                            result.is_err(),
                            "a full extra turn or a distinct endpoint cannot be admitted"
                        );
                    }
                }
            }
        }
    }
}
