//! Public image-query controls shared by native and WASM consumers.
use hyperbrep::*;
use hypercurve::{Curve2, LineSeg2, Point2 as Uv};
use hyperlimit::{PredicateOutcome, point3_equal};

struct IsoCircle {
    surface: Surface,
    center: Point3,
    axes: [Vector3; 2],
    radius: Real,
    fixed: Real,
    varying_u: bool,
    periodic: bool,
}

fn cases() -> Vec<IsoCircle> {
    let origin = Point3::origin();
    let frame = || [Vector3::x(), Vector3::y()];
    let half_pi = (Real::pi() / Real::from(2)).unwrap();
    let angle = (Real::pi() / Real::from(4)).unwrap();
    let sphere = Surface::sphere(
        origin.clone(),
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        Real::from(2),
    )
    .unwrap();
    let torus = Surface::torus(
        origin.clone(),
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        Real::from(3),
        Real::one(),
    )
    .unwrap();
    vec![
        IsoCircle {
            surface: Surface::cylinder(
                origin.clone(),
                Vector3::x(),
                Vector3::y(),
                Vector3::z(),
                Real::one(),
            )
            .unwrap(),
            center: Point3::new(0.into(), 0.into(), 2.into()),
            axes: frame(),
            radius: Real::one(),
            fixed: 2.into(),
            varying_u: true,
            periodic: true,
        },
        IsoCircle {
            surface: sphere.clone(),
            center: origin.clone(),
            axes: frame(),
            radius: 2.into(),
            fixed: 0.into(),
            varying_u: true,
            periodic: true,
        },
        IsoCircle {
            surface: sphere.clone(),
            center: origin.clone() + Vector3::z() * (Real::from(2) * angle.clone().sin()),
            axes: frame(),
            radius: Real::from(2) * angle.clone().cos(),
            fixed: angle.clone(),
            varying_u: true,
            periodic: true,
        },
        IsoCircle {
            surface: sphere,
            center: origin.clone(),
            axes: [Vector3::y(), Vector3::z()],
            radius: 2.into(),
            fixed: half_pi.clone(),
            varying_u: false,
            periodic: false,
        },
        IsoCircle {
            surface: Surface::cone(
                origin.clone(),
                Vector3::x(),
                Vector3::y(),
                Vector3::z(),
                angle,
            )
            .unwrap(),
            center: Point3::new(0.into(), 0.into(), 1.into()),
            axes: frame(),
            radius: Real::one(),
            fixed: Real::from(2).sqrt().unwrap(),
            varying_u: true,
            periodic: true,
        },
        IsoCircle {
            surface: torus.clone(),
            center: Point3::new(0.into(), 0.into(), 1.into()),
            axes: frame(),
            radius: 3.into(),
            fixed: half_pi.clone(),
            varying_u: true,
            periodic: true,
        },
        IsoCircle {
            surface: torus,
            center: Point3::new(0.into(), 3.into(), 0.into()),
            axes: [Vector3::y(), Vector3::z()],
            radius: Real::one(),
            fixed: half_pi,
            varying_u: false,
            periodic: true,
        },
    ]
}

pub fn parameterizations() {
    for (case_index, case) in cases().into_iter().enumerate() {
        for partial in [false, true] {
            for rotated in [false, true] {
                for reverse_curve in [false, true] {
                    for direction in [Direction::Forward, Direction::Reversed] {
                        let mut builder = ModelBuilder::new();
                        let phase = if rotated {
                            (Real::pi() / Real::from(2)).unwrap()
                        } else {
                            Real::zero()
                        };
                        let (x, y) = if rotated {
                            (case.axes[1].clone(), -case.axes[0].clone())
                        } else {
                            (case.axes[0].clone(), case.axes[1].clone())
                        };
                        let (a, b) = if case.periodic {
                            (Real::zero(), Real::tau())
                        } else {
                            (
                                (-Real::pi() / Real::from(4)).unwrap(),
                                (Real::pi() / Real::from(4)).unwrap(),
                            )
                        };
                        let (a, b) = (a - &phase, b - &phase);
                        let curve = Curve3::circle_arc(
                            case.center.clone(),
                            x,
                            y,
                            case.radius.clone(),
                            a.clone(),
                            b.clone(),
                        )
                        .unwrap();
                        let curve = if reverse_curve {
                            curve.reversed().unwrap()
                        } else {
                            curve
                        };
                        let quarter = ((&b - &a) / Real::from(4)).unwrap();
                        let (start, end) = if partial {
                            (&a + &quarter, &b - &quarter)
                        } else {
                            (a.clone(), b.clone())
                        };
                        let first = curve.point_at(&start.clone().into()).unwrap();
                        let second = curve.point_at(&end.clone().into()).unwrap();
                        let first = builder.vertex(first).unwrap();
                        let second = if !partial && case.periodic {
                            first.clone()
                        } else {
                            builder.vertex(second).unwrap()
                        };
                        let curve_key = builder.curve(curve.clone()).unwrap();
                        let edge = builder
                            .edge(
                                &first,
                                &second,
                                &curve_key,
                                ParameterDomain::new(start.clone(), end.clone())
                                    .unwrap()
                                    .into(),
                            )
                            .unwrap();
                        let theta = |t: &Real| {
                            if reverse_curve {
                                &a + &b - t + &phase
                            } else {
                                t + &phase
                            }
                        };
                        let uv = |angle: Real| {
                            if case.varying_u {
                                Uv::new(angle, case.fixed.clone())
                            } else {
                                Uv::new(case.fixed.clone(), angle)
                            }
                        };
                        let (from, to) = if direction == Direction::Forward {
                            (&start, &end)
                        } else {
                            (&end, &start)
                        };
                        let pcurve = builder
                            .pcurve(Pcurve::new(Curve2::from(
                                LineSeg2::try_new(uv(theta(from)), uv(theta(to))).unwrap(),
                            )))
                            .unwrap();
                        let correspondence =
                            ParameterCorrespondence::affine(to - from, from.clone()).unwrap();
                        let surface = builder.surface(case.surface.clone()).unwrap();
                        for _ in 0..2 {
                            builder.validate_edge_image(&edge, direction, &pcurve, correspondence.clone(), &surface).unwrap_or_else(|e| panic!("case {case_index}, partial {partial}, rotated {rotated}, reverse {reverse_curve}, direction {direction:?}: {e:?}"));
                        }
                        // An independent evaluation oracle for the fixture: the
                        // source-domain points equal the explicitly chosen iso.
                        for fraction in [0, 1, 2, 3, 4] {
                            let t = from
                                + (to - from) * (Real::from(fraction) / Real::from(4)).unwrap();
                            let point = curve.point_at(&t.clone().into()).unwrap();
                            let surface_point = case
                                .surface
                                .point_at(&if case.varying_u {
                                    Point2::new(theta(&t), case.fixed.clone())
                                } else {
                                    Point2::new(case.fixed.clone(), theta(&t))
                                })
                                .unwrap();
                            assert!(matches!(
                                point3_equal(
                                    point.try_coordinates().unwrap(),
                                    &surface_point,
                                    STRICT_PREDICATES
                                ),
                                PredicateOutcome::Decided { value: true, .. }
                            ));
                        }
                    }
                }
            }
        }
    }
}

pub fn faults() {
    for case in cases() {
        for radial in [-case.axes[0].clone(), case.axes[0].cross(&case.axes[1])] {
            let mut builder = ModelBuilder::new();
            let curve = Curve3::circle_arc(
                case.center.clone(),
                radial,
                case.axes[1].clone(),
                case.radius.clone(),
                Real::zero(),
                (Real::pi() / Real::from(4)).unwrap(),
            )
            .unwrap();
            let first = builder
                .vertex(curve.point_at(&Real::zero().into()).unwrap())
                .unwrap();
            let end = curve.domain().end().clone();
            let second = builder
                .vertex(curve.point_at(&end.clone().into()).unwrap())
                .unwrap();
            let curve = builder.curve(curve).unwrap();
            let edge = builder
                .edge(
                    &first,
                    &second,
                    &curve,
                    ParameterDomain::new(Real::zero(), end.clone())
                        .unwrap()
                        .into(),
                )
                .unwrap();
            let uv = |angle| {
                if case.varying_u {
                    Uv::new(angle, case.fixed.clone())
                } else {
                    Uv::new(case.fixed.clone(), angle)
                }
            };
            let pcurve = builder
                .pcurve(Pcurve::new(Curve2::from(
                    LineSeg2::try_new(uv(Real::zero()), uv(end.clone())).unwrap(),
                )))
                .unwrap();
            let surface = builder.surface(case.surface.clone()).unwrap();
            let result =
                hypercurve::evaluate_under(hypercurve::PredicatePolicy::APPROXIMATE_512, || {
                    builder.validate_edge_image(
                        &edge,
                        Direction::Forward,
                        &pcurve,
                        ParameterCorrespondence::affine(end, Real::zero()).unwrap(),
                        &surface,
                    )
                });
            assert!(matches!(
                result.certified().unwrap(),
                Err(BuildError::EdgeUseSupportMismatch)
            ));
        }
    }
}
