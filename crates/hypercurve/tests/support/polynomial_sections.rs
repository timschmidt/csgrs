//! Sections of polynomial graph branches by authored rational lines, including
//! retained rational similarities and selected restrictions of both operands.
//!
//! Expected values are derived independently of the section kernel: rational
//! contact coordinates are solved by hand, and the irrational roots of
//! `y^5 + 5y + 5 - x` are bracketed by sign changes of the univariate fibers.

use hypercurve::{
    BezierParameter2, Curve2, CurveFamily2, CurveIntersectionContact2, CurveParameter2,
    CurveParameterRange2, CurvePoint2, CurvePointDefinition2, ExactCurveError, LineSeg2, Point2,
    PolynomialCurveSet2, Real, Similarity2, UncertaintyReason,
};
use hyperreal::RealSign;
use hypersolve::bivariate_topology::{
    BivariateCurveDomain, BivariateTopologyConfig, decompose_bivariate_curve,
};
use hypersolve::{AlgebraicFiberRootIsolationConfig, BivariatePolynomial, work_budget};
use std::cmp::Ordering;

fn q(n: i64, d: i64) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

fn p(x: Real, y: Real) -> CurvePoint2 {
    Point2::new(x, y).into()
}

fn equation(coefficients: &[(usize, usize, i64)]) -> BivariatePolynomial {
    let mut rows = Vec::<Vec<Real>>::new();
    for &(i, j, value) in coefficients {
        if rows.len() <= i {
            rows.resize(i + 1, Vec::new());
        }
        if rows[i].len() <= j {
            rows[i].resize(j + 1, Real::zero());
        }
        rows[i][j] = value.into();
    }
    BivariatePolynomial::new(rows)
}

fn set(coefficients: &[(usize, usize, i64)]) -> PolynomialCurveSet2 {
    let topology = decompose_bivariate_curve(
        &equation(coefficients),
        &BivariateCurveDomain {
            first: [(-1).into(), 1.into()],
            second: [(-2).into(), 2.into()],
        },
        BivariateTopologyConfig::default(),
    )
    .unwrap();
    PolynomialCurveSet2::from_topology(topology, AlgebraicFiberRootIsolationConfig::default())
        .unwrap()
}

/// `y^5 + 5y + 5 - x`: `F_y = 5y^4 + 5 > 0`, and the fiber at `x = 0` is
/// irreducible of degree five by Eisenstein's criterion at 5.
const QUINTIC: &[(usize, usize, i64)] = &[(0, 0, 5), (0, 1, 5), (0, 5, 1), (1, 0, -1)];
/// `y^2 - x`, with a vertical tangent at the event `x = 0`.
const PARABOLA: &[(usize, usize, i64)] = &[(0, 2, 1), (1, 0, -1)];
/// `y^2 - x^3`, with a cusp at the event `x = 0`.
const CUSP: &[(usize, usize, i64)] = &[(0, 2, 1), (3, 0, -1)];

fn vertical(x: Real, y0: Real, y1: Real) -> Curve2 {
    LineSeg2::try_new(Point2::new(x.clone(), y0), Point2::new(x, y1))
        .unwrap()
        .into()
}

fn tiny() -> Real {
    let half = (Real::one() / Real::from(1_u64 << 40)).unwrap();
    &half * &half
}

fn one_contact(first: &Curve2, second: &Curve2) -> CurveIntersectionContact2 {
    let result = first.intersect_curve(second).unwrap();
    assert!(result.is_complete());
    assert!(result.overlaps().is_empty() && result.parameter_components().is_empty());
    assert_eq!(result.contacts().len(), 1);
    result.contacts()[0].clone()
}

fn assert_disjoint(first: &Curve2, second: &Curve2) {
    for (a, b) in [(first, second), (second, first)] {
        assert!(a.intersect_curve(b).unwrap().is_disjoint());
        let topology = a.intersection_topology(b).unwrap();
        assert_eq!((topology.first().len(), topology.second().len()), (1, 1));
        assert_eq!(&topology.first()[0], a);
        assert_eq!(&topology.second()[0], b);
    }
}

fn order(point: &CurvePoint2, [a, b]: [i64; 2], value: Real) -> Ordering {
    point
        .affine_form(a.into(), b.into(), Real::zero())
        .compare_to_real(&value)
        .unwrap()
}

/// The exact source fiber point: it lies on `F = 0` and on `x = c`, and keeps
/// its polynomial-fiber definition instead of Cartesian coordinates.
fn assert_fiber_point(point: &CurvePoint2, equation: &BivariatePolynomial, x: &Real) {
    assert_eq!(order(point, [1, 0], x.clone()), Ordering::Equal);
    let residual = point
        .rational_form(
            equation.clone(),
            BivariatePolynomial::new(vec![vec![Real::one()]]),
        )
        .unwrap();
    assert_eq!(
        residual.compare_to_real(&Real::zero()).unwrap(),
        Ordering::Equal
    );
    assert!(matches!(
        point.definition().unwrap(),
        CurvePointDefinition2::PolynomialFiber { .. }
    ));
    let replay = point.definition().unwrap().into_point().unwrap();
    assert!(replay.coincides_with(point).unwrap());
}

/// The line parameter is the affine form `(y - y0) / dy` of the contact point,
/// so evaluating the line replays the polynomial point itself.
fn assert_line_replays(line: &Curve2, parameter: &CurveParameter2, point: &CurvePoint2) {
    let evaluated = line.point_at(parameter).unwrap();
    assert!(evaluated.coincides_with(point).unwrap());
    if parameter.scalar().is_none() {
        let form = parameter.affine_form_source().expect("affine source");
        assert!(form.source().coincides_with(point).unwrap());
        assert!(evaluated.coordinates().is_none());
        assert!(matches!(
            evaluated.definition().unwrap(),
            CurvePointDefinition2::PolynomialFiber { .. }
        ));
    }
}

fn check_contact(
    contact: &CurveIntersectionContact2,
    branch_first: bool,
    branch_parameter: &Real,
    transverse: Option<RealSign>,
) -> (CurveParameter2, CurveParameter2) {
    let (branch, line) = if branch_first {
        (contact.first(), contact.second())
    } else {
        (contact.second(), contact.first())
    };
    assert_eq!((branch.span_index(), line.span_index()), (0, 0));
    let branch_value = branch.parameter().unwrap();
    assert_eq!(
        branch_value
            .compare(&branch_parameter.clone().into())
            .unwrap(),
        Ordering::Equal
    );
    assert_eq!(contact.is_certified_transverse(), transverse.is_some());
    assert_eq!(contact.tangent_cross_sign(), transverse);
    (branch_value, line.parameter().unwrap())
}

fn opposite(sign: RealSign) -> RealSign {
    match sign {
        RealSign::Positive => RealSign::Negative,
        RealSign::Negative => RealSign::Positive,
        RealSign::Zero => RealSign::Zero,
    }
}

pub fn quintic_section_splits_both_operands_and_continues() {
    let quintic = set(QUINTIC);
    assert_eq!(quintic.curves().len(), 1);
    assert!(quintic.isolated_points().is_empty());
    let equation = quintic.topology().equation().clone();
    let curve = quintic.curves()[0].clone();
    assert!(
        curve
            .start()
            .coincides_with(&p((-1).into(), (-1).into()))
            .unwrap()
    );
    let line = vertical(Real::zero(), (-3).into(), 3.into());

    // Both APIs and both operand orders. An upward line crosses the
    // increasing-x branch with positive cross product; swapping negates it.
    let mut point = None;
    for (first, second, branch_first) in [(&curve, &line, true), (&line, &curve, false)] {
        let contact = one_contact(first, second);
        let expected = if branch_first {
            RealSign::Positive
        } else {
            RealSign::Negative
        };
        let (_, t) = check_contact(&contact, branch_first, &Real::zero(), Some(expected));
        let contact_point = contact.point();
        assert_fiber_point(contact_point, &equation, &Real::zero());
        assert!(contact_point.coordinates().is_none());
        // y^5 + 5y + 5 changes sign on (-89/100, -88/100).
        assert_eq!(order(contact_point, [0, 1], q(-89, 100)), Ordering::Greater);
        assert_eq!(order(contact_point, [0, 1], q(-88, 100)), Ordering::Less);
        // t = (y + 3) / 6 is a selected value, not a promoted scalar.
        assert!(t.scalar().is_none() && t.as_bezier_parameter().is_none());
        assert_eq!(t.compare(&q(211, 600).into()).unwrap(), Ordering::Greater);
        assert_eq!(t.compare(&q(212, 600).into()).unwrap(), Ordering::Less);
        assert_line_replays(&line, &t, contact_point);
        // One reversed traversal negates the cross product; two restore it.
        for (a, b, sign) in [
            (first, second, expected),
            (second, first, opposite(expected)),
        ] {
            let (a_reversed, b_reversed) = (a.reversed().unwrap(), b.reversed().unwrap());
            for (a, b, sign) in [
                (&a_reversed, b, opposite(sign)),
                (a, &b_reversed, opposite(sign)),
                (&a_reversed, &b_reversed, sign),
            ] {
                let reversed = one_contact(a, b);
                assert_eq!(reversed.tangent_cross_sign(), Some(sign));
                assert!(reversed.point().coincides_with(contact_point).unwrap());
            }
        }
        point = Some(contact_point.clone());
    }
    let point = point.unwrap();

    // Split both operands in either order.
    let topology = curve.intersection_topology(&line).unwrap();
    let swapped = line.intersection_topology(&curve).unwrap();
    for (pieces, other) in [
        (topology.first(), swapped.second()),
        (topology.second(), swapped.first()),
    ] {
        assert_eq!(pieces.len(), 2);
        assert_eq!(other.len(), 2);
        for (piece, again) in pieces.iter().zip(other) {
            assert!(piece.start().coincides_with(&again.start()).unwrap());
            assert!(piece.end().coincides_with(&again.end()).unwrap());
        }
        assert!(pieces[0].end().coincides_with(&point).unwrap());
        assert!(pieces[1].start().coincides_with(&point).unwrap());
    }
    let (left, right) = (&topology.first()[0], &topology.first()[1]);
    let (below, above) = (&topology.second()[0], &topology.second()[1]);
    for piece in [left, right] {
        assert_eq!(piece.family(), CurveFamily2::ImplicitPolynomial);
        assert!(
            piece
                .polynomial_branch()
                .unwrap()
                .shares_source(curve.polynomial_branch().unwrap())
        );
    }
    assert_eq!(left.parameter_domain().end().scalar(), Some(&Real::zero()));
    assert_eq!(
        right.parameter_domain().start().scalar(),
        Some(&Real::zero())
    );
    // The line pieces end at the polynomial point itself.
    for endpoint in [below.end(), above.start()] {
        assert!(endpoint.coordinates().is_none());
        assert_fiber_point(&endpoint, &equation, &Real::zero());
    }

    // Restrict and reverse the right piece, then section it at x = 1/2.
    let restricted = right
        .restrict_source(
            CurveParameterRange2::try_new(Real::zero().into(), q(3, 4).into()).unwrap(),
        )
        .unwrap()
        .reversed()
        .unwrap();
    let half = vertical(q(1, 2), (-3).into(), 3.into());
    for (branch, line, sign) in [
        (&restricted, half.clone(), RealSign::Negative),
        (&restricted, half.reversed().unwrap(), RealSign::Positive),
        (right, half.clone(), RealSign::Positive),
    ] {
        for branch_first in [true, false] {
            let contact = if branch_first {
                one_contact(branch, &line)
            } else {
                one_contact(&line, branch)
            };
            let expected = if branch_first { sign } else { opposite(sign) };
            let (_, t) = check_contact(&contact, branch_first, &q(1, 2), Some(expected));
            assert_fiber_point(contact.point(), &equation, &q(1, 2));
            // y^5 + 5y + 9/2 changes sign on (-83/100, -82/100).
            assert_eq!(
                order(contact.point(), [0, 1], q(-83, 100)),
                Ordering::Greater
            );
            assert_eq!(order(contact.point(), [0, 1], q(-82, 100)), Ordering::Less);
            assert_line_replays(&line, &t, contact.point());
        }
        let topology = branch.intersection_topology(&line).unwrap();
        assert_eq!((topology.first().len(), topology.second().len()), (2, 2));
        assert!(
            topology.first()[0]
                .end()
                .coincides_with(&topology.first()[1].start())
                .unwrap()
        );
    }

    // The returned line pieces meet the returned branch pieces at their shared
    // endpoint. Both contact parameters are the pieces' retained boundaries.
    for (line_piece, line_end) in [(below, true), (above, false)] {
        for (branch, branch_end) in [
            (left.clone(), true),
            (right.clone(), false),
            (restricted.clone(), false),
            (left.reversed().unwrap(), true),
        ] {
            for branch_first in [true, false] {
                let contact = if branch_first {
                    one_contact(&branch, line_piece)
                } else {
                    one_contact(line_piece, &branch)
                };
                assert!(contact.point().coincides_with(&point).unwrap());
                // Both line pieces run upward; F_y > 0 at the cut endpoint.
                let sign = if branch.polynomial_branch().unwrap().is_reversed() {
                    RealSign::Negative
                } else {
                    RealSign::Positive
                };
                let sign = if branch_first { sign } else { opposite(sign) };
                let (s, t) = check_contact(&contact, branch_first, &Real::zero(), Some(sign));
                let domain = branch.parameter_domain();
                let boundary = if branch_end {
                    domain.end()
                } else {
                    domain.start()
                };
                assert_eq!(s.compare(boundary).unwrap(), Ordering::Equal);
                let domain = line_piece.parameter_domain();
                let boundary = if line_end {
                    domain.end()
                } else {
                    domain.start()
                };
                assert_eq!(t, *boundary);
                let topology = if branch_first {
                    branch.intersection_topology(line_piece).unwrap()
                } else {
                    line_piece.intersection_topology(&branch).unwrap()
                };
                assert_eq!((topology.first().len(), topology.second().len()), (1, 1));
            }
        }
    }
}

pub fn quintic_sections_respect_both_active_domains() {
    let quintic = set(QUINTIC);
    let curve = &quintic.curves()[0];
    let equation = quintic.topology().equation().clone();
    // Outside the branch, and outside the line's range.
    assert_disjoint(curve, &vertical(q(3, 2), (-3).into(), 3.into()));
    assert_disjoint(curve, &vertical(Real::zero(), Real::zero(), 3.into()));
    // A short line containing the root, and one just above it.
    let short = vertical(Real::zero(), q(-89, 100), q(-88, 100));
    let contact = one_contact(curve, &short);
    check_contact(&contact, true, &Real::zero(), Some(RealSign::Positive));
    assert_line_replays(
        &short,
        &contact.second().parameter().unwrap(),
        contact.point(),
    );
    assert_disjoint(curve, &vertical(Real::zero(), q(-88, 100), q(-87, 100)));

    // The domain-boundary event (-1, -1) is the first endpoint of both curves.
    // F_y = 10 there, so the contact is regular.
    let boundary = vertical((-1).into(), (-1).into(), 1.into());
    for branch_first in [true, false] {
        let contact = if branch_first {
            one_contact(curve, &boundary)
        } else {
            one_contact(&boundary, curve)
        };
        let sign = if branch_first {
            RealSign::Positive
        } else {
            RealSign::Negative
        };
        let (s, t) = check_contact(&contact, branch_first, &(-1).into(), Some(sign));
        assert_eq!(s, *curve.parameter_domain().start());
        assert_eq!(t.scalar(), Some(&Real::zero()));
        assert!(
            contact
                .point()
                .coincides_with(&p((-1).into(), (-1).into()))
                .unwrap()
        );
        let topology = if branch_first {
            curve.intersection_topology(&boundary).unwrap()
        } else {
            boundary.intersection_topology(curve).unwrap()
        };
        assert_eq!((topology.first().len(), topology.second().len()), (1, 1));
    }

    // (2101/3125, -4/5) is an interior rational point of the branch and the
    // first endpoint of the line, which stays whole.
    let x = q(2101, 3125);
    let line = vertical(x.clone(), q(-4, 5), 1.into());
    let contact = one_contact(curve, &line);
    let (_, t) = check_contact(&contact, true, &x, Some(RealSign::Positive));
    assert_eq!(t, *line.parameter_domain().start());
    assert!(
        contact
            .point()
            .coincides_with(&p(x.clone(), q(-4, 5)))
            .unwrap()
    );
    assert_fiber_point(contact.point(), &equation, &x);
    let topology = curve.intersection_topology(&line).unwrap();
    assert_eq!((topology.first().len(), topology.second().len()), (2, 1));

    // A cut 2^-80 beyond a previous cut stays distinct from it.
    let tiny = tiny();
    let right = curve
        .intersection_topology(&vertical(Real::zero(), (-3).into(), 3.into()))
        .unwrap()
        .first()[1]
        .clone();
    let near = vertical(tiny.clone(), (-3).into(), 3.into());
    let contact = one_contact(&right, &near);
    check_contact(&contact, true, &tiny, Some(RealSign::Positive));
    assert!(!contact.point().coincides_with(&right.start()).unwrap());
    let pieces = right.intersection_topology(&near).unwrap();
    let sliver = &pieces.first()[0];
    assert_eq!(sliver.parameter_domain().end().scalar(), Some(&tiny));
    let half_tiny = (tiny.clone() / Real::from(2)).unwrap();
    let contact = one_contact(sliver, &vertical(half_tiny.clone(), (-3).into(), 3.into()));
    assert_eq!(order(contact.point(), [1, 0], half_tiny), Ordering::Equal);
}

pub fn vertical_tangent_and_cusp_events_are_not_transverse() {
    for (coefficients, quarter) in [(PARABOLA, q(1, 2)), (CUSP, q(1, 8))] {
        let set = set(coefficients);
        assert_eq!(set.curves().len(), 2);
        let origin = p(Real::zero(), Real::zero());
        let line = vertical(Real::zero(), (-1).into(), 1.into());
        for (index, curve) in set.curves().iter().enumerate() {
            assert!(curve.start().coincides_with(&origin).unwrap());
            // Both branches start at the event; F_y = 2y vanishes there.
            for branch_first in [true, false] {
                let contact = if branch_first {
                    one_contact(curve, &line)
                } else {
                    one_contact(&line, curve)
                };
                let (s, t) = check_contact(&contact, branch_first, &Real::zero(), None);
                assert_eq!(s, *curve.parameter_domain().start());
                assert_eq!(t.compare(&q(1, 2).into()).unwrap(), Ordering::Equal);
                assert_line_replays(&line, &t, contact.point());
                assert!(contact.point().coincides_with(&origin).unwrap());
            }
            let topology = curve.intersection_topology(&line).unwrap();
            assert_eq!((topology.first().len(), topology.second().len()), (1, 2));
            assert_eq!(&topology.first()[0], curve);

            // Away from the event the branches are regular graphs.
            let y = if index == 0 {
                -&quarter
            } else {
                quarter.clone()
            };
            let contact = one_contact(curve, &vertical(q(1, 4), (-1).into(), 1.into()));
            check_contact(&contact, true, &q(1, 4), Some(RealSign::Positive));
            assert!(contact.point().coincides_with(&p(q(1, 4), y)).unwrap());
        }
    }

    // Near the parabola's event: x = 2^-80 has the rational roots +-2^-40.
    let parabola = set(PARABOLA);
    let tiny = tiny();
    let root = (Real::one() / Real::from(1_u64 << 40)).unwrap();
    let near = vertical(tiny.clone(), (-1).into(), 1.into());
    for (index, curve) in parabola.curves().iter().enumerate() {
        let y = if index == 0 { -&root } else { root.clone() };
        let contact = one_contact(curve, &near);
        check_contact(&contact, true, &tiny, Some(RealSign::Positive));
        assert!(contact.point().coincides_with(&p(tiny.clone(), y)).unwrap());
        assert!(!contact.point().coincides_with(&curve.start()).unwrap());
        let topology = curve.intersection_topology(&near).unwrap();
        let sliver = &topology.first()[0];
        assert_eq!(sliver.parameter_domain().end().scalar(), Some(&tiny));
        // The sliver's event endpoint keeps its non-transverse classification.
        let contact = one_contact(sliver, &vertical(Real::zero(), (-1).into(), 1.into()));
        check_contact(&contact, true, &Real::zero(), None);
    }
    // A line ending 2^-80 short of the upper root misses it; one 2^-80 past
    // the root contains it.
    let upper = &parabola.curves()[1];
    let half = q(1, 2);
    assert_disjoint(upper, &vertical(q(1, 4), &half + &tiny, 1.into()));
    let short = vertical(q(1, 4), &half - &tiny, &half + &tiny);
    let contact = one_contact(upper, &short);
    let t = contact.second().parameter().unwrap();
    assert_eq!(t.compare(&q(1, 2).into()).unwrap(), Ordering::Equal);
    assert_line_replays(&short, &t, contact.point());
    assert_disjoint(&parabola.curves()[0], &short);
}

pub fn selected_line_parameters_replay_their_transformed_fiber() {
    let set = set(QUINTIC);
    let source = &set.curves()[0];
    let point = source.point_at(&Real::zero().into()).unwrap();
    let parameter = point
        .affine_form(Real::zero(), q(1, 6), q(1, 2))
        .parameter()
        .unwrap();
    assert!(parameter.scalar().is_none() && parameter.as_bezier_parameter().is_none());
    let shift = Similarity2::try_from_real_affine(
        1.into(),
        0.into(),
        0.into(),
        1.into(),
        2.into(),
        (-1).into(),
    )
    .unwrap();
    let expected = source
        .transform_similarity(&shift)
        .unwrap()
        .point_at(&Real::zero().into())
        .unwrap();
    let line = vertical(2.into(), (-4).into(), 2.into());
    // This parameter is a form of the point in the original plane. Evaluating
    // the shifted line must retain its image of that same correlated fiber.
    let evaluated = line.point_at(&parameter).unwrap();
    assert!(evaluated.coincides_with(&expected).unwrap());
    assert!(
        evaluated
            .definition()
            .unwrap()
            .into_point()
            .unwrap()
            .coincides_with(&expected)
            .unwrap()
    );
    let shifted = vertical(
        Real::from(2) + q(1, 1_000_000_000_000),
        (-4).into(),
        2.into(),
    );
    assert!(
        !shifted
            .point_at(&parameter)
            .unwrap()
            .coincides_with(&expected)
            .unwrap()
    );
    let outside = point
        .affine_form(Real::zero(), Real::one(), Real::from(10))
        .parameter()
        .unwrap();
    assert!(matches!(
        line.point_at(&outside),
        Err(ExactCurveError::Invalid { .. })
    ));
}

pub fn mapped_quintic_line_sections_keep_source_parameters() {
    let quintic = set(QUINTIC);
    let source = &quintic.curves()[0];
    let translation = Similarity2::try_from_real_affine(
        1.into(),
        0.into(),
        0.into(),
        1.into(),
        1.into(),
        0.into(),
    )
    .unwrap();
    let moved = source.transform_similarity(&translation).unwrap();
    let line = vertical(Real::zero(), (-3).into(), 3.into());
    // The original unsupported translated-branch fixture: the source endpoint
    // (-1,-1) maps to (0,-1), at line parameter 1/3 and source abscissa -1.
    for (first, second, branch_first) in [(&moved, &line, true), (&line, &moved, false)] {
        let contact = one_contact(first, second);
        assert!(
            contact
                .point()
                .coincides_with(&p(0.into(), (-1).into()))
                .unwrap()
        );
        let (branch_parameter, line_parameter) = if branch_first {
            (
                contact.first().local_parameter(),
                contact.second().local_parameter(),
            )
        } else {
            (
                contact.second().local_parameter(),
                contact.first().local_parameter(),
            )
        };
        assert_eq!(
            branch_parameter.compare(&Real::from(-1).into()).unwrap(),
            Ordering::Equal
        );
        assert_eq!(
            line_parameter.compare(&q(1, 3).into()).unwrap(),
            Ordering::Equal
        );
        assert!(
            line.point_at(line_parameter)
                .unwrap()
                .coincides_with(contact.point())
                .unwrap()
        );
        assert_eq!(
            contact.tangent_cross_sign(),
            Some(if branch_first {
                RealSign::Positive
            } else {
                RealSign::Negative
            })
        );
    }

    // Integral rotations/scales/reflections have rational inverse entries.
    // The nonrational fiber at source x=0 stays selected in every world chart.
    for [a, b, d, e, tx, ty] in [
        [1, 0, 0, 1, 2, -1],
        [0, -1, 1, 0, 3, 4],
        [3, -4, 4, 3, 2, -1],
        [-3, 4, 4, 3, -2, 1],
        [1, 1, -1, 1, 2, 3],
    ] {
        let transform = Similarity2::try_from_real_affine(
            a.into(),
            b.into(),
            d.into(),
            e.into(),
            tx.into(),
            ty.into(),
        )
        .unwrap();
        let branch = source.transform_similarity(&transform).unwrap();
        let line = vertical(0.into(), (-3).into(), 3.into())
            .transform_similarity(&transform)
            .unwrap();
        let scale_squared = a * a + d * d;
        let source_x = BivariatePolynomial::new(vec![
            vec![q(-(a * tx + d * ty), scale_squared), q(d, scale_squared)],
            vec![q(a, scale_squared)],
        ]);
        let source_y = BivariatePolynomial::new(vec![
            vec![q(-(b * tx + e * ty), scale_squared), q(e, scale_squared)],
            vec![q(b, scale_squared)],
        ]);
        use hypersolve::bivariate_arithmetic::{
            bivariate_add, bivariate_multiply, bivariate_scale,
        };
        let one = BivariatePolynomial::new(vec![vec![Real::one()]]);
        let mut fifth = one.clone();
        for _ in 0..5 {
            fifth = bivariate_multiply(&fifth, &source_y);
        }
        let expected_equation = bivariate_add(
            &bivariate_add(&fifth, &bivariate_scale(source_y, &Real::from(5))),
            &bivariate_add(
                &BivariatePolynomial::new(vec![vec![Real::from(5)]]),
                &bivariate_scale(source_x, &Real::from(-1)),
            ),
        );
        for branch_reversed in [false, true] {
            let branch = if branch_reversed {
                branch.reversed().unwrap()
            } else {
                branch.clone()
            };
            for line_reversed in [false, true] {
                let line = if line_reversed {
                    line.reversed().unwrap()
                } else {
                    line.clone()
                };
                for (first, second, branch_first) in
                    [(&branch, &line, true), (&line, &branch, false)]
                {
                    let contact = one_contact(first, second);
                    let point = contact.point();
                    assert!(point.coordinates().is_none());
                    assert_eq!(
                        point
                            .rational_form(expected_equation.clone(), one.clone())
                            .unwrap()
                            .compare_to_real(&Real::zero())
                            .unwrap(),
                        Ordering::Equal,
                    );
                    let selected_y =
                        point.affine_form(b.into(), e.into(), (-(b * tx + e * ty)).into());
                    assert_eq!(
                        selected_y
                            .compare_to_real(&(Real::from(scale_squared) * q(-89, 100)))
                            .unwrap(),
                        Ordering::Greater
                    );
                    assert_eq!(
                        selected_y
                            .compare_to_real(&(Real::from(scale_squared) * q(-88, 100)))
                            .unwrap(),
                        Ordering::Less
                    );
                    let (branch_parameter, line_parameter) = if branch_first {
                        (
                            contact.first().local_parameter(),
                            contact.second().local_parameter(),
                        )
                    } else {
                        (
                            contact.second().local_parameter(),
                            contact.first().local_parameter(),
                        )
                    };
                    assert_eq!(
                        branch_parameter.compare(&Real::zero().into()).unwrap(),
                        Ordering::Equal
                    );
                    assert!(
                        line.point_at(line_parameter)
                            .unwrap()
                            .coincides_with(point)
                            .unwrap()
                    );
                    assert!(
                        point
                            .definition()
                            .unwrap()
                            .into_point()
                            .unwrap()
                            .coincides_with(point)
                            .unwrap()
                    );
                    let negative =
                        (a * e - b * d < 0) ^ branch_reversed ^ line_reversed ^ !branch_first;
                    assert_eq!(
                        contact.tangent_cross_sign(),
                        Some(if negative {
                            RealSign::Negative
                        } else {
                            RealSign::Positive
                        })
                    );
                    assert!(contact.is_certified_transverse());
                }
            }
        }
        let topology = branch.intersection_topology(&line).unwrap();
        assert_eq!((topology.first().len(), topology.second().len()), (2, 2));
        for piece in topology.first() {
            assert!(
                piece
                    .polynomial_branch()
                    .unwrap()
                    .shares_source(source.polynomial_branch().unwrap())
            );
            assert!(
                one_contact(piece, &line)
                    .point()
                    .coincides_with(&piece.point_at(&Real::zero().into()).unwrap())
                    .unwrap()
            );
        }
        for piece in topology.second() {
            one_contact(&branch, &piece.reversed().unwrap());
        }
        let shortened = line
            .restrict_source(
                CurveParameterRange2::try_new(Real::zero().into(), q(1, 4).into()).unwrap(),
            )
            .unwrap();
        assert_disjoint(&branch, &shortened);
        let outside = vertical(Real::from(-1) - tiny(), (-3).into(), 3.into())
            .transform_similarity(&transform)
            .unwrap();
        assert_disjoint(&branch, &outside);

        // A sloped source section selects an algebraic source abscissa as
        // well as a line parameter. Preserve that base selection through
        // trimming and another transform of both resulting operands.
        let sloped = segment([(-1).into(), (-3).into()], [1.into(), 3.into()])
            .transform_similarity(&transform)
            .unwrap();
        let contact = one_contact(&branch, &sloped);
        let s = contact.first().parameter().unwrap();
        assert!(s.scalar().is_none());
        assert_eq!(s.compare(&q(-31, 100).into()).unwrap(), Ordering::Greater);
        assert_eq!(s.compare(&q(-30, 100).into()).unwrap(), Ordering::Less);
        assert_eq!(
            contact
                .point()
                .affine_form(
                    (3 * a - b).into(),
                    (3 * d - e).into(),
                    (-((3 * a - b) * tx + (3 * d - e) * ty)).into(),
                )
                .compare_to_real(&Real::zero())
                .unwrap(),
            Ordering::Equal,
        );
        assert_eq!(
            contact
                .point()
                .rational_form(expected_equation, one)
                .unwrap()
                .compare_to_real(&Real::zero())
                .unwrap(),
            Ordering::Equal,
        );
        let t = contact.second().parameter().unwrap();
        let evaluated = sloped.point_at(&t).unwrap();
        for axis in [hypercurve::Axis2::X, hypercurve::Axis2::Y] {
            for (first, second) in [(&evaluated, contact.point()), (contact.point(), &evaluated)] {
                assert_eq!(
                    first.compare_coordinate(second, axis).unwrap(),
                    Ordering::Equal
                );
            }
        }
        let replay = evaluated.definition().unwrap().into_point().unwrap();
        assert!(replay.coincides_with(contact.point()).unwrap());
        assert!(evaluated.coincides_with(contact.point()).unwrap());
        let shift = Similarity2::try_from_real_affine(
            1.into(),
            0.into(),
            0.into(),
            1.into(),
            tiny(),
            0.into(),
        )
        .unwrap();
        let nearby = sloped
            .transform_similarity(&shift)
            .unwrap()
            .point_at(&t)
            .unwrap();
        assert_eq!(
            nearby
                .compare_coordinate(contact.point(), hypercurve::Axis2::X)
                .unwrap(),
            Ordering::Greater
        );
        assert_eq!(
            contact
                .point()
                .compare_coordinate(&nearby, hypercurve::Axis2::X)
                .unwrap(),
            Ordering::Less
        );
        assert_eq!(
            nearby
                .compare_coordinate(contact.point(), hypercurve::Axis2::Y)
                .unwrap(),
            Ordering::Equal
        );
        assert!(!nearby.coincides_with(contact.point()).unwrap());
        let topology = branch.intersection_topology(&sloped).unwrap();
        assert_eq!((topology.first().len(), topology.second().len()), (2, 2));
        for (piece, line) in [
            (&topology.first()[0], &topology.second()[1]),
            (&topology.first()[1], &topology.second()[0]),
        ] {
            assert!(
                one_contact(piece, line)
                    .point()
                    .coincides_with(contact.point())
                    .unwrap()
            );
            let piece = piece
                .transform_similarity(&translation)
                .unwrap()
                .reversed()
                .unwrap();
            let line = line.transform_similarity(&translation).unwrap();
            let next = one_contact(&piece, &line);
            assert_eq!(
                next.first().parameter().unwrap().compare(&s).unwrap(),
                Ordering::Equal
            );
            assert!(
                piece
                    .point_at(&s)
                    .unwrap()
                    .coincides_with(next.point())
                    .unwrap()
            );
            assert!(
                line.point_at(&next.second().parameter().unwrap())
                    .unwrap()
                    .coincides_with(next.point())
                    .unwrap()
            );
        }
    }
}

pub fn mapped_sections_preserve_singular_and_tangent_contacts() {
    for [a, b, d, e, tx, ty] in [[0, -1, 1, 0, 3, 4], [-3, 4, 4, 3, -2, 1]] {
        let transform = Similarity2::try_from_real_affine(
            a.into(),
            b.into(),
            d.into(),
            e.into(),
            tx.into(),
            ty.into(),
        )
        .unwrap();
        let image = |x: Real, y: Real| {
            p(
                Real::from(a) * &x + Real::from(b) * &y + Real::from(tx),
                Real::from(d) * x + Real::from(e) * y + Real::from(ty),
            )
        };
        let origin = image(Real::zero(), Real::zero());
        let event_line = vertical(Real::zero(), (-1).into(), 1.into())
            .transform_similarity(&transform)
            .unwrap();
        for coefficients in [PARABOLA, CUSP] {
            for source in set(coefficients).curves() {
                let curve = source.transform_similarity(&transform).unwrap();
                for curve in [curve.clone(), curve.reversed().unwrap()] {
                    for (first, second) in [(&curve, &event_line), (&event_line, &curve)] {
                        let contact = one_contact(first, second);
                        assert!(contact.point().coincides_with(&origin).unwrap());
                        assert_eq!(contact.tangent_cross_sign(), None);
                        assert!(!contact.is_certified_transverse());
                    }
                    let topology = curve.intersection_topology(&event_line).unwrap();
                    assert_eq!((topology.first().len(), topology.second().len()), (1, 2));
                    assert_eq!(&topology.first()[0], &curve);
                }
            }
        }

        let curve = set(GRAPH_PARABOLA).curves()[0]
            .transform_similarity(&transform)
            .unwrap();
        let tangent = segment([(-1).into(), Real::zero()], [1.into(), Real::zero()])
            .transform_similarity(&transform)
            .unwrap();
        for (first, second) in [(&curve, &tangent), (&tangent, &curve)] {
            let contact = one_contact(first, second);
            assert!(contact.point().coincides_with(&origin).unwrap());
            assert_eq!(contact.tangent_cross_sign(), Some(RealSign::Zero));
            assert!(!contact.is_certified_transverse());
            let topology = first.intersection_topology(second).unwrap();
            assert_eq!((topology.first().len(), topology.second().len()), (2, 2));
            for piece in topology.first() {
                assert_eq!(
                    one_contact(piece, second).tangent_cross_sign(),
                    Some(RealSign::Zero)
                );
            }
        }
        let secant = segment([(-1).into(), q(1, 4)], [1.into(), q(1, 4)])
            .transform_similarity(&transform)
            .unwrap();
        let result = curve.intersect_curve(&secant).unwrap();
        assert!(result.is_complete());
        assert_eq!(result.contacts().len(), 2);
        for (contact, (x, mut sign)) in result.contacts().iter().zip([
            (q(-1, 2), RealSign::Positive),
            (q(1, 2), RealSign::Negative),
        ]) {
            if a * e - b * d < 0 {
                sign = opposite(sign);
            }
            assert_eq!(contact.tangent_cross_sign(), Some(sign));
            assert!(contact.point().coincides_with(&image(x, q(1, 4))).unwrap());
        }

        // A continuous common component retains its paired source charts.
        let diagonal = set(DIAGONAL).curves()[0]
            .transform_similarity(&transform)
            .unwrap();
        let coincident = segment([(-1).into(), (-1).into()], [1.into(), 1.into()])
            .transform_similarity(&transform)
            .unwrap();
        for (first, second) in [(&diagonal, &coincident), (&coincident, &diagonal)] {
            let topology = first.intersection_topology(second).unwrap();
            assert!(topology.result().is_complete());
            assert!(topology.result().contacts().is_empty());
            assert_eq!(topology.result().overlaps().len(), 1);
            assert_eq!((topology.first().len(), topology.second().len()), (1, 1));
            let overlap = &topology.result().overlaps()[0];
            assert_eq!(
                overlap.orientation(),
                hypercurve::CurveOverlapOrientation2::Same
            );
            for (a, b) in [
                (
                    overlap.first_range().start(),
                    overlap.second_range().start(),
                ),
                (overlap.first_range().end(), overlap.second_range().end()),
            ] {
                assert!(
                    first
                        .point_at(a)
                        .unwrap()
                        .coincides_with(&second.point_at(b).unwrap())
                        .unwrap()
                );
            }
        }
        let distinct = segment(
            [(-1).into(), Real::from(-1) + tiny()],
            [1.into(), Real::one() + tiny()],
        )
        .transform_similarity(&transform)
        .unwrap();
        assert_disjoint(&diagonal, &distinct);
    }
}

pub fn mapped_sections_survive_budget_retry_and_policy_changes() {
    let transform = Similarity2::try_from_real_affine(
        (-3).into(),
        4.into(),
        4.into(),
        3.into(),
        (-2).into(),
        1.into(),
    )
    .unwrap();
    let curve = set(QUINTIC).curves()[0]
        .transform_similarity(&transform)
        .unwrap();
    let line = vertical(q(1, 2), (-3).into(), 3.into())
        .transform_similarity(&transform)
        .unwrap();
    let denied = work_budget::with_budget(work_budget::WorkBudget::ZERO, || {
        let error = curve.intersect_curve(&line).unwrap_err();
        assert!(matches!(error, ExactCurveError::Blocked(blocker)
            if blocker.reason() == UncertaintyReason::BudgetExhausted));
    });
    let reached = denied.result.unwrap_err();
    assert_eq!((reached.limit, reached.used), (0, 0));
    assert!(reached.requested > 0);
    let recovered = work_budget::with_budget(
        work_budget::WorkBudget {
            refinements: 100_000,
            promotions: 100_000,
        },
        || curve.intersection_topology(&line),
    );
    assert!(recovered.usage.refinements + recovered.usage.promotions > 0);
    let topology = recovered.result.unwrap().unwrap();
    assert_eq!((topology.first().len(), topology.second().len()), (2, 2));
    let expected = curve.point_at(&q(1, 2).into()).unwrap();
    assert!(
        topology.result().contacts()[0]
            .point()
            .coincides_with(&expected)
            .unwrap()
    );
    let (pieces, lines) = (topology.first().to_vec(), topology.second().to_vec());
    drop(topology);
    drop(curve);
    drop(line);
    let replay = expected.definition().unwrap().into_point().unwrap();
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
        hypercurve::PredicatePolicy::STRICT,
    ] {
        let outcome = hypercurve::evaluate_under(policy, || {
            let contact = one_contact(&pieces[1], &lines[0]);
            assert!(contact.point().coincides_with(&replay).unwrap());
            let t = contact.second().parameter().unwrap();
            assert!(
                lines[0]
                    .point_at(&t)
                    .unwrap()
                    .coincides_with(&replay)
                    .unwrap()
            );
            let topology = lines[1].intersection_topology(&pieces[0]).unwrap();
            assert_eq!((topology.first().len(), topology.second().len()), (1, 1));
        });
        assert_eq!(outcome.certainty(), hypercurve::CurveCertainty::Certified);
        outcome.certified().unwrap();
    }
}

pub fn polynomial_pair_sections_and_region_admission() {
    let quintic = set(QUINTIC);
    let curve = &quintic.curves()[0];
    // The unchanged irrational vertical and sloped lines are positive
    // witnesses in polynomial_algebraic_lines: respectively one selected
    // contact and a proved empty intersection.
    // The original rational translation case is now a positive witness in
    // mapped_quintic_line_sections_keep_source_parameters, with the same input.
    // Retain the original positive parabola branch: it is disjoint from
    // this quintic's negative y branch, in both operand orders and APIs.
    let parabola = set(PARABOLA);
    for (a, b) in [
        (curve, &parabola.curves()[1]),
        (&parabola.curves()[1], curve),
    ] {
        assert!(a.intersect_curve(b).unwrap().is_disjoint());
        let topology = a.intersection_topology(b).unwrap();
        assert!(topology.result().is_disjoint());
        assert_eq!((topology.first().len(), topology.second().len()), (1, 1));
    }
    // Preserve the original H08 boundary. Its polynomial supports now enter
    // the same arrangement as the closing line, with their selected roots.
    let range = CurveParameterRange2::try_new(Real::zero().into(), q(1, 4).into()).unwrap();
    let path = hypercurve::CurvePath2::try_new(vec![
        parabola.curves()[0].restrict_source(range.clone()).unwrap(),
        vertical(q(1, 4), q(-1, 2), q(1, 2)),
        parabola.curves()[1]
            .restrict_source(range)
            .unwrap()
            .reversed()
            .unwrap(),
    ])
    .unwrap();
    assert_eq!(
        path.simple_loop_orientation().unwrap(),
        hypercurve::CurveLoopOrientation2::Counterclockwise
    );
    let region =
        hypercurve::CurveRegion2::try_from_boundary_paths(&[path], hypercurve::FillRule::NonZero)
            .unwrap();
    assert_eq!(region.boundary_loops().len(), 1);
    assert_eq!(region.boundary_loops()[0].curves().len(), 3);
    assert_eq!(
        region
            .classify_point(&Point2::new(q(1, 8), Real::zero()).into())
            .unwrap(),
        hypercurve::RegionPointLocation::Inside
    );
}

pub fn sections_survive_context_drop_budgets_and_policies() {
    let (pieces, lines, point) = {
        let quintic = set(QUINTIC);
        let line = vertical(Real::zero(), (-3).into(), 3.into());
        let topology = quintic.curves()[0].intersection_topology(&line).unwrap();
        let point = topology.result().contacts()[0].point().clone();
        (topology.first().to_vec(), topology.second().to_vec(), point)
    };
    // The source set, curve, line and topology are gone.
    let replay = point.definition().unwrap().into_point().unwrap();
    for (branch, line) in [(&pieces[1], &lines[0]), (&pieces[0], &lines[1])] {
        let contact = one_contact(branch, line);
        assert!(contact.point().coincides_with(&replay).unwrap());
        assert!(
            line.end().coincides_with(&replay).unwrap()
                || line.start().coincides_with(&replay).unwrap()
        );
    }

    let denied = work_budget::with_budget(work_budget::WorkBudget::ZERO, || {
        pieces[1].intersect_curve(&vertical(q(1, 2), (-3).into(), 3.into()))
    });
    assert!(denied.result.is_err());
    // Two parabola roots in one fiber need subdivision to separate.
    let limited = PolynomialCurveSet2::from_topology(
        set(PARABOLA).topology().clone(),
        AlgebraicFiberRootIsolationConfig {
            max_subdivision_depth: 0,
            refinement_steps: 0,
        },
    )
    .unwrap();
    let error = limited.curves()[1]
        .intersect_curve(&vertical(q(1, 4), (-1).into(), 1.into()))
        .unwrap_err();
    assert!(
        matches!(&error, ExactCurveError::Blocked(blocker) if blocker.reason() == UncertaintyReason::BudgetExhausted),
        "{error:?}"
    );
    let recovered = one_contact(&pieces[1], &vertical(q(1, 2), (-3).into(), 3.into()));
    assert_eq!(order(recovered.point(), [1, 0], q(1, 2)), Ordering::Equal);

    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
        hypercurve::PredicatePolicy::STRICT,
    ] {
        let outcome = hypercurve::evaluate_under(policy, || {
            let contact = one_contact(&pieces[1], &vertical(q(1, 2), (-3).into(), 3.into()));
            let topology = lines[0].intersection_topology(&pieces[1]).unwrap();
            (contact.point().clone(), topology.first().len())
        });
        assert_eq!(outcome.certainty(), hypercurve::CurveCertainty::Certified);
        let (contact, count) = outcome.certified().unwrap();
        assert_eq!(count, 1);
        assert_eq!(order(&contact, [1, 0], q(1, 2)), Ordering::Equal);
    }

    // A path consumer shares the same contact and split authority.
    let quintic = set(QUINTIC);
    let path = hypercurve::CurvePath2::try_new(vec![quintic.curves()[0].clone()]).unwrap();
    let line = hypercurve::CurvePath2::try_new(vec![vertical(Real::zero(), (-3).into(), 3.into())])
        .unwrap();
    let topology = path.intersection_topology(&line).unwrap();
    assert_eq!(topology.first()[0].curves().len(), 2);
    assert_eq!(topology.second()[0].curves().len(), 2);
    assert!(matches!(
        topology.first()[0].curves()[0].end().definition().unwrap(),
        CurvePointDefinition2::PolynomialFiber {
            parameter: BezierParameter2::Exact(_),
            ..
        }
    ));
}

fn segment(start: [Real; 2], end: [Real; 2]) -> Curve2 {
    let [x0, y0] = start;
    let [x1, y1] = end;
    LineSeg2::try_new(Point2::new(x0, y0), Point2::new(x1, y1))
        .unwrap()
        .into()
}

/// `16x^2 - 16x + 16y^2 - 16y + 7`: the circle of radius 1/4 about (1/2, 1/2).
const CIRCLE: &[(usize, usize, i64)] =
    &[(0, 0, 7), (1, 0, -16), (2, 0, 16), (0, 1, -16), (0, 2, 16)];
/// `y - x^2`, a single graph branch.
const GRAPH_PARABOLA: &[(usize, usize, i64)] = &[(0, 1, 1), (2, 0, -1)];
/// `y - x`, which contains the line `y = x`.
const DIAGONAL: &[(usize, usize, i64)] = &[(0, 1, 1), (1, 0, -1)];

fn assert_on_line(point: &CurvePoint2, [a, b, c]: [i64; 3]) {
    assert_eq!(
        point
            .affine_form(a.into(), b.into(), c.into())
            .compare_to_real(&Real::zero())
            .unwrap(),
        Ordering::Equal
    );
}

/// Returns the branch abscissa and line parameter of a contact.
fn check_contact_at(
    contact: &CurveIntersectionContact2,
    branch_first: bool,
    cross: Option<RealSign>,
) -> (CurveParameter2, CurveParameter2) {
    let (branch, line) = if branch_first {
        (contact.first(), contact.second())
    } else {
        (contact.second(), contact.first())
    };
    assert_eq!((branch.span_index(), line.span_index()), (0, 0));
    assert_eq!(
        contact.is_certified_transverse(),
        cross.is_some_and(|sign| sign != RealSign::Zero)
    );
    assert_eq!(contact.tangent_cross_sign(), cross);
    (branch.parameter().unwrap(), line.parameter().unwrap())
}

pub fn sloped_section_splits_both_operands_and_continues() {
    let quintic = set(QUINTIC);
    let equation = quintic.topology().equation().clone();
    let curve = quintic.curves()[0].clone();
    // y = 3x: F(x, 3x) = 243x^5 + 14x + 5 changes sign on (-31/100, -30/100).
    let line = segment([(-1).into(), (-3).into()], [1.into(), 3.into()]);
    let mut point = None;
    for (first, second, branch_first) in [(&curve, &line, true), (&line, &curve, false)] {
        let contact = one_contact(first, second);
        let sign = if branch_first {
            RealSign::Positive
        } else {
            RealSign::Negative
        };
        let (x, t) = check_contact_at(&contact, branch_first, Some(sign));
        let contact_point = contact.point();
        assert!(contact_point.coordinates().is_none());
        assert_on_line(contact_point, [3, -1, 0]);
        assert_eq!(order(contact_point, [1, 0], q(-31, 100)), Ordering::Greater);
        assert_eq!(order(contact_point, [1, 0], q(-30, 100)), Ordering::Less);
        assert_eq!(x.compare(&q(-31, 100).into()).unwrap(), Ordering::Greater);
        let residual = contact_point
            .rational_form(
                equation.clone(),
                BivariatePolynomial::new(vec![vec![Real::one()]]),
            )
            .unwrap();
        assert_eq!(
            residual.compare_to_real(&Real::zero()).unwrap(),
            Ordering::Equal
        );
        // t = (y + 3) / 6 replays the branch point.
        assert!(t.scalar().is_none());
        assert_line_replays(&line, &t, contact_point);
        for (a, b, sign) in [(first, second, sign), (second, first, opposite(sign))] {
            let (a_reversed, b_reversed) = (a.reversed().unwrap(), b.reversed().unwrap());
            for (a, b, sign) in [
                (&a_reversed, b, opposite(sign)),
                (a, &b_reversed, opposite(sign)),
                (&a_reversed, &b_reversed, sign),
            ] {
                let contact = one_contact(a, b);
                assert_eq!(contact.tangent_cross_sign(), Some(sign));
                assert!(contact.point().coincides_with(contact_point).unwrap());
            }
        }
        point = Some(contact_point.clone());
    }
    let point = point.unwrap();

    let topology = curve.intersection_topology(&line).unwrap();
    let swapped = line.intersection_topology(&curve).unwrap();
    assert_eq!((topology.first().len(), topology.second().len()), (2, 2));
    assert_eq!((swapped.first().len(), swapped.second().len()), (2, 2));
    for pieces in [
        topology.first(),
        topology.second(),
        swapped.first(),
        swapped.second(),
    ] {
        assert!(pieces[0].end().coincides_with(&point).unwrap());
        assert!(pieces[1].start().coincides_with(&point).unwrap());
    }
    let (left, right) = (&topology.first()[0], &topology.first()[1]);
    let (below, above) = (&topology.second()[0], &topology.second()[1]);
    assert!(below.end().coordinates().is_none());
    // Each returned piece meets the other operand's adjacent pieces exactly
    // at their retained boundaries, and nowhere else.
    for (line_piece, line_end) in [(below, true), (above, false)] {
        for (branch, branch_end) in [
            (left.clone(), true),
            (right.clone(), false),
            (right.reversed().unwrap(), false),
        ] {
            let contact = one_contact(&branch, line_piece);
            assert!(contact.point().coincides_with(&point).unwrap());
            let domain = branch.parameter_domain();
            let boundary = if branch_end {
                domain.end()
            } else {
                domain.start()
            };
            assert_eq!(contact.first().parameter().unwrap(), *boundary);
            let domain = line_piece.parameter_domain();
            let boundary = if line_end {
                domain.end()
            } else {
                domain.start()
            };
            assert_eq!(contact.second().parameter().unwrap(), *boundary);
            let topology = line_piece.intersection_topology(&branch).unwrap();
            assert_eq!((topology.first().len(), topology.second().len()), (1, 1));
        }
    }
    // A second sloped line, y = x/2 - 9/10, crosses the restricted right
    // piece on (0, 1/10) and misses the left piece.
    let restricted = right
        .restrict_source(
            CurveParameterRange2::try_new(Real::zero().into(), Real::one().into()).unwrap(),
        )
        .unwrap();
    let second = segment([(-1).into(), q(-7, 5)], [1.into(), q(-2, 5)]);
    let contact = one_contact(&restricted, &second);
    assert_on_line(contact.point(), [5, -10, -9]);
    assert_eq!(
        order(contact.point(), [1, 0], Real::zero()),
        Ordering::Greater
    );
    assert_eq!(order(contact.point(), [1, 0], q(1, 10)), Ordering::Less);
    assert!(
        one_contact(&restricted.reversed().unwrap(), &second)
            .point()
            .coincides_with(contact.point())
            .unwrap()
    );
    assert!(left.intersect_curve(&second).unwrap().is_disjoint());
}

pub fn horizontal_section_keeps_only_its_own_branch_roots() {
    let circle = set(CIRCLE);
    assert_eq!(circle.curves().len(), 2);
    let (lower, upper) = (&circle.curves()[0], &circle.curves()[1]);
    // y = 3/5: (x - 1/2)^2 = 21/400, and both roots lie on the upper branch.
    let line = segment([Real::zero(), q(3, 5)], [1.into(), q(3, 5)]);
    assert_disjoint(lower, &line);
    let result = upper.intersect_curve(&line).unwrap();
    assert!(result.is_complete());
    assert_eq!(result.contacts().len(), 2);
    // The upper branch rises, then falls: against an eastward line its cross
    // sign is negative and then positive.
    for (contact, (sign, bounds)) in result.contacts().iter().zip([
        (RealSign::Negative, (q(27, 100), q(28, 100))),
        (RealSign::Positive, (q(72, 100), q(73, 100))),
    ]) {
        let (x, t) = check_contact_at(contact, true, Some(sign));
        assert_on_line(contact.point(), [0, 5, -3]);
        assert_eq!(order(contact.point(), [1, 0], bounds.0), Ordering::Greater);
        assert_eq!(order(contact.point(), [1, 0], bounds.1), Ordering::Less);
        // t = x on this line, in the base root's own field.
        assert_eq!(t.compare(&x).unwrap(), Ordering::Equal);
        assert!(
            line.point_at(&t)
                .unwrap()
                .coincides_with(contact.point())
                .unwrap()
        );
    }
    let topology = upper.intersection_topology(&line).unwrap();
    assert_eq!((topology.first().len(), topology.second().len()), (3, 3));
    // The middle pieces meet at both of their endpoints.
    let (arc, chord) = (&topology.first()[1], &topology.second()[1]);
    let result = arc.intersect_curve(chord).unwrap();
    assert_eq!(result.contacts().len(), 2);
    for contact in result.contacts() {
        let s = contact.first().parameter().unwrap();
        let domain = arc.parameter_domain();
        assert!(s == *domain.start() || s == *domain.end());
    }
    let pieces = arc.intersection_topology(chord).unwrap();
    assert_eq!((pieces.first().len(), pieces.second().len()), (1, 1));
}

pub fn tangent_lines_and_line_components_keep_their_meaning() {
    let parabola = set(GRAPH_PARABOLA);
    assert_eq!(parabola.curves().len(), 1);
    let curve = &parabola.curves()[0];
    // y = 0 is tangent at the origin: a contact with a certified zero cross
    // sign, neither a crossing nor an empty result.
    let tangent = segment([(-1).into(), Real::zero()], [1.into(), Real::zero()]);
    for branch_first in [true, false] {
        let contact = if branch_first {
            one_contact(curve, &tangent)
        } else {
            one_contact(&tangent, curve)
        };
        let (x, t) = check_contact_at(&contact, branch_first, Some(RealSign::Zero));
        assert_eq!(x.compare(&Real::zero().into()).unwrap(), Ordering::Equal);
        assert_eq!(t.compare(&q(1, 2).into()).unwrap(), Ordering::Equal);
        assert!(
            contact
                .point()
                .coincides_with(&p(Real::zero(), Real::zero()))
                .unwrap()
        );
    }
    let topology = curve.intersection_topology(&tangent).unwrap();
    assert_eq!((topology.first().len(), topology.second().len()), (2, 2));
    // y = 1/4 crosses twice; y = -1/4 misses.
    let secant = segment([(-1).into(), q(1, 4)], [1.into(), q(1, 4)]);
    let result = curve.intersect_curve(&secant).unwrap();
    assert_eq!(result.contacts().len(), 2);
    for (contact, (x, sign)) in result.contacts().iter().zip([
        (q(-1, 2), RealSign::Positive),
        (q(1, 2), RealSign::Negative),
    ]) {
        check_contact_at(contact, true, Some(sign));
        assert!(contact.point().coincides_with(&p(x, q(1, 4))).unwrap());
    }
    assert_disjoint(
        curve,
        &segment([(-1).into(), q(-1, 4)], [1.into(), q(-1, 4)]),
    );
    // A line through the domain-boundary event (-1, -1) of the quintic.
    let quintic = set(QUINTIC);
    let corner = segment([(-1).into(), (-1).into()], [1.into(), 1.into()]);
    let contact = one_contact(&quintic.curves()[0], &corner);
    let (x, t) = check_contact_at(&contact, true, Some(RealSign::Positive));
    assert_eq!(x, *quintic.curves()[0].parameter_domain().start());
    assert_eq!(t, *corner.parameter_domain().start());
    // The original shared-component fixture now retains a complete overlap.
    let diagonal = set(DIAGONAL);
    for (a, b) in [
        (&diagonal.curves()[0], &corner),
        (&corner, &diagonal.curves()[0]),
    ] {
        let topology = a.intersection_topology(b).unwrap();
        assert!(topology.result().is_complete());
        assert!(topology.result().contacts().is_empty());
        assert_eq!(topology.result().overlaps().len(), 1);
        assert_eq!((topology.first().len(), topology.second().len()), (1, 1));
        let overlap = &topology.result().overlaps()[0];
        for (s, t) in [
            (
                overlap.first_range().start(),
                overlap.second_range().start(),
            ),
            (overlap.first_range().end(), overlap.second_range().end()),
        ] {
            assert!(
                a.point_at(s)
                    .unwrap()
                    .coincides_with(&b.point_at(t).unwrap())
                    .unwrap()
            );
        }
    }
    // A crossing line on the same branch is still admitted.
    let crossing = segment([(-1).into(), 1.into()], [1.into(), (-1).into()]);
    let contact = one_contact(&diagonal.curves()[0], &crossing);
    assert!(
        contact
            .point()
            .coincides_with(&p(Real::zero(), Real::zero()))
            .unwrap()
    );
}
