#![no_main]

mod support;

use hypercurve::{
    BezierAlgebraicParameter2, BezierParameter2, BezierParameterInterval,
    BezierParameterPolynomial, Classification, Curve2, CurveContext, CurvePath2, CurvePoint2,
    CurveRegion2, LineSeg2, Point2, QuadraticBezier2, RationalQuadraticBezier2, Real,
};
use libfuzzer_sys::fuzz_target;

fn real_from_byte(byte: u8) -> Real {
    Real::from(byte as i32 - 128)
}

fn unit_from_byte(byte: u8) -> Real {
    (Real::from((byte % 17) as i32) / Real::from(16_i32)).unwrap()
}

fn rational(numerator: i32, denominator: i32) -> Real {
    (Real::from(numerator) / Real::from(denominator)).unwrap()
}

fn point(x: u8, y: u8) -> Point2 {
    Point2::new(real_from_byte(x), real_from_byte(y))
}

fn algebraic_sqrt_half(policy: &CurveContext) -> Option<BezierParameter2> {
    let polynomial = match support::under_classified_result(policy, || {
        BezierParameterPolynomial::try_new_power_basis(vec![
            Real::from(-1_i32),
            Real::from(0_i32),
            Real::from(2_i32),
        ])
    })
    .ok()?
    {
        Classification::Decided(polynomial) => polynomial,
        Classification::Uncertain(_) => return None,
    };
    let interval = match support::under_classified_result(policy, || {
        BezierParameterInterval::try_new(rational(2, 3), rational(3, 4))
    })
    .ok()?
    {
        Classification::Decided(interval) => interval,
        Classification::Uncertain(_) => return None,
    };
    let parameter = match support::under_classified_result(policy, || {
        BezierAlgebraicParameter2::try_isolate(polynomial, interval)
    })
    .ok()?
    {
        Classification::Decided(parameter) => parameter,
        Classification::Uncertain(_) => return None,
    };
    Some(BezierParameter2::Algebraic(parameter))
}

fn algebraic_sqrt_eighth(policy: &CurveContext) -> Option<BezierParameter2> {
    let polynomial = match support::under_classified_result(policy, || {
        BezierParameterPolynomial::try_new_power_basis(vec![
            Real::from(-1_i32),
            Real::from(0_i32),
            Real::from(8_i32),
        ])
    })
    .ok()?
    {
        Classification::Decided(polynomial) => polynomial,
        Classification::Uncertain(_) => return None,
    };
    let interval = match support::under_classified_result(policy, || {
        BezierParameterInterval::try_new(rational(1, 3), rational(2, 5))
    })
    .ok()?
    {
        Classification::Decided(interval) => interval,
        Classification::Uncertain(_) => return None,
    };
    let parameter = match support::under_classified_result(policy, || {
        BezierAlgebraicParameter2::try_isolate(polynomial, interval)
    })
    .ok()?
    {
        Classification::Decided(parameter) => parameter,
        Classification::Uncertain(_) => return None,
    };
    Some(BezierParameter2::Algebraic(parameter))
}

fn algebraic_chord(start: Point2, end: Point2, policy: &CurveContext) -> Option<Curve2> {
    support::under(policy, || {
        Curve2::try_line(CurvePoint2::from(start), CurvePoint2::from(end))
    })
    .ok()
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 8 {
        return;
    }

    let policy = CurveContext::STRICT;
    for chunk in data.chunks(8).take(8) {
        if chunk.len() < 8 {
            break;
        }
        let (start, end) = (point(chunk[0], chunk[1]), point(chunk[4], chunk[5]));
        let curve = Curve2::from(QuadraticBezier2::new(
            start.clone(),
            point(chunk[2], chunk[3]),
            end.clone(),
        ));
        let mut cuts = Vec::new();
        if !matches!(chunk[6] % 17, 0 | 16) {
            cuts.push(BezierParameter2::Exact(unit_from_byte(chunk[6])));
        }
        cuts.extend(algebraic_sqrt_half(&policy));
        cuts.extend(algebraic_sqrt_eighth(&policy));
        for cut in cuts {
            let Ok(outcome) = support::under(&policy, || curve.split_at(cut.into())) else {
                continue;
            };
            let (head, tail) = outcome;
            let Ok(closing) = LineSeg2::try_new(end.clone(), start.clone()) else {
                continue;
            };
            let Ok(path) = CurvePath2::try_new(vec![head, tail, closing.into()]) else {
                continue;
            };
            if let Ok(region) = CurveRegion2::try_from_boundary_paths(
                std::slice::from_ref(&path),
                hypercurve::FillRule::EvenOdd,
            ) {
                let _ = region.signed_area();
                let _ = region.loop_roles();
                let _ = region.bounds();
            }
        }
    }
    let algebraic_outer = [
        (
            Point2::new(rational(-3, 1), rational(-3, 1)),
            Point2::new(rational(3, 1), rational(-3, 1)),
        ),
        (
            Point2::new(rational(3, 1), rational(-3, 1)),
            Point2::new(rational(3, 1), rational(3, 1)),
        ),
        (
            Point2::new(rational(3, 1), rational(3, 1)),
            Point2::new(rational(-3, 1), rational(3, 1)),
        ),
        (
            Point2::new(rational(-3, 1), rational(3, 1)),
            Point2::new(rational(-3, 1), rational(-3, 1)),
        ),
    ];
    let algebraic_inner = [
        (
            Point2::new(rational(-1, 1), rational(-1, 1)),
            Point2::new(rational(1, 1), rational(-1, 1)),
        ),
        (
            Point2::new(rational(1, 1), rational(-1, 1)),
            Point2::new(rational(1, 1), rational(1, 1)),
        ),
        (
            Point2::new(rational(1, 1), rational(1, 1)),
            Point2::new(rational(-1, 1), rational(1, 1)),
        ),
        (
            Point2::new(rational(-1, 1), rational(1, 1)),
            Point2::new(rational(-1, 1), rational(-1, 1)),
        ),
    ];
    let outer = algebraic_outer
        .into_iter()
        .filter_map(|(start, end)| algebraic_chord(start, end, &policy))
        .collect::<Vec<_>>();
    let inner = algebraic_inner
        .into_iter()
        .filter_map(|(start, end)| algebraic_chord(start, end, &policy))
        .collect::<Vec<_>>();
    if outer.len() == 4 && inner.len() == 4 {
        if let (Ok(outer), Ok(inner)) = (CurvePath2::try_new(outer), CurvePath2::try_new(inner)) {
            if let Ok(region) = CurveRegion2::try_from_boundary_paths(
                &[outer, inner],
                hypercurve::FillRule::EvenOdd,
            ) {
                let _ = region.loop_roles();
            }
        }
    }

    for chunk in data.chunks(9).take(4) {
        if chunk.len() < 9 {
            break;
        }
        let weight = Real::from((chunk[8] % 31) as i32 + 1);
        if let Ok(conic) = RationalQuadraticBezier2::try_unit_end_weights(
            point(chunk[0], chunk[1]),
            point(chunk[2], chunk[3]),
            point(chunk[4], chunk[5]),
            weight,
        ) {
            let _ = conic.signed_area_contribution();
        }
    }
});
