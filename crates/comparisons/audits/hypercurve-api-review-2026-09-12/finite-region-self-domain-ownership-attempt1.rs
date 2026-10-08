use hypercurve::*;

fn p(x: i32, y: i32) -> Point2 { Point2::from_values(x, y) }
fn q(n: i32, d: i32) -> Real { (Real::from(n) / Real::from(d)).unwrap() }
fn exact<T>(out: CurveOutcome<T>) -> T {
    assert_eq!(out.certainty, CurveCertainty::Certified);
    out.value
}

fn decided<T>(value: Classification<T>) -> T {
    match value { Classification::Decided(v) => v, Classification::Uncertain(e) => panic!("{e:?}") }
}

fn region(curves: Vec<Curve2>, policy: &CurveContext) -> CurveRegion2 {
    let path = CurvePath2::try_new(curves).unwrap();
    exact(CurveRegion2::try_from_boundary_paths_with_loop_semantics(
        &[path], &[CurveRegionLoopRole::Material], &[FillRule::NonZero], policy,
    ).unwrap())
}

fn fixture(finite: bool, policy: &CurveContext) -> (Curve2, CurveRegion2) {
    // The same nodal cubic in two charts: P(t)=(t²-1,t³-t), t in [-2,2],
    // and Q(s)=P(4s-2), s in [0,1]. Both are closed by the same chord.
    let source = if finite {
        let fragment = BezierSplitFragment2::RetainedBezier {
            source_curve: BezierSubcurve2::Cubic(CubicBezier2::new(
                p(-1, 0), Point2::new((-1).into(), q(-1, 3)),
                Point2::new(q(-2, 3), q(-2, 3)), p(0, 0),
            )),
            start: BezierParameter2::Exact((-2).into()),
            end: BezierParameter2::Exact(2.into()),
            start_image: None, end_image: None, reversed: false,
        };
        let chord = BezierSplitFragment2::AlgebraicChord(decided(
            BezierAlgebraicChord2::try_new(p(3,6).into(), p(3,-6).into(), policy).unwrap()
        ));
        let boundary = CurveRegionBoundaryLoop2::new(vec![fragment, chord], policy).unwrap();
        let raw = CurveRegion2::new(vec![boundary]).unwrap();
        let paths = decided(exact(raw.boundary_paths(policy).unwrap()));
        let source = paths[0].curves()[0].clone();
        let region = exact(CurveRegion2::try_from_boundary_paths_with_loop_semantics(
            &paths, &[CurveRegionLoopRole::Material], &[FillRule::NonZero], policy,
        ).unwrap());
        return (source, region);
    } else {
        Curve2::from(CubicBezier2::new(
            p(3, -6), Point2::new(q(-7, 3), q(26, 3)),
            Point2::new(q(-7, 3), q(-26, 3)), p(3, 6),
        ))
    };
    let region = region(vec![source.clone(), Curve2::from(
        LineSeg2::try_new(p(3, 6), p(3, -6)).unwrap(),
    )], policy);
    (source, region)
}

fn box_region(left: Real, right: Real, lower: Real, upper: Real, policy: &CurveContext) -> CurveRegion2 {
    let corners = [Point2::new(left.clone(), lower.clone()), Point2::new(right.clone(), lower),
        Point2::new(right, upper.clone()), Point2::new(left, upper)];
    region((0..4).map(|i| Curve2::from(LineSeg2::try_new(corners[i].clone(), corners[(i+1)%4].clone()).unwrap())).collect(), policy)
}

fn location(label: &str, region: &CurveRegion2, point: Point2, expected: RegionPointLocation, policy: &CurveContext, failures: &mut usize) {
    let result = region.classify_point(&point, policy);
    let valid = matches!(&result, Ok(out) if out.certainty == CurveCertainty::Certified && out.value == Classification::Decided(expected));
    println!("{label} point={point:?} expected={expected:?} result={result:?} valid={valid}");
    if !valid { *failures += 1; }
}

fn inspect(label: &str, region: &CurveRegion2, policy: &CurveContext, failures: &mut usize) {
    for (point, expected) in [
        (Point2::new(q(-1,2), Real::zero()), RegionPointLocation::Inside),
        (p(1,0), RegionPointLocation::Inside),
        (p(-2,0), RegionPointLocation::Outside),
        (p(4,0), RegionPointLocation::Outside),
        (Point2::new(Real::zero(), q(1,4)), RegionPointLocation::Outside),
        (p(0,0), RegionPointLocation::Boundary),
        (p(-1,0), RegionPointLocation::Boundary),
    ] { location(label, region, point, expected, policy, failures); }
}

fn inspect_ownership(label: &str, region: &CurveRegion2, policy: &CurveContext, failures: &mut usize) {
    let sides = decided(exact(region.filled_side_is_left(policy).unwrap()));
    let paths = decided(exact(region.boundary_paths(policy).unwrap()));
    for (path, &left) in paths.iter().zip(sides) {
        for curve in path.curves() {
            let range = curve.parameter_domain();
            let (Some(start), Some(end)) = (range.start().scalar(), range.end().scalar()) else { panic!("fixture cuts are represented") };
            let parameter = ((start + end) / Real::from(2)).unwrap();
            let point = exact(curve.point_at(&parameter.clone().into(), policy).unwrap());
            let point = point.coordinates().expect("fixture midpoint has represented coordinates");
            let tangent = exact(curve.derivative_at(&parameter, policy).unwrap());
            for side in [true, false] {
                let step = if side { q(1,128) } else { q(-1,128) };
                let sample = Point2::new(point.x() - &step * tangent.dy(), point.y() + &step * tangent.dx());
                let expected = if side == left { RegionPointLocation::Inside } else { RegionPointLocation::Outside };
                location(&format!("{label} ownership left={left} sample-left={side}"), region, sample, expected, policy, failures);
            }
        }
    }
}

fn main() {
    let mut failures = 0;
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        for finite in [false, true] {
            let label = format!("finite={finite} policy={policy:?}");
            let (source, raw) = fixture(finite, &policy);
            let result = exact(source.intersect_curve(&source, &policy).unwrap());
            println!("{label} self contacts={} overlaps={} complete={}", result.contacts().len(), result.overlaps().len(), result.is_complete());
            if !result.is_complete() || result.contacts().len() != 2 { failures += 1; }
            for contact in result.contacts() {
                assert_eq!(exact(contact.point().coincides_with(&p(0,0).into(), &policy)), Classification::Decided(true));
                for loc in [contact.first(), contact.second()] {
                    let Classification::Decided(parameter) = loc.parameter(&policy).unwrap() else { panic!("retained location is exact") };
                    let point = exact(source.point_at(&parameter, &policy).unwrap());
                    assert_eq!(exact(point.coincides_with(contact.point(), &policy)), Classification::Decided(true));
                }
            }
            inspect(&format!("{label} raw"), &raw, &policy, &mut failures);
            match raw.regularized_region(&policy) {
                Ok(out) => {
                    let normalized = exact(out);
                    println!("{label} regularized loops={}", normalized.boundary_loops().len());
                    inspect(&format!("{label} normalized"), &normalized, &policy, &mut failures);
                    inspect_ownership(&label, &normalized, &policy, &mut failures);
                }
                Err(error) => { println!("{label} regularized error={error:?}"); failures += 1; }
            }
            for (name, clip, center) in [
                ("left", box_region(q(-13,16), q(-11,16), q(-1,16), q(1,16), &policy), Point2::new(q(-3,4), Real::zero())),
                ("right", box_region(q(1,2), Real::one(), q(-1,4), q(1,4), &policy), Point2::new(q(3,4), Real::zero())),
            ] {
                match raw.boolean_regions(&clip, &policy) {
                    Ok(out) => {
                        let result = exact(out);
                        location(&format!("{label} {name} intersection"), result.intersection(), center, RegionPointLocation::Inside, &policy, &mut failures);
                    }
                    Err(error) => { println!("{label} {name} boolean error={error:?}"); failures += 1; }
                }
            }
        }
    }
    println!("failures={failures}");
    assert_eq!(failures, 0);
}
