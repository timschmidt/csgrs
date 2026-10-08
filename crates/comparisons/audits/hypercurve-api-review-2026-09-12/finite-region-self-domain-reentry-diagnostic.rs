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

fn fixture(finite: bool, elevated: bool, reversed: bool, policy: &CurveContext) -> (Curve2, CurveRegion2) {
    let cubic = if finite {
        CubicBezier2::new(p(-1,0), Point2::new((-1).into(),q(-1,3)), Point2::new(q(-2,3),q(-2,3)),p(0,0))
    } else {
        CubicBezier2::new(p(3,-6),Point2::new(q(-7,3),q(26,3)),Point2::new(q(-7,3),q(-26,3)),p(3,6))
    };
    let source_curve = if elevated {
        BezierSubcurve2::Rational(RationalBezier2::try_new(cubic.control_points().into_iter().cloned().collect(),vec![Real::one();4]).unwrap().elevated_to_degree(5).unwrap())
    } else { BezierSubcurve2::Cubic(cubic) };
    let fragment = BezierSplitFragment2::RetainedBezier {
        source_curve, start:BezierParameter2::Exact(if finite {(-2).into()}else{0.into()}),
        end:BezierParameter2::Exact(if finite {2.into()}else{1.into()}),reversed:false,start_image:None,end_image:None,
    };
    let chord = BezierSplitFragment2::AlgebraicChord(decided(BezierAlgebraicChord2::try_new(p(3,6).into(),p(3,-6).into(),policy).unwrap()));
    let boundary=CurveRegionBoundaryLoop2::new(vec![fragment,chord],policy).unwrap();
    let raw=CurveRegion2::new(vec![boundary]).unwrap();
    let paths=decided(exact(raw.boundary_paths(policy).unwrap()));
    let source=paths[0].curves()[0].clone();
    let mut curves=vec![source.clone(),Curve2::from(LineSeg2::try_new(p(3,6),p(3,-6)).unwrap())];
    if reversed {curves=curves.into_iter().rev().map(|c|exact(c.reversed(policy).unwrap())).collect();}
    (source,region(curves,policy))
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
    let mut checked = 0;
    let point = p(-1,0);
    for (path, &left) in paths.iter().zip(sides) {
        for curve in path.curves() {
            for parameter in [Real::zero(), q(1,2)] {
                let Ok(out) = curve.point_at(&parameter.clone().into(), policy) else { continue; };
                if exact(exact(out).coincides_with(&point.clone().into(), policy)) != Classification::Decided(true) { continue; }
                let tangent = exact(curve.derivative_at(&parameter, policy).unwrap());
                checked += 1;
                for side in [true, false] {
                    let step = if side { q(1,128) } else { q(-1,128) };
                    let sample = Point2::new(point.x() - &step * tangent.dy(), point.y() + &step * tangent.dx());
                    let expected = if side == left { RegionPointLocation::Inside } else { RegionPointLocation::Outside };
                    location(&format!("{label} ownership left={left} sample-left={side}"), region, sample, expected, policy, failures);
                }
            }
        }
    }
    assert_eq!(checked, 1, "one regular visit to the leftmost point");
}

fn main() {
    for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512] {
        for finite in [false,true] { for elevated in [false,true] { for reversed in [false,true] {
            let (_,mut region)=fixture(finite,elevated,reversed,&policy);
            for generation in 0..2 {
                print!("finite={finite} elevated={elevated} reversed={reversed} generation={generation} policy={policy:?} ");
                match region.regularized_region(&policy) {
                    Ok(out)=>{region=exact(out);println!("loops={}",region.boundary_loops().len());},
                    Err(err)=>{println!("ERROR {err:?}");break;}
                }
                region=exact(region.boolean_region(&region,BooleanOp::Union,&policy).unwrap());
            }
        }}}
    }
}
