use hypercurve::*;

fn q(n: i32, d: i32) -> Real { (Real::from(n) / Real::from(d)).unwrap() }
fn p(x: i32, y: i32) -> Point2 { Point2::from_values(x, y) }
fn certified<T>(out: CurveOutcome<T>) -> T {
    assert_eq!(out.certainty, CurveCertainty::Certified);
    out.value
}
fn decided<T>(out: Classification<T>) -> T {
    match out { Classification::Decided(value) => value, Classification::Uncertain(reason) => panic!("uncertified: {reason:?}") }
}
fn same(a: CurvePoint2, b: CurvePoint2, policy: &CurveContext) {
    assert!(decided(certified(a.coincides_with(&b, policy))));
}
fn rectangle(radius: i32) -> Contour2 {
    let points = [p(-radius,-radius), p(radius,-radius), p(radius,radius), p(-radius,radius)];
    Contour2::try_new((0..4).map(|i| Segment2::Line(LineSeg2::try_new(points[i].clone(), points[(i+1)%4].clone()).unwrap())).collect()).unwrap()
}
fn expected(row_quarters: i32) -> Vec<(i32,i32)> {
    let mut windows = Vec::<(i32,i32)>::new();
    for pair in [-6,-5,-4,-3,-2,-1,1,2,3,4,5,6].windows(2) {
        let middle_twice: i32 = pair[0] + pair[1];
        let depth: i32 = [(5,1),(4,-1),(3,1),(2,-1),(1,1)].into_iter()
            .filter(|&(radius,_)| middle_twice.abs()<2*radius && row_quarters.abs()<4*radius)
            .map(|(_,sign)| sign).sum();
        if depth>0 {
            if let Some(last) = windows.last_mut() && last.1 == pair[0] { last.1=pair[1]; }
            else { windows.push((pair[0],pair[1])); }
        }
    }
    windows
}
fn sources(row: i32, vertical: bool, policy: &CurveContext) -> Vec<Curve2> {
    let point = |x| if vertical { Point2::new(q(row,4),Real::from(x)) } else { Point2::new(Real::from(x),q(row,4)) };
    let controls = vec![point(-6),point(0),point(6)];
    let knots = [2,2,3,4,4].into_iter().map(Real::from).collect::<Vec<_>>();
    vec![
        Curve2::from(LineSeg2::try_new(point(-6),point(6)).unwrap()),
        Curve2::from(QuadraticBezier2::new(point(-6),point(0),point(6))),
        Curve2::from(CubicBezier2::new(point(-6),point(-2),point(2),point(6))),
        Curve2::from(RationalQuadraticBezier2::try_new(point(-6),point(0),point(6),Real::one(),Real::from(2),Real::from(3)).unwrap()),
        Curve2::from(RationalBezier2::try_new(vec![point(-6),point(-2),point(2),point(6)],vec![Real::one(),Real::from(2),Real::from(3),Real::from(4)]).unwrap()),
        certified(Curve2::try_polynomial_bspline(1,controls.clone(),knots.clone(),policy).unwrap()),
        certified(Curve2::try_nurbs(1,controls,vec![Real::one(),Real::from(2),Real::from(3)],knots,policy).unwrap()),
    ]
}
fn main() {
    let mut queries=0;
    let mut replays=0;
    let mut repeated=0;
    for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512] {
        let region = certified(CurveRegion2::try_from_native_contours(vec![rectangle(5),rectangle(3),rectangle(1)],vec![rectangle(4),rectangle(2)],&policy).unwrap());
        for row in [-21,-17,-9,-1,1,9,17,21] {
            for vertical in [false,true] {
                let point = |x| if vertical { Point2::new(q(row,4),Real::from(x)) } else { Point2::new(Real::from(x),q(row,4)) };
                for source in sources(row,vertical,&policy) {
                    for reversed in [false,true] {
                        let source = if reversed { certified(source.reversed(&policy).unwrap()) } else {source.clone()};
                        let path=CurvePath2::try_new(vec![source.clone()]).unwrap();
                        let result=certified(path.trim_inside_region(&region,&policy).unwrap());
                        let mut windows=expected(row);
                        if reversed { windows.reverse(); for pair in &mut windows { *pair=(pair.1,pair.0); } }
                        assert_eq!(result.len(),windows.len(),"row={row} vertical={vertical} reversed={reversed} family={:?}",source.family());
                        for (chunk,(start,end)) in result.iter().zip(windows) {
                            let first=chunk.fragments().first().unwrap().trim_fragment();
                            let last=chunk.fragments().last().unwrap().trim_fragment();
                            same(first.curve().start(),point(start).into(),&policy);
                            same(last.curve().end(),point(end).into(),&policy);
                            assert!(!first.start_boundary_contacts().is_empty());
                            assert!(!last.end_boundary_contacts().is_empty());
                            let curves=chunk.fragments().iter().map(|part| {
                                let part=part.trim_fragment();
                                let range=decided(part.parameter_range(&policy).unwrap());
                                same(certified(source.point_at(range.start(),&policy).unwrap()),part.curve().start(),&policy);
                                same(certified(source.point_at(range.end(),&policy).unwrap()),part.curve().end(),&policy);
                                replays+=2;
                                part.curve().clone()
                            }).collect();
                            let next=CurvePath2::try_new(curves).unwrap();
                            let retrimmed=certified(next.trim_inside_region(&region,&policy).unwrap());
                            assert_eq!(retrimmed.len(),1);
                            let fragment=retrimmed[0].fragments().first().unwrap().trim_fragment();
                            same(fragment.curve().start(),point(start).into(),&policy);
                            let fragment=retrimmed[0].fragments().last().unwrap().trim_fragment();
                            same(fragment.curve().end(),point(end).into(),&policy);
                            repeated+=1;
                        }
                        queries+=1;
                    }
                }
            }
        }
    }
    println!("{{\"queries\":{queries},\"endpoint_replays\":{replays},\"repeated_path_trims\":{repeated},\"certainty\":\"Certified\"}}");
}
