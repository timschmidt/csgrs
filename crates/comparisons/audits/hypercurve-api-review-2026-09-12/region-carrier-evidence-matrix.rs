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
    let mut boundary_replays=0;
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
                                for (contacts, endpoint) in [(part.start_boundary_contacts(),part.curve().start()),(part.end_boundary_contacts(),part.curve().end())] {
                                    for contact in contacts {
                                        let point=certified(contact.carrier().curve().point_at(contact.boundary_parameter(),&policy).unwrap());
                                        same(point.clone(),endpoint.clone(),&policy);
                                        if let Some(retained)=contact.point() { same(point,retained.clone(),&policy); }
                                        boundary_replays+=1;
                                    }
                                }
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
    println!("native matrix done: queries={queries} boundary_replays={boundary_replays}");
    let analytic_replays=analytic_cases();
    println!("{{\"boundary_replays\":{boundary_replays},\"analytic_replays\":{analytic_replays},\"queries\":{queries},\"endpoint_replays\":{replays},\"repeated_path_trims\":{repeated},\"certainty\":\"Certified\"}}");
}

fn analytic_cap(distance: Real, dx: i32, policy: &CurveContext) -> CurveRegion2 {
    let point=|x,y|p(x+dx,y);
    let parallel=QuadraticBezier2::new(point(0,0),point(2,2),point(4,0)).parallel_left(distance).unwrap();
    let left=decided(parallel.point_at(&Real::zero(),policy).unwrap());
    let right=decided(parallel.point_at(&Real::one(),policy).unwrap());
    let range=decided(BezierParameterRange2::try_new(BezierParameter2::Exact(Real::one()),BezierParameter2::Exact(Real::zero()),policy).unwrap());
    let analytic=decided(BezierParallelFragment2::try_new(parallel,range,policy).unwrap());
    let lower_left=Point2::new(left.x().clone(),(-2).into());
    let lower_right=Point2::new(right.x().clone(),(-2).into());
    let line=|a:Point2,b:Point2| BezierSplitFragment2::Materialized {
        start:BezierParameter2::Exact(Real::zero()),end:BezierParameter2::Exact(Real::one()),
        curve:BezierSubcurve2::Quadratic(QuadraticBezier2::new(a.clone(),a.lerp(&b,q(1,2)),b)),
    };
    let boundary=CurveRegionBoundaryLoop2::new(vec![BezierSplitFragment2::AnalyticParallel(analytic),line(left,lower_left.clone()),line(lower_left,lower_right.clone()),line(lower_right,right)],policy).unwrap();
    CurveRegion2::try_new_with_loop_topology(vec![boundary],vec![CurveRegionLoopRole::Material],vec![FillRule::NonZero],vec![CurveBoundaryInteriorSide2::Left]).unwrap()
}
fn report_replay(report: &CurveRegionIntersectionResult2, policy: &CurveContext) -> usize {
    assert!(report.is_complete());
    let mut count=0;
    for contact in report.contacts() {
        let first=certified(contact.first().curve().point_at(contact.first_parameter(),policy).unwrap());
        let second=certified(contact.second().curve().point_at(contact.second_parameter(),policy).unwrap());
        same(first.clone(),second,policy);
        if let Some(point)=contact.point() {same(first,point.clone(),policy);}
        count+=2;
    }
    for overlap in report.overlaps() {
        for (a,b) in [(overlap.first_range().start(),overlap.second_range().start()),(overlap.first_range().end(),overlap.second_range().end())] {
            same(certified(overlap.first().curve().point_at(a,policy).unwrap()),certified(overlap.second().curve().point_at(b,policy).unwrap()),policy);
            count+=2;
        }
    }
    count
}
fn analytic_cases() -> usize {
    let mut count=0;
    for policy in [CurveContext::STRICT,CurveContext::APPROXIMATE_512] {
        for distance in [q(1,2),q(1,1)] { for dx in [0,5] {
            println!("analytic cap: policy={policy:?} distance={distance:?} dx={dx}");
            let cap=analytic_cap(distance.clone(),dx,&policy);
            println!("cap constructed");
            let points=[p(1+dx,-3),p(3+dx,-3),p(3+dx,3),p(1+dx,3)];
            let contour=Contour2::try_new((0..4).map(|i|Segment2::Line(LineSeg2::try_new(points[i].clone(),points[(i+1)%4].clone()).unwrap())).collect()).unwrap();
            let rectangle=certified(CurveRegion2::try_from_native_material_contours(vec![contour],&policy).unwrap());
            for (index,(a,b)) in [(&cap,&rectangle),(&rectangle,&cap),(&cap,&cap)].into_iter().enumerate() {
                println!("region query {index}");
                let report=certified(a.intersect_region(b,&policy).unwrap());
                println!("query returned: contacts={} overlaps={}", report.contacts().len(),report.overlaps().len());
                count+=report_replay(&report,&policy);
                println!("query replayed");
            }
            for x in [1,2,3] {
                println!("trim at x={x}");
                let line=Curve2::from(LineSeg2::try_new(p(x+dx,-3),p(x+dx,3)).unwrap());
                let pieces=certified(line.trim_inside_region_with_parameters(&cap,&policy).unwrap());
                assert_eq!(pieces.len(),1);
                let piece=&pieces[0];
                for (contacts,endpoint) in [(piece.start_boundary_contacts(),piece.curve().start()),(piece.end_boundary_contacts(),piece.curve().end())] {
                    assert!(!contacts.is_empty());
                    for contact in contacts {
                        let point=certified(contact.carrier().curve().point_at(contact.boundary_parameter(),&policy).unwrap());
                        same(point,endpoint.clone(),&policy);
                        count+=1;
                    }
                }
                if x==2 {same(piece.curve().end(),Point2::new((2+dx).into(),Real::one()+&distance).into(),&policy);}
            }
        }}
    }
    count
}
