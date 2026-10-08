use hypercurve::*;
fn p(x: i32, y: i32) -> Point2 { Point2::from_values(x, y) }
fn q(n: i32, d: i32) -> Real { (Real::from(n) / Real::from(d)).unwrap() }
fn exact<T>(v: Classification<T>) -> T { match v { Classification::Decided(v) => v, Classification::Uncertain(e) => panic!("{e:?}") } }
fn certified<T>(v: CurveOutcome<T>) -> T { assert_eq!(v.certainty, CurveCertainty::Certified); v.value }
fn exterior_line(policy: &CurveContext) -> Curve2 {
    // P(t)=(-3+2t,0) on [1,2] is the same finite segment as (-1,0)->(1,0).
    // Publish it as an edge of a valid rectangle, using its retained source chart.
    let first = BezierSplitFragment2::RetainedBezier {
        reversed: false,
        source_curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(-3, 0), p(-2, 0), p(-1, 0))),
        start: BezierParameter2::Exact(Real::one()),
        end: BezierParameter2::Exact(Real::from(2)),
        start_image: None,
        end_image: None,
    };
    let mut edges = vec![first];
    for (a, b) in [(p(1, 0), p(1, 1)), (p(1, 1), p(-1, 1)), (p(-1, 1), p(-1, 0))] {
        edges.push(BezierSplitFragment2::AlgebraicChord(exact(BezierAlgebraicChord2::try_new(a.into(), b.into(), policy).unwrap())));
    }
    let boundary = CurveRegionBoundaryLoop2::new(edges, policy).unwrap();
    let region = CurveRegion2::try_new_with_loop_topology(vec![boundary], vec![CurveRegionLoopRole::Material], vec![FillRule::NonZero], vec![CurveBoundaryInteriorSide2::Left]).unwrap();
    exact(certified(region.boundary_paths(policy).unwrap()))[0].curves()[0].clone()
}
fn main() {
    let (mut complete, mut incomplete, mut replays, mut topologies) = (0, 0, 0, 0);
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let path = CurvePath2::try_new(vec![
            LineSeg2::try_new(p(-4, 0), p(0, 0)).unwrap().into(),
            QuadraticBezier2::new(p(0, 0), p(0, 1), p(1, 2)).into(),
        ]).unwrap();
        let CurveCornerSolutions2::Unique(path) = certified(path.fillet_vertex_by_radius(1, q(1, 4), CurveCornerMode2::TrimOnly, &policy).unwrap()) else { panic!("unique fillet") };
        let circle = &path.curves()[1];
        assert!(circle.geometry().is_none());
        let native = Curve2::from(LineSeg2::try_new(p(-1, 0), p(1, 0)).unwrap());
        let exterior = exterior_line(&policy);
        assert!(exact(certified(native.start().coincides_with(&exterior.start(), &policy))));
        assert!(exact(certified(native.end().coincides_with(&exterior.end(), &policy))));
        for (name, line) in [("unit", &native), ("exterior", &exterior)] {
            for swapped in [false, true] {
                let (a, b) = if swapped { (line, circle) } else { (circle, line) };
                hyperreal::dispatch_trace::reset();
                let result = hyperreal::dispatch_trace::with_recording(|| certified(a.intersect_curve(b, &policy).unwrap()));
                for entry in hyperreal::dispatch_trace::take() { if entry.layer == "hypercurve" { eprintln!("{entry:?}"); } }
                println!("{name} swapped={swapped}: complete={} contacts={} overlaps={} blockers={:?}", result.is_complete(), result.contacts().len(), result.overlaps().len(), result.blockers());
                assert!(result.is_complete()); assert_eq!(result.contacts().len(), 1);
                let contact=&result.contacts()[0];
                for (curve, location) in [(a, contact.first()), (b, contact.second())] {
                    let parameter=exact(location.parameter(&policy).unwrap());
                    let point=certified(curve.point_at(&parameter, &policy).unwrap());
                    assert!(exact(certified(point.coincides_with(contact.point(), &policy))));
                    replays+=1;
                }
                let topology=certified(a.intersection_topology(b, &policy).unwrap());
                assert!(topology.result().is_complete()); topologies+=1;
                if result.is_complete() { complete += 1; } else { incomplete += 1; }
            }
        }
    }
    println!("{{\"complete\":{complete},\"incomplete\":{incomplete},\"point_replays\":{replays},\"topologies\":{topologies}}}");
}
