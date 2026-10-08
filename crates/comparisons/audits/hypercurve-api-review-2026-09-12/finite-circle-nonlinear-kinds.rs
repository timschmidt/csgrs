use hypercurve::*;
fn p(x: i32, y: i32) -> Point2 { Point2::from_values(x, y) }
fn q(n: i32, d: i32) -> Real { (Real::from(n) / Real::from(d)).unwrap() }
fn exact<T>(v: Classification<T>) -> T { match v { Classification::Decided(v) => v, Classification::Uncertain(e) => panic!("{e:?}") } }
fn certified<T>(v: CurveOutcome<T>) -> T { assert_eq!(v.certainty, CurveCertainty::Certified); v.value }
fn exterior_parabola(policy: &CurveContext) -> Curve2 {
    // P(t)=(-(t-1)^2,t-1) on [1,2] has the same image and traversal
    // as Q(u)=(-u^2,u) on [0,1]. Its chord closes a positive-area cap.
    let first = BezierSplitFragment2::RetainedBezier {
        reversed: false,
        source_curve: BezierSubcurve2::Quadratic(QuadraticBezier2::new(p(-1, -1), Point2::new(Real::zero(), q(-1, 2)), p(0, 0))),
        start: BezierParameter2::Exact(Real::one()),
        end: BezierParameter2::Exact(Real::from(2)),
        start_image: None,
        end_image: None,
    };
    let edges = vec![first, BezierSplitFragment2::AlgebraicChord(exact(BezierAlgebraicChord2::try_new(p(-1, 1).into(), p(0, 0).into(), policy).unwrap()))];
    let boundary = CurveRegionBoundaryLoop2::new(edges, policy).unwrap();
    let region = CurveRegion2::try_new_with_loop_topology(vec![boundary], vec![CurveRegionLoopRole::Material], vec![FillRule::NonZero], vec![CurveBoundaryInteriorSide2::Left]).unwrap();
    exact(certified(region.boundary_paths(policy).unwrap()))[0].curves()[0].clone()
}
fn brief(value: &impl std::fmt::Debug) -> String {
    struct Prefix(String);
    impl std::fmt::Write for Prefix {
        fn write_str(&mut self, value: &str) -> std::fmt::Result {
            let remaining = 180_usize.saturating_sub(self.0.chars().count());
            self.0.extend(value.chars().take(remaining));
            if value.chars().count() > remaining { Err(std::fmt::Error) } else { Ok(()) }
        }
    }
    let mut output=Prefix(String::new());
    let _=std::fmt::write(&mut output,format_args!("{value:?}"));
    output.0
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
        let native = Curve2::from(QuadraticBezier2::new(p(0, 0), Point2::new(Real::zero(), q(1, 2)), p(-1, 1)));
        let exterior = exterior_parabola(&policy);
        assert!(exterior.geometry().is_none());
        assert!(exact(certified(native.start().coincides_with(&exterior.start(), &policy))));
        assert!(exact(certified(native.end().coincides_with(&exterior.end(), &policy))));
        for (name, line) in [("unit", &native), ("exterior", &exterior)] {
            for swapped in [false, true] {
                let (a, b) = if swapped { (line, circle) } else { (circle, line) };
                println!("starting {name} swapped={swapped} policy={policy:?}");
                let result = certified(a.intersect_curve(b, &policy).unwrap());
                println!("{name} swapped={swapped}: complete={} contacts={} overlaps={} blockers={:?}", result.is_complete(), result.contacts().len(), result.overlaps().len(), result.blockers());
                if !result.is_complete() { incomplete += 1; continue; }
                assert_eq!(result.contacts().len(), 1);
                let contact=&result.contacts()[0];
                for (curve, location) in [(a, contact.first()), (b, contact.second())] {
                    let parameter=exact(location.parameter(&policy).unwrap());
                    println!("parameter {}",brief(&parameter));
                    let point=certified(curve.point_at(&parameter, &policy).unwrap());
                    println!("point {}",brief(&point));
                    println!("contact {}",brief(contact.point()));
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
