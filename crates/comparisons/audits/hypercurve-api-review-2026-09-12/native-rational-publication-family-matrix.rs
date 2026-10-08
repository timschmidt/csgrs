use hypercurve::*;
fn p(x: i32, y: i32) -> Point2 { Point2::from_values(x, y) }
fn q(n: i32, d: i32) -> Real { (Real::from(n) / Real::from(d)).unwrap() }
fn exact<T>(v: Classification<T>) -> T { match v { Classification::Decided(v) => v, Classification::Uncertain(e) => panic!("{e:?}") } }
fn certified<T>(v: CurveOutcome<T>) -> T { assert_eq!(v.certainty, CurveCertainty::Certified); v.value }
fn orient(c: &Curve2, reverse: bool, policy: &CurveContext) -> Curve2 { if reverse { certified(c.reversed(policy).unwrap()) } else { c.clone() } }
fn same(a: &CurvePoint2, b: &CurvePoint2, policy: &CurveContext) { assert!(exact(certified(a.coincides_with(b, policy)))); }
fn equal(a: &CurveParameter2, b: &CurveParameter2, policy: &CurveContext) -> bool { exact(certified(a.compare(b, policy).unwrap())).is_eq() }

fn retraces(policy: &CurveContext) -> Vec<Curve2> {
    let controls = vec![p(0, 0), p(2, 0), p(0, 0)];
    let knots = [2, 2, 2, 6, 6, 6].map(Real::from).to_vec();
    vec![
        QuadraticBezier2::new(p(0, 0), p(2, 0), p(0, 0)).into(),
        CubicBezier2::new(p(0, 0), Point2::new(q(4, 3), 0.into()), Point2::new(q(4, 3), 0.into()), p(0, 0)).into(),
        RationalQuadraticBezier2::try_new(p(0, 0), p(2, 0), p(0, 0), 3.into(), 3.into(), 3.into()).unwrap().into(),
        RationalBezier2::try_new(controls.clone(), vec![Real::one(); 3]).unwrap().elevated_to_degree(5).unwrap().into(),
        certified(Curve2::try_polynomial_bspline(2, controls.clone(), knots.clone(), policy).unwrap()),
        certified(Curve2::try_nurbs(2, controls, vec![3.into(); 3], knots, policy).unwrap()),
    ]
}

fn nodes(policy: &CurveContext) -> Vec<Curve2> {
    let controls = vec![p(12, -9), p(-4, 13), p(-4, -13), p(12, 9)];
    let knots = [2, 2, 2, 2, 6, 6, 6, 6].map(Real::from).to_vec();
    vec![
        CubicBezier2::new(controls[0].clone(), controls[1].clone(), controls[2].clone(), controls[3].clone()).into(),
        RationalBezier2::try_new(controls.clone(), vec![3.into(); 4]).unwrap().elevated_to_degree(5).unwrap().into(),
        certified(Curve2::try_polynomial_bspline(3, controls.clone(), knots.clone(), policy).unwrap()),
        certified(Curve2::try_nurbs(3, controls, vec![3.into(); 4], knots, policy).unwrap()),
    ]
}

fn main() {
    let (mut queries, mut restrictions, mut point_replays, mut path_queries) = (0, 0, 0, 0);
    for policy in [CurveContext::STRICT, CurveContext::APPROXIMATE_512] {
        let curves = retraces(&policy);
        for a in &curves { for b in &curves { for reverse_a in [false, true] { for reverse_b in [false, true] {
            let a = orient(a, reverse_a, &policy);
            let b = orient(b, reverse_b, &policy);
            let result = certified(a.intersect_curve(&b, &policy).unwrap());
            assert!(result.is_complete(), "{:?}", result.blockers());
            assert!(result.contacts().is_empty());
            queries += 1;
            let native_a = certified(a.native_bezier_fragments(&policy).unwrap())[0].clone().into_curve();
            let native_b = certified(b.native_bezier_fragments(&policy).unwrap())[0].clone().into_curve();
            let mut retained = Vec::new();
            for overlap in result.overlaps() {
                if let Some(overlap) = exact(certified(overlap.restrict(
                    [Real::zero().into(), q(1, 4).into()], [q(3, 4).into(), Real::one().into()], &policy,
                ).unwrap())) { retained.push(overlap); }
            }
            assert_eq!(retained.len(), 1);
            restrictions += 1;
            let mut overlap = retained.pop().unwrap();
            for _ in 0..4 {
                overlap = exact(certified(overlap.restrict(
                    [q(1, 8).into(), q(1, 4).into()], [q(3, 4).into(), q(7, 8).into()], &policy,
                ).unwrap())).unwrap();
                restrictions += 1;
                for (t, u) in [(overlap.first_range().start(), overlap.second_range().start()), (overlap.first_range().end(), overlap.second_range().end())] {
                    let p = certified(native_a.point_at(t, &policy).unwrap());
                    let q = certified(native_b.point_at(u, &policy).unwrap());
                    same(&p, &q, &policy);
                    point_replays += 2;
                }
            }
        }}}}
        let curves = nodes(&policy);
        for a in &curves { for b in &curves { for reverse_a in [false, true] { for reverse_b in [false, true] {
            let a = orient(a, reverse_a, &policy);
            let b = orient(b, reverse_b, &policy);
            let result = certified(a.intersect_curve(&b, &policy).unwrap());
            assert!(result.is_complete(), "{:?}", result.blockers());
            assert_eq!(result.contacts().len(), 2);
            assert_eq!(result.overlaps().len(), 1);
            queries += 1;
            for t in [q(1, 4), q(3, 4)] {
                let u = if reverse_a == reverse_b { Real::one() - &t } else { t.clone() };
                let contact = result.contacts().iter().find(|c| equal(c.first().local_parameter(), &t.clone().into(), &policy)).unwrap();
                assert!(equal(contact.second().local_parameter(), &u.into(), &policy));
                assert!(contact.is_certified_transverse());
                same(contact.point(), &p(3, 0).into(), &policy);
                for (curve, location) in [(&a, contact.first()), (&b, contact.second())] {
                    let parameter = exact(location.parameter(&policy).unwrap());
                    let point = certified(curve.point_at(&parameter, &policy).unwrap());
                    same(&point, contact.point(), &policy);
                    point_replays += 1;
                }
            }
        }}}}
        for curve in &curves {
            let path = CurvePath2::try_new(vec![curve.clone()]).unwrap();
            let result = certified(path.intersection_topology(&path, &policy).unwrap());
            assert!(result.result().is_complete());
            assert_eq!(result.result().contacts().len(), 2);
            assert_eq!(result.first()[0].curves().len(), 3);
            assert_eq!(result.second()[0].curves().len(), 3);
            path_queries += 1;
        }
    }
    println!("{{\"complete_queries\":{queries},\"restrictions\":{restrictions},\"point_replays\":{point_replays},\"path_topologies\":{path_queries}}}");
}
