use hypercurve::*;
fn r(v:i32)->Real { v.into() }
fn q(n:i32,d:i32)->Real { (r(n)/r(d)).unwrap() }
fn p(x:i32,y:i32)->Point2 {Point2::new(r(x),r(y))}
fn main() {
    let started=std::time::Instant::now();
    let source=RationalBezier2::try_new(vec![p(0,0),p(0,0),p(0,0),Point2::new(q(1,30),r(0)),Point2::new(q(2,15),q(1,10)),Point2::new(q(2,15),q(1,2))],vec![r(1);6]).unwrap();
    for policy in [CurveContext::STRICT] {
        println!("policy {policy:?}");
        for distance in [q(-1,20)] {
            let parallel=source.parallel_left(distance.clone()).unwrap();
            println!("parallel endpoint {:?}",parallel.point_at(&r(0),&policy));
            println!("global PH {:?}",parallel.exact_pythagorean_hodograph_offset(&policy).map(|v|v.map(|c|c.map(|c|c.curve().degree()))));
            let path=CurvePath2::try_new(vec![Curve2::from(source.clone()),Curve2::from(LineSeg2::try_new(source.end().clone(),Point2::new(r(-1),q(1,2))).unwrap()),Curve2::from(LineSeg2::try_new(Point2::new(r(-1),q(1,2)),p(-1,0)).unwrap()),Curve2::from(LineSeg2::try_new(p(-1,0),p(0,0)).unwrap())]).unwrap();
            let region=match CurveRegion2::try_from_boundary_paths_with_loop_semantics(&[path],&[CurveRegionLoopRole::Material],&[FillRule::NonZero],&policy) {
                Ok(region)=>{println!("admission {:?} {} loops",region.certainty,region.value.boundary_loops().len());region.into_value()},
                Err(e)=>{println!("admission error {e:?}");continue;}
            };
            let offset_start=std::time::Instant::now();
            match region.offset(distance.clone(),&OffsetCornerStyle2::Round,&policy) {
                Ok(result)=>{println!("offset {distance:?}: {:?}, {} loops",result.certainty,result.value.boundary_loops().len());println!("first offset {:?}",offset_start.elapsed()); println!("location {:?}",result.value.classify_point(&Point2::new(q(-1,2),q(1,4)),&policy)); for boundary in result.value.boundary_loops() { for f in boundary.fragments() {match f {
BezierSplitFragment2::Materialized{curve,..} => println!("materialized degree {}",match curve {BezierSubcurve2::Quadratic(_)|BezierSubcurve2::RationalQuadratic(_)=>2,BezierSubcurve2::Cubic(_)=>3,BezierSubcurve2::Rational(c)=>c.degree()}),
BezierSplitFragment2::AlgebraicEndpointImages{source_curve,..}=>println!("algebraic images degree {}",match source_curve {BezierSubcurve2::Quadratic(_)|BezierSubcurve2::RationalQuadratic(_)=>2,BezierSubcurve2::Cubic(_)=>3,BezierSubcurve2::Rational(c)=>c.degree()}),
BezierSplitFragment2::AnalyticParallel(p)=>println!("analytic parallel source degree {}",p.parallel().source_degree()),
BezierSplitFragment2::SelectedFiber(_)=>println!("selected fiber"),
BezierSplitFragment2::AlgebraicChord(_)=>println!("chord"),
BezierSplitFragment2::AlgebraicCuspSemicircle(_)=>println!("selected circle"),
}}} if std::env::var_os("PH_INSPECT_ONLY").is_some() { return; } TRACE_SECOND_OFFSET.store(true, std::sync::atomic::Ordering::Relaxed); let second_start=std::time::Instant::now(); let second=result.value.offset(q(-1,40),&OffsetCornerStyle2::Round,&policy); println!("second offset {:?}: {:?}",second_start.elapsed(),second.as_ref().map(|r|(r.certainty,r.value.boundary_loops().len()))); println!("total {:?}",started.elapsed());},
                Err(e)=>println!("offset {distance:?} error {e:?}"),
            }
        }
    }
}

static TRACE_SECOND_OFFSET: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
static TRACE_QUERY: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
static TRACE_PEAK: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
unsafe extern "Rust" {
    #[link_name = "__real__RNvNtCshMNGXr61hBB_10hypersolve9root_sign21sign_at_selected_root"]
    fn original_selected_sign(defining: &[Real], predicate: &[Real], interval: &hypersolve::IsolatedRootInterval) -> Option<std::cmp::Ordering>;
}
#[unsafe(export_name = "__wrap__RNvNtCshMNGXr61hBB_10hypersolve9root_sign21sign_at_selected_root")]
fn trace_selected_sign(defining: &[Real], predicate: &[Real], interval: &hypersolve::IsolatedRootInterval) -> Option<std::cmp::Ordering> {
    use std::sync::atomic::Ordering::Relaxed;
    if !TRACE_SECOND_OFFSET.load(Relaxed) {
        return unsafe { original_selected_sign(defining,predicate,interval) };
    }
    let id=TRACE_QUERY.fetch_add(1,Relaxed)+1;
    let max_bits=defining.iter().chain(predicate).filter_map(Real::exact_rational_ref)
        .map(|r| r.numerator().bits().max(r.denominator().bits())).max().unwrap_or(0);
    let peak=max_bits>=1024 && max_bits>TRACE_PEAK.fetch_max(max_bits,Relaxed);
    if peak {
        let mut dump=format!("query {id}\n");
        for (label,values) in [("defining",defining),("predicate",predicate),("interval",&[interval.lower.clone(),interval.upper.clone()][..])] {
            dump+=&format!("{label} {}\n",values.len());
            for value in values {
                if let Some(r)=value.exact_rational_ref() {
                    dump+=&format!("{:?} {} {}\n",r.sign(),r.numerator().to_str_radix(16),r.denominator().to_str_radix(16));
                } else { dump+="nonrational\n"; }
            }
        }
        std::fs::write(format!("stationary-ph-scalar-query-{id}.txt"),dump).unwrap();
        eprintln!("scalar_query_enter id={id} defining={} predicate={} max_bits={max_bits}",defining.len(),predicate.len());
    }
    let started=std::time::Instant::now();
    let result=unsafe {original_selected_sign(defining,predicate,interval)};
    let elapsed=started.elapsed();
    if peak || elapsed.as_millis()>=20 {
        eprintln!("scalar_query_return id={id} defining={} predicate={} max_bits={max_bits} elapsed={elapsed:?} result={result:?}",defining.len(),predicate.len());
    }
    result
}
