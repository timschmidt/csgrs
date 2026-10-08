use hypercurve::{Classification, CurveContext, CurveError, CurveRegion2, ExactCurveError};
fn main() {
    let mut rejected = 0;
    for (index, policy) in [CurveContext::STRICT, CurveContext::APPROXIMATE_512].iter().enumerate() {
        match CurveRegion2::try_from_native_boundary_contours(Vec::new(), policy) {
            Err(ExactCurveError::Invalid { cause: CurveError::Topology(_), .. }) => {
                rejected += 1;
                println!("policy={index} status=invalid-topology");
            }
            Ok(outcome) => match outcome.into_value() {
                Classification::Decided(region) => println!("policy={index} status=decided empty={}", region.is_empty()),
                Classification::Uncertain(_) => println!("policy={index} status=uncertain"),
            },
            Err(_) => println!("policy={index} status=other-error"),
        }
    }
    println!("verified_rejections={rejected}");
    assert_eq!(rejected, 2, "baseline did not reproduce the suspected empty input rejection");
}
