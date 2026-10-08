use hypercurve::{CurveContext, CurveError, CurveRegion2, ExactCurveError, FillRule};
fn main() {
    let mut rejected = 0;
    for (policy_index, policy) in [CurveContext::STRICT, CurveContext::APPROXIMATE_512].iter().enumerate() {
        for (rule_index, rule) in [FillRule::EvenOdd, FillRule::NonZero].iter().enumerate() {
            match CurveRegion2::arrange_unordered_segments(&[], *rule, policy) {
                Err(ExactCurveError::Invalid { cause: CurveError::EmptyCurveString, .. }) => {
                    rejected += 1;
                    println!("policy={policy_index} rule={rule_index} status=invalid-empty-curve");
                }
                Ok(result) => println!("policy={policy_index} rule={rule_index} status=arranged has_region={}", result.value.region().is_some()),
                Err(_) => println!("policy={policy_index} rule={rule_index} status=other-error"),
            }
        }
    }
    println!("verified_rejections={rejected}");
    assert_eq!(rejected, 4, "baseline did not reproduce empty arrangement rejection");
}
