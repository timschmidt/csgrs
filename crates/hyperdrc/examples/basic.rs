use hyperdrc::{Check, CheckExecutionStatus, CheckRunDisposition, ReadinessRunner};

fn main() {
    let runner = ReadinessRunner::new([Check::StackupReadiness, Check::TestpointCoverageReadiness]);
    let mut findings = Vec::new();
    let coverage = runner
        .run(&mut findings, |check, _| {
            Ok::<_, std::convert::Infallible>(match check {
                Check::StackupReadiness => CheckRunDisposition::Executed,
                _ => {
                    CheckRunDisposition::NotApplicable("design declares no testpoint intent".into())
                }
            })
        })
        .expect("infallible readiness adapter");

    assert!(coverage.is_complete());
    assert_eq!(coverage.checks[0].status, CheckExecutionStatus::Passed);
    assert_eq!(
        coverage.checks[1].status,
        CheckExecutionStatus::NotApplicable
    );
}
