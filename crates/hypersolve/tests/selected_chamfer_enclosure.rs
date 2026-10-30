//! The original second-chamfer scalar replays without coordinate projection.
#[path = "support/selected_polynomial_replay.rs"]
#[allow(dead_code)]
mod codec;
#[path = "support/recursive_quadratic.rs"]
#[allow(dead_code)]
mod support;
use hyperreal::{Real, RealSign};
use hypersolve::classification::Classification;
use hypersolve::work_budget::{WorkBudget, with_budget};

fn decided<T: std::fmt::Debug>(value: Classification<T>) -> T {
    match value {
        Classification::Decided(value) => value,
        other => panic!("expected exact completion: {other:?}"),
    }
}
#[test]
fn second_chamfer_scalar_enclosure_keeps_its_selected_graph() {
    let graph = codec::decode(include_str!("fixtures/selected_chamfer_enclosure.json"));
    let policy = support::TestContext::approximate();
    let values = decided(graph.replay(10000, &policy).unwrap());
    assert_eq!(values.len(), 1);
    let value = &values[0];
    let refused = with_budget(WorkBudget::ZERO, || value.enclosed(&policy).unwrap());
    assert!(refused.result.is_err());
    let retry = with_budget(
        WorkBudget {
            refinements: 10000,
            promotions: 0,
        },
        || {
            let enclosure = decided(value.enclosed(&policy).unwrap());
            assert_eq!(enclosure.interval().lower, Real::from(-1));
            assert_eq!(enclosure.interval().upper, Real::zero());
            let refined = decided(enclosure.refined(4, &policy).unwrap());
            // An independent exact rational interval replay of the original
            // native cubic and selected degrees 2/6/6 places this value here.
            assert_eq!(refined.interval().lower, support::fraction(-15, 16));
            assert_eq!(refined.interval().upper, support::fraction(-7, 8));
            assert!(refined.value().field().shares_authority(value.field()));
            assert_eq!(decided(value.sign(&policy).unwrap()), RealSign::Negative);
            assert_eq!(
                decided(
                    value
                        .add(&value.field().constant(Real::one()).unwrap())
                        .unwrap()
                        .sign(&policy)
                        .unwrap()
                ),
                RealSign::Positive
            );
        },
    );
    assert!(retry.result.is_ok());
    assert_eq!(policy.approximations(), 0);
    const CHILD: &str = "HYPERSOLVE_CHAMFER_ENCLOSURE_CHILD";
    if std::env::var_os(CHILD).is_none() {
        let child = std::process::Command::new(std::env::current_exe().unwrap())
            .args([
                "--exact",
                "second_chamfer_scalar_enclosure_keeps_its_selected_graph",
                "--nocapture",
            ])
            .env(CHILD, "1")
            .output()
            .unwrap();
        assert!(
            child.status.success(),
            "{}{}",
            String::from_utf8_lossy(&child.stdout),
            String::from_utf8_lossy(&child.stderr)
        );
    }
}
