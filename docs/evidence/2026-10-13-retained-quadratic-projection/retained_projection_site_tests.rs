use super::*;
use crate::retained_projection_diagnostic::Report;

fn raw(approximation: Approximation) -> Computable {
    Computable {
        internal: Arc::new(Node::new(
            approximation,
            BoundCache::Invalid,
            ExactSignCache::Unknown,
        )),
        signal: None,
    }
}

fn quadratic() -> Computable {
    raw(Approximation::Add(
        Computable::rational(Rational::new(5)),
        Computable::sqrt_rational(Rational::new(13)),
    ))
}

#[test]
fn missing_and_approximation_only_cells_are_distinguished() {
    for warm_approximation in [false, true] {
        let rational = Rational::fraction(17, 13).unwrap();
        let value = raw(Approximation::Ratio(rational.clone()));
        assert!(value.internal.cache.cell().is_none());
        if warm_approximation {
            let _ = value.approx(-8);
        }
        let before = value.cached();
        let report = Report::begin();
        assert_eq!(
            value.exact_quadratic_surd_parts(),
            Some((rational, Rational::zero(), None))
        );
        let actual = report.finish();
        assert_eq!(actual.queries, [1, 1, 0, 0, 0]);
        assert_eq!(
            actual.reads,
            if warm_approximation {
                [[0, 1, 0], [0, 0, 0]]
            } else {
                [[1, 0, 0], [0, 0, 0]]
            }
        );
        assert_eq!(actual.copies, [[[0; 4]; 6]; 2]);
        assert_eq!(value.cached(), before);
        assert_eq!(Arc::strong_count(&value.internal), 1);
    }
}

#[test]
fn retained_quadratic_root_copy_preserves_value_and_owners() {
    let value = quadratic();
    let clone = value.clone();
    let weak = Arc::downgrade(&value.internal);
    let expected = Some((Rational::new(5), Rational::one(), Some(Rational::new(13))));
    assert_eq!(value.exact_quadratic_surd_parts(), expected);
    assert!(value.internal.cache.pi_laurent_polynomial().is_some());
    let owners = Arc::strong_count(&value.internal);
    let report = Report::begin();
    assert_eq!(clone.exact_quadratic_surd_parts(), expected);
    let actual = report.finish();
    assert_eq!(actual.queries, [1, 1, 0, 0, 0]);
    assert_eq!(actual.reads, [[0, 0, 1], [0, 0, 0]]);
    assert_eq!(actual.copies[0][2], [1, 1, 1, 1]);
    assert_eq!(Arc::strong_count(&value.internal), owners);
    assert!(value.cached().is_none());
    drop(value);
    drop(clone);
    assert!(weak.upgrade().is_none());
}

#[test]
fn retained_nonquadratic_root_declines_and_runs_original_retry() {
    let value = raw(Approximation::Add(
        Computable::sqrt_rational(Rational::new(13)),
        Computable::sqrt_rational(Rational::new(17)),
    ));
    let clone = value.clone();
    let weak = Arc::downgrade(&value.internal);
    assert!(value.exact_quadratic_surd_parts().is_none());
    assert!(value.internal.cache.pi_laurent_polynomial().is_some());
    let owners = Arc::strong_count(&value.internal);
    let report = Report::begin();
    assert!(clone.exact_quadratic_surd_parts().is_none());
    let actual = report.finish();
    assert_eq!(actual.queries, [1, 0, 1, 0, 1]);
    assert_eq!(actual.reads, [[0, 0, 1], [0, 0, 0]]);
    assert_eq!(actual.copies[0][3], [1, 1, 2, 1]);
    assert_eq!(Arc::strong_count(&value.internal), owners);
    assert!(value.cached().is_none());
    drop(value);
    drop(clone);
    assert!(weak.upgrade().is_none());
}

#[test]
fn recursive_copy_is_separate_from_the_missing_root_cell() {
    let child = quadratic();
    let clone = child.clone();
    assert_eq!(
        child.exact_quadratic_surd_parts(),
        Some((Rational::new(5), Rational::one(), Some(Rational::new(13))))
    );
    let root = raw(Approximation::Add(
        raw(Approximation::Ratio(Rational::fraction(7, 19).unwrap())),
        child,
    ));
    assert!(root.internal.cache.cell().is_none());
    let child_owners = Arc::strong_count(&clone.internal);
    let report = Report::begin();
    assert_eq!(
        root.exact_quadratic_surd_parts(),
        Some((
            Rational::fraction(102, 19).unwrap(),
            Rational::one(),
            Some(Rational::new(13))
        ))
    );
    let actual = report.finish();
    assert_eq!(actual.queries, [1, 1, 0, 0, 0]);
    assert_eq!(actual.reads, [[1, 0, 0], [1, 0, 1]]);
    assert_eq!(actual.copies[1][2], [1, 1, 1, 1]);
    assert_eq!(actual.copies[0], [[0; 4]; 6]);
    assert_eq!(Arc::strong_count(&root.internal), 1);
    assert_eq!(Arc::strong_count(&clone.internal), child_owners);
    assert!(root.cached().is_none());
}

#[test]
fn reads_outside_quadratic_projection_are_not_counted() {
    let value = quadratic();
    let _clone = value.clone();
    assert!(value.exact_quadratic_surd_parts().is_some());
    let report = Report::begin();
    assert!(value.internal.cache.pi_laurent_polynomial().is_some());
    let actual = report.finish();
    assert_eq!(actual.queries, [0; 5]);
    assert_eq!(actual.reads, [[0; 3]; 2]);
    assert_eq!(actual.copies, [[[0; 4]; 6]; 2]);
}
