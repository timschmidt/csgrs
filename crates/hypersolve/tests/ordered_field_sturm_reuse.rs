//! Preparation counts distinguish arithmetic reuse from refinement budgets.
use hyperreal::Real;
use hypersolve::work_budget::{WorkBudget, with_budget};
use hypersolve::{
    OrderedFieldPolynomialContext, OrderedFieldRootIsolationStatus as Status,
    isolate_ordered_field_polynomial_roots_complete as isolate,
};
use std::{cell::Cell, cmp::Ordering};

#[derive(Default)]
struct Field {
    preparations: Cell<usize>,
    limit: Option<usize>,
}
impl OrderedFieldPolynomialContext<Real> for Field {
    type Error = ();
    fn constant(&mut self, x: &Real) -> Result<Real, ()> {
        Ok(x.clone())
    }
    fn add(&mut self, a: &Real, b: &Real) -> Result<Real, ()> {
        Ok(a + b)
    }
    fn multiply(&mut self, a: &Real, b: &Real) -> Result<Real, ()> {
        Ok(a * b)
    }
    fn scale(&mut self, a: &Real, b: &Real) -> Result<Real, ()> {
        Ok(a * b)
    }
    fn normalize_positive_scale(&mut self, _: &mut [Real]) {}
    fn sign(&mut self, x: &Real) -> Result<Ordering, ()> {
        x.partial_cmp(&Real::zero()).ok_or(())
    }
    fn sign_if_separated(&mut self, x: &Real) -> Result<Option<Ordering>, ()> {
        self.sign(x).map(Some)
    }
    fn remainder_elimination_budget(&self) -> Option<usize> {
        self.preparations.set(self.preparations.get() + 1);
        self.limit
    }
}
fn polynomial(c: &[i32]) -> Vec<Real> {
    c.iter().copied().map(Real::from).collect()
}
fn evaluate(c: &[Real], x: &Real) -> Real {
    c.iter().rev().fold(Real::zero(), |sum, c| sum * x + c)
}
#[test]
fn complete_isolation_prepares_one_chain_per_temporary_polynomial() {
    let mut samples = Vec::new();
    for (coefficients, lower, expected_roots, expected_chains) in [
        (vec![-2, 0, 1], 1, 1, 1),
        (vec![0, -2, 0, 1], -2, 3, 2),
        (vec![0, 0, 4, 0, -4, 0, 1], -2, 3, 2),
    ] {
        let original = polynomial(&coefficients);
        let mut field = Field::default();
        let result = isolate(original.clone(), &lower.into(), &2.into(), 8, &mut field).unwrap();
        assert_eq!(result.status, Status::Isolated);
        assert_eq!(result.intervals.len(), expected_roots);
        let preparations = field.preparations.get();
        for interval in &result.intervals {
            if let Some(root) = &interval.exact_root {
                assert_eq!(root, &Real::zero());
                assert!(evaluate(&original, root).definitely_zero());
            } else {
                assert!(!evaluate(&original, &interval.lower).definitely_zero());
                assert!(!evaluate(&original, &interval.upper).definitely_zero());
                let root = Real::from(2).sqrt().unwrap();
                let root = if interval.upper <= Real::zero() {
                    -root
                } else {
                    root
                };
                assert!(interval.lower < root && root < interval.upper);
            }
        }
        samples.push((
            coefficients.len() - 1,
            preparations,
            expected_chains,
            result.subdivision_steps,
        ));
    }
    eprintln!("complete isolation (degree, preparations, expected, subdivisions): {samples:?}");
    assert!(
        samples
            .iter()
            .all(|(_, actual, expected, _)| actual == expected),
        "each temporary polynomial needs one prepared chain: {samples:?}"
    );
}
#[test]
fn prepared_isolation_preserves_exhaustion_and_larger_retry() {
    let low = with_budget(
        WorkBudget {
            refinements: 2,
            promotions: 0,
        },
        || {
            let result = isolate(
                polynomial(&[-2, 0, 1]),
                &1.into(),
                &2.into(),
                8,
                &mut Field::default(),
            )
            .unwrap();
            assert_eq!(result.status, Status::BudgetExhausted);
            assert!(result.intervals.is_empty());
        },
    );
    assert!(low.result.is_err());
    assert_eq!(low.usage.refinements, 2);
    let retry = with_budget(
        WorkBudget {
            refinements: 8,
            promotions: 0,
        },
        || {
            isolate(
                polynomial(&[-2, 0, 1]),
                &1.into(),
                &2.into(),
                8,
                &mut Field::default(),
            )
            .unwrap()
        },
    );
    assert_eq!(retry.result.unwrap().status, Status::Isolated);
    assert_eq!(retry.usage.refinements, 8);
    assert_eq!(retry.usage.promotions, 0);
    let limited = with_budget(WorkBudget::ZERO, || {
        isolate(
            polynomial(&[-2, 0, 1]),
            &1.into(),
            &2.into(),
            8,
            &mut Field {
                limit: Some(0),
                ..Field::default()
            },
        )
        .unwrap()
    });
    let report = limited.result.unwrap();
    assert_eq!(report.status, Status::CompleteFallbackRequired);
    assert!(report.intervals.is_empty());
    assert_eq!(limited.usage.refinements, 0);
}
