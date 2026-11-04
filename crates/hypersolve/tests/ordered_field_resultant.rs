//! Elimination must retain selected coefficient fields for its next consumer.
#[path = "support/recursive_quadratic.rs"]
#[allow(dead_code)]
mod support;
use hyperreal::{Real, RealSign};
use hypersolve::classification::Classification;
use hypersolve::recursive_quadratic_field::RecursiveQuadraticField;
use hypersolve::selected_polynomial_field::{SelectedPolynomialContext, SelectedPolynomialField};
use hypersolve::{
    CurveResultantParameter, OrderedFieldRootIsolationStatus,
    isolate_ordered_field_polynomial_roots_complete, ordered_field_resultant_bivariate,
};
use support::{TestContext, constant_tensor};

fn decided<T: std::fmt::Debug>(value: Classification<T>) -> T {
    match value {
        Classification::Decided(value) => value,
        other => panic!("exact result: {other:?}"),
    }
}
fn base() -> SelectedPolynomialField {
    SelectedPolynomialField::from_recursive(
        RecursiveQuadraticField::base(
            vec![],
            constant_tensor(0, Real::one()),
            constant_tensor(0, Real::one()),
        )
        .unwrap(),
    )
}
#[test]
fn selected_quintic_coefficients_feed_elimination_and_another_selection() {
    for policy in [TestContext::strict(), TestContext::approximate()] {
        let q = base();
        let field = decided(
            q.select(
                [5, 5, 0, 0, 0, 1]
                    .into_iter()
                    .map(|n| q.constant(n.into()).unwrap())
                    .collect(),
                (-1).into(),
                0.into(),
                &policy,
            )
            .unwrap(),
        );
        let a = field.generator().unwrap();
        let b = a.add(&field.constant(1.into()).unwrap()).unwrap();
        let c = |n: i32| field.constant(n.into()).unwrap();
        // F=t-s^2, G=t-(a+1). Elimination is exactly s^2-(a+1),
        // including its scale. It has two real roots, one in each half.
        let first = vec![vec![c(0), c(1)], vec![], vec![c(-1)]];
        let second = vec![vec![b.scale(&(-1).into()).unwrap(), c(1)]];
        let mut context = SelectedPolynomialContext {
            field: field.clone(),
            policy: &policy,
        };
        let result = ordered_field_resultant_bivariate(
            &first,
            &second,
            CurveResultantParameter::First,
            &mut context,
        )
        .unwrap();
        assert_eq!(result.len(), 3);
        assert_eq!(
            decided(result[0].add(&b).unwrap().sign(&policy).unwrap()),
            RealSign::Zero
        );
        assert_eq!(decided(result[1].sign(&policy).unwrap()), RealSign::Zero);
        assert_eq!(
            decided(result[2].subtract(&c(1)).unwrap().sign(&policy).unwrap()),
            RealSign::Zero
        );
        let report = isolate_ordered_field_polynomial_roots_complete(
            result.clone(),
            &(-1).into(),
            &1.into(),
            4,
            &mut context,
        )
        .unwrap();
        assert_eq!(report.status, OrderedFieldRootIsolationStatus::Isolated);
        assert_eq!(report.intervals.len(), 2);
        let root = decided(
            field
                .select(
                    result,
                    report.intervals[1].lower.clone(),
                    report.intervals[1].upper.clone(),
                    &policy,
                )
                .unwrap(),
        );
        let s = root.generator().unwrap();
        let lifted = root.lift(&b).unwrap();
        assert_eq!(
            decided(
                s.multiply(&s)
                    .unwrap()
                    .subtract(&lifted)
                    .unwrap()
                    .sign(&policy)
                    .unwrap()
            ),
            RealSign::Zero
        );
        assert_eq!(policy.approximations(), 0);
    }
}

use hypersolve::selected_polynomial_field::SelectedPolynomialValue as Value;
fn same(a: &Value, b: &Value, policy: &TestContext) {
    assert_eq!(
        decided(a.subtract(b).unwrap().sign(policy).unwrap()),
        RealSign::Zero
    );
}
fn product(a: &[Value], b: &[Value], field: &SelectedPolynomialField) -> Vec<Value> {
    let mut result = vec![field.constant(0.into()).unwrap(); a.len() + b.len() - 1];
    for (i, a) in a.iter().enumerate() {
        for (j, b) in b.iter().enumerate() {
            result[i + j] = result[i + j].add(&a.multiply(b).unwrap()).unwrap();
        }
    }
    result
}
fn power(value: &Value, n: usize) -> Value {
    (0..n).fold(value.field().constant(1.into()).unwrap(), |a, _| {
        a.multiply(value).unwrap()
    })
}
fn selected(policy: &TestContext) -> (SelectedPolynomialField, Value) {
    let q = base();
    let field = decided(
        q.select(
            [5, 5, 0, 0, 0, 1]
                .into_iter()
                .map(|n| q.constant(n.into()).unwrap())
                .collect(),
            (-1).into(),
            0.into(),
            policy,
        )
        .unwrap(),
    );
    let a = field.generator().unwrap();
    (field, a)
}
fn factor_product(roots: &[Vec<Value>], field: &SelectedPolynomialField) -> Vec<Vec<Value>> {
    let zero = || field.constant(0.into()).unwrap();
    let mut result = vec![vec![field.constant(1.into()).unwrap()]];
    // Test-side multiplication of the explicit factored equations, with
    // independent pairwise-difference products as the resultant oracle.
    for root in roots {
        let mut next = vec![
            vec![zero(); result.iter().map(Vec::len).max().unwrap() + 1];
            result.len() + root.len() - 1
        ];
        for (i, row) in result.iter().enumerate() {
            for (j, c) in row.iter().enumerate() {
                next[i][j + 1] = next[i][j + 1].add(c).unwrap();
                for (k, r) in root.iter().enumerate() {
                    next[i + k][j] = next[i + k][j].subtract(&c.multiply(r).unwrap()).unwrap();
                }
            }
        }
        result = next;
    }
    result
}
#[test]
fn factored_systems_keep_exact_scales_and_dependent_coefficient_fields() {
    let policy = TestContext::strict();
    let (parent, a) = selected(&policy);
    let one = parent.constant(1.into()).unwrap();
    let b = a.add(&one).unwrap();
    let coefficients = std::iter::once(b.scale(&(-1).into()).unwrap())
        .chain((1..7).map(|_| parent.constant(0.into()).unwrap()))
        .chain(std::iter::once(one))
        .collect();
    let field = decided(
        parent
            .select(coefficients, 0.into(), 1.into(), &policy)
            .unwrap(),
    );
    let (a, b) = (field.lift(&a).unwrap(), field.generator().unwrap());
    let c = |n: i32| field.constant(n.into()).unwrap();
    for (m, n) in [(1, 1), (2, 1), (1, 3), (2, 2), (3, 2), (3, 3)] {
        let f = (0..m)
            .map(|i| vec![a.scale(&Real::from(i + 1)).unwrap(), b.clone()])
            .collect::<Vec<_>>();
        let g = (0..n)
            .map(|j| {
                vec![
                    b.scale(&Real::from(j + 1)).unwrap().add(&c(1)).unwrap(),
                    a.clone(),
                ]
            })
            .collect::<Vec<_>>();
        let mut first = factor_product(&f, &field);
        let mut second = factor_product(&g, &field);
        let two_thirds = (Real::from(2) / Real::from(3)).unwrap();
        for c in first.iter_mut().flatten() {
            *c = c.scale(&two_thirds).unwrap();
        }
        for c in second.iter_mut().flatten() {
            *c = c.scale(&Real::from(-5)).unwrap();
        }
        let mut expected = vec![c(1)];
        for f in &f {
            for g in &g {
                expected = product(
                    &expected,
                    &[f[0].subtract(&g[0]).unwrap(), f[1].subtract(&g[1]).unwrap()],
                    &field,
                );
            }
        }
        let scale = two_thirds.powi(n.into()).unwrap() * Real::from(-5).powi(m.into()).unwrap();
        for c in &mut expected {
            *c = c.scale(&scale).unwrap();
        }
        let mut context = SelectedPolynomialContext {
            field: field.clone(),
            policy: &policy,
        };
        let actual = ordered_field_resultant_bivariate(
            &first,
            &second,
            CurveResultantParameter::First,
            &mut context,
        )
        .unwrap();
        assert_eq!(actual.len(), expected.len());
        for (a, b) in actual.iter().zip(&expected) {
            same(a, b, &policy);
        }
        let transposed = |p: &[Vec<Value>]| {
            (0..p.iter().map(Vec::len).max().unwrap())
                .map(|j| {
                    p.iter()
                        .map(|r| r.get(j).cloned().unwrap_or_else(|| c(0)))
                        .collect()
                })
                .collect::<Vec<Vec<Value>>>()
        };
        let second_axis = ordered_field_resultant_bivariate(
            &transposed(&first),
            &transposed(&second),
            CurveResultantParameter::Second,
            &mut context,
        )
        .unwrap();
        for (a, b) in second_axis.iter().zip(&expected) {
            same(a, b, &policy);
        }
        assert_eq!(second_axis.len(), expected.len());
    }
}
#[test]
fn degree_seventeen_supports_do_not_require_scalar_projection() {
    let policy = TestContext::strict();
    let (field, a) = selected(&policy);
    let b = a.add(&field.constant(1.into()).unwrap()).unwrap();
    let c = |n: i32| field.constant(n.into()).unwrap();
    let mut first = vec![vec![c(0); 18], vec![], vec![], vec![c(-1)]];
    first[0][17] = c(1);
    let second = vec![vec![b.scale(&(-1).into()).unwrap(), c(1)]];
    let mut context = SelectedPolynomialContext {
        field: field.clone(),
        policy: &policy,
    };
    let actual = ordered_field_resultant_bivariate(
        &first,
        &second,
        CurveResultantParameter::First,
        &mut context,
    )
    .unwrap();
    assert_eq!(actual.len(), 4);
    same(
        &actual[0],
        &power(&b, 17).scale(&(-1).into()).unwrap(),
        &policy,
    );
    same(&actual[1], &c(0), &policy);
    same(&actual[2], &c(0), &policy);
    same(&actual[3], &c(1), &policy);
}
#[test]
fn zero_constants_common_components_and_cancelled_leading_terms_are_exact() {
    let policy = TestContext::strict();
    let (field, a) = selected(&policy);
    let c = |n: i32| field.constant(n.into()).unwrap();
    let relation = power(&a, 5)
        .add(&a.scale(&5.into()).unwrap())
        .unwrap()
        .add(&c(5))
        .unwrap();
    same(&relation, &c(0), &policy);
    let mut context = SelectedPolynomialContext {
        field: field.clone(),
        policy: &policy,
    };
    for (f, g, expected) in [
        (vec![], vec![vec![a.clone()]], vec![c(0)]),
        (vec![vec![c(2)]], vec![vec![c(3)]], vec![c(1)]),
        (vec![vec![c(2)]], vec![vec![c(1), c(0), c(1)]], vec![c(4)]),
        (
            vec![vec![c(0), c(1)], vec![c(-1)]],
            vec![vec![c(0), c(1)], vec![c(-1)]],
            vec![c(0)],
        ),
        (
            vec![vec![a.scale(&(-1).into()).unwrap(), c(1), relation.clone()]],
            vec![vec![c(0), c(1)]],
            vec![a.clone()],
        ),
    ] {
        let actual =
            ordered_field_resultant_bivariate(&f, &g, CurveResultantParameter::First, &mut context)
                .unwrap();
        assert_eq!(actual.len(), expected.len());
        for (a, b) in actual.iter().zip(expected.iter()) {
            same(a, b, &policy);
        }
    }
}

use hypersolve::selected_polynomial_field::SelectedPolynomialError;
use hypersolve::work_budget::{
    CancellationToken, WorkBudget, WorkKind, with_budget, with_cancellable_budget,
};
use hypersolve::{OrderedFieldPolynomialContext, OrderedFieldResultantError};
use std::{cell::Cell, cmp::Ordering};
struct CancellingContext<'a> {
    inner: SelectedPolynomialContext<'a, TestContext>,
    token: CancellationToken,
    at_zero: bool,
}
impl OrderedFieldPolynomialContext<Value> for CancellingContext<'_> {
    type Error = SelectedPolynomialError<support::TestError>;
    fn constant(&mut self, v: &Real) -> Result<Value, Self::Error> {
        let value = self.inner.constant(v)?;
        if self.at_zero && v.definitely_zero() {
            self.token.cancel();
        }
        Ok(value)
    }
    fn add(&mut self, a: &Value, b: &Value) -> Result<Value, Self::Error> {
        self.inner.add(a, b)
    }
    fn multiply(&mut self, a: &Value, b: &Value) -> Result<Value, Self::Error> {
        let v = self.inner.multiply(a, b)?;
        if !self.at_zero {
            self.token.cancel();
        }
        Ok(v)
    }
    fn scale(&mut self, a: &Value, b: &Real) -> Result<Value, Self::Error> {
        self.inner.scale(a, b)
    }
    fn normalize_positive_scale(&mut self, v: &mut [Value]) {
        self.inner.normalize_positive_scale(v);
    }
    fn sign(&mut self, v: &Value) -> Result<Ordering, Self::Error> {
        self.inner.sign(v)
    }
    fn sign_if_separated(&mut self, v: &Value) -> Result<Option<Ordering>, Self::Error> {
        self.inner.sign_if_separated(v)
    }
}
#[test]
fn cancellation_during_the_last_zero_construction_does_not_publish_success() {
    let policy = TestContext::strict();
    let field = base();
    let token = CancellationToken::new();
    let mut context = CancellingContext {
        inner: SelectedPolynomialContext {
            field,
            policy: &policy,
        },
        token: token.clone(),
        at_zero: true,
    };
    let declined = Cell::new(false);
    let outcome = with_cancellable_budget(WorkBudget::UNLIMITED, &token, || {
        let result = ordered_field_resultant_bivariate(
            &[],
            &[],
            CurveResultantParameter::First,
            &mut context,
        );
        declined.set(matches!(
            result,
            Err(OrderedFieldResultantError::BudgetExhausted)
        ));
        result
    });
    assert_eq!(outcome.result.unwrap_err().kind, WorkKind::Cancellation);
    assert!(
        declined.get(),
        "a completed zero cannot escape after cancellation is observed in its construction"
    );
}
#[test]
fn actual_entry_denial_mid_computation_cancellation_and_retry() {
    let policy = TestContext::strict();
    let (field, a) = selected(&policy);
    let c = |n: i32| field.constant(n.into()).unwrap();
    let first = vec![vec![c(0), c(1)], vec![], vec![c(-1)]];
    let second = vec![vec![a.scale(&(-1).into()).unwrap(), c(1)]];
    let mut context = SelectedPolynomialContext {
        field: field.clone(),
        policy: &policy,
    };
    let outcome = with_budget(
        WorkBudget {
            refinements: 0,
            promotions: 0,
        },
        || {
            ordered_field_resultant_bivariate(
                &first,
                &second,
                CurveResultantParameter::First,
                &mut context,
            )
        },
    );
    let receipt = outcome.result.unwrap_err();
    assert_eq!(receipt.kind, WorkKind::AlgebraicPromotion);
    assert_eq!((receipt.used, receipt.limit, receipt.requested), (0, 0, 1));
    for precancelled in [false, true] {
        let token = CancellationToken::new();
        if precancelled {
            token.cancel();
        }
        let mut context = CancellingContext {
            inner: SelectedPolynomialContext {
                field: field.clone(),
                policy: &policy,
            },
            token: token.clone(),
            at_zero: false,
        };
        let declined = Cell::new(false);
        let outcome = with_cancellable_budget(WorkBudget::UNLIMITED, &token, || {
            let value = ordered_field_resultant_bivariate(
                &first,
                &second,
                CurveResultantParameter::First,
                &mut context,
            );
            declined.set(matches!(
                value,
                Err(OrderedFieldResultantError::BudgetExhausted)
            ));
            value
        });
        assert_eq!(outcome.result.unwrap_err().kind, WorkKind::Cancellation);
        assert!(declined.get());
    }
    let result = ordered_field_resultant_bivariate(
        &first,
        &second,
        CurveResultantParameter::First,
        &mut context,
    )
    .unwrap();
    assert_eq!(result.len(), 3);
    same(&result[0], &a.scale(&(-1).into()).unwrap(), &policy);
    same(&result[2], &c(1), &policy);
}
#[test]
fn an_empty_operand_does_not_hide_an_invalid_coefficient_field() {
    let policy = TestContext::strict();
    let (_, a) = selected(&policy);
    let mut context = SelectedPolynomialContext {
        field: base(),
        policy: &policy,
    };
    let result = ordered_field_resultant_bivariate(
        &[],
        &[vec![a]],
        CurveResultantParameter::First,
        &mut context,
    );
    assert!(matches!(
        result,
        Err(OrderedFieldResultantError::Context(_))
    ));
}

#[cfg(not(target_arch = "wasm32"))]
#[path = "support/selected_polynomial_replay.rs"]
mod codec;
#[cfg(not(target_arch = "wasm32"))]
#[test]
fn eliminated_coefficients_replay_cold_and_reselect_their_dependent_root() {
    use hypersolve::selected_polynomial_field::SelectedPolynomialDefinition;
    const INPUT: &str = "HYPERSOLVE_FIELD_RESULTANT_REPLAY";
    fn verify(wire: &str) {
        let policy = TestContext::strict();
        let definition = codec::decode(wire);
        let values = decided(definition.replay(10000, &policy).unwrap());
        let [a, b, p0, p1, p2, s] = values.as_slice() else {
            panic!("six retained field values")
        };
        let field = s.field();
        let c = |n: i32| field.constant(n.into()).unwrap();
        let a = field.lift(a).unwrap();
        let b = field.lift(b).unwrap();
        same(
            &power(&a, 5)
                .add(&a.scale(&5.into()).unwrap())
                .unwrap()
                .add(&c(5))
                .unwrap(),
            &c(0),
            &policy,
        );
        same(&b.subtract(&a).unwrap(), &c(1), &policy);
        same(&s.multiply(s).unwrap(), &b, &policy);
        assert_eq!(decided(s.sign(&policy).unwrap()), RealSign::Positive);
        let first = vec![vec![c(0), c(1)], vec![], vec![c(-1)]];
        let second = vec![vec![b.scale(&(-1).into()).unwrap(), c(1)]];
        let mut context = SelectedPolynomialContext {
            field: field.clone(),
            policy: &policy,
        };
        let result = ordered_field_resultant_bivariate(
            &first,
            &second,
            CurveResultantParameter::First,
            &mut context,
        )
        .unwrap();
        for (actual, expected) in result.iter().zip([p0, p1, p2]) {
            same(actual, expected, &policy);
        }
        let report = isolate_ordered_field_polynomial_roots_complete(
            result,
            &(-1).into(),
            &1.into(),
            4,
            &mut context,
        )
        .unwrap();
        assert_eq!(report.status, OrderedFieldRootIsolationStatus::Isolated);
        assert_eq!(report.intervals.len(), 2);
        assert_eq!(codec::encode(&definition), wire);
    }
    if let Some(path) = std::env::var_os(INPUT) {
        verify(&std::fs::read_to_string(path).unwrap());
        return;
    }
    let policy = TestContext::strict();
    let (field, a) = selected(&policy);
    let c = |n: i32| field.constant(n.into()).unwrap();
    let b = a.add(&c(1)).unwrap();
    let first = vec![vec![c(0), c(1)], vec![], vec![c(-1)]];
    let second = vec![vec![b.scale(&(-1).into()).unwrap(), c(1)]];
    let mut context = SelectedPolynomialContext {
        field: field.clone(),
        policy: &policy,
    };
    let result = ordered_field_resultant_bivariate(
        &first,
        &second,
        CurveResultantParameter::First,
        &mut context,
    )
    .unwrap();
    let root = decided(
        field
            .select(result.clone(), 0.into(), 1.into(), &policy)
            .unwrap(),
    )
    .generator()
    .unwrap();
    let definition = SelectedPolynomialDefinition::from_values(
        &[
            a,
            b,
            result[0].clone(),
            result[1].clone(),
            result[2].clone(),
            root,
        ],
        10000,
    )
    .unwrap();
    let wire = codec::encode(&definition);
    verify(&wire);
    let path = std::env::temp_dir().join(format!(
        "hypersolve-field-resultant-{}.json",
        std::process::id()
    ));
    std::fs::write(&path, &wire).unwrap();
    let child = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "eliminated_coefficients_replay_cold_and_reselect_their_dependent_root",
            "--nocapture",
        ])
        .env(INPUT, &path)
        .output()
        .unwrap();
    std::fs::remove_file(path).unwrap();
    assert!(
        child.status.success(),
        "{}{}",
        String::from_utf8_lossy(&child.stdout),
        String::from_utf8_lossy(&child.stderr)
    );
}

#[test]
fn repeated_selected_definitions_trim_zero_leading_coefficients_by_value() {
    let policy = TestContext::strict();
    let q = base();
    // -3(a^5+5a+5)^2 has one selected distinct root in (-1,0).
    // Its degree-5 factor is zero in value, without assuming structural zero.
    let field = decided(
        q.select(
            [-75, -150, -75, 0, 0, -30, -30, 0, 0, 0, -3]
                .into_iter()
                .map(|n| q.constant(n.into()).unwrap())
                .collect(),
            (-1).into(),
            0.into(),
            &policy,
        )
        .unwrap(),
    );
    let c = |n: i32| field.constant(n.into()).unwrap();
    let a = field.generator().unwrap();
    let relation = power(&a, 5)
        .add(&a.scale(&5.into()).unwrap())
        .unwrap()
        .add(&c(5))
        .unwrap();
    same(&relation, &c(0), &policy);
    let first = vec![vec![c(0), c(0), c(1)], vec![c(-1)]];
    let mut second = vec![vec![c(0); 10]];
    second[0][0] = a.scale(&(-1).into()).unwrap();
    second[0][1] = c(1);
    second[0][9] = relation;
    let mut context = SelectedPolynomialContext {
        field: field.clone(),
        policy: &policy,
    };
    let result = ordered_field_resultant_bivariate(
        &first,
        &second,
        CurveResultantParameter::First,
        &mut context,
    )
    .unwrap();
    assert_eq!(result.len(), 2);
    same(&result[0], &power(&a, 2), &policy);
    same(&result[1], &c(-1), &policy);
    assert_eq!(field.defining_polynomial().unwrap().len(), 11);
}
#[test]
fn zero_resultants_do_not_assert_real_components_or_finite_root_sets() {
    let policy = TestContext::strict();
    let field = base();
    let c = |n: i32| field.constant(n.into()).unwrap();
    // This common complex support has no real point: s^2+t^2+1>0.
    let first = vec![vec![c(1), c(0), c(1)], vec![], vec![c(1)]];
    let mut context = SelectedPolynomialContext {
        field: field.clone(),
        policy: &policy,
    };
    let result = ordered_field_resultant_bivariate(
        &first,
        &first,
        CurveResultantParameter::First,
        &mut context,
    )
    .unwrap();
    assert_eq!(result.len(), 1);
    same(&result[0], &c(0), &policy);
    let report = isolate_ordered_field_polynomial_roots_complete(
        result,
        &(-1).into(),
        &1.into(),
        0,
        &mut context,
    )
    .unwrap();
    assert_eq!(
        report.status,
        OrderedFieldRootIsolationStatus::IdenticallyZero
    );
    assert!(report.intervals.is_empty());
}
#[test]
fn independent_coefficient_fields_join_without_coordinate_materialization() {
    let policy = TestContext::strict();
    let q = base();
    let k = decided(
        q.select(
            [-2, 0, 0, 0, 0, 1]
                .into_iter()
                .map(|n| q.constant(n.into()).unwrap())
                .collect(),
            1.into(),
            2.into(),
            &policy,
        )
        .unwrap(),
    );
    let l = decided(
        q.select(
            [-3, 0, 0, 0, 0, 0, 0, 1]
                .into_iter()
                .map(|n| q.constant(n.into()).unwrap())
                .collect(),
            1.into(),
            2.into(),
            &policy,
        )
        .unwrap(),
    );
    let a = k.generator().unwrap();
    let b = l.generator().unwrap();
    let join = decided(k.joined_with(&l, &policy).unwrap());
    let field = join.field();
    let a = join.embed(&a).unwrap();
    let b = join.embed(&b).unwrap();
    let sum = a.add(&b).unwrap();
    let c = |n: i32| field.constant(n.into()).unwrap();
    // Both Eisenstein degrees are coprime, so the joint field has degree 35.
    let first = vec![vec![c(0), c(1)], vec![c(-1)]];
    let second = vec![vec![sum.scale(&(-1).into()).unwrap(), c(1)]];
    let mut context = SelectedPolynomialContext {
        field: field.clone(),
        policy: &policy,
    };
    let result = ordered_field_resultant_bivariate(
        &first,
        &second,
        CurveResultantParameter::First,
        &mut context,
    )
    .unwrap();
    assert_eq!(result.len(), 2);
    same(&result[0], &sum.scale(&(-1).into()).unwrap(), &policy);
    same(&result[1], &c(1), &policy);
    let root = decided(field.select(result, 2.into(), 4.into(), &policy).unwrap())
        .generator()
        .unwrap();
    same(&root, &root.field().lift(&sum).unwrap(), &policy);
}

struct CountedRationalField {
    calls: usize,
    limit: usize,
}
impl CountedRationalField {
    fn tick(&mut self) -> Result<(), &'static str> {
        if self.calls == self.limit {
            return Err("declared coefficient callback allowance reached");
        }
        self.calls += 1;
        Ok(())
    }
}
impl OrderedFieldPolynomialContext<Real> for CountedRationalField {
    type Error = &'static str;
    fn constant(&mut self, v: &Real) -> Result<Real, Self::Error> {
        self.tick()?;
        Ok(v.clone())
    }
    fn add(&mut self, a: &Real, b: &Real) -> Result<Real, Self::Error> {
        self.tick()?;
        Ok(a + b)
    }
    fn multiply(&mut self, a: &Real, b: &Real) -> Result<Real, Self::Error> {
        self.tick()?;
        Ok(a * b)
    }
    fn scale(&mut self, a: &Real, b: &Real) -> Result<Real, Self::Error> {
        self.tick()?;
        Ok(a * b)
    }
    fn normalize_positive_scale(&mut self, _: &mut [Real]) {}
    fn sign(&mut self, v: &Real) -> Result<Ordering, Self::Error> {
        self.tick()?;
        v.partial_cmp(&Real::zero())
            .ok_or("undecided exact rational sign")
    }
    fn sign_if_separated(&mut self, v: &Real) -> Result<Option<Ordering>, Self::Error> {
        self.sign(v).map(Some)
    }
}
#[test]
fn constant_eliminated_polynomials_keep_high_degree_completion_compact() {
    // Res_t(t^257-s^3, 2) = 2^257. A constant substitution must
    // not need a 257-by-257 characteristic-polynomial matrix.
    let mut first = vec![vec![Real::zero(); 258], vec![], vec![], vec![(-1).into()]];
    first[0][257] = Real::one();
    let second = vec![vec![Real::from(2)]];
    let mut context = CountedRationalField {
        calls: 0,
        limit: 10000,
    };
    let result = ordered_field_resultant_bivariate(
        &first,
        &second,
        CurveResultantParameter::First,
        &mut context,
    )
    .unwrap();
    assert_eq!(result, vec![Real::from(2).powi(257.into()).unwrap()]);
}
#[test]
fn ragged_storage_is_not_expanded_to_its_bounding_rectangle() {
    let mut first = vec![vec![]; 1025];
    first[0] = vec![Real::zero(); 1025];
    first[0][1024] = Real::one();
    first[1024] = vec![Real::one()];
    let mut context = CountedRationalField {
        calls: 0,
        limit: 10000,
    };
    let result = ordered_field_resultant_bivariate(
        &first,
        &[],
        CurveResultantParameter::First,
        &mut context,
    )
    .unwrap();
    assert_eq!(result, vec![Real::zero()]);
}

#[test]
fn constant_in_the_eliminated_axis_preserves_the_full_retained_polynomial_power() {
    let policy = TestContext::strict();
    let (field, a) = selected(&policy);
    let c = |n: i32| field.constant(n.into()).unwrap();
    let mut first = vec![vec![c(0); 18], vec![], vec![], vec![c(-1)]];
    first[0][17] = c(3);
    let second = vec![vec![a.clone()], vec![c(2)]];
    let expected = (0..17).fold(vec![c(1)], |p, _| product(&p, &[a.clone(), c(2)], &field));
    let mut context = SelectedPolynomialContext {
        field: field.clone(),
        policy: &policy,
    };
    for (f, g) in [(&first, &second), (&second, &first)] {
        let result =
            ordered_field_resultant_bivariate(f, g, CurveResultantParameter::First, &mut context)
                .unwrap();
        assert_eq!(result.len(), 18);
        for (actual, expected) in result.iter().zip(&expected) {
            same(actual, expected, &policy);
        }
    }
}

#[test]
fn linear_elimination_preserves_nonconstant_leading_coefficients_and_scale() {
    let policy = TestContext::strict();
    let (field, a) = selected(&policy);
    let c = |n: i32| field.constant(n.into()).unwrap();
    let roots = (1..=3).map(|i| vec![a.clone(), c(i)]).collect::<Vec<_>>();
    let mut first = factor_product(&roots, &field);
    let numerator = [a.clone(), c(0), c(1)];
    let denominator = [c(2), c(1)];
    let mut second = vec![
        vec![a.scale(&(-1).into()).unwrap(), c(2)],
        vec![c(0), c(1)],
        vec![c(-1)],
    ];
    for v in first.iter_mut().flatten() {
        *v = v.scale(&(Real::from(2) / Real::from(3)).unwrap()).unwrap();
    }
    for v in second.iter_mut().flatten() {
        *v = v.scale(&Real::from(-5)).unwrap();
    }
    let mut expected = vec![c(1)];
    for root in &roots {
        let mut factor = product(&denominator, root, &field);
        for (i, value) in numerator.iter().enumerate() {
            factor[i] = factor[i].subtract(value).unwrap();
        }
        expected = product(&expected, &factor, &field);
    }
    let scale = (Real::from(-250) / Real::from(3)).unwrap();
    for v in &mut expected {
        *v = v.scale(&scale).unwrap();
    }
    while expected.len() > 1
        && decided(expected.last().unwrap().sign(&policy).unwrap()) == RealSign::Zero
    {
        expected.pop();
    }
    let mut context = SelectedPolynomialContext {
        field: field.clone(),
        policy: &policy,
    };
    for (f, g, sign) in [(&first, &second, 1), (&second, &first, -1)] {
        let result =
            ordered_field_resultant_bivariate(f, g, CurveResultantParameter::First, &mut context)
                .unwrap();
        assert_eq!(result.len(), expected.len());
        for (actual, expected) in result.iter().zip(&expected) {
            same(actual, &expected.scale(&Real::from(sign)).unwrap(), &policy);
        }
    }
}
#[test]
fn native_and_selected_coefficient_bezout_schedules_keep_factored_magnitudes() {
    use hypersolve::{
        BivariatePolynomial, CurveIntersectionResultantConfig, CurveIntersectionResultantStatus,
    };
    let policy = TestContext::strict();
    let field = base();
    let c = |n: i32| field.constant(n.into()).unwrap();
    for n in [2, 3, 4, 5] {
        let roots = (0..n).map(|i| vec![c(i + 1), c(1)]).collect::<Vec<_>>();
        let other = (0..n).map(|i| vec![c(-i - 1), c(2)]).collect::<Vec<_>>();
        let first = factor_product(&roots, &field);
        let second = factor_product(&other, &field);
        let mut expected = vec![c(1)];
        for f in &roots {
            for g in &other {
                expected = product(&expected, &[f[0].subtract(&g[0]).unwrap(), c(-1)], &field);
            }
        }
        let mut context = SelectedPolynomialContext {
            field: field.clone(),
            policy: &policy,
        };
        let result = ordered_field_resultant_bivariate(
            &first,
            &second,
            CurveResultantParameter::First,
            &mut context,
        )
        .unwrap();
        assert_eq!(result.len(), expected.len());
        for (a, b) in result.iter().zip(&expected) {
            same(a, b, &policy);
        }
        // The independent native integer-polynomial Bareiss path shares only
        // the Bezout coefficient construction, not this field determinant.
        let authored = |slope: i32, negative: bool| {
            let mut rows = vec![vec![Real::one()]];
            for i in 0..n {
                let offset = Real::from(if negative { -i - 1 } else { i + 1 });
                let width = rows.iter().map(Vec::len).max().unwrap() + 1;
                let mut next = vec![vec![Real::zero(); width]; rows.len() + 1];
                for (i, row) in rows.iter().enumerate() {
                    for (j, value) in row.iter().enumerate() {
                        next[i][j + 1] += value;
                        next[i][j] -= value * &offset;
                        next[i + 1][j] -= value * Real::from(slope);
                    }
                }
                rows = next;
            }
            BivariatePolynomial::new(rows)
        };
        let native_first = authored(1, false);
        let native_second = authored(2, true);
        // Independently authored rational grids denote the same equations;
        // an arbitrary enclosure is not a materialization certificate.
        for (values, authored) in [(&first, &native_first), (&second, &native_second)] {
            assert_eq!(values.len(), authored.coefficients.len());
            for (row, authored) in values.iter().zip(&authored.coefficients) {
                assert_eq!(row.len(), authored.len());
                for (value, authored) in row.iter().zip(authored) {
                    same(value, &field.constant(authored.clone()).unwrap(), &policy);
                }
            }
        }
        let native = hypersolve::resultant_bivariate_polynomial_system(
            &native_first,
            &native_second,
            CurveResultantParameter::First,
            CurveIntersectionResultantConfig {
                max_resultant_degree: 128,
                ..Default::default()
            },
        );
        assert_eq!(native.status, CurveIntersectionResultantStatus::Constructed);
        assert_eq!(native.resultant_coefficients.len(), expected.len());
        for (a, b) in native.resultant_coefficients.iter().zip(&expected) {
            same(&field.constant(a.clone()).unwrap(), b, &policy);
        }
    }
}
