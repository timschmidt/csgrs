//! Dependent algebraic roots must keep their defining field relations.

#[path = "support/recursive_quadratic.rs"]
#[allow(dead_code)]
mod support;

use hyperreal::{Real, RealSign};
use hypersolve::classification::Classification;
use hypersolve::recursive_quadratic_field::RecursiveQuadraticField;
use hypersolve::selected_polynomial_field::{SelectedPolynomialField, SelectedPolynomialValue};
use hypersolve::work_budget::{WorkBudget, WorkKind, with_budget};
use support::*;

fn base() -> SelectedPolynomialField {
    SelectedPolynomialField::from_recursive(
        RecursiveQuadraticField::base(
            Vec::new(),
            constant_tensor(0, real(1)),
            constant_tensor(0, real(1)),
        )
        .unwrap(),
    )
}

fn decided<T: std::fmt::Debug>(value: Classification<T>) -> T {
    match value {
        Classification::Decided(value) => value,
        other => panic!("expected exact completion, got {other:?}"),
    }
}

fn coefficients(field: &SelectedPolynomialField, values: &[i32]) -> Vec<SelectedPolynomialValue> {
    values
        .iter()
        .map(|v| field.constant(Real::from(*v)).unwrap())
        .collect()
}

fn select(
    field: &SelectedPolynomialField,
    values: Vec<SelectedPolynomialValue>,
    lo: i32,
    hi: i32,
) -> SelectedPolynomialField {
    decided(
        field
            .select(
                values,
                Real::from(lo),
                Real::from(hi),
                &TestContext::strict(),
            )
            .unwrap(),
    )
}

fn power(value: &SelectedPolynomialValue, mut exponent: usize) -> SelectedPolynomialValue {
    let mut result = value.field().constant(real(1)).unwrap();
    let mut factor = value.clone();
    while exponent != 0 {
        if exponent % 2 == 1 {
            result = result.multiply(&factor).unwrap();
        }
        exponent /= 2;
        if exponent != 0 {
            factor = factor.multiply(&factor).unwrap();
        }
    }
    result
}

fn is_zero(value: &SelectedPolynomialValue) {
    assert_eq!(
        value.sign(&TestContext::strict()).unwrap(),
        Classification::Decided(RealSign::Zero)
    );
}

#[test]
fn selected_field_simple_root_discovery_control() {
    use hypersolve::selected_polynomial_field::SelectedPolynomialContext;
    use hypersolve::{
        OrderedFieldRootIsolationStatus,
        isolate_ordered_field_polynomial_roots_complete as isolate_ordered_field_polynomial_roots,
    };
    let q = base();
    let parent = select(&q, coefficients(&q, &[-2, 0, 0, 1]), 1, 2);
    let a = parent.generator().unwrap();
    let polynomial = vec![
        a.scale(&real(-1)).unwrap(),
        parent.constant(real(0)).unwrap(),
        parent.constant(real(0)).unwrap(),
        parent.constant(real(1)).unwrap(),
    ];
    let policy = TestContext::strict();
    let mut context = SelectedPolynomialContext {
        field: parent.clone(),
        policy: &policy,
    };
    let report = isolate_ordered_field_polynomial_roots(
        polynomial.clone(),
        &real(1),
        &real(2),
        8,
        &mut context,
    )
    .unwrap();
    eprintln!(
        "simple status={:?}, intervals={}",
        report.status,
        report.intervals.len()
    );
    assert_eq!(report.status, OrderedFieldRootIsolationStatus::Isolated);
    assert_eq!(report.intervals.len(), 1);
    let interval = &report.intervals[0];
    let selected = decided(
        parent
            .select(
                polynomial,
                interval.lower.clone(),
                interval.upper.clone(),
                &policy,
            )
            .unwrap(),
    );
    let b = selected.generator().unwrap();
    is_zero(&power(&b, 3).subtract(&selected.lift(&a).unwrap()).unwrap());
    is_zero(
        &power(&b, 9)
            .subtract(&selected.constant(real(2)).unwrap())
            .unwrap(),
    );
    let graph = hypersolve::selected_polynomial_field::SelectedPolynomialDefinition::from_values(
        &[b.clone(), selected.lift(&a).unwrap()],
        10000,
    )
    .unwrap();
    let values = decided(graph.replay(10000, &policy).unwrap());
    is_zero(&power(&values[0], 3).subtract(&values[1]).unwrap());
}
#[test]
fn selected_field_repeated_root_discovery_frontier() {
    use hypersolve::selected_polynomial_field::SelectedPolynomialContext;
    use hypersolve::{
        OrderedFieldRootIsolationStatus,
        isolate_ordered_field_polynomial_roots_complete as isolate_ordered_field_polynomial_roots,
    };
    let q = base();
    let parent = select(&q, coefficients(&q, &[-2, 0, 0, 1]), 1, 2);
    let a = parent.generator().unwrap();
    let mut polynomial = coefficients(&parent, &[0, 0, 0, 0, 0, 0, 1]);
    polynomial[0] = power(&a, 2);
    polynomial[3] = a.scale(&real(-2)).unwrap();
    // (t^3-a)^2 has exactly one distinct real root in (1,2). Selection with a
    // supplied interval works; automatic discovery must preserve multiplicity.
    let selected = select(&parent, polynomial.clone(), 1, 2);
    is_zero(
        &power(&selected.generator().unwrap(), 3)
            .subtract(&selected.lift(&a).unwrap())
            .unwrap(),
    );
    let policy = TestContext::strict();
    let mut context = SelectedPolynomialContext {
        field: parent.clone(),
        policy: &policy,
    };
    let report = isolate_ordered_field_polynomial_roots(
        polynomial.clone(),
        &real(1),
        &real(2),
        8,
        &mut context,
    )
    .unwrap();
    eprintln!(
        "repeated status={:?}, intervals={}",
        report.status,
        report.intervals.len()
    );
    assert_eq!(
        report.status,
        OrderedFieldRootIsolationStatus::Isolated,
        "complete selected-root discovery is still required"
    );
    assert_eq!(report.intervals.len(), 1);
    let interval = &report.intervals[0];
    let discovered = decided(
        parent
            .select(
                polynomial,
                interval.lower.clone(),
                interval.upper.clone(),
                &policy,
            )
            .unwrap(),
    );
    assert_eq!(discovered.defining_polynomial().unwrap().len(), 7);
    let value = discovered.generator().unwrap();
    is_zero(
        &power(&value, 3)
            .subtract(&discovered.lift(&a).unwrap())
            .unwrap(),
    );
    let graph = hypersolve::selected_polynomial_field::SelectedPolynomialDefinition::from_values(
        &[value],
        10000,
    )
    .unwrap();
    let replay = decided(graph.replay(10000, &policy).unwrap());
    is_zero(
        &power(&replay[0], 9)
            .subtract(&replay[0].field().constant(real(2)).unwrap())
            .unwrap(),
    );
}
#[test]
fn selected_field_elliptic_contact_discovery_frontier() {
    use hypersolve::BivariatePolynomial;
    use hypersolve::recursive_quadratic_field::compose_bivariate_homogeneous_polynomial;

    // The Boolean-generated corner of y^2=1+x^3 clipped at y=1/2 is
    // q=(a,1/2), where 4a^3+3=0, -1<a<0. Its coordinates are not
    // independently materialized. This is the original radius 1/64 witness.
    let root = selected_root(
        vec![real(3), real(0), real(0), real(4)],
        real(-1),
        real(0),
        13,
    );
    let native = RecursiveQuadraticField::base(
        vec![root.clone()],
        constant_tensor(1, real(1)),
        constant_tensor(1, real(1)),
    )
    .unwrap();
    let a = native.retained_root_value(&root).unwrap();
    let r = fraction(1, 64);
    let zero = native.constant(real(0)).unwrap();
    let one = native.constant(real(1)).unwrap();
    let radius = native.constant(r.clone()).unwrap();
    let x = [
        a.add(&radius).unwrap(),
        zero.clone(),
        a.subtract(&radius).unwrap(),
    ];
    let y = [
        native.constant(fraction(1, 2)).unwrap(),
        native.constant(real(2) * &r).unwrap(),
        native.constant(fraction(1, 2)).unwrap(),
    ];
    let w = [one.clone(), zero, one];
    let source = BivariatePolynomial::new(vec![
        vec![real(-1), real(0), real(1)],
        vec![],
        vec![],
        vec![real(-1)],
    ]);
    let contact = compose_bivariate_homogeneous_polynomial(&source, &x, &y, &w, 3).unwrap();
    let parent = SelectedPolynomialField::from_recursive(native);
    let equation: Vec<_> = contact
        .iter()
        .map(|v| parent.import_recursive(v).unwrap())
        .collect();
    eprintln!("elliptic contact: select retained circle root");
    let policy = TestContext::strict();
    let mut context = hypersolve::selected_polynomial_field::SelectedPolynomialContext {
        field: parent.clone(),
        policy: &policy,
    };
    let report = hypersolve::isolate_ordered_field_polynomial_roots_complete(
        equation.clone(),
        &real(-4),
        &real(4),
        8,
        &mut context,
    )
    .unwrap();
    eprintln!(
        "contact status={:?}, intervals={}",
        report.status,
        report.intervals.len()
    );
    assert_eq!(
        report.status,
        hypersolve::OrderedFieldRootIsolationStatus::Isolated
    );
    assert_eq!(report.intervals.len(), 2);
    let interval = report
        .intervals
        .iter()
        .find(|interval| interval.upper < real(-1))
        .expect("original left/down contact must be discovered");
    let field = decided(
        parent
            .select(
                equation,
                interval.lower.clone(),
                interval.upper.clone(),
                &policy,
            )
            .unwrap(),
    );
    let t = field.generator().unwrap();
    let a = field.import_recursive(&a).unwrap();
    let one = field.constant(real(1)).unwrap();
    let t2 = power(&t, 2);
    let denominator = one.add(&t2).unwrap();
    eprintln!("elliptic contact: construct correlated coordinates");
    let dx = decided(
        one.subtract(&t2)
            .unwrap()
            .scale(&r)
            .unwrap()
            .divide(&denominator, &TestContext::strict())
            .unwrap(),
    );
    let dy = decided(
        t.scale(&(real(2) * &r))
            .unwrap()
            .divide(&denominator, &TestContext::strict())
            .unwrap(),
    );
    let x = a.add(&dx).unwrap();
    let y = field.constant(fraction(1, 2)).unwrap().add(&dy).unwrap();
    assert!(x.field().shares_authority(y.field()));
    eprintln!("elliptic contact: replay original source and distance");
    is_zero(
        &power(&y, 2)
            .subtract(&power(&x, 3))
            .unwrap()
            .subtract(&one)
            .unwrap(),
    );
    is_zero(
        &power(&dx, 2)
            .add(&power(&dy, 2))
            .unwrap()
            .subtract(&field.constant(&r * &r).unwrap())
            .unwrap(),
    );
    // The cut remains on the original upper branch between x=-1 and x=a;
    // the right/up circle contact would violate the authored trim.
    for value in [dx, dy] {
        assert_sign(value.sign(&TestContext::strict()), RealSign::Negative);
    }
    assert_sign(
        x.add(&one).unwrap().sign(&TestContext::strict()),
        RealSign::Positive,
    );
    assert_sign(y.sign(&TestContext::strict()), RealSign::Positive);
}

#[path = "support/selected_polynomial_replay.rs"]
mod codec;

fn discover(
    field: &SelectedPolynomialField,
    polynomial: Vec<SelectedPolynomialValue>,
    lower: &Real,
    upper: &Real,
    steps: usize,
    policy: &TestContext,
) -> hypersolve::OrderedFieldRootIsolationReport {
    let mut context = hypersolve::selected_polynomial_field::SelectedPolynomialContext {
        field: field.clone(),
        policy,
    };
    hypersolve::isolate_ordered_field_polynomial_roots_complete(
        polynomial,
        lower,
        upper,
        steps,
        &mut context,
    )
    .unwrap()
}

fn product(
    field: &SelectedPolynomialField,
    first: &[SelectedPolynomialValue],
    second: &[SelectedPolynomialValue],
) -> Vec<SelectedPolynomialValue> {
    let mut result = vec![field.constant(real(0)).unwrap(); first.len() + second.len() - 1];
    for (i, a) in first.iter().enumerate() {
        for (j, b) in second.iter().enumerate() {
            result[i + j] = result[i + j].add(&a.multiply(b).unwrap()).unwrap();
        }
    }
    result
}

#[test]
fn complete_discovery_keeps_original_multiplicities_and_excludes_exact_roots() {
    use hypersolve::OrderedFieldRootIsolationStatus as Status;
    let q = base();
    // -3(t+2)(t-2)^2 t(t^2-2): endpoints, midpoint and two irrational roots.
    let mut polynomial = coefficients(&q, &[-3]);
    for factor in [&[2, 1][..], &[-2, 1], &[-2, 1], &[0, 1], &[-2, 0, 1]] {
        polynomial = product(&q, &polynomial, &coefficients(&q, factor));
    }
    polynomial.extend(coefficients(&q, &[0, 0]));
    let policy = TestContext::strict();
    for steps in [0, 4] {
        let report = discover(&q, polynomial.clone(), &real(-2), &real(2), steps, &policy);
        assert_eq!(report.status, Status::Isolated);
        assert_eq!(report.intervals.len(), 5);
        for (index, root) in report.intervals.iter().enumerate() {
            if let Some(exact) = &root.exact_root {
                assert_eq!(
                    exact,
                    &real(match index {
                        0 => -2,
                        2 => 0,
                        4 => 2,
                        _ => panic!("unexpected exact root"),
                    })
                );
            } else {
                assert!(index == 1 || index == 3);
                assert!(root.lower > real(-2) && root.upper < real(2));
                assert!(root.upper < real(0) || root.lower > real(0));
                let field = decided(
                    q.select(
                        polynomial.clone(),
                        root.lower.clone(),
                        root.upper.clone(),
                        &policy,
                    )
                    .unwrap(),
                );
                // The selection stores the original relation, including t-2's multiplicity.
                assert_eq!(field.defining_polynomial().unwrap().len(), polynomial.len());
                let value = field.generator().unwrap();
                is_zero(
                    &power(&value, 2)
                        .subtract(&field.constant(real(2)).unwrap())
                        .unwrap(),
                );
                assert_sign(
                    value.sign(&policy),
                    if index == 1 {
                        RealSign::Negative
                    } else {
                        RealSign::Positive
                    },
                );
            }
        }
    }
}

#[test]
fn complete_discovery_resolves_field_degree_drop_and_independent_roots() {
    use hypersolve::OrderedFieldRootIsolationStatus as Status;
    let q = base();
    let parent = select(&q, coefficients(&q, &[-2, 0, 0, 1]), 1, 2);
    let a = parent.generator().unwrap();
    let zero = power(&a, 3)
        .subtract(&parent.constant(real(2)).unwrap())
        .unwrap();
    let polynomial = vec![
        a.scale(&real(-1)).unwrap(),
        parent.constant(real(0)).unwrap(),
        parent.constant(real(1)).unwrap(),
        zero,
    ];
    for policy in [TestContext::strict(), TestContext::approximate()] {
        let report = discover(&parent, polynomial.clone(), &real(-2), &real(2), 3, &policy);
        assert_eq!(report.status, Status::Isolated);
        assert_eq!(report.intervals.len(), 2);
        let values: Vec<_> = report
            .intervals
            .iter()
            .map(|root| {
                let field = decided(
                    parent
                        .select(
                            polynomial.clone(),
                            root.lower.clone(),
                            root.upper.clone(),
                            &policy,
                        )
                        .unwrap(),
                );
                let value = field.generator().unwrap();
                is_zero(&power(&value, 2).subtract(&field.lift(&a).unwrap()).unwrap());
                is_zero(
                    &power(&value, 6)
                        .subtract(&field.constant(real(2)).unwrap())
                        .unwrap(),
                );
                value
            })
            .collect();
        let join = decided(
            values[0]
                .field()
                .joined_with(values[1].field(), &policy)
                .unwrap(),
        );
        is_zero(
            &join
                .embed(&values[0])
                .unwrap()
                .add(&join.embed(&values[1]).unwrap())
                .unwrap(),
        );
        assert_eq!(policy.approximations(), 0);
    }
}

#[test]
fn complete_discovery_separates_near_roots_and_refuses_without_partial_success() {
    use hypersolve::OrderedFieldRootIsolationStatus as Status;
    let q = base();
    let small = fraction(1, 1_u64 << 40);
    let small = &small * &small;
    let small = &small * &small;
    let second = real(1) + &small;
    let polynomial = vec![
        q.constant(second.clone()).unwrap(),
        q.constant(-(real(1) + &second)).unwrap(),
        q.constant(real(1)).unwrap(),
    ];
    let policy = TestContext::strict();
    let run = || discover(&q, polynomial.clone(), &real(0), &real(2), 0, &policy);
    let expected = run();
    assert_eq!(expected.status, Status::Isolated);
    assert_eq!(expected.intervals.len(), 2);
    assert!(expected.subdivision_steps >= 160);
    for (interval, value) in expected.intervals.iter().zip([real(1), second]) {
        if let Some(root) = &interval.exact_root {
            assert_eq!(root, &value);
        } else {
            assert!(interval.lower <= value && value <= interval.upper);
            let field = decided(
                q.select(
                    polynomial.clone(),
                    interval.lower.clone(),
                    interval.upper.clone(),
                    &policy,
                )
                .unwrap(),
            );
            is_zero(
                &field
                    .generator()
                    .unwrap()
                    .subtract(&field.constant(value).unwrap())
                    .unwrap(),
            );
        }
    }
    assert!(expected.intervals[0].upper < expected.intervals[1].lower);
    for _ in 0..2 {
        for limit in [0, 1, 8] {
            let refusal = with_budget(
                WorkBudget {
                    refinements: limit,
                    promotions: 0,
                },
                || {
                    let report = run();
                    assert_eq!(report.status, Status::BudgetExhausted);
                    assert!(report.intervals.is_empty());
                    report
                },
            );
            assert_eq!(refusal.result.unwrap_err().kind, WorkKind::RootRefinement);
            assert_eq!(refusal.usage.refinements, limit);
        }
        let retry = with_budget(
            WorkBudget {
                refinements: 1024,
                promotions: 0,
            },
            run,
        );
        assert_eq!(retry.result.unwrap(), expected);
    }
}

#[test]
fn complete_discovery_keeps_zero_rootless_and_invalid_outcomes_distinct() {
    use hypersolve::OrderedFieldRootIsolationStatus as Status;
    let q = base();
    let policy = TestContext::strict();
    for (p, status) in [
        (vec![], Status::IdenticallyZero),
        (vec![0, 0], Status::IdenticallyZero),
        (vec![1], Status::Isolated),
        (vec![1, 0, 1], Status::Isolated),
    ] {
        let report = discover(&q, coefficients(&q, &p), &real(-2), &real(2), 0, &policy);
        assert_eq!(report.status, status);
        assert!(report.intervals.is_empty());
    }
    for (lower, upper) in [(1, 1), (2, 1)] {
        let report = discover(
            &q,
            coefficients(&q, &[-2, 0, 1]),
            &real(lower),
            &real(upper),
            0,
            &policy,
        );
        assert_eq!(report.status, Status::InvalidInterval);
        assert!(report.intervals.is_empty());
    }
    let rational = discover(
        &q,
        coefficients(&q, &[0, 0, 0, 1]),
        &real(0),
        &real(1),
        0,
        &policy,
    );
    assert_eq!(rational.status, Status::Isolated);
    assert_eq!(rational.intervals.len(), 1);
    assert_eq!(rational.intervals[0].exact_root, Some(real(0)));
    let bounded = hypersolve::isolate_ordered_field_polynomial_roots(
        coefficients(&q, &[4, 0, -4, 0, 1]),
        &real(1),
        &real(2),
        hypersolve::OrderedFieldRootIsolationConfig {
            max_subdivision_depth: 12,
            refinement_steps: 0,
        },
        &mut hypersolve::selected_polynomial_field::SelectedPolynomialContext {
            field: q,
            policy: &policy,
        },
    )
    .unwrap();
    assert_eq!(bounded.status, Status::CompleteFallbackRequired);
    assert!(bounded.intervals.is_empty());
}

fn discovered_root(value: &SelectedPolynomialValue, degree: usize) -> SelectedPolynomialValue {
    let parent = value.field();
    let mut equation = vec![parent.constant(real(0)).unwrap(); degree + 1];
    equation[0] = value.scale(&real(-1)).unwrap();
    equation[degree] = parent.constant(real(1)).unwrap();
    let policy = TestContext::strict();
    let report = discover(parent, equation.clone(), &real(1), &real(2), 0, &policy);
    assert_eq!(
        report.status,
        hypersolve::OrderedFieldRootIsolationStatus::Isolated
    );
    assert_eq!(report.intervals.len(), 1);
    let interval = &report.intervals[0];
    decided(
        parent
            .select(
                equation,
                interval.lower.clone(),
                interval.upper.clone(),
                &policy,
            )
            .unwrap(),
    )
    .generator()
    .unwrap()
}

#[test]
fn discovered_roots_replay_in_a_fresh_process_and_feed_further_discovery() {
    use hypersolve::selected_polynomial_field::SelectedPolynomialDefinition;
    let q = base();
    let a = discovered_root(&q.constant(real(2)).unwrap(), 3);
    let b = discovered_root(&a, 7);
    is_zero(
        &power(&b, 21)
            .subtract(&b.field().constant(real(2)).unwrap())
            .unwrap(),
    );
    let graph = SelectedPolynomialDefinition::from_values(&[a, b], 10000).unwrap();
    let wire = codec::encode(&graph);
    assert_eq!(codec::encode(&codec::decode(&wire)), wire);
    let path = std::env::temp_dir().join(format!(
        "hypersolve-root-discovery-{}-{}.json",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    std::fs::write(&path, wire).unwrap();
    let child = std::process::Command::new(std::env::current_exe().unwrap())
        .args(["--exact", "discovered_root_replay_child", "--nocapture"])
        .env("HYPERSOLVE_ROOT_DISCOVERY_REPLAY", &path)
        .output();
    std::fs::remove_file(path).unwrap();
    let child = child.unwrap();
    assert!(
        child.status.success(),
        "{}{}",
        String::from_utf8_lossy(&child.stdout),
        String::from_utf8_lossy(&child.stderr)
    );
}

#[test]
fn discovered_root_replay_child() {
    let Some(path) = std::env::var_os("HYPERSOLVE_ROOT_DISCOVERY_REPLAY") else {
        return;
    };
    let graph = codec::decode(&std::fs::read_to_string(path).unwrap());
    let values = decided(graph.replay(10000, &TestContext::strict()).unwrap());
    let [a, b] = values.as_slice() else {
        panic!("two discovered roots")
    };
    is_zero(
        &power(a, 3)
            .subtract(&a.field().constant(real(2)).unwrap())
            .unwrap(),
    );
    is_zero(&power(b, 7).subtract(a).unwrap());
    let c = discovered_root(b, 3);
    is_zero(&power(&c, 3).subtract(b).unwrap());
    is_zero(
        &power(&c, 63)
            .subtract(&c.field().constant(real(2)).unwrap())
            .unwrap(),
    );
}

struct LimitedRemainder<F>(F);

impl<C, F: hypersolve::OrderedFieldPolynomialContext<C>>
    hypersolve::OrderedFieldPolynomialContext<C> for LimitedRemainder<F>
{
    type Error = F::Error;
    fn constant(&mut self, value: &Real) -> Result<C, Self::Error> {
        self.0.constant(value)
    }
    fn add(&mut self, a: &C, b: &C) -> Result<C, Self::Error> {
        self.0.add(a, b)
    }
    fn multiply(&mut self, a: &C, b: &C) -> Result<C, Self::Error> {
        self.0.multiply(a, b)
    }
    fn scale(&mut self, value: &C, scale: &Real) -> Result<C, Self::Error> {
        self.0.scale(value, scale)
    }
    fn normalize_positive_scale(&mut self, values: &mut [C]) {
        self.0.normalize_positive_scale(values);
    }
    fn sign(&mut self, value: &C) -> Result<std::cmp::Ordering, Self::Error> {
        self.0.sign(value)
    }
    fn sign_if_separated(&mut self, value: &C) -> Result<Option<std::cmp::Ordering>, Self::Error> {
        self.0.sign_if_separated(value)
    }
    fn remainder_elimination_budget(&self) -> Option<usize> {
        Some(0)
    }
}

#[test]
fn complete_discovery_retains_context_fallback_without_inventing_exhaustion() {
    let q = base();
    let policy = TestContext::strict();
    let mut context = LimitedRemainder(
        hypersolve::selected_polynomial_field::SelectedPolynomialContext {
            field: q.clone(),
            policy: &policy,
        },
    );
    let result = with_budget(WorkBudget::ZERO, || {
        hypersolve::isolate_ordered_field_polynomial_roots_complete(
            coefficients(&q, &[-2, 0, 0, 1]),
            &real(1),
            &real(2),
            4,
            &mut context,
        )
        .unwrap()
    });
    let report = result
        .result
        .expect("no subdivision or promotion was attempted");
    assert_eq!(
        report.status,
        hypersolve::OrderedFieldRootIsolationStatus::CompleteFallbackRequired
    );
    assert!(report.intervals.is_empty());
    assert_eq!(result.usage.refinements, 0);
    assert_eq!(result.usage.promotions, 0);
    let retry = discover(
        &q,
        coefficients(&q, &[-2, 0, 0, 1]),
        &real(1),
        &real(2),
        4,
        &policy,
    );
    assert_eq!(
        retry.status,
        hypersolve::OrderedFieldRootIsolationStatus::Isolated
    );
    assert_eq!(retry.intervals.len(), 1);
}

#[test]
fn complete_discovery_admits_degree_seventeen_over_an_algebraic_field() {
    let q = base();
    let a = discovered_root(&q.constant(real(2)).unwrap(), 3);
    let value = discovered_root(&a, 17);
    is_zero(
        &power(&value, 17)
            .subtract(&value.field().lift(&a).unwrap())
            .unwrap(),
    );
    is_zero(
        &power(&value, 51)
            .subtract(&value.field().constant(real(2)).unwrap())
            .unwrap(),
    );
    assert!(value.field().parent().unwrap().shares_authority(a.field()));
}

#[test]
fn complete_discovery_enforces_local_depth_without_partial_roots_or_poisoning_retry() {
    use hypersolve::{OrderedFieldRootIsolationConfig, OrderedFieldRootIsolationStatus as Status};
    let field = base();
    let policy = TestContext::strict();
    let run = |values: &[i32], lower, upper, depth, refinements| {
        let mut context = hypersolve::selected_polynomial_field::SelectedPolynomialContext {
            field: field.clone(),
            policy: &policy,
        };
        hypersolve::isolate_ordered_field_polynomial_roots_complete_with_config(
            coefficients(&field, values),
            &real(lower),
            &real(upper),
            OrderedFieldRootIsolationConfig {
                max_subdivision_depth: depth,
                refinement_steps: refinements,
            },
            &mut context,
        )
        .unwrap()
    };
    for (values, lower, upper, depth, refinements, steps) in [
        (&[-2, 0, 1][..], -2, 2, 0, 0, 0),
        (&[-2, 0, 1][..], -2, 2, 1, 2, 1),
        (&[0, -2, 0, 1][..], 0, 2, 0, 0, 0),
    ] {
        let report = run(values, lower, upper, depth, refinements);
        assert_eq!(report.status, Status::BudgetExhausted);
        assert!(report.intervals.is_empty());
        assert_eq!(report.subdivision_steps, steps);
    }
    let separated = run(&[-2, 0, 1], -2, 2, 1, 0);
    assert_eq!(separated.status, Status::Isolated);
    assert_eq!(separated.intervals.len(), 2);
    assert_eq!(separated.subdivision_steps, 1);
    for root in separated.intervals {
        let selected = decided(
            field
                .select(
                    coefficients(&field, &[-2, 0, 1]),
                    root.lower,
                    root.upper,
                    &policy,
                )
                .unwrap(),
        );
        let value = selected.generator().unwrap();
        is_zero(
            &value
                .multiply(&value)
                .unwrap()
                .subtract(&selected.constant(real(2)).unwrap())
                .unwrap(),
        );
    }
    let exact = run(&[0, -2, 0, 1], 0, 2, 32, 3);
    assert_eq!(exact.status, Status::Isolated);
    assert_eq!(exact.intervals.len(), 2);
    assert_eq!(exact.intervals[0].exact_root, Some(real(0)));
    assert!(exact.intervals[1].lower > real(0));
    let control = discover(
        &field,
        coefficients(&field, &[-2, 0, 1]),
        &real(-2),
        &real(2),
        0,
        &policy,
    );
    assert_eq!(control.status, Status::Isolated);
    assert_eq!(control.intervals.len(), 2);
}

#[test]
fn selected_value_endpoints_count_distinct_roots_without_projection() {
    use hypersolve::selected_polynomial_field::SelectedPolynomialContext;
    use hypersolve::{
        ordered_field_count_roots_between_values as count,
        ordered_field_polynomial_sign_at_value as sign_at,
    };
    let q = base();
    let field = select(&q, coefficients(&q, &[-2, 0, 0, 1]), 1, 2);
    let a = field.generator().unwrap();
    let one = field.constant(real(1)).unwrap();
    let minus_two = field.constant(real(-2)).unwrap();
    let epsilon = field
        .constant((Real::one() / Real::from(1_u128 << 80)).unwrap())
        .unwrap();
    let below = a.subtract(&epsilon).unwrap();
    let above = a.add(&epsilon).unwrap();
    // -3(t-a)^2(t+1)^2 retains two distinct roots, both repeated.
    let factor = [
        a.scale(&real(-1)).unwrap(),
        one.subtract(&a).unwrap(),
        one.clone(),
    ];
    let mut polynomial = vec![field.constant(real(0)).unwrap(); 5];
    for (i, first) in factor.iter().enumerate() {
        for (j, second) in factor.iter().enumerate() {
            polynomial[i + j] = polynomial[i + j]
                .add(&first.multiply(second).unwrap().scale(&real(-3)).unwrap())
                .unwrap();
        }
    }
    let policy = TestContext::strict();
    let mut context = SelectedPolynomialContext {
        field: field.clone(),
        policy: &policy,
    };
    assert_eq!(
        sign_at(&polynomial, &a, &mut context).unwrap(),
        std::cmp::Ordering::Equal
    );
    for (lower, upper, expected) in [
        (&minus_two, &below, Some(1)),
        (&minus_two, &above, Some(2)),
        (&below, &above, Some(1)),
        (&minus_two, &a, None),
        (&a, &above, None),
        (&above, &below, None),
        (&a, &a, None),
    ] {
        assert_eq!(
            count(&polynomial, lower, upper, &mut context).unwrap(),
            expected
        );
    }
    assert_eq!(
        count(
            &[field.constant(real(0)).unwrap()],
            &below,
            &above,
            &mut context
        )
        .unwrap(),
        None
    );
    assert_eq!(
        count(
            &[field.constant(real(-3)).unwrap()],
            &below,
            &above,
            &mut context
        )
        .unwrap(),
        Some(0)
    );
}
