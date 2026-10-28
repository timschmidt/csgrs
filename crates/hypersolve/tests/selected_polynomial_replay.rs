//! Exact graph replay must retain joint selections and original equations.

#[path = "support/recursive_quadratic.rs"]
#[allow(dead_code)]
mod support;

#[path = "support/selected_polynomial_replay.rs"]
mod codec;

use hyperreal::RealSign;
use hypersolve::classification::Classification;
use hypersolve::recursive_quadratic_field::RecursiveQuadraticField;
use hypersolve::selected_polynomial_field::{
    SelectedPolynomialDefinition, SelectedPolynomialDefinitionNode as Node,
    SelectedPolynomialError, SelectedPolynomialField, SelectedPolynomialNodeLimit,
    SelectedPolynomialReplayError as ReplayError, SelectedPolynomialValue,
};
use hypersolve::work_budget::{WorkBudget, WorkKind, with_budget};
use support::*;

fn decided<T: std::fmt::Debug>(result: Classification<T>) -> T {
    match result {
        Classification::Decided(value) => value,
        other => panic!("expected exact completion: {other:?}"),
    }
}

fn definition(values: &[SelectedPolynomialValue]) -> SelectedPolynomialDefinition {
    SelectedPolynomialDefinition::from_values(values, 10000).unwrap()
}

fn replay(graph: &SelectedPolynomialDefinition) -> Vec<SelectedPolynomialValue> {
    decided(graph.replay(10000, &TestContext::strict()).unwrap())
}

#[test]
fn native_and_repeated_polynomial_relations_survive_replay() {
    let source = cube_root_two();
    let native = RecursiveQuadraticField::base(
        vec![source.clone()],
        constant_tensor(1, real(2)),
        constant_tensor(1, real(3)),
    )
    .unwrap();
    let speeds = hypersolve::recursive_quadratic_field::RecursiveQuadraticValue::from_base(
        base_arc(&native),
        hypersolve::radical_expression::TwoSquareRootExpression {
            rational: constant_tensor(1, real(1)),
            first: constant_tensor(1, real(2)),
            second: constant_tensor(1, real(-3)),
            product: constant_tensor(1, real(4)),
        },
    )
    .unwrap();
    let a = native.retained_root_value(&source).unwrap();
    let radicand = a.add(&native.constant(real(1)).unwrap()).unwrap();
    let native = native.extension(radicand).unwrap();
    let n = generator(&native);
    let field = SelectedPolynomialField::from_recursive(native);
    let speeds = field.import_recursive(&speeds).unwrap();
    let a = field.import_recursive(&a).unwrap();
    let n = field.import_recursive(&n).unwrap();
    let quotient = decided(a.divide(&n, &TestContext::strict()).unwrap());
    let b = root(&n, 3);
    let q = base();
    // -3(t^3-2)^2 retains its repeated, nonmonic defining relation.
    let coefficients = [-12, 0, 0, 12, 0, 0, -3]
        .into_iter()
        .map(|v| q.constant(real(v)).unwrap())
        .collect();
    let repeated = decided(
        q.select(coefficients, real(1), real(2), &TestContext::strict())
            .unwrap(),
    )
    .generator()
    .unwrap();
    let graph = definition(&[a, n, quotient, b, repeated, speeds]);
    let wire = codec::encode(&graph);
    assert_eq!(codec::encode(&codec::decode(&wire)), wire);
    let values = replay(&codec::decode(&wire));
    let [a, n, quotient, b, repeated, speeds] = values.as_slice() else {
        panic!("six values")
    };
    zero(
        &power(a, 3)
            .subtract(&a.field().constant(real(2)).unwrap())
            .unwrap(),
    );
    zero(
        &power(n, 2)
            .subtract(a)
            .unwrap()
            .subtract(&a.field().constant(real(1)).unwrap())
            .unwrap(),
    );
    zero(
        &speeds
            .subtract(
                &speeds
                    .field()
                    .constant(base_closed_form(2, 3, [1, 2, -3, 4]))
                    .unwrap(),
            )
            .unwrap(),
    );
    zero(&quotient.multiply(n).unwrap().subtract(a).unwrap());
    zero(&power(b, 3).subtract(n).unwrap());
    zero(
        &power(repeated, 3)
            .subtract(&repeated.field().constant(real(2)).unwrap())
            .unwrap(),
    );
    assert_eq!(repeated.field().defining_polynomial().unwrap().len(), 7);
    let leading = repeated
        .field()
        .defining_polynomial()
        .unwrap()
        .last()
        .unwrap();
    zero(
        &leading
            .subtract(&leading.field().constant(real(-3)).unwrap())
            .unwrap(),
    );
    for kind in 0..8 {
        assert!(
            graph.nodes.iter().any(|node| matches!(
                (kind, node),
                (0, Node::NativeBase { .. })
                    | (1, Node::NativeExtension { .. })
                    | (2, Node::NativeBaseValue { .. })
                    | (3, Node::NativeExtensionValue { .. })
                    | (4, Node::NativeField { .. })
                    | (5, Node::PolynomialField { .. })
                    | (6, Node::NativeValue { .. })
                    | (7, Node::PolynomialValue { .. })
            )),
            "missing node variant {kind}"
        );
    }
}

#[test]
fn replayed_nearby_selections_remain_distinct_and_joinable() {
    let q = base();
    let small = fraction(1, 1_u64 << 40);
    let small = &small * &small;
    let small = &small * &small;
    let half = &small * fraction(1, 2);
    let next = real(1) + &small;
    let equation = vec![
        q.constant(next.clone()).unwrap(),
        q.constant(-(real(1) + &next)).unwrap(),
        q.constant(real(1)).unwrap(),
    ];
    let values: Vec<_> = [real(1), next]
        .into_iter()
        .map(|center| {
            decided(
                q.select(
                    equation.clone(),
                    &center - &half,
                    &center + &half,
                    &TestContext::strict(),
                )
                .unwrap(),
            )
            .generator()
            .unwrap()
        })
        .collect();
    let values = replay(&definition(&values));
    let join = decided(
        values[0]
            .field()
            .joined_with(values[1].field(), &TestContext::strict())
            .unwrap(),
    );
    let gap = join
        .embed(&values[1])
        .unwrap()
        .subtract(&join.embed(&values[0]).unwrap())
        .unwrap();
    zero(&gap.subtract(&gap.field().constant(small).unwrap()).unwrap());
    assert_eq!(
        gap.sign(&TestContext::strict()).unwrap(),
        Classification::Decided(RealSign::Positive)
    );
}

#[test]
fn graph_replay_rejects_invalid_references_selectors_and_denominators() {
    let q = base();
    let a = root(&q.constant(real(2)).unwrap(), 3);
    let graph = definition(&[a]);
    let mut bad = graph.clone();
    bad.roots = vec![0];
    assert!(matches!(
        bad.replay(10000, &TestContext::strict()),
        Err(ReplayError::InvalidReference { .. })
    ));
    let mut bad = graph.clone();
    let end = bad.nodes.len();
    bad.nodes.push(Node::NativeField { native: end });
    assert!(
        matches!(bad.replay(10000, &TestContext::strict()), Err(ReplayError::InvalidReference { node, reference }) if node == end && reference == end)
    );
    let mut bad = graph.clone();
    let index = bad
        .nodes
        .iter()
        .position(|n| matches!(n, Node::PolynomialField { .. }))
        .unwrap();
    if let Node::PolynomialField { coefficients, .. } = &mut bad.nodes[index] {
        coefficients[0] = 0;
    }
    assert!(matches!(
        bad.replay(10000, &TestContext::strict()),
        Err(ReplayError::InvalidReference { .. })
    ));
    let mut bad = graph.clone();
    if let Node::PolynomialField { lower, upper, .. } = &mut bad.nodes[index] {
        *lower = real(3);
        *upper = real(4);
    }
    assert!(matches!(
        bad.replay(10000, &TestContext::strict()),
        Err(ReplayError::Field(SelectedPolynomialError::NoRoot))
    ));
    let mut unused = graph.clone();
    unused.nodes.push(bad.nodes[index].clone());
    assert!(matches!(
        unused.replay(10000, &TestContext::strict()),
        Err(ReplayError::Field(SelectedPolynomialError::NoRoot))
    ));
    let other = base().constant(real(1)).unwrap();
    let a = root(&q.constant(real(2)).unwrap(), 3);
    let mut mismatched = definition(&[other, a]);
    let foreign = mismatched.roots[0];
    let output = mismatched.roots[1];
    if let Node::PolynomialValue { numerator, .. } = &mut mismatched.nodes[output] {
        *numerator = vec![foreign];
    }
    assert!(matches!(
        mismatched.replay(10000, &TestContext::strict()),
        Err(ReplayError::InvalidCoordinates { .. })
    ));

    let mut bad = graph.clone();
    if let Node::PolynomialValue { denominator, .. } = &mut bad.nodes[graph.roots[0]] {
        denominator.clear();
    } else {
        panic!("polynomial value")
    }
    assert!(matches!(
        bad.replay(10000, &TestContext::strict()),
        Err(ReplayError::Field(SelectedPolynomialError::ZeroDenominator))
    ));
    let mut bad = graph.clone();
    if let Node::NativeBase {
        first_speed_squared,
        ..
    } = &mut bad.nodes[0]
    {
        *first_speed_squared = constant_tensor(1, real(1));
    }
    assert!(matches!(
        bad.replay(10000, &TestContext::strict()),
        Err(ReplayError::InvalidCoordinates { .. })
    ));
    // A native rational zero denominator is independently guarded as well.
    let mut native = definition(&[q.constant(real(0)).unwrap(), q.constant(real(1)).unwrap()]);
    let Node::NativeValue {
        numerator: zero, ..
    } = native.nodes[native.roots[0]]
    else {
        panic!("native value")
    };
    if let Node::NativeValue { denominator, .. } = &mut native.nodes[native.roots[1]] {
        *denominator = zero;
    }
    assert!(matches!(
        native.replay(10000, &TestContext::strict()),
        Err(ReplayError::Field(SelectedPolynomialError::ZeroDenominator))
    ));
    assert_eq!(replay(&graph).len(), 1);
}

#[test]
fn native_replay_rechecks_root_ownership_and_radical_preconditions() {
    let source = cube_root_two();
    let native = RecursiveQuadraticField::base(
        vec![source.clone()],
        constant_tensor(1, real(1)),
        constant_tensor(1, real(1)),
    )
    .unwrap();
    let a = native.retained_root_value(&source).unwrap();
    let q = SelectedPolynomialField::from_recursive(native);
    let graph = definition(&[q.import_recursive(&a).unwrap()]);
    let mut owned = graph.clone();
    if let Node::NativeBase { sources, .. } = &mut owned.nodes[0] {
        sources[0].polynomial = vec![real(0), real(-1), real(1)];
        sources[0].lower = real(0);
        sources[0].upper = real(1);
    }
    let value = &replay(&owned)[0];
    zero(
        &value
            .subtract(&value.field().constant(real(1)).unwrap())
            .unwrap(),
    );
    let mut bad = owned.clone();
    if let Node::NativeBase { sources, .. } = &mut bad.nodes[0] {
        sources[0].lower = real(-1);
    }
    assert!(matches!(
        bad.replay(10000, &TestContext::strict()),
        Err(ReplayError::InvalidNativeRoot { .. })
    ));
    // An explicit point is checked against its equation, not a serialized count.
    if let Node::NativeBase { sources, .. } = &mut bad.nodes[0] {
        sources[0].exact_root = Some(real(3));
    }
    assert!(matches!(
        bad.replay(10000, &TestContext::strict()),
        Err(ReplayError::InvalidNativeRoot { .. })
    ));
    if let Node::NativeBase { sources, .. } = &mut bad.nodes[0] {
        sources[0].exact_root = Some(real(1));
    }
    let value = &replay(&bad)[0];
    zero(
        &value
            .subtract(&value.field().constant(real(1)).unwrap())
            .unwrap(),
    );
    let mut bad = graph.clone();
    if let Node::NativeBase {
        first_speed_squared,
        ..
    } = &mut bad.nodes[0]
    {
        *first_speed_squared = constant_tensor(1, real(-1));
    }
    assert!(matches!(
        bad.replay(10000, &TestContext::strict()),
        Err(ReplayError::InvalidRadicand { .. })
    ));
    if let Node::NativeBase {
        first_speed_squared,
        ..
    } = &mut bad.nodes[0]
    {
        *first_speed_squared = constant_tensor(1, real(0));
    }
    assert_eq!(replay(&bad).len(), 1);
    let native = quadratic_base(2, 3);
    let native = native.extension(native.constant(real(2)).unwrap()).unwrap();
    let n = generator(&native);
    let field = SelectedPolynomialField::from_recursive(native);
    let graph = definition(&[field.import_recursive(&n).unwrap()]);
    let radicand = graph
        .nodes
        .iter()
        .find_map(|n| {
            if let Node::NativeExtension { radicand, .. } = n {
                Some(*radicand)
            } else {
                None
            }
        })
        .unwrap();
    for value in [0, -1] {
        let mut bad = graph.clone();
        if let Node::NativeBaseValue { expression, .. } = &mut bad.nodes[radicand] {
            *expression = hypersolve::radical_expression::TwoSquareRootExpression::from_rational(
                constant_tensor(0, real(value)),
            )
            .unwrap();
        }
        assert!(matches!(
            bad.replay(10000, &TestContext::strict()),
            Err(ReplayError::InvalidRadicand { .. })
        ));
    }
}

#[test]
fn graph_limits_refuse_atomically_and_retry_without_approximation() {
    let q = base();
    let a = root(&q.constant(real(2)).unwrap(), 3);
    let roots = [a.clone(), a.clone(), a];
    let graph = definition(&roots);
    assert_eq!(graph.roots[0], graph.roots[1]);
    assert_eq!(graph.roots[1], graph.roots[2]);
    let limit = graph.nodes.len() - 1;
    assert!(
        matches!(SelectedPolynomialDefinition::from_values(&roots, limit), Err(SelectedPolynomialNodeLimit { limit: n }) if n == limit)
    );
    assert!(
        matches!(graph.replay(limit, &TestContext::strict()), Err(ReplayError::NodeLimit(SelectedPolynomialNodeLimit { limit: n })) if n == limit)
    );
    let policy = TestContext::approximate();
    let refused = with_budget(WorkBudget::ZERO, || {
        let outcome = graph.replay(graph.nodes.len(), &policy).unwrap();
        assert!(matches!(
            outcome,
            Classification::Uncertain(
                hypersolve::classification::UncertaintyReason::BudgetExhausted
            )
        ));
    });
    assert_eq!(
        refused.result.unwrap_err().kind,
        WorkKind::AlgebraicPromotion
    );
    let retried = with_budget(
        WorkBudget {
            promotions: 10,
            refinements: 1000,
        },
        || graph.replay(graph.nodes.len(), &policy).unwrap(),
    );
    let values = decided(retried.result.unwrap());
    assert_eq!(retried.usage.promotions, 2);
    zero(
        &power(&values[0], 3)
            .subtract(&values[0].field().constant(real(2)).unwrap())
            .unwrap(),
    );
    assert!(values[0].field().shares_authority(values[2].field()));
    let aliases = definition(&values);
    assert_eq!(aliases.roots[0], aliases.roots[2]);
    assert_eq!(policy.approximations(), 0);
    let empty = SelectedPolynomialDefinition {
        nodes: Vec::new(),
        roots: Vec::new(),
    };
    assert!(decided(empty.replay(0, &policy).unwrap()).is_empty());
}

#[test]
fn graph_encoding_shares_repeated_outputs_and_replays_in_a_fresh_process() {
    let q = base();
    let a = root(&q.constant(real(2)).unwrap(), 3);
    let b = root(&a, 3);
    let c = root(&b, 5);
    let roots = [a, b, c];
    let graph = definition(&roots);
    let wire = codec::encode(&graph);
    assert_eq!(wire, codec::encode(&definition(&roots)));
    let repeated = definition(&vec![roots[2].clone(); 1000]);
    assert!(repeated.nodes.len() <= graph.nodes.len());
    assert!(repeated.roots.iter().all(|id| *id == repeated.roots[0]));
    eprintln!(
        "replay fixture: {} nodes, {} exact JSON bytes; 1000 output aliases use {} nodes",
        graph.nodes.len(),
        wire.len(),
        repeated.nodes.len()
    );
    let path = std::env::temp_dir().join(format!(
        "hypersolve-selected-replay-{}-{}.json",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    ));
    std::fs::write(&path, wire).unwrap();
    let child = std::process::Command::new(std::env::current_exe().unwrap())
        .args(["--exact", "replay_in_fresh_process_child", "--nocapture"])
        .env("HYPERSOLVE_SELECTED_REPLAY_TEST_INPUT", &path)
        .output();
    std::fs::remove_file(path).unwrap();
    let child = child.unwrap();
    assert!(
        child.status.success(),
        "fresh replay failed: {}{}",
        String::from_utf8_lossy(&child.stdout),
        String::from_utf8_lossy(&child.stderr)
    );
}

#[test]
fn replay_in_fresh_process_child() {
    let Some(path) = std::env::var_os("HYPERSOLVE_SELECTED_REPLAY_TEST_INPUT") else {
        return;
    };
    let graph = codec::decode(&std::fs::read_to_string(path).unwrap());
    let values = replay(&graph);
    let [a, b, c] = values.as_slice() else {
        panic!("three exact outputs")
    };
    zero(
        &power(a, 3)
            .subtract(&a.field().constant(real(2)).unwrap())
            .unwrap(),
    );
    zero(&power(b, 3).subtract(a).unwrap());
    zero(&power(c, 5).subtract(b).unwrap());
    zero(
        &power(c, 45)
            .subtract(&c.field().constant(real(2)).unwrap())
            .unwrap(),
    );
    assert!(b.field().parent().unwrap().shares_authority(a.field()));
    assert!(c.field().parent().unwrap().shares_authority(b.field()));
    let next = root(c, 3);
    zero(&power(&next, 3).subtract(c).unwrap());
}

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

fn root(value: &SelectedPolynomialValue, degree: usize) -> SelectedPolynomialValue {
    let field = value.field();
    let mut coefficients = vec![field.constant(real(0)).unwrap(); degree + 1];
    coefficients[0] = value.scale(&real(-1)).unwrap();
    coefficients[degree] = field.constant(real(1)).unwrap();
    decided(
        field
            .select(coefficients, real(1), real(2), &TestContext::strict())
            .unwrap(),
    )
    .generator()
    .unwrap()
}

fn power(value: &SelectedPolynomialValue, exponent: usize) -> SelectedPolynomialValue {
    (0..exponent).fold(value.field().constant(real(1)).unwrap(), |v, _| {
        v.multiply(value).unwrap()
    })
}

fn zero(value: &SelectedPolynomialValue) {
    assert_eq!(
        value.sign(&TestContext::strict()).unwrap(),
        Classification::Decided(RealSign::Zero)
    );
}

#[test]
fn dependent_polynomial_graph_replay_retains_joint_authority() {
    let q = base();
    let a = root(&q.constant(real(2)).unwrap(), 3);
    let b = root(&a, 3);
    let c = root(&b, 5);
    let x = c.add(&b).unwrap();
    let y = decided(c.divide(&x, &TestContext::strict()).unwrap());
    let graph = SelectedPolynomialDefinition::from_values(&[a, b, c, x, y], 10000).unwrap();
    let values = decided(graph.replay(10000, &TestContext::strict()).unwrap());
    let [a, b, c, x, y] = values.as_slice() else {
        panic!("five roots")
    };
    assert!(b.field().parent().unwrap().shares_authority(a.field()));
    assert!(c.field().parent().unwrap().shares_authority(b.field()));
    assert!(x.field().shares_authority(c.field()));
    assert!(y.field().shares_authority(c.field()));
    zero(
        &power(a, 3)
            .subtract(&a.field().constant(real(2)).unwrap())
            .unwrap(),
    );
    zero(&power(b, 3).subtract(a).unwrap());
    zero(&power(c, 5).subtract(b).unwrap());
    zero(
        &power(c, 45)
            .subtract(&c.field().constant(real(2)).unwrap())
            .unwrap(),
    );
    zero(&x.multiply(y).unwrap().subtract(c).unwrap());
    let next = root(c, 3);
    zero(&power(&next, 3).subtract(c).unwrap());
    let bounds = decided(c.enclosed(&TestContext::strict()).unwrap());
    let bounds = decided(bounds.refined(5, &TestContext::strict()).unwrap());
    assert_eq!(
        &bounds.interval().upper - &bounds.interval().lower,
        fraction(1, 32)
    );
}

#[test]
fn elliptic_contact_graph_replays_original_curve_distance_and_side() {
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
    let equation = contact
        .iter()
        .map(|v| parent.import_recursive(v).unwrap())
        .collect();
    eprintln!("elliptic contact: select retained circle root");
    let field = decided(
        parent
            .select(equation, real(-2), real(-1), &TestContext::strict())
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
    let graph = definition(&[x, y, dx, dy]);
    let mut values = replay(&codec::decode(&codec::encode(&graph))).into_iter();
    let x = values.next().unwrap();
    let y = values.next().unwrap();
    let dx = values.next().unwrap();
    let dy = values.next().unwrap();
    let field = x.field().clone();
    let one = field.constant(real(1)).unwrap();
    assert!(x.field().shares_authority(y.field()));
    eprintln!("elliptic contact: replay original source and distance");
    crate::zero(
        &power(&y, 2)
            .subtract(&power(&x, 3))
            .unwrap()
            .subtract(&one)
            .unwrap(),
    );
    crate::zero(
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
