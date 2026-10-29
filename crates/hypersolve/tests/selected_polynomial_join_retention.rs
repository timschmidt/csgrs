#[path = "support/recursive_quadratic.rs"]
#[allow(dead_code)]
mod support;
use hypersolve::classification::Classification;
use hypersolve::recursive_quadratic_field::RecursiveQuadraticField;
use hypersolve::selected_polynomial_field::*;
use support::*;
fn value<T>(v: Classification<T>) -> T {
    match v {
        Classification::Decided(v) => v,
        _ => panic!("exact"),
    }
}
fn root(n: i64) -> SelectedPolynomialField {
    let field = SelectedPolynomialField::from_recursive(
        RecursiveQuadraticField::base(
            vec![],
            constant_tensor(0, real(1)),
            constant_tensor(0, real(1)),
        )
        .unwrap(),
    );
    let coeff = [(-n), 0, 0, 1]
        .into_iter()
        .map(|x| field.constant(real(x)).unwrap())
        .collect();
    value(
        field
            .select(coeff, real(1), real(2), &TestContext::strict())
            .unwrap(),
    )
}
fn depth(field: &SelectedPolynomialField) -> usize {
    let mut f = field;
    let mut n = 0;
    while let Some(p) = f.parent() {
        n += 1;
        f = p;
    }
    n
}
#[test]
fn a_native_base_already_containing_all_sources_keeps_polynomial_ancestry() {
    let a = root(2);
    let b = root(3);
    let c = root(2);
    let ab = value(a.joined_with(&b, &TestContext::strict()).unwrap());
    assert_eq!(depth(ab.field()), 2);
    let abc = value(ab.field().joined_with(&c, &TestContext::strict()).unwrap());
    assert_eq!(depth(abc.field()), 3);
    assert!(
        abc.field().parent().unwrap().shares_authority(ab.field()),
        "a native base with every required source must retain its already admitted polynomial ancestors"
    );
    let reused = value(
        abc.field()
            .joined_with(ab.field(), &TestContext::strict())
            .unwrap(),
    );
    assert_eq!(
        depth(reused.field()),
        3,
        "reconsuming the original shared operand must not append its generators again"
    );
    assert!(reused.field().shares_authority(abc.field()));
}

#[test]
fn a_contained_native_axis_retains_its_original_polynomial_tower() {
    use hyperreal::RealSign;
    use hypersolve::recursive_quadratic_field::recursive_quadratic_rational_value;
    let context = TestContext::strict();
    let native = |sources: Vec<_>| {
        let one = constant_tensor(sources.len(), real(1));
        RecursiveQuadraticField::base(sources, one.clone(), one).unwrap()
    };
    let first_native = native(vec![cube_root_two(), cube_root_three()]);
    let second_native = native(vec![cube_root_two()]);
    let first = SelectedPolynomialField::from_recursive(first_native.clone());
    let second = SelectedPolynomialField::from_recursive(second_native.clone());
    let a = first
        .import_recursive(
            &recursive_quadratic_rational_value(
                &base_arc(&first_native),
                hypersolve::DenseTensorPolynomial::from_axis_polynomial(2, 0, &[real(0), real(1)])
                    .unwrap(),
            )
            .unwrap(),
        )
        .unwrap();
    let b = second
        .import_recursive(
            &recursive_quadratic_rational_value(
                &base_arc(&second_native),
                axis_tensor(&[real(0), real(1)]),
            )
            .unwrap(),
        )
        .unwrap();
    let selected = value(
        first
            .select(
                vec![
                    a.scale(&real(-1)).unwrap(),
                    first.constant(real(0)).unwrap(),
                    first.constant(real(0)).unwrap(),
                    first.constant(real(1)).unwrap(),
                ],
                real(1),
                real(2),
                &context,
            )
            .unwrap(),
    );
    let joined = value(selected.joined_with(&second, &context).unwrap());
    assert!(joined.field().shares_authority(&selected));
    assert_eq!(
        joined
            .embed(&a)
            .unwrap()
            .subtract(&joined.embed(&b).unwrap())
            .unwrap()
            .sign(&context)
            .unwrap(),
        Classification::Decided(RealSign::Zero)
    );
    let g = joined.embed(&selected.generator().unwrap()).unwrap();
    assert_eq!(
        g.multiply(&g)
            .unwrap()
            .multiply(&g)
            .unwrap()
            .subtract(&joined.embed(&b).unwrap())
            .unwrap()
            .sign(&context)
            .unwrap(),
        Classification::Decided(RealSign::Zero)
    );
    let replay = SelectedPolynomialDefinition::from_values(&[g], 1000).unwrap();
    assert_eq!(value(replay.replay(1000, &context).unwrap()).len(), 1);
}
