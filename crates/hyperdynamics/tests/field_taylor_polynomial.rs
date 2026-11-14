#![cfg(feature = "taylor-polynomial")]
use hyperdynamics::field::{
    EnclosedVectorField, EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition,
    taylor::FieldTaylorRequest,
};
use hyperinterval::{DyadicBudget, DyadicInterval};
use hyperlattice_interval::BoxN;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{ExpressionBudget, ExpressionProgram, TaylorPolynomialModel},
};
use std::sync::Arc;
fn d(l: i64, u: i64, e: i32) -> DyadicInterval {
    DyadicInterval::new(l.into(), u.into(), e).unwrap()
}
fn budget() -> EnclosureBudget {
    EnclosureBudget::new(EnclosurePolicy::default())
}
#[test]
fn field_output_products_retain_correlation_and_original_time_state_binding() {
    let (t, x, y) = (SymbolId(10), SymbolId(20), SymbolId(30));
    let variable = Expr::anonymous_symbol(x);
    let source = ExpressionProgram::compile(
        &[variable.clone(), Expr::int(1) - variable],
        &[y, t, x],
        &mut ExpressionBudget::new(100_000, 10_000_000),
    )
    .unwrap();
    let field = ExpressionField::<2>::from_definition(
        FieldDefinition {
            source: source.definition().clone(),
            time: t,
            state: vec![x, y],
        },
        &mut budget(),
    )
    .unwrap();
    let model = field
        .taylor_model(
            FieldTaylorRequest {
                time: d(0, 1, 0),
                state: BoxN::new([d(0, 1, 0), d(-2, 2, 0)]).unwrap(),
                order: 2,
            },
            &mut budget(),
        )
        .unwrap();
    let native = Arc::new(model.model().unwrap().clone());
    let (u, v) = (
        Expr::anonymous_symbol(SymbolId(1)),
        Expr::anonymous_symbol(SymbolId(2)),
    );
    let mut graph = ExpressionBudget::new(100_000, 10_000_000);
    let mut arithmetic = DyadicBudget::new(16384, 1_000_000_000);
    let map =
        ExpressionProgram::compile(&[u * v], &[SymbolId(1), SymbolId(2)], &mut graph).unwrap();
    let product = TaylorPolynomialModel::new(native, map, &mut graph, &mut arithmetic).unwrap();
    assert_eq!(model.identity(), field.identity());
    assert_eq!(
        product.source().source().definition(),
        field.identity().source()
    );
    assert_eq!(
        product.source().request().domain,
        vec![d(-2, 2, 0), d(0, 1, 0), d(0, 1, 0)]
    );
    assert_eq!(product.range(), &[d(0, 1, -2)]);
    let independent = model.image().lanes().unwrap()[0]
        .mul(&model.image().lanes().unwrap()[1], &mut arithmetic)
        .unwrap();
    assert_eq!(independent, d(0, 1, 0));
    let point = product
        .enclose_on(
            &[d(0, 0, 0), d(1, 1, -1), d(1, 1, -1)],
            &mut graph,
            &mut arithmetic,
        )
        .unwrap();
    assert_eq!(point.bounds(), &[d(1, 1, -2)]);
    let u = Expr::anonymous_symbol(SymbolId(1));
    let square = ExpressionProgram::compile(&[u.clone() * u], &[SymbolId(1)], &mut graph).unwrap();
    let squared = product
        .transform(square, &mut graph, &mut arithmetic)
        .unwrap();
    assert_eq!(squared.range(), &[d(0, 1, -4)]);
}
