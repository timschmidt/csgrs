#![no_main]
use hyperdynamics::field::{
    EnclosedVectorField, EnclosureBudget, EnclosurePolicy, ExpressionField, FieldDefinition,
    coordinates::{
        CoordinateEvolutionStatus, CoordinateField, CoordinateFieldDefinition, CoordinateStatus,
    },
    evolution::{EvolutionOptions, EvolutionRequest},
    picard::PicardProposal,
};
use hyperinterval::DyadicInterval;
use hyperlattice_interval::BoxN;
use hyperreal::Real;
use hypersolve::{
    Expr, SymbolId,
    expression_program::{CoordinateChange, CoordinateError, ExpressionBudget, ExpressionProgram},
};
use libfuzzer_sys::fuzz_target;

fn b() -> EnclosureBudget {
    EnclosureBudget::new(EnclosurePolicy::default())
}
fn g() -> ExpressionBudget {
    ExpressionBudget::new(1_000_000, 100_000_000)
}
fn d(a: i64, b: i64) -> DyadicInterval {
    DyadicInterval::new(a.into(), b.into(), 0).unwrap()
}
const X: SymbolId = SymbolId(0);
const Y: SymbolId = SymbolId(1);
const T: SymbolId = SymbolId(2);
fn definition(outputs: &[Expr]) -> FieldDefinition {
    FieldDefinition {
        source: ExpressionProgram::compile(outputs, &[Y, T, X], &mut g())
            .unwrap()
            .definition()
            .clone(),
        time: T,
        state: vec![X, Y],
    }
}
fuzz_target!(|data: &[u8]| {
    if data.len() < 8 {
        return;
    }
    let [k, c, e, a, bv, t, p, q] = std::array::from_fn(|i| i64::from(data[i] % 17) - 8);
    let x = c + a + k * bv;
    let y = e + bv;
    let change = CoordinateChange {
        variables: vec![X, Y],
        to_source: vec![vec![1.into(), k.into()], vec![0.into(), 1.into()]],
        from_source: vec![vec![1.into(), (-k).into()], vec![0.into(), 1.into()]],
        origin: vec![c.into(), e.into()],
    };
    let sx = Expr::anonymous_symbol(X);
    let sy = Expr::anonymous_symbol(Y);
    let original = definition(&[
        sx.clone().powi(2) + sy.clone() + Expr::anonymous_symbol(T),
        sy.powi(2) - sx,
    ]);
    let source = ExpressionProgram::from_definition(original.source.clone(), &mut g()).unwrap();
    let chart = source.change_coordinates(change.clone(), &mut g()).unwrap();
    let expected = [x * x + y + t - k * (y * y - x), y * y - x].map(Real::from);
    assert_eq!(
        chart
            .program()
            .eval_real(&[bv.into(), t.into(), a.into()], &mut g())
            .unwrap(),
        expected
    );
    let jac = chart.program().jacobian(&[X, Y], &mut g()).unwrap();
    let expected = [
        x * x + y + t - k * (y * y - x),
        y * y - x,
        2 * x + k,
        -1,
        2 * k * x + 1 - 2 * k * y + k * k,
        2 * y - k,
    ]
    .map(Real::from);
    assert_eq!(
        jac.program()
            .eval_real(&[bv.into(), t.into(), a.into()], &mut g())
            .unwrap(),
        expected
    );
    let mut invalid = change.clone();
    invalid.from_source[0][0] = 2.into();
    assert!(matches!(
        source.change_coordinates(invalid, &mut g()),
        Err(CoordinateError::NotInverse { .. })
    ));
    let mut measured = g();
    source
        .change_coordinates(change.clone(), &mut measured)
        .unwrap();
    let work = g().remaining_work() - measured.remaining_work();
    assert!(
        source
            .change_coordinates(
                change.clone(),
                &mut ExpressionBudget::new(1_000_000, work - 1)
            )
            .is_err()
    );
    let field = ExpressionField::<2>::from_definition(original, &mut b())
        .unwrap()
        .in_coordinates(change.clone(), &mut b())
        .unwrap();
    let value = field
        .transformed_field()
        .image(
            &d(t, t),
            &BoxN::new([d(a, a), d(bv, bv)]).unwrap(),
            &mut b(),
        )
        .unwrap();
    assert_eq!(
        value.lanes(),
        Some(&[
            d(
                x * x + y + t - k * (y * y - x),
                x * x + y + t - k * (y * y - x)
            ),
            d(y * y - x, y * y - x)
        ])
    );
    let replay = CoordinateFieldDefinition {
        source: definition(&[Expr::int(p), Expr::int(q)]),
        change,
    };
    let encoded = serde_json::to_vec(&replay).unwrap();
    let field =
        CoordinateField::<2>::from_definition(serde_json::from_slice(&encoded).unwrap(), &mut b())
            .unwrap();
    let da = p - k * q;
    let proof = field.certify_step(
        PicardProposal {
            start: d(0, 0),
            end: d(1, 1),
            initial: BoxN::new([d(a, a), d(bv, bv)]).unwrap(),
            candidate: BoxN::new([
                d(a.min(a + da), a.max(a + da)),
                d(bv.min(bv + q), bv.max(bv + q)),
            ])
            .unwrap(),
        },
        &mut b(),
    );
    assert!(matches!(proof.status(), CoordinateStatus::Certified));
    assert_eq!(
        proof.source_end_enclosure().unwrap().lanes(),
        Some(&[d(x + p, x + p), d(y + q, y + q)])
    );
    let steps = 1 + i64::from(data[0] % 4);
    let evolution = field.integrate(
        EvolutionRequest {
            start: d(0, 0),
            end: d(steps, steps),
            initial: BoxN::new([d(a, a), d(bv, bv)]).unwrap(),
        },
        EvolutionOptions {
            maximum_step: Some(d(1, 1)),
            ..EvolutionOptions::default()
        },
        &mut b(),
    );
    assert!(matches!(
        evolution.status(),
        CoordinateEvolutionStatus::Completed
    ));
    assert_eq!(evolution.segments().len(), steps as usize);
    for (index, segment) in evolution.segments().enumerate() {
        let t = i64::try_from(index + 1).unwrap();
        assert_eq!(
            segment.source_end_enclosure().lanes(),
            Some(&[d(x + t * p, x + t * p), d(y + t * q, y + t * q)])
        );
        assert_eq!(
            segment.native().end_enclosure().unwrap().lanes(),
            Some(&[d(a + t * da, a + t * da), d(bv + t * q, bv + t * q)])
        );
    }
});
