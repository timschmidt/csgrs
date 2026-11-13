use hyperlimit::{PredicatePolicy, classify_real_sign};
use hyperreal::{Real, dispatch_trace};
use hypersolve::{
    Expr, SymbolId,
    expression_program::{ExpressionBudget, ExpressionProgram},
};
use std::fmt::Write;

// The baseline was generated before the derivative-rule extraction. Both policies
// consume every result; exact output serialization and certainty receipts are
// fingerprinted separately from the fixed full dispatch/rational counts.
pub fn snapshot() -> (String, String) {
    let mut summary = String::new();
    let mut evidence = String::new();
    for policy in [PredicatePolicy::STRICT, PredicatePolicy::APPROXIMATE_512] {
        let x = Expr::anonymous_symbol(SymbolId(0));
        let y = Expr::anonymous_symbol(SymbolId(1));
        let outputs = vec![
            Expr::int(7),
            x.clone(),
            y.clone(),
            -x.clone(),
            x.clone() + y.clone(),
            x.clone() - y.clone(),
            x.clone() * y.clone(),
            x.clone() / y.clone(),
            (x.clone() + Expr::int(1)).powi(0),
            (x.clone() + Expr::int(1)).powi(1),
            (x.clone() + Expr::int(1)).powi(3),
            (x.clone() + Expr::int(1)).powi(-3),
            (x.clone() + Expr::int(1)).powi(i64::MIN),
            (x.clone() + Expr::int(1)).powi(i64::MAX),
            (x.clone() + Expr::int(4)).sqrt(),
            x.clone().sin(),
            x.clone().cos(),
            (x.clone() + Expr::int(1)).ln(),
            (x.clone() + Expr::int(1)).log10(),
            x.clone().asin(),
            x.clone().acos(),
            (x.clone() + Expr::real((Real::from(5) / Real::from(4)).unwrap())).acosh(),
            x.atanh(),
        ];
        let mut graph = ExpressionBudget::new(1_000_000, 100_000_000);
        let source =
            ExpressionProgram::compile(&outputs, &[SymbolId(1), SymbolId(0)], &mut graph).unwrap();
        dispatch_trace::reset();
        let (jacobian, values, receipts) = dispatch_trace::with_recording(|| {
            let jacobian = source
                .jacobian(&[SymbolId(0), SymbolId(1)], &mut graph)
                .unwrap();
            let values = jacobian
                .program()
                .eval_real(&[Real::from(2), Real::zero()], &mut graph)
                .unwrap();
            let receipts: Vec<_> = values
                .iter()
                .map(|v| classify_real_sign(v, policy))
                .collect();
            (jacobian, values, receipts)
        });
        let trace = dispatch_trace::take_trace();
        let exact = serde_json::to_string(&(
            jacobian.source().definition(),
            jacobian.program().definition(),
            values,
        ))
        .unwrap();
        let row = format!("{exact};receipts={receipts:?}");
        let hash = row.bytes().fold(0xcbf29ce484222325u64, |h, b| {
            (h ^ u64::from(b)).wrapping_mul(0x100000001b3)
        });
        writeln!(
            summary,
            "policy={policy:?};work={};bytes={};fnv1a={hash:016x};trace={trace:?}",
            100_000_000 - graph.remaining_work(),
            row.len()
        )
        .unwrap();
        writeln!(evidence, "policy={policy:?};{row}").unwrap();
    }
    (summary, evidence)
}
