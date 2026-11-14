#![cfg(feature = "interval-residuals")]
use hypercircuit::{
    CircuitError, CircuitParameter, ComponentId, LinearMnaSystem, LinearStamp, MnaUnknown, NetId,
    Real,
    residual::{
        CircuitResidualProgram, ResidualDisposition, ResidualError, ResidualInput,
        ResidualInputKind,
    },
};
use hyperinterval::{DyadicBudget, DyadicInterval};
use hypersolve::{
    Expr, SymbolId,
    expression_program::{ExpressionBudget, ExpressionError, ExpressionProgram},
    work_budget::CancellationToken,
};

fn g() -> ExpressionBudget {
    ExpressionBudget::new(100_000, 10_000_000)
}
fn a() -> DyadicBudget {
    DyadicBudget::new(4096, 100_000_000)
}
fn d(lo: i64, hi: i64, exponent: i32) -> DyadicInterval {
    DyadicInterval::new(lo.into(), hi.into(), exponent).unwrap()
}
fn net(name: &str) -> NetId {
    NetId::new(name).unwrap()
}
fn unknown(symbol: u32, name: &str) -> ResidualInput {
    ResidualInput {
        symbol: SymbolId(symbol),
        kind: ResidualInputKind::Unknown(MnaUnknown::NetVoltage(net(name))),
    }
}
fn parameter(symbol: u32, name: &str) -> ResidualInput {
    ResidualInput {
        symbol: SymbolId(symbol),
        kind: ResidualInputKind::Parameter {
            component: ComponentId::new("R1").unwrap(),
            parameter: CircuitParameter {
                name: name.into(),
                value: Real::from(1),
                unit: "ohm".into(),
                source: "supplier resistance tolerance".into(),
            },
        },
    }
}
fn source(outputs: &[Expr], bindings: &[SymbolId]) -> ExpressionProgram {
    ExpressionProgram::compile(outputs, bindings, &mut g()).unwrap()
}
fn resistor() -> CircuitResidualProgram {
    // Intentionally unsorted source bindings: V and R, never inferred from ids.
    let v = Expr::anonymous_symbol(SymbolId(9));
    let r = Expr::anonymous_symbol(SymbolId(2));
    CircuitResidualProgram::new(
        source(&[v / r - Expr::int(1)], &[SymbolId(9), SymbolId(2)]),
        vec![unknown(9, "v"), parameter(2, "resistance")],
        vec!["R1 KCL (A)".into()],
        &mut g(),
    )
    .unwrap()
}

#[test]
fn tolerance_source_has_independent_residual_derivatives_and_no_false_root_claim() {
    let program = resistor();
    let args = [d(3, 4, 0), d(1, 2, 0)];
    let report = program.enclose(&args, -16, &mut g(), &mut a()).unwrap();
    assert_eq!(report.residuals(), &[d(1, 6, -1)]);
    assert_eq!(
        report.disposition(),
        ResidualDisposition::Excluded { equation: 0 }
    );
    assert_eq!(report.evidence().arguments(), &args);
    assert_eq!(report.evidence().precision(), -16);
    assert_eq!(report.circuit().inputs(), program.inputs());
    assert_eq!(report.circuit().equations(), &["R1 KCL (A)"]);
    let matrix = report
        .evidence()
        .matrix::<1, 2>(&mut g(), &mut a())
        .unwrap();
    assert_eq!(
        matrix.intervals().rows(),
        Some(&[[d(1, 8, -2), d(-16, -3, -2)]])
    );
    assert_eq!(
        matrix.intervals().norm_infinity(&mut a()).unwrap(),
        d(1, 6, 0)
    );
    // The existing quotient derivative is R/R^2. Its natural interval image is
    // conservative, not the minimal hull of 1/R. Keep the exact analytical
    // derivative range as an independent inclusion check, plus corner equalities.
    assert!(
        d(1, 2, -1)
            .subset_of(&matrix.intervals().rows().unwrap()[0][0], &mut a())
            .unwrap()
    );
    let possible = program
        .enclose(&[d(1, 2, 0), d(1, 2, 0)], -16, &mut g(), &mut a())
        .unwrap();
    assert_eq!(possible.disposition(), ResidualDisposition::NotExcluded);
    assert_eq!(possible.residuals(), &[d(-1, 2, -1)]);
    // Corner enumeration independently evaluates (V/R-1, 1/R, -V/R^2).
    for v in [3, 4] {
        for r in [1, 2] {
            let values = program
                .jacobian()
                .program()
                .eval_real(&[Real::from(v), Real::from(r)], &mut g())
                .unwrap();
            assert_eq!(
                values,
                vec![
                    (Real::from(v) / Real::from(r)).unwrap() - Real::one(),
                    (Real::one() / Real::from(r)).unwrap(),
                    (-Real::from(v) / Real::from(r * r)).unwrap()
                ]
            );
        }
    }
}

#[test]
fn mna_lowering_retains_exact_equations_unknown_order_and_snapshot_identity() {
    let stamps = [LinearStamp::Conductance {
        component: ComponentId::new("G1").unwrap(),
        part: None,
        pos: Some(net("v")),
        neg: None,
        conductance: Real::from(2),
    }];
    let mut system = LinearMnaSystem::from_stamps(vec![net("v")], &stamps).unwrap();
    system.rhs[0] = Real::from(3);
    let program = system.residual_program(&mut g()).unwrap();
    assert_eq!(program.inputs(), &[unknown(0, "v")]);
    for value in -4..=4 {
        let exact = system.replay_candidate(&[Real::from(value)]).unwrap();
        let report = program
            .enclose(&[d(value, value, 0)], -16, &mut g(), &mut a())
            .unwrap();
        assert_eq!(exact.residuals, vec![Real::from(2 * value - 3)]);
        assert_eq!(report.residuals(), &[d(2 * value - 3, 2 * value - 3, 0)]);
        assert_eq!(
            report
                .evidence()
                .matrix::<1, 1>(&mut g(), &mut a())
                .unwrap()
                .intervals()
                .rows(),
            Some(&[[d(2, 2, 0)]])
        );
    }
    system.matrix[0][0] = Real::from(100);
    system.rhs[0] = Real::from(0);
    system.unknowns[0] = MnaUnknown::NetVoltage(net("changed"));
    assert_eq!(program.inputs(), &[unknown(0, "v")]);
    let report = program
        .enclose(&[d(1, 2, 0)], -16, &mut g(), &mut a())
        .unwrap();
    assert_eq!(report.residuals(), &[d(-1, 1, 0)]);
    assert_eq!(report.disposition(), ResidualDisposition::NotExcluded);
    assert_ne!(
        system
            .residual_program(&mut g())
            .unwrap()
            .source()
            .definition(),
        program.source().definition()
    );
}

#[test]
fn all_linear_stamp_families_share_existing_mna_authority() {
    use hypercircuit::BranchId;
    let component = ComponentId::new("test").unwrap();
    let stamps = [
        LinearStamp::VoltageSource {
            component: component.clone(),
            branch: BranchId::new("i").unwrap(),
            pos: Some(net("a")),
            neg: None,
            voltage: Real::from(5),
        },
        LinearStamp::Conductance {
            component: component.clone(),
            part: None,
            pos: Some(net("a")),
            neg: Some(net("b")),
            conductance: Real::from(2),
        },
        LinearStamp::CurrentSource {
            component: component.clone(),
            pos: Some(net("b")),
            neg: None,
            current: Real::from(3),
        },
        LinearStamp::Vccs {
            component: component.clone(),
            pos: Some(net("b")),
            neg: None,
            ctrl_pos: Some(net("a")),
            ctrl_neg: Some(net("b")),
            transconductance: Real::from(4),
        },
        LinearStamp::Companion {
            component,
            pos: Some(net("b")),
            neg: None,
            conductance: Real::from(1),
            history_current: Real::from(6),
        },
    ];
    let system = LinearMnaSystem::from_stamps(vec![net("a"), net("b")], &stamps).unwrap();
    assert_eq!(
        system.matrix,
        [[2, -2, 1], [2, -1, 0], [1, 0, 0]]
            .map(|r| r.map(Real::from).to_vec())
            .to_vec()
    );
    assert_eq!(
        system.rhs,
        vec![Real::zero(), Real::from(-9), Real::from(5)]
    );
    let program = system.residual_program(&mut g()).unwrap();
    let report = program
        .enclose(
            &[d(4, 6, 0), d(1, 2, 0), d(-1, 1, 0)],
            -16,
            &mut g(),
            &mut a(),
        )
        .unwrap();
    assert_eq!(
        report.residuals(),
        &[d(3, 11, 0), d(15, 20, 0), d(-1, 1, 0)]
    );
    assert_eq!(
        report
            .evidence()
            .matrix::<3, 3>(&mut g(), &mut a())
            .unwrap()
            .intervals()
            .rows(),
        Some(&[
            [d(2, 2, 0), d(-2, -2, 0), d(1, 1, 0)],
            [d(2, 2, 0), d(-1, -1, 0), d(0, 0, 0)],
            [d(1, 1, 0), d(0, 0, 0), d(0, 0, 0)]
        ])
    );
}

#[test]
fn parameter_occurrences_share_one_symbol_and_generated_source_is_consumable() {
    let r = Expr::anonymous_symbol(SymbolId(2));
    let raw = source(&[r.clone() * r.clone(), r.clone() - r], &[SymbolId(2)]);
    let program = CircuitResidualProgram::new(
        raw,
        vec![parameter(2, "resistance")],
        vec!["square".into(), "shared cancellation".into()],
        &mut g(),
    )
    .unwrap();
    let report = program
        .enclose(&[d(1, 2, 0)], -16, &mut g(), &mut a())
        .unwrap();
    assert_eq!(
        report
            .evidence()
            .matrix::<2, 1>(&mut g(), &mut a())
            .unwrap()
            .intervals()
            .rows(),
        Some(&[[d(2, 4, 0)], [d(0, 0, 0)]])
    );
    assert_eq!(report.residuals(), &[d(1, 4, 0), d(-1, 1, 0)]); // Bounds do not invent cancellation of uncertain lanes.
    let derivative_program = CircuitResidualProgram::new(
        program.jacobian().program().clone(),
        program.inputs().to_vec(),
        vec![
            "square".into(),
            "cancel".into(),
            "slope".into(),
            "zero".into(),
        ],
        &mut g(),
    )
    .unwrap();
    let next = derivative_program
        .enclose(&[d(1, 2, 0)], -16, &mut g(), &mut a())
        .unwrap();
    assert_eq!(
        next.residuals(),
        &[d(1, 4, 0), d(-1, 1, 0), d(2, 4, 0), d(0, 0, 0)]
    );
    assert_eq!(
        next.evidence()
            .matrix::<4, 1>(&mut g(), &mut a())
            .unwrap()
            .intervals()
            .rows()
            .unwrap()[2],
        [d(2, 2, 0)]
    );
}

#[test]
fn invalid_metadata_and_mna_shapes_do_not_become_empty_success() {
    let raw = source(&[Expr::int(1)], &[SymbolId(9), SymbolId(2)]);
    assert!(matches!(
        CircuitResidualProgram::new(raw.clone(), vec![], vec!["row".into()], &mut g()),
        Err(ResidualError::MetadataCount { inputs: true, .. })
    ));
    assert!(matches!(
        CircuitResidualProgram::new(
            raw.clone(),
            vec![unknown(9, "v"), parameter(2, "r")],
            vec![],
            &mut g()
        ),
        Err(ResidualError::MetadataCount { inputs: false, .. })
    ));
    assert!(matches!(
        CircuitResidualProgram::new(
            raw.clone(),
            vec![unknown(2, "v"), parameter(9, "r")],
            vec!["row".into()],
            &mut g()
        ),
        Err(ResidualError::InputSymbol { slot: 0 })
    ));
    assert!(matches!(
        CircuitResidualProgram::new(
            raw.clone(),
            vec![unknown(9, "v"), unknown(2, "v")],
            vec!["row".into()],
            &mut g()
        ),
        Err(ResidualError::DuplicateInput { slot: 1 })
    ));
    assert!(matches!(
        CircuitResidualProgram::new(
            raw.clone(),
            vec![parameter(9, "r"), parameter(2, "r")],
            vec!["row".into()],
            &mut g()
        ),
        Err(ResidualError::DuplicateInput { slot: 1 })
    ));
    assert!(matches!(
        CircuitResidualProgram::new(
            raw,
            vec![unknown(9, "v"), parameter(2, "")],
            vec!["row".into()],
            &mut g()
        ),
        Err(ResidualError::EmptyParameterName { slot: 1 })
    ));
    let bad = LinearMnaSystem {
        unknowns: vec![],
        matrix: vec![vec![]],
        rhs: vec![],
    };
    assert!(matches!(
        bad.residual_program(&mut g()),
        Err(ResidualError::Circuit(
            CircuitError::CandidateLengthMismatch
        ))
    ));
    let duplicate = LinearMnaSystem {
        unknowns: vec![MnaUnknown::NetVoltage(net("v")); 2],
        matrix: vec![vec![Real::one(); 2]; 2],
        rhs: vec![Real::zero(); 2],
    };
    assert!(matches!(
        duplicate.residual_program(&mut g()),
        Err(ResidualError::DuplicateInput { slot: 1 })
    ));
    assert!(matches!(
        resistor().enclose(&[DyadicInterval::empty()], -16, &mut g(), &mut a()),
        Err(ResidualError::Expression(
            ExpressionError::BindingCount { .. }
        ))
    ));
}

#[test]
fn empty_unit_and_zero_equation_products_remain_distinct() {
    let report = resistor()
        .enclose(
            &[DyadicInterval::empty(), d(0, 0, 0)],
            -16,
            &mut g(),
            &mut a(),
        )
        .unwrap();
    assert_eq!(report.disposition(), ResidualDisposition::Empty);
    assert!(report.residuals()[0].is_empty());
    assert!(
        report
            .evidence()
            .matrix::<1, 2>(&mut g(), &mut a())
            .unwrap()
            .intervals()
            .is_empty()
    );
    let unit = LinearMnaSystem {
        unknowns: vec![],
        matrix: vec![],
        rhs: vec![],
    }
    .residual_program(&mut g())
    .unwrap();
    assert_eq!(
        unit.enclose(&[], -16, &mut g(), &mut a())
            .unwrap()
            .disposition(),
        ResidualDisposition::NotExcluded
    );
    let zero = CircuitResidualProgram::new(
        source(&[], &[SymbolId(9)]),
        vec![unknown(9, "v")],
        vec![],
        &mut g(),
    )
    .unwrap();
    assert_eq!(
        zero.enclose(&[d(0, 1, 0)], -16, &mut g(), &mut a())
            .unwrap()
            .disposition(),
        ResidualDisposition::NotExcluded
    );
    assert_eq!(
        zero.enclose(&[DyadicInterval::empty()], -16, &mut g(), &mut a())
            .unwrap()
            .disposition(),
        ResidualDisposition::Empty
    );
    let constant = CircuitResidualProgram::new(
        source(&[Expr::int(1)], &[]),
        vec![],
        vec!["inconsistent".into()],
        &mut g(),
    )
    .unwrap();
    assert_eq!(
        constant
            .enclose(&[], -16, &mut g(), &mut a())
            .unwrap()
            .disposition(),
        ResidualDisposition::Excluded { equation: 0 }
    );
}

#[test]
fn nonlinear_guards_and_typed_causes_survive() {
    use std::error::Error;
    let program = resistor();
    let failure = program
        .enclose(&[d(1, 2, 0), d(0, 1, 0)], -16, &mut g(), &mut a())
        .unwrap_err();
    assert!(matches!(failure, ResidualError::Expression(_)));
    assert!(failure.source().is_some());
    // Normalized Shockley relation in its admitted inverse form: ln(1+I)-V=0,
    // with I>-1. The expression grammar has logarithm but no exponential node.
    let i = Expr::anonymous_symbol(SymbolId(9));
    let v = Expr::anonymous_symbol(SymbolId(2));
    let diode = CircuitResidualProgram::new(
        source(&[(Expr::int(1) + i).ln() - v], &[SymbolId(9), SymbolId(2)]),
        vec![
            ResidualInput {
                symbol: SymbolId(9),
                kind: ResidualInputKind::Unknown(MnaUnknown::BranchCurrent(
                    hypercircuit::BranchId::new("diode_current").unwrap(),
                )),
            },
            unknown(2, "diode_voltage"),
        ],
        vec!["normalized inverse Shockley relation".into()],
        &mut g(),
    )
    .unwrap();
    let zero = diode
        .enclose(&[d(0, 0, 0), d(0, 0, 0)], -24, &mut g(), &mut a())
        .unwrap();
    assert_eq!(zero.residuals(), &[d(0, 0, 0)]);
    assert_eq!(
        zero.evidence()
            .matrix::<1, 2>(&mut g(), &mut a())
            .unwrap()
            .intervals()
            .rows(),
        Some(&[[d(1, 1, 0), d(-1, -1, 0)]])
    );
    let positive = diode
        .enclose(&[d(1, 2, 0), d(0, 0, 0)], -24, &mut g(), &mut a())
        .unwrap();
    assert_eq!(
        positive.disposition(),
        ResidualDisposition::Excluded { equation: 0 }
    );
    assert!(
        diode
            .enclose(&[d(-1, 1, 0), d(0, 0, 0)], -24, &mut g(), &mut a())
            .is_err()
    );
}

#[test]
fn graph_native_limits_cancellation_and_retry_preserve_original_source() {
    let program = resistor();
    let original = program.source().definition().clone();
    let args = [d(1, 2, 0), d(1, 2, 0)];
    for mut graph in [
        ExpressionBudget::new(0, 10_000_000),
        ExpressionBudget::new(100_000, 0),
    ] {
        assert!(program.enclose(&args, -16, &mut graph, &mut a()).is_err());
    }
    assert!(
        program
            .enclose(&args, -16, &mut g(), &mut DyadicBudget::new(4096, 0))
            .is_err()
    );
    let token = CancellationToken::new();
    token.cancel();
    let mut cancelled = g();
    cancelled.cancellation = Some(token);
    assert!(matches!(
        program.enclose(&args, -16, &mut cancelled, &mut a()),
        Err(ResidualError::Expression(ExpressionError::Cancelled))
    ));
    let mut measured = g();
    program
        .enclose(&args, -16, &mut measured, &mut a())
        .unwrap();
    let spent = g().remaining_work() - measured.remaining_work();
    assert!(matches!(
        program.enclose(
            &args,
            -16,
            &mut ExpressionBudget::new(100_000, spent - 1),
            &mut a()
        ),
        Err(ResidualError::Expression(ExpressionError::WorkExhausted))
    ));
    assert_eq!(program.source().definition(), &original);
    assert_eq!(
        program
            .enclose(&args, -16, &mut g(), &mut a())
            .unwrap()
            .disposition(),
        ResidualDisposition::NotExcluded
    );
}

#[test]
fn definitions_rebuild_in_a_fresh_process_without_live_evidence() {
    const MARKER: &str = "HYPERCIRCUIT_RESIDUAL_REPLAY_CHILD";
    if let Some(payload) = std::env::var_os(MARKER) {
        // Fixture wire format: exact signed-integer A,b and dyadic endpoints.
        // This is test replay of definitions, not a general circuit interchange.
        let values: Vec<i64> = payload
            .to_str()
            .unwrap()
            .split(',')
            .map(|x| x.parse().unwrap())
            .collect();
        assert_eq!(values.len(), 4);
        let system = LinearMnaSystem {
            unknowns: vec![MnaUnknown::NetVoltage(net("fresh"))],
            matrix: vec![vec![Real::from(values[0])]],
            rhs: vec![Real::from(values[1])],
        };
        let program = system.residual_program(&mut g()).unwrap();
        let report = program
            .enclose(&[d(values[2], values[3], 0)], -16, &mut g(), &mut a())
            .unwrap();
        assert_eq!(report.residuals(), &[d(-1, 1, 0)]);
        assert_eq!(
            report
                .evidence()
                .matrix::<1, 1>(&mut g(), &mut a())
                .unwrap()
                .intervals()
                .rows(),
            Some(&[[d(2, 2, 0)]])
        );
        assert_eq!(report.disposition(), ResidualDisposition::NotExcluded);
        let mut invalid = program.source().definition().clone();
        invalid.outputs[0] = invalid.nodes.len();
        assert!(matches!(
            ExpressionProgram::from_definition(invalid, &mut g()),
            Err(ExpressionError::InvalidOutput { .. })
        ));
        return;
    }
    let output = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            "definitions_rebuild_in_a_fresh_process_without_live_evidence",
            "--nocapture",
        ])
        .env(MARKER, "2,3,1,2")
        .output()
        .unwrap();
    assert!(
        output.status.success(),
        "{}\n{}",
        String::from_utf8_lossy(&output.stdout),
        String::from_utf8_lossy(&output.stderr)
    );
}

#[cfg(feature = "dispatch-trace")]
#[test]
fn proof_getters_preserve_zero_dispatch() {
    let report = resistor()
        .enclose(&[d(1, 2, 0), d(1, 2, 0)], -16, &mut g(), &mut a())
        .unwrap();
    hyperreal::dispatch_trace::reset();
    let _recording = hyperreal::dispatch_trace::recording_scope();
    std::hint::black_box(report.circuit().source());
    std::hint::black_box(report.circuit().inputs());
    std::hint::black_box(report.circuit().equations());
    std::hint::black_box(report.circuit().jacobian());
    std::hint::black_box(report.evidence());
    std::hint::black_box(report.residuals());
    std::hint::black_box(report.disposition());
    assert!(hyperreal::dispatch_trace::snapshot().is_empty());
}

#[test]
fn construction_cutoffs_and_algebraic_coefficients_keep_exact_source() {
    let root = Real::from(2).sqrt().unwrap();
    let system = LinearMnaSystem {
        unknowns: vec![MnaUnknown::NetVoltage(net("algebraic"))],
        matrix: vec![vec![root.clone()]],
        rhs: vec![Real::zero()],
    };
    let mut measured = g();
    let program = system.residual_program(&mut measured).unwrap();
    let spent = g().remaining_work() - measured.remaining_work();
    assert!(matches!(
        system.residual_program(&mut ExpressionBudget::new(100_000, spent - 1)),
        Err(ResidualError::Expression(ExpressionError::WorkExhausted))
    ));
    assert!(matches!(
        system.residual_program(&mut ExpressionBudget::new(0, 10_000_000)),
        Err(ResidualError::Expression(ExpressionError::SizeExhausted))
    ));
    let token = CancellationToken::new();
    token.cancel();
    let mut cancelled = g();
    cancelled.cancellation = Some(token);
    assert!(matches!(
        system.residual_program(&mut cancelled),
        Err(ResidualError::Expression(ExpressionError::Cancelled))
    ));
    let exact = program
        .source()
        .eval_real(&[Real::one()], &mut g())
        .unwrap();
    assert_eq!(exact, vec![root]);
    let report = program
        .enclose(&[d(1, 1, 0)], -24, &mut g(), &mut a())
        .unwrap();
    let bounds = report.residuals()[0].to_rational(&mut a()).unwrap();
    // Independent positive-square isolator, retaining the exact sqrt(2) source.
    let lower = bounds.lower().unwrap();
    let upper = bounds.upper().unwrap();
    assert!(lower > &hyperreal::Rational::from(0));
    assert!(lower.clone() * lower.clone() <= hyperreal::Rational::from(2));
    assert!(upper.clone() * upper.clone() >= hyperreal::Rational::from(2));
    assert_eq!(
        report.disposition(),
        ResidualDisposition::Excluded { equation: 0 }
    );
    assert_eq!(
        system
            .residual_program(&mut g())
            .unwrap()
            .source()
            .definition(),
        program.source().definition()
    );
}
