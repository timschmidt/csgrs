use super::*;
use hyperdynamics::field::coordinates::{QrCoordinateField, QrFieldError};
use hyperreal::Real;
fn basis() -> Vec<Vec<Real>> {
    vec![vec![3.into(), (-4).into()], vec![4.into(), 3.into()]]
}
fn ratio(n: i64, d: i64) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
fn rotated() -> ExpressionField<2> {
    let x = Expr::anonymous_symbol(X);
    let y = Expr::anonymous_symbol(Y);
    field(
        &[
            Expr::real(ratio(-7, 25)) * x.clone() + Expr::real(ratio(24, 25)) * y.clone(),
            Expr::real(ratio(24, 25)) * x + Expr::real(ratio(7, 25)) * y,
        ],
        vec![X, Y],
    )
}
#[test]
fn qr_frame_derivatives_follow_independent_exact_diagonalization() {
    let f = rotated();
    let qr = f
        .in_qr_coordinates(basis(), vec![0.into(), 0.into()], &mut budget())
        .unwrap();
    for x in -3..=3 {
        for y in -3..=3 {
            let values = qr
                .chart()
                .transformed_field()
                .image(
                    &d(0, 0),
                    &BoxN::new([d(x, x), d(y, y)]).unwrap(),
                    &mut budget(),
                )
                .unwrap();
            assert_eq!(
                qr.chart().transformed_field().identity().source().variables,
                vec![Y, X, T]
            );
            assert_eq!(
                qr.factorization()
                    .coordinates()
                    .program()
                    .eval_real(
                        &[y.into(), x.into(), 0.into()],
                        &mut ExpressionBudget::new(1_000_000, 100_000_000)
                    )
                    .unwrap(),
                vec![Real::from(x), Real::from(-y)]
            );
            for (lane, expected) in values.lanes().unwrap().iter().zip([d(x, x), d(-y, -y)]) {
                tight(lane, &expected);
            }
        }
    }
    let jac = qr
        .chart()
        .transformed_field()
        .jacobian(
            &d(0, 0),
            &BoxN::new([d(-1, 1), d(-1, 1)]).unwrap(),
            &mut budget(),
        )
        .unwrap();
    for (row, expected) in jac
        .rows()
        .unwrap()
        .iter()
        .zip([[d(1, 1), d(0, 0)], [d(0, 0), d(-1, -1)]])
    {
        for (lane, point) in row.iter().zip(expected) {
            tight(lane, &point);
        }
    }
    assert_eq!(
        qr.factorization().coordinates().program().definition(),
        qr.chart().transformation().program().definition()
    );
    assert_eq!(qr.chart().original_field().identity(), f.identity());
}
#[test]
fn qr_frame_certifies_a_long_step_and_its_endpoint_is_transported() {
    use hyperdynamics::field::picard::PicardFailure;
    let f = rotated();
    let qr = f
        .in_qr_coordinates(basis(), vec![0.into(), 0.into()], &mut budget())
        .unwrap();
    let end = DyadicInterval::point(7.into(), -3).unwrap();
    // Both attempts start from exactly the same singleton zero; no changed
    // input-set/width contract accounts for the contraction difference.
    let input = BoxN::new([d(0, 0), d(0, 0)]).unwrap();
    let proposal = PicardProposal {
        start: d(0, 0),
        end: end.clone(),
        initial: input.clone(),
        candidate: input,
    };
    let old = f.certify_step(proposal.clone(), &mut budget());
    assert!(matches!(
        old.status(),
        PicardStatus::Rejected(PicardFailure::ContractionNotCertified)
    ));
    let step = qr.chart().certify_step(proposal, &mut budget());
    assert!(
        matches!(step.status(), CoordinateStatus::Certified),
        "{:?}",
        step.status()
    );
    assert_eq!(
        step.source_end_enclosure().unwrap().lanes(),
        Some(&[d(0, 0), d(0, 0)])
    );
    tight(step.native().evidence().contraction().unwrap(), &end);
    let other = f
        .in_qr_coordinates(
            vec![vec![1.into(), 0.into()], vec![0.into(), 1.into()]],
            vec![1.into(), (-2).into()],
            &mut budget(),
        )
        .unwrap();
    let transported = qr
        .chart()
        .transition_to(other.chart(), &mut budget())
        .unwrap()
        .enclose(
            step.native().end_enclosure().unwrap().clone(),
            &mut budget(),
        )
        .unwrap();
    assert_eq!(transported.enclosure().lanes(), Some(&[d(-1, -1), d(2, 2)]));
    let next = other.chart().certify_step(
        PicardProposal {
            start: end,
            end: d(1, 1),
            initial: transported.enclosure().clone(),
            candidate: transported.enclosure().clone(),
        },
        &mut budget(),
    );
    assert!(matches!(next.status(), CoordinateStatus::Certified));
    assert_eq!(
        next.source_end_enclosure().unwrap().lanes(),
        Some(&[d(0, 0), d(0, 0)])
    );
}
fn constant_chart() -> QrCoordinateField<2> {
    field(&[Expr::int(2), Expr::int(1)], vec![X, Y])
        .in_qr_coordinates(basis(), vec![10.into(), (-3).into()], &mut budget())
        .unwrap()
}
#[test]
fn uncertain_initial_set_empty_unit_and_tight_candidate_rejection() {
    let qr = constant_chart();
    let tight_candidate = PicardProposal {
        start: d(0, 0),
        end: d(1, 1),
        initial: BoxN::new([d(-1, 1), d(-1, 1)]).unwrap(),
        candidate: BoxN::new([d(-1, 3), d(-2, 1)]).unwrap(),
    };
    assert!(matches!(
        qr.chart()
            .certify_step(tight_candidate.clone(), &mut budget())
            .status(),
        CoordinateStatus::Rejected(
            hyperdynamics::field::picard::PicardFailure::SelfMapNotCertified
        )
    ));
    let step = qr.chart().certify_step(
        PicardProposal {
            candidate: BoxN::new([d(-2, 4), d(-3, 2)]).unwrap(),
            ..tight_candidate
        },
        &mut budget(),
    );
    assert!(
        matches!(step.status(), CoordinateStatus::Certified),
        "{:?}",
        step.status()
    );
    // Source x(1)=12+(3a-4b)/5, y(1)=-2+(4a+3b)/5 for a,b in [-1,1].
    let expected = [(53, 67), (-17, -3)].map(|(l, u)| {
        (
            hyperreal::Rational::from(l) / hyperreal::Rational::from(5),
            hyperreal::Rational::from(u) / hyperreal::Rational::from(5),
        )
    });
    for (lane, (low, high)) in step
        .source_end_enclosure()
        .unwrap()
        .lanes()
        .unwrap()
        .iter()
        .zip(expected)
    {
        let mut scalar = hyperinterval::DyadicBudget::new(16384, 1_000_000_000);
        let bounds = lane.to_rational(&mut scalar).unwrap();
        assert!(bounds.contains_point(&low));
        assert!(bounds.contains_point(&high));
    }
    let empty = qr.chart().certify_step(
        PicardProposal {
            initial: BoxN::empty().unwrap(),
            ..proposal()
        },
        &mut budget(),
    );
    assert!(matches!(empty.status(), CoordinateStatus::Empty));
    let unit = field::<0>(&[], vec![])
        .in_qr_coordinates(vec![], vec![], &mut budget())
        .unwrap();
    let step = unit.chart().certify_step(
        PicardProposal {
            start: d(0, 0),
            end: d(1, 1),
            initial: BoxN::new([]).unwrap(),
            candidate: BoxN::new([]).unwrap(),
        },
        &mut budget(),
    );
    assert!(matches!(step.status(), CoordinateStatus::Certified));
    assert!(!step.source_end_enclosure().unwrap().is_empty());
}
#[test]
fn field_construction_final_cutoff_and_original_errors_are_preserved() {
    let f = field::<2>(&[Expr::int(2), Expr::int(1)], vec![X, Y]);
    let mut full = budget();
    f.in_qr_coordinates(basis(), vec![10.into(), (-3).into()], &mut full)
        .unwrap();
    let used = EnclosurePolicy::default().graph_work - full.remaining_graph_work();
    let mut short = EnclosureBudget::new(EnclosurePolicy {
        graph_work: used - 1,
        ..EnclosurePolicy::default()
    });
    assert!(matches!(
        f.in_qr_coordinates(basis(), vec![10.into(), (-3).into()], &mut short),
        Err(QrFieldError::Field(
            hyperdynamics::field::FieldError::Expression(
                hypersolve::expression_program::ExpressionError::WorkExhausted
            )
        ))
    ));
    let mut exact = EnclosureBudget::new(EnclosurePolicy {
        graph_work: used,
        ..EnclosurePolicy::default()
    });
    f.in_qr_coordinates(basis(), vec![10.into(), (-3).into()], &mut exact)
        .unwrap();
    assert_eq!(exact.remaining_graph_work(), 0);
    assert!(matches!(
        f.in_qr_coordinates(
            vec![vec![0.into(), 0.into()], vec![0.into(), 0.into()]],
            vec![0.into(), 0.into()],
            &mut budget()
        ),
        Err(QrFieldError::Qr(
            hypersolve::expression_program::QrCoordinateError::DependentColumn { .. }
        ))
    ));
    let token = hypersolve::work_budget::CancellationToken::new();
    token.cancel();
    let mut b = budget();
    b.set_cancellation(Some(token));
    assert!(
        f.in_qr_coordinates(basis(), vec![0.into(), 0.into()], &mut b)
            .is_err()
    );
    f.in_qr_coordinates(basis(), vec![0.into(), 0.into()], &mut budget())
        .unwrap();
}

fn tight(lane: &DyadicInterval, expected: &DyadicInterval) {
    let mut work = hyperinterval::DyadicBudget::new(16384, 1_000_000_000);
    assert!(expected.subset_of(lane, &mut work).unwrap());
    assert!(
        lane.width(&mut work)
            .unwrap()
            .compare_singleton(&DyadicInterval::point(1.into(), -48).unwrap(), &mut work)
            .unwrap()
            != std::cmp::Ordering::Greater
    );
}
#[cfg(feature = "taylor-flow")]
#[test]
fn qr_generated_taylor_endpoints_reframe_and_continue() {
    use hyperdynamics::field::taylor_flow::FlowTaylorStatus;
    let qr = constant_chart();
    let f = qr
        .chart()
        .transformed_field()
        .flow_taylor(2, &mut budget())
        .unwrap();
    let first = f.certify_step(
        PicardProposal {
            start: d(0, 0),
            end: d(1, 1),
            initial: BoxN::new([d(-1, 1), d(-1, 1)]).unwrap(),
            candidate: BoxN::new([d(-2, 4), d(-3, 2)]).unwrap(),
        },
        &mut budget(),
    );
    assert!(matches!(first.status(), FlowTaylorStatus::Certified));
    let next = qr
        .chart()
        .original_field()
        .in_qr_coordinates(basis(), vec![12.into(), (-2).into()], &mut budget())
        .unwrap();
    let transport = qr
        .chart()
        .transition_to(next.chart(), &mut budget())
        .unwrap()
        .enclose(first.end_enclosure().unwrap().clone(), &mut budget())
        .unwrap();
    let second = next
        .chart()
        .transformed_field()
        .flow_taylor(2, &mut budget())
        .unwrap()
        .certify_step(
            PicardProposal {
                start: d(1, 1),
                end: d(2, 2),
                initial: transport.enclosure().clone(),
                candidate: BoxN::new([d(-2, 4), d(-3, 2)]).unwrap(),
            },
            &mut budget(),
        );
    assert!(
        matches!(second.status(), FlowTaylorStatus::Certified),
        "{:?}",
        second.status()
    );
    let physical = next
        .chart()
        .source_enclosure(second.end_enclosure().unwrap(), &mut budget())
        .unwrap();
    for (lane, (l, u)) in physical.lanes().unwrap().iter().zip([(63, 77), (-12, 2)]) {
        let range = lane
            .to_rational(&mut hyperinterval::DyadicBudget::new(16384, 1_000_000_000))
            .unwrap();
        assert!(
            range.contains_point(&(hyperreal::Rational::from(l) / hyperreal::Rational::from(5)))
        );
        assert!(
            range.contains_point(&(hyperreal::Rational::from(u) / hyperreal::Rational::from(5)))
        );
    }
}
#[cfg(feature = "dispatch-trace")]
#[test]
fn qr_evidence_getters_do_not_repeat_predicates() {
    use hyperreal::dispatch_trace;
    let qr = constant_chart();
    dispatch_trace::reset();
    dispatch_trace::with_recording(|| {
        let f = qr.factorization();
        std::hint::black_box(f.request());
        std::hint::black_box(f.upper());
        std::hint::black_box(f.squared_norms());
        std::hint::black_box(f.norm_evidence());
        std::hint::black_box(f.reconstruction_evidence());
        std::hint::black_box(f.coordinates());
        std::hint::black_box(qr.chart());
    });
    let trace = dispatch_trace::take_trace();
    assert!(trace.dispatch.is_empty());
    assert_eq!(trace.rational, Default::default());
}
#[cfg(feature = "serde")]
#[test]
fn qr_field_replays_in_fresh_processes_without_serialized_certificates() {
    const KEY: &str = "HYPERDYNAMICS_QR_REPLAY";
    if let Ok(data) = std::env::var(KEY) {
        let (definition, request): (
            FieldDefinition,
            hypersolve::expression_program::QrCoordinateRequest,
        ) = serde_json::from_str(&data).unwrap();
        let f = ExpressionField::<2>::from_definition(definition, &mut budget()).unwrap();
        let mut invalid = request.basis.clone();
        for row in &mut invalid {
            row[1] = row[0].clone();
        }
        assert!(matches!(
            f.in_qr_coordinates(invalid, request.origin.clone(), &mut budget()),
            Err(QrFieldError::Qr(
                hypersolve::expression_program::QrCoordinateError::DependentColumn { .. }
            ))
        ));
        let mut work = budget();
        let qr = f
            .in_qr_coordinates(request.basis, request.origin, &mut work)
            .unwrap();
        let step = qr.chart().certify_step(
            PicardProposal {
                start: d(0, 0),
                end: d(1, 1),
                initial: BoxN::new([d(0, 0), d(0, 0)]).unwrap(),
                candidate: BoxN::new([d(-1, 3), d(-2, 1)]).unwrap(),
            },
            &mut work,
        );
        assert!(matches!(step.status(), CoordinateStatus::Certified));
        println!(
            "QR {:?} STEP {:?} GRAPH {} SCALAR {}",
            qr,
            step,
            work.remaining_graph_work(),
            work.remaining_scalar_work()
        );
        return;
    }
    let qr = constant_chart();
    let id = qr.chart().original_field().identity();
    let definition = FieldDefinition {
        source: id.source().clone(),
        time: id.time(),
        state: id.state().to_vec(),
    };
    let data = serde_json::to_string(&(definition, qr.factorization().request())).unwrap();
    let mut previous = None;
    for _ in 0..2 {
        let child = std::process::Command::new(crate::replay::binary().unwrap())
            .args([
                "--exact",
                "qr::qr_field_replays_in_fresh_processes_without_serialized_certificates",
                "--nocapture",
            ])
            .env(KEY, &data)
            .output()
            .unwrap();
        assert!(
            child.status.success(),
            "{} {}",
            String::from_utf8_lossy(&child.stdout),
            String::from_utf8_lossy(&child.stderr)
        );
        let out = String::from_utf8(child.stdout).unwrap();
        let line = out
            .lines()
            .find(|line| line.starts_with("QR "))
            .unwrap()
            .to_owned();
        if let Some(old) = previous {
            assert_eq!(line, old);
        }
        previous = Some(line);
    }
}
