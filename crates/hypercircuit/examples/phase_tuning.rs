use hypercircuit::{
    BoardOutline, Design, DifferentialPairRule, PcbStackup, PhaseTuningGroupId, PhaseTuningStatus,
    PhaseTuningSynthesisPolicy, PhaseTuningSynthesisStatus, Real, Route,
};
use hyperlattice::Point2;
use hyperpath::TraceLayer;

fn point(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut design = Design::new(
        "phase-tuned-pair",
        BoardOutline::rectangle(Real::from(12), Real::from(10)),
        PcbStackup::single_layer(Real::one(), None),
    )?;
    let positive = design.signal("DATA+")?;
    let negative = design.signal("DATA-")?;
    let pair = design.differential_pair(
        DifferentialPairRule::new("data", &positive, &negative, Real::one()).max_skew(Real::one()),
    )?;
    design.route(
        &positive,
        Route::new("data-p", TraceLayer(0), Real::one()).line(point(2, 4), point(8, 4)),
    )?;
    design.route(
        &negative,
        Route::new("data-n", TraceLayer(0), Real::one()).line(point(2, 6), point(8, 6)),
    )?;

    let checked = design.finish()?;
    let synthesis = checked.layout.synthesize_phase_tuning(
        &checked.circuit,
        PhaseTuningSynthesisPolicy {
            target_length: Some(Real::from(10)),
            differential_pair: Some(pair.id().clone()),
            ..PhaseTuningSynthesisPolicy::new(
                PhaseTuningGroupId::new("data-phase")?,
                [positive.id().clone(), negative.id().clone()],
                Real::one(),
                Real::one(),
                2,
            )
        },
    );
    assert_eq!(synthesis.status, PhaseTuningSynthesisStatus::Certified);
    let retained = synthesis
        .apply_intent_to(&checked.circuit, &checked.layout)
        .expect("certified synthesis adds retained intent atomically");
    let report = retained.realize_phase_tuning(&checked.circuit, &synthesis.group_id);
    assert_eq!(report.status, PhaseTuningStatus::Applied);
    assert_eq!(report.realized_skew, Some(Real::zero()));
    let tuned = report
        .apply_to(&retained)
        .expect("accepted group applies every route atomically");
    let replay = tuned.realize_phase_tuning(&checked.circuit, &synthesis.group_id);
    assert_eq!(replay.status, PhaseTuningStatus::AlreadySatisfied);

    println!(
        "phase group {} tuned {} routes to exact zero skew",
        synthesis.group_id.as_str(),
        report.tuned_routes.len()
    );
    Ok(())
}
