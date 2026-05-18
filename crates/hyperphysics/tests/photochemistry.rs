use hyperphysics::{
    CureStatus, ExposureMode, PhotochemicalConcentrations, PhotochemicalPolicy, PhysicsError,
    ReactionDiffusionState, ReactionDiffusionTransport, Real, VatPhotopolymerWorkingCurve,
};
use proptest::prelude::*;

fn r(value: i32) -> Real {
    value.into()
}

fn concentrations() -> PhotochemicalConcentrations {
    PhotochemicalConcentrations::new(r(1), r(2), r(0), r(0)).unwrap()
}

#[test]
fn working_curve_replay_collapses_exact_e_ratio_log() {
    let setup = VatPhotopolymerWorkingCurve::new(
        "jacobs-fixture",
        Real::e(),
        r(1),
        r(2),
        r(1),
        concentrations(),
        ExposureMode::OnePhoton,
    )
    .unwrap();

    let report = setup.replay().unwrap();

    assert_eq!(report.exposure_ratio, Real::e());
    assert_eq!(report.cure_depth, r(2));
    assert_eq!(report.decision.status, CureStatus::ClearsLayer);
    assert_eq!(report.decision.margin, Some(r(1)));
    assert_eq!(report.policy, PhotochemicalPolicy::WorkingCurveReplay);
    assert_eq!(report.expression, "C_d = D_p ln(E / E_c)");
}

#[test]
fn working_curve_reports_under_cure_for_threshold_exposure() {
    let setup = VatPhotopolymerWorkingCurve::new(
        "threshold",
        r(5),
        r(5),
        r(10),
        r(1),
        concentrations(),
        ExposureMode::ClipDeadZone,
    )
    .unwrap();

    let report = setup.replay().unwrap();

    assert_eq!(report.exposure_ratio, r(1));
    assert_eq!(report.cure_depth, Real::zero());
    assert_eq!(report.decision.status, CureStatus::UnderCured);
    assert_eq!(report.decision.margin, Some(r(-1)));
}

#[test]
fn invalid_photochemical_inputs_are_rejected() {
    assert_eq!(
        PhotochemicalConcentrations::new(r(-1), r(0), r(0), r(0)).unwrap_err(),
        PhysicsError::NegativeConcentration
    );
    assert_eq!(
        VatPhotopolymerWorkingCurve::new(
            "bad-exposure",
            r(0),
            r(1),
            r(1),
            r(1),
            concentrations(),
            ExposureMode::TwoPhoton,
        )
        .unwrap_err(),
        PhysicsError::NonPositiveExposure
    );
    assert_eq!(
        VatPhotopolymerWorkingCurve::new(
            "bad-depth",
            r(1),
            r(1),
            r(0),
            r(1),
            concentrations(),
            ExposureMode::Multiphoton,
        )
        .unwrap_err(),
        PhysicsError::NonPositivePenetrationDepth
    );
}

#[test]
fn reaction_diffusion_state_rejects_invalid_fractions_and_negative_transport() {
    assert_eq!(
        ReactionDiffusionState::new(
            "bad-conversion",
            concentrations(),
            r(2),
            r(0),
            r(0),
            r(0),
            r(1),
        )
        .unwrap_err(),
        PhysicsError::InvalidFraction
    );
    assert_eq!(
        ReactionDiffusionTransport::new(r(0), r(0), r(-1), r(0)).unwrap_err(),
        PhysicsError::NegativeDiffusionCoefficient
    );
}

#[test]
fn diffusive_courant_report_is_exact_for_all_species() {
    let transport = ReactionDiffusionTransport::new(r(1), r(2), r(3), r(4)).unwrap();

    let report = transport.diffusive_courant_report(r(2), r(2)).unwrap();

    assert_eq!(
        report.absorber,
        Real::new(hyperreal::Rational::fraction(1, 2).unwrap())
    );
    assert_eq!(report.initiator, r(1));
    assert_eq!(
        report.oxygen,
        Real::new(hyperreal::Rational::fraction(3, 2).unwrap())
    );
    assert_eq!(report.inhibitor, r(2));
    assert_eq!(report.expression, "D dt / h^2");
    assert_eq!(report.policy, PhotochemicalPolicy::ReactionDiffusionAdapter);
}

proptest! {
    #[test]
    fn generated_threshold_exposures_have_zero_cure_depth(
        exposure in 1_i32..100,
        penetration in 1_i32..100,
        layer in 1_i32..100,
    ) {
        let setup = VatPhotopolymerWorkingCurve::new(
            "generated-threshold",
            r(exposure),
            r(exposure),
            r(penetration),
            r(layer),
            concentrations(),
            ExposureMode::ComputedAxialVolumetric,
        ).unwrap();
        let report = setup.replay().unwrap();
        prop_assert_eq!(report.cure_depth, Real::zero());
        prop_assert_eq!(report.decision.status, CureStatus::UnderCured);
    }

    #[test]
    fn generated_diffusion_numbers_are_d_dt_over_h_squared(
        diffusion in 0_i32..100,
        dt in 1_i32..100,
        h in 1_i32..20,
    ) {
        let transport = ReactionDiffusionTransport::new(r(diffusion), r(0), r(0), r(0)).unwrap();

        let report = transport.diffusive_courant_report(r(dt), r(h)).unwrap();

        let expected = (&r(diffusion) * &r(dt)) / &(&r(h) * &r(h));
        prop_assert_eq!(report.absorber, expected.unwrap());
    }
}
