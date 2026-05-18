use hyperphysics::{
    HeatSource3, LumpedRcThermalStepReport, LumpedThermalNode, PhysicsError, Real,
    SteadySlabConductionReport, TemperatureField3, ThermalContactPair3, ThermalMaterial,
    ThermalPolicy, ThermalPort3, ThermalReportStatus, TransientThermalStepReport,
};
use hyperreal::Rational;
use proptest::prelude::*;

fn r(value: i32) -> Real {
    value.into()
}

fn q(numerator: i64, denominator: u64) -> Real {
    Real::new(Rational::fraction(numerator, denominator).unwrap())
}

#[test]
fn exact_slab_conduction_matches_fourier_closed_form() {
    let copper = ThermalMaterial::new("copper", r(400)).unwrap();
    let report =
        SteadySlabConductionReport::through_slab(copper, r(2), r(4), r(310), r(300), Real::zero())
            .unwrap();

    assert_eq!(report.heat_rate, r(8000));
    assert_eq!(report.equivalent_resistance, q(1, 800));
    assert_eq!(report.status, ThermalReportStatus::Exact);
}

#[test]
fn contact_resistance_reduces_heat_rate_exactly() {
    let material = ThermalMaterial::new("fixture", r(10)).unwrap();
    let report =
        SteadySlabConductionReport::through_slab(material, r(1), r(1), r(110), r(100), q(9, 10))
            .unwrap();

    assert_eq!(report.equivalent_resistance, r(1));
    assert_eq!(report.heat_rate, r(10));
}

#[test]
fn joule_heating_and_thermal_ports_keep_coupling_handles() {
    let source = SteadySlabConductionReport::joule_heating("trace-r1", r(3), r(2)).unwrap();
    assert_eq!(source.power, r(18));

    let port = ThermalPort3 {
        handle: "thermal:trace-r1".into(),
        domain: "hypercircuit".into(),
        temperature: TemperatureField3 {
            source: "sensor".into(),
            temperature: r(300),
            status: ThermalReportStatus::Exact,
        },
        heat_source: Some(HeatSource3 {
            source: source.source,
            power: source.power,
            status: source.status,
        }),
    };
    assert_eq!(port.domain, "hypercircuit");
    assert_eq!(port.heat_source.unwrap().power, r(18));
}

#[test]
fn invalid_thermal_inputs_are_rejected_before_report_creation() {
    assert_eq!(
        ThermalMaterial::new("bad", r(0)).unwrap_err(),
        PhysicsError::NonPositiveThermalConductivity
    );
    assert_eq!(
        ThermalContactPair3::new("a", "b", r(0), r(0)).unwrap_err(),
        PhysicsError::NonPositiveArea
    );
    assert_eq!(
        SteadySlabConductionReport::through_slab(
            ThermalMaterial::new("ok", r(1)).unwrap(),
            r(1),
            r(1),
            r(1),
            r(0),
            r(-1),
        )
        .unwrap_err(),
        PhysicsError::NegativeResistance
    );
}

#[test]
fn transient_energy_balance_step_is_exact() {
    let node = LumpedThermalNode::new("chip", r(300), r(10)).unwrap();

    let report = TransientThermalStepReport::energy_balance_step(node, r(2), r(15)).unwrap();

    assert_eq!(report.temperature_delta, r(3));
    assert_eq!(report.next_temperature, r(303));
    assert_eq!(report.policy, ThermalPolicy::TransientHeatEquationStep);
    assert_eq!(report.status, ThermalReportStatus::Exact);
}

#[test]
fn lumped_rc_step_reports_conduction_and_source_terms() {
    let node = LumpedThermalNode::new("resistor", r(330), r(10)).unwrap();

    let report =
        LumpedRcThermalStepReport::explicit_euler_step(node, r(300), r(3), r(20), r(2)).unwrap();

    assert_eq!(report.conductive_heat_rate, r(-10));
    assert_eq!(report.net_heat_rate, r(10));
    assert_eq!(report.temperature_delta, r(2));
    assert_eq!(report.next_temperature, r(332));
    assert_eq!(report.policy, ThermalPolicy::LumpedRcNetwork);
}

#[test]
fn transient_reports_reject_invalid_domains() {
    assert_eq!(
        LumpedThermalNode::new("bad", r(300), r(0)).unwrap_err(),
        PhysicsError::NonPositiveThermalCapacitance
    );
    let node = LumpedThermalNode::new("ok", r(300), r(1)).unwrap();
    assert_eq!(
        TransientThermalStepReport::energy_balance_step(node.clone(), r(0), r(1)).unwrap_err(),
        PhysicsError::NonPositiveTimeStep
    );
    assert_eq!(
        LumpedRcThermalStepReport::explicit_euler_step(node, r(300), r(0), r(1), r(1)).unwrap_err(),
        PhysicsError::InvalidThermalDenominator
    );
}

proptest! {
    #[test]
    fn generated_slab_heat_rate_matches_exact_formula(
        conductivity in 1_i32..50,
        area in 1_i32..20,
        thickness in 1_i32..20,
        delta in 1_i32..100,
    ) {
        let material = ThermalMaterial::new("generated", r(conductivity)).unwrap();
        let report = SteadySlabConductionReport::through_slab(
            material,
            r(thickness),
            r(area),
            r(300 + delta),
            r(300),
            Real::zero(),
        ).unwrap();
        let expected = q(
            i64::from(conductivity * area * delta),
            u64::from(thickness as u32),
        );
        prop_assert_eq!(report.heat_rate, expected);
    }

    #[test]
    fn generated_lumped_rc_matches_energy_balance(
        heat_capacity in 1_i32..100,
        resistance in 1_i32..50,
        time_step in 1_i32..20,
        delta in -100_i32..100,
        source in -100_i32..100,
    ) {
        let ambient = r(300);
        let node = LumpedThermalNode::new("generated", r(300 + delta), r(heat_capacity)).unwrap();

        let report = LumpedRcThermalStepReport::explicit_euler_step(
            node,
            ambient,
            r(resistance),
            r(source),
            r(time_step),
        ).unwrap();

        let conductive = q(-i64::from(delta), u64::from(resistance as u32));
        let expected_net = r(source) + conductive;
        let expected_delta = (&r(time_step) * &expected_net) / &r(heat_capacity);
        prop_assert_eq!(report.net_heat_rate, expected_net);
        prop_assert_eq!(report.temperature_delta, expected_delta.unwrap());
    }
}
