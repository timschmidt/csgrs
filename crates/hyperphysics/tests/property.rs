use hyperphysics::{
    ElasticDerivationReport, ExternalReplacementStatus, MaterialAssertion, MaterialPropertyGraph,
    MaterialPropertyKind, MaterialState, PhysicsError, PropertyResolutionStatus, PropertyValue,
    Real, SourceSpec,
};
use hyperreal::Rational;
use proptest::prelude::*;

fn r(value: i32) -> Real {
    value.into()
}

fn q(numerator: i64, denominator: u64) -> Real {
    Real::new(Rational::fraction(numerator, denominator).unwrap())
}

fn source(locator: &str) -> SourceSpec {
    SourceSpec::new("test-datasheet", locator)
}

fn assertion(
    kind: MaterialPropertyKind,
    value: PropertyValue,
    unit: &str,
    locator: &str,
) -> MaterialAssertion {
    MaterialAssertion {
        kind,
        value,
        unit: unit.into(),
        state: MaterialState::Solid,
        condition: None,
        source: source(locator),
    }
}

#[test]
fn agreeing_exact_assertions_resolve_to_exact_known() {
    let mut graph = MaterialPropertyGraph::default();
    graph.push(assertion(
        MaterialPropertyKind::Density,
        PropertyValue::exact_scalar(r(7850)),
        "kg/m^3",
        "density-a",
    ));
    graph.push(assertion(
        MaterialPropertyKind::Density,
        PropertyValue::exact_scalar(r(7850)),
        "kg/m^3",
        "density-b",
    ));

    let report = graph.resolve(&MaterialPropertyKind::Density);

    assert_eq!(report.status, PropertyResolutionStatus::ExactKnown);
    assert_eq!(report.value, Some(PropertyValue::exact_scalar(r(7850))));
    assert_eq!(report.sources.len(), 2);
}

#[test]
fn conflicting_exact_assertions_are_not_silently_averaged() {
    let mut graph = MaterialPropertyGraph::default();
    graph.push(assertion(
        MaterialPropertyKind::ThermalConductivity,
        PropertyValue::exact_scalar(r(10)),
        "W/(m*K)",
        "conductivity-a",
    ));
    graph.push(assertion(
        MaterialPropertyKind::ThermalConductivity,
        PropertyValue::exact_scalar(r(11)),
        "W/(m*K)",
        "conductivity-b",
    ));

    let report = graph.resolve(&MaterialPropertyKind::ThermalConductivity);

    assert_eq!(report.status, PropertyResolutionStatus::Conflict);
    assert_eq!(report.value, None);
    assert!(
        report
            .evidence
            .iter()
            .any(|line| line.contains("conflicting"))
    );
}

#[test]
fn interval_values_are_validated_and_resolved_without_point_guessing() {
    let interval = PropertyValue::interval(r(3), r(5)).unwrap();
    let mut graph = MaterialPropertyGraph::default();
    graph.push(assertion(
        MaterialPropertyKind::RefractiveIndex,
        interval.clone(),
        "1",
        "index-band",
    ));

    let report = graph.resolve(&MaterialPropertyKind::RefractiveIndex);

    assert_eq!(report.status, PropertyResolutionStatus::Interval);
    assert_eq!(report.value, Some(interval));
    assert_eq!(
        PropertyValue::interval(r(6), r(5)),
        Err(PhysicsError::InvalidPropertyInterval)
    );
}

#[test]
fn external_proposals_keep_replacement_status_visible() {
    let mut graph = MaterialPropertyGraph::default();
    graph.push(assertion(
        MaterialPropertyKind::Viscosity,
        PropertyValue::external_proposal(q(3, 2), ExternalReplacementStatus::ReplacementPlanned),
        "Pa*s",
        "legacy-fit",
    ));

    let report = graph.resolve(&MaterialPropertyKind::Viscosity);

    assert_eq!(report.status, PropertyResolutionStatus::ExternalProposal);
    assert_eq!(
        report.value,
        Some(PropertyValue::external_proposal(
            q(3, 2),
            ExternalReplacementStatus::ReplacementPlanned
        ))
    );
}

#[test]
fn unknown_assertions_do_not_create_defaults() {
    let mut graph = MaterialPropertyGraph::default();
    graph.push(assertion(
        MaterialPropertyKind::ElectricalConductivity,
        PropertyValue::Unknown,
        "S/m",
        "missing-table-cell",
    ));

    let report = graph.resolve(&MaterialPropertyKind::ElectricalConductivity);

    assert_eq!(report.status, PropertyResolutionStatus::Unknown);
    assert_eq!(report.value, None);
}

#[test]
fn isotropic_shear_modulus_derivation_carries_sources_and_assumption() {
    let mut graph = MaterialPropertyGraph::default();
    graph.push(assertion(
        MaterialPropertyKind::YoungModulus,
        PropertyValue::exact_scalar(r(200)),
        "GPa",
        "young",
    ));
    graph.push(assertion(
        MaterialPropertyKind::PoissonRatio,
        PropertyValue::exact_scalar(q(1, 4)),
        "1",
        "poisson",
    ));

    let report: ElasticDerivationReport = graph.derive_isotropic_shear_modulus().unwrap().unwrap();

    assert_eq!(report.kind, MaterialPropertyKind::ShearModulus);
    assert_eq!(report.value, r(80));
    assert_eq!(report.sources.len(), 2);
    assert!(report.assumption.contains("isotropic"));
}

#[test]
fn invalid_elastic_constants_are_rejected() {
    let mut zero_young = MaterialPropertyGraph::default();
    zero_young.push(assertion(
        MaterialPropertyKind::YoungModulus,
        PropertyValue::exact_scalar(Real::zero()),
        "GPa",
        "zero-young",
    ));
    zero_young.push(assertion(
        MaterialPropertyKind::PoissonRatio,
        PropertyValue::exact_scalar(q(1, 4)),
        "1",
        "poisson",
    ));

    assert_eq!(
        zero_young.derive_isotropic_shear_modulus(),
        Err(PhysicsError::InvalidElasticConstant)
    );

    let mut singular_poisson = MaterialPropertyGraph::default();
    singular_poisson.push(assertion(
        MaterialPropertyKind::YoungModulus,
        PropertyValue::exact_scalar(r(200)),
        "GPa",
        "young",
    ));
    singular_poisson.push(assertion(
        MaterialPropertyKind::PoissonRatio,
        PropertyValue::exact_scalar(r(-1)),
        "1",
        "singular-poisson",
    ));

    assert_eq!(
        singular_poisson.derive_isotropic_shear_modulus(),
        Err(PhysicsError::InvalidElasticConstant)
    );
}

proptest! {
    #[test]
    fn generated_agreeing_exact_values_resolve(value in -10_000_i32..10_000_i32) {
        let mut graph = MaterialPropertyGraph::default();
        graph.push(assertion(
            MaterialPropertyKind::Custom("generated".into()),
            PropertyValue::exact_scalar(r(value)),
            "arb",
            "generated-a",
        ));
        graph.push(assertion(
            MaterialPropertyKind::Custom("generated".into()),
            PropertyValue::exact_scalar(r(value)),
            "arb",
            "generated-b",
        ));

        let report = graph.resolve(&MaterialPropertyKind::Custom("generated".into()));

        prop_assert_eq!(report.status, PropertyResolutionStatus::ExactKnown);
        prop_assert_eq!(report.value, Some(PropertyValue::exact_scalar(r(value))));
    }

    #[test]
    fn generated_descending_intervals_are_rejected(lower in -10_000_i32..10_000_i32, gap in 1_i32..10_000_i32) {
        let upper = lower - gap;

        prop_assert_eq!(
            PropertyValue::interval(r(lower), r(upper)),
            Err(PhysicsError::InvalidPropertyInterval)
        );
    }
}
