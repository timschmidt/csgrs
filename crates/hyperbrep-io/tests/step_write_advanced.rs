use hyperbrep::{OperationContext, Real, WorkBudget, builder};
use hyperbrep_io::step::{
    ImportLimits, StepBody, StepFile, StepReadOptions, StepWriteOptions, write_advanced,
};

#[allow(dead_code)]
#[path = "support/analytic_export.rs"]
mod analytic;

#[test]
fn native_analytic_scaling_exports_and_reimports_in_exact_units() {
    analytic::scaled_units();
}

#[test]
fn closed_partial_reversed_and_supplied_pcurves_keep_exact_maps() {
    analytic::imported_circles();
}
#[test]
fn oblique_and_radical_frames_survive_subdivision_translation_and_reflection() {
    analytic::frames_and_edits();
}
#[test]
fn planar_holes_and_sheared_charts_preserve_explicit_topology() {
    analytic::planar_holes();
}
#[test]
fn output_units_occurrences_and_near_coincidence_remain_exact() {
    analytic::units_and_occurrences();
}
#[test]
fn construction_export_reimport_boolean_empty_and_recovery_compose() {
    analytic::composition();
}
#[test]
fn unsupported_and_nonrepresentable_geometry_fails_without_partial_output() {
    analytic::failures_and_recovery();
}
#[test]
fn inclusive_output_limits_are_typed_and_retryable() {
    analytic::limits();
}

#[test]
fn exact_cylinder_exports_reimports_and_remains_consumable() {
    let context = OperationContext::strict(WorkBudget {
        refinements: 10_000,
        promotions: 1000,
    });
    let source = builder::cylinder(Real::from(5), Real::from(7)).unwrap();
    let output = write_advanced(
        &[StepBody {
            solid: source.bodies().next().unwrap(),
            name: "native cylinder",
        }],
        &StepWriteOptions::new("cylinder.step", "2026-10-16T00:00:00"),
        &context,
    )
    .result
    .unwrap()
    .unwrap();
    let file = StepFile::read(output.text().as_bytes(), &StepReadOptions::default()).unwrap();
    let imported = file
        .import_representation(output.representation(), &ImportLimits::default(), &context)
        .result
        .unwrap()
        .unwrap();
    assert_eq!(imported.bodies().len(), 1);
    assert_eq!(
        (imported.bodies()[0].solid().volume().unwrap() - Real::from(175) * Real::pi())
            .exact_rational_normal_form(),
        Some(hyperreal::Rational::zero())
    );
    let native = imported.bodies()[0].model().to_json().unwrap();
    let replay = hyperbrep::RawModel::from_json(&native, &hyperbrep::ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(replay.to_json().unwrap(), native);
}
