#![cfg(feature = "dispatch-trace")]

use hyperparts::{
    CircuitJsonSourceRecord, EdaAuthoringBundle, EdaExactField, EdaIntakeStatus, PartId, SourceRef,
    VariantId, import_eda_authoring_bundle,
};

#[test]
fn exact_eda_numeric_intake_does_not_request_approximation() {
    let bundle = EdaAuthoringBundle {
        source: SourceRef::new("trace", "exact-circuit-json").unwrap(),
        part: PartId::new("eda:trace-board").unwrap(),
        variant: VariantId::new("A").unwrap(),
        display_name: "Trace Board".into(),
        circuit_records: vec![CircuitJsonSourceRecord {
            id: "R1".into(),
            kind: "resistor".into(),
            reference: Some("R1".into()),
            nets: vec!["A".into(), "B".into()],
            exact_fields: vec![
                EdaExactField {
                    field: "resistance".into(),
                    value: Some("100/300".into()),
                    unit: Some("ohm".into()),
                },
                EdaExactField {
                    field: "tolerance".into(),
                    value: Some("0.125".into()),
                    unit: Some("%".into()),
                },
            ],
        }],
        footprint: None,
        model_references: Vec::new(),
        package: None,
        routes: Vec::new(),
        fabrication: Vec::new(),
    };

    hyperreal::dispatch_trace::reset();
    let _recording = hyperreal::dispatch_trace::recording_scope();
    let result = import_eda_authoring_bundle(bundle);
    assert_eq!(result.status, EdaIntakeStatus::Accepted);

    let correlation = hyperreal::dispatch_trace::snapshot_trace().correlation_summary();
    assert!(correlation.rational_reductions > 0);
    assert!(correlation.rational_gcds > 0);
    assert_eq!(correlation.approximation_events, 0);
    assert_eq!(correlation.unknown_fact_events, 0);
}
