#![no_main]

use hyperparts::{
    AutorouterOutputRecord, CircuitJsonSourceRecord, EdaAuthoringBundle, EdaExactField,
    EdaFabricationReadiness, EdaFootprintString, EdaModelStatus, EdaPackageMetadata, EdaPackagePin,
    EdaRouteStatus, FabricationOutputRecord, GeneratedModelReference, PartId, PinFunction,
    ProcessKind, SourceRef, VariantId, import_eda_authoring_bundle,
};
use libfuzzer_sys::fuzz_target;

fn token(data: &[u8], start: usize, len: usize) -> String {
    if data.get(start).is_some_and(|byte| byte % 11 == 0) {
        return String::new();
    }
    data.iter()
        .skip(start)
        .take(len)
        .map(|byte| char::from(32 + (byte % 95)))
        .collect()
}

fn numericish(data: &[u8], index: usize) -> Option<String> {
    let byte = *data.get(index).unwrap_or(&0);
    match byte % 7 {
        0 => None,
        1 => Some(String::new()),
        2 => Some("NaN".into()),
        3 => Some(format!("{}", byte)),
        4 => Some(format!("{}.{}", byte, byte % 10)),
        5 => Some(format!("{}/{}", byte, (byte % 9) + 1)),
        _ => Some(token(data, index, 5)),
    }
}

fn pin_function(byte: u8) -> PinFunction {
    match byte % 6 {
        0 => PinFunction::Power,
        1 => PinFunction::Ground,
        2 => PinFunction::Digital,
        3 => PinFunction::Analog,
        4 => PinFunction::NoConnect,
        _ => PinFunction::Unknown,
    }
}

fuzz_target!(|data: &[u8]| {
    if data.len() < 16 {
        return;
    }

    let bundle = EdaAuthoringBundle {
        source: SourceRef::new("fuzz", "eda-authoring").unwrap(),
        part: PartId::new("fuzz:board").unwrap(),
        variant: VariantId::new("A").unwrap(),
        display_name: token(data, 0, 8),
        circuit_records: vec![CircuitJsonSourceRecord {
            id: token(data, 1, 4),
            kind: token(data, 2, 4),
            reference: Some(token(data, 3, 4)),
            nets: vec![token(data, 4, 4), token(data, 5, 4)]
                .into_iter()
                .filter(|net| !net.is_empty())
                .collect(),
            exact_fields: vec![EdaExactField {
                field: "value".into(),
                value: numericish(data, 6),
                unit: Some(token(data, 7, 3)),
            }],
        }],
        footprint: Some(EdaFootprintString {
            handle: token(data, 8, 6),
            expression: format!(
                "{}:pins={},pitch={}mm",
                token(data, 9, 4),
                token(data, 10, 3),
                token(data, 11, 5)
            ),
        }),
        model_references: vec![GeneratedModelReference {
            handle: token(data, 12, 6),
            owner: token(data, 13, 5),
            format: token(data, 14, 4),
            uri: None,
            units: Some("mm".into()),
            status: match data[0] % 5 {
                0 => EdaModelStatus::Exact,
                1 => EdaModelStatus::Certified,
                2 => EdaModelStatus::LossyPreview,
                3 => EdaModelStatus::DisplayOnly,
                _ => EdaModelStatus::Missing,
            },
        }],
        package: Some(EdaPackageMetadata {
            name: token(data, 15, 5),
            handle: token(data, 0, 5),
            terminal_count: Some((data[1] % 4) as usize),
            pins: vec![EdaPackagePin {
                terminal: token(data, 2, 3),
                name: token(data, 3, 3),
                function: pin_function(data[4]),
                voltage_min: numericish(data, 5),
                voltage_max: numericish(data, 6),
            }],
        }),
        routes: vec![AutorouterOutputRecord {
            route_id: token(data, 7, 5),
            net: token(data, 8, 4),
            geometry_handle: Some(token(data, 9, 5)),
            units: Some("mm".into()),
            exact_grid: numericish(data, 10),
            status: match data[2] % 4 {
                0 => EdaRouteStatus::Exact,
                1 => EdaRouteStatus::Certified,
                2 => EdaRouteStatus::Lossy,
                _ => EdaRouteStatus::Missing,
            },
        }],
        fabrication: vec![FabricationOutputRecord {
            artifact_id: token(data, 11, 5),
            format: token(data, 12, 4),
            process: ProcessKind::Pcb,
            readiness: match data[3] % 4 {
                0 => EdaFabricationReadiness::Ready,
                1 => EdaFabricationReadiness::NeedsReview,
                2 => EdaFabricationReadiness::Failed,
                _ => EdaFabricationReadiness::Unknown,
            },
            notes: Vec::new(),
        }],
    };

    let result = import_eda_authoring_bundle(bundle);
    assert_eq!(result.graph.import_reports().len(), 1);
    assert_eq!(result.import_report.imported_family_count, 1);
    assert_eq!(result.import_report.imported_variant_count, 1);
    for handoff in &result.circuit_handoffs {
        assert_eq!(handoff.owner, "hypercircuit");
    }
    for handoff in &result.route_handoffs {
        assert_eq!(handoff.owner, "hyperpath");
    }
    for handoff in &result.drc_handoffs {
        assert_eq!(handoff.owner, "hyperdrc");
    }
});
