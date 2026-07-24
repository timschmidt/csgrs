#![cfg(feature = "geometry")]

use hypercircuit::{
    LEGACY_CSGRS_ELECTRONICS_REMOVAL_VERSION, LEGACY_CSGRS_ELECTRONICS_SCHEMA,
    LEGACY_CSGRS_ELECTRONICS_VERSION, LegacyCsgrsElectronicsImport,
    LegacyCsgrsElectronicsImportError, LegacyCsgrsElectronicsOmission, LegacyCsgrsTerminalClaim,
};

fn retired_handoff() -> LegacyCsgrsElectronicsImport {
    LegacyCsgrsElectronicsImport {
        schema: LEGACY_CSGRS_ELECTRONICS_SCHEMA.into(),
        version: LEGACY_CSGRS_ELECTRONICS_VERSION,
        source_handle: "mesh/qfn-16".into(),
        family_id: "legacy-qfn".into(),
        variant_id: "qfn-16".into(),
        package_aspects: vec!["package".into()],
        electrical_aspects: vec!["electrical".into()],
        terminal_claims: vec![LegacyCsgrsTerminalClaim {
            handle: "terminal-1".into(),
            name: "1".into(),
            role: "pin".into(),
        }],
        omissions: vec![
            LegacyCsgrsElectronicsOmission::LandPatternRequiresAuthoring {
                aspect: "package".into(),
            },
            LegacyCsgrsElectronicsOmission::DeviceModelRequiresAuthoring {
                aspect: "electrical".into(),
            },
            LegacyCsgrsElectronicsOmission::TerminalRequiresExplicitMapping {
                terminal: "terminal-1".into(),
                role: "pin".into(),
            },
        ],
    }
}

#[test]
fn retired_csgrs_handoff_remains_readable_without_live_geometry_markers() {
    assert_eq!(LEGACY_CSGRS_ELECTRONICS_REMOVAL_VERSION, "0.4.0");
    let imported = retired_handoff();
    assert!(imported.requires_review());

    let json = imported.to_json().expect("migration handoff serializes");
    assert_eq!(
        LegacyCsgrsElectronicsImport::from_json(&json).expect("migration handoff round-trips"),
        imported
    );

    let future = json.replace(
        "\"version\": 1",
        &format!("\"version\": {}", LEGACY_CSGRS_ELECTRONICS_VERSION + 1),
    );
    assert_eq!(
        LegacyCsgrsElectronicsImport::from_json(&future),
        Err(LegacyCsgrsElectronicsImportError::Version(
            LEGACY_CSGRS_ELECTRONICS_VERSION + 1
        ))
    );
}

#[test]
fn retired_handoff_rejects_other_schema_families() {
    let json = retired_handoff()
        .to_json()
        .unwrap()
        .replace(LEGACY_CSGRS_ELECTRONICS_SCHEMA, "other.schema");

    assert_eq!(
        LegacyCsgrsElectronicsImport::from_json(&json),
        Err(LegacyCsgrsElectronicsImportError::Schema(
            "other.schema".into()
        ))
    );
}
