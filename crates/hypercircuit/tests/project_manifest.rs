#![cfg(feature = "interchange")]

use hypercircuit::{
    HYPERCIRCUIT_PROJECT_SCHEMA, HYPERCIRCUIT_PROJECT_VERSION, ProjectManifest,
    ProjectManifestError, ProjectProviderKind,
};

const MANIFEST: &str = r#"schema = "org.hypercircuit.project"
version = 1

[project]
name = "controller"
version = "2.4.0"
default-design = "prototype"

[designs.prototype]
provider = "command"
command = ["./emit-prototype", "--semantic-json"]

[designs.production]
provider = "cargo"
package = "controller-designs"
binary = "production-board"
features = ["production"]

[[pcb-materials]]
handle = "hyperphysics:FR4"
relative-permittivity = "21/5"
loss-tangent = "0.018"
source-authority = "laminate-manufacturer"
source-locator = "datasheet/revision-a"
source-freshness = "2026-06"
condition = "1 MHz nominal"
"#;

#[test]
fn project_manifest_round_trips_and_selects_command_and_cargo_providers() {
    let manifest = ProjectManifest::from_toml(MANIFEST).unwrap();
    assert_eq!(manifest.schema, HYPERCIRCUIT_PROJECT_SCHEMA);
    assert_eq!(manifest.version, HYPERCIRCUIT_PROJECT_VERSION);

    let (name, default) = manifest.design(None).unwrap();
    assert_eq!(name, "prototype");
    assert_eq!(default.provider, ProjectProviderKind::Command);
    assert_eq!(default.command[0], "./emit-prototype");

    let (_, production) = manifest.design(Some("production")).unwrap();
    assert_eq!(production.provider, ProjectProviderKind::Cargo);
    assert_eq!(production.package.as_deref(), Some("controller-designs"));
    assert_eq!(production.binary.as_deref(), Some("production-board"));
    assert_eq!(manifest.pcb_materials.len(), 1);
    assert_eq!(manifest.pcb_materials[0].handle, "hyperphysics:FR4");
    assert_eq!(
        ProjectManifest::from_toml(&manifest.to_toml_pretty().unwrap()).unwrap(),
        manifest
    );
}

#[cfg(feature = "drc")]
#[test]
fn project_materials_resolve_to_exact_hyperphysics_property_graphs() {
    use hypercircuit::{PCB_LOSS_TANGENT_PROPERTY, PCB_RELATIVE_PERMITTIVITY_PROPERTY};
    use hyperphysics::{MaterialPropertyKind, PropertyValue};

    let manifest = ProjectManifest::from_toml(MANIFEST).unwrap();
    let library = manifest.pcb_material_library().unwrap();
    let graph = library.get("hyperphysics:FR4").unwrap();
    let relative_permittivity = graph.resolve(&MaterialPropertyKind::Custom(
        PCB_RELATIVE_PERMITTIVITY_PROPERTY.into(),
    ));
    let loss_tangent = graph.resolve(&MaterialPropertyKind::Custom(
        PCB_LOSS_TANGENT_PROPERTY.into(),
    ));

    assert_eq!(
        relative_permittivity.value,
        Some(PropertyValue::exact_scalar(
            (hypercircuit::Real::from(21) / hypercircuit::Real::from(5)).unwrap()
        ))
    );
    assert_eq!(loss_tangent.sources[0].authority, "laminate-manufacturer");
    assert_eq!(
        loss_tangent.sources[0].freshness.as_deref(),
        Some("2026-06")
    );
}

#[test]
fn project_manifest_rejects_unknown_defaults_and_mixed_provider_fields() {
    let unknown = MANIFEST.replace(
        "default-design = \"prototype\"",
        "default-design = \"missing\"",
    );
    assert_eq!(
        ProjectManifest::from_toml(&unknown).unwrap_err(),
        ProjectManifestError::UnknownDesign("missing".into())
    );

    let mixed = MANIFEST.replace(
        "command = [\"./emit-prototype\", \"--semantic-json\"]",
        "command = [\"./emit-prototype\"]\npackage = \"not-allowed\"",
    );
    assert!(matches!(
        ProjectManifest::from_toml(&mixed),
        Err(ProjectManifestError::InvalidProvider { design, .. }) if design == "prototype"
    ));

    let duplicate = format!(
        "{MANIFEST}\n{}",
        r#"[[pcb-materials]]
handle = "hyperphysics:FR4"
relative-permittivity = "4.3"
loss-tangent = "0.02"
source-authority = "other"
source-locator = "other"
"#
    );
    assert_eq!(
        ProjectManifest::from_toml(&duplicate).unwrap_err(),
        ProjectManifestError::DuplicatePcbMaterial("hyperphysics:FR4".into())
    );

    let invalid = MANIFEST.replace(
        "relative-permittivity = \"21/5\"",
        "relative-permittivity = \"0\"",
    );
    assert!(matches!(
        ProjectManifest::from_toml(&invalid),
        Err(ProjectManifestError::InvalidPcbMaterial { material, .. })
            if material == "hyperphysics:FR4"
    ));
}
