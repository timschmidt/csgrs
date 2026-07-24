#[cfg(feature = "interchange")]
use hypercircuit::{
    BoardOutline, CircuitPackageExportKind, CircuitPackageLock, CircuitPackageStore, Design,
    DeviceModelKind, Footprint, PartDefinition, PartInstance, PartLibraryArtifact, PartSymbolUnit,
    PcbStackup, Real, SchematicPinSide, SchematicPoint, SymbolPin, SymbolUnitPlacement, pin,
};
use hypercircuit::{
    CircuitPackageCatalog, CircuitPackageName, CircuitPackageRelease, PackageDigest,
    PackageRequirement, PackageResolutionError, PackageSource,
};
#[cfg(feature = "interchange")]
use hyperlattice::Point2;
#[cfg(feature = "interchange")]
use hyperpath::TraceLayer;
use semver::Version;

fn name(value: &str) -> CircuitPackageName {
    CircuitPackageName::new(value).unwrap()
}

fn requirement(package: &str, version: &str) -> PackageRequirement {
    PackageRequirement::parse(name(package), version, None).unwrap()
}

fn release(
    package: &str,
    version: &str,
    digest: &str,
    dependencies: Vec<PackageRequirement>,
) -> CircuitPackageRelease {
    CircuitPackageRelease {
        name: name(package),
        version: Version::parse(version).unwrap(),
        source: PackageSource::Registry("hyper.example".into()),
        digest: PackageDigest::new("sha256", digest).unwrap(),
        exports: Vec::new(),
        dependencies,
    }
}

#[test]
fn resolver_backtracks_to_highest_mutually_compatible_release_and_verifies_lock() {
    let catalog = CircuitPackageCatalog {
        releases: vec![
            release("footprints", "1.5.0", "f15", Vec::new()),
            release("footprints", "2.0.0", "f20", Vec::new()),
            release(
                "sensor",
                "1.0.0",
                "s10",
                vec![requirement("footprints", ">=1.0, <2.0")],
            ),
            release(
                "sensor",
                "2.0.0",
                "s20",
                vec![requirement("footprints", ">=2.0")],
            ),
        ],
    };

    let lock = catalog
        .resolve(&[
            requirement("sensor", ">=1.0"),
            requirement("footprints", "<2.0"),
        ])
        .unwrap();
    assert_eq!(lock.packages.len(), 2);
    assert_eq!(lock.packages[0].name, name("footprints"));
    assert_eq!(lock.packages[0].version, Version::parse("1.5.0").unwrap());
    assert_eq!(lock.packages[1].version, Version::parse("1.0.0").unwrap());
    catalog.verify_lock(&lock).unwrap();

    let mut tampered = lock.clone();
    tampered.packages[0].digest = PackageDigest::new("sha256", "wrong").unwrap();
    assert_eq!(
        catalog.verify_lock(&tampered),
        Err(PackageResolutionError::DigestMismatch(name("footprints")))
    );
}

#[cfg(feature = "interchange")]
#[test]
fn package_lock_json_round_trips_exact_coordinates_and_provenance() {
    let catalog = CircuitPackageCatalog {
        releases: vec![release("symbols", "3.2.1", "abc123", Vec::new())],
    };
    let lock = catalog.resolve(&[requirement("symbols", "^3")]).unwrap();
    let json = lock.to_json().unwrap();
    let restored = CircuitPackageLock::from_json(&json).unwrap();
    assert_eq!(restored, lock);
    catalog.verify_lock(&restored).unwrap();
}

#[cfg(feature = "interchange")]
#[test]
fn portable_part_library_publishes_locks_loads_and_instantiates_without_duplication() {
    let mut source_design = Design::new(
        "library-source",
        BoardOutline::rectangle(Real::from(10), Real::from(10)),
        PcbStackup::single_layer(Real::one(), None),
    )
    .unwrap();
    let symbol = PartSymbolUnit::new(1, Real::from(6), Real::from(3))
        .pin(SymbolPin::new(
            "1",
            SchematicPoint::new(Real::from(-4), Real::zero()),
            SchematicPinSide::Left,
        ))
        .pin(SymbolPin::new(
            "2",
            SchematicPoint::new(Real::from(4), Real::zero()),
            SchematicPinSide::Right,
        ))
        .rectangular_body(Real::one())
        .unwrap();
    let source = source_design
        .define_part(
            PartDefinition::new("resistor-0603", "0603 resistor")
                .model_kind(DeviceModelKind::Resistor)
                .part_ref("hyperparts:resistor-0603")
                .pin(pin("1").pad("1"))
                .pin(pin("2").pad("2"))
                .symbol_name("R")
                .symbol_unit(symbol)
                .footprint(Footprint::two_pad_smd(
                    Real::one(),
                    Real::one(),
                    Real::from(2),
                    vec![TraceLayer(0)],
                )),
        )
        .unwrap();
    let portable = source_design.export_part(&source).unwrap();
    let artifact = PartLibraryArtifact::new(
        name("passives"),
        Version::parse("1.2.3").unwrap(),
        vec![portable],
    )
    .unwrap();
    let json = artifact.to_json_pretty().unwrap();
    let decoded = PartLibraryArtifact::from_json(&json).unwrap();
    assert_eq!(decoded, artifact);
    assert_eq!(decoded.digest().unwrap(), artifact.digest().unwrap());

    let unique = format!(
        "hypercircuit-package-{}-{}",
        std::process::id(),
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_nanos()
    );
    let root = std::env::temp_dir().join(unique);
    let store = CircuitPackageStore::new(&root);
    let published = store
        .publish(
            &artifact,
            PackageSource::Registry("registry.example".into()),
        )
        .unwrap();
    assert!(published.created);
    assert_eq!(
        published.release.exports[0].kind,
        CircuitPackageExportKind::PartDefinition
    );
    assert!(
        !store
            .publish(
                &artifact,
                PackageSource::Registry("registry.example".into())
            )
            .unwrap()
            .created
    );

    let catalog = CircuitPackageCatalog {
        releases: vec![published.release],
    };
    let lock = catalog.resolve(&[requirement("passives", "^1.2")]).unwrap();
    let loaded = store.load_verified_lock(&catalog, &lock).unwrap();
    assert_eq!(loaded, vec![artifact]);

    let mut design = Design::new(
        "library-consumer",
        BoardOutline::rectangle(Real::from(20), Real::from(10)),
        PcbStackup::single_layer(Real::one(), None),
    )
    .unwrap();
    let common = design.ground("GND").unwrap();
    let resistor = design
        .import_part(loaded[0].part("resistor-0603").unwrap())
        .unwrap();
    let first = design
        .instantiate(
            &resistor,
            PartInstance::new("R1")
                .parameter("resistance", Real::from(1_000), "ohm")
                .symbol(SymbolUnitPlacement::new(
                    1,
                    SchematicPoint::new(Real::from(4), Real::from(3)),
                ))
                .at(Point2::new(Real::from(5), Real::from(5))),
        )
        .unwrap();
    let second = design
        .instantiate(
            &resistor,
            PartInstance::new("R2")
                .parameter("resistance", Real::from(2_200), "ohm")
                .symbol(SymbolUnitPlacement::new(
                    1,
                    SchematicPoint::new(Real::from(14), Real::from(3)),
                ))
                .at(Point2::new(Real::from(14), Real::from(5))),
        )
        .unwrap();
    design
        .connect(
            &common,
            [
                first.pin("1").unwrap(),
                first.pin("2").unwrap(),
                second.pin("1").unwrap(),
                second.pin("2").unwrap(),
            ],
        )
        .unwrap();
    let checked = design.finish().unwrap();
    assert_eq!(checked.circuit.device_models.len(), 1);
    assert_eq!(checked.circuit.instances.len(), 2);
    assert_eq!(checked.schematic.symbol_definitions.len(), 1);
    assert_eq!(checked.layout.land_patterns.len(), 1);
    assert_eq!(checked.circuit.lower_linear_devices().stamps.len(), 2);

    std::fs::write(&published.path, b"tampered").unwrap();
    assert_eq!(
        store.load(&lock.packages[0]),
        Err(PackageResolutionError::DigestMismatch(name("passives")))
    );
    std::fs::remove_dir_all(root).unwrap();
}
