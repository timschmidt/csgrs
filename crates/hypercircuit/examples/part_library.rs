//! Publish, lock, load, and instantiate a portable reusable part library.

use std::path::PathBuf;

use hypercircuit::{
    BoardOutline, CircuitPackageCatalog, CircuitPackageName, CircuitPackageStore, Design,
    DeviceModelKind, Footprint, PackageRequirement, PackageSource, PartDefinition, PartInstance,
    PartLibraryArtifact, PartSymbolUnit, PcbStackup, Real, SchematicPinSide, SchematicPoint,
    SymbolPin, SymbolUnitPlacement, pin,
};
use hyperlattice::Point2;
use hyperpath::TraceLayer;
use semver::Version;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let requested_root = std::env::args_os().nth(1).map(PathBuf::from);
    let temporary = requested_root.is_none();
    let root = requested_root.unwrap_or_else(|| {
        std::env::temp_dir().join(format!("hypercircuit-part-library-{}", std::process::id()))
    });

    let mut source = Design::new(
        "library-source",
        BoardOutline::rectangle(Real::from(10), Real::from(10)),
        PcbStackup::single_layer(Real::one(), None),
    )?;
    let definition = source.define_part(
        PartDefinition::new("resistor-0603", "0603 resistor")
            .model_kind(DeviceModelKind::Resistor)
            .part_ref("hyperparts:resistor-0603")
            .pin(pin("1").pad("1"))
            .pin(pin("2").pad("2"))
            .symbol_name("R")
            .symbol_unit(
                PartSymbolUnit::new(1, Real::from(6), Real::from(3))
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
                    .rectangular_body(Real::one())?,
            )
            .footprint(Footprint::two_pad_smd(
                Real::one(),
                Real::one(),
                Real::from(2),
                vec![TraceLayer(0)],
            )),
    )?;
    let artifact = PartLibraryArtifact::new(
        CircuitPackageName::new("passives")?,
        Version::parse("1.0.0")?,
        vec![source.export_part(&definition)?],
    )?;
    let store = CircuitPackageStore::new(&root);
    let published = store.publish(
        &artifact,
        PackageSource::Registry("registry.example".into()),
    )?;
    let catalog = CircuitPackageCatalog {
        releases: vec![published.release.clone()],
    };
    let lock = catalog.resolve(&[PackageRequirement::parse(
        CircuitPackageName::new("passives")?,
        "^1",
        None,
    )?])?;
    let loaded = store.load_verified_lock(&catalog, &lock)?;

    let mut consumer = Design::new(
        "library-consumer",
        BoardOutline::rectangle(Real::from(20), Real::from(10)),
        PcbStackup::single_layer(Real::one(), None),
    )?;
    let common = consumer.ground("GND")?;
    let resistor = consumer.import_part(
        loaded[0]
            .part("resistor-0603")
            .expect("published export exists"),
    )?;
    let r1 = consumer.instantiate(
        &resistor,
        PartInstance::new("R1")
            .parameter("resistance", Real::from(1_000), "ohm")
            .symbol(SymbolUnitPlacement::new(
                1,
                SchematicPoint::new(Real::from(4), Real::from(3)),
            ))
            .at(Point2::new(Real::from(5), Real::from(5))),
    )?;
    let r2 = consumer.instantiate(
        &resistor,
        PartInstance::new("R2")
            .parameter("resistance", Real::from(2_200), "ohm")
            .symbol(SymbolUnitPlacement::new(
                1,
                SchematicPoint::new(Real::from(14), Real::from(3)),
            ))
            .at(Point2::new(Real::from(14), Real::from(5))),
    )?;
    consumer.connect(
        &common,
        [r1.pin("1")?, r1.pin("2")?, r2.pin("1")?, r2.pin("2")?],
    )?;
    let checked = consumer.finish()?;
    println!(
        "{}@{} {}: {} model, {} instances, {} symbol definition, {} land pattern",
        artifact.package.as_str(),
        artifact.version,
        artifact.digest()?.value,
        checked.circuit.device_models.len(),
        checked.circuit.instances.len(),
        checked.schematic.symbol_definitions.len(),
        checked.layout.land_patterns.len(),
    );

    if temporary {
        std::fs::remove_dir_all(root)?;
    }
    Ok(())
}
