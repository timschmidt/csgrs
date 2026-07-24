#![cfg(feature = "interchange")]

use std::path::Path;

use hypercircuit::{
    BoardOutline, Design, DeviceModelId, DeviceModelKind, DrillShape, KiCadLibraryImportOmission,
    KiCadPartLibraryImportOptions, KiCadPartLibraryImportReport, LandPatternId, PartInstance,
    PcbStackup, Real, SchematicPoint, SchematicSymbolDefinitionId, SymbolUnitPlacement,
};
use hyperlattice::Point2;

const SYMBOL_LIBRARY: &str = r#"
(kicad_symbol_lib
  (version 20241209)
  (generator kicad_symbol_editor)
  (symbol "Base"
    (in_bom yes)
    (on_board yes)
    (property "Reference" "R" (at 0 0 0) (effects (font (size 1.27 1.27))))
    (property "Value" "Base" (at 0 0 0) (effects (font (size 1.27 1.27))))
    (property "Footprint" "" (at 0 0 0) (effects (font (size 1.27 1.27))))
    (property "Datasheet" "~" (at 0 0 0) (effects (font (size 1.27 1.27))))
    (symbol "Base_0_1"
      (rectangle
        (start -1 -2)
        (end 1 2)
        (stroke (width 0.25) (type default))
        (fill (type none)))
      (curve
        (pts (xy -1 -1) (xy 0 1) (xy 1 -1))
        (stroke (width 0.1) (type default))
        (fill (type none))))
    (symbol "Base_1_1"
      (pin passive line
        (at 0 3 270)
        (length 1)
        (name "~" (effects (font (size 1 1))))
        (number "1" (effects (font (size 1 1)))))
      (pin passive inverted
        (at 0 -3 90)
        (length 1)
        (name "~" (effects (font (size 1 1))))
        (number "2" (effects (font (size 1 1)))))))
  (symbol "Child"
    (extends "Base")
    (property "Value" "Child" (at 0 0 0) (effects (font (size 1.27 1.27))))
    (property "Description" "Imported exact resistor" (at 0 0 0)
      (effects (font (size 1 1)) (hide yes))))
)
"#;

const FOOTPRINT: &str = r#"
(footprint "R_Test"
  (version 20241229)
  (generator kicad-footprint-generator)
  (layer "F.Cu")
  (fp_line
    (start -0.2 -0.5)
    (end 0.2 -0.5)
    (stroke (width 0.12) (type solid))
    (layer "F.SilkS"))
  (fp_rect
    (start -1.5 -0.75)
    (end 1.5 0.75)
    (stroke (width 0.05) (type solid))
    (fill no)
    (layer "F.CrtYd"))
  (fp_arc
    (start -0.5 0)
    (mid 0 0.5)
    (end 0.5 0)
    (stroke (width 0.1) (type solid))
    (layer "F.Fab"))
  (pad "1" smd roundrect
    (at -0.8 0 45)
    (size 0.8 0.95)
    (layers "F.Cu" "F.Mask" "F.Paste")
    (roundrect_rratio 0.25))
  (pad "2" thru_hole oval
    (at 0.8 0)
    (size 1.5 2.5)
    (drill oval 0.6 1.4)
    (layers "*.Cu" "*.Mask"))
  (model "${KICAD9_3DMODEL_DIR}/R_Test.step"
    (offset (xyz 1 2 3))
    (scale (xyz 1 2 0.5))
    (rotate (xyz 10 20 30)))
  (model "package://resistor/marking.obj"
    (offset (xyz -1 0 0.25))
    (scale (xyz 0.5 0.5 0.5))
    (rotate (xyz 0 0 180)))
)
"#;

fn options() -> KiCadPartLibraryImportOptions {
    KiCadPartLibraryImportOptions::new(
        "resistor-test",
        DeviceModelId::new("resistor-test.model").unwrap(),
        DeviceModelKind::Resistor,
        SchematicSymbolDefinitionId::new("resistor-test.symbol").unwrap(),
        LandPatternId::new("resistor-test.footprint").unwrap(),
    )
}

#[test]
fn native_library_pair_imports_to_one_portable_part_and_checked_design() {
    let report =
        KiCadPartLibraryImportReport::from_str(SYMBOL_LIBRARY, "Child", FOOTPRINT, options())
            .unwrap();
    assert_eq!(report.part.model.pins.len(), 2);
    assert_eq!(report.part.symbol.as_ref().unwrap().units.len(), 1);
    assert_eq!(report.part.land_pattern.as_ref().unwrap().pads.len(), 2);
    assert_eq!(report.part.land_pattern.as_ref().unwrap().graphics.len(), 2);
    assert_eq!(
        report
            .symbol_properties
            .get("Description")
            .map(String::as_str),
        Some("Imported exact resistor")
    );
    assert!(!report.numeric_imports.is_empty());
    assert!(report.omissions.iter().any(|omission| matches!(
        omission,
        KiCadLibraryImportOmission::SymbolInheritanceFlattened { child, parent }
            if child == "Child" && parent == "Base"
    )));
    assert!(report.omissions.iter().any(|omission| matches!(
        omission,
        KiCadLibraryImportOmission::UnsupportedSymbolGraphic { primitive, .. }
            if primitive == "curve"
    )));
    assert!(report.omissions.iter().any(|omission| matches!(
        omission,
        KiCadLibraryImportOmission::UnsupportedFootprintGraphic { primitive, .. }
            if primitive == "fp_arc"
    )));
    let land_pattern = report.part.land_pattern.as_ref().unwrap();
    assert_eq!(land_pattern.pads[0].rotation_degrees, Real::from(45));
    assert_eq!(land_pattern.pads[1].rotation_degrees, Real::zero());
    assert_eq!(
        land_pattern.pads[1].drill,
        Some(DrillShape::Slot {
            start: Point2::new(Real::zero(), -(Real::from(2) / Real::from(5)).unwrap()),
            end: Point2::new(Real::zero(), (Real::from(2) / Real::from(5)).unwrap()),
            width: (Real::from(3) / Real::from(5)).unwrap(),
        })
    );
    assert_eq!(land_pattern.models.len(), 2);
    let model = &land_pattern.models[0];
    assert_eq!(model.uri, "${KICAD9_3DMODEL_DIR}/R_Test.step");
    assert_eq!(model.format, hypercircuit::Pcb3dModelFormat::Step);
    assert_eq!(
        model.transform,
        hypercircuit::Pcb3dModelTransform {
            offset_x: Real::from(1),
            offset_y: Real::from(2),
            offset_z: Real::from(3),
            rotate_x_degrees: Real::from(10),
            rotate_y_degrees: Real::from(20),
            rotate_z_degrees: Real::from(30),
            scale_x: Real::from(1),
            scale_y: Real::from(2),
            scale_z: (Real::one() / Real::from(2)).unwrap(),
        }
    );
    assert_eq!(land_pattern.models[1].uri, "package://resistor/marking.obj");
    assert_eq!(
        land_pattern.models[1].format,
        hypercircuit::Pcb3dModelFormat::WavefrontObj
    );

    let mut design = Design::new(
        "native-library-consumer",
        BoardOutline::rectangle(Real::from(10), Real::from(8)),
        PcbStackup::single_layer(Real::one(), None),
    )
    .unwrap();
    let ground = design.ground("GND").unwrap();
    let resistor = design.import_part(&report.part).unwrap();
    let instance = design
        .instantiate(
            &resistor,
            PartInstance::new("R1")
                .parameter("resistance", Real::from(1_000), "ohm")
                .symbol(SymbolUnitPlacement::new(
                    1,
                    SchematicPoint::new(Real::from(3), Real::from(3)),
                ))
                .at(Point2::new(Real::from(5), Real::from(4))),
        )
        .unwrap();
    design
        .connect(
            &ground,
            [instance.pin("1").unwrap(), instance.pin("2").unwrap()],
        )
        .unwrap();
    let checked = design.finish().unwrap();
    assert_eq!(checked.circuit.lower_linear_devices().stamps.len(), 1);
    assert_eq!(checked.schematic.symbol_definitions.len(), 1);
    assert_eq!(checked.layout.land_patterns[0].graphics.len(), 2);
    assert_eq!(checked.layout.land_patterns[0].models.len(), 2);
}

#[test]
fn installed_official_resistor_libraries_import_when_available() {
    let symbols = Path::new("/usr/share/kicad/symbols/Device.kicad_sym");
    let footprint =
        Path::new("/usr/share/kicad/footprints/Resistor_SMD.pretty/R_0603_1608Metric.kicad_mod");
    if !symbols.exists() || !footprint.exists() {
        return;
    }
    let report =
        KiCadPartLibraryImportReport::from_paths(symbols, "R", footprint, options()).unwrap();
    assert_eq!(
        report
            .part
            .model
            .pins
            .iter()
            .map(|pin| pin.pin.as_str())
            .collect::<Vec<_>>(),
        ["1", "2"]
    );
    assert_eq!(report.part.land_pattern.as_ref().unwrap().pads.len(), 2);
    report.part.validate().unwrap();
}

#[test]
fn installed_official_multipart_logic_and_dip_libraries_import_when_available() {
    let symbols = Path::new("/usr/share/kicad/symbols/74xx.kicad_sym");
    let footprint =
        Path::new("/usr/share/kicad/footprints/Package_DIP.pretty/DIP-14_W7.62mm.kicad_mod");
    if !symbols.exists() || !footprint.exists() {
        return;
    }
    let mut import_options = options();
    import_options.export_name = "74ls00-dip".into();
    import_options.model_kind = DeviceModelKind::Custom("quad NAND gate".into());
    let report =
        KiCadPartLibraryImportReport::from_paths(symbols, "74LS00", footprint, import_options)
            .unwrap();
    assert_eq!(report.part.symbol.as_ref().unwrap().units.len(), 5);
    assert_eq!(report.part.model.pins.len(), 14);
    assert_eq!(report.part.land_pattern.as_ref().unwrap().pads.len(), 14);
    report.part.validate().unwrap();
}
