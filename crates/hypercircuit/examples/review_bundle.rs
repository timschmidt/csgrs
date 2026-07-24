//! Emit schematic, PCB, and 3D review artifacts from one checked declarative design.

use std::env;
use std::fs;
use std::io;
use std::path::PathBuf;

use hypercircuit::{
    BoardOutline, Design, DrillShape, Footprint, LandPatternBody, LandPatternPad,
    MaterializationOptions, PadId, PadShape, Pcb3dModelFormat, Pcb3dModelReference,
    Pcb3dModelTransform, PcbStackup, PcbSvgOptions, Plating, Real, Route, SchematicPinSide,
    SchematicPoint, SchematicSvgOptions, StackupLayer, StackupLayerKind, Symbol, SymbolPin, parts,
};
use hyperlattice::Point2;
use hyperpath::TraceLayer;

fn point(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn smd_pad(id: &str, x: i64, layer: TraceLayer) -> LandPatternPad {
    LandPatternPad {
        id: PadId::new(id).expect("constant pad id is valid"),
        center: point(x, 0),
        rotation_degrees: Real::zero(),
        copper_layers: vec![layer],
        shape: PadShape::Rectangle {
            width: Real::from(3),
            height: Real::from(3),
        },
        drill: None,
        plating: Plating::Unspecified,
        solder_mask_margin: Some((Real::one() / Real::from(10)).expect("nonzero denominator")),
        paste_margin: Some(-(Real::one() / Real::from(10)).expect("nonzero denominator")),
    }
}

fn review_footprint(left_pin: &str, right_pin: &str, layer: TraceLayer) -> Footprint {
    Footprint::new()
        .pad(smd_pad(left_pin, -2, layer))
        .pad(smd_pad(right_pin, 2, layer))
        .pad(LandPatternPad {
            id: PadId::new("mount").expect("constant pad id is valid"),
            center: point(0, 4),
            rotation_degrees: Real::zero(),
            copper_layers: Vec::new(),
            shape: PadShape::Circle {
                diameter: Real::from(3),
            },
            drill: Some(DrillShape::Round {
                diameter: Real::from(2),
            }),
            plating: Plating::NonPlated,
            solder_mask_margin: None,
            paste_margin: None,
        })
        .body(LandPatternBody {
            outline: vec![point(-4, -3), point(4, -3), point(4, 3), point(-4, 3)],
            height: Real::from(2),
            standoff: (Real::one() / Real::from(5)).expect("nonzero denominator"),
        })
        .model(Pcb3dModelReference {
            uri: "embedded://review-package.obj".into(),
            format: Pcb3dModelFormat::WavefrontObj,
            transform: Pcb3dModelTransform {
                offset_x: Real::from(-2),
                offset_y: Real::from(-1),
                offset_z: (Real::one() / Real::from(5)).expect("nonzero denominator"),
                scale_x: Real::from(4),
                scale_y: Real::from(2),
                scale_z: Real::from(2),
                ..Default::default()
            },
        })
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let output = env::args_os().nth(1).map(PathBuf::from).ok_or_else(|| {
        io::Error::new(
            io::ErrorKind::InvalidInput,
            "usage: review_bundle OUTPUT_DIRECTORY",
        )
    })?;
    fs::create_dir_all(&output)?;

    let front = TraceLayer(0);
    let back = TraceLayer(1);
    let copper = (Real::from(35) / Real::from(1_000)).expect("nonzero denominator");
    let mask = (Real::one() / Real::from(50)).expect("nonzero denominator");
    let stackup = PcbStackup {
        layers: vec![
            StackupLayer {
                name: "F.Mask".into(),
                kind: StackupLayerKind::SolderMask,
                thickness: mask.clone(),
                material: Some("hyperphysics:solder-mask".into()),
            },
            StackupLayer {
                name: "F.Cu".into(),
                kind: StackupLayerKind::Conductor(front),
                thickness: copper.clone(),
                material: Some("hyperphysics:copper".into()),
            },
            StackupLayer {
                name: "Core".into(),
                kind: StackupLayerKind::Dielectric,
                thickness: (Real::from(153) / Real::from(100)).expect("nonzero denominator"),
                material: Some("hyperphysics:FR4".into()),
            },
            StackupLayer {
                name: "B.Cu".into(),
                kind: StackupLayerKind::Conductor(back),
                thickness: copper,
                material: Some("hyperphysics:copper".into()),
            },
            StackupLayer {
                name: "B.Mask".into(),
                kind: StackupLayerKind::SolderMask,
                thickness: mask,
                material: Some("hyperphysics:solder-mask".into()),
            },
        ],
    };
    let mut design = Design::new(
        "review-bundle",
        BoardOutline::rectangle(Real::from(30), Real::from(20)),
        stackup,
    )?;
    let output_net = design.signal("OUT")?;
    let ground = design.ground("GND")?;
    let source = design.add(
        parts::voltage_source("V1", Real::from(5))
            .symbol(
                Symbol::new(
                    SchematicPoint::new(Real::from(5), Real::from(5)),
                    Real::from(5),
                    Real::from(4),
                )
                .pin(SymbolPin::new(
                    "neg",
                    SchematicPoint::new(Real::from(-3), Real::zero()),
                    SchematicPinSide::Left,
                ))
                .pin(SymbolPin::new(
                    "pos",
                    SchematicPoint::new(Real::from(3), Real::zero()),
                    SchematicPinSide::Right,
                )),
            )
            .footprint(review_footprint("neg", "pos", front))
            .at(point(7, 10)),
    )?;
    let load = design.add(
        parts::resistor("R1", Real::from(1_000))
            .symbol(Symbol::two_pin_horizontal(
                SchematicPoint::new(Real::from(16), Real::from(5)),
                Real::from(3),
                Real::from(5),
                Real::from(4),
            ))
            .footprint(review_footprint("1", "2", front))
            .at(point(23, 10)),
    )?;
    design.connect(&output_net, [source.pin("pos")?, load.pin("1")?])?;
    design.connect(&ground, [source.pin("neg")?, load.pin("2")?])?;
    design.route(
        &output_net,
        Route::new("out", front, Real::one()).line(point(9, 10), point(21, 10)),
    )?;
    design.route(
        &ground,
        Route::new("ground", front, Real::one())
            .line(point(5, 10), point(5, 16))
            .line(point(5, 16), point(25, 16))
            .line(point(25, 16), point(25, 10)),
    )?;

    let checked = design.finish()?;
    let schematic = checked
        .schematic
        .to_svg(&checked.circuit, SchematicSvgOptions::default())?;
    let pcb = checked
        .layout
        .to_svg(&checked.circuit, PcbSvgOptions::default())?;
    let materialized = checked
        .layout
        .materialize(&checked.circuit, MaterializationOptions::default())?;
    let package_obj = b"v 0 0 0\nv 1 0 0\nv 1 1 0\nv 0 1 0\nv 0 0 1\nv 1 0 1\nv 1 1 1\nv 0 1 1\n\
f 1 4 3 2\nf 5 6 7 8\nf 1 2 6 5\nf 2 3 7 6\nf 3 4 8 7\nf 4 1 5 8\n";
    let mut resolver = |reference: &Pcb3dModelReference| {
        (reference.uri == "embedded://review-package.obj")
            .then(|| package_obj.to_vec())
            .ok_or_else(|| format!("unknown embedded model {}", reference.uri))
    };
    let assembly = materialized.stackup_3d_with_model_resolver(&checked.layout, &mut resolver);
    let scene = assembly.to_gltf("review-bundle")?;

    fs::write(output.join("schematic.svg"), schematic.svg)?;
    fs::write(output.join("pcb.svg"), pcb.svg)?;
    fs::write(output.join("board.gltf"), &scene.gltf)?;
    fs::write(
        output.join("review-manifest.json"),
        serde_json::to_vec_pretty(&serde_json::json!({
            "schema": "hypercircuit.review-bundle",
            "version": 1,
            "source": checked.circuit.id.as_str(),
            "simulation_stamps": checked.circuit.lower_linear_devices().stamps.len(),
            "scene_objects": scene.objects.iter().map(|object| serde_json::json!({
                "name": object.name,
                "kind": format!("{:?}", object.kind),
                "triangles": object.triangle_count,
            })).collect::<Vec<_>>(),
            "coordinate_encoding": format!("{:?}", scene.coordinate_encoding),
            "resolved_models": assembly.model_resolutions.iter().map(|evidence| {
                serde_json::json!({
                    "instance": evidence.instance.as_str(),
                    "land_pattern": evidence.land_pattern.as_str(),
                    "model_index": evidence.model_index,
                    "uri": evidence.uri,
                    "format": format!("{:?}", evidence.format),
                    "sha256": evidence.source_sha256,
                    "triangles": evidence.triangle_count,
                    "source_scene_index": evidence.source_scene_index,
                    "source_mesh_nodes": evidence.source_mesh_node_count,
                    "source_primitives": evidence.source_primitive_count,
                    "ignored_non_mesh_geometry": evidence.ignored_non_mesh_geometry_count,
                    "ignored_degenerate_polygons": evidence.ignored_degenerate_polygon_count,
                    "ignored_degenerate_triangles": evidence.ignored_degenerate_triangle_count,
                })
            }).collect::<Vec<_>>(),
            "subtractions": assembly.subtractions.iter().map(|evidence| {
                format!("{evidence:?}")
            }).collect::<Vec<_>>(),
            "omissions": scene.assembly_omissions.iter().map(|omission| {
                format!("{omission:?}")
            }).collect::<Vec<_>>(),
        }))?,
    )?;
    println!(
        "wrote schematic.svg, pcb.svg, board.gltf, and review-manifest.json to {}",
        output.display()
    );
    Ok(())
}
