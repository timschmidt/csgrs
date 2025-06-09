//! This example demos creating gears, racks, and spurs

use csgrs::CSG;
use std::{fs, path::Path};

const PATH: &str = "stl/gears";

fn main() {
    // Ensure the folder exists
    let _ = fs::create_dir_all(PATH);

    let gear_involute_2d = CSG::involute_gear_2d(
        2.0,  // module [mm]
        20,   // z – number of teeth
        20.0, // α – pressure angle [deg]
        0.05, // radial clearance
        0.02, // backlash at pitch line
        14,   // segments per involute flank
        None,
    );
    write_example(&gear_involute_2d, "gear_involute_2d");

    let gear_cycloid_2d = CSG::cycloidal_gear_2d(
        2.0,  // module
        17,   // gear teeth
        18,   // mating pin-wheel teeth (zₚ = z±1)
        0.05, // clearance
        20,   // segments per flank
        None,
    );
    write_example(&gear_cycloid_2d, "gear_cycloid_2d");

    let rack_involute = CSG::involute_rack_2d(
        2.0,  // module
        12,   // number of rack teeth to generate
        20.0, // pressure angle
        0.05, // clearance
        0.02, // backlash
        None,
    );
    write_example(&rack_involute, "rack_involute");

    let rack_cycloid = CSG::cycloidal_rack_2d(
        2.0,  // module
        12,   // teeth
        1.0,  // generating-circle radius  (≈ m/2 for a conventional pin-rack)
        0.05, // clearance
        24,   // segments per flank
        None,
    );
    write_example(&rack_cycloid, "rack_cycloid");

    let spur_involute = CSG::spur_gear_involute(
        2.0, 20, 20.0, 0.05, 0.02, 14, 12.0, // face-width (extrusion thickness)
        None,
    );
    write_example(&spur_involute, "spur_involute");

    let spur_cycloid = CSG::spur_gear_cycloid(
        2.0, 17, 18, 0.05, 20, 12.0, // thickness
        None,
    );
    write_example(&spur_cycloid, "spur_cycloid");

    // let helical = CSG::helical_involute_gear(
    // 2.0,   // module
    // 20,    // z
    // 20.0,  // pressure angle
    // 0.05, 0.02, 14,
    // 25.0,   // face-width
    // 15.0,   // helix angle β [deg]
    // 40,     // axial slices (resolution of the twist)
    // None,
    // );
    // write_example(&helical, "helical_gear");
}

fn write_example(shape: &CSG, name: &str) {
    let _ = fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        shape.to_stl_binary(name).unwrap(),
    );
}
