//! This example shows adding mass properties to a `CSG`

use csgrs::prelude::*;

const PATH: &str = "stl/mass_properties";

fn main() {
    let cube = CSG::<()>::cube(2.0, 2.0, 2.0, None);

    // 14) Mass properties (just printing them)
    let (mass, com, principal_frame) = cube.mass_properties(1.0);
    println!("Cube mass = {mass}");
    println!("Cube center of mass = {:?}", com);
    println!("Cube principal inertia local frame = {:?}", principal_frame);
}
