//! This example shows Triply‑Periodic Minimal Surfaces
//! gyroid and Schwarz‑P/D shapes that use the current CSG volume as a bounding region
//!
//! Note: this requires `chull-io`

use csgrs::csg::CSG;
use std::{fs, path::Path};

const PATH: &str = "stl/tpms";

fn main() {
    // Ensure the folder exists
    let _ = fs::create_dir_all(PATH);

    let cube = CSG::cube(2.0, 2.0, 2.0, None)
        .translate(1.0, 0.0, 0.0)
        .rotate(0.0, 45.0, 0.0)
        .scale(1.0, 0.5, 2.0);
    let sphere = CSG::sphere(1.0, 16, 8, None);

    let union_shape = cube.union(&sphere);
    let hull_of_union = union_shape.convex_hull();

    // ---------- Triply‑Periodic Minimal Surfaces ----------
    let gyroid_inside_cube = hull_of_union
        .scale(20.0, 20.0, 20.0)
        .gyroid(64, 2.0, 0.0, None);
    write_example(&gyroid_inside_cube, "gyroid_cube");

    let schwarzp_inside_cube = hull_of_union
        .scale(20.0, 20.0, 20.0)
        .schwarz_p(64, 2.0, 0.0, None);
    write_example(&schwarzp_inside_cube, "schwarz_p_cube");

    let schwarzd_inside_cube = hull_of_union
        .scale(20.0, 20.0, 20.0)
        .schwarz_d(64, 2.0, 0.0, None);
    write_example(&schwarzd_inside_cube, "schwarz_d_cube");
}

fn write_example(shape: &CSG, name: &str) {
    let _ = fs::write(
        Path::new(PATH).join(name).with_extension("stl"),
        shape.to_stl_binary(name).unwrap(),
    );
}
