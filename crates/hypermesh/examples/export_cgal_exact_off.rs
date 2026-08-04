#[path = "../competitive/support.rs"]
#[allow(dead_code)]
mod competitive_support;

use std::fmt::Write as _;
use std::path::{Path, PathBuf};

use competitive_support::{RawMesh, clipped_voxel_torus_case, corpus};
use hypermesh::Real;

fn exact_scalar(value: f64, output: &mut String) {
    let rational = Real::try_from(value)
        .expect("fixture coordinate is finite")
        .exact_rational()
        .expect("a finite binary64 fixture coordinate is an exact rational");
    if rational.is_negative() {
        output.push('-');
    }
    write!(
        output,
        "{}/{}",
        rational.numerator(),
        rational.denominator()
    )
    .unwrap();
}

fn write_exact_off(path: &Path, mesh: &RawMesh) {
    let mut output = String::new();
    writeln!(
        output,
        "OFF\n{} {} 0",
        mesh.positions.len(),
        mesh.triangles.len()
    )
    .unwrap();
    for point in &mesh.positions {
        for (axis, &coordinate) in point.iter().enumerate() {
            if axis != 0 {
                output.push(' ');
            }
            exact_scalar(coordinate, &mut output);
        }
        output.push('\n');
    }
    for triangle in &mesh.triangles {
        writeln!(output, "3 {} {} {}", triangle[0], triangle[1], triangle[2]).unwrap();
    }
    std::fs::write(path, output)
        .unwrap_or_else(|error| panic!("failed to write {}: {error}", path.display()));
}

fn main() {
    let mut args = std::env::args().skip(1);
    let fixture = args
        .next()
        .expect("expected <competitive-fixture> <output-directory>");
    let output_directory = PathBuf::from(
        args.next()
            .expect("expected <competitive-fixture> <output-directory>"),
    );
    assert!(
        args.next().is_none(),
        "expected exactly one fixture and one output directory"
    );
    let case = match fixture.as_str() {
        "clipped_voxel_torus_33" => clipped_voxel_torus_case(33),
        "clipped_voxel_torus_65" => clipped_voxel_torus_case(65),
        _ => corpus()
            .into_iter()
            .find(|case| case.name == fixture)
            .unwrap_or_else(|| panic!("unknown competitive fixture {fixture}")),
    };
    std::fs::create_dir_all(&output_directory)
        .unwrap_or_else(|error| panic!("failed to create {}: {error}", output_directory.display()));
    let left = output_directory.join(format!("{}-left.off", case.name));
    let right = output_directory.join(format!("{}-right.off", case.name));
    write_exact_off(&left, &case.left);
    write_exact_off(&right, &case.right);
    println!("{}\n{}", left.display(), right.display());
}
