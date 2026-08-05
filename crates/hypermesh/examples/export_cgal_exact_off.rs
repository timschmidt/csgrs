#[path = "../competitive/support.rs"]
#[allow(dead_code)]
mod competitive_support;

use std::fmt::Write as _;
use std::path::{Path, PathBuf};

use competitive_support::{
    WIDE_RATIONAL_DIVISIONS, clipped_voxel_torus_case, corpus, dense_coplanar_box_case,
    exact_mesh_pair, large_boolean_case, sparse_multishell_tetrahedra_case,
    wide_rational_overlapping_box_case, wide_rational_shift,
};
use hypermesh::{Real, TriangleMesh};

fn write_exact_scalar(value: &Real, output: &mut String) {
    let rational = value
        .exact_rational()
        .expect("the exact CGAL fixture coordinate must be rational");
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

fn write_exact_triangle_mesh(path: &Path, mesh: &TriangleMesh) {
    let mut output = String::new();
    writeln!(
        output,
        "OFF\n{} {} 0",
        mesh.positions.len(),
        mesh.triangles.len()
    )
    .unwrap();
    for point in mesh.positions.iter() {
        for (axis, coordinate) in [&point.x, &point.y, &point.z].into_iter().enumerate() {
            if axis != 0 {
                output.push(' ');
            }
            write_exact_scalar(coordinate, &mut output);
        }
        output.push('\n');
    }
    for triangle in mesh.triangles.iter() {
        writeln!(output, "3 {} {} {}", triangle.v0, triangle.v1, triangle.v2).unwrap();
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
    std::fs::create_dir_all(&output_directory)
        .unwrap_or_else(|error| panic!("failed to create {}: {error}", output_directory.display()));
    let case = if let Some(shift) = wide_rational_shift(&fixture) {
        wide_rational_overlapping_box_case(WIDE_RATIONAL_DIVISIONS, shift)
    } else {
        exact_mesh_pair(match fixture.as_str() {
            "clipped_voxel_torus_33" => clipped_voxel_torus_case(33),
            "clipped_voxel_torus_65" => clipped_voxel_torus_case(65),
            "dense_coplanar_boxes_4" => dense_coplanar_box_case(4),
            "dense_coplanar_boxes_16" => dense_coplanar_box_case(16),
            "dense_coplanar_boxes_32" => dense_coplanar_box_case(32),
            "sparse_multishell_tetrahedra_64" => sparse_multishell_tetrahedra_case(64),
            "sparse_multishell_tetrahedra_512" => sparse_multishell_tetrahedra_case(512),
            "subdivided_overlapping_boxes_3072_each" => large_boolean_case(),
            _ => corpus()
                .into_iter()
                .find(|case| case.name == fixture)
                .unwrap_or_else(|| panic!("unknown competitive fixture {fixture}")),
        })
    };
    let left = output_directory.join(format!("{}-left.off", case.name));
    let right = output_directory.join(format!("{}-right.off", case.name));
    write_exact_triangle_mesh(&left, &case.left);
    write_exact_triangle_mesh(&right, &case.right);
    println!("{}\n{}", left.display(), right.display());
}
