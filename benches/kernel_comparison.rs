//! Workloads with counterparts in `benchmarks/native` for direct kernel comparison.

mod support;

use std::hint::black_box;

use csgrs::{
    Real,
    csg::CSG,
    mesh::{Mesh, plane::Plane},
    sketch::Profile,
};
use hyperlattice::{Matrix4, Vector3};
use support::{Config, Measurement};

fn measurement(mesh: &Mesh<()>, input_facets: usize) -> Measurement {
    let facets = facet_count(mesh);
    let vertices = mesh
        .polygons
        .iter()
        .map(|polygon| polygon.vertices().len())
        .sum::<usize>();
    Measurement::new(
        input_facets as u64,
        facets as u64,
        (facets as u64).rotate_left(17) ^ vertices as u64,
    )
}

fn geometry_measurement(mesh: &Mesh<()>, input_facets: usize) -> Measurement {
    let facets = facet_count(mesh);
    let mut corners = 0_usize;
    let mut checksum = facets as u64;
    for polygon in &mesh.polygons {
        for vertex in polygon.vertices() {
            corners += 1;
            for coordinate in [&vertex.position.x, &vertex.position.y, &vertex.position.z] {
                checksum = checksum.rotate_left(7)
                    ^ coordinate.to_f64_lossy().unwrap_or_default().to_bits();
            }
        }
    }
    Measurement::new(input_facets as u64, facets as u64, checksum ^ corners as u64)
}

fn facet_count(mesh: &Mesh<()>) -> usize {
    mesh.polygons
        .iter()
        .map(|polygon| polygon.vertices().len().saturating_sub(2))
        .sum()
}

fn main() {
    #[cfg(feature = "dispatch-trace")]
    {
        hyperreal::dispatch_trace::reset();
        hyperreal::dispatch_trace::with_recording(run);
        let trace = hyperreal::dispatch_trace::take_trace();
        eprintln!("dispatch correlation: {:?}", trace.correlation_summary());
        for summary in trace.operation_summaries() {
            eprintln!(
                "dispatch operation: {}/{}/{}",
                summary.layer, summary.operation, summary.count
            );
        }
        for summary in trace.dispatch {
            eprintln!(
                "dispatch path: {}/{}/{}/{}",
                summary.layer, summary.operation, summary.path, summary.count
            );
        }
    }

    #[cfg(not(feature = "dispatch-trace"))]
    run();
}

fn run() {
    support::print_header();
    let config = Config::from_env();

    config.run("kernel", "construct_box", "unit", 64, || {
        let mesh = black_box(Mesh::cube(Real::from(2_u8), ()));
        measurement(&mesh, 0)
    });

    for (case, segments, stacks, iterations) in [("medium", 32, 16, 8), ("large", 64, 32, 2)] {
        config.run("kernel", "construct_sphere", case, iterations, || {
            let mesh = black_box(Mesh::sphere(Real::from(10_u8), segments, stacks, ()));
            measurement(&mesh, 0)
        });
    }
    config.run("precision", "construct_sphere", "high_resolution", 1, || {
        let mesh = black_box(Mesh::sphere(Real::from(10_u8), 128, 64, ()));
        measurement(&mesh, 0)
    });

    let transform_source = Mesh::sphere(Real::from(10_u8), 32, 16, ());
    let transform_input = facet_count(&transform_source);
    config.run("kernel", "translate", "sphere_medium", 8, || {
        let mesh = black_box(&transform_source).translate(
            Real::from(3_u8),
            Real::from(-2_i8),
            Real::from(5_u8),
        );
        geometry_measurement(&mesh, transform_input)
    });
    config.run("kernel", "rotate_xyz", "sphere_medium", 8, || {
        let mesh = black_box(&transform_source).rotate(
            Real::from(17_u8),
            Real::from(29_u8),
            Real::from(43_u8),
        );
        geometry_measurement(&mesh, transform_input)
    });
    let half = (Real::one() / Real::from(2_u8)).expect("nonzero denominator");
    let three_halves = (Real::from(3_u8) / Real::from(2_u8)).expect("nonzero denominator");
    config.run("kernel", "scale_nonuniform", "sphere_medium", 8, || {
        let mesh = black_box(&transform_source).scale(
            Real::from(2_u8),
            half.clone(),
            three_halves.clone(),
        );
        geometry_measurement(&mesh, transform_input)
    });
    let mirror_plane = Plane::from_normal(Vector3::x(), Real::one());
    config.run("kernel", "mirror", "sphere_across_x_eq_1", 8, || {
        let mesh = black_box(&transform_source).mirror(mirror_plane.clone());
        geometry_measurement(&mesh, transform_input)
    });
    let quarter = (Real::one() / Real::from(4_u8)).expect("nonzero denominator");
    let fifth = (Real::one() / Real::from(5_u8)).expect("nonzero denominator");
    let affine = Matrix4::from_row_major([
        Real::one(),
        quarter,
        Real::zero(),
        Real::from(2_u8),
        Real::zero(),
        Real::one(),
        fifth,
        Real::from(-3_i8),
        Real::zero(),
        Real::zero(),
        Real::one(),
        Real::from(4_u8),
        Real::zero(),
        Real::zero(),
        Real::zero(),
        Real::one(),
    ]);
    config.run("kernel", "affine_transform", "sphere_shear", 8, || {
        let mesh = black_box(&transform_source).transform(black_box(&affine));
        geometry_measurement(&mesh, transform_input)
    });
    config.run("kernel", "inverse", "sphere_orientation", 16, || {
        let mesh = black_box(&transform_source).inverse();
        geometry_measurement(&mesh, transform_input)
    });

    // Keep exact Boolean samples practical enough for repeated measurements.
    // Higher tessellation stress remains covered by construction/analysis cases.
    let boolean_left = Mesh::sphere(Real::from(10_u8), 12, 6, ());
    let boolean_right = Mesh::cube(Real::from(14_u8), ()).translate(
        Real::from(3_u8),
        Real::from(2_u8),
        Real::from(1_u8),
    );
    let boolean_input = facet_count(&boolean_left) + facet_count(&boolean_right);
    config.run("kernel", "boolean_union", "sphere_box", 1, || {
        let mesh = black_box(&boolean_left)
            .try_union(black_box(&boolean_right))
            .expect("comparison union must remain valid");
        measurement(&mesh, boolean_input)
    });
    config.run("kernel", "boolean_difference", "sphere_box", 1, || {
        let mesh = black_box(&boolean_left)
            .try_difference(black_box(&boolean_right))
            .expect("comparison difference must remain valid");
        measurement(&mesh, boolean_input)
    });
    config.run("kernel", "boolean_intersection", "sphere_box", 1, || {
        let mesh = black_box(&boolean_left)
            .try_intersection(black_box(&boolean_right))
            .expect("comparison intersection must remain valid");
        measurement(&mesh, boolean_input)
    });
    config.run("kernel", "boolean_xor", "sphere_box", 1, || {
        let mesh = black_box(&boolean_left)
            .try_xor(black_box(&boolean_right))
            .expect("comparison xor must remain valid");
        measurement(&mesh, boolean_input)
    });

    // An exact rational overlap ten times larger than OCCT's documented
    // Precision::Confusion() threshold. This anchors thin-feature behavior
    // without asking the double/tolerance kernel to resolve below its contract.
    let sliver_shift =
        (Real::from(1_999_999_u64) / Real::from(1_000_000_u64)).expect("nonzero denominator");
    let sliver_thickness =
        (Real::one() / Real::from(1_000_000_u64)).expect("nonzero denominator");
    let sliver_left = Mesh::cube(Real::from(2_u8), ());
    let sliver_right =
        Mesh::cube(Real::from(2_u8), ()).translate(sliver_shift, Real::zero(), Real::zero());
    let sliver_input = facet_count(&sliver_left) + facet_count(&sliver_right);
    config.run("precision", "boolean_sliver", "overlap_1e-6", 1, || {
        let mesh = black_box(&sliver_left)
            .try_intersection(black_box(&sliver_right))
            .expect("exact rational sliver intersection must remain valid");
        assert!(!mesh.polygons.is_empty(), "sliver intersection was lost");
        let bounds = mesh.bounding_box();
        assert_eq!(
            bounds.maxs.x.clone() - bounds.mins.x.clone(),
            sliver_thickness,
            "sliver thickness changed"
        );
        measurement(&mesh, sliver_input)
    });

    let profile = Profile::circle(Real::from(6_u8), 64);
    config.run("kernel", "extrude", "circle_64", 8, || {
        let mesh = black_box(&profile).extrude(Real::from(20_u8), ());
        measurement(&mesh, 64)
    });

    let analysis_source = Mesh::sphere(Real::from(10_u8), 32, 16, ());
    config.run("kernel", "triangulate", "sphere_medium", 16, || {
        let mesh = black_box(&analysis_source).triangulate();
        measurement(&mesh, facet_count(&analysis_source))
    });
    config.run("kernel", "bounding_box", "sphere_medium", 128, || {
        let bounds = black_box(&analysis_source).bounding_box();
        let checksum = bounds.maxs.x.to_f64_lossy().unwrap_or_default().to_bits();
        Measurement::new(facet_count(&analysis_source) as u64, 6, checksum)
    });
    config.run("kernel", "mass_properties", "sphere_medium", 4, || {
        let (mass, center, _) = black_box(&analysis_source)
            .mass_properties(Real::one())
            .expect("closed comparison sphere has mass properties");
        let checksum = mass.to_f64_lossy().unwrap_or_default().to_bits()
            ^ center.x.to_f64_lossy().unwrap_or_default().to_bits();
        Measurement::new(facet_count(&analysis_source) as u64, 10, checksum)
    });

    config.run("kernel", "stl_write", "sphere_medium", 8, || {
        let bytes = black_box(&analysis_source)
            .to_stl_binary("benchmark")
            .expect("comparison mesh is STL representable");
        Measurement::new(
            facet_count(&analysis_source) as u64,
            bytes.len() as u64,
            bytes.len() as u64,
        )
    });
}
