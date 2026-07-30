//! Workloads with counterparts in `benchmarks/native` for direct kernel comparison.

#[path = "../support/generated_corpus.rs"]
mod generated_corpus;
#[path = "../support/harness.rs"]
mod support;
#[path = "../support/yeahright.rs"]
#[allow(dead_code)]
mod yeahright_fixture;

use std::{
    collections::{HashMap, VecDeque},
    fs::File,
    hint::black_box,
    io::BufReader,
    num::NonZeroU32,
    path::{Path, PathBuf},
};

use csgrs::{
    AttributedMesh, Real, curve,
    solid::{self, SolidExt},
};
use hyperlattice::{Matrix4, Point3, Vector3};
use hypermesh::{Plane, Triangle, TriangleMesh};
use support::{Config, Measurement};

fn measurement(mesh: &TriangleMesh, input_facets: usize) -> Measurement {
    let facets = mesh.triangles.len();
    let vertices = mesh.positions.len();
    Measurement::new(
        input_facets as u64,
        facets as u64,
        (facets as u64).rotate_left(17) ^ vertices as u64,
    )
}

fn geometry_measurement(mesh: &TriangleMesh, input_facets: usize) -> Measurement {
    fn coordinate_fingerprint(coordinate: &Real) -> u64 {
        // `Real` may rebuild an equivalent symbolic approximation graph when a
        // transformed mesh is cloned. Hashing every raw f64 mantissa bit would
        // then make a harmless sub-nanometre approximation difference look like
        // nondeterministic geometry. Quantize only at this benchmark/export
        // boundary; all geometric construction and predicates remain exact.
        const UNITS_PER_COORDINATE: f64 = 1_000_000_000.0;
        coordinate
            .to_f64_lossy()
            .filter(|value| value.is_finite())
            .map(|value| (value * UNITS_PER_COORDINATE).round() as i64 as u64)
            .unwrap_or_default()
    }

    let facets = facet_count(mesh);
    let mut corners = 0_usize;
    let mut checksum = facets as u64;
    let finite_positions = mesh.finite_positions();
    for triangle in mesh.triangles.iter() {
        for index in triangle.indices() {
            corners += 1;
            let position = &mesh.positions[index];
            if let Some(position) = finite_positions.map(|positions| positions[index]) {
                for coordinate in position {
                    checksum = checksum.rotate_left(7)
                        ^ ((coordinate * 1_000_000_000.0).round() as i64 as u64);
                }
            } else {
                for coordinate in [&position.x, &position.y, &position.z] {
                    checksum = checksum.rotate_left(7) ^ coordinate_fingerprint(coordinate);
                }
            }
        }
    }
    Measurement::new(input_facets as u64, facets as u64, checksum ^ corners as u64)
}

fn facet_count(mesh: &TriangleMesh) -> usize {
    mesh.triangles.len()
}

fn triangle_normal(mesh: &TriangleMesh, triangle: Triangle) -> Vector3 {
    let [a, b, c] = triangle.indices().map(|index| {
        mesh.positions
            .get(index)
            .expect("benchmark triangle indices stay in range")
    });
    (b - a).cross(&(c - a))
}

fn yeahright_control_path() -> PathBuf {
    yeahright_fixture::control_mesh_path()
}

fn yeahright_boolean_hull_path() -> PathBuf {
    yeahright_fixture::boolean_hull_path()
}

fn import_obj(path: &Path) -> TriangleMesh {
    let file = File::open(path)
        .unwrap_or_else(|error| panic!("failed to open {}: {error}", path.display()));
    csgrs::io::obj::from_obj(BufReader::new(file))
        .unwrap_or_else(|error| panic!("failed to import {}: {error}", path.display()))
}

fn import_oriented_obj(path: &Path) -> TriangleMesh {
    orient_closed_triangle_mesh(&import_obj(path))
}

fn import_yeahright_control() -> TriangleMesh {
    import_oriented_obj(&yeahright_control_path())
}

fn import_yeahright_control_attributed() -> AttributedMesh<()> {
    let file = File::open(yeahright_control_path()).expect("YeahRight control OBJ");
    csgrs::io::obj::from_obj_attributed(BufReader::new(file))
        .expect("full-resolution attributed OBJ")
}

fn import_attributed_obj(path: &Path) -> AttributedMesh<()> {
    let file = File::open(path)
        .unwrap_or_else(|error| panic!("failed to open {}: {error}", path.display()));
    csgrs::io::obj::from_obj_attributed(BufReader::new(file))
        .unwrap_or_else(|error| panic!("failed to import {}: {error}", path.display()))
}

fn run_generated_corpus(config: &Config, case: &str, path: &Path) {
    let attributed = import_attributed_obj(path);
    let source = attributed.geometry();
    let input = facet_count(source);
    assert!(
        source.is_closed_manifold(),
        "generated corpus must stay closed and manifold"
    );

    config.run("corpus", "obj_import", case, 1, || {
        measurement(&black_box(import_obj(path)), input)
    });
    config.run("corpus", "translate", case, 1, || {
        geometry_measurement(
            &black_box(source).translated(Real::one(), Real::from(2_u8), Real::from(3_u8)),
            input,
        )
    });
    config.run("corpus", "bounding_box", case, 1, || {
        let bounds = solid::bounding_box(black_box(source));
        let checksum = bounds.maxs.x.to_f64_lossy().unwrap_or_default().to_bits()
            ^ bounds.maxs.y.to_f64_lossy().unwrap_or_default().to_bits()
            ^ bounds.maxs.z.to_f64_lossy().unwrap_or_default().to_bits();
        Measurement::new(input as u64, 6, checksum)
    });
    config.run("corpus", "graphics_buffers", case, 1, || {
        let graphics = black_box(&attributed)
            .exact_gpu_mesh_buffers()
            .expect("generated OBJ graphics conversion must remain valid");
        Measurement::new(
            input as u64,
            graphics.indices.len() as u64,
            (graphics.vertices.len() as u64).rotate_left(17) ^ graphics.indices.len() as u64,
        )
    });
    config.run("corpus", "connectivity", case, 1, || {
        let (vertices, adjacency) = black_box(source).connectivity_counts();
        Measurement::new(
            input as u64,
            vertices as u64,
            (vertices as u64).rotate_left(17) ^ adjacency as u64,
        )
    });
    config.run("corpus", "is_manifold", case, 1, || {
        Measurement::new(
            input as u64,
            1,
            u64::from(black_box(source).is_closed_manifold()),
        )
    });
}

fn orient_closed_triangle_mesh(source: &TriangleMesh) -> TriangleMesh {
    type EdgeIncidence = (usize, bool);

    #[derive(Default)]
    struct EdgeIncidences {
        rows: [Option<EdgeIncidence>; 2],
        overflow: bool,
    }

    impl EdgeIncidences {
        fn push(&mut self, incidence: EdgeIncidence) {
            if let Some(slot) = self.rows.iter_mut().find(|slot| slot.is_none()) {
                *slot = Some(incidence);
            } else {
                self.overflow = true;
            }
        }

        fn pair(&self) -> Option<[EdgeIncidence; 2]> {
            if self.overflow {
                return None;
            }
            Some([self.rows[0]?, self.rows[1]?])
        }
    }

    let mut triangles = source.triangles.to_vec();
    let mut edges = HashMap::<(usize, usize), EdgeIncidences>::new();
    for (triangle_index, triangle) in triangles.iter().enumerate() {
        let [a, b, c] = triangle.indices();
        for [a, b] in [[a, b], [b, c], [c, a]] {
            edges
                .entry((a.min(b), a.max(b)))
                .or_default()
                .push((triangle_index, a < b));
        }
    }
    assert!(
        edges.values().all(|incidence| incidence.pair().is_some()),
        "YeahRight control mesh must be closed before winding normalization"
    );

    let mut flipped = vec![None; triangles.len()];
    let mut components = Vec::<Vec<usize>>::new();
    let mut adjacent = vec![[None; 3]; triangles.len()];
    for incidence in edges.values() {
        let [(left, left_forward), (right, right_forward)] =
            incidence.pair().expect("closed edges have two incidences");
        let differs = left_forward == right_forward;
        *adjacent[left]
            .iter_mut()
            .find(|slot| slot.is_none())
            .expect("triangle has at most three neighboring faces") = Some((right, differs));
        *adjacent[right]
            .iter_mut()
            .find(|slot| slot.is_none())
            .expect("triangle has at most three neighboring faces") = Some((left, differs));
    }
    for seed in 0..triangles.len() {
        if flipped[seed].is_some() {
            continue;
        }
        flipped[seed] = Some(false);
        let mut queue = VecDeque::from([seed]);
        let mut component = Vec::new();
        while let Some(current) = queue.pop_front() {
            component.push(current);
            let current_flip = flipped[current].expect("queued triangles have orientation");
            for (neighbor, differs) in adjacent[current]
                .into_iter()
                .map(|slot| slot.expect("closed triangle has three neighbors"))
            {
                let required = current_flip ^ differs;
                if let Some(existing) = flipped[neighbor] {
                    assert_eq!(existing, required, "YeahRight surface must be orientable");
                } else {
                    flipped[neighbor] = Some(required);
                    queue.push_back(neighbor);
                }
            }
        }
        components.push(component);
    }
    let mut orientation_changed = false;
    for (triangle, flip) in triangles.iter_mut().zip(flipped) {
        if flip.expect("every triangle belongs to an oriented component") {
            std::mem::swap(&mut triangle.v1, &mut triangle.v2);
            orientation_changed = true;
        }
    }

    for component in components {
        let signed_volume = component.iter().fold(Real::zero(), |sum, &triangle_index| {
            let [a, b, c] = triangles[triangle_index]
                .indices()
                .map(|index| &source.positions[index]);
            sum + a.to_vector().dot(&b.to_vector().cross(&c.to_vector()))
        });
        match hyperlimit::classify_real_sign(&signed_volume).value() {
            Some(hyperlimit::Sign::Negative) => {
                for triangle_index in component {
                    let triangle = &mut triangles[triangle_index];
                    std::mem::swap(&mut triangle.v1, &mut triangle.v2);
                }
                orientation_changed = true;
            },
            Some(hyperlimit::Sign::Positive) => {},
            Some(hyperlimit::Sign::Zero) | None => {
                panic!("YeahRight component orientation must be certifiable")
            },
        }
    }

    // Preserve the native allocation when no winding changes are needed. The
    // control mesh is already outward-oriented, so rebuilding 11,894 rows here
    // would benchmark setup loss rather than OBJ import.
    if !orientation_changed {
        return source.clone();
    }

    TriangleMesh::new(source.positions.to_vec(), triangles)
}

fn yeahright_boolean_operand(source: &TriangleMesh) -> TriangleMesh {
    // The quarter turn has exact coefficients, and the offset keeps the two
    // genus-131 surfaces in substantial but non-identical overlap.
    solid::rotate(source, Real::zero(), Real::from(90_u8), Real::zero()).translated(
        Real::one(),
        Real::from(12_u8),
        Real::one(),
    )
}

fn main() {
    run();
}

fn run() {
    support::print_header();
    let config = Config::from_env();

    config.run("kernel", "construct_box", "unit", 64, || {
        let mesh = black_box(solid::cube(Real::from(2_u8)));
        measurement(&mesh, 0)
    });
    config.run("kernel", "construct_cuboid", "2x4x6", 32, || {
        let mesh = black_box(solid::cuboid(
            Real::from(2_u8),
            Real::from(4_u8),
            Real::from(6_u8),
        ));
        measurement(&mesh, 0)
    });
    config.run("kernel", "construct_cylinder", "r6_h20_s64", 8, || {
        let mesh = black_box(solid::cylinder(Real::from(6_u8), Real::from(20_u8), 64));
        measurement(&mesh, 0)
    });
    config.run("kernel", "construct_frustum", "r6_r2_h20_s64", 8, || {
        let mesh = black_box(solid::frustum(
            Real::from(6_u8),
            Real::from(2_u8),
            Real::from(20_u8),
            64,
        ));
        measurement(&mesh, 0)
    });
    config.run("kernel", "construct_octahedron", "r10", 32, || {
        measurement(&black_box(solid::octahedron(Real::from(10_u8))), 0)
    });
    config.run("kernel", "construct_icosahedron", "r10", 16, || {
        measurement(&black_box(solid::icosahedron(Real::from(10_u8))), 0)
    });

    for (case, segments, stacks, iterations) in [("medium", 32, 16, 8), ("large", 64, 32, 2)] {
        config.run("kernel", "construct_sphere", case, iterations, || {
            let mesh = black_box(solid::sphere(Real::from(10_u8), segments, stacks));
            measurement(&mesh, 0)
        });
    }
    config.run("precision", "construct_sphere", "high_resolution", 1, || {
        let mesh = black_box(solid::sphere(Real::from(10_u8), 128, 64));
        measurement(&mesh, 0)
    });
    config.run("kernel", "construct_ellipsoid", "r10_6_4_s32x16", 4, || {
        measurement(
            &black_box(solid::ellipsoid(
                Real::from(10_u8),
                Real::from(6_u8),
                Real::from(4_u8),
                32,
                16,
            )),
            0,
        )
    });
    config.run("kernel", "construct_torus", "r10_2_s32x16", 2, || {
        measurement(
            &black_box(solid::torus(Real::from(10_u8), Real::from(2_u8), 32, 16)),
            0,
        )
    });

    let transform_source = solid::sphere(Real::from(10_u8), 32, 16);
    let transform_input = facet_count(&transform_source);
    config.run("kernel", "translate", "sphere_medium", 8, || {
        let mesh = black_box(&transform_source).translated(
            Real::from(3_u8),
            Real::from(-2_i8),
            Real::from(5_u8),
        );
        geometry_measurement(&mesh, transform_input)
    });
    config.run("kernel", "rotate_xyz", "sphere_medium", 8, || {
        let mesh = solid::rotate(
            black_box(&transform_source),
            Real::from(17_u8),
            Real::from(29_u8),
            Real::from(43_u8),
        );
        geometry_measurement(&mesh, transform_input)
    });
    let half = (Real::one() / Real::from(2_u8)).expect("nonzero denominator");
    let three_halves = (Real::from(3_u8) / Real::from(2_u8)).expect("nonzero denominator");
    config.run("kernel", "scale_nonuniform", "sphere_medium", 8, || {
        let mesh = solid::scale(
            black_box(&transform_source),
            Real::from(2_u8),
            half.clone(),
            three_halves.clone(),
        );
        geometry_measurement(&mesh, transform_input)
    });
    let mirror_plane = Plane::axis_aligned(0, Real::one());
    config.run("kernel", "mirror", "sphere_across_x_eq_1", 8, || {
        let mesh = solid::mirror(black_box(&transform_source), &mirror_plane);
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
        let mesh = solid::transform(black_box(&transform_source), black_box(&affine));
        geometry_measurement(&mesh, transform_input)
    });
    config.run("kernel", "inverse", "sphere_orientation", 16, || {
        let mesh = solid::inverse(black_box(&transform_source));
        geometry_measurement(&mesh, transform_input)
    });
    let off_center = solid::cube(Real::from(2_u8)).translated(
        Real::from(7_u8),
        Real::from(-3_i8),
        Real::from(5_u8),
    );
    config.run("kernel", "center", "translated_box", 32, || {
        geometry_measurement(&solid::center(black_box(&off_center)), 12)
    });
    config.run("kernel", "scale_uniform", "sphere_medium", 8, || {
        geometry_measurement(
            &solid::scale(
                black_box(&transform_source),
                Real::from(2_u8),
                Real::from(2_u8),
                Real::from(2_u8),
            ),
            transform_input,
        )
    });

    // Keep exact Boolean samples practical enough for repeated measurements.
    // Higher tessellation stress remains covered by construction/analysis cases.
    let boolean_left = solid::sphere(Real::from(10_u8), 12, 6);
    let boolean_right = solid::cube(Real::from(14_u8)).translated(
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
    #[cfg(feature = "dispatch-trace")]
    black_box(boolean_left.try_difference(&boolean_right).unwrap());
    config.run("kernel", "boolean_difference", "sphere_box", 1, || {
        let mesh = black_box(&boolean_left)
            .try_difference(black_box(&boolean_right))
            .expect("comparison difference must remain valid");
        measurement(&mesh, boolean_input)
    });
    #[cfg(feature = "dispatch-trace")]
    black_box(boolean_left.try_intersection(&boolean_right).unwrap());
    config.run("kernel", "boolean_intersection", "sphere_box", 1, || {
        let mesh = black_box(&boolean_left)
            .try_intersection(black_box(&boolean_right))
            .expect("comparison intersection must remain valid");
        measurement(&mesh, boolean_input)
    });
    #[cfg(feature = "dispatch-trace")]
    black_box(boolean_left.try_xor(&boolean_right).unwrap());
    config.run("kernel", "boolean_xor", "sphere_box", 1, || {
        let mesh = black_box(&boolean_left)
            .try_xor(black_box(&boolean_right))
            .expect("comparison xor must remain valid");
        measurement(&mesh, boolean_input)
    });

    let topology_left = solid::cube(Real::from(4_u8));
    let topology_disjoint = solid::cube(Real::from(4_u8)).translated(
        Real::from(10_u8),
        Real::zero(),
        Real::zero(),
    );
    let topology_contained = solid::cube(Real::from(2_u8));
    let topology_touching =
        solid::cube(Real::from(4_u8)).translated(Real::from(4_u8), Real::zero(), Real::zero());
    config.run("kernel", "boolean_union", "disjoint_boxes", 8, || {
        measurement(
            &black_box(&topology_left)
                .try_union(black_box(&topology_disjoint))
                .unwrap(),
            24,
        )
    });
    config.run("kernel", "boolean_difference", "contained_boxes", 1, || {
        measurement(
            &black_box(&topology_left)
                .try_difference(black_box(&topology_contained))
                .unwrap(),
            24,
        )
    });
    config.run("kernel", "boolean_union", "face_touching_boxes", 1, || {
        measurement(
            &black_box(&topology_left)
                .try_union(black_box(&topology_touching))
                .unwrap(),
            24,
        )
    });
    config.run("kernel", "boolean_intersection", "identical_boxes", 8, || {
        measurement(
            &black_box(&topology_left)
                .try_intersection(black_box(&topology_left))
                .unwrap(),
            24,
        )
    });

    // An exact rational overlap ten times larger than OCCT's documented
    // Precision::Confusion() threshold. This anchors thin-feature behavior
    // without asking the double/tolerance kernel to resolve below its contract.
    let sliver_shift =
        (Real::from(1_999_999_u64) / Real::from(1_000_000_u64)).expect("nonzero denominator");
    let sliver_thickness =
        (Real::one() / Real::from(1_000_000_u64)).expect("nonzero denominator");
    let sliver_left = solid::cube(Real::from(2_u8));
    let sliver_right =
        solid::cube(Real::from(2_u8)).translated(sliver_shift, Real::zero(), Real::zero());
    let sliver_input = facet_count(&sliver_left) + facet_count(&sliver_right);
    config.run("precision", "boolean_sliver", "overlap_1e-6", 1, || {
        let mesh = black_box(&sliver_left)
            .try_intersection(black_box(&sliver_right))
            .expect("exact rational sliver intersection must remain valid");
        assert!(!mesh.triangles.is_empty(), "sliver intersection was lost");
        let bounds = solid::bounding_box(&mesh);
        assert_eq!(
            bounds.maxs.x.clone() - bounds.mins.x.clone(),
            sliver_thickness,
            "sliver thickness changed"
        );
        measurement(&mesh, sliver_input)
    });

    let profile = curve::circle(Real::from(6_u8), 64);
    config.run("kernel", "extrude", "circle_64", 8, || {
        let mesh = curve::extrude(black_box(&profile), Real::from(20_u8));
        measurement(&mesh, 64)
    });

    let distribution_source = solid::cube(Real::one());
    config.run("kernel", "distribute_linear", "box_8", 1, || {
        measurement(
            &solid::distribute_linear(
                black_box(&distribution_source),
                8,
                Vector3::x(),
                Real::from(2_u8),
            ),
            12,
        )
    });
    config.run("kernel", "distribute_grid", "box_4x4", 1, || {
        measurement(
            &solid::distribute_grid(
                black_box(&distribution_source),
                4,
                4,
                Real::from(2_u8),
                Real::from(2_u8),
            ),
            12,
        )
    });
    config.run(
        "kernel",
        "distribute_arc",
        "box_12_30_degree_steps",
        1,
        || {
            measurement(
                &solid::distribute_arc(
                    black_box(&distribution_source),
                    12,
                    Real::from(10_u8),
                    Real::zero(),
                    Real::from(330_u16),
                ),
                12,
            )
        },
    );

    let analysis_source = solid::sphere(Real::from(10_u8), 32, 16);
    config.run("kernel", "triangulate", "sphere_medium", 16, || {
        let mesh = black_box(&analysis_source).clone();
        measurement(&mesh, facet_count(&analysis_source))
    });
    config.run("kernel", "subdivide", "sphere_medium_level1", 2, || {
        let mesh = solid::subdivide(black_box(&analysis_source), NonZeroU32::new(1).unwrap());
        measurement(&mesh, facet_count(&analysis_source))
    });
    config.run("kernel", "renormalize", "sphere_medium", 4, || {
        let mesh = solid::renormalized(black_box(&analysis_source));
        geometry_measurement(&mesh, facet_count(&analysis_source))
    });
    config.run("kernel", "materialize_finite", "sphere_medium", 4, || {
        let mesh = solid::materialize_finite(black_box(&analysis_source))
            .expect("comparison sphere is finite");
        geometry_measurement(&mesh, facet_count(&analysis_source))
    });
    config.run("kernel", "bounding_box", "sphere_medium", 128, || {
        let bounds = solid::bounding_box(black_box(&analysis_source));
        let checksum = bounds.maxs.x.to_f64_lossy().unwrap_or_default().to_bits();
        Measurement::new(facet_count(&analysis_source) as u64, 6, checksum)
    });
    config.run("kernel", "mass_properties", "sphere_medium", 4, || {
        let report = solid::exact_mass_properties(black_box(&analysis_source), Real::one())
            .expect("closed comparison sphere has mass properties");
        let checksum = report.mass.to_f64_lossy().unwrap_or_default().to_bits()
            ^ report.center_of_mass.0[0]
                .to_f64_lossy()
                .unwrap_or_default()
                .to_bits();
        Measurement::new(facet_count(&analysis_source) as u64, 10, checksum)
    });
    config.run("kernel", "vertices", "sphere_medium", 32, || {
        let facets = black_box(&analysis_source).triangles.len();
        let vertices = analysis_source.positions.len();
        Measurement::new(facets as u64, vertices as u64, vertices as u64)
    });
    config.run("kernel", "graphics_buffers", "sphere_medium", 16, || {
        let graphics = black_box(&analysis_source)
            .exact_gpu_mesh_buffers()
            .expect("exact graphics conversion must remain valid");
        Measurement::new(
            facet_count(&analysis_source) as u64,
            graphics.indices.len() as u64,
            (graphics.vertices.len() as u64).rotate_left(17) ^ graphics.indices.len() as u64,
        )
    });
    config.run("kernel", "connectivity", "sphere_medium", 8, || {
        let (vertices, adjacency) = black_box(&analysis_source).connectivity_counts();
        Measurement::new(
            facet_count(&analysis_source) as u64,
            vertices as u64,
            (vertices as u64).rotate_left(17) ^ adjacency as u64,
        )
    });
    config.run("kernel", "is_manifold", "sphere_medium", 32, || {
        let manifold = black_box(&analysis_source).is_closed_manifold();
        Measurement::new(facet_count(&analysis_source) as u64, 1, u64::from(manifold))
    });
    config.run("kernel", "contains_point", "sphere_two_queries", 8, || {
        let inside = solid::contains_point(black_box(&analysis_source), &Point3::origin())
            .expect("certified containment");
        let outside = solid::contains_point(
            black_box(&analysis_source),
            &Point3::new(Real::from(20_u8), Real::zero(), Real::zero()),
        )
        .expect("certified containment");
        Measurement::new(
            facet_count(&analysis_source) as u64,
            2,
            u64::from(inside) | (u64::from(outside) << 1),
        )
    });
    config.run("kernel", "ray_intersections", "sphere_diameter", 8, || {
        let hits = solid::ray_intersections(
            black_box(&analysis_source),
            &Point3::new(Real::from(-20_i8), Real::zero(), Real::zero()),
            &Vector3::x(),
        )
        .expect("certified ray intersections");
        Measurement::new(
            facet_count(&analysis_source) as u64,
            hits.len() as u64,
            hits.len() as u64,
        )
    });
    config.run(
        "kernel",
        "polyline_intersections",
        "sphere_diameter",
        8,
        || {
            let hits = solid::polyline_intersections(
                black_box(&analysis_source),
                &[
                    Point3::new(Real::from(-20_i8), Real::zero(), Real::zero()),
                    Point3::new(Real::from(20_i8), Real::zero(), Real::zero()),
                ],
            )
            .expect("certified polyline intersections");
            Measurement::new(
                facet_count(&analysis_source) as u64,
                hits.len() as u64,
                hits.len() as u64,
            )
        },
    );
    config.run("kernel", "dihedral_angle", "box_adjacent_faces", 32, || {
        let box_mesh = solid::cube(Real::from(2_u8));
        let first = box_mesh.triangles[0];
        let first_normal = triangle_normal(&box_mesh, first);
        let second = box_mesh
            .triangles
            .iter()
            .copied()
            .find(|triangle| {
                hyperlimit::classify_real_sign(
                    &first_normal.dot(&triangle_normal(&box_mesh, *triangle)),
                )
                .value()
                    == Some(hyperlimit::Sign::Zero)
            })
            .expect("cube has an adjacent orthogonal face");
        let angle =
            solid::dihedral_angle(&box_mesh, first, second).expect("cube triangles are valid");
        Measurement::new(12, 1, angle.to_f64_lossy().unwrap_or_default().to_bits())
    });

    run_generated_corpus(
        &config,
        "deterministic_concave_labyrinth_31x31x6",
        &generated_corpus::concave_path(),
    );
    run_generated_corpus(
        &config,
        "sierpinski_foam_level3",
        &generated_corpus::sierpinski_foam_path(),
    );

    if yeahright_fixture::enabled() {
        config.run(
            "corpus",
            "obj_import",
            "yeahright_control_genus131",
            1,
            || measurement(&black_box(import_yeahright_control()), 5_845),
        );

        let yeahright_attributed = import_yeahright_control_attributed();
        let yeahright_source = yeahright_attributed.geometry().clone();
        let yeahright_input = facet_count(&yeahright_source);

        config.run(
            "corpus",
            "rotate_translate",
            "yeahright_control_rot90_offset",
            1,
            || {
                geometry_measurement(
                    &yeahright_boolean_operand(black_box(&yeahright_source)),
                    yeahright_input,
                )
            },
        );
        config.run(
            "corpus",
            "bounding_box",
            "yeahright_control_genus131",
            1,
            || {
                let bounds = solid::bounding_box(black_box(&yeahright_source));
                let checksum = bounds.maxs.x.to_f64_lossy().unwrap_or_default().to_bits()
                    ^ bounds.maxs.y.to_f64_lossy().unwrap_or_default().to_bits()
                    ^ bounds.maxs.z.to_f64_lossy().unwrap_or_default().to_bits();
                Measurement::new(yeahright_input as u64, 6, checksum)
            },
        );
        config.run(
            "corpus",
            "graphics_buffers",
            "yeahright_control_genus131",
            1,
            || {
                let graphics = black_box(&yeahright_attributed)
                    .exact_gpu_mesh_buffers()
                    .expect("authored OBJ graphics conversion must remain valid");
                Measurement::new(
                    yeahright_input as u64,
                    graphics.indices.len() as u64,
                    (graphics.vertices.len() as u64).rotate_left(17)
                        ^ graphics.indices.len() as u64,
                )
            },
        );
        config.run(
            "corpus",
            "connectivity",
            "yeahright_control_genus131",
            1,
            || {
                let (vertices, adjacency) = black_box(&yeahright_source).connectivity_counts();
                Measurement::new(
                    yeahright_input as u64,
                    vertices as u64,
                    (vertices as u64).rotate_left(17) ^ adjacency as u64,
                )
            },
        );
        config.run(
            "corpus",
            "is_manifold",
            "yeahright_control_genus131",
            1,
            || {
                let manifold = black_box(&yeahright_source).is_closed_manifold();
                Measurement::new(yeahright_input as u64, 1, u64::from(manifold))
            },
        );

        let yeahright_boolean_source = import_oriented_obj(&yeahright_boolean_hull_path());
        let yeahright_box = solid::cuboid(
            Real::from(20_u8),
            Real::from(40_u8),
            Real::from(40_u8),
        )
        .translated(Real::from(-10_i8), Real::from(6_u8), Real::zero());
        let yeahright_box_input =
            facet_count(&yeahright_boolean_source) + facet_count(&yeahright_box);
        config.run("corpus", "boolean_all", "yeahright_hull_box", 1, || {
            let source = black_box(&yeahright_boolean_source);
            let clipping_box = black_box(&yeahright_box);
            let outputs = [
                source
                    .try_union(clipping_box)
                    .expect("YeahRight box union must remain valid"),
                source
                    .try_difference(clipping_box)
                    .expect("YeahRight box difference must remain valid"),
                source
                    .try_intersection(clipping_box)
                    .expect("YeahRight box intersection must remain valid"),
                source
                    .try_xor(clipping_box)
                    .expect("YeahRight box xor must remain valid"),
            ];
            assert!(
                !outputs[2].triangles.is_empty(),
                "YeahRight proxy must intersect the clipping box"
            );
            outputs.iter().fold(Measurement::default(), |total, output| {
                let current = measurement(output, yeahright_box_input);
                Measurement::new(
                    total.work_units.saturating_add(current.work_units),
                    total.output_size.saturating_add(current.output_size),
                    total.checksum.wrapping_add(current.checksum),
                )
            })
        });
        let yeahright_stress_source = yeahright_boolean_source.clone();
        let yeahright_copy = yeahright_boolean_operand(&yeahright_stress_source);
        let yeahright_boolean_input =
            facet_count(&yeahright_stress_source) + facet_count(&yeahright_copy);
        config.run(
            "stress",
            "boolean_union",
            "yeahright_control_hull_rot90_offset",
            1,
            || {
                let output = black_box(&yeahright_stress_source)
                    .try_union(black_box(&yeahright_copy))
                    .expect("YeahRight union must remain valid");
                measurement(&output, yeahright_boolean_input)
            },
        );
        config.run(
            "stress",
            "boolean_difference",
            "yeahright_control_hull_rot90_offset",
            1,
            || {
                let output = black_box(&yeahright_stress_source)
                    .try_difference(black_box(&yeahright_copy))
                    .expect("YeahRight difference must remain valid");
                measurement(&output, yeahright_boolean_input)
            },
        );
        config.run(
            "stress",
            "boolean_intersection",
            "yeahright_control_hull_rot90_offset",
            1,
            || {
                let output = black_box(&yeahright_stress_source)
                    .try_intersection(black_box(&yeahright_copy))
                    .expect("YeahRight intersection must remain valid");
                assert!(
                    !output.triangles.is_empty(),
                    "YeahRight stress operands must overlap"
                );
                measurement(&output, yeahright_boolean_input)
            },
        );
        config.run(
            "stress",
            "boolean_xor",
            "yeahright_control_hull_rot90_offset",
            1,
            || {
                let output = black_box(&yeahright_stress_source)
                    .try_xor(black_box(&yeahright_copy))
                    .expect("YeahRight xor must remain valid");
                measurement(&output, yeahright_boolean_input)
            },
        );

        // Opt-in only: this exact 11,894-by-11,894-triangle preparation reached
        // roughly 116 GiB RSS and invoked the Linux OOM killer during validation.
        let yeahright_dangerous_copy = yeahright_boolean_operand(&yeahright_source);
        config.run(
            "dangerous",
            "boolean_intersection",
            "yeahright_control_full_rot90_offset_dangerous",
            1,
            || {
                let output = black_box(&yeahright_source)
                    .try_intersection(black_box(&yeahright_dangerous_copy))
                    .expect("full-resolution YeahRight intersection must remain valid");
                measurement(&output, yeahright_input * 2)
            },
        );
    }

    config.run("kernel", "stl_write", "sphere_medium", 8, || {
        let bytes = csgrs::io::stl::to_stl_binary(black_box(&analysis_source), "benchmark")
            .expect("comparison mesh is STL representable");
        Measurement::new(
            facet_count(&analysis_source) as u64,
            bytes.len() as u64,
            bytes.len() as u64,
        )
    });
}
