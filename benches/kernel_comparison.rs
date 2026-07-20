//! Workloads with counterparts in `benchmarks/native` for direct kernel comparison.

mod support;

use std::{
    collections::{HashMap, VecDeque},
    fs::File,
    hint::black_box,
    io::BufReader,
    num::NonZeroU32,
    path::{Path, PathBuf},
};

use csgrs::{
    Real,
    csg::CSG,
    mesh::{Mesh, plane::Plane},
    sketch::Profile,
    triangulated::IndexedTriangleMesh3D,
};
use hyperlattice::{Matrix4, Point3, Vector3};
use support::{Config, Measurement};

fn measurement(mesh: &Mesh<()>, input_facets: usize) -> Measurement {
    let (facets, vertices) = mesh.topology_counts();
    Measurement::new(
        input_facets as u64,
        facets as u64,
        (facets as u64).rotate_left(17) ^ vertices as u64,
    )
}

fn geometry_measurement(mesh: &Mesh<()>, input_facets: usize) -> Measurement {
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
    for polygon in &mesh.polygons {
        for (polygon_corner, position_f64) in polygon.position_f64_iter().enumerate() {
            corners += 1;
            if let Some(position) = position_f64 {
                for coordinate in position {
                    checksum = checksum.rotate_left(7)
                        ^ ((coordinate * 1_000_000_000.0).round() as i64 as u64);
                }
            } else {
                let vertex = polygon
                    .vertex_iter()
                    .nth(polygon_corner)
                    .expect("position and exact vertex iterators have equal length");
                for coordinate in [&vertex.position.x, &vertex.position.y, &vertex.position.z]
                {
                    checksum = checksum.rotate_left(7) ^ coordinate_fingerprint(coordinate);
                }
            }
        }
    }
    Measurement::new(input_facets as u64, facets as u64, checksum ^ corners as u64)
}

fn facet_count(mesh: &Mesh<()>) -> usize {
    mesh.topology_counts().0
}

fn yeahright_control_path() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("benchmarks/data/yeahright/controlmesh.obj")
}

fn yeahright_boolean_proxy_path() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("benchmarks/data/yeahright/controlmesh_boolean_proxy.obj")
}

fn yeahright_boolean_hull_path() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("benchmarks/data/yeahright/yeahright_boolean_hull.obj")
}

fn import_oriented_obj(path: &Path) -> Mesh<()> {
    let file = File::open(path)
        .unwrap_or_else(|error| panic!("failed to open {}: {error}", path.display()));
    let mesh = Mesh::from_obj(BufReader::new(file), ())
        .unwrap_or_else(|error| panic!("failed to import {}: {error}", path.display()));
    orient_closed_triangle_mesh(&mesh)
}

fn import_yeahright_control() -> Mesh<()> {
    import_oriented_obj(&yeahright_control_path())
}

fn orient_closed_triangle_mesh(source: &Mesh<()>) -> Mesh<()> {
    type EdgeIncidence = (usize, bool);

    let buffers = source.to_hypermesh_buffers();
    let mut triangles = buffers
        .indices
        .chunks_exact(3)
        .map(|indices| [indices[0], indices[1], indices[2]])
        .collect::<Vec<_>>();
    assert_eq!(
        source.polygons.len(),
        triangles.len(),
        "oriented OBJ source must already be triangulated"
    );
    let mut triangle_normals: Vec<[Vector3; 3]> = source
        .polygons
        .iter()
        .map(|polygon| {
            let mut vertices = polygon.vertex_iter();
            std::array::from_fn(|_| {
                vertices
                    .next()
                    .expect("triangulated polygon has three vertices")
                    .normal
                    .clone()
            })
        })
        .collect::<Vec<_>>();
    let mut edges = HashMap::<(usize, usize), Vec<EdgeIncidence>>::new();
    for (triangle_index, triangle) in triangles.iter().enumerate() {
        for [a, b] in [
            [triangle[0], triangle[1]],
            [triangle[1], triangle[2]],
            [triangle[2], triangle[0]],
        ] {
            edges
                .entry((a.min(b), a.max(b)))
                .or_default()
                .push((triangle_index, a < b));
        }
    }
    assert!(
        edges.values().all(|incidence| incidence.len() == 2),
        "YeahRight control mesh must be closed before winding normalization"
    );

    let mut flipped = vec![None; triangles.len()];
    let mut components = Vec::<Vec<usize>>::new();
    let mut adjacent = vec![Vec::<(usize, bool)>::new(); triangles.len()];
    for incidence in edges.values() {
        let (left, left_forward) = incidence[0];
        let (right, right_forward) = incidence[1];
        let differs = left_forward == right_forward;
        adjacent[left].push((right, differs));
        adjacent[right].push((left, differs));
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
            for &(neighbor, differs) in &adjacent[current] {
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
    for (triangle_index, (triangle, flip)) in triangles.iter_mut().zip(flipped).enumerate() {
        if flip.expect("every triangle belongs to an oriented component") {
            triangle.swap(1, 2);
            triangle_normals[triangle_index].swap(1, 2);
            orientation_changed = true;
        }
    }

    for component in components {
        let signed_volume = component
            .iter()
            .map(|&triangle_index| {
                let triangle = triangles[triangle_index];
                let position = |index: usize| {
                    let offset = index * 3;
                    [
                        buffers.positions[offset].to_f64_lossy().unwrap_or_default(),
                        buffers.positions[offset + 1]
                            .to_f64_lossy()
                            .unwrap_or_default(),
                        buffers.positions[offset + 2]
                            .to_f64_lossy()
                            .unwrap_or_default(),
                    ]
                };
                let [ax, ay, az] = position(triangle[0]);
                let [bx, by, bz] = position(triangle[1]);
                let [cx, cy, cz] = position(triangle[2]);
                ax * (by * cz - bz * cy) + ay * (bz * cx - bx * cz) + az * (bx * cy - by * cx)
            })
            .sum::<f64>();
        if signed_volume < 0.0 {
            for triangle_index in component {
                triangles[triangle_index].swap(1, 2);
                triangle_normals[triangle_index].swap(1, 2);
            }
            orientation_changed = true;
        }
    }

    // Preserve the source mesh's indexed carrier and retained construction
    // facts when normalization proves that no triangle needs flipping. The
    // YeahRight corpus is already consistently outward-oriented, so rebuilding
    // 11,894 polygons here would benchmark setup loss rather than OBJ import.
    if !orientation_changed {
        return source.clone();
    }

    let positions = buffers
        .positions
        .chunks_exact(3)
        .map(|coordinates| {
            Point3::new(
                coordinates[0].clone(),
                coordinates[1].clone(),
                coordinates[2].clone(),
            )
        })
        .collect::<Vec<_>>();
    let mut normals = Vec::<Vector3>::new();
    let mut position_normals = vec![Vec::<(Vector3, usize)>::new(); positions.len()];
    let faces = triangles
        .into_iter()
        .zip(triangle_normals)
        .map(|(triangle, triangle_normals)| {
            std::array::from_fn(|corner| {
                let position = triangle[corner];
                let normal = &triangle_normals[corner];
                let normal_index = position_normals[position]
                    .iter()
                    .find(|(existing, _)| existing == normal)
                    .map(|(_, index)| *index)
                    .unwrap_or_else(|| {
                        let index = normals.len();
                        normals.push(normal.clone());
                        position_normals[position].push((normal.clone(), index));
                        index
                    });
                (position, normal_index)
            })
        })
        .collect();
    Mesh::from_indexed_triangles(
        IndexedTriangleMesh3D {
            positions,
            normals,
            faces,
        },
        (),
    )
    .expect("oriented YeahRight triangle rows stay in range")
}

fn yeahright_boolean_operand(source: &Mesh<()>) -> Mesh<()> {
    // The quarter turn has exact coefficients, and the offset keeps the two
    // genus-131 surfaces in substantial but non-identical overlap.
    source
        .rotate(Real::zero(), Real::from(90_u8), Real::zero())
        .translate(Real::one(), Real::from(12_u8), Real::one())
}

fn main() {
    run();
}

fn run() {
    support::print_header();
    let config = Config::from_env();

    config.run("kernel", "construct_box", "unit", 64, || {
        let mesh = black_box(Mesh::cube(Real::from(2_u8), ()));
        measurement(&mesh, 0)
    });
    config.run("kernel", "construct_cuboid", "2x4x6", 32, || {
        let mesh = black_box(Mesh::cuboid(
            Real::from(2_u8),
            Real::from(4_u8),
            Real::from(6_u8),
            (),
        ));
        measurement(&mesh, 0)
    });
    config.run("kernel", "construct_cylinder", "r6_h20_s64", 8, || {
        let mesh = black_box(Mesh::cylinder(Real::from(6_u8), Real::from(20_u8), 64, ()));
        measurement(&mesh, 0)
    });
    config.run("kernel", "construct_frustum", "r6_r2_h20_s64", 8, || {
        let mesh = black_box(Mesh::frustum(
            Real::from(6_u8),
            Real::from(2_u8),
            Real::from(20_u8),
            64,
            (),
        ));
        measurement(&mesh, 0)
    });
    config.run("kernel", "construct_octahedron", "r10", 32, || {
        measurement(&black_box(Mesh::octahedron(Real::from(10_u8), ())), 0)
    });
    config.run("kernel", "construct_icosahedron", "r10", 16, || {
        measurement(&black_box(Mesh::icosahedron(Real::from(10_u8), ())), 0)
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
    config.run("kernel", "construct_ellipsoid", "r10_6_4_s32x16", 4, || {
        measurement(
            &black_box(Mesh::ellipsoid(
                Real::from(10_u8),
                Real::from(6_u8),
                Real::from(4_u8),
                32,
                16,
                (),
            )),
            0,
        )
    });
    config.run("kernel", "construct_torus", "r10_2_s32x16", 2, || {
        measurement(
            &black_box(Mesh::torus(Real::from(10_u8), Real::from(2_u8), 32, 16, ())),
            0,
        )
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
    let off_center = Mesh::cube(Real::from(2_u8), ()).translate(
        Real::from(7_u8),
        Real::from(-3_i8),
        Real::from(5_u8),
    );
    config.run("kernel", "center", "translated_box", 32, || {
        geometry_measurement(&black_box(&off_center).center(), 12)
    });
    config.run("kernel", "scale_uniform", "sphere_medium", 8, || {
        geometry_measurement(
            &black_box(&transform_source).scale(
                Real::from(2_u8),
                Real::from(2_u8),
                Real::from(2_u8),
            ),
            transform_input,
        )
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

    let topology_left = Mesh::cube(Real::from(4_u8), ());
    let topology_disjoint = Mesh::cube(Real::from(4_u8), ()).translate(
        Real::from(10_u8),
        Real::zero(),
        Real::zero(),
    );
    let topology_contained = Mesh::cube(Real::from(2_u8), ());
    let topology_touching = Mesh::cube(Real::from(4_u8), ()).translate(
        Real::from(4_u8),
        Real::zero(),
        Real::zero(),
    );
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

    let distribution_source = Mesh::cube(Real::one(), ());
    config.run("kernel", "distribute_linear", "box_8", 1, || {
        measurement(
            &black_box(&distribution_source).distribute_linear(
                8,
                Vector3::x(),
                Real::from(2_u8),
            ),
            12,
        )
    });
    config.run("kernel", "distribute_grid", "box_4x4", 1, || {
        measurement(
            &black_box(&distribution_source).distribute_grid(
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
                &black_box(&distribution_source).distribute_arc(
                    12,
                    Real::from(10_u8),
                    Real::zero(),
                    Real::from(330_u16),
                ),
                12,
            )
        },
    );

    let analysis_source = Mesh::sphere(Real::from(10_u8), 32, 16, ());
    config.run("kernel", "triangulate", "sphere_medium", 16, || {
        let mesh = black_box(&analysis_source).triangulate();
        measurement(&mesh, facet_count(&analysis_source))
    });
    config.run("kernel", "subdivide", "sphere_medium_level1", 2, || {
        let mesh =
            black_box(&analysis_source).subdivide_triangles(NonZeroU32::new(1).unwrap());
        measurement(&mesh, facet_count(&analysis_source))
    });
    config.run("kernel", "renormalize", "sphere_medium", 4, || {
        let mut mesh = black_box(&analysis_source).clone();
        mesh.renormalize();
        geometry_measurement(&mesh, facet_count(&analysis_source))
    });
    config.run("kernel", "materialize_finite", "sphere_medium", 4, || {
        let mesh = black_box(&analysis_source)
            .materialize_finite_output()
            .expect("comparison sphere is finite");
        geometry_measurement(&mesh, facet_count(&analysis_source))
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
    config.run("kernel", "vertices", "sphere_medium", 32, || {
        let (facets, vertices) = black_box(&analysis_source).topology_counts();
        Measurement::new(facets as u64, vertices as u64, vertices as u64)
    });
    config.run("kernel", "graphics_buffers", "sphere_medium", 16, || {
        let graphics = black_box(&analysis_source).build_graphics_mesh();
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
        let manifold = black_box(&analysis_source).is_manifold();
        Measurement::new(facet_count(&analysis_source) as u64, 1, u64::from(manifold))
    });
    config.run("kernel", "contains_point", "sphere_two_queries", 8, || {
        let inside = black_box(&analysis_source).contains_vertex(&Point3::origin());
        let outside = black_box(&analysis_source).contains_vertex(&Point3::new(
            Real::from(20_u8),
            Real::zero(),
            Real::zero(),
        ));
        Measurement::new(
            facet_count(&analysis_source) as u64,
            2,
            u64::from(inside) | (u64::from(outside) << 1),
        )
    });
    config.run("kernel", "ray_intersections", "sphere_diameter", 8, || {
        let hits = black_box(&analysis_source).ray_intersections(
            &Point3::new(Real::from(-20_i8), Real::zero(), Real::zero()),
            &Vector3::x(),
        );
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
            let hits = black_box(&analysis_source).intersect_polyline(&[
                Point3::new(Real::from(-20_i8), Real::zero(), Real::zero()),
                Point3::new(Real::from(20_i8), Real::zero(), Real::zero()),
            ]);
            Measurement::new(
                facet_count(&analysis_source) as u64,
                hits.len() as u64,
                hits.len() as u64,
            )
        },
    );
    config.run("kernel", "dihedral_angle", "box_adjacent_faces", 32, || {
        let box_mesh = Mesh::cube(Real::from(2_u8), ());
        let first = &box_mesh.polygons[0];
        let second = box_mesh
            .polygons
            .iter()
            .find(|polygon| {
                first.plane().normal().dot(&polygon.plane().normal()) == Real::zero()
            })
            .expect("cube has an adjacent orthogonal face");
        let angle = Mesh::dihedral_angle(first, second);
        Measurement::new(12, 1, angle.to_f64_lossy().unwrap_or_default().to_bits())
    });

    config.run(
        "corpus",
        "obj_import",
        "yeahright_control_genus131",
        1,
        || measurement(&black_box(import_yeahright_control()), 5_845),
    );

    let yeahright_source = import_yeahright_control();
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
            let bounds = black_box(&yeahright_source).bounding_box();
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
            let graphics = black_box(&yeahright_source).build_graphics_mesh();
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
            let manifold = black_box(&yeahright_source).is_manifold();
            Measurement::new(yeahright_input as u64, 1, u64::from(manifold))
        },
    );

    let yeahright_boolean_source = import_oriented_obj(&yeahright_boolean_hull_path());
    let yeahright_box =
        Mesh::cuboid(Real::from(20_u8), Real::from(40_u8), Real::from(40_u8), ()).translate(
            Real::from(-10_i8),
            Real::from(6_u8),
            Real::zero(),
        );
    let yeahright_box_input =
        facet_count(&yeahright_boolean_source) + facet_count(&yeahright_box);
    config.run("corpus", "boolean_all", "yeahright_hull_box", 1, || {
        let prepared = black_box(&yeahright_boolean_source)
            .try_prepare_boolean(black_box(&yeahright_box))
            .expect("YeahRight box Booleans must prepare");
        let outputs = [
            prepared
                .try_union()
                .expect("YeahRight box union must remain valid"),
            prepared
                .try_difference()
                .expect("YeahRight box difference must remain valid"),
            prepared
                .try_intersection()
                .expect("YeahRight box intersection must remain valid"),
            prepared
                .try_xor()
                .expect("YeahRight box xor must remain valid"),
        ];
        assert!(
            !outputs[2].polygons.is_empty(),
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
    let yeahright_stress_source = import_oriented_obj(&yeahright_boolean_proxy_path());
    let yeahright_copy = yeahright_boolean_operand(&yeahright_stress_source);
    let yeahright_boolean_input =
        facet_count(&yeahright_stress_source) + facet_count(&yeahright_copy);
    config.run(
        "stress",
        "boolean_union",
        "yeahright_genus131_proxy_rot90_offset",
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
        "yeahright_genus131_proxy_rot90_offset",
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
        "yeahright_genus131_proxy_rot90_offset",
        1,
        || {
            let output = black_box(&yeahright_stress_source)
                .try_intersection(black_box(&yeahright_copy))
                .expect("YeahRight intersection must remain valid");
            assert!(
                !output.polygons.is_empty(),
                "YeahRight stress operands must overlap"
            );
            measurement(&output, yeahright_boolean_input)
        },
    );
    config.run(
        "stress",
        "boolean_xor",
        "yeahright_genus131_proxy_rot90_offset",
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
