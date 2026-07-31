//! Generate lightweight PNG renders for README assets.
//!
//! Geometry is constructed with `hyperlattice::Real`; primitive floats are used
//! only after projection into the image/raster boundary.

use csgrs::solid::MetaBall;
use csgrs::{
    GeometryContext, TriangleMesh, curve,
    solid::{self, SolidExt},
};
use hypercurve::{
    Classification, CurvePath2, CurvePolicy, CurveRegion2, CurveString2,
    FiniteProjectionOptions, FiniteRegionProfile2, Point2,
};
use hyperlattice::{Point3, Real, Vector3};
use image::{GrayImage, Luma, Rgba, RgbaImage};
use std::{collections::BTreeSet, fs, path::PathBuf};

const SIZE: u32 = 768;
const PADDING: f64 = 0.12;
const CURVE_CHORD_ERROR: f64 = 1.0e-3;
const BG: Rgba<u8> = Rgba([0, 0, 0, 0]);
const FACE_2D: Rgba<u8> = Rgba([68, 150, 173, 255]);
const FACE_SHADOW_3D: Rgba<u8> = Rgba([54, 108, 142, 255]);
const FACE_LIGHT_3D: Rgba<u8> = Rgba([120, 201, 204, 255]);
const EDGE: Rgba<u8> = Rgba([16, 34, 49, 255]);
const VERTEX: Rgba<u8> = Rgba([255, 203, 79, 255]);
const EDGE_WIDTH_2D: f64 = 3.25;
const VIEW_DIRECTION: [f64; 3] = [1.0, 1.0, 0.9];
const LIGHT_DIRECTION: [f64; 3] = [0.25, 0.55, 1.0];

fn main() {
    let output_dir = output_dir();
    fs::create_dir_all(&output_dir).expect("create README render output directory");

    render_readme_curves();
    render_readme_meshes();

    println!("README renders written to {}/*.png", output_dir.display());
}

fn render_readme_curves() {
    render_curve("square", &curve::square(r(2.0)));
    render_curve("rectangle", &curve::rectangle(r(2.4), r(1.35)));
    render_curve("circle", &curve::circle(r(1.0), 96));
    render_curve("right_triangle", &curve::right_triangle(r(2.0), r(1.5)));
    render_curve(
        "polygon",
        &curve::polygon(&[[r(0.0), r(1.2)], [r(-1.1), r(-0.8)], [r(1.1), r(-0.8)]]),
    );
    render_curve(
        "rounded_rectangle",
        &curve::rounded_rectangle(r(2.4), r(1.5), r(0.28), 12),
    );
    render_curve("ellipse", &curve::ellipse(r(2.3), r(1.3), 96));
    render_curve("regular_ngon", &curve::regular_ngon(6, r(1.0)));
    render_curve("curve_arrow", &curve::arrow(r(2.2), r(0.35), r(0.8), r(1.0)));
    render_curve(
        "trapezoid",
        &curve::trapezoid(r(1.2), r(2.2), r(1.4), r(0.45)),
    );
    render_curve("star", &curve::star(5, r(1.1), r(0.45)));
    render_curve("teardrop", &curve::teardrop(r(1.5), r(2.3), 96));
    render_curve("curve_egg", &curve::egg(r(1.6), r(2.2), 96));
    render_curve("squircle", &curve::squircle(r(2.0), r(1.6), 128));
    render_curve(
        "keyhole",
        &curve::keyhole(
            r(0.75),
            r(0.55),
            r(1.2),
            96,
            &GeometryContext::APPROXIMATE_512,
        )
        .expect("keyhole Boolean")
        .into_value(),
    );
    render_curve(
        "reuleaux",
        &curve::reuleaux(3, r(1.5), 96, &GeometryContext::APPROXIMATE_512)
            .expect("Reuleaux intersections")
            .into_value(),
    );
    render_curve("heart", &curve::heart(r(2.0), r(1.8), 160));
    render_curve(
        "ring",
        &curve::ring(r(1.4), r(0.35), 96, &GeometryContext::APPROXIMATE_512)
            .expect("ring Boolean")
            .into_value(),
    );
    render_curve("pie_slice", &curve::pie_slice(r(1.1), r(-35.0), r(115.0), 64));
    render_curve(
        "crescent",
        &curve::crescent(r(1.1), r(0.9), r(0.48), 96, &GeometryContext::APPROXIMATE_512)
            .expect("crescent Boolean")
            .into_value(),
    );
    render_curve(
        "supershape",
        &curve::supershape(r(1.0), r(1.0), r(5.0), r(2.0), r(7.0), r(7.0), 160),
    );
    render_curve(
        "circle_with_keyway",
        &curve::circle_with_keyway(
            r(1.0),
            96,
            r(0.45),
            r(0.35),
            &GeometryContext::APPROXIMATE_512,
        )
        .expect("keyway Boolean")
        .into_value(),
    );
    render_curve(
        "circle_with_flat",
        &curve::circle_with_flat(r(1.0), 96, r(0.72), &GeometryContext::APPROXIMATE_512)
            .expect("single-flat Boolean")
            .into_value(),
    );
    render_curve(
        "circle_with_two_flats",
        &curve::circle_with_two_flats(r(1.0), 96, r(0.72), &GeometryContext::APPROXIMATE_512)
            .expect("double-flat Boolean")
            .into_value(),
    );
    render_curve(
        "involute_gear",
        &curve::involute_gear(r(0.22), 14, r(20.0), r(0.02), r(0.01), 5),
    );
    render_curve(
        "cycloidal_gear",
        &curve::cycloidal_gear(r(0.22), 14, r(0.275), r(0.02), 8),
    );
    render_curve(
        "involute_rack",
        &curve::involute_rack(r(0.32), 7, r(20.0), r(0.02), r(0.01)),
    );
    render_curve(
        "cycloidal_rack",
        &curve::cycloidal_rack(r(0.32), 7, r(0.02), 12),
    );
    render_curve(
        "airfoil_naca4",
        &curve::airfoil_naca4(r(2.0), r(4.0), r(12.0), r(3.0), 96),
    );
    render_curve(
        "bezier",
        &curve::bezier_region(
            &[
                [r(0.0), r(0.0)],
                [r(1.4), r(-0.2)],
                [r(1.7), r(1.2)],
                [r(0.1), r(1.5)],
                [r(0.0), r(0.0)],
            ],
            96,
        ),
    );
    let curve_balls = [(Point2::new(r(0.0), r(0.0)), r(1.0))];
    render_curve(
        "metaballs_2d",
        &curve::metaballs(&curve_balls, (24, 24), r(1.0), r(0.0)),
    );
    render_curve("from_image", &readme_raster_shape());
    render_curve(
        "truetype",
        &curve::truetype_text("CSG", include_bytes!("../asar.ttf"), r(72.0)),
    );
    render_open_curves(
        "bezier_path",
        &[curve::bezier_path(
            &[
                [r(-1.1), r(-0.5)],
                [r(-0.4), r(1.2)],
                [r(0.5), r(-1.1)],
                [r(1.1), r(0.6)],
            ],
            96,
        )
        .expect("Bezier path")],
        &[],
    );
    render_open_curves(
        "bspline_path",
        &[curve::bspline_path(
            &[
                [r(-1.2), r(-0.7)],
                [r(-0.7), r(0.9)],
                [r(0.0), r(1.1)],
                [r(0.7), r(-0.8)],
                [r(1.2), r(0.4)],
            ],
            3,
            32,
        )
        .expect("B-spline path")],
        &[],
    );
    render_open_curves(
        "hilbert_strings",
        &[],
        &curve::hilbert_strings(&curve::square(r(2.0)), 4, r(0.12)),
    );
    render_open_curves(
        "hershey_strings",
        &[],
        &curve::hershey_strings("CSG", &hypercurve::hershey::fonts::ROWMANS, r(1.0)),
    );
}

fn readme_raster_shape() -> CurveRegion2 {
    let mut image = GrayImage::from_pixel(16, 16, Luma([0]));
    for y in 3..13 {
        for x in 3..13 {
            image.put_pixel(x, y, Luma([255]));
        }
    }
    curve::from_image(&image, 128).expect("trace README raster shape")
}

fn render_readme_meshes() {
    render_mesh("cube", &solid::cube(r(2.0)));
    render_mesh("cuboid", &solid::cuboid(r(1.4), r(2.3), r(0.95)));
    render_mesh("sphere", &solid::sphere(r(1.0), 32, 16));
    render_mesh("cylinder", &solid::cylinder(r(1.0), r(2.0), 32));
    render_mesh(
        "ellipsoid",
        &solid::ellipsoid(r(1.25), r(0.8), r(1.65), 32, 16),
    );
    render_mesh("frustum", &solid::frustum(r(0.65), r(1.05), r(2.0), 32));
    render_mesh(
        "frustum_between",
        &solid::frustum_between(p3(-0.8, -0.5, -0.7), p3(0.8, 0.5, 1.2), r(0.65), r(0.3), 32),
    );
    render_mesh("octahedron", &solid::octahedron(r(1.2)));
    render_mesh("icosahedron", &solid::icosahedron(r(1.2)));
    render_mesh("torus", &solid::torus(r(1.25), r(0.35), 36, 14));
    render_mesh(
        "mesh_arrow",
        &solid::arrow(Point3::origin(), v3(0.8, 0.4, 2.0), 32, false),
    );
    render_mesh("polyhedron", &readme_polyhedron());
    render_mesh(
        "teardrop_cylinder",
        &solid::teardrop_cylinder(
            r(1.5),
            r(2.3),
            r(0.8),
            48,
            &GeometryContext::APPROXIMATE_512,
        )
        .expect("teardrop cylinder")
        .into_value(),
    );
    render_mesh(
        "spur_gear_involute",
        &solid::spur_gear_involute(
            r(0.22),
            14,
            r(20.0),
            r(0.02),
            r(0.01),
            5,
            r(0.5),
            &GeometryContext::APPROXIMATE_512,
        )
        .expect("involute spur gear")
        .into_value(),
    );
    render_mesh(
        "spur_gear_cycloid",
        &solid::spur_gear_cycloid(
            r(0.22),
            14,
            r(0.275),
            r(0.02),
            8,
            r(0.5),
            &GeometryContext::APPROXIMATE_512,
        )
        .expect("cycloidal spur gear")
        .into_value(),
    );
    render_mesh(
        "helical_involute_gear",
        &solid::helical_involute_gear(
            r(0.22),
            14,
            r(20.0),
            r(0.02),
            r(0.01),
            5,
            r(0.8),
            r(24.0),
            8,
            &GeometryContext::APPROXIMATE_512,
        )
        .expect("helical involute gear")
        .into_value(),
    );

    let star = curve::star(5, r(1.0), r(0.45));
    render_mesh(
        "extrude",
        &curve::try_extrude(&star, r(0.65), &csgrs::GeometryContext::STRICT)
            .expect("extrude")
            .into_value(),
    );
    render_mesh(
        "extrude_vector",
        &curve::try_extrude_vector(
            &star,
            v3(0.45, 0.25, 0.9),
            &csgrs::GeometryContext::STRICT,
        )
        .expect("vector extrude")
        .into_value(),
    );
    render_mesh(
        "extrude_twisted",
        &curve::extrude_twisted(
            &star,
            r(1.4),
            r(95.0),
            [r(0.65), r(0.65)],
            10,
            &GeometryContext::APPROXIMATE_512,
        )
        .expect("twisted extrude")
        .into_value(),
    );
    let revolve_profile = curve::translated(&curve::circle(r(0.18), 32), r(1.0), r(0.0));
    render_mesh(
        "revolve",
        &curve::revolve(
            &revolve_profile,
            r(265.0),
            32,
            &csgrs::GeometryContext::STRICT,
        )
        .expect("revolve")
        .into_value(),
    );
    render_mesh(
        "sweep",
        &curve::try_sweep(
            &curve::square(r(0.4)),
            &[p3(0.0, 0.0, -0.8), p3(0.0, 0.0, 1.1)],
            &GeometryContext::STRICT,
        )
        .expect("sweep")
        .into_value(),
    );
    render_mesh("loft", &solid::loft(&readme_loft_sections()).expect("loft"));

    render_mesh("inverse", &solid::inverse(&solid::sphere(r(1.0), 32, 16)));
    render_mesh("csg", &cube_minus_translated_sphere());
    render_mesh(
        "convex_hull",
        &solid::convex_hull(&solid::cube(r(1.2))).expect("convex hull"),
    );
    render_mesh(
        "minkowski_sum",
        &solid::minkowski_sum(&solid::cube(r(1.1)), &solid::octahedron(r(0.45)))
            .expect("Minkowski sum"),
    );
    let projection_source = solid::frustum(r(0.65), r(1.05), r(2.0), 12);
    render_curve("flatten", &solid::flatten(&projection_source));
    let (slice, _, _) = solid::slice_z(&projection_source, r(1.0));
    render_curve("slice_z", &slice);

    render_implicit_meshes();
}

fn render_implicit_meshes() {
    render_metaballs_mesh();
    render_sdf_mesh();
    render_tpms_meshes();
}

fn render_metaballs_mesh() {
    let balls = [
        MetaBall::new(p3(-0.55, 0.0, 0.0), r(0.65)),
        MetaBall::new(p3(0.55, 0.0, 0.0), r(0.65)),
        MetaBall::new(p3(0.0, 0.55, 0.25), r(0.58)),
    ];
    render_mesh(
        "metaballs_3d",
        &solid::metaballs(&balls, (8, 8, 8), r(0.7), r(0.25)),
    );
}

fn render_sdf_mesh() {
    render_mesh(
        "sdf",
        &solid::sdf(
            |point| {
                point.x.clone() * point.x.clone()
                    + point.y.clone() * point.y.clone()
                    + point.z.clone() * point.z.clone()
                    - r(0.72)
            },
            (16, 16, 16),
            p3(-1.0, -1.0, -1.0),
            p3(1.0, 1.0, 1.0),
            r(0.0),
        ),
    );
}

fn render_tpms_meshes() {
    // Keep implicit thumbnail meshes legible once their vertices are visible.
    let tpms_box = solid::cube(r(2.0));
    render_mesh(
        "gyroid",
        &solid::gyroid_solid(&tpms_box, 24, r(2.0), r(0.0), r(0.18)),
    );
    render_mesh(
        "schwarz_p",
        &solid::schwarz_p_solid(&tpms_box, 24, r(2.0), r(0.0), r(0.18)),
    );
    render_mesh(
        "schwarz_d",
        &solid::schwarz_d_solid(&tpms_box, 24, r(2.0), r(0.0), r(0.18)),
    );
    render_mesh(
        "gyroid_surface",
        &solid::gyroid(&tpms_box, 24, r(2.0), r(0.2)),
    );
    render_mesh(
        "schwarz_p_surface",
        &solid::schwarz_p(&tpms_box, 24, r(2.0), r(0.2)),
    );
    render_mesh(
        "schwarz_d_surface",
        &solid::schwarz_d(&tpms_box, 24, r(2.0), r(0.2)),
    );
}

fn readme_polyhedron() -> TriangleMesh {
    let points = [
        [r(-1.0), r(-1.0), r(0.0)],
        [r(1.0), r(-1.0), r(0.0)],
        [r(1.0), r(1.0), r(0.0)],
        [r(-1.0), r(1.0), r(0.0)],
        [r(0.0), r(0.0), r(1.7)],
    ];
    let faces: [&[usize]; 5] = [
        &[3, 2, 1, 0],
        &[0, 1, 4],
        &[1, 2, 4],
        &[2, 3, 4],
        &[3, 0, 4],
    ];
    solid::polyhedron(&points, &faces).expect("README polyhedron")
}

fn readme_loft_sections() -> Vec<Vec<Point3>> {
    vec![
        vec![
            p3(-0.8, -0.8, -0.8),
            p3(0.8, -0.8, -0.8),
            p3(0.8, 0.8, -0.8),
            p3(-0.8, 0.8, -0.8),
        ],
        vec![
            p3(-1.1, -0.45, 0.0),
            p3(1.1, -0.45, 0.0),
            p3(1.1, 0.45, 0.0),
            p3(-1.1, 0.45, 0.0),
        ],
        vec![
            p3(-0.45, -0.65, 1.0),
            p3(0.45, -0.65, 1.0),
            p3(0.45, 0.65, 1.0),
            p3(-0.45, 0.65, 1.0),
        ],
    ]
}

fn cube_minus_translated_sphere() -> TriangleMesh {
    let cube = solid::cube(r(2.0));
    let sphere = solid::sphere(r(1.25), 16, 8).translated(r(1.0), r(1.0), r(1.0));
    let context = hypermesh::MeshContext::new(hypermesh::PredicatePolicy::APPROXIMATE_512);
    hypermesh::boolean_triangle_meshes(
        &context,
        &cube,
        &sphere,
        hypermesh::BooleanOp::Difference,
        hypermesh::EmberConfig::default(),
    )
    .expect("translated-sphere difference")
    .into_value()
}

fn render_curve(name: &str, region: &CurveRegion2) {
    let mut image = RgbaImage::from_pixel(SIZE, SIZE, BG);
    let projection = FiniteProjectionOptions::try_new(CURVE_CHORD_ERROR)
        .expect("README curve chord error is positive");
    // Material/hole ownership and edge identity are decided on CurveRegion2
    // before the finite raster projection is created.
    let profiles = expect_decided(
        region
            .project_to_finite_profiles_exact(&projection, &CurvePolicy::STRICT)
            .expect("project exact CurveRegion2 profiles"),
        "exact CurveRegion2 profile topology",
    );
    let edge_paths = expect_decided(
        region
            .project_to_finite_curve_paths(&CurvePolicy::STRICT)
            .expect("project exact CurveRegion2 edge paths"),
        "exact CurveRegion2 edge topology",
    );
    let Some(bounds) = curve_bounds(&profiles) else {
        save_image(name, &image);
        return;
    };
    let map = Map2::new(bounds);

    for profile in &profiles {
        for triangle in profile
            .triangulate(&CurvePolicy::STRICT)
            .expect("triangulate exact CurveRegion2 profile")
            .into_value()
        {
            let [a, b, c] = triangle.map(|point| map.point((point[0], point[1])));
            fill_triangle_2d(&mut image, a, b, c, FACE_2D);
        }
    }

    let edge_polylines = project_edge_paths(&edge_paths, &projection);
    for points in &edge_polylines {
        stroke_points(&mut image, &map, points, EDGE_WIDTH_2D, EDGE);
    }

    let vertex_count = edge_paths.iter().map(|path| path.curves().len()).sum();
    let radius = vertex_radius(vertex_count);
    for path in &edge_paths {
        // Mark exact curve-fragment junctions, not chordization samples.
        for edge in path.curves() {
            let vertex = (real_to_f64(edge.start().x()), real_to_f64(edge.start().y()));
            draw_vertex_2d(&mut image, map.point(vertex), radius);
        }
    }
    save_image(name, &image);
}

fn render_open_curves(name: &str, paths: &[CurvePath2], strings: &[CurveString2]) {
    let mut image = RgbaImage::from_pixel(SIZE, SIZE, BG);
    let projection = FiniteProjectionOptions::try_new(CURVE_CHORD_ERROR)
        .expect("README curve chord error is positive");
    let mut polylines = paths
        .iter()
        .map(|path| {
            path.project_to_finite_polyline(&projection)
                .expect("project exact open CurvePath2")
                .points()
                .to_vec()
        })
        .collect::<Vec<_>>();
    polylines.extend(strings.iter().map(|string| {
        string
            .project_to_finite_polyline(&projection)
            .expect("project exact open CurveString2")
            .points()
            .to_vec()
    }));
    let mut bounds = None;
    for point in polylines.iter().flat_map(|polyline| polyline.iter()) {
        include_point(&mut bounds, (point[0], point[1]));
    }
    let Some(bounds) = bounds else {
        save_image(name, &image);
        return;
    };
    let map = Map2::new(bounds);
    for points in &polylines {
        stroke_points(&mut image, &map, points, EDGE_WIDTH_2D, EDGE);
    }

    let mut vertices = Vec::new();
    for path in paths {
        vertices.extend(path.curves().iter().map(|edge| edge.start()));
        vertices.push(path.end());
    }
    for string in strings {
        vertices.extend(string.segments().iter().map(|segment| segment.start()));
        if let Some(last) = string.segments().last() {
            vertices.push(last.end());
        }
    }
    let radius = vertex_radius(vertices.len());
    for vertex in vertices {
        draw_vertex_2d(
            &mut image,
            map.point((real_to_f64(vertex.x()), real_to_f64(vertex.y()))),
            radius,
        );
    }
    save_image(name, &image);
}

fn render_mesh(name: &str, mesh: &TriangleMesh) {
    let mut image = RgbaImage::from_pixel(SIZE, SIZE, BG);
    if mesh.triangles.is_empty() {
        save_image(name, &image);
        return;
    }

    let world = mesh.positions.iter().map(point3_to_f64).collect::<Vec<_>>();
    let projected = world.iter().copied().map(project_point).collect::<Vec<_>>();
    let mut edges = BTreeSet::new();
    let mut used_vertices = BTreeSet::new();
    for triangle in mesh.triangles.iter() {
        let [a, b, c] = triangle.indices();
        used_vertices.extend([a, b, c]);
        for (from, to) in [(a, b), (b, c), (c, a)] {
            edges.insert(if from < to { (from, to) } else { (to, from) });
        }
    }

    let mut bounds = None;
    for &index in &used_vertices {
        include_point(&mut bounds, projected[index].plane);
    }
    let Some(bounds) = bounds else {
        save_image(name, &image);
        return;
    };
    let map = Map2::new(bounds);
    let raster = projected
        .iter()
        .map(|point| {
            let (x, y) = map.point(point.plane);
            RasterPoint3 {
                x,
                y,
                depth: point.depth,
            }
        })
        .collect::<Vec<_>>();

    let (min_depth, max_depth) = used_vertices
        .iter()
        .map(|&index| projected[index].depth)
        .fold((f64::INFINITY, f64::NEG_INFINITY), |(min, max), depth| {
            (min.min(depth), max.max(depth))
        });
    let depth_bias = ((max_depth - min_depth).abs() * 0.006).max(1.0e-9);
    let mut depth_buffer = vec![f64::NEG_INFINITY; (SIZE * SIZE) as usize];

    // Faces establish visibility; topology overlays are then depth-tested so
    // rear edges and vertices cannot bleed through the solid.
    for triangle in mesh.triangles.iter() {
        let indices = triangle.indices();
        let points = indices.map(|index| raster[index]);
        let color = shaded_face_color(indices.map(|index| world[index]));
        fill_triangle_3d(&mut image, &mut depth_buffer, points, color);
    }

    let edge_width = edge_width(edges.len());
    for &(from, to) in &edges {
        draw_depth_line(
            &mut image,
            &depth_buffer,
            raster[from],
            raster[to],
            edge_width,
            depth_bias,
            EDGE,
        );
    }

    let radius = vertex_radius(used_vertices.len());
    for index in used_vertices {
        draw_vertex_3d(&mut image, &depth_buffer, raster[index], radius, depth_bias);
    }
    save_image(name, &image);
}

fn expect_decided<T>(classification: Classification<T>, context: &str) -> T {
    match classification {
        Classification::Decided(value) => value,
        Classification::Uncertain(reason) => {
            panic!("{context} was uncertain: {reason:?}")
        },
    }
}

fn project_edge_paths(
    paths: &[CurvePath2],
    projection: &FiniteProjectionOptions,
) -> Vec<Vec<[f64; 2]>> {
    paths
        .iter()
        .map(|path| {
            path.project_to_finite_polyline(projection)
                .expect("rasterize exact CurveRegion2 edge path")
                .points()
                .to_vec()
        })
        .collect()
}

fn stroke_points(
    image: &mut RgbaImage,
    map: &Map2,
    points: &[[f64; 2]],
    width: f64,
    color: Rgba<u8>,
) {
    let pixels = points
        .iter()
        .map(|point| map.point((point[0], point[1])))
        .collect::<Vec<_>>();
    for pair in pixels.windows(2) {
        draw_line(image, pair[0], pair[1], width, color);
    }
}

fn curve_bounds(profiles: &[FiniteRegionProfile2]) -> Option<(f64, f64, f64, f64)> {
    let mut bounds = None;
    for profile in profiles {
        include_points(&mut bounds, profile.material().points());
        for hole in profile.holes() {
            include_points(&mut bounds, hole.points());
        }
    }
    bounds
}

fn include_points(bounds: &mut Option<(f64, f64, f64, f64)>, points: &[[f64; 2]]) {
    for point in points {
        include_point(bounds, (point[0], point[1]));
    }
}

fn include_point(bounds: &mut Option<(f64, f64, f64, f64)>, point: (f64, f64)) {
    *bounds = Some(match *bounds {
        Some((min_x, min_y, max_x, max_y)) => (
            min_x.min(point.0),
            min_y.min(point.1),
            max_x.max(point.0),
            max_y.max(point.1),
        ),
        None => (point.0, point.1, point.0, point.1),
    });
}

#[derive(Clone, Copy)]
struct ProjectedPoint3 {
    plane: (f64, f64),
    depth: f64,
}

#[derive(Clone, Copy)]
struct RasterPoint3 {
    x: f64,
    y: f64,
    depth: f64,
}

impl RasterPoint3 {
    const fn plane(self) -> (f64, f64) {
        (self.x, self.y)
    }
}

fn point3_to_f64(point: &Point3) -> [f64; 3] {
    [
        real_to_f64(&point.x),
        real_to_f64(&point.y),
        real_to_f64(&point.z),
    ]
}

fn project_point([x, y, z]: [f64; 3]) -> ProjectedPoint3 {
    ProjectedPoint3 {
        plane: (x - y, (x + y) * 0.45 - z),
        depth: x + y + z * 0.9,
    }
}

fn shaded_face_color([a, b, c]: [[f64; 3]; 3]) -> Rgba<u8> {
    let ab = subtract3(b, a);
    let ac = subtract3(c, a);
    let Some(mut normal) = normalize3(cross3(ab, ac)) else {
        return FACE_SHADOW_3D;
    };
    let view = normalize3(VIEW_DIRECTION).expect("view direction is nonzero");
    if dot3(normal, view) < 0.0 {
        normal = normal.map(|component| -component);
    }
    let light = normalize3(LIGHT_DIRECTION).expect("light direction is nonzero");
    let diffuse = dot3(normal, light).max(0.0);
    let facing = dot3(normal, view).clamp(0.0, 1.0);
    let amount = (0.18 + 0.68 * diffuse + 0.14 * facing).clamp(0.0, 1.0);
    mix_color(FACE_SHADOW_3D, FACE_LIGHT_3D, amount)
}

fn subtract3(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
    [left[0] - right[0], left[1] - right[1], left[2] - right[2]]
}

fn cross3(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
    [
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    ]
}

fn dot3(left: [f64; 3], right: [f64; 3]) -> f64 {
    left[0] * right[0] + left[1] * right[1] + left[2] * right[2]
}

fn normalize3(vector: [f64; 3]) -> Option<[f64; 3]> {
    let length = dot3(vector, vector).sqrt();
    (length > f64::EPSILON).then(|| vector.map(|component| component / length))
}

fn mix_color(shadow: Rgba<u8>, light: Rgba<u8>, amount: f64) -> Rgba<u8> {
    Rgba(std::array::from_fn(|channel| {
        let value =
            f64::from(shadow[channel]) * (1.0 - amount) + f64::from(light[channel]) * amount;
        value.round().clamp(0.0, 255.0) as u8
    }))
}

fn fill_triangle_2d(
    image: &mut RgbaImage,
    a: (f64, f64),
    b: (f64, f64),
    c: (f64, f64),
    color: Rgba<u8>,
) {
    let Some((min_x, min_y, max_x, max_y)) = triangle_pixel_bounds(a, b, c) else {
        return;
    };
    let area = edge(a, b, c);
    if area.abs() <= f64::EPSILON {
        return;
    }

    for y in min_y..=max_y {
        for x in min_x..=max_x {
            let p = (x as f64 + 0.5, y as f64 + 0.5);
            let w0 = edge(b, c, p) / area;
            let w1 = edge(c, a, p) / area;
            let w2 = edge(a, b, p) / area;
            if w0 >= -1.0e-9 && w1 >= -1.0e-9 && w2 >= -1.0e-9 {
                image.put_pixel(x, y, color);
            }
        }
    }
}

fn fill_triangle_3d(
    image: &mut RgbaImage,
    depth_buffer: &mut [f64],
    [a, b, c]: [RasterPoint3; 3],
    color: Rgba<u8>,
) {
    let Some((min_x, min_y, max_x, max_y)) =
        triangle_pixel_bounds(a.plane(), b.plane(), c.plane())
    else {
        return;
    };
    let area = edge(a.plane(), b.plane(), c.plane());
    if area.abs() <= f64::EPSILON {
        return;
    }

    for y in min_y..=max_y {
        for x in min_x..=max_x {
            let point = (x as f64 + 0.5, y as f64 + 0.5);
            let w0 = edge(b.plane(), c.plane(), point) / area;
            let w1 = edge(c.plane(), a.plane(), point) / area;
            let w2 = edge(a.plane(), b.plane(), point) / area;
            if w0 < -1.0e-9 || w1 < -1.0e-9 || w2 < -1.0e-9 {
                continue;
            }

            let depth = w0 * a.depth + w1 * b.depth + w2 * c.depth;
            let index = pixel_index(x, y);
            if depth >= depth_buffer[index] {
                depth_buffer[index] = depth;
                image.put_pixel(x, y, color);
            }
        }
    }
}

fn triangle_pixel_bounds(
    a: (f64, f64),
    b: (f64, f64),
    c: (f64, f64),
) -> Option<(u32, u32, u32, u32)> {
    let min_x = a.0.min(b.0).min(c.0).floor().max(0.0) as u32;
    let min_y = a.1.min(b.1).min(c.1).floor().max(0.0) as u32;
    let max_x = a.0.max(b.0).max(c.0).ceil().min(f64::from(SIZE - 1)) as u32;
    let max_y = a.1.max(b.1).max(c.1).ceil().min(f64::from(SIZE - 1)) as u32;
    (min_x <= max_x && min_y <= max_y).then_some((min_x, min_y, max_x, max_y))
}

fn edge(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> f64 {
    (c.0 - a.0) * (b.1 - a.1) - (c.1 - a.1) * (b.0 - a.0)
}

fn draw_line(
    image: &mut RgbaImage,
    a: (f64, f64),
    b: (f64, f64),
    width: f64,
    color: Rgba<u8>,
) {
    let half_width = width * 0.5;
    let Some((min_x, min_y, max_x, max_y)) = line_pixel_bounds(a, b, half_width) else {
        return;
    };
    let delta = (b.0 - a.0, b.1 - a.1);
    let length_squared = delta.0 * delta.0 + delta.1 * delta.1;

    for y in min_y..=max_y {
        for x in min_x..=max_x {
            let point = (x as f64 + 0.5, y as f64 + 0.5);
            let t = closest_line_parameter(point, a, delta, length_squared);
            let closest = (a.0 + delta.0 * t, a.1 + delta.1 * t);
            let distance = (point.0 - closest.0).hypot(point.1 - closest.1);
            let coverage = (half_width + 0.5 - distance).clamp(0.0, 1.0);
            if coverage > 0.0 {
                blend_pixel(image, x, y, color, coverage);
            }
        }
    }
}

fn draw_depth_line(
    image: &mut RgbaImage,
    depth_buffer: &[f64],
    a: RasterPoint3,
    b: RasterPoint3,
    width: f64,
    depth_bias: f64,
    color: Rgba<u8>,
) {
    let half_width = width * 0.5;
    let Some((min_x, min_y, max_x, max_y)) =
        line_pixel_bounds(a.plane(), b.plane(), half_width)
    else {
        return;
    };
    let delta = (b.x - a.x, b.y - a.y);
    let length_squared = delta.0 * delta.0 + delta.1 * delta.1;

    for y in min_y..=max_y {
        for x in min_x..=max_x {
            let point = (x as f64 + 0.5, y as f64 + 0.5);
            let t = closest_line_parameter(point, a.plane(), delta, length_squared);
            let closest = (a.x + delta.0 * t, a.y + delta.1 * t);
            let distance = (point.0 - closest.0).hypot(point.1 - closest.1);
            let coverage = (half_width + 0.5 - distance).clamp(0.0, 1.0);
            let depth = a.depth + (b.depth - a.depth) * t;
            let index = pixel_index(x, y);
            if coverage > 0.0 && depth + depth_bias >= depth_buffer[index] {
                blend_pixel(image, x, y, color, coverage);
            }
        }
    }
}

fn line_pixel_bounds(
    a: (f64, f64),
    b: (f64, f64),
    half_width: f64,
) -> Option<(u32, u32, u32, u32)> {
    let padding = half_width + 1.0;
    let min_x = (a.0.min(b.0) - padding).floor().max(0.0) as u32;
    let min_y = (a.1.min(b.1) - padding).floor().max(0.0) as u32;
    let max_x = (a.0.max(b.0) + padding).ceil().min(f64::from(SIZE - 1)) as u32;
    let max_y = (a.1.max(b.1) + padding).ceil().min(f64::from(SIZE - 1)) as u32;
    (min_x <= max_x && min_y <= max_y).then_some((min_x, min_y, max_x, max_y))
}

fn closest_line_parameter(
    point: (f64, f64),
    start: (f64, f64),
    delta: (f64, f64),
    length_squared: f64,
) -> f64 {
    if length_squared <= f64::EPSILON {
        0.0
    } else {
        (((point.0 - start.0) * delta.0 + (point.1 - start.1) * delta.1) / length_squared)
            .clamp(0.0, 1.0)
    }
}

fn vertex_radius(vertex_count: usize) -> f64 {
    match vertex_count {
        0..=48 => 4.75,
        49..=192 => 3.75,
        193..=768 => 2.5,
        769..=2_048 => 1.35,
        _ => 0.75,
    }
}

fn edge_width(edge_count: usize) -> f64 {
    match edge_count {
        0..=100 => 2.5,
        101..=1_000 => 1.9,
        1_001..=5_000 => 1.35,
        _ => 0.85,
    }
}

fn draw_vertex_2d(image: &mut RgbaImage, center: (f64, f64), radius: f64) {
    draw_disc(image, center, vertex_outline_radius(radius), EDGE);
    draw_disc(image, center, radius, VERTEX);
}

fn draw_vertex_3d(
    image: &mut RgbaImage,
    depth_buffer: &[f64],
    center: RasterPoint3,
    radius: f64,
    depth_bias: f64,
) {
    draw_depth_disc(
        image,
        depth_buffer,
        center,
        vertex_outline_radius(radius),
        depth_bias,
        EDGE,
    );
    draw_depth_disc(image, depth_buffer, center, radius, depth_bias, VERTEX);
}

fn vertex_outline_radius(radius: f64) -> f64 {
    radius + (radius * 0.35 + 0.5).min(1.35)
}

fn draw_disc(image: &mut RgbaImage, center: (f64, f64), radius: f64, color: Rgba<u8>) {
    let Some((min_x, min_y, max_x, max_y)) = disc_pixel_bounds(center, radius) else {
        return;
    };
    for y in min_y..=max_y {
        for x in min_x..=max_x {
            let distance = (x as f64 + 0.5 - center.0).hypot(y as f64 + 0.5 - center.1);
            let coverage = (radius + 0.5 - distance).clamp(0.0, 1.0);
            if coverage > 0.0 {
                blend_pixel(image, x, y, color, coverage);
            }
        }
    }
}

fn draw_depth_disc(
    image: &mut RgbaImage,
    depth_buffer: &[f64],
    center: RasterPoint3,
    radius: f64,
    depth_bias: f64,
    color: Rgba<u8>,
) {
    let Some((min_x, min_y, max_x, max_y)) = disc_pixel_bounds(center.plane(), radius) else {
        return;
    };
    for y in min_y..=max_y {
        for x in min_x..=max_x {
            let distance = (x as f64 + 0.5 - center.x).hypot(y as f64 + 0.5 - center.y);
            let coverage = (radius + 0.5 - distance).clamp(0.0, 1.0);
            if coverage > 0.0 && center.depth + depth_bias >= depth_buffer[pixel_index(x, y)] {
                blend_pixel(image, x, y, color, coverage);
            }
        }
    }
}

fn disc_pixel_bounds(center: (f64, f64), radius: f64) -> Option<(u32, u32, u32, u32)> {
    let padding = radius + 1.0;
    let min_x = (center.0 - padding).floor().max(0.0) as u32;
    let min_y = (center.1 - padding).floor().max(0.0) as u32;
    let max_x = (center.0 + padding).ceil().min(f64::from(SIZE - 1)) as u32;
    let max_y = (center.1 + padding).ceil().min(f64::from(SIZE - 1)) as u32;
    (min_x <= max_x && min_y <= max_y).then_some((min_x, min_y, max_x, max_y))
}

fn pixel_index(x: u32, y: u32) -> usize {
    (y * SIZE + x) as usize
}

fn blend_pixel(image: &mut RgbaImage, x: u32, y: u32, color: Rgba<u8>, coverage: f64) {
    let destination = *image.get_pixel(x, y);
    let source_alpha = f64::from(color[3]) / 255.0 * coverage;
    let destination_alpha = f64::from(destination[3]) / 255.0;
    let output_alpha = source_alpha + destination_alpha * (1.0 - source_alpha);
    if output_alpha <= f64::EPSILON {
        return;
    }

    let mut output = [0_u8; 4];
    for channel in 0..3 {
        let source = f64::from(color[channel]) / 255.0;
        let destination = f64::from(destination[channel]) / 255.0;
        let value = (source * source_alpha
            + destination * destination_alpha * (1.0 - source_alpha))
            / output_alpha;
        output[channel] = (value * 255.0).round().clamp(0.0, 255.0) as u8;
    }
    output[3] = (output_alpha * 255.0).round().clamp(0.0, 255.0) as u8;
    image.put_pixel(x, y, Rgba(output));
}

struct Map2 {
    min_x: f64,
    min_y: f64,
    scale: f64,
    offset_x: f64,
    offset_y: f64,
}

impl Map2 {
    fn new((min_x, min_y, max_x, max_y): (f64, f64, f64, f64)) -> Self {
        let width = (max_x - min_x).max(1.0e-6);
        let height = (max_y - min_y).max(1.0e-6);
        let scale = (SIZE as f64 * (1.0 - 2.0 * PADDING)) / width.max(height);
        Self {
            min_x,
            min_y,
            scale,
            offset_x: (SIZE as f64 - width * scale) * 0.5,
            offset_y: (SIZE as f64 - height * scale) * 0.5,
        }
    }

    fn point(&self, point: (f64, f64)) -> (f64, f64) {
        let px = (point.0 - self.min_x) * self.scale + self.offset_x;
        let py = SIZE as f64 - 1.0 - ((point.1 - self.min_y) * self.scale + self.offset_y);
        (
            px.clamp(0.0, SIZE as f64 - 1.0),
            py.clamp(0.0, SIZE as f64 - 1.0),
        )
    }
}

fn save_image(name: &str, image: &RgbaImage) {
    let path = output_dir().join(name).with_extension("png");
    image.save(&path).expect("save README render");
    println!("wrote {}", path.display());
}

fn output_dir() -> PathBuf {
    std::env::var_os("README_RENDER_OUTPUT_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("docs"))
}

fn r(value: f64) -> Real {
    Real::try_from(value).expect("README render constants are finite")
}

fn real_to_f64(value: &Real) -> f64 {
    value
        .to_f64_lossy()
        .expect("README render coordinates are finitely projectable")
}

fn p3(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

fn v3(x: f64, y: f64, z: f64) -> Vector3 {
    Vector3::from_xyz(r(x), r(y), r(z))
}
