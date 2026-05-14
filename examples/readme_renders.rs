//! Generate the PNG renders referenced by the README.
//!
//! This is intentionally a small pure-Rust software renderer. It does not try
//! to be a production renderer; it just keeps README images reproducible without
//! requiring Blender, OpenSCAD, a browser, or system graphics libraries.
//!
//! Run:
//!
//! ```text
//! cargo run --example readme_renders
//! ```

use csgrs::csg::CSG;
use csgrs::float_types::Real;
use csgrs::mesh::Mesh;
use csgrs::mesh::metaballs::MetaBall;
use csgrs::polygon::Polygon;
use csgrs::sketch::Sketch;
use csgrs::vertex::Vertex;
use geo::{
    BoundingRect, Contains, CoordsIter, Geometry, GeometryCollection, Point as GeoPoint,
};
use image::{GrayImage, Luma, Rgba, RgbaImage};
use nalgebra::{Point2, Point3, Vector3};
use std::fs;
use std::path::Path;

const SIZE: u32 = 1024;
const PADDING: Real = 0.12;

const BG: Rgba<u8> = Rgba([0, 0, 0, 0]);
const INK_2D: Rgba<u8> = Rgba([42, 95, 143, 255]);
const EDGE_2D: Rgba<u8> = Rgba([15, 41, 70, 255]);
const LINE_2D: Rgba<u8> = Rgba([31, 80, 126, 255]);

fn main() {
    fs::create_dir_all("docs").expect("create docs directory");

    render_readme_sketches();
    render_readme_meshes();

    println!("README renders written to docs/*.png");
}

fn render_readme_sketches() {
    let font_data = include_bytes!("../asar.ttf");

    let square = Sketch::<()>::square(2.0, None);
    render_sketch("square", &square);
    render_sketch("rectangle", &Sketch::<()>::rectangle(2.4, 1.35, None));
    render_sketch("circle", &Sketch::<()>::circle(1.0, 96, None));
    render_sketch(
        "polygon",
        &Sketch::<()>::polygon(&[[0.0, 1.2], [-1.1, -0.8], [1.1, -0.8]], None),
    );
    render_sketch(
        "rounded_rectangle",
        &Sketch::<()>::rounded_rectangle(2.4, 1.5, 0.28, 12, None),
    );
    render_sketch("ellipse", &Sketch::<()>::ellipse(2.3, 1.3, 96, None));
    render_sketch("regular_ngon", &Sketch::<()>::regular_ngon(6, 1.0, None));
    render_sketch(
        "sketch_arrow",
        &Sketch::<()>::arrow(2.2, 0.35, 0.8, 1.0, None),
    );
    render_sketch(
        "right_triangle",
        &Sketch::<()>::right_triangle(2.0, 1.5, None),
    );
    render_sketch(
        "trapezoid",
        &Sketch::<()>::trapezoid(1.2, 2.2, 1.4, 0.45, None),
    );
    render_sketch("star", &Sketch::<()>::star(5, 1.1, 0.45, None));
    render_sketch("teardrop", &Sketch::<()>::teardrop(1.5, 2.2, 80, None));
    render_sketch("sketch_egg", &Sketch::<()>::egg(1.5, 2.2, 96, None));
    render_sketch("squircle", &Sketch::<()>::squircle(2.0, 2.0, 96, None));
    render_sketch("keyhole", &Sketch::<()>::keyhole(0.7, 0.55, 1.35, 64, None));
    render_sketch("reuleaux", &Sketch::<()>::reuleaux(3, 1.0, 24, None));
    render_sketch("ring", &Sketch::<()>::ring(1.4, 0.35, 96, None));
    render_sketch(
        "pie_slice",
        &Sketch::<()>::pie_slice(1.1, -35.0, 115.0, 64, None),
    );
    render_sketch(
        "supershape",
        &Sketch::<()>::supershape(1.0, 1.0, 6.0, 0.35, 0.8, 0.8, 240, None),
    );
    render_sketch(
        "circle_with_keyway",
        &Sketch::<()>::circle_with_keyway(1.0, 96, 0.42, 0.35, None),
    );
    render_sketch(
        "circle_with_flat",
        &Sketch::<()>::circle_with_flat(1.0, 96, 0.55, None),
    );
    render_sketch(
        "circle_with_two_flats",
        &Sketch::<()>::circle_with_two_flats(1.0, 96, 0.55, None),
    );
    render_sketch("text", &Sketch::<()>::text("HELLO", font_data, 28.0, None));
    render_sketch(
        "airfoil_naca4",
        &Sketch::<()>::airfoil_naca4(2.0, 4.0, 12.0, 2.4, 80, None),
    );
    render_sketch(
        "bspline",
        &Sketch::<()>::bspline(
            &[[-1.3, -0.7], [-0.6, 1.0], [0.5, 1.1], [1.3, -0.6]],
            3,
            32,
            None,
        ),
    );
    render_sketch("heart", &Sketch::<()>::heart(2.0, 1.8, 160, None));
    render_sketch("crescent", &Sketch::<()>::crescent(1.1, 0.85, 0.45, 96, None));
    render_sketch(
        "involute_gear",
        &Sketch::<()>::involute_gear(0.2, 18, 20.0, 0.0, 0.0, 8, None),
    );
    render_sketch(
        "hilbert_curve",
        &Sketch::<()>::square(2.0, None).hilbert_curve(5, 0.08),
    );

    let mut image = GrayImage::new(16, 16);
    for y in 3..13 {
        for x in 3..13 {
            let dx = x as i32 - 8;
            let dy = y as i32 - 8;
            if dx * dx + dy * dy < 30 {
                image.put_pixel(x, y, Luma([255]));
            }
        }
    }
    render_sketch(
        "from_image",
        &Sketch::<()>::from_image(&image, 128, true, None),
    );

    let metaballs = [
        (Point2::new(-0.45, 0.0), 0.72),
        (Point2::new(0.45, 0.0), 0.72),
        (Point2::new(0.0, 0.55), 0.58),
    ];
    render_sketch(
        "metaballs_2d",
        &Sketch::<()>::metaballs(&metaballs, (48, 48), 0.7, 0.25, None),
    );

    let bezier = Sketch::<()>::bezier(
        &[[-1.2, -0.75], [-0.55, 1.05], [0.55, -1.0], [1.2, 0.75]],
        96,
        None,
    );
    render_sketch("bezier", &bezier);
}

fn render_readme_meshes() {
    render_mesh("cube", &Mesh::<()>::cube(2.0, None));
    render_mesh("cuboid", &Mesh::<()>::cuboid(1.4, 2.3, 0.95, None));
    render_mesh("sphere", &Mesh::<()>::sphere(1.0, 32, 16, None));
    render_mesh("cylinder", &Mesh::<()>::cylinder(1.0, 2.0, 32, None));
    render_mesh("frustum", &Mesh::<()>::frustum(0.65, 1.05, 2.0, 32, None));
    render_mesh(
        "frustum_ptp",
        &Mesh::<()>::frustum_ptp(
            Point3::new(-0.8, -0.4, -0.4),
            Point3::new(0.8, 0.45, 1.1),
            0.35,
            0.75,
            32,
            None,
        ),
    );
    render_mesh("octahedron", &Mesh::<()>::octahedron(1.2, None));
    render_mesh("icosahedron", &Mesh::<()>::icosahedron(1.2, None));
    render_mesh("torus", &Mesh::<()>::torus(1.25, 0.35, 36, 14, None));
    render_mesh("mesh_egg", &Mesh::<()>::egg(1.3, 2.0, 32, 24, None));
    render_mesh("mesh_teardrop", &Mesh::<()>::teardrop(1.4, 2.0, 32, 24, None));
    render_mesh(
        "teardrop_cylinder",
        &Mesh::<()>::teardrop_cylinder(1.2, 2.1, 1.4, 32, None),
    );
    render_mesh(
        "ellipsoid",
        &Mesh::<()>::ellipsoid(1.4, 0.85, 1.8, 32, 16, None),
    );
    render_mesh(
        "mesh_arrow",
        &Mesh::<()>::arrow(Point3::origin(), Vector3::new(0.8, 0.4, 2.0), 32, false, None),
    );
    render_mesh(
        "polyhedron",
        &Mesh::<()>::polyhedron(
            &[
                [1.0, 1.0, 1.0],
                [-1.0, -1.0, 1.0],
                [-1.0, 1.0, -1.0],
                [1.0, -1.0, -1.0],
            ],
            &[&[0, 1, 2], &[0, 3, 1], &[0, 2, 3], &[1, 3, 2]],
            None,
        )
        .expect("polyhedron"),
    );

    let star = Sketch::<()>::star(5, 1.0, 0.45, None);
    render_mesh("extrude", &star.extrude(0.65));
    render_mesh(
        "extrude_vector",
        &star.extrude_vector(Vector3::new(0.45, 0.25, 0.9)),
    );
    render_mesh(
        "revolve",
        &Sketch::<()>::circle(0.18, 32, None)
            .translate(1.0, 0.0, 0.0)
            .revolve(265.0, 32)
            .expect("revolve"),
    );
    let bottom = Polygon::new(
        vec![
            Vertex::new(Point3::new(-0.8, -0.8, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.8, -0.8, 0.0), Vector3::z()),
            Vertex::new(Point3::new(0.8, 0.8, 0.0), Vector3::z()),
            Vertex::new(Point3::new(-0.8, 0.8, 0.0), Vector3::z()),
        ],
        None,
    );
    let top = Polygon::new(
        vec![
            Vertex::new(Point3::new(-0.45, -0.45, 1.2), Vector3::z()),
            Vertex::new(Point3::new(0.45, -0.45, 1.2), Vector3::z()),
            Vertex::new(Point3::new(0.45, 0.45, 1.2), Vector3::z()),
            Vertex::new(Point3::new(-0.45, 0.45, 1.2), Vector3::z()),
        ],
        None,
    );
    render_mesh(
        "loft",
        &Sketch::<()>::loft(&bottom, &top, true).expect("loft"),
    );
    render_mesh(
        "sweep",
        &Sketch::<()>::circle(0.18, 24, None).sweep(&[
            Point3::new(-1.2, -0.6, 0.0),
            Point3::new(-0.45, 0.4, 0.55),
            Point3::new(0.4, -0.25, 1.0),
            Point3::new(1.15, 0.45, 1.25),
        ]),
    );

    let cube = Mesh::<()>::cube(2.0, None);
    render_mesh("inverse", &Mesh::<()>::sphere(1.0, 32, 16, None).inverse());
    render_mesh(
        "cube_sphere_difference",
        &cube.difference(&Mesh::<()>::sphere(1.25, 32, 16, None)),
    );
    render_mesh(
        "csg",
        &Mesh::<()>::cube(2.0, None).union(&Mesh::<()>::sphere(1.15, 32, 16, None)),
    );
    render_mesh("convex_hull", &Mesh::<()>::cube(1.2, None).convex_hull());
    render_mesh(
        "minkowski_sum",
        &Mesh::<()>::cube(1.1, None).minkowski_sum(&Mesh::<()>::sphere(0.45, 16, 8, None)),
    );
    render_mesh(
        "subdivide_triangles",
        &Mesh::<()>::cube(2.0, None).subdivide_triangles(1.try_into().unwrap()),
    );

    let balls = [
        MetaBall::new(Point3::new(-0.55, 0.0, 0.0), 0.65),
        MetaBall::new(Point3::new(0.55, 0.0, 0.0), 0.65),
        MetaBall::new(Point3::new(0.0, 0.55, 0.25), 0.58),
    ];
    render_mesh(
        "metaballs_3d",
        &Mesh::<()>::metaballs(&balls, (16, 16, 16), 0.7, 0.25, None),
    );

    render_mesh(
        "sdf",
        &Mesh::<()>::sdf(
            |p| p.coords.norm() - 1.0,
            (16, 16, 16),
            Point3::new(-1.2, -1.2, -1.2),
            Point3::new(1.2, 1.2, 1.2),
            0.0,
            None,
        ),
    );

    let tpms_box = Mesh::<()>::cube(2.0, None);
    render_mesh("gyroid", &tpms_box.gyroid(14, 0.75, 0.0, None));
    render_mesh("schwarz_p", &tpms_box.schwarz_p(14, 0.75, 0.0, None));
    render_mesh("schwarz_d", &tpms_box.schwarz_d(14, 0.75, 0.0, None));
    render_mesh(
        "spur_gear_involute",
        &Mesh::<()>::spur_gear_involute(0.18, 22, 20.0, 0.0, 0.0, 8, 0.35, None),
    );

    render_distribution_examples();
}

fn render_distribution_examples() {
    let base = Mesh::<()>::sphere(0.13, 16, 8, None);
    let line = (0..7)
        .map(|i| base.translate(i as Real * 0.38 - 1.14, 0.0, 0.0))
        .reduce(|a, b| a.union(&b))
        .unwrap();
    render_mesh("distribute_linear", &line);

    let grid = (0..3)
        .flat_map(|row| (0..3).map(move |col| (row, col)))
        .map(|(row, col)| {
            base.translate(col as Real * 0.45 - 0.45, row as Real * 0.45 - 0.45, 0.0)
        })
        .reduce(|a, b| a.union(&b))
        .unwrap();
    render_mesh("distribute_grid", &grid);

    let arc = (0..9)
        .map(|i| {
            let a = -120.0_f64.to_radians() as Real
                + i as Real * (240.0_f64.to_radians() as Real / 8.0);
            base.translate(a.cos(), a.sin(), 0.0)
        })
        .reduce(|a, b| a.union(&b))
        .unwrap();
    render_mesh("distribute_arc", &arc);
}

fn render_sketch(name: &str, sketch: &Sketch<()>) {
    let mut image = RgbaImage::from_pixel(SIZE, SIZE, BG);
    let Some(bounds) = sketch.geometry.bounding_rect() else {
        save_image(name, &image);
        return;
    };
    let map = Map2::new(bounds.min().x, bounds.min().y, bounds.max().x, bounds.max().y);

    fill_geometry(&mut image, &map, &sketch.geometry);
    stroke_geometry(&mut image, &map, &sketch.geometry);
    save_image(name, &image);
}

fn fill_geometry(image: &mut RgbaImage, map: &Map2, geometry: &GeometryCollection<Real>) {
    for geometry in geometry {
        match geometry {
            Geometry::Polygon(poly) => fill_polygon(image, map, poly),
            Geometry::MultiPolygon(mp) => {
                for poly in &mp.0 {
                    fill_polygon(image, map, poly);
                }
            },
            Geometry::GeometryCollection(gc) => fill_geometry(image, map, gc),
            _ => {},
        }
    }
}

fn stroke_geometry(image: &mut RgbaImage, map: &Map2, geometry: &GeometryCollection<Real>) {
    for geometry in geometry {
        match geometry {
            Geometry::Polygon(poly) => {
                stroke_line_string(image, map, poly.exterior(), EDGE_2D);
                for ring in poly.interiors() {
                    stroke_line_string(image, map, ring, EDGE_2D);
                }
            },
            Geometry::MultiPolygon(mp) => {
                for poly in &mp.0 {
                    stroke_line_string(image, map, poly.exterior(), EDGE_2D);
                    for ring in poly.interiors() {
                        stroke_line_string(image, map, ring, EDGE_2D);
                    }
                }
            },
            Geometry::Line(line) => {
                draw_line(
                    image,
                    map.point(line.start.x, line.start.y),
                    map.point(line.end.x, line.end.y),
                    LINE_2D,
                );
            },
            Geometry::LineString(line) => stroke_line_string(image, map, line, LINE_2D),
            Geometry::MultiLineString(lines) => {
                for line in lines {
                    stroke_line_string(image, map, line, LINE_2D);
                }
            },
            Geometry::GeometryCollection(gc) => stroke_geometry(image, map, gc),
            _ => {},
        }
    }
}

fn fill_polygon(image: &mut RgbaImage, map: &Map2, poly: &geo::Polygon<Real>) {
    let Some(bounds) = poly.bounding_rect() else {
        return;
    };
    let (min_x, max_y) = map.point(bounds.min().x, bounds.min().y);
    let (max_x, min_y) = map.point(bounds.max().x, bounds.max().y);
    let x0 = min_x.min(max_x).saturating_sub(1).min(SIZE - 1);
    let x1 = min_x.max(max_x).saturating_add(1).min(SIZE - 1);
    let y0 = min_y.min(max_y).saturating_sub(1).min(SIZE - 1);
    let y1 = min_y.max(max_y).saturating_add(1).min(SIZE - 1);

    for y in y0..=y1 {
        for x in x0..=x1 {
            let (wx, wy) = map.world(x, y);
            if poly.contains(&GeoPoint::new(wx, wy)) {
                image.put_pixel(x, y, INK_2D);
            }
        }
    }
}

fn stroke_line_string(
    image: &mut RgbaImage,
    map: &Map2,
    line: &geo::LineString<Real>,
    color: Rgba<u8>,
) {
    let points = line
        .coords_iter()
        .map(|c| map.point(c.x, c.y))
        .collect::<Vec<_>>();
    for pair in points.windows(2) {
        draw_line(image, pair[0], pair[1], color);
    }
}

fn render_mesh(name: &str, mesh: &Mesh<()>) {
    let mut image = RgbaImage::from_pixel(SIZE, SIZE, BG);
    let triangles = mesh.triangulate().polygons;
    if triangles.is_empty() {
        save_image(name, &image);
        return;
    }

    let projected = triangles
        .iter()
        .filter_map(project_triangle)
        .collect::<Vec<_>>();
    if projected.is_empty() {
        save_image(name, &image);
        return;
    }

    let bounds = projected_bounds_from_mesh(mesh);
    let map = Map2::new(bounds.0, bounds.1, bounds.2, bounds.3);

    let mut ordered = projected;
    ordered.sort_by(|a, b| {
        a.depth
            .partial_cmp(&b.depth)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    for tri in ordered {
        fill_projected_triangle(&mut image, &map, &tri);
    }

    save_image(name, &image);
}

fn projected_bounds_from_mesh(mesh: &Mesh<()>) -> (Real, Real, Real, Real) {
    let aabb = mesh.bounding_box();
    aabb_corners(aabb.mins, aabb.maxs).iter().fold(
        (Real::MAX, Real::MAX, -Real::MAX, -Real::MAX),
        |(min_x, min_y, max_x, max_y), point| {
            let ((x, y), _) = project_point(*point);
            (min_x.min(x), min_y.min(y), max_x.max(x), max_y.max(y))
        },
    )
}

fn aabb_corners(mins: Point3<Real>, maxs: Point3<Real>) -> [Point3<Real>; 8] {
    [
        Point3::new(mins.x, mins.y, mins.z),
        Point3::new(maxs.x, mins.y, mins.z),
        Point3::new(mins.x, maxs.y, mins.z),
        Point3::new(maxs.x, maxs.y, mins.z),
        Point3::new(mins.x, mins.y, maxs.z),
        Point3::new(maxs.x, mins.y, maxs.z),
        Point3::new(mins.x, maxs.y, maxs.z),
        Point3::new(maxs.x, maxs.y, maxs.z),
    ]
}

fn camera_axes() -> (Vector3<Real>, Vector3<Real>, Vector3<Real>) {
    let screen_x = Vector3::new(1.0, -1.0, 0.0).normalize();
    let view_dir = Vector3::new(1.0, 1.0, 1.0).normalize();
    let screen_y = view_dir.cross(&screen_x).normalize();
    (screen_x, screen_y, view_dir)
}

fn project_triangle(poly: &Polygon<()>) -> Option<ProjectedTriangle> {
    if poly.vertices.len() < 3 {
        return None;
    }
    let a = poly.vertices[0].position;
    let b = poly.vertices[1].position;
    let c = poly.vertices[2].position;
    let normal = (b - a).cross(&(c - a));
    let normal = normal.try_normalize(1e-9 as Real)?;
    let light = Vector3::new(-0.35, -0.45, 0.82).normalize();
    let shade = normal.dot(&light).abs().mul_add(0.45, 0.35).clamp(0.18, 1.0);
    let color = shade_color(shade);

    let pa = project_point(a);
    let pb = project_point(b);
    let pc = project_point(c);
    Some(ProjectedTriangle {
        points: [pa.0, pb.0, pc.0],
        depth: (pa.1 + pb.1 + pc.1) / 3.0,
        color,
    })
}

fn project_point(p: Point3<Real>) -> ((Real, Real), Real) {
    let (screen_x, screen_y, view_dir) = camera_axes();
    (
        (p.coords.dot(&screen_x), p.coords.dot(&screen_y)),
        p.coords.dot(&view_dir),
    )
}

fn shade_color(shade: Real) -> Rgba<u8> {
    let r = (70.0 + shade * 70.0).clamp(0.0, 255.0) as u8;
    let g = (105.0 + shade * 85.0).clamp(0.0, 255.0) as u8;
    let b = (140.0 + shade * 95.0).clamp(0.0, 255.0) as u8;
    Rgba([r, g, b, 255])
}

fn fill_projected_triangle(image: &mut RgbaImage, map: &Map2, tri: &ProjectedTriangle) {
    let p = tri.points.map(|point| map.point(point.0, point.1));
    let min_x = p
        .iter()
        .map(|p| p.0)
        .min()
        .unwrap()
        .saturating_sub(1)
        .min(SIZE - 1);
    let max_x = p
        .iter()
        .map(|p| p.0)
        .max()
        .unwrap()
        .saturating_add(1)
        .min(SIZE - 1);
    let min_y = p
        .iter()
        .map(|p| p.1)
        .min()
        .unwrap()
        .saturating_sub(1)
        .min(SIZE - 1);
    let max_y = p
        .iter()
        .map(|p| p.1)
        .max()
        .unwrap()
        .saturating_add(1)
        .min(SIZE - 1);

    let a = (p[0].0 as Real, p[0].1 as Real);
    let b = (p[1].0 as Real, p[1].1 as Real);
    let c = (p[2].0 as Real, p[2].1 as Real);
    let area = edge_function(a, b, c);
    if area.abs() < 1e-9 as Real {
        return;
    }

    for y in min_y..=max_y {
        for x in min_x..=max_x {
            let q = (x as Real + 0.5, y as Real + 0.5);
            let w0 = edge_function(b, c, q);
            let w1 = edge_function(c, a, q);
            let w2 = edge_function(a, b, q);
            if (w0 >= 0.0 && w1 >= 0.0 && w2 >= 0.0) || (w0 <= 0.0 && w1 <= 0.0 && w2 <= 0.0) {
                image.put_pixel(x, y, tri.color);
            }
        }
    }

    draw_line(image, p[0], p[1], Rgba([31, 47, 64, 140]));
    draw_line(image, p[1], p[2], Rgba([31, 47, 64, 140]));
    draw_line(image, p[2], p[0], Rgba([31, 47, 64, 140]));
}

fn edge_function(a: (Real, Real), b: (Real, Real), c: (Real, Real)) -> Real {
    (c.0 - a.0) * (b.1 - a.1) - (c.1 - a.1) * (b.0 - a.0)
}

fn draw_line(image: &mut RgbaImage, a: (u32, u32), b: (u32, u32), color: Rgba<u8>) {
    let (mut x0, mut y0) = (a.0 as i32, a.1 as i32);
    let (x1, y1) = (b.0 as i32, b.1 as i32);
    let dx = (x1 - x0).abs();
    let dy = -(y1 - y0).abs();
    let sx = if x0 < x1 { 1 } else { -1 };
    let sy = if y0 < y1 { 1 } else { -1 };
    let mut err = dx + dy;

    loop {
        if x0 >= 0 && y0 >= 0 && x0 < SIZE as i32 && y0 < SIZE as i32 {
            image.put_pixel(x0 as u32, y0 as u32, color);
        }
        if x0 == x1 && y0 == y1 {
            break;
        }
        let e2 = 2 * err;
        if e2 >= dy {
            err += dy;
            x0 += sx;
        }
        if e2 <= dx {
            err += dx;
            y0 += sy;
        }
    }
}

fn save_image(name: &str, image: &RgbaImage) {
    let path = Path::new("docs").join(name).with_extension("png");
    image.save(&path).expect("save README render");
    println!("wrote {}", path.display());
}

#[derive(Clone, Copy)]
struct ProjectedTriangle {
    points: [(Real, Real); 3],
    depth: Real,
    color: Rgba<u8>,
}

#[derive(Clone, Copy)]
struct Map2 {
    min_x: Real,
    min_y: Real,
    scale: Real,
    offset_x: Real,
    offset_y: Real,
}

impl Map2 {
    fn new(min_x: Real, min_y: Real, max_x: Real, max_y: Real) -> Self {
        let width = (max_x - min_x).max(1e-6 as Real);
        let height = (max_y - min_y).max(1e-6 as Real);
        let scale = (SIZE as Real * (1.0 - 2.0 * PADDING)) / width.max(height);
        let drawn_w = width * scale;
        let drawn_h = height * scale;
        Self {
            min_x,
            min_y,
            scale,
            offset_x: (SIZE as Real - drawn_w) * 0.5,
            offset_y: (SIZE as Real - drawn_h) * 0.5,
        }
    }

    fn point(&self, x: Real, y: Real) -> (u32, u32) {
        let px = ((x - self.min_x) * self.scale + self.offset_x).round();
        let py =
            (SIZE as Real - 1.0 - ((y - self.min_y) * self.scale + self.offset_y)).round();
        (
            px.clamp(0.0, SIZE as Real - 1.0) as u32,
            py.clamp(0.0, SIZE as Real - 1.0) as u32,
        )
    }

    fn world(&self, x: u32, y: u32) -> (Real, Real) {
        let wx = (x as Real + 0.5 - self.offset_x) / self.scale + self.min_x;
        let wy =
            (SIZE as Real - 1.0 - y as Real + 0.5 - self.offset_y) / self.scale + self.min_y;
        (wx, wy)
    }
}
