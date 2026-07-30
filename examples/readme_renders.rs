//! Generate lightweight PNG renders for README assets.
//!
//! Geometry is constructed with `hyperlattice::Real`; primitive floats are used
//! only after projection into the image/raster boundary.

use csgrs::solid::MetaBall;
use csgrs::{
    GeometryContext, TriangleMesh, curve,
    solid::{self, SolidExt},
};
use hypercurve::CurveRegion2;
use hyperlattice::{Point3, Real, Vector3};
use image::{Rgba, RgbaImage};
use std::fs;
use std::path::PathBuf;

const SIZE: u32 = 768;
const PADDING: f64 = 0.12;
const BG: Rgba<u8> = Rgba([0, 0, 0, 0]);
const INK_2D: Rgba<u8> = Rgba([42, 95, 143, 255]);
const EDGE_2D: Rgba<u8> = Rgba([15, 41, 70, 255]);
const FILL_3D: Rgba<u8> = Rgba([104, 145, 178, 255]);
const EDGE_3D: Rgba<u8> = Rgba([31, 47, 64, 160]);

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
    render_curve("heart", &curve::heart(r(2.0), r(1.8), 160));
    render_curve(
        "ring",
        &curve::ring(r(1.4), r(0.35), 96, &GeometryContext::APPROXIMATE_512)
            .expect("ring Boolean")
            .into_value(),
    );
    render_curve("pie_slice", &curve::pie_slice(r(1.1), r(-35.0), r(115.0), 64));
}

fn render_readme_meshes() {
    render_mesh("cube", &solid::cube(r(2.0)));
    render_mesh("cuboid", &solid::cuboid(r(1.4), r(2.3), r(0.95)));
    render_mesh("sphere", &solid::sphere(r(1.0), 32, 16));
    render_mesh("cylinder", &solid::cylinder(r(1.0), r(2.0), 32));
    render_mesh("frustum", &solid::frustum(r(0.65), r(1.05), r(2.0), 32));
    render_mesh("octahedron", &solid::octahedron(r(1.2)));
    render_mesh("icosahedron", &solid::icosahedron(r(1.2)));
    render_mesh("torus", &solid::torus(r(1.25), r(0.35), 36, 14));
    render_mesh(
        "mesh_arrow",
        &solid::arrow(Point3::origin(), v3(0.8, 0.4, 2.0), 32, false),
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
        "revolve",
        &curve::revolve(
            &curve::circle(r(0.18), 32),
            r(265.0),
            32,
            &csgrs::GeometryContext::STRICT,
        )
        .expect("revolve")
        .into_value()
        .translated(r(1.0), r(0.0), r(0.0)),
    );

    render_mesh("inverse", &solid::inverse(&solid::sphere(r(1.0), 32, 16)));
    render_mesh("csg", &cube_minus_translated_sphere());
    render_mesh(
        "convex_hull",
        &solid::convex_hull(&solid::cube(r(1.2))).expect("convex hull"),
    );
    render_mesh(
        "minkowski_sum",
        &solid::minkowski_sum(&solid::cube(r(1.1)), &solid::sphere(r(0.45), 16, 8))
            .expect("Minkowski sum"),
    );

    let balls = [
        MetaBall::new(p3(-0.55, 0.0, 0.0), r(0.65)),
        MetaBall::new(p3(0.55, 0.0, 0.0), r(0.65)),
        MetaBall::new(p3(0.0, 0.55, 0.25), r(0.58)),
    ];
    render_mesh(
        "metaballs_3d",
        &solid::metaballs(&balls, (16, 16, 16), r(0.7), r(0.25)),
    );

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
}

fn cube_minus_translated_sphere() -> TriangleMesh {
    let cube = solid::cube(r(2.0));
    let sphere = solid::sphere(r(1.25), 16, 8).translated(r(1.0), r(1.0), r(1.0));
    cube.try_difference(&sphere).expect("difference")
}

fn render_curve(name: &str, region: &CurveRegion2) {
    let mut image = RgbaImage::from_pixel(SIZE, SIZE, BG);
    let profiles = curve::try_finite_profiles(region, &csgrs::GeometryContext::STRICT)
        .expect("finite curve profiles")
        .into_value();
    let wires = Vec::<Vec<[Real; 2]>>::new();
    let Some(bounds) = curve_bounds(&profiles, &wires) else {
        save_image(name, &image);
        return;
    };
    let map = Map2::new(bounds);

    for profile in &profiles {
        stroke_points(&mut image, &map, profile.material().points(), EDGE_2D);
        for hole in profile.holes() {
            stroke_points(&mut image, &map, hole.points(), EDGE_2D);
        }
    }
    for wire in &wires {
        stroke_real_points(&mut image, &map, wire, INK_2D);
    }
    save_image(name, &image);
}

fn render_mesh(name: &str, mesh: &TriangleMesh) {
    let mut image = RgbaImage::from_pixel(SIZE, SIZE, BG);
    if mesh.triangles.is_empty() {
        save_image(name, &image);
        return;
    }

    let projected = mesh
        .triangles
        .iter()
        .map(|triangle| {
            let [a, b, c] = triangle.indices();
            [
                project_point(&mesh.positions[a]),
                project_point(&mesh.positions[b]),
                project_point(&mesh.positions[c]),
            ]
        })
        .collect::<Vec<_>>();

    let Some(bounds) = projected_bounds(&projected) else {
        save_image(name, &image);
        return;
    };
    let map = Map2::new(bounds);

    for tri in projected {
        let a = map.point(tri[0]);
        let b = map.point(tri[1]);
        let c = map.point(tri[2]);
        fill_triangle(&mut image, a, b, c, FILL_3D);
        draw_line(&mut image, a, b, EDGE_3D);
        draw_line(&mut image, b, c, EDGE_3D);
        draw_line(&mut image, c, a, EDGE_3D);
    }
    save_image(name, &image);
}

fn stroke_points(image: &mut RgbaImage, map: &Map2, points: &[[f64; 2]], color: Rgba<u8>) {
    let pixels = points
        .iter()
        .map(|point| map.point((point[0], point[1])))
        .collect::<Vec<_>>();
    for pair in pixels.windows(2) {
        draw_line(image, pair[0], pair[1], color);
    }
}

fn stroke_real_points(
    image: &mut RgbaImage,
    map: &Map2,
    points: &[[Real; 2]],
    color: Rgba<u8>,
) {
    let pixels = points
        .iter()
        .map(|point| map.point((real_to_f64(&point[0]), real_to_f64(&point[1]))))
        .collect::<Vec<_>>();
    for pair in pixels.windows(2) {
        draw_line(image, pair[0], pair[1], color);
    }
}

fn curve_bounds(
    profiles: &[hypercurve::FiniteRegionProfile2],
    wires: &[Vec<[Real; 2]>],
) -> Option<(f64, f64, f64, f64)> {
    let mut bounds = None;
    for profile in profiles {
        include_points(&mut bounds, profile.material().points());
        for hole in profile.holes() {
            include_points(&mut bounds, hole.points());
        }
    }
    for wire in wires {
        include_real_points(&mut bounds, wire);
    }
    bounds
}

fn projected_bounds(triangles: &[[(f64, f64); 3]]) -> Option<(f64, f64, f64, f64)> {
    let mut bounds = None;
    for point in triangles.iter().flat_map(|tri| tri.iter()) {
        include_point(&mut bounds, *point);
    }
    bounds
}

fn include_points(bounds: &mut Option<(f64, f64, f64, f64)>, points: &[[f64; 2]]) {
    for point in points {
        include_point(bounds, (point[0], point[1]));
    }
}

fn include_real_points(bounds: &mut Option<(f64, f64, f64, f64)>, points: &[[Real; 2]]) {
    for point in points {
        include_point(bounds, (real_to_f64(&point[0]), real_to_f64(&point[1])));
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

fn project_point(point: &Point3) -> (f64, f64) {
    let x = real_to_f64(&point.x);
    let y = real_to_f64(&point.y);
    let z = real_to_f64(&point.z);
    (x - y, (x + y) * 0.45 - z)
}

fn fill_triangle(
    image: &mut RgbaImage,
    a: (u32, u32),
    b: (u32, u32),
    c: (u32, u32),
    color: Rgba<u8>,
) {
    let min_x = a.0.min(b.0).min(c.0);
    let max_x = a.0.max(b.0).max(c.0);
    let min_y = a.1.min(b.1).min(c.1);
    let max_y = a.1.max(b.1).max(c.1);
    let af = (a.0 as f64, a.1 as f64);
    let bf = (b.0 as f64, b.1 as f64);
    let cf = (c.0 as f64, c.1 as f64);
    let area = edge(af, bf, cf);
    if area.abs() < f64::EPSILON {
        return;
    }
    for y in min_y..=max_y {
        for x in min_x..=max_x {
            let p = (x as f64 + 0.5, y as f64 + 0.5);
            let w0 = edge(bf, cf, p);
            let w1 = edge(cf, af, p);
            let w2 = edge(af, bf, p);
            if (w0 >= 0.0 && w1 >= 0.0 && w2 >= 0.0) || (w0 <= 0.0 && w1 <= 0.0 && w2 <= 0.0) {
                image.put_pixel(x, y, color);
            }
        }
    }
}

fn edge(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> f64 {
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

    fn point(&self, point: (f64, f64)) -> (u32, u32) {
        let px = ((point.0 - self.min_x) * self.scale + self.offset_x).round();
        let py = (SIZE as f64 - 1.0 - ((point.1 - self.min_y) * self.scale + self.offset_y))
            .round();
        (
            px.clamp(0.0, SIZE as f64 - 1.0) as u32,
            py.clamp(0.0, SIZE as f64 - 1.0) as u32,
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
    value.to_f64_lossy().unwrap_or(0.0)
}

fn p3(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

fn v3(x: f64, y: f64, z: f64) -> Vector3 {
    Vector3::from_xyz(r(x), r(y), r(z))
}
