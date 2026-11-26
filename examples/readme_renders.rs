//! Generate lightweight PNG renders for README assets.
//!
//! Finite fixture literals enter native geometry as exact binary values.
//! Curves/meshes are projected to binary64 for display; raster tolerances,
//! shading and depth biases do not participate in native geometry decisions.
//! Several fixtures permit approximate predicates and extract their outcomes
//! without reporting receipts. PNGs are illustrations, not geometry certificates.
//!
//! Requires `curve,image-io,truetype-text,metaballs,sdf,offset,hershey-text`;
//! the last feature is used by the text renderer even though the manifest's
//! required-feature list omits it. Default features currently supply it.
//! `README_RENDER_OUTPUT_DIR` overrides the working-directory-relative `docs`
//! destination. Presence of `README_RENDER_TEXT_ONLY` selects only two text
//! images. Files are replaced individually; failures panic and earlier writes remain.

use csgrs::solid::MetaBall;
use csgrs::{
    GeometryContext, TriangleMesh, curve,
    solid::{self, SolidExt},
};
use hypercurve::{
    CurvePath2, CurveRegion2, CurveString2, FiniteProjectionOptions, FiniteRegionProfile2,
    Point2,
};
use hyperlattice::{Point3, Real, Vector3};
use image::{GrayImage, Luma, Rgba, RgbaImage};
use std::{collections::BTreeSet, fs, path::PathBuf};

/// Square raster side length and fixed depth-buffer stride in pixels.
const SIZE: u32 = 768;
/// Fractional margin reserved on each side of the longest mapped extent.
const PADDING: f64 = 0.12;
/// Model-space chord-error request for finite curve display projection, not a pixel bound.
const CURVE_CHORD_ERROR: f64 = 1.0e-3;
/// Transparent initial canvas, also saved for empty geometry.
const BG: Rgba<u8> = Rgba([0, 0, 0, 0]);
/// Flat fill color for projected planar material triangles.
const FACE_2D: Rgba<u8> = Rgba([68, 150, 173, 255]);
/// Dark endpoint of the face-shading ramp and degenerate-normal fallback.
const FACE_SHADOW_3D: Rgba<u8> = Rgba([54, 108, 142, 255]);
/// Light endpoint of the face-shading ramp.
const FACE_LIGHT_3D: Rgba<u8> = Rgba([120, 201, 204, 255]);
/// Stroke color for edges and vertex outlines.
const EDGE: Rgba<u8> = Rgba([16, 34, 49, 255]);
/// Interior color of represented-vertex markers.
const VERTEX: Rgba<u8> = Rgba([255, 203, 79, 255]);
/// Pixel width of planar region/path strokes.
const EDGE_WIDTH_2D: f64 = 3.25;
/// Unnormalized viewing direction, aligned with increasing projected depth.
const VIEW_DIRECTION: [f64; 3] = [1.0, 1.0, 0.9];
/// Unnormalized fixed light direction for two-sided face shading.
const LIGHT_DIRECTION: [f64; 3] = [0.25, 0.55, 1.0];

/// Creates the selected directory and renders text only or the full curve/mesh catalog.
/// Any present text-only environment value selects text mode; no receipt summary,
/// output-set verification or atomic batch publication is performed.
fn main() {
    let output_dir = output_dir();
    fs::create_dir_all(&output_dir).expect("create README render output directory");

    if std::env::var_os("README_RENDER_TEXT_ONLY").is_some() {
        render_readme_text();
    } else {
        render_readme_curves();
        render_readme_meshes();
    }

    println!("README renders written to {}/*.png", output_dir.display());
}

/// Constructs the fixed planar catalog, raster/text fixtures and open curve examples.
/// Several Boolean fixtures use `APPROXIMATE_512`; extracted values are rendered
/// without reporting those receipts. Any construction/projection/write error panics.
fn render_readme_curves() {
    render_curve("square", &curve::square(2.0).expect("valid planar fixture"));
    render_curve(
        "rectangle",
        &curve::rectangle(2.4, 1.35).expect("valid planar fixture"),
    );
    render_curve(
        "circle",
        &curve::circle(1.0, 96).expect("valid planar fixture"),
    );
    render_curve(
        "right_triangle",
        &curve::right_triangle(2.0, 1.5).expect("valid planar fixture"),
    );
    render_curve(
        "polygon",
        &curve::polygon(&[[r(0.0), r(1.2)], [r(-1.1), r(-0.8)], [r(1.1), r(-0.8)]])
            .expect("valid planar fixture"),
    );
    render_curve(
        "rounded_rectangle",
        &curve::rounded_rectangle(2.4, 1.5, 0.28, 12).expect("valid planar fixture"),
    );
    render_curve(
        "ellipse",
        &curve::ellipse(2.3, 1.3, 96).expect("valid planar fixture"),
    );
    render_curve(
        "regular_ngon",
        &curve::regular_ngon(6, 1.0).expect("valid planar fixture"),
    );
    render_curve(
        "curve_arrow",
        &curve::arrow(2.2, 0.35, 0.8, 1.0).expect("valid sampled profile fixture"),
    );
    render_curve(
        "trapezoid",
        &curve::trapezoid(1.2, 2.2, 1.4, 0.45).expect("valid sampled profile fixture"),
    );
    render_curve(
        "star",
        &curve::star(5, 1.1, 0.45).expect("valid sampled profile fixture"),
    );
    render_curve(
        "teardrop",
        &curve::teardrop(1.5, 2.3, 96).expect("valid sampled profile fixture"),
    );
    render_curve(
        "curve_egg",
        &curve::egg(1.6, 2.2, 96).expect("valid sampled profile fixture"),
    );
    render_curve(
        "squircle",
        &curve::squircle(2.0, 1.6, 128).expect("valid sampled profile fixture"),
    );
    render_curve(
        "keyhole",
        &curve::keyhole(r(0.75), r(0.55), r(1.2), 96, GeometryContext::APPROXIMATE_512)
            .expect("keyhole Boolean")
            .into_value(),
    );
    render_curve(
        "reuleaux",
        &curve::reuleaux(3, r(1.5), 96, GeometryContext::APPROXIMATE_512)
            .expect("Reuleaux intersections")
            .into_value(),
    );
    render_curve(
        "heart",
        &curve::heart(2.0, 1.8, 160).expect("valid sampled profile fixture"),
    );
    render_curve(
        "ring",
        &curve::ring(r(1.4), r(0.35), 96, GeometryContext::APPROXIMATE_512)
            .expect("ring Boolean")
            .into_value(),
    );
    render_curve(
        "pie_slice",
        &curve::pie_slice(1.1, -35.0, 115.0, 64).expect("valid sampled profile fixture"),
    );
    render_curve(
        "crescent",
        &curve::crescent(r(1.1), r(0.9), r(0.48), 96, GeometryContext::APPROXIMATE_512)
            .expect("crescent Boolean")
            .into_value(),
    );
    render_curve(
        "supershape",
        &curve::supershape(1.0, 1.0, 5.0, 2.0, 7.0, 7.0, 160)
            .expect("valid sampled profile fixture"),
    );
    render_curve(
        "circle_with_keyway",
        &curve::circle_with_keyway(
            r(1.0),
            96,
            r(0.45),
            r(0.35),
            GeometryContext::APPROXIMATE_512,
        )
        .expect("keyway Boolean")
        .into_value(),
    );
    render_curve(
        "circle_with_flat",
        &curve::circle_with_flat(r(1.0), 96, r(0.72), GeometryContext::APPROXIMATE_512)
            .expect("single-flat Boolean")
            .into_value(),
    );
    render_curve(
        "circle_with_two_flats",
        &curve::circle_with_two_flats(r(1.0), 96, r(0.72), GeometryContext::APPROXIMATE_512)
            .expect("double-flat Boolean")
            .into_value(),
    );
    render_curve(
        "involute_gear",
        &curve::involute_gear(0.22, 14, 20.0, 0.02, 0.01, 5)
            .expect("valid sampled mechanical fixture"),
    );
    render_curve(
        "cycloidal_gear",
        &curve::cycloidal_gear(0.22, 14, 0.275, 0.02, 8)
            .expect("valid sampled mechanical fixture"),
    );
    render_curve(
        "involute_rack",
        &curve::involute_rack(0.32, 7, 20.0, 0.02, 0.01)
            .expect("valid sampled mechanical fixture"),
    );
    render_curve(
        "cycloidal_rack",
        &curve::cycloidal_rack(0.32, 7, 0.02, 12).expect("valid sampled mechanical fixture"),
    );
    render_curve(
        "airfoil_naca4",
        &curve::airfoil_naca4(2.0, 4.0, 12.0, 3.0, 96)
            .expect("valid sampled mechanical fixture"),
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
        )
        .expect("valid closed Bezier fixture"),
    );
    let curve_balls = [(Point2::new(r(0.0), r(0.0)), r(1.0))];
    render_curve(
        "metaballs_2d",
        &curve::metaballs(&curve_balls, (24, 24), 1.0, 0.0)
            .expect("valid planar metaball fixture"),
    );
    render_curve("from_image", &readme_raster_shape());
    render_readme_text();
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
        &curve::hilbert_strings(&curve::square(2.0).expect("valid planar fixture"), 4, 0.12)
            .expect("exact Hilbert strings in a square"),
    );
}

/// Renders `CSG` from the compile-time bundled TrueType font and ROWMANS Hershey strokes.
/// Requires both font features even when only one output is of interest.
fn render_readme_text() {
    render_curve(
        "truetype",
        &curve::truetype_text("CSG", include_bytes!("../asar.ttf"), 72.0)
            .expect("valid TrueType outlines"),
    );
    render_open_curves(
        "hershey_strings",
        &[],
        &curve::hershey_strings("CSG", &hypercurve::hershey::fonts::ROWMANS, 1.0)
            .expect("valid Hershey strokes"),
    );
}

/// Traces a 10-by-10 white pixel block inside a 16-by-16 black grayscale fixture.
/// The threshold is 128; the resulting geometry represents that raster selection.
fn readme_raster_shape() -> CurveRegion2 {
    let mut image = GrayImage::from_pixel(16, 16, Luma([0]));
    for y in 3..13 {
        for x in 3..13 {
            image.put_pixel(x, y, Luma([255]));
        }
    }
    curve::from_image(&image, 128).expect("trace README raster shape")
}

/// Renders fixed primitive, extrusion, sweep, Boolean, projection and implicit fixtures.
/// Mixes strict wrappers and explicitly approximate-policy outcomes; no aggregate
/// receipt or independent model oracle is produced for the catalog.
fn render_readme_meshes() {
    render_mesh("cube", &solid::cube(2.0).expect("valid primitive fixture"));
    render_mesh(
        "cuboid",
        &solid::cuboid(1.4, 2.3, 0.95).expect("valid primitive fixture"),
    );
    render_mesh(
        "sphere",
        &solid::sphere(1.0, 32, 16).expect("valid primitive fixture"),
    );
    render_mesh(
        "cylinder",
        &solid::cylinder(1.0, 2.0, 32).expect("valid primitive fixture"),
    );
    render_mesh(
        "ellipsoid",
        &solid::ellipsoid(1.25, 0.8, 1.65, 32, 16).expect("valid primitive fixture"),
    );
    render_mesh(
        "frustum",
        &solid::frustum(0.65, 1.05, 2.0, 32).expect("valid primitive fixture"),
    );
    render_mesh(
        "frustum_between",
        &solid::frustum_between(p3(-0.8, -0.5, -0.7), p3(0.8, 0.5, 1.2), 0.65, 0.3, 32)
            .expect("valid primitive fixture"),
    );
    render_mesh(
        "octahedron",
        &solid::octahedron(1.2).expect("valid primitive fixture"),
    );
    render_mesh(
        "icosahedron",
        &solid::icosahedron(1.2).expect("valid primitive fixture"),
    );
    render_mesh(
        "torus",
        &solid::torus(1.25, 0.35, 36, 14).expect("valid primitive fixture"),
    );
    render_mesh(
        "mesh_arrow",
        &solid::arrow(Point3::origin(), v3(0.8, 0.4, 2.0), 32, false)
            .expect("valid primitive fixture"),
    );
    render_mesh("polyhedron", &readme_polyhedron());
    render_mesh(
        "teardrop_cylinder",
        &solid::teardrop_cylinder(
            r(1.5),
            r(2.3),
            r(0.8),
            48,
            GeometryContext::APPROXIMATE_512,
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
            GeometryContext::APPROXIMATE_512,
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
            GeometryContext::APPROXIMATE_512,
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
            GeometryContext::APPROXIMATE_512,
        )
        .expect("helical involute gear")
        .into_value(),
    );

    let star = curve::star(5, 1.0, 0.45).expect("valid sampled profile fixture");
    render_mesh(
        "extrude",
        &curve::try_extrude(&star, r(0.65), csgrs::GeometryContext::STRICT)
            .expect("extrude")
            .into_value(),
    );
    render_mesh(
        "extrude_vector",
        &curve::try_extrude_vector(&star, v3(0.45, 0.25, 0.9), csgrs::GeometryContext::STRICT)
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
            GeometryContext::APPROXIMATE_512,
        )
        .expect("twisted extrude")
        .into_value(),
    );
    let revolve_profile = curve::translated(
        &curve::circle(0.18, 32).expect("valid planar fixture"),
        1.0,
        0.0,
    )
    .expect("valid planar transform fixture");
    render_mesh(
        "revolve",
        &curve::revolve(&revolve_profile, r(265.0), 32, csgrs::GeometryContext::STRICT)
            .expect("revolve")
            .into_value(),
    );
    render_mesh(
        "sweep",
        &curve::try_sweep(
            &curve::square(0.4).expect("valid planar fixture"),
            &[p3(0.0, 0.0, -0.8), p3(0.0, 0.0, 1.1)],
            GeometryContext::STRICT,
        )
        .expect("sweep")
        .into_value(),
    );
    render_mesh("loft", &solid::loft(&readme_loft_sections()).expect("loft"));

    render_mesh(
        "inverse",
        &solid::inverse(&solid::sphere(1.0, 32, 16).expect("valid primitive fixture")),
    );
    render_mesh("csg", &cube_minus_translated_sphere());
    render_mesh(
        "convex_hull",
        &solid::convex_hull(&solid::cube(1.2).expect("valid primitive fixture"))
            .expect("convex hull"),
    );
    render_mesh(
        "minkowski_sum",
        &solid::minkowski_sum(
            &solid::cube(1.1).expect("valid primitive fixture"),
            &solid::octahedron(0.45).expect("valid primitive fixture"),
        )
        .expect("Minkowski sum"),
    );
    let projection_source =
        solid::frustum(0.65, 1.05, 2.0, 12).expect("valid primitive fixture");
    render_curve(
        "flatten",
        &solid::flatten(&projection_source).expect("valid projection fixture"),
    );
    let (slice, _, _) = solid::slice_z(&projection_source, 1.0).expect("valid slice fixture");
    render_curve("slice_z", &slice);

    render_implicit_meshes();
}

/// Runs metaball, quadratic-field and six TPMS illustration builders in that order.
fn render_implicit_meshes() {
    render_metaballs_mesh();
    render_sdf_mesh();
    render_tpms_meshes();
}

/// Samples three fixed positive metaballs on an 8-cubed grid for a display mesh.
fn render_metaballs_mesh() {
    let balls = [
        MetaBall::new(p3(-0.55, 0.0, 0.0), r(0.65)),
        MetaBall::new(p3(0.55, 0.0, 0.0), r(0.65)),
        MetaBall::new(p3(0.0, 0.55, 0.25), r(0.58)),
    ];
    render_mesh(
        "metaballs_3d",
        &solid::metaballs(&balls, (8, 8, 8), 0.7, 0.25).expect("valid metaball fixture"),
    );
}

/// Samples `x² + y² + z² - 0.72` on a 16-cubed grid in `[-1,1]³` at isovalue zero.
/// This implicit field is not a signed distance; the result is a sampled mesh.
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
            0.0,
        )
        .expect("valid SDF fixture"),
    );
}

/// Renders band-solid and surface variants of three TPMS fields in a two-unit box.
/// Fixed grid/threshold parameters produce finite illustrations, not analytic surfaces.
fn render_tpms_meshes() {
    // Keep implicit thumbnail meshes legible once their vertices are visible.
    let tpms_box = solid::cube(2.0).expect("valid primitive fixture");
    render_mesh(
        "gyroid",
        &solid::gyroid_solid(&tpms_box, 24, 2.0, 0.0, 0.18).expect("valid modeling fixture"),
    );
    render_mesh(
        "schwarz_p",
        &solid::schwarz_p_solid(&tpms_box, 24, 2.0, 0.0, 0.18)
            .expect("valid modeling fixture"),
    );
    render_mesh(
        "schwarz_d",
        &solid::schwarz_d_solid(&tpms_box, 24, 2.0, 0.0, 0.18)
            .expect("valid modeling fixture"),
    );
    render_mesh(
        "gyroid_surface",
        &solid::gyroid(&tpms_box, 24, 2.0, 0.2).expect("valid modeling fixture"),
    );
    render_mesh(
        "schwarz_p_surface",
        &solid::schwarz_p(&tpms_box, 24, 2.0, 0.2).expect("valid modeling fixture"),
    );
    render_mesh(
        "schwarz_d_surface",
        &solid::schwarz_d(&tpms_box, 24, 2.0, 0.2).expect("valid modeling fixture"),
    );
}

/// Builds a square-base pyramid from five authored points and oriented polygon faces.
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

/// Returns three four-corner rectangular sections at Z=-0.8, 0 and 1 with matching order.
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

/// Evaluates one native cube-minus-sampled-sphere program with `APPROXIMATE_512`.
/// Extracts the last materialized mesh without reporting the outcome receipt;
/// operation/unbounded/materialization/empty-result failures panic.
fn cube_minus_translated_sphere() -> TriangleMesh {
    let cube = solid::cube(2.0).expect("valid primitive fixture");
    let sphere = solid::sphere(1.25, 16, 8)
        .expect("valid primitive fixture")
        .translated(1.0, 1.0, 1.0)
        .expect("valid transform fixture");
    let context = hypermesh::MeshContext::new(hypermesh::PredicatePolicy::APPROXIMATE_512);
    hypermesh::boolean(
        &context,
        &[cube.as_ref(), sphere.as_ref()],
        hypermesh::BooleanProgram::Operation(hypermesh::BooleanOp::Difference),
    )
    .expect("translated-sphere difference")
    .into_value()
    .into_triangle_meshes()
    .expect("translated-sphere difference is bounded")
    .pop()
    .expect("one translated-sphere result")
}

/// Projects exact loop roles, fills finite profile triangles, strokes boundaries and marks endpoints.
/// Uses represented source vertices rather than subdivision vertices. Empty bounds
/// save a transparent image; projection, triangulation and write failures panic.
/// Subsequent pixel coverage is approximate and supplies no native geometry evidence.
fn render_curve(name: &str, region: &CurveRegion2) {
    let mut image = RgbaImage::from_pixel(SIZE, SIZE, BG);
    let projection = FiniteProjectionOptions::try_new(CURVE_CHORD_ERROR)
        .expect("README curve chord error is positive");
    // Material/hole ownership and edge identity are decided on CurveRegion2
    // before the finite raster projection is created.
    let profiles = region
        .project_to_finite_profiles_exact(&projection)
        .expect("project exact CurveRegion2 profiles");
    let edge_paths = region
        .project_to_finite_curve_paths()
        .expect("project exact CurveRegion2 edge paths");
    let Some(bounds) = curve_bounds(&profiles) else {
        save_image(name, &image);
        return;
    };
    let map = Map2::new(bounds);

    for profile in &profiles {
        for triangle in profile
            .triangulate()
            .expect("triangulate exact CurveRegion2 profile")
        {
            let [a, b, c] = triangle.map(|point| map.point((point[0], point[1])));
            fill_triangle_2d(&mut image, a, b, c, FACE_2D);
        }
    }

    let edge_polylines = project_edge_paths(&edge_paths, &projection);
    for points in &edge_polylines {
        stroke_points(&mut image, &map, points, EDGE_WIDTH_2D, EDGE);
    }

    let vertices = exact_region_vertices(region);
    let radius = vertex_radius(vertices.len());
    for vertex in vertices {
        let vertex = (real_to_f64(vertex.x()), real_to_f64(vertex.y()));
        draw_vertex_2d(&mut image, map.point(vertex), radius);
    }
    save_image(name, &image);
}

/// Collects source segment starts from a native contour view, retaining duplicates.
/// An absent or failed fast view falls back to represented boundary start coordinates;
/// selected endpoints without coordinates are omitted, not approximated from isolators.
fn exact_region_vertices(region: &CurveRegion2) -> Vec<Point2> {
    if let Ok(Some(native)) = region.native_contours_fast_path() {
        return native
            .material_contours()
            .iter()
            .chain(native.hole_contours())
            .flat_map(|contour| contour.segments().iter())
            .map(|segment| segment.start().clone())
            .collect();
    }

    // Higher-order regions have no native line/arc view. Draw only represented
    // endpoints retained by the exact boundary itself. A selected endpoint
    // without coordinates stays unmarked: turning an isolating interval into a
    // screen coordinate would invent a projected vertex.
    region
        .boundary_loops()
        .iter()
        .flat_map(|boundary| boundary.curves())
        .filter_map(|curve| curve.start().coordinates().cloned())
        .collect()
}

/// Projects paths/strings to strokes and marks represented starts plus each final endpoint.
/// Does not fill or force closure; missing selected coordinates remain unmarked.
/// Repeated markers are retained. No projected points yields a transparent PNG;
/// failed projection or writing panics.
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
        vertices.extend(
            path.curves()
                .iter()
                .filter_map(|edge| edge.start().coordinates().cloned()),
        );
        vertices.extend(path.end().coordinates().cloned());
    }
    for string in strings {
        vertices.extend(
            string
                .segments()
                .iter()
                .map(|segment| segment.start().clone()),
        );
        if let Some(last) = string.segments().last() {
            vertices.push(last.end().clone());
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

/// Projects all stored positions, shades triangles and overlays used indexed edges/vertices.
/// Bounds use referenced positions, but unused positions are still projected. Invalid
/// indices can panic. Empty triangle lists save a transparent image before projection.
/// Includes triangulation diagonals; edge identity is by indices, without welding.
/// Positive depth bias is a display heuristic, not certified hidden-line geometry.
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

    // Faces establish raster depth; biased overlay tests keep edges/vertices
    // visible near a surface without claiming exact occlusion.
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

/// Converts each retained display boundary path to a finite polyline at the requested chord error.
/// Preserves path order and panics on any failed projection.
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

/// Maps finite polyline points to pixels and strokes consecutive pairs only.
/// Does not add closure; fewer than two points draws nothing.
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

/// Accumulates finite coordinate bounds over material and hole vertices; no points yields `None`.
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

/// Extends display bounds with every supplied point, preserving `None` for an empty slice.
fn include_points(bounds: &mut Option<(f64, f64, f64, f64)>, points: &[[f64; 2]]) {
    for point in points {
        include_point(bounds, (point[0], point[1]));
    }
}

/// Initializes or extends `(min_x,min_y,max_x,max_y)` using float min/max without finite checks.
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
/// Floating display-plane position and unnormalized linear view depth.
struct ProjectedPoint3 {
    /// Coordinates before image fitting and vertical flip.
    plane: (f64, f64),
    /// Larger values are nearer in the raster depth test.
    depth: f64,
}

#[derive(Clone, Copy)]
/// Image-space point paired with the original projected view depth.
struct RasterPoint3 {
    /// Horizontal pixel coordinate.
    x: f64,
    /// Vertically flipped pixel coordinate.
    y: f64,
    /// View depth, not rescaled by the 2D image map.
    depth: f64,
}

impl RasterPoint3 {
    /// Extracts the pixel position for coverage/bounds calculations.
    const fn plane(self) -> (f64, f64) {
        (self.x, self.y)
    }
}

/// Lossily projects native XYZ coordinates for display, panicking on unavailable conversion.
fn point3_to_f64(point: &Point3) -> [f64; 3] {
    [
        real_to_f64(&point.x),
        real_to_f64(&point.y),
        real_to_f64(&point.z),
    ]
}

/// Uses plane `(x-y, 0.45(x+y)-z)` and depth `x+y+0.9z` without perspective division.
/// The perpendicular plane axes have different scales; this is a fixed display map.
fn project_point([x, y, z]: [f64; 3]) -> ProjectedPoint3 {
    ProjectedPoint3 {
        plane: (x - y, (x + y) * 0.45 - z),
        depth: x + y + z * 0.9,
    }
}

/// Flips the float face normal toward the viewer and mixes ambient/diffuse/facing terms.
/// Failed normalization uses the shadow color; winding does not cause backface culling.
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

/// Subtracts display-space vectors componentwise.
fn subtract3(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
    [left[0] - right[0], left[1] - right[1], left[2] - right[2]]
}

/// Computes the floating right-handed cross product for shading normals.
fn cross3(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
    [
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    ]
}

/// Computes a floating dot product for shading and display normalization.
fn dot3(left: [f64; 3], right: [f64; 3]) -> f64 {
    left[0] * right[0] + left[1] * right[1] + left[2] * right[2]
}

/// Divides by the float norm only when it exceeds `f64::EPSILON`.
/// NaN/too-small norms return `None`; overflow/infinite components are not separately checked.
fn normalize3(vector: [f64; 3]) -> Option<[f64; 3]> {
    let length = dot3(vector, vector).sqrt();
    (length > f64::EPSILON).then(|| vector.map(|component| component / length))
}

/// Interpolates all four byte channels, rounding and clamping each result to 0..255.
/// This is encoded-channel arithmetic, without linear-light color conversion.
fn mix_color(shadow: Rgba<u8>, light: Rgba<u8>, amount: f64) -> Rgba<u8> {
    Rgba(std::array::from_fn(|channel| {
        let value =
            f64::from(shadow[channel]) * (1.0 - amount) + f64::from(light[channel]) * amount;
        value.round().clamp(0.0, 255.0) as u8
    }))
}

/// Replaces pixel-center samples inside a float triangle using a `1e-9` barycentric slack.
/// Skips near-zero signed area; assumes a `SIZE` square image and finite raster inputs.
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

/// Rasterizes barycentric pixel-center coverage and linearly interpolated view depth.
/// Nearer/equal samples replace both color and depth, so later faces win exact ties.
/// Uses float area/slack thresholds; assumes a `SIZE²` depth buffer and square image.
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

/// Forms an inclusive floored/ceiled raster box and limits it to the fixed image extent.
/// Returns `None` for inverted resulting ranges; float finiteness is not validated.
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

/// Returns `(c-a) × (b-a)`, the negative conventional oriented twice-area.
/// Consistent use in numerator and denominator gives raster barycentric weights.
fn edge(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> f64 {
    (c.0 - a.0) * (b.1 - a.1) - (c.1 - a.1) * (b.0 - a.0)
}

/// Blends a round-ended stroke using distance to the clamped segment and a one-pixel coverage ramp.
/// Assumes nonnegative width and finite pixels; does not inspect geometry/depth.
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

/// Blends a stroke only where interpolated depth plus bias reaches the face depth buffer.
/// Reads but does not update depth, so overlays do not occlude later overlays.
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

/// Builds an inclusive line bounding box padded by half-width plus one pixel.
/// Clips to the fixed image extent without validating width or finite coordinates.
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

/// Projects a pixel onto a segment and clamps to `[0,1]`; tiny squared length returns zero.
/// `delta` and `length_squared` must describe the same segment; no consistency check is made.
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

/// Chooses a smaller fixed marker radius for denser vertex sets; counts include duplicate markers.
fn vertex_radius(vertex_count: usize) -> f64 {
    match vertex_count {
        0..=48 => 4.75,
        49..=192 => 3.75,
        193..=768 => 2.5,
        769..=2_048 => 1.35,
        _ => 0.75,
    }
}

/// Chooses one of four pixel stroke widths from the number of distinct indexed mesh edges.
fn edge_width(edge_count: usize) -> f64 {
    match edge_count {
        0..=100 => 2.5,
        101..=1_000 => 1.9,
        1_001..=5_000 => 1.35,
        _ => 0.85,
    }
}

/// Blends an edge-colored outline disc followed by the vertex-colored inner disc.
fn draw_vertex_2d(image: &mut RgbaImage, center: (f64, f64), radius: f64) {
    draw_disc(image, center, vertex_outline_radius(radius), EDGE);
    draw_disc(image, center, radius, VERTEX);
}

/// Draws outlined vertex discs using their common center depth and the face-buffer bias test.
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

/// Adds an outline thickness capped at 1.35 pixels for the caller's positive marker radius.
fn vertex_outline_radius(radius: f64) -> f64 {
    radius + (radius * 0.35 + 0.5).min(1.35)
}

/// Blends circular pixel-center coverage with a one-pixel edge ramp and no depth test.
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

/// Blends disc coverage where center depth plus bias reaches the face buffer.
/// Uses one depth across the marker and never updates the buffer itself.
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

/// Bounds a disc with radius-plus-one padding, clipped to the fixed square canvas.
fn disc_pixel_bounds(center: (f64, f64), radius: f64) -> Option<(u32, u32, u32, u32)> {
    let padding = radius + 1.0;
    let min_x = (center.0 - padding).floor().max(0.0) as u32;
    let min_y = (center.1 - padding).floor().max(0.0) as u32;
    let max_x = (center.0 + padding).ceil().min(f64::from(SIZE - 1)) as u32;
    let max_y = (center.1 + padding).ceil().min(f64::from(SIZE - 1)) as u32;
    (min_x <= max_x && min_y <= max_y).then_some((min_x, min_y, max_x, max_y))
}

/// Computes row-major depth-buffer indexing; callers must supply coordinates below `SIZE`.
fn pixel_index(x: u32, y: u32) -> usize {
    (y * SIZE + x) as usize
}

/// Composites source-over in straight RGBA byte-channel space, scaled by coverage.
/// Skips negligible output alpha, then rounds/clamps channels; no gamma correction.
/// Assumes valid pixel coordinates and coverage in `[0,1]`.
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

/// Uniform fit of finite display bounds to the fixed square image, followed by Y flip/clamping.
struct Map2 {
    /// Minimum authored display X before fitting.
    min_x: f64,
    /// Minimum authored display Y before fitting.
    min_y: f64,
    /// Shared pixels-per-display-unit scale.
    scale: f64,
    /// Horizontal centering margin in pixels.
    offset_x: f64,
    /// Vertical centering margin before flipping image Y.
    offset_y: f64,
}

impl Map2 {
    /// Fits the longest extent inside fractional padding, flooring both extents at `1e-6`.
    /// Assumes meaningful finite bounds; inverted/nonfinite data is not rejected.
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

    /// Translates/scales, flips Y and clamps both coordinates into the pixel extent.
    /// Out-of-bounds points collapse to the border rather than receiving geometric clipping.
    fn point(&self, point: (f64, f64)) -> (f64, f64) {
        let px = (point.0 - self.min_x) * self.scale + self.offset_x;
        let py = SIZE as f64 - 1.0 - ((point.1 - self.min_y) * self.scale + self.offset_y);
        (
            px.clamp(0.0, SIZE as f64 - 1.0),
            py.clamp(0.0, SIZE as f64 - 1.0),
        )
    }
}

/// Replaces the named PNG in the currently selected directory and prints its path.
/// Assumes directory creation already occurred; encoding/IO failure panics.
fn save_image(name: &str, image: &RgbaImage) {
    let path = output_dir().join(name).with_extension("png");
    image.save(&path).expect("save README render");
    println!("wrote {}", path.display());
}

/// Reads the OS-string output override, otherwise returning working-directory-relative `docs`.
/// Re-evaluated on each save; empty or relative override values are not normalized.
fn output_dir() -> PathBuf {
    std::env::var_os("README_RENDER_OUTPUT_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| PathBuf::from("docs"))
}

/// Promotes finite binary64 fixture data exactly; nonfinite input panics.
fn r(value: f64) -> Real {
    Real::try_from(value).expect("README render constants are finite")
}

/// Extracts the lossy display conversion, panicking on `None` without an extra finite-value filter.
fn real_to_f64(value: &Real) -> f64 {
    value
        .to_f64_lossy()
        .expect("README render coordinates are finitely projectable")
}

/// Constructs a native point from exactly promoted finite float fixture coordinates.
fn p3(x: f64, y: f64, z: f64) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

/// Constructs a native vector from exactly promoted finite float fixture components.
fn v3(x: f64, y: f64, z: f64) -> Vector3 {
    Vector3::from_xyz(r(x), r(y), r(z))
}

#[cfg(test)]
/// Witness for retaining authored arc endpoints separately from display subdivision vertices.
mod tests {
    use super::*;
    use hypercurve::{BulgeVertex2, Contour2};

    #[test]
    /// Requires two semicircle endpoints despite more than two projected boundary fragments.
    /// Checks native marker membership/count, not PNG bytes or raster visibility.
    fn exact_region_vertices_ignore_arc_projection_fragments() {
        let right = Point2::new(Real::one(), Real::zero());
        let left = Point2::new(-Real::one(), Real::zero());
        let contour = Contour2::from_bulge_vertices(&[
            BulgeVertex2::new(right.clone(), Real::one()),
            BulgeVertex2::new(left.clone(), Real::one()),
        ])
        .expect("two exact semicircles form a closed contour");
        let region = CurveRegion2::try_from_native_material_contours(vec![contour])
            .expect("promote exact circular contour");

        let projected_paths = region
            .project_to_finite_curve_paths()
            .expect("project circular boundary");
        let projected_vertices = projected_paths
            .iter()
            .map(|path| path.curves().len())
            .sum::<usize>();
        assert!(
            projected_vertices > 2,
            "arc-to-Bezier projection must expose the former false markers"
        );

        let vertices = exact_region_vertices(&region);
        assert_eq!(vertices.len(), 2);
        assert!(vertices.contains(&right));
        assert!(vertices.contains(&left));
    }
}
