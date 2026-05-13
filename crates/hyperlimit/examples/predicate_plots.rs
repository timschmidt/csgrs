//! Render per-pixel predicate plots.
//!
//! Usage:
//!
//! ```sh
//! cargo run --example predicate_plots -- --out doc/predicate-plots --size 512
//! RUSTFLAGS='-Ctarget-cpu=haswell' cargo run --example predicate_plots --features geogram,robust,hyperreal,hyperlattice,interval -- --backend all --out doc/predicate-plots --size 512
//! ```
//!
//! Images are written as dependency-free PNG files.

use std::collections::BTreeSet;
use std::env;
use std::fs::{self, File};
use std::io::{self, Write};
use std::path::{Path, PathBuf};

#[cfg(any(feature = "hyperreal", feature = "hyperlattice"))]
use hyperlimit::BorrowedPredicateScalar;
use hyperlimit::orient::{
    classify_point_line_with_policy, incircle2d_with_policy, insphere3d_with_policy,
    orient2d_with_policy,
};
use hyperlimit::plane::{
    Plane3, classify_point_oriented_plane_with_policy, classify_point_plane_with_policy,
};
use hyperlimit::{
    Certainty, Escalation, LineSide, PlaneSide, Point2, Point3, PredicateOutcome, PredicatePolicy,
    Sign,
};

const DEFAULT_SIZE: usize = 512;
const DEFAULT_OUT_DIR: &str = "doc/predicate-plots";
const PLOTS_MD: &str = "plots.md";
const WORLD_MIN: f64 = -1.35;
const WORLD_MAX: f64 = 1.35;
const FP_ZOOM_SPAN: f64 = 1.0e-15;

const ORIENT_LINE_ZOOM: ZoomView = ZoomView::new(0.025, -0.05, FP_ZOOM_SPAN);
const INCIRCLE_ZOOM: ZoomView = ZoomView::new(-0.7, -0.45, FP_ZOOM_SPAN);
const EXPLICIT_PLANE_ZOOM: ZoomView = ZoomView::new(0.0625, 0.0, FP_ZOOM_SPAN);
const ORIENTED_PLANE_ZOOM: ZoomView = ZoomView::new(
    -0.120_833_333_333_333_33,
    -0.554_166_666_666_666_7,
    FP_ZOOM_SPAN,
);
const INSPHERE_ZOOM: ZoomView = ZoomView::new(0.82, 0.0, FP_ZOOM_SPAN);

#[derive(Clone, Copy)]
struct PlotConfig {
    name: &'static str,
    policy: PredicatePolicy,
}

#[derive(Clone, Copy)]
struct ZoomView {
    center_x: f64,
    center_y: f64,
    span: f64,
}

impl ZoomView {
    const fn new(center_x: f64, center_y: f64, span: f64) -> Self {
        Self {
            center_x,
            center_y,
            span,
        }
    }
}

fn main() -> io::Result<()> {
    let args = Args::parse()?;
    if args.check_gallery {
        return validate_plots_md(&args.out_dir);
    }

    fs::create_dir_all(&args.out_dir)?;

    let mut manifest = String::new();
    manifest.push_str("hyperlimit predicate plot demo\n");
    manifest.push_str(&format!("size: {}x{}\n", args.size, args.size));
    manifest.push_str(&format!("backend selection: {}\n", args.backend));
    manifest.push_str("colors: blue=positive/left/above, orange=negative/right/below, white=zero/on, black=unknown\n\n");

    let configs = [
        PlotConfig {
            name: "strict",
            policy: PredicatePolicy::STRICT,
        },
        PlotConfig {
            name: "approximate",
            policy: PredicatePolicy::APPROXIMATE,
        },
        PlotConfig {
            name: "strict_no_fallback",
            policy: PredicatePolicy {
                allow_robust_fallback: false,
                ..PredicatePolicy::STRICT
            },
        },
    ];

    if args.wants("f64") {
        for config in configs {
            render_f64_plots(
                &args.out_dir,
                args.size,
                config,
                !args.zoom_only,
                &mut manifest,
            )?;
        }
    }

    #[cfg(feature = "hyperreal")]
    if args.wants("hyperreal") {
        for config in configs {
            if args.zoom_only {
                copy_f64_zoom_aliases(
                    &args.out_dir,
                    "hyperreal",
                    "hyperreal::Real",
                    config,
                    &mut manifest,
                )?;
            } else {
                render_scalar_plots(
                    &args.out_dir,
                    args.size,
                    "hyperreal",
                    "hyperreal::Real",
                    config,
                    true,
                    real,
                    &mut manifest,
                )?;
            }
        }
    }

    #[cfg(not(feature = "hyperreal"))]
    if args.backend == "hyperreal" {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "--backend hyperreal requires the hyperreal feature",
        ));
    }

    #[cfg(feature = "hyperlattice")]
    if args.wants("hyperlattice") {
        for config in configs {
            if args.zoom_only {
                copy_f64_zoom_aliases(
                    &args.out_dir,
                    "hyperlattice",
                    "hyperlattice::Scalar<DefaultBackend>",
                    config,
                    &mut manifest,
                )?;
            } else {
                render_scalar_plots(
                    &args.out_dir,
                    args.size,
                    "hyperlattice",
                    "hyperlattice::Scalar<DefaultBackend>",
                    config,
                    true,
                    realistic_scalar,
                    &mut manifest,
                )?;
            }
        }
    }

    #[cfg(not(feature = "hyperlattice"))]
    if args.backend == "hyperlattice" {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "--backend hyperlattice requires the hyperlattice feature",
        ));
    }

    #[cfg(feature = "interval")]
    if args.wants("interval") {
        render_interval_plots(&args.out_dir, args.size, !args.zoom_only, &mut manifest)?;
    }

    #[cfg(not(feature = "interval"))]
    if args.backend == "interval" {
        return Err(io::Error::new(
            io::ErrorKind::InvalidInput,
            "--backend interval requires the interval feature",
        ));
    }

    let manifest_path = args.out_dir.join("manifest.txt");
    fs::write(manifest_path, manifest)?;
    if args.should_validate_gallery() {
        validate_plots_md(&args.out_dir)?;
    }
    Ok(())
}

fn render_f64_plots(
    out_dir: &Path,
    size: usize,
    config: PlotConfig,
    render_normal: bool,
    manifest: &mut String,
) -> io::Result<()> {
    let a = Point2::new(-0.85, -0.55);
    let b = Point2::new(0.9, 0.45);

    write_plot_pair(
        out_dir,
        size,
        &format!("f64_orient2d_{}.png", config.name),
        ORIENT_LINE_ZOOM,
        config.policy,
        render_normal,
        |x, y, policy| color_sign(orient2d_with_policy(&a, &b, &Point2::new(x, y), policy)),
        manifest,
        "f64 orient2d over moving third point",
    )?;

    write_plot_pair(
        out_dir,
        size,
        &format!("f64_line_side_{}.png", config.name),
        ORIENT_LINE_ZOOM,
        config.policy,
        render_normal,
        |x, y, policy| {
            color_line_side(classify_point_line_with_policy(
                &a,
                &b,
                &Point2::new(x, y),
                policy,
            ))
        },
        manifest,
        "f64 line-side classification over moving point",
    )?;

    let ca = Point2::new(-0.7, -0.45);
    let cb = Point2::new(0.75, -0.35);
    let cc = Point2::new(-0.05, 0.85);

    write_plot_pair(
        out_dir,
        size,
        &format!("f64_incircle2d_{}.png", config.name),
        INCIRCLE_ZOOM,
        config.policy,
        render_normal,
        |x, y, policy| {
            color_sign(incircle2d_with_policy(
                &ca,
                &cb,
                &cc,
                &Point2::new(x, y),
                policy,
            ))
        },
        manifest,
        "f64 incircle over moving fourth point",
    )?;

    let plane = Plane3::new(Point3::new(0.8, -0.55, 0.0), -0.05);
    write_plot_pair(
        out_dir,
        size,
        &format!("f64_explicit_plane_{}.png", config.name),
        EXPLICIT_PLANE_ZOOM,
        config.policy,
        render_normal,
        |x, y, policy| {
            color_plane_side(classify_point_plane_with_policy(
                &Point3::new(x, y, 0.0),
                &plane,
                policy,
            ))
        },
        manifest,
        "f64 explicit plane equation sampled on z=0",
    )?;

    let pa = Point3::new(-0.85, -0.7, -0.25);
    let pb = Point3::new(0.9, -0.35, 0.35);
    let pc = Point3::new(-0.35, 0.85, 0.05);
    write_plot_pair(
        out_dir,
        size,
        &format!("f64_oriented_plane_{}.png", config.name),
        ORIENTED_PLANE_ZOOM,
        config.policy,
        render_normal,
        |x, y, policy| {
            color_plane_side(classify_point_oriented_plane_with_policy(
                &pa,
                &pb,
                &pc,
                &Point3::new(x, y, 0.0),
                policy,
            ))
        },
        manifest,
        "f64 oriented plane through three points sampled on z=0",
    )?;

    let sa = Point3::new(0.82, 0.0, 0.0);
    let sb = Point3::new(-0.82, 0.0, 0.0);
    let sc = Point3::new(0.0, 0.82, 0.0);
    let sd = Point3::new(0.0, 0.0, 0.82);
    write_plot_pair(
        out_dir,
        size,
        &format!("f64_insphere3d_{}.png", config.name),
        INSPHERE_ZOOM,
        config.policy,
        render_normal,
        |x, y, policy| {
            color_sign(insphere3d_with_policy(
                &sa,
                &sb,
                &sc,
                &sd,
                &Point3::new(x, y, 0.0),
                policy,
            ))
        },
        manifest,
        "f64 insphere over moving fifth point on z=0",
    )
}

#[cfg(any(feature = "hyperreal", feature = "hyperlattice"))]
fn copy_f64_zoom_aliases(
    out_dir: &Path,
    prefix: &str,
    label: &str,
    config: PlotConfig,
    manifest: &mut String,
) -> io::Result<()> {
    let predicates = [
        ("orient2d", "orient2d over moving third point"),
        ("line_side", "line-side classification over moving point"),
        ("incircle2d", "incircle over moving fourth point"),
        ("explicit_plane", "explicit plane equation sampled on z=0"),
        (
            "oriented_plane",
            "oriented plane through three points sampled on z=0",
        ),
        ("insphere3d", "insphere over moving fifth point on z=0"),
    ];

    for (predicate, description) in predicates {
        let name = format!("{prefix}_{predicate}_{}.png", config.name);
        let zoom_name = zoom_name(&name);
        let source = out_dir.join(format!("f64_{predicate}_{}_fp_zoom.png", config.name));
        let destination = out_dir.join(&zoom_name);
        fs::copy(&source, &destination)?;

        manifest.push_str(&format!("{name}: {label} {description}\n"));
        manifest.push_str(&format!(
            "{zoom_name}: {label} {description}, floating-point zoom with span {FP_ZOOM_SPAN}\n"
        ));
    }

    Ok(())
}

#[cfg(any(feature = "hyperreal", feature = "hyperlattice"))]
#[allow(clippy::too_many_arguments)]
fn render_scalar_plots<S>(
    out_dir: &Path,
    size: usize,
    prefix: &str,
    label: &str,
    config: PlotConfig,
    render_normal: bool,
    scalar: fn(f64) -> S,
    manifest: &mut String,
) -> io::Result<()>
where
    S: BorrowedPredicateScalar,
{
    let a = Point2::new(scalar(-0.85), scalar(-0.55));
    let b = Point2::new(scalar(0.9), scalar(0.45));

    write_plot_pair(
        out_dir,
        size,
        &format!("{prefix}_orient2d_{}.png", config.name),
        ORIENT_LINE_ZOOM,
        config.policy,
        render_normal,
        |x, y, policy| {
            color_sign(orient2d_with_policy(
                &a,
                &b,
                &Point2::new(scalar(x), scalar(y)),
                policy,
            ))
        },
        manifest,
        &format!("{label} orient2d over moving third point"),
    )?;

    write_plot_pair(
        out_dir,
        size,
        &format!("{prefix}_line_side_{}.png", config.name),
        ORIENT_LINE_ZOOM,
        config.policy,
        render_normal,
        |x, y, policy| {
            color_line_side(classify_point_line_with_policy(
                &a,
                &b,
                &Point2::new(scalar(x), scalar(y)),
                policy,
            ))
        },
        manifest,
        &format!("{label} line-side classification over moving point"),
    )?;

    let ca = Point2::new(scalar(-0.7), scalar(-0.45));
    let cb = Point2::new(scalar(0.75), scalar(-0.35));
    let cc = Point2::new(scalar(-0.05), scalar(0.85));
    write_plot_pair(
        out_dir,
        size,
        &format!("{prefix}_incircle2d_{}.png", config.name),
        INCIRCLE_ZOOM,
        config.policy,
        render_normal,
        |x, y, policy| {
            color_sign(incircle2d_with_policy(
                &ca,
                &cb,
                &cc,
                &Point2::new(scalar(x), scalar(y)),
                policy,
            ))
        },
        manifest,
        &format!("{label} incircle over moving fourth point"),
    )?;

    let plane = Plane3::new(
        Point3::new(scalar(0.8), scalar(-0.55), scalar(0.0)),
        scalar(-0.05),
    );
    write_plot_pair(
        out_dir,
        size,
        &format!("{prefix}_explicit_plane_{}.png", config.name),
        EXPLICIT_PLANE_ZOOM,
        config.policy,
        render_normal,
        |x, y, policy| {
            color_plane_side(classify_point_plane_with_policy(
                &Point3::new(scalar(x), scalar(y), scalar(0.0)),
                &plane,
                policy,
            ))
        },
        manifest,
        &format!("{label} explicit plane equation sampled on z=0"),
    )?;

    let pa = Point3::new(scalar(-0.85), scalar(-0.7), scalar(-0.25));
    let pb = Point3::new(scalar(0.9), scalar(-0.35), scalar(0.35));
    let pc = Point3::new(scalar(-0.35), scalar(0.85), scalar(0.05));
    write_plot_pair(
        out_dir,
        size,
        &format!("{prefix}_oriented_plane_{}.png", config.name),
        ORIENTED_PLANE_ZOOM,
        config.policy,
        render_normal,
        |x, y, policy| {
            color_plane_side(classify_point_oriented_plane_with_policy(
                &pa,
                &pb,
                &pc,
                &Point3::new(scalar(x), scalar(y), scalar(0.0)),
                policy,
            ))
        },
        manifest,
        &format!("{label} oriented plane through three points sampled on z=0"),
    )?;

    let sa = Point3::new(scalar(0.82), scalar(0.0), scalar(0.0));
    let sb = Point3::new(scalar(-0.82), scalar(0.0), scalar(0.0));
    let sc = Point3::new(scalar(0.0), scalar(0.82), scalar(0.0));
    let sd = Point3::new(scalar(0.0), scalar(0.0), scalar(0.82));
    write_plot_pair(
        out_dir,
        size,
        &format!("{prefix}_insphere3d_{}.png", config.name),
        INSPHERE_ZOOM,
        config.policy,
        render_normal,
        |x, y, policy| {
            color_sign(insphere3d_with_policy(
                &sa,
                &sb,
                &sc,
                &sd,
                &Point3::new(scalar(x), scalar(y), scalar(0.0)),
                policy,
            ))
        },
        manifest,
        &format!("{label} insphere over moving fifth point on z=0"),
    )
}

#[cfg(feature = "interval")]
fn render_interval_plots(
    out_dir: &Path,
    size: usize,
    render_normal: bool,
    manifest: &mut String,
) -> io::Result<()> {
    let config = PlotConfig {
        name: "interval_cells_strict",
        policy: PredicatePolicy::STRICT,
    };

    let a = Point2::new(interval(-0.85, -0.85), interval(-0.55, -0.55));
    let b = Point2::new(interval(0.9, 0.9), interval(0.45, 0.45));
    write_cell_plot_pair(
        out_dir,
        size,
        "interval_orient2d_cells_strict.png",
        ORIENT_LINE_ZOOM,
        config.policy,
        render_normal,
        |x0, x1, y0, y1, policy| {
            color_sign(orient2d_with_policy(
                &a,
                &b,
                &Point2::new(interval(x0, x1), interval(y0, y1)),
                policy,
            ))
        },
        manifest,
        "inari interval orient2d using each pixel as an interval cell",
    )?;

    let ca = Point2::new(interval(-0.7, -0.7), interval(-0.45, -0.45));
    let cb = Point2::new(interval(0.75, 0.75), interval(-0.35, -0.35));
    let cc = Point2::new(interval(-0.05, -0.05), interval(0.85, 0.85));
    write_cell_plot_pair(
        out_dir,
        size,
        "interval_incircle2d_cells_strict.png",
        INCIRCLE_ZOOM,
        config.policy,
        render_normal,
        |x0, x1, y0, y1, policy| {
            color_sign(incircle2d_with_policy(
                &ca,
                &cb,
                &cc,
                &Point2::new(interval(x0, x1), interval(y0, y1)),
                policy,
            ))
        },
        manifest,
        "inari interval incircle using each pixel as an interval cell",
    )?;

    let plane = Plane3::new(
        Point3::new(
            interval(0.8, 0.8),
            interval(-0.55, -0.55),
            interval(0.0, 0.0),
        ),
        interval(-0.05, -0.05),
    );
    write_cell_plot_pair(
        out_dir,
        size,
        "interval_explicit_plane_cells_strict.png",
        EXPLICIT_PLANE_ZOOM,
        config.policy,
        render_normal,
        |x0, x1, y0, y1, policy| {
            color_plane_side(classify_point_plane_with_policy(
                &Point3::new(interval(x0, x1), interval(y0, y1), interval(0.0, 0.0)),
                &plane,
                policy,
            ))
        },
        manifest,
        "inari interval explicit plane using each pixel as an interval cell",
    )
}

#[allow(clippy::too_many_arguments)]
fn write_plot_pair(
    out_dir: &Path,
    size: usize,
    name: &str,
    zoom: ZoomView,
    normal_policy: PredicatePolicy,
    render_normal: bool,
    mut sample: impl FnMut(f64, f64, PredicatePolicy) -> [u8; 3],
    manifest: &mut String,
    description: &str,
) -> io::Result<()> {
    if render_normal {
        write_png_plot(out_dir, size, name, |ix, iy| {
            let (x, y) = pixel_center(size, ix, iy);
            sample(x, y, normal_policy)
        })?;
    }
    manifest.push_str(&format!("{name}: {description}\n"));

    if !render_normal {
        let zoom_name = zoom_name(name);
        write_png_plot(out_dir, size, &zoom_name, |ix, iy| {
            let (x, y) = pixel_center_in_view(size, ix, iy, zoom);
            sample(x, y, PredicatePolicy::APPROXIMATE)
        })?;
        manifest.push_str(&format!(
            "{zoom_name}: {description}, floating-point zoom centered at ({}, {}) with span {}\n",
            zoom.center_x, zoom.center_y, zoom.span
        ));
    }
    Ok(())
}

#[cfg(feature = "interval")]
fn write_cell_plot_pair(
    out_dir: &Path,
    size: usize,
    name: &str,
    zoom: ZoomView,
    normal_policy: PredicatePolicy,
    render_normal: bool,
    mut sample: impl FnMut(f64, f64, f64, f64, PredicatePolicy) -> [u8; 3],
    manifest: &mut String,
    description: &str,
) -> io::Result<()> {
    if render_normal {
        write_png_plot(out_dir, size, name, |ix, iy| {
            let (x0, x1, y0, y1) = pixel_cell(size, ix, iy);
            sample(x0, x1, y0, y1, normal_policy)
        })?;
    }
    manifest.push_str(&format!("{name}: {description}\n"));

    if !render_normal {
        let zoom_name = zoom_name(name);
        write_png_plot(out_dir, size, &zoom_name, |ix, iy| {
            let (x0, x1, y0, y1) = pixel_cell_in_view(size, ix, iy, zoom);
            sample(x0, x1, y0, y1, PredicatePolicy::APPROXIMATE)
        })?;
        manifest.push_str(&format!(
            "{zoom_name}: {description}, floating-point zoom centered at ({}, {}) with span {}\n",
            zoom.center_x, zoom.center_y, zoom.span
        ));
    }
    Ok(())
}

fn write_png_plot(
    out_dir: &Path,
    size: usize,
    name: &str,
    mut sample: impl FnMut(usize, usize) -> [u8; 3],
) -> io::Result<()> {
    let path = out_dir.join(name);
    let mut pixels = Vec::with_capacity(size * size * 3);
    for iy in 0..size {
        for ix in 0..size {
            pixels.extend_from_slice(&sample(ix, iy));
        }
    }
    write_png(&path, size, size, &pixels)
}

fn pixel_center(size: usize, ix: usize, iy: usize) -> (f64, f64) {
    let step = (WORLD_MAX - WORLD_MIN) / size as f64;
    let x = WORLD_MIN + (ix as f64 + 0.5) * step;
    let y = WORLD_MAX - (iy as f64 + 0.5) * step;
    (x, y)
}

fn pixel_center_in_view(size: usize, ix: usize, iy: usize, view: ZoomView) -> (f64, f64) {
    let step = view.span / size as f64;
    let x = view.center_x - view.span / 2.0 + (ix as f64 + 0.5) * step;
    let y = view.center_y + view.span / 2.0 - (iy as f64 + 0.5) * step;
    (x, y)
}

#[cfg(feature = "interval")]
fn pixel_cell(size: usize, ix: usize, iy: usize) -> (f64, f64, f64, f64) {
    let step = (WORLD_MAX - WORLD_MIN) / size as f64;
    let x0 = WORLD_MIN + ix as f64 * step;
    let x1 = x0 + step;
    let y1 = WORLD_MAX - iy as f64 * step;
    let y0 = y1 - step;
    (x0, x1, y0, y1)
}

#[cfg(feature = "interval")]
fn pixel_cell_in_view(size: usize, ix: usize, iy: usize, view: ZoomView) -> (f64, f64, f64, f64) {
    let step = view.span / size as f64;
    let x0 = view.center_x - view.span / 2.0 + ix as f64 * step;
    let x1 = x0 + step;
    let y1 = view.center_y + view.span / 2.0 - iy as f64 * step;
    let y0 = y1 - step;
    (x0, x1, y0, y1)
}

fn zoom_name(name: &str) -> String {
    let Some(stem) = name.strip_suffix(".png") else {
        return format!("{name}_fp_zoom");
    };
    format!("{stem}_fp_zoom.png")
}

fn color_sign(outcome: PredicateOutcome<Sign>) -> [u8; 3] {
    match outcome {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => color_decision(value, certainty, stage),
        PredicateOutcome::Unknown { .. } => UNKNOWN,
    }
}

fn color_line_side(outcome: PredicateOutcome<LineSide>) -> [u8; 3] {
    match outcome {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => {
            let sign = match value {
                LineSide::Right => Sign::Negative,
                LineSide::On => Sign::Zero,
                LineSide::Left => Sign::Positive,
            };
            color_decision(sign, certainty, stage)
        }
        PredicateOutcome::Unknown { .. } => UNKNOWN,
    }
}

fn color_plane_side(outcome: PredicateOutcome<PlaneSide>) -> [u8; 3] {
    match outcome {
        PredicateOutcome::Decided {
            value,
            certainty,
            stage,
        } => {
            let sign = match value {
                PlaneSide::Below => Sign::Negative,
                PlaneSide::On => Sign::Zero,
                PlaneSide::Above => Sign::Positive,
            };
            color_decision(sign, certainty, stage)
        }
        PredicateOutcome::Unknown { .. } => UNKNOWN,
    }
}

fn color_decision(sign: Sign, certainty: Certainty, stage: Escalation) -> [u8; 3] {
    let base = match sign {
        Sign::Negative => NEGATIVE,
        Sign::Zero => ZERO,
        Sign::Positive => POSITIVE,
    };

    match (certainty, stage) {
        (Certainty::Approximate, _) => blend(base, [118, 118, 118], 0.32),
        (_, Escalation::RobustFallback) => blend(base, [255, 255, 255], 0.18),
        (_, Escalation::Refined | Escalation::Exact) => blend(base, [255, 255, 255], 0.1),
        _ => base,
    }
}

#[cfg(feature = "interval")]
fn interval(inf: f64, sup: f64) -> inari::Interval {
    inari::Interval::try_from((inf, sup)).expect("valid demo interval")
}

#[cfg(feature = "hyperreal")]
fn real(value: f64) -> hyperreal::Real {
    hyperreal::Real::try_from(value).expect("valid demo real")
}

#[cfg(feature = "hyperlattice")]
fn realistic_scalar(value: f64) -> hyperlattice::Scalar<hyperlattice::DefaultBackend> {
    hyperlattice::Scalar::try_from(value).expect("valid demo scalar")
}

fn write_png(path: &Path, width: usize, height: usize, rgb: &[u8]) -> io::Result<()> {
    let mut writer = File::create(path)?;
    writer.write_all(b"\x89PNG\r\n\x1a\n")?;

    let mut ihdr = Vec::with_capacity(13);
    ihdr.extend_from_slice(&(width as u32).to_be_bytes());
    ihdr.extend_from_slice(&(height as u32).to_be_bytes());
    ihdr.extend_from_slice(&[8, 2, 0, 0, 0]);
    write_png_chunk(&mut writer, b"IHDR", &ihdr)?;

    let mut scanlines = Vec::with_capacity(height * (1 + width * 3));
    for row in 0..height {
        scanlines.push(0);
        let start = row * width * 3;
        scanlines.extend_from_slice(&rgb[start..start + width * 3]);
    }

    let mut zlib = Vec::with_capacity(scanlines.len() + scanlines.len() / 65_535 * 5 + 10);
    zlib.extend_from_slice(&[0x78, 0x01]);
    let chunk_count = scanlines.len().div_ceil(65_535);
    for (index, chunk) in scanlines.chunks(65_535).enumerate() {
        let final_block = index + 1 == chunk_count;
        zlib.push(u8::from(final_block));
        let len = chunk.len() as u16;
        zlib.extend_from_slice(&len.to_le_bytes());
        zlib.extend_from_slice(&(!len).to_le_bytes());
        zlib.extend_from_slice(chunk);
    }
    zlib.extend_from_slice(&adler32(&scanlines).to_be_bytes());
    write_png_chunk(&mut writer, b"IDAT", &zlib)?;
    write_png_chunk(&mut writer, b"IEND", &[])
}

fn write_png_chunk(writer: &mut File, kind: &[u8; 4], data: &[u8]) -> io::Result<()> {
    writer.write_all(&(data.len() as u32).to_be_bytes())?;
    writer.write_all(kind)?;
    writer.write_all(data)?;
    let mut crc_data = Vec::with_capacity(kind.len() + data.len());
    crc_data.extend_from_slice(kind);
    crc_data.extend_from_slice(data);
    writer.write_all(&crc32(&crc_data).to_be_bytes())
}

fn adler32(data: &[u8]) -> u32 {
    const MOD: u32 = 65_521;
    let mut a = 1;
    let mut b = 0;
    for byte in data {
        a = (a + u32::from(*byte)) % MOD;
        b = (b + a) % MOD;
    }
    (b << 16) | a
}

fn crc32(data: &[u8]) -> u32 {
    let mut crc = 0xffff_ffff;
    for byte in data {
        crc ^= u32::from(*byte);
        for _ in 0..8 {
            let mask = 0u32.wrapping_sub(crc & 1);
            crc = (crc >> 1) ^ (0xedb8_8320 & mask);
        }
    }
    !crc
}

fn validate_plots_md(out_dir: &Path) -> io::Result<()> {
    let plots_md = Path::new(PLOTS_MD);
    if !plots_md.exists() {
        return Ok(());
    }

    let manifest = fs::read_to_string(out_dir.join("manifest.txt"))?;
    let gallery = fs::read_to_string(plots_md)?;
    let out_dir = out_dir.to_string_lossy().replace('\\', "/");
    let expected_prefix = format!("./{out_dir}/");
    let legacy_prefix = format!("{out_dir}/");

    let expected: BTreeSet<&str> = manifest
        .lines()
        .filter_map(|line| line.split_once(':').map(|(name, _)| name))
        .filter(|name| name.ends_with(".png"))
        .collect();

    let mut missing = Vec::new();
    for name in &expected {
        let reference = format!("![{name}]({expected_prefix}{name})");
        if !gallery.contains(&reference) {
            missing.push((*name).to_string());
        }
    }

    let mut extra = Vec::new();
    let mut malformed = Vec::new();
    for image in markdown_images(&gallery) {
        let name = image
            .path
            .strip_prefix(&expected_prefix)
            .or_else(|| image.path.strip_prefix(&legacy_prefix));
        let Some(name) = name else {
            continue;
        };

        if !expected.contains(name) {
            extra.push(name.to_string());
        }

        let expected_path = format!("{expected_prefix}{name}");
        if image.alt != name || image.path != expected_path {
            malformed.push(image.raw);
        }
    }

    if missing.is_empty() && extra.is_empty() && malformed.is_empty() {
        return Ok(());
    }

    let mut message = String::from("plots.md is out of sync with the generated plot manifest");
    append_limited_list(&mut message, "missing explicit references", &missing);
    append_limited_list(&mut message, "extra references", &extra);
    append_limited_list(&mut message, "non-canonical references", &malformed);

    Err(io::Error::new(io::ErrorKind::InvalidData, message))
}

struct MarkdownImage {
    alt: String,
    path: String,
    raw: String,
}

fn markdown_images(markdown: &str) -> Vec<MarkdownImage> {
    let mut images = Vec::new();
    let mut cursor = 0;

    while let Some(start) = markdown[cursor..].find("![") {
        let start = cursor + start;
        let alt_start = start + 2;
        let Some(alt_end_offset) = markdown[alt_start..].find("](") else {
            break;
        };
        let alt_end = alt_start + alt_end_offset;
        let path_start = alt_end + 2;
        let Some(path_end_offset) = markdown[path_start..].find(')') else {
            break;
        };
        let path_end = path_start + path_end_offset;
        let raw_end = path_end + 1;

        images.push(MarkdownImage {
            alt: markdown[alt_start..alt_end].to_string(),
            path: markdown[path_start..path_end].to_string(),
            raw: markdown[start..raw_end].to_string(),
        });
        cursor = raw_end;
    }

    images
}

fn append_limited_list(message: &mut String, label: &str, items: &[String]) {
    if items.is_empty() {
        return;
    }

    message.push_str(&format!("\n{label}:"));
    for item in items.iter().take(8) {
        message.push_str(&format!("\n  - {item}"));
    }
    if items.len() > 8 {
        message.push_str(&format!("\n  - ... and {} more", items.len() - 8));
    }
}

fn blend(a: [u8; 3], b: [u8; 3], t: f64) -> [u8; 3] {
    [mix(a[0], b[0], t), mix(a[1], b[1], t), mix(a[2], b[2], t)]
}

fn mix(a: u8, b: u8, t: f64) -> u8 {
    ((a as f64 * (1.0 - t) + b as f64 * t).round()).clamp(0.0, 255.0) as u8
}

const POSITIVE: [u8; 3] = [46, 118, 220];
const NEGATIVE: [u8; 3] = [226, 98, 28];
const ZERO: [u8; 3] = [246, 246, 246];
const UNKNOWN: [u8; 3] = [18, 18, 18];

struct Args {
    out_dir: PathBuf,
    size: usize,
    backend: String,
    zoom_only: bool,
    check_gallery: bool,
}

impl Args {
    fn parse() -> io::Result<Self> {
        let mut out_dir = PathBuf::from(DEFAULT_OUT_DIR);
        let mut size = DEFAULT_SIZE;
        let mut backend = String::from("f64");
        let mut zoom_only = false;
        let mut check_gallery = false;
        let mut args = env::args().skip(1);

        while let Some(arg) = args.next() {
            match arg.as_str() {
                "--out" => {
                    let value = args.next().ok_or_else(|| {
                        io::Error::new(io::ErrorKind::InvalidInput, "--out requires a path")
                    })?;
                    out_dir = PathBuf::from(value);
                }
                "--size" => {
                    let value = args.next().ok_or_else(|| {
                        io::Error::new(io::ErrorKind::InvalidInput, "--size requires a number")
                    })?;
                    size = value.parse().map_err(|_| {
                        io::Error::new(io::ErrorKind::InvalidInput, "--size must be a number")
                    })?;
                    if size < 512 {
                        return Err(io::Error::new(
                            io::ErrorKind::InvalidInput,
                            "--size must be at least 512",
                        ));
                    }
                }
                "--backend" => {
                    backend = args.next().ok_or_else(|| {
                        io::Error::new(io::ErrorKind::InvalidInput, "--backend requires a value")
                    })?;
                    if !matches!(
                        backend.as_str(),
                        "f64" | "hyperreal" | "hyperlattice" | "interval" | "all"
                    ) {
                        return Err(io::Error::new(
                            io::ErrorKind::InvalidInput,
                            "--backend must be one of: f64, hyperreal, hyperlattice, interval, all",
                        ));
                    }
                }
                "--zoom-only" => {
                    zoom_only = true;
                }
                "--check-gallery" => {
                    check_gallery = true;
                }
                "--help" | "-h" => {
                    println!(
                        "usage: cargo run --example predicate_plots -- [--backend NAME] [--out DIR] [--size N] [--zoom-only] [--check-gallery]\n\nNAME is one of: f64, hyperreal, hyperlattice, interval, all.\nN must be at least 512. Images are written as PNG files."
                    );
                    std::process::exit(0);
                }
                _ => {
                    return Err(io::Error::new(
                        io::ErrorKind::InvalidInput,
                        format!("unknown argument: {arg}"),
                    ));
                }
            }
        }

        Ok(Self {
            out_dir,
            size,
            backend,
            zoom_only,
            check_gallery,
        })
    }

    fn should_validate_gallery(&self) -> bool {
        self.backend == "all" && !self.zoom_only && self.out_dir == Path::new(DEFAULT_OUT_DIR)
    }

    fn wants(&self, backend: &str) -> bool {
        self.backend == "all" || self.backend == backend
    }
}
