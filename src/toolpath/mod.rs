//! Toolpath generation for FDM, milling/routing (2.5D), laser & plasma cutting, and lathing
//! built on top of `csgrs` data structures (`Sketch`, `Mesh`, `Real`).
//!
//! The goal is to provide *robust, composable, and controller‑agnostic* path primitives
//! plus a couple of high‑level strategies (perimeters+infill, contour+offset pocketing,
//! kerf compensation with lead‑ins, simple lathe rough/finish passes).
//!
//! ## Highlights
//! - Pure‐Rust, no alloc-heavy geometry beyond what `csgrs` already uses
//! - Works directly with `Sketch` (2D) and, for FDM, accepts pre‑sliced layer `Sketch`es
//! - Uses `Sketch::offset/_rounded`, `Sketch::hilbert_curve`, and ring extraction
//! - Emits neutral `Toolpath` moves and optional G‑code via `gcode` submodule
//!
//! ### Feature flags
//! - Requires `csgrs` to be compiled with the `offset` feature for kerf/tool radius comp.

use core::fmt::Debug;
use crate::math_ndsp::Point3;

type Real = f64;
use crate::math_ndsp::consts::{EPSILON};
use crate::sketch::Sketch;

// ==========================
// Public types & primitives
// ==========================

/// Machine families we target.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum MachineKind {
    #[default]
    Fdm, // 3 axis, fused deposition modeling (extruding) 3D printer
    Mill,   // 3 axis, spinning non-zero width cutter
    Router, // 2.5 axis, spinning non-zero width cutter
    Laser,  // 2.5 axis, near-zero width cutter
    Plasma, // 2.5 axis, non-zero-width cutter with nonlinear shape properties needs startup and lead-in
    Lathe,  // 3 axis, spinning workpiece, fixed cutter
}

/// Unified linear/rapid move primitive. Arcs are optional; most CAM stacks linearize.
#[derive(Clone, Debug)]
pub struct PathMove {
    pub is_rapid: bool,
    pub pos: Point3<Real>,
    /// Optional extruder E (FDM) or power percentage (Laser/Plasma: 0..1)
    pub scalar: Option<Real>,
    /// Feed (mm/min) for cutting moves; `None` means keep last feed
    pub feed: Option<Real>,
    pub comment: Option<String>,
}

#[derive(Clone, Debug, Default)]
pub struct Toolpath {
    pub kind: MachineKind,
    pub moves: Vec<PathMove>,
}

impl Toolpath {
    pub const fn new(kind: MachineKind) -> Self {
        Self {
            kind,
            moves: Vec::new(),
        }
    }
    pub fn travel_to<P: Into<Point3<Real>>>(&mut self, p: P) {
        self.moves.push(PathMove {
            is_rapid: true,
            pos: p.into(),
            scalar: None,
            feed: None,
            comment: None,
        });
    }
    pub fn cut_to<P: Into<Point3<Real>>>(
        &mut self,
        p: P,
        feed: Option<Real>,
        scalar: Option<Real>,
    ) {
        self.moves.push(PathMove {
            is_rapid: false,
            pos: p.into(),
            scalar,
            feed,
            comment: None,
        });
    }
    pub fn annotate<S: Into<String>>(&mut self, s: S) {
        self.moves.push(PathMove {
            is_rapid: true,
            pos: Point3::origin(),
            scalar: None,
            feed: None,
            comment: Some(s.into()),
        });
    }
}

/// Generic tool definition (router/mill/lathe).
#[derive(Clone, Debug)]
pub struct Tool {
    /// Cutter diameter (or nozzle width for FDM if you prefer to reuse it)
    pub diameter: Real,
    /// Corner/tip radius (0 for flat end mill, >0 for bull‑nose, for kerf lead‑ins).
    pub corner_radius: Real,
}

/// FDM nozzle model.
#[derive(Clone, Debug)]
pub struct Nozzle {
    pub width: Real, // track width (path spacing)
    pub layer_height: Real,
    pub keepout_radii: Option<Vec<(Real, Real)>>, // Vec of keepout radius and height
}

/// Generic feed/power bundle.
#[derive(Clone, Debug, Default)]
pub struct Feeds {
    pub travel: Real,        // rapid-ish traverse (mm/min) used for non-cut moves
    pub xy: Real,            // planar feed (mm/min)
    pub plunge: Real,        // Z feed (mm/min) when plunging
    pub rpm: Option<Real>,   // spindle RPM if applicable
    pub power: Option<Real>, // 0..1 for laser/plasma power (cut)
    pub pierce_ms: Option<u64>, // pierce dwell for plasma/laser
}

// ==================
// 2D Ring extraction
// ==================

use geo::{CoordsIter, Geometry, MultiPolygon};

/// Iterate all rings (exterior + holes) as `Vec<(x,y)>`. Exteriors first, then holes.
fn rings_of<S: Clone + Send + Sync + Debug>(sk: &Sketch<S>) -> Vec<Vec<(Real, Real)>> {
    let mp: MultiPolygon<Real> = sk.to_multipolygon();
    let mut rings = Vec::new();
    for poly in mp.0 {
        rings.push(poly.exterior().coords_iter().map(|c| (c.x, c.y)).collect());
        for hole in poly.interiors() {
            rings.push(hole.coords_iter().map(|c| (c.x, c.y)).collect());
        }
    }
    rings
}

// ================================
// FDM (Additive) — per‑layer paths
// ================================

#[derive(Clone, Debug)]
pub struct FdmLayerCfg {
    pub nozzle: Nozzle,
    /// number of perimeters (shells)
    pub perimeters: usize,
    /// Hilbert order (≥ 3 recommended). Increase for denser, smoother infill.
    pub hilbert_order: usize,
    /// Infill density 0..1 (we achieve by spacing samples along Hilbert)
    pub infill_density: Real,
    /// e axis per XY mm (for volumetric E, pass mm³/mm and postprocessor can convert)
    pub e_per_mm: Real,
}

impl Default for FdmLayerCfg {
    fn default() -> Self {
        Self {
            nozzle: Nozzle {
                width: 0.42,
                layer_height: 0.2,
                keepout_radii: None,
            },
            perimeters: 2,
            hilbert_order: 6,
            infill_density: 0.2,
            e_per_mm: 0.05,
        }
    }
}

/// Build a single FDM layer toolpath from a **closed** 2D `Sketch` at plane `z`.
///
/// Steps:
/// 1) centerline compensation (offset by −nozzle_width/2)
/// 2) N inward perimeter offsets
/// 3) Hilbert infill clipped to remaining area
pub fn fdm_layer_from_sketch<S: Clone + Send + Sync + Debug>(
    region: &Sketch<S>,
    z: Real,
    cfg: &FdmLayerCfg,
    feeds: &Feeds,
) -> Toolpath {
    let mut tp = Toolpath::new(MachineKind::Fdm);

    // 1) Centerline compensation: toolpath corresponds to nozzle center.
    #[cfg(feature = "offset")]
    let compensated = region.offset_rounded(-0.5 * cfg.nozzle.width).renormalize();
    #[cfg(not(feature = "offset"))]
    let compensated = region.clone();

    // 2) Perimeter loops
    for k in 0..cfg.perimeters {
        let off = -(k as Real) * cfg.nozzle.width;
        #[cfg(feature = "offset")]
        let loop_sk = compensated.offset_rounded(off).renormalize();
        #[cfg(not(feature = "offset"))]
        let loop_sk = compensated.clone();
        for ring in rings_of(&loop_sk) {
            if ring.len() < 2 {
                continue;
            }
            // travel to start
            let (x0, y0) = ring[0];
            tp.travel_to(Point3::new(x0, y0, z));
            // cut around
            let mut last = (x0, y0);
            for &(x, y) in ring.iter().skip(1) {
                let seg_len = ((x - last.0).powi(2) + (y - last.1).powi(2)).sqrt();
                tp.cut_to(
                    Point3::new(x, y, z),
                    Some(feeds.xy),
                    Some(cfg.e_per_mm * seg_len),
                );
                last = (x, y);
            }
        }
    }

    // 3) Infill — build remaining area after shells and fill with Hilbert curve
    #[cfg(feature = "offset")]
    let core_area = compensated
        .offset_rounded(-(cfg.perimeters as Real) * cfg.nozzle.width)
        .renormalize();
    #[cfg(not(feature = "offset"))]
    let core_area = compensated.clone();

    // Use Hilbert path and clip against region; padding = 1/2 track width.
    let padding = 0.5 * cfg.nozzle.width;
    let curve = core_area.hilbert_curve(cfg.hilbert_order.max(3), padding);

    // We will downsample segments to achieve the target density.
    let keep_every = (1.0 / cfg.infill_density.max(0.01)).round().max(1.0) as usize;
    for geom in curve.geometry {
        if let Geometry::LineString(ls) = geom {
            let coords: Vec<_> = ls.0;
            if coords.len() < 2 {
                continue;
            }
            let (sx, sy) = (coords[0].x, coords[0].y);
            tp.travel_to(Point3::new(sx, sy, z));
            let mut last = (sx, sy);
            for (i, c) in coords.iter().enumerate().skip(1) {
                if i % keep_every != 0 {
                    continue;
                }
                let (x, y) = (c.x, c.y);
                let seg_len = ((x - last.0).powi(2) + (y - last.1).powi(2)).sqrt();
                tp.cut_to(
                    Point3::new(x, y, z),
                    Some(feeds.xy),
                    Some(cfg.e_per_mm * seg_len),
                );
                last = (x, y);
            }
        }
    }

    tp
}

// ========================================
// 2D Cutting (Laser/Plasma) — kerf & leads
// ========================================

#[derive(Clone, Debug)]
pub struct LeadInOut {
    pub length: Real,
    pub radius: Real,
}
impl Default for LeadInOut {
    fn default() -> Self {
        Self {
            length: 1.0,
            radius: 0.0,
        }
    }
}

/// Kerf compensation side for a ring.
#[derive(Clone, Copy, Debug)]
pub enum KerfSide {
    Outside,
    Inside,
}

/// Generate a kerf‑compensated contour for 2D cutters.
/// - `kerf` is full kerf width; we offset by ±kerf/2 toward Outside/Inside.
/// - For exteriors we usually cut Outside; for holes (CW) we usually cut Inside.
pub fn cut2d_contours<S: Clone + Send + Sync + Debug>(
    region: &Sketch<S>,
    z: Real,
    kerf: Real,
    side: KerfSide,
    lead: Option<LeadInOut>,
    feeds: &Feeds,
    kind: MachineKind, // Laser or Plasma
) -> Toolpath {
    let mut tp = Toolpath::new(kind);
    let sign = match side {
        KerfSide::Outside => 1.0,
        KerfSide::Inside => -1.0,
    };
    #[cfg(feature = "offset")]
    let comp = region.offset_rounded(sign * 0.5 * kerf).renormalize();
    #[cfg(not(feature = "offset"))]
    let comp = region.clone();

    let ld = lead.unwrap_or_default();

    for ring in rings_of(&comp) {
        if ring.len() < 3 {
            continue;
        }
        // Optional short tangential lead‑in
        let (x0, y0) = ring[0];
        let (x1, y1) = ring[1];
        let dir = ((x1 - x0), (y1 - y0));
        let len = (dir.0 * dir.0 + dir.1 * dir.1).sqrt();
        let ux = if len > EPSILON { dir.0 / len } else { 1.0 };
        let uy = if len > EPSILON { dir.1 / len } else { 0.0 };
        let lx = x0 - ux * ld.length;
        let ly = y0 - uy * ld.length;

        tp.travel_to(Point3::new(lx, ly, z));
        if let Some(ms) = feeds.pierce_ms {
            tp.annotate(format!("PIERCE {} ms", ms));
        }

        tp.cut_to(Point3::new(x0, y0, z), Some(feeds.xy), feeds.power); // strike in

        // Follow contour
        let mut last = (x0, y0);
        for &(x, y) in ring.iter().skip(1) {
            tp.cut_to(Point3::new(x, y, z), Some(feeds.xy), feeds.power);
            last = (x, y);
        }
        // Lead‑out
        let ox = last.0 + ux * ld.length;
        let oy = last.1 + uy * ld.length;
        tp.cut_to(Point3::new(ox, oy, z), Some(feeds.xy), feeds.power);
    }

    tp
}

// =====================================
// 2.5D Pocketing (Milling / Routing)
// =====================================

#[derive(Clone, Debug)]
pub struct PocketCfg {
    pub tool: Tool,
    pub stepover: Real,     // usually 0.4..0.8 × tool.diameter
    pub stepdown: Real,     // depth per pass (>0)
    pub depth: Real,        // total positive depth (absolute value)
    pub finish_allow: Real, // radial stock to leave for finishing
}

impl Default for PocketCfg {
    fn default() -> Self {
        Self {
            tool: Tool {
                diameter: 3.175,
                corner_radius: 0.0,
            },
            stepover: 0.6,
            stepdown: 1.5,
            depth: 3.0,
            finish_allow: 0.2,
        }
    }
}

/// Concentric‐offset pocketing with Z stepdowns; emits Outside finish pass at final depth.
pub fn pocket2d<S: Clone + Send + Sync + Debug>(
    region: &Sketch<S>,
    z_safety: Real,
    base_z: Real, // top surface Z (e.g. 0), pocket goes toward negative (base_z − depth)
    cfg: &PocketCfg,
    feeds: &Feeds,
    is_router: bool,
) -> Toolpath {
    let mut tp = Toolpath::new(if is_router {
        MachineKind::Router
    } else {
        MachineKind::Mill
    });

    let tool_r = 0.5 * cfg.tool.diameter;

    #[cfg(feature = "offset")]
    let mut area = region.offset_rounded(-tool_r).renormalize(); // centerline comp
    #[cfg(not(feature = "offset"))]
    let mut area = region.clone();

    // Z stepdowns
    let mut z_levels: Vec<Real> = Vec::new();
    let mut z = base_z - cfg.stepdown;
    let z_bottom = base_z - cfg.depth;
    while z > z_bottom + EPSILON {
        z_levels.push(z);
        z -= cfg.stepdown;
    }
    z_levels.push(z_bottom);

    // Concentric offsets inward with stepover until empty
    #[cfg(feature = "offset")]
    let radial_step = cfg.stepover.max(EPSILON) * cfg.tool.diameter; // fraction * D
    #[cfg(not(feature = "offset"))]
    let radial_step = cfg.tool.diameter * cfg.stepover;

    for &zl in &z_levels {
        let mut pass = 0usize;
        loop {
            let rings = rings_of(&area);
            if rings.is_empty() {
                break;
            }
            // Walk all rings at this Z
            for ring in rings {
                if ring.len() < 2 {
                    continue;
                }
                // entry
                let (x0, y0) = ring[0];
                tp.travel_to(Point3::new(x0, y0, z_safety));
                tp.cut_to(Point3::new(x0, y0, zl), Some(feeds.plunge), None);
                let mut last = (x0, y0);
                for &(x, y) in ring.iter().skip(1) {
                    tp.cut_to(Point3::new(x, y, zl), Some(feeds.xy), None);
                    last = (x, y);
                }
                // retract
                tp.travel_to(Point3::new(last.0, last.1, z_safety));
            }
            // Next inward area
            #[cfg(feature = "offset")]
            {
                area = area.offset_rounded(-radial_step).renormalize();
            }
            #[cfg(not(feature = "offset"))]
            {
                break;
            }
            pass += 1;
            if pass > 10_000 {
                break;
            }
        }
        // Reset area for next Z (start again from outer, less risk of isolated islands per level)
        #[cfg(feature = "offset")]
        {
            area = region.offset_rounded(-tool_r).renormalize();
        }
    }

    // Finish pass on boundary at final depth, taking finish_allow
    #[cfg(feature = "offset")]
    let finish = region
        .offset_rounded(-(tool_r - cfg.finish_allow.max(0.0)))
        .renormalize();
    #[cfg(not(feature = "offset"))]
    let finish = region.clone();

    for ring in rings_of(&finish) {
        if ring.len() < 2 {
            continue;
        }
        let (x0, y0) = ring[0];
        tp.travel_to(Point3::new(x0, y0, z_safety));
        tp.cut_to(
            Point3::new(x0, y0, z_levels.last().copied().unwrap_or(base_z)),
            Some(feeds.plunge),
            None,
        );
        for &(x, y) in ring.iter().skip(1) {
            tp.cut_to(
                Point3::new(x, y, z_levels.last().copied().unwrap_or(base_z)),
                Some(feeds.xy),
                None,
            );
        }
        tp.travel_to(Point3::new(x0, y0, z_safety));
    }

    tp
}

// ==============
// Lathe (turning)
// ==============

#[derive(Clone, Debug)]
pub struct LatheCfg {
    pub doc_radial: Real, // radial depth of cut per pass
    pub feed: Real,       // feed along Z (mm/min)
}
impl Default for LatheCfg {
    fn default() -> Self {
        Self {
            doc_radial: 0.5,
            feed: 200.0,
        }
    }
}

/// Interpret a 2D `Sketch` as a lathe profile in the X (radius) vs Y (Z‑axis) plane.
/// *Assumptions*: profile is a closed polygon whose **exterior** defines final OD.
/// We generate simple roughing passes from initial stock radius down to profile.
pub fn lathe_rough_from_profile<S: Clone + Send + Sync + Debug>(
    profile_xy: &Sketch<S>,
    z_min: Real,
    z_max: Real,
    stock_radius: Real,
    cfg: &LatheCfg,
) -> Toolpath {
    let mut tp = Toolpath::new(MachineKind::Lathe);

    // Sample target radius r(z) by intersecting horizontal lines with polygon exterior
    let mp = profile_xy.to_multipolygon();
    let poly = if mp.0.is_empty() {
        return tp;
    } else {
        mp.0[0].clone()
    };

    let dz = ((z_max - z_min).abs() / 200.0).max(0.25); // ~200 steps or 0.25mm
    let mut samples: Vec<(Real, Real)> = Vec::new(); // (z, r)
    let mut z = z_min.min(z_max);
    let z_end = z_min.max(z_max);
    while z <= z_end + EPSILON {
        // Walk edges, find intersections with y = z, choose max x (outer radius)
        let mut xs: Vec<Real> = Vec::new();
        let ext = poly.exterior();
        let coords = &ext.0;
        for w in coords.windows(2) {
            let (x1, y1) = (w[0].x, w[0].y);
            let (x2, y2) = (w[1].x, w[1].y);
            // Does segment cross the horizontal line?
            if (y1 <= z && y2 >= z) || (y2 <= z && y1 >= z) {
                let dy = y2 - y1;
                if dy.abs() < EPSILON {
                    xs.push(x1.max(x2));
                } else {
                    let t = (z - y1) / dy; // 0..1
                    if (-1e-6..=1.0 + 1e-6).contains(&t) {
                        let x = x1 + t * (x2 - x1);
                        xs.push(x);
                    }
                }
            }
        }
        if let Some(max_x) = xs
            .into_iter()
            .fold(None, |acc: Option<Real>, v| Some(acc.map_or(v, |a| a.max(v))))
        {
            samples.push((z, max_x.max(0.0)));
        }
        z += dz;
    }

    // Generate radial passes: start from stock_radius and step down by doc_radial toward r(z)
    if samples.is_empty() {
        return tp;
    }
    let mut current_r = stock_radius;
    while current_r > 0.0 {
        // One roughing sweep along Z at constant max(current_r, r(z))
        let mut first = true;
        for &(zz, r_target) in &samples {
            let r_path = current_r.max(r_target);
            let x = r_path; // X = radius (diameter/2). Controller may expect diameter mode; keep radius here.
            if first {
                tp.travel_to(Point3::new(x, zz, 0.0));
                first = false;
            } else {
                tp.cut_to(Point3::new(x, zz, 0.0), Some(cfg.feed), None);
            }
        }
        // Return rapid
        if let Some(&(zz, r_target)) = samples.first() {
            tp.travel_to(Point3::new(current_r.max(r_target), zz, 0.0));
        }
        // Next radial step
        current_r -= cfg.doc_radial;
        // Stop when next pass would be within small tolerance from final profile everywhere
        let worst_gap = samples
            .iter()
            .map(|&(_, r)| (current_r - r).max(0.0))
            .fold(0.0, Real::max);
        if worst_gap < 0.05 {
            break;
        }
    }

    tp
}

// =================
// Simple G‑code I/O
// =================

pub mod gcode {
    //! Minimal G‑code writer for the neutral `Toolpath` above.
    use super::*;

    #[derive(Clone, Debug, Default)]
    pub struct Post {
        pub absolute_e: bool, // FDM E in absolute (M82) or relative (M83)
        pub units_mm: bool,   // G21 (true) / G20 (false)
        pub z_safe: Real,     // default travel Z for routers/mills
    }

    impl Post {
        pub fn write(&self, tp: &Toolpath) -> String {
            let mut out = String::new();
            if self.units_mm {
                out.push_str("G21\n");
            } else {
                out.push_str("G20\n");
            }
            if tp.kind == MachineKind::Fdm {
                out.push_str(if self.absolute_e { "M82\n" } else { "M83\n" })
            }
            let mut last_feed: Option<Real> = None;
            let mut e_acc: Real = 0.0;
            for mv in &tp.moves {
                if let Some(ref c) = mv.comment {
                    out.push_str(&format!("; {}\n", c));
                    continue;
                }
                let (x, y, z) = (mv.pos.x, mv.pos.y, mv.pos.z);
                let f = mv.feed.or(last_feed);
                match tp.kind {
                    MachineKind::Fdm => {
                        if mv.is_rapid {
                            out.push_str(&format!("G0 X{:.4} Y{:.4} Z{:.4}\n", x, y, z));
                        } else {
                            let e = mv.scalar.unwrap_or(0.0);
                            e_acc += e;
                            if self.absolute_e {
                                out.push_str(&format!(
                                    "G1 X{:.4} Y{:.4} Z{:.4} E{:.5}",
                                    x, y, z, e_acc
                                ));
                            } else {
                                out.push_str(&format!(
                                    "G1 X{:.4} Y{:.4} Z{:.4} E{:.5}",
                                    x, y, z, e
                                ));
                            }
                            if let Some(ff) = f {
                                out.push_str(&format!(" F{:.1}", ff));
                            }
                            out.push('\n');
                        }
                    },
                    MachineKind::Laser | MachineKind::Plasma => {
                        if mv.is_rapid {
                            out.push_str(&format!("G0 X{:.4} Y{:.4} Z{:.4}\n", x, y, z));
                        } else {
                            if let Some(p) = mv.scalar {
                                out.push_str(&format!(
                                    "M3 S{:.3}\n",
                                    (p * 1000.0).clamp(0.0, 1000.0)
                                ));
                            }
                            out.push_str(&format!("G1 X{:.4} Y{:.4} Z{:.4}", x, y, z));
                            if let Some(ff) = f {
                                out.push_str(&format!(" F{:.1}", ff));
                            }
                            out.push_str("\nM5\n");
                        }
                    },
                    _ => {
                        // Mill / Router / Lathe
                        if mv.is_rapid {
                            out.push_str(&format!("G0 X{:.4} Y{:.4} Z{:.4}\n", x, y, z));
                        } else {
                            out.push_str(&format!("G1 X{:.4} Y{:.4} Z{:.4}", x, y, z));
                            if let Some(ff) = f {
                                out.push_str(&format!(" F{:.1}", ff));
                            }
                            out.push('\n');
                        }
                    },
                }
                last_feed = f;
            }
            out
        }
    }
}

// ======================
// Tiny conveniences / API
// ======================

/// Convenience: Convert a `Sketch` that represents a single closed polygon (no holes)
/// into a planar travel+cut path at a constant Z.
pub fn contour_only<S: Clone + Send + Sync + Debug>(
    sk: &Sketch<S>,
    z: Real,
    feed: Real,
    kind: MachineKind,
) -> Toolpath {
    let mut tp = Toolpath::new(kind);
    for ring in rings_of(sk) {
        if ring.len() < 2 {
            continue;
        }
        let (x0, y0) = ring[0];
        tp.travel_to(Point3::new(x0, y0, z));
        for &(x, y) in ring.iter().skip(1) {
            tp.cut_to(Point3::new(x, y, z), Some(feed), None);
        }
    }
    tp
}
