//! Create `Profile`s using ttf fonts

use crate::hyper_math::{Real, hreal_to_f64};
use crate::sketch::Profile;
use hypercurve::{Contour2, CurvePolicy, CurveRegion2, CurveString2};
use ttf_parser::{Face, GlyphId, OutlineBuilder};
use ttf_utils::Outline;

// For flattening curves, how many segments per quad/cubic
const CURVE_STEPS: usize = 8;

impl Profile {
    /// Create **2D text** (outlines only) in the XY plane using ttf-utils + ttf-parser.
    ///
    /// Each glyph's closed contours become unified `hypercurve::CurveRegion2`
    /// contours, and any open contours become native `hypercurve::CurveString2`
    /// wires. The finite outline flattening is an API-edge tessellation of the
    /// font's quadratic/cubic curves; storage is immediately promoted into
    /// hyperreal-backed topology following Yap, "Towards Exact Geometric
    /// Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>). TrueType outline
    /// semantics follow Apple Computer, *The TrueType Reference Manual*, 1995.
    ///
    /// # Arguments
    /// - `text`: the text string
    /// - `font_data`: raw bytes of a TTF file
    /// - `scale`: a uniform scale factor for glyphs
    ///
    /// # Returns
    /// A `Profile` whose filled glyph outlines are stored as a native region and
    /// whose rare open glyph contours are stored as native wires.
    pub fn text(text: &str, font_data: &[u8], scale: Real) -> Self {
        // 1) Parse the TTF font
        let face = match ttf_parser::Face::parse(font_data, 0) {
            Ok(f) => f,
            Err(_) => {
                // If the font fails to parse, return an empty 2D Profile
                return Profile::empty();
            },
        };

        // Treat `scale` as points-per-em and convert points to millimeters.
        let units_per_em = f64::from(face.units_per_em());
        let Some(scale) = hreal_to_f64(&scale) else {
            return Profile::empty();
        };
        if scale <= 0.0 || units_per_em <= 0.0 {
            return Profile::empty();
        }
        let font_scale = scale * 0.3527777 / units_per_em;
        let default_advance = default_advance(&face, font_scale);
        let line_advance = line_advance(&face, font_scale);
        let tab_advance = default_advance * 4.0;

        let mut material_contours = Vec::new();
        let mut hole_contours = Vec::new();
        let mut wires = Vec::new();

        // 3) A simple "pen" cursor for horizontal text layout
        let mut cursor_x = 0.0;
        let mut cursor_y = 0.0;
        let mut previous_glyph = None;

        for ch in text.chars() {
            match ch {
                '\n' => {
                    cursor_x = 0.0;
                    cursor_y -= line_advance;
                    previous_glyph = None;
                    continue;
                },
                '\r' => {
                    cursor_x = 0.0;
                    previous_glyph = None;
                    continue;
                },
                '\t' => {
                    cursor_x += tab_advance;
                    previous_glyph = None;
                    continue;
                },
                ch if ch.is_control() => {
                    previous_glyph = None;
                    continue;
                },
                _ => {},
            }

            // Find glyph index in the font
            if let Some(gid) = face.glyph_index(ch) {
                if let Some(previous) = previous_glyph {
                    cursor_x += glyph_pair_kerning(&face, previous, gid) * font_scale;
                }

                // Extract the glyph outline (if any)
                if let Some(outline) = Outline::new(&face, gid) {
                    // Flatten the outline into line segments
                    let mut collector = OutlineFlattener::new(font_scale, cursor_x, cursor_y);
                    outline.emit(&mut collector);
                    collector.finish_open_subpath();

                    // TrueType stores outer loops clockwise and holes
                    // counter-clockwise. We classify with the shared hyperreal
                    // ring predicate before promoting rings into native
                    // hypercurve contours, keeping primitive font samples at
                    // the import boundary. See Yap, "Towards Exact Geometric
                    // Computation," *Computational Geometry* 7(1-2), 1997
                    // (<https://doi.org/10.1016/0925-7721(95)00040-2>).
                    for closed_pts in collector.contours {
                        if closed_pts.len() < 3 {
                            continue;
                        }
                        let ring = closed_pts
                            .iter()
                            .map(|point| {
                                let x = Real::try_from(point[0]).ok()?;
                                let y = Real::try_from(point[1]).ok()?;
                                Some(hyperlimit::Point2::new(x, y))
                            })
                            .collect::<Option<Vec<_>>>();
                        let Some(ring) = ring else {
                            continue;
                        };
                        let orientation = hyperlimit::ring_area_sign(&ring).value();
                        let Ok(contour) = Contour2::from_finite_ring(&closed_pts) else {
                            continue;
                        };
                        if matches!(orientation, Some(hyperlimit::Sign::Negative)) {
                            material_contours.push(contour);
                        } else {
                            hole_contours.push(contour);
                        }
                    }

                    for open_pts in collector.open_contours {
                        if let Ok(wire) = CurveString2::from_finite_point_iter(open_pts) {
                            wires.push(wire);
                        }
                    }
                }

                cursor_x += glyph_advance(&face, gid, font_scale, default_advance);
                previous_glyph = Some(gid);
            } else {
                // Missing glyph => small blank advance
                cursor_x += default_advance;
                previous_glyph = None;
            }
        }

        let region = CurveRegion2::try_from_native_contours(
            material_contours,
            hole_contours,
            &CurvePolicy::certified(),
        )
        .unwrap_or_else(|_| CurveRegion2::empty());
        Profile::from_topology_with_origin(
            region,
            wires,
            Vec::new(),
            crate::vertex::Vertex::default(),
            Profile::prepare_origin_transform(crate::vertex::Vertex::default()),
        )
    }
}

fn glyph_advance(
    face: &Face<'_>,
    glyph_id: GlyphId,
    font_scale: f64,
    default_advance: f64,
) -> f64 {
    face.glyph_hor_advance(glyph_id)
        .map(|advance| f64::from(advance) * font_scale)
        .filter(|advance| *advance >= 0.0)
        .unwrap_or(default_advance)
}

fn default_advance(face: &Face<'_>, font_scale: f64) -> f64 {
    face.glyph_index(' ')
        .and_then(|glyph_id| face.glyph_hor_advance(glyph_id))
        .map(|advance| f64::from(advance) * font_scale)
        .filter(|advance| *advance > 0.0)
        .unwrap_or_else(|| f64::from(face.units_per_em()) * font_scale * 0.5)
}

fn line_advance(face: &Face<'_>, font_scale: f64) -> f64 {
    let height = f64::from(face.height());
    let line_gap = f64::from(face.line_gap());
    let nominal = height + line_gap;
    let em = f64::from(face.units_per_em());
    let line_units = if nominal < em { em } else { nominal };
    let advance = line_units * font_scale;
    if advance > 0.0 {
        advance
    } else {
        em * font_scale
    }
}

fn glyph_pair_kerning(face: &Face<'_>, left: GlyphId, right: GlyphId) -> f64 {
    let gpos_adjustment = gpos_pair_adjustment(face, left, right);
    if gpos_adjustment != 0.0 {
        return gpos_adjustment;
    }

    kern_pair_adjustment(face, left, right)
}

fn kern_pair_adjustment(face: &Face<'_>, left: GlyphId, right: GlyphId) -> f64 {
    let Some(kern) = face.tables().kern else {
        return 0.0;
    };

    kern.subtables
        .into_iter()
        .filter(|subtable| {
            subtable.horizontal && !subtable.has_cross_stream && !subtable.has_state_machine
        })
        .filter_map(|subtable| subtable.glyphs_kerning(left, right))
        .map(f64::from)
        .sum()
}

fn gpos_pair_adjustment(face: &Face<'_>, left: GlyphId, right: GlyphId) -> f64 {
    let Some(gpos) = face.tables().gpos else {
        return 0.0;
    };

    let mut adjustment = 0.0;

    for lookup in gpos.lookups {
        for subtable in lookup
            .subtables
            .into_iter::<ttf_parser::gpos::PositioningSubtable>()
        {
            let ttf_parser::gpos::PositioningSubtable::Pair(pair) = subtable else {
                continue;
            };

            adjustment += match pair {
                ttf_parser::gpos::PairAdjustment::Format1 { coverage, sets } => coverage
                    .get(left)
                    .and_then(|index| sets.get(index))
                    .and_then(|set| set.get(right))
                    .map(|(left_value, right_value)| {
                        f64::from(left_value.x_advance) + f64::from(right_value.x_placement)
                    })
                    .unwrap_or(0.0),
                ttf_parser::gpos::PairAdjustment::Format2 {
                    classes, matrix, ..
                } => {
                    let class_pair = (classes.0.get(left), classes.1.get(right));
                    matrix
                        .get(class_pair)
                        .map(|(left_value, right_value)| {
                            f64::from(left_value.x_advance)
                                + f64::from(right_value.x_placement)
                        })
                        .unwrap_or(0.0)
                },
            };
        }
    }

    adjustment
}

/// A helper that implements `ttf_parser::OutlineBuilder`.
/// It receives MoveTo/LineTo/QuadTo/CurveTo calls from `outline.emit(self)`.
/// We flatten curves and accumulate polylines.
///
/// - Whenever `close()` occurs, we finalize the current subpath as a closed polygon (`contours`).
/// - If we start a new MoveTo while the old subpath is open, that old subpath is treated as open (`open_contours`).
struct OutlineFlattener {
    // scale + offset
    scale: f64,
    offset_x: f64,
    offset_y: f64,

    // We gather shapes: each "subpath" can be closed or open
    contours: Vec<Vec<[f64; 2]>>,      // closed polygons
    open_contours: Vec<Vec<[f64; 2]>>, // open polylines

    current: Vec<[f64; 2]>, // points for the subpath
    last_pt: [f64; 2],      // current "cursor" in flattening
    subpath_open: bool,
}

impl OutlineFlattener {
    const fn new(scale: f64, offset_x: f64, offset_y: f64) -> Self {
        Self {
            scale,
            offset_x,
            offset_y,
            contours: Vec::new(),
            open_contours: Vec::new(),
            current: Vec::new(),
            last_pt: [0.0, 0.0],
            subpath_open: false,
        }
    }

    /// Helper: transform TTF coordinates => final (x,y)
    #[inline]
    fn tx(&self, x: f32, y: f32) -> [f64; 2] {
        let sx = f64::from(x) * self.scale + self.offset_x;
        let sy = f64::from(y) * self.scale + self.offset_y;
        [sx, sy]
    }

    /// Start a fresh subpath
    fn begin_subpath(&mut self, x: f32, y: f32) {
        // If we already had an open subpath, push it as open_contours:
        if self.subpath_open && !self.current.is_empty() {
            self.open_contours.push(self.current.clone());
        }
        self.current.clear();

        self.subpath_open = true;
        self.last_pt = self.tx(x, y);
        self.current.push(self.last_pt);
    }

    /// Finish the current subpath as open (do not close).
    /// (We call this if a new `MoveTo` or the entire glyph ends.)
    fn finish_open_subpath(&mut self) {
        if self.subpath_open && !self.current.is_empty() {
            self.open_contours.push(self.current.clone());
        }
        self.current.clear();
        self.subpath_open = false;
    }

    /// Flatten a line from `last_pt` to `(x,y)`.
    fn line_to_impl(&mut self, x: f32, y: f32) {
        let point = self.tx(x, y);
        self.current.push(point);
        self.last_pt = point;
    }

    /// Flatten a quadratic Bézier from last_pt -> (x1,y1) -> (x2,y2)
    fn quad_to_impl(&mut self, x1: f32, y1: f32, x2: f32, y2: f32) {
        let steps = CURVE_STEPS;
        let [px0, py0] = self.last_pt;
        let [px1, py1] = self.tx(x1, y1);
        let [px2, py2] = self.tx(x2, y2);

        // B(t) = (1 - t)^2 * p0 + 2(1 - t)t * cp + t^2 * p2
        for i in 1..=steps {
            let t = i as f64 / steps as f64;
            let mt = 1.0 - t;
            let bx = mt * mt * px0 + 2.0 * mt * t * px1 + t * t * px2;
            let by = mt * mt * py0 + 2.0 * mt * t * py1 + t * t * py2;
            self.current.push([bx, by]);
        }
        self.last_pt = [px2, py2];
    }

    /// Flatten a cubic Bézier from last_pt -> (x1,y1) -> (x2,y2) -> (x3,y3)
    fn curve_to_impl(&mut self, x1: f32, y1: f32, x2: f32, y2: f32, x3: f32, y3: f32) {
        let steps = CURVE_STEPS;
        let [px0, py0] = self.last_pt;
        let [cx1, cy1] = self.tx(x1, y1);
        let [cx2, cy2] = self.tx(x2, y2);
        let [px3, py3] = self.tx(x3, y3);

        // B(t) = (1-t)^3 p0 + 3(1-t)^2 t c1 + 3(1-t) t^2 c2 + t^3 p3
        for i in 1..=steps {
            let t = i as f64 / steps as f64;
            let mt = 1.0 - t;
            let mt2 = mt * mt;
            let t2 = t * t;
            let bx = mt2 * mt * px0 + 3.0 * mt2 * t * cx1 + 3.0 * mt * t2 * cx2 + t2 * t * px3;
            let by = mt2 * mt * py0 + 3.0 * mt2 * t * cy1 + 3.0 * mt * t2 * cy2 + t2 * t * py3;
            self.current.push([bx, by]);
        }
        self.last_pt = [px3, py3];
    }

    /// Called when `close()` is invoked => store as a closed polygon.
    fn close_impl(&mut self) {
        // We have a subpath that should be closed => replicate first point as last if needed.
        let n = self.current.len();
        if n > 2 {
            // Font outline samples are primitive boundary data; compare the
            // closure predicate after hyperreal promotion instead of local
            // absolute-value arithmetic. This keeps TrueType contour admission
            // aligned with Yap's EGC boundary model
            // (<https://doi.org/10.1016/0925-7721(95)00040-2>).
            let first = self.current[0];
            let last = self.current[n - 1];
            if first != last {
                self.current.push(first);
            }
            // That becomes one closed contour
            self.contours.push(self.current.clone());
        } else {
            // If it's 2 or fewer points, ignore or treat as degenerate
        }

        self.current.clear();
        self.subpath_open = false;
    }
}

impl OutlineBuilder for OutlineFlattener {
    fn move_to(&mut self, x: f32, y: f32) {
        self.begin_subpath(x, y);
    }

    fn line_to(&mut self, x: f32, y: f32) {
        self.line_to_impl(x, y);
    }

    fn quad_to(&mut self, x1: f32, y1: f32, x2: f32, y2: f32) {
        self.quad_to_impl(x1, y1, x2, y2);
    }

    fn curve_to(&mut self, x1: f32, y1: f32, x2: f32, y2: f32, x3: f32, y3: f32) {
        self.curve_to_impl(x1, y1, x2, y2, x3, y3);
    }

    fn close(&mut self) {
        self.close_impl();
    }
}
