use crate::csg::CSG;
use crate::float_types::Real;
use geo::{
    Area, Geometry, GeometryCollection, LineString, Orient,
    Polygon as GeoPolygon, orient::Direction,
};
use std::fmt::Debug;
use ttf_parser::OutlineBuilder;
use ttf_utils::Outline;

// For flattening curves, how many segments per quad/cubic
const CURVE_STEPS: usize = 8;

impl<S: Clone + Debug> CSG<S>
where S: Clone + Send + Sync {
    /// Create **2D text** (outlines only) in the XY plane using ttf-utils + ttf-parser.
    ///
    /// Each glyph’s closed contours become one or more `Polygon`s (with holes if needed),
    /// and any open contours become `LineString`s.
    ///
    /// # Arguments
    /// - `text`: the text string (no multiline logic here)
    /// - `font_data`: raw bytes of a TTF file
    /// - `scale`: a uniform scale factor for glyphs
    /// - `metadata`: optional metadata for the resulting `CSG`
    ///
    /// # Returns
    /// A `CSG` whose `geometry` contains:
    /// - One or more `Polygon`s for each glyph,
    /// - A set of `LineString`s for any open contours (rare in standard fonts),
    /// all positioned in the XY plane at z=0.
    pub fn text(text: &str, font_data: &[u8], scale: Real, metadata: Option<S>) -> Self {
        // 1) Parse the TTF font
        let face = match ttf_parser::Face::parse(font_data, 0) {
            Ok(f) => f,
            Err(_) => {
                // If the font fails to parse, return an empty 2D CSG
                return CSG::new();
            }
        };

        // 1 font unit, 2048 font units / em, scale points / em, 0.352777 points / mm
        let font_scale = 1.0 / 2048.0 * scale * 0.3527777;

        // 2) We'll collect all glyph geometry into one GeometryCollection
        let mut geo_coll = GeometryCollection::default();

        // 3) A simple "pen" cursor for horizontal text layout
        let mut cursor_x = 0.0 as Real;

        for ch in text.chars() {
            // Skip control chars:
            if ch.is_control() {
                continue;
            }

            // Find glyph index in the font
            if let Some(gid) = face.glyph_index(ch) {
                // Extract the glyph outline (if any)
                if let Some(outline) = Outline::new(&face, gid) {
                    // Flatten the outline into line segments
                    let mut collector = OutlineFlattener::new(font_scale as Real, cursor_x as Real, 0.0);
                    outline.emit(&mut collector);

                    // Now `collector.contours` holds closed subpaths,
                    // and `collector.open_contours` holds open polylines.

                    // -------------------------
                    // Handle all CLOSED subpaths (which might be outer shapes or holes):
                    // -------------------------
                    if !collector.contours.is_empty() {
                        // We can have multiple outer loops and multiple inner loops (holes).
                        let mut outer_rings = Vec::new();
                        let mut hole_rings = Vec::new();

                        for closed_pts in collector.contours {
                            if closed_pts.len() < 3 {
                                continue; // degenerate
                            }

                            let ring = LineString::from(closed_pts);

                            // We need to measure signed area.  The `signed_area` method works on a Polygon,
                            // so construct a temporary single-ring polygon:
                            let tmp_poly = GeoPolygon::new(ring.clone(), vec![]);
                            let area = tmp_poly.signed_area();

                            // ttf files store outer loops as CW and inner loops as CCW
                            if area < 0.0 {
                                // This is an outer ring
                                outer_rings.push(ring);
                            } else {
                                // This is a hole ring
                                hole_rings.push(ring);
                            }
                        }

                        // Typically, a TrueType glyph has exactly one outer ring and 0+ holes.
                        // But in some tricky glyphs, you might see multiple separate outer rings.
                        // We'll create one Polygon for the first outer ring with all holes,
                        // then if there are additional outer rings, each becomes its own separate Polygon.
                        if !outer_rings.is_empty() {
                            let first_outer = outer_rings.remove(0);

                            // The “primary” polygon: first outer + all holes
                            let polygon_2d = GeoPolygon::new(first_outer, hole_rings);
                            let oriented = polygon_2d.orient(Direction::Default);
                            geo_coll.0.push(Geometry::Polygon(oriented));

                            // If there are leftover outer rings, push them each as a separate polygon (no holes):
                            // todo: test bounding boxes and sort holes appropriately
                            for extra_outer in outer_rings {
                                let poly_2d = GeoPolygon::new(extra_outer, vec![]);
                                let oriented = poly_2d.orient(Direction::Default);
                                geo_coll.0.push(Geometry::Polygon(oriented));
                            }
                        }
                    }

                    // -------------------------
                    // Handle all OPEN subpaths => store as LineStrings:
                    // -------------------------
                    for open_pts in collector.open_contours {
                        if open_pts.len() >= 2 {
                            geo_coll.0.push(Geometry::LineString(LineString::from(open_pts)));
                        }
                    }

                    // Finally, advance our pen by the glyph's bounding-box width
                    let bbox = outline.bbox();
                    let glyph_width = bbox.width() as Real * font_scale;
                    cursor_x += glyph_width;
                } else {
                    // If there's no outline (e.g., space), just move a bit
                    cursor_x += font_scale as Real * 0.3;
                }
            } else {
                // Missing glyph => small blank advance
                cursor_x += font_scale as Real * 0.3;
            }
        }

        // Build a 2D CSG from the collected geometry
        CSG::from_geo(geo_coll, metadata)
    }
}

/// A helper that implements `ttf_parser::OutlineBuilder`.
/// It receives MoveTo/LineTo/QuadTo/CurveTo calls from `outline.emit(self)`.
/// We flatten curves and accumulate polylines.
///
/// - Whenever `close()` occurs, we finalize the current subpath as a closed polygon (`contours`).
/// - If we start a new MoveTo while the old subpath is open, that old subpath is treated as open (`open_contours`).
struct OutlineFlattener {
    // scale + offset
    scale: Real,
    offset_x: Real,
    offset_y: Real,

    // We gather shapes: each "subpath" can be closed or open
    contours: Vec<Vec<(Real, Real)>>,      // closed polygons
    open_contours: Vec<Vec<(Real, Real)>>, // open polylines

    current: Vec<(Real, Real)>, // points for the subpath
    last_pt: (Real, Real),      // current "cursor" in flattening
    subpath_open: bool,
}

impl OutlineFlattener {
    const fn new(scale: Real, offset_x: Real, offset_y: Real) -> Self {
        Self {
            scale,
            offset_x,
            offset_y,
            contours: Vec::new(),
            open_contours: Vec::new(),
            current: Vec::new(),
            last_pt: (0.0, 0.0),
            subpath_open: false,
        }
    }

    /// Helper: transform TTF coordinates => final (x,y)
    #[inline]
    fn tx(&self, x: f32, y: f32) -> (Real, Real) {
        let sx = x as Real * self.scale + self.offset_x;
        let sy = y as Real * self.scale + self.offset_y;
        (sx, sy)
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
    fn _finish_open_subpath(&mut self) {
        if self.subpath_open && !self.current.is_empty() {
            self.open_contours.push(self.current.clone());
        }
        self.current.clear();
        self.subpath_open = false;
    }

    /// Flatten a line from `last_pt` to `(x,y)`.
    fn line_to_impl(&mut self, x: f32, y: f32) {
        let (xx, yy) = self.tx(x, y);
        self.current.push((xx, yy));
        self.last_pt = (xx, yy);
    }

    /// Flatten a quadratic Bézier from last_pt -> (x1,y1) -> (x2,y2)
    fn quad_to_impl(&mut self, x1: f32, y1: f32, x2: f32, y2: f32) {
        let steps = CURVE_STEPS;
        let (px0, py0) = self.last_pt;
        let (px1, py1) = self.tx(x1, y1);
        let (px2, py2) = self.tx(x2, y2);

        // B(t) = (1 - t)^2 * p0 + 2(1 - t)t * cp + t^2 * p2
        for i in 1..=steps {
            let t = i as Real / steps as Real;
            let mt = 1.0 - t;
            let bx = mt*mt*px0 + 2.0*mt*t*px1 + t*t*px2;
            let by = mt*mt*py0 + 2.0*mt*t*py1 + t*t*py2;
            self.current.push((bx, by));
        }
        self.last_pt = (px2, py2);
    }

    /// Flatten a cubic Bézier from last_pt -> (x1,y1) -> (x2,y2) -> (x3,y3)
    fn curve_to_impl(&mut self, x1: f32, y1: f32, x2: f32, y2: f32, x3: f32, y3: f32) {
        let steps = CURVE_STEPS;
        let (px0, py0) = self.last_pt;
        let (cx1, cy1) = self.tx(x1, y1);
        let (cx2, cy2) = self.tx(x2, y2);
        let (px3, py3) = self.tx(x3, y3);

        // B(t) = (1-t)^3 p0 + 3(1-t)^2 t c1 + 3(1-t) t^2 c2 + t^3 p3
        for i in 1..=steps {
            let t = i as Real / steps as Real;
            let mt = 1.0 - t;
            let mt2 = mt*mt;
            let t2  = t*t;
            let bx = mt2*mt*px0
                + 3.0*mt2*t*cx1
                + 3.0*mt*t2*cx2
                + t2*t*px3;
            let by = mt2*mt*py0
                + 3.0*mt2*t*cy1
                + 3.0*mt*t2*cy2
                + t2*t*py3;
            self.current.push((bx, by));
        }
        self.last_pt = (px3, py3);
    }

    /// Called when `close()` is invoked => store as a closed polygon.
    fn close_impl(&mut self) {
        // We have a subpath that should be closed => replicate first point as last if needed.
        let n = self.current.len();
        if n > 2 {
            // If the last point != the first, close it.
            let first = self.current[0];
            let last = self.current[n - 1];
            if (first.0 - last.0).abs() > Real::EPSILON || (first.1 - last.1).abs() > Real::EPSILON {
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
