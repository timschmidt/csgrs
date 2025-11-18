//! Create `Sketch`s using single stroke Hershey fonts

use crate::float_types::Real;
use crate::sketch::Sketch;
use geo::{Geometry, GeometryCollection, LineString, coord};
use hershey::{Font, Glyph as HersheyGlyph, Vector as HersheyVector};
use std::fmt::Debug;
use std::sync::OnceLock;

impl<S: Clone + Debug + Send + Sync> Sketch<S> {
    /// Creates **2D line-stroke text** in the XY plane using a Hershey font.
    ///
    /// Each glyph’s strokes become one or more `LineString<Real>` entries in `geometry`.
    /// If you need them filled or thickened, you can later offset or extrude these lines.
    ///
    /// # Parameters
    /// - `text`: The text to render
    /// - `font`: The Hershey font (e.g., `hershey::fonts::GOTHIC_ENG_SANS`)
    /// - `size`: Scale factor for glyphs
    /// - `metadata`: Optional user data to store in the resulting Sketch
    ///
    /// # Returns
    /// A new `Sketch` where each glyph stroke is a `Geometry::LineString` in `geometry`.
    pub fn from_hershey(
        text: &str,
        font: &Font,
        size: Real,
        metadata: Option<S>,
    ) -> Sketch<S> {
        let mut all_strokes = Vec::new();
        let mut cursor_x: Real = 0.0;

        for ch in text.chars() {
            // Skip control chars or spaces as needed
            if ch.is_control() {
                continue;
            }

            // Attempt to find a glyph in this font
            match font.glyph(ch) {
                Ok(glyph) => {
                    // Convert the Hershey lines to geo::LineString objects
                    let glyph_width = (glyph.max_x - glyph.min_x) as Real;
                    let strokes = build_hershey_glyph_lines(&glyph, size, cursor_x, 0.0);

                    // Collect them
                    all_strokes.extend(strokes);

                    // Advance the pen in X
                    cursor_x += glyph_width * size * 0.8;
                },
                Err(_) => {
                    // Missing glyph => skip or just advance
                    cursor_x += 6.0 * size;
                },
            }
        }

        // Insert each stroke as a separate LineString in the geometry
        let mut geo_coll = GeometryCollection::default();
        for line_str in all_strokes {
            geo_coll.0.push(Geometry::LineString(line_str));
        }

        // Return a new Sketch that has no 3D polygons, but has these lines in geometry.
        Sketch {
            geometry: geo_coll,
            bounding_box: OnceLock::new(),
            metadata,
        }
    }
}

/// Helper for building open polygons from a single Hershey `Glyph`.
fn build_hershey_glyph_lines(
    glyph: &HersheyGlyph,
    scale: Real,
    offset_x: Real,
    offset_y: Real,
) -> Vec<geo::LineString<Real>> {
    let mut strokes = Vec::new();

    // We'll accumulate each stroke’s points in `current_coords`,
    // resetting whenever Hershey issues a "MoveTo"
    let mut current_coords = Vec::new();

    for vector_cmd in &glyph.vectors {
        match vector_cmd {
            HersheyVector::MoveTo { x, y } => {
                // If we already had 2+ points, that stroke is complete:
                if current_coords.len() >= 2 {
                    strokes.push(LineString::from(current_coords));
                }
                // Start a new stroke
                current_coords = Vec::new();
                let px = offset_x + (*x as Real) * scale;
                let py = offset_y + (*y as Real) * scale;
                current_coords.push(coord! { x: px, y: py });
            },
            HersheyVector::LineTo { x, y } => {
                let px = offset_x + (*x as Real) * scale;
                let py = offset_y + (*y as Real) * scale;
                current_coords.push(coord! { x: px, y: py });
            },
        }
    }

    // End-of-glyph: if our final stroke has 2+ points, convert to a line string
    if current_coords.len() >= 2 {
        strokes.push(LineString::from(current_coords));
    }

    strokes
}
