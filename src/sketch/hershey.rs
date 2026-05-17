//! Create `Sketch`s using single stroke Hershey fonts

use crate::float_types::Real;
use crate::sketch::{Sketch, wire_from_points};
use hershey::{Font, Glyph as HersheyGlyph, Vector as HersheyVector};
use std::fmt::Debug;

impl<M: Clone + Debug + Send + Sync> Sketch<M> {
    /// Creates **2D line-stroke text** in the XY plane using a Hershey font.
    ///
    /// Each glyph stroke becomes a native `hypercurve::CurveString2` wire.
    /// If you need the strokes filled or thickened, offset or extrude the
    /// finite projection at the API boundary. This keeps path topology in
    /// hyperreal-backed curve strings rather than in the temporary `geo` cache,
    /// following Yap, "Towards Exact Geometric Computation," *Computational
    /// Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>). The stroke-font data
    /// model follows the Hershey vector font system distributed by the U.S.
    /// National Bureau of Standards as NBS Special Publication 424, 1976.
    ///
    /// # Parameters
    /// - `text`: The text to render
    /// - `font`: The Hershey font (e.g., `hershey::fonts::GOTHIC_ENG_SANS`)
    /// - `size`: Scale factor for glyphs
    /// - `metadata`: Optional user data to store in the resulting Sketch
    ///
    /// # Returns
    /// A new `Sketch` where each glyph stroke is a native open wire.
    pub fn from_hershey(text: &str, font: &Font, size: Real, metadata: M) -> Sketch<M> {
        let mut wires = Vec::new();
        let mut cursor_x: Real = 0.0;

        for ch in text.chars() {
            // Hershey fonts do not carry filled glyphs; whitespace is advance only.
            if ch.is_control() || ch.is_whitespace() {
                cursor_x += 6.0 * size;
                continue;
            }

            // Attempt to find a glyph in this font
            match font.glyph(ch) {
                Ok(glyph) => {
                    let glyph_width = (glyph.max_x - glyph.min_x) as Real;
                    wires.extend(
                        build_hershey_glyph_lines(&glyph, size, cursor_x, 0.0)
                            .into_iter()
                            .filter_map(wire_from_points),
                    );

                    // Advance the pen in X
                    cursor_x += glyph_width * size * 0.8;
                },
                Err(_) => {
                    // Missing glyph => skip or just advance
                    cursor_x += 6.0 * size;
                },
            }
        }

        Sketch::from_wires(wires, metadata)
    }
}

/// Helper for building open polygons from a single Hershey `Glyph`.
fn build_hershey_glyph_lines(
    glyph: &HersheyGlyph,
    scale: Real,
    offset_x: Real,
    offset_y: Real,
) -> Vec<Vec<[Real; 2]>> {
    let mut strokes = Vec::new();

    // We'll accumulate each stroke’s points in `current_coords`,
    // resetting whenever Hershey issues a "MoveTo"
    let mut current_coords = Vec::new();

    for vector_cmd in &glyph.vectors {
        match vector_cmd {
            HersheyVector::MoveTo { x, y } => {
                // If we already had 2+ points, that stroke is complete:
                if current_coords.len() >= 2 {
                    strokes.push(current_coords);
                }
                // Start a new stroke
                current_coords = Vec::new();
                let px = offset_x + (*x as Real) * scale;
                let py = offset_y + (*y as Real) * scale;
                current_coords.push([px, py]);
            },
            HersheyVector::LineTo { x, y } => {
                let px = offset_x + (*x as Real) * scale;
                let py = offset_y + (*y as Real) * scale;
                current_coords.push([px, py]);
            },
        }
    }

    // End-of-glyph: if our final stroke has 2+ points, keep it as one wire run.
    if current_coords.len() >= 2 {
        strokes.push(current_coords);
    }

    strokes
}
