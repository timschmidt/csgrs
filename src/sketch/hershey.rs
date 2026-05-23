//! Create `Profile`s using single stroke Hershey fonts

use crate::sketch::Profile;
use hershey::{Font, Glyph as HersheyGlyph, Vector as HersheyVector};
use hypercurve::CurveString2;
use hyperlattice::Real;
use std::fmt::Debug;

impl<M: Clone + Debug + Send + Sync> Profile<M> {
    /// Creates **2D line-stroke text** in the XY plane using a Hershey font.
    ///
    /// Each glyph stroke becomes a native `hypercurve::CurveString2` wire.
    /// If you need the strokes filled or thickened, offset or extrude the
    /// finite projection at the API boundary. This keeps path topology in
    /// hyperreal-backed curve strings rather than in a finite compatibility cache,
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
    /// - `metadata`: Optional user data to store in the resulting Profile
    ///
    /// # Returns
    /// A new `Profile` where each glyph stroke is a native open wire.
    pub fn from_hershey(text: &str, font: &Font, size: Real, metadata: M) -> Profile<M> {
        let mut wires = Vec::new();
        let mut cursor_x = Real::zero();

        for ch in text.chars() {
            // Hershey fonts do not carry filled glyphs; whitespace is advance only.
            if ch.is_control() || ch.is_whitespace() {
                cursor_x += Real::from(6_u8) * size.clone();
                continue;
            }

            // Attempt to find a glyph in this font
            match font.glyph(ch) {
                Ok(glyph) => {
                    let glyph_width = Real::from(glyph.max_x - glyph.min_x);
                    wires.extend(
                        build_hershey_glyph_lines(
                            &glyph,
                            size.clone(),
                            cursor_x.clone(),
                            Real::zero(),
                        )
                        .into_iter()
                        .filter_map(|points| CurveString2::from_real_point_iter(points).ok()),
                    );

                    // Advance the pen in X
                    cursor_x += glyph_width * size.clone() * 0.8;
                },
                Err(_) => {
                    // Missing glyph => skip or just advance
                    cursor_x += Real::from(6_u8) * size.clone();
                },
            }
        }

        Profile::from_wires(wires, metadata)
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
                let px = offset_x.clone() + Real::from(*x) * scale.clone();
                let py = offset_y.clone() + Real::from(*y) * scale.clone();
                current_coords.push([px, py]);
            },
            HersheyVector::LineTo { x, y } => {
                let px = offset_x.clone() + Real::from(*x) * scale.clone();
                let py = offset_y.clone() + Real::from(*y) * scale.clone();
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
