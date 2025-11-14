//! Create `Sketch`s from images

use crate::io::svg::FromSVG;
use crate::sketch::Sketch;
use crate::traits::CSG;
use image::GrayImage;
use std::fmt::Debug;

impl<S: Clone + Debug + Send + Sync> Sketch<S> {
    /// Builds a new Sketch from the "on" pixels of a grayscale image,
    /// tracing connected outlines (and holes) via the `contour_tracing` code.
    ///
    /// - `img` – The raster source (`image::GrayImage`).
    /// - `threshold` – Pixels whose value is **≥ `threshold`** are treated as *solid*; all others are ignored.
    /// - `closepaths` – Forwarded to the contour tracer; when `true` it will attempt to close any open contours so that we get valid closed polygons wherever possible.
    /// - `metadata`: optional metadata to attach to the resulting polygons
    ///
    /// # Returns
    /// A 2D shape in the XY plane (z=0) representing all traced contours. Each contour
    /// becomes a polygon. The polygons are *not* automatically unioned; they are simply
    /// collected in one `Sketch`.
    ///
    /// # Example
    /// ```no_run
    /// # use csgrs::sketch::Sketch;
    /// # use image::{GrayImage, Luma};
    /// # fn main() {
    /// let img: GrayImage = image::open("my_binary.png").unwrap().to_luma8();
    /// let my_sketch = Sketch::<()>::from_image(&img, 128, true, None);
    /// // optionally extrude it:
    /// let my_mesh = my_sketch.extrude(5.0);
    /// # }
    /// ```
    pub fn from_image(
        img: &GrayImage,
        threshold: u8,
        closepaths: bool,
        metadata: Option<S>,
    ) -> Self {
        let width = img.width() as usize;
        let height = img.height() as usize;

        // ------------------------------------------------------------------
        // 1. Raster → binary matrix (Vec<Vec<i8>> expected by contour_tracing)
        // ------------------------------------------------------------------
        let mut bits = Vec::with_capacity(height);
        for y in 0..height {
            let mut row = Vec::with_capacity(width);
            for x in 0..width {
                let v = img.get_pixel(x as u32, y as u32)[0];
                row.push((v >= threshold) as i8);
            }
            bits.push(row);
        }

        // ---------------------------------------------------------------
        // 2. Trace the contours; we get back *one* SVG path string.
        // ---------------------------------------------------------------
        let svg_path = contour_tracing::array::bits_to_paths(bits, closepaths);

        // ------------------------------------------------------------------
        // 3. Preferred path: convert via the full SVG parser (if available).
        // ------------------------------------------------------------------
        let svg_doc = format!(
            r#"<svg viewBox="0 0 {w} {h}" xmlns="http://www.w3.org/2000/svg">
<path d="{d}" fill="black" stroke="none" fill-rule="evenodd"/>
</svg>"#,
            w = img.width(),
            h = img.height(),
            d = svg_path
        );

        if let Ok(parsed) = <Sketch<S>>::from_svg(&svg_doc, metadata) {
            parsed
        } else {
            Sketch::new()
        }
    }
}
