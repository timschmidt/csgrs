//! Create `Sketch`s from images

use crate::float_types::{EPSILON, Real};
use crate::sketch::Sketch;
use image::GrayImage;
use std::fmt::Debug;

impl<S: Clone + Debug + Send + Sync> Sketch<S> {
    /// Builds a new Sketch from the "on" pixels of a grayscale image,
    /// tracing connected outlines (and holes) via the `contour_tracing` code.
    ///
    /// - `img`: a reference to a GrayImage
    /// - `threshold`: pixels >= threshold are treated as "on/foreground", else off/background
    /// - `closepaths`: if true, each traced path is closed with 'Z' in the SVG path commands
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
        use geo::{Geometry, GeometryCollection, LineString, coord};

        let width = img.width() as usize;
        let height = img.height() as usize;

        /* ---------- step 1 : bitmap → svg path (unchanged) ---------- */
        let mut bits = Vec::with_capacity(height);
        for y in 0..height {
            let mut row = Vec::with_capacity(width);
            for x in 0..width {
                let v = img.get_pixel(x as u32, y as u32)[0];
                row.push((v >= threshold) as i8);
            }
            bits.push(row);
        }
        let svg_path = contour_tracing::array::bits_to_paths(bits, closepaths);
        let polylines = Self::parse_svg_path_into_polylines(&svg_path);

        /* ---------- step 2 : polylines → geo geometries ---------- */
        let mut gc = GeometryCollection::<Real>::default();

        for mut pl in polylines {
            if pl.len() < 2 {
                continue;
            }

            // Are first & last points coincident?
            let closed = {
                let first = pl[0];
                let last = pl[pl.len() - 1];
                ((first.0 - last.0).abs() as Real) < EPSILON
                    && ((first.1 - last.1).abs() as Real) < EPSILON
            };

            // Convert to Real coords (+ make sure polygons are explicitly closed)
            if closed && pl.len() >= 4 {
                // guarantee first==last for LineString
                if ((pl[0].0 - pl[pl.len() - 1].0).abs() as Real) > EPSILON
                    || ((pl[0].1 - pl[pl.len() - 1].1).abs() as Real) > EPSILON
                {
                    pl.push(pl[0]);
                }
                let ls: LineString<Real> = pl
                    .into_iter()
                    .map(|(x, y)| coord! { x: x as Real, y: y as Real })
                    .collect();
                gc.0.push(Geometry::Polygon(geo::Polygon::new(ls, vec![])));
            } else {
                let ls: LineString<Real> = pl
                    .into_iter()
                    .map(|(x, y)| coord! { x: x as Real, y: y as Real })
                    .collect();
                gc.0.push(Geometry::LineString(ls));
            }
        }

        /* ---------- step 3 : build the Sketch ---------- */
        Sketch::from_geo(gc, metadata)
    }

    /// Internal helper to parse a minimal subset of SVG path commands:
    /// - M x y   => move absolute
    /// - H x     => horizontal line
    /// - V y     => vertical line
    /// - Z       => close path
    ///
    /// Returns a `Vec` of polylines, each polyline is a list of `(x, y)` in integer coords.
    fn parse_svg_path_into_polylines(path_str: &str) -> Vec<Vec<(f32, f32)>> {
        let mut polylines = Vec::new();
        let mut current_poly = Vec::new();

        let mut current_x = 0.0_f32;
        let mut current_y = 0.0_f32;
        let mut chars = path_str.trim().chars().peekable();

        // We'll read tokens that could be:
        //  - a letter (M/H/V/Z)
        //  - a number (which may be float, but from bits_to_paths it's all integer steps)
        //  - whitespace or other
        //
        // This small scanner accumulates tokens so we can parse them easily.
        fn read_number<I: Iterator<Item = char>>(
            iter: &mut std::iter::Peekable<I>,
        ) -> Option<f32> {
            let mut buf = String::new();
            // skip leading spaces
            while let Some(&ch) = iter.peek() {
                if ch.is_whitespace() {
                    iter.next();
                } else {
                    break;
                }
            }
            // read sign or digits
            while let Some(&ch) = iter.peek() {
                if ch.is_ascii_digit() || ch == '.' || ch == '-' {
                    buf.push(ch);
                    iter.next();
                } else {
                    break;
                }
            }
            if buf.is_empty() {
                return None;
            }
            // parse as f32
            buf.parse().ok()
        }

        while let Some(ch) = chars.next() {
            match ch {
                'M' | 'm' => {
                    // Move command => read 2 numbers for x,y
                    if !current_poly.is_empty() {
                        // start a new polyline
                        polylines.push(current_poly);
                        current_poly = Vec::new();
                    }
                    let nx = read_number(&mut chars).unwrap_or(current_x);
                    let ny = read_number(&mut chars).unwrap_or(current_y);
                    current_x = nx;
                    current_y = ny;
                    current_poly.push((current_x, current_y));
                },
                'H' | 'h' => {
                    // Horizontal line => read 1 number for x
                    let nx = read_number(&mut chars).unwrap_or(current_x);
                    current_x = nx;
                    current_poly.push((current_x, current_y));
                },
                'V' | 'v' => {
                    // Vertical line => read 1 number for y
                    let ny = read_number(&mut chars).unwrap_or(current_y);
                    current_y = ny;
                    current_poly.push((current_x, current_y));
                },
                'Z' | 'z' => {
                    // Close path
                    // We'll let the calling code decide if it must explicitly connect back.
                    // For now, we just note that this polyline ends.
                    if !current_poly.is_empty() {
                        polylines.push(std::mem::take(&mut current_poly));
                    }
                },
                // Possibly other characters (digits) or spaces:
                c if c.is_whitespace() || c.is_ascii_digit() || c == '-' => {
                    // Could be an inlined number if the path commands had no letter.
                    // Typically bits_to_paths always has M/H/V so we might ignore it or handle gracefully.
                    // If you want robust parsing, you can push this char back and try read_number.
                },
                _ => {
                    // ignoring other
                },
            }
        }

        // If the last polyline is non‐empty, push it.
        if !current_poly.is_empty() {
            polylines.push(current_poly);
        }

        polylines
    }
}
