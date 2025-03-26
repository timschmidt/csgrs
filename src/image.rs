use crate::csg::CSG;
use crate::float_types::{EPSILON, Real};
use crate::polygon::Polygon;
use crate::vertex::Vertex;
use image::GrayImage;
use nalgebra::{Point3, Vector3};
use std::fmt::Debug;

impl<S: Clone + Debug> CSG<S>
where S: Clone + Send + Sync {
    /// Builds a new CSG from the “on” pixels of a grayscale image,
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
    /// collected in one `CSG`.
    ///
    /// # Example
    /// ```no_run
    /// # use csgrs::csg::CSG;
    /// # use image::{GrayImage, Luma};
    /// # fn main() {
    /// let img: GrayImage = image::open("my_binary.png").unwrap().to_luma8();
    /// let csg2d = CSG::from_image(&img, 128, true, None);
    /// // optionally extrude it:
    /// let shape3d = csg2d.extrude(5.0);
    /// # }
    /// ```
    pub fn from_image(
        img: &GrayImage,
        threshold: u8,
        closepaths: bool,
        metadata: Option<S>,
    ) -> Self {
        // Convert the image into a 2D array of bits for the contour_tracing::array::bits_to_paths function.
        // We treat pixels >= threshold as 1, else 0.
        let width = img.width() as usize;
        let height = img.height() as usize;
        let mut bits = Vec::with_capacity(height);
        for y in 0..height {
            let mut row = Vec::with_capacity(width);
            for x in 0..width {
                let px_val = img.get_pixel(x as u32, y as u32)[0];
                if px_val >= threshold {
                    row.push(1);
                } else {
                    row.push(0);
                }
            }
            bits.push(row);
        }

        // Use contour_tracing::array::bits_to_paths to get a single SVG path string
        // containing multiple “move” commands for each outline/hole.
        let svg_path = contour_tracing::array::bits_to_paths(bits, closepaths);
        // This might look like: "M1 0H4V1H1M6 0H11V5H6 ..." etc.

        // Parse the path string into one or more polylines. Each polyline
        // starts with an 'M x y' and then “H x” or “V y” commands until the next 'M' or end.
        let polylines = Self::parse_svg_path_into_polylines(&svg_path);

        // Convert each polyline into a Polygon in the XY plane at z=0,
        // storing them in a `Vec<Polygon<S>>`.
        let mut all_polygons = Vec::new();
        for pl in polylines {
            if pl.len() < 2 {
                continue;
            }
            // Build vertices with normal = +Z
            let normal = Vector3::z();
            let mut verts = Vec::with_capacity(pl.len());
            for &(x, y) in &pl {
                verts.push(Vertex::new(Point3::new(x as Real, y as Real, 0.0), normal));
            }
            // If the path was not closed and we used closepaths == true, we might need to ensure the first/last are the same.
            if (verts.first().unwrap().pos - verts.last().unwrap().pos).norm() > EPSILON {
                // close it
                verts.push(verts.first().unwrap().clone());
            }
            let poly = Polygon::new(verts, metadata.clone());
            all_polygons.push(poly);
        }

        // Build a CSG from those polygons
        CSG::from_polygons(&all_polygons)
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
        //  - a number (which may be float, but from bits_to_paths it’s all integer steps)
        //  - whitespace or other
        //
        // This small scanner accumulates tokens so we can parse them easily.
        fn read_number<I: Iterator<Item = char>>(iter: &mut std::iter::Peekable<I>) -> Option<f32> {
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
                }
                'H' | 'h' => {
                    // Horizontal line => read 1 number for x
                    let nx = read_number(&mut chars).unwrap_or(current_x);
                    current_x = nx;
                    current_poly.push((current_x, current_y));
                }
                'V' | 'v' => {
                    // Vertical line => read 1 number for y
                    let ny = read_number(&mut chars).unwrap_or(current_y);
                    current_y = ny;
                    current_poly.push((current_x, current_y));
                }
                'Z' | 'z' => {
                    // Close path
                    // We'll let the calling code decide if it must explicitly connect back.
                    // For now, we just note that this polyline ends.
                    if !current_poly.is_empty() {
                        polylines.push(std::mem::take(&mut current_poly));
                    }
                }
                // Possibly other characters (digits) or spaces:
                c if c.is_whitespace() || c.is_ascii_digit() || c == '-' => {
                    // Could be an inlined number if the path commands had no letter.
                    // Typically bits_to_paths always has M/H/V so we might ignore it or handle gracefully.
                    // If you want robust parsing, you can push this char back and try read_number.
                }
                _ => {
                    // ignoring other
                }
            }
        }

        // If the last polyline is non‐empty, push it.
        if !current_poly.is_empty() {
            polylines.push(current_poly);
        }

        polylines
    }
}
