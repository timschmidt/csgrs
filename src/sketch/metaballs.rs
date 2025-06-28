use crate::float_types::{EPSILON, Real};
use crate::sketch::sketch::Sketch;
use crate::traits::CSGOps;
use geo::{
    Coord, CoordsIter, Geometry, GeometryCollection, LineString, Polygon as GeoPolygon, coord,
};
use hashbrown::HashMap;
use std::fmt::Debug;

impl<S: Clone + Debug + Send + Sync> Sketch<S> {
    /// Create a 2D metaball iso-contour in XY plane from a set of 2D metaballs.
    /// - `balls`: array of (center, radius).
    /// - `resolution`: (nx, ny) grid resolution for marching squares.
    /// - `iso_value`: threshold for the iso-surface.
    /// - `padding`: extra boundary beyond each ball's radius.
    /// - `metadata`: optional user metadata.
    pub fn metaballs(
        balls: &[(nalgebra::Point2<Real>, Real)],
        resolution: (usize, usize),
        iso_value: Real,
        padding: Real,
        metadata: Option<S>,
    ) -> Sketch<S> {
        let (nx, ny) = resolution;
        if balls.is_empty() || nx < 2 || ny < 2 {
            return Sketch::new();
        }

        // 1) Compute bounding box around all metaballs
        let mut min_x = Real::MAX;
        let mut min_y = Real::MAX;
        let mut max_x = -Real::MAX;
        let mut max_y = -Real::MAX;
        for (center, r) in balls {
            let rr = *r + padding;
            if center.x - rr < min_x {
                min_x = center.x - rr;
            }
            if center.x + rr > max_x {
                max_x = center.x + rr;
            }
            if center.y - rr < min_y {
                min_y = center.y - rr;
            }
            if center.y + rr > max_y {
                max_y = center.y + rr;
            }
        }

        let dx = (max_x - min_x) / (nx as Real - 1.0);
        let dy = (max_y - min_y) / (ny as Real - 1.0);

        // 2) Fill a grid with the summed “influence” minus iso_value
        fn scalar_field(balls: &[(nalgebra::Point2<Real>, Real)], x: Real, y: Real) -> Real {
            let mut v = 0.0;
            for (c, r) in balls {
                let dx = x - c.x;
                let dy = y - c.y;
                let dist_sq = dx * dx + dy * dy + EPSILON;
                v += (r * r) / dist_sq;
            }
            v
        }

        let mut grid = vec![0.0; nx * ny];
        let index = |ix: usize, iy: usize| -> usize { iy * nx + ix };
        for iy in 0..ny {
            let yv = min_y + (iy as Real) * dy;
            for ix in 0..nx {
                let xv = min_x + (ix as Real) * dx;
                let val = scalar_field(balls, xv, yv) - iso_value;
                grid[index(ix, iy)] = val;
            }
        }

        // 3) Marching squares -> line segments
        let mut contours = Vec::<LineString<Real>>::new();

        // Interpolator:
        let interpolate = |(x1, y1, v1): (Real, Real, Real),
                           (x2, y2, v2): (Real, Real, Real)|
         -> (Real, Real) {
            let denom = (v2 - v1).abs();
            if denom < EPSILON {
                (x1, y1)
            } else {
                let t = -v1 / (v2 - v1); // crossing at 0
                (x1 + t * (x2 - x1), y1 + t * (y2 - y1))
            }
        };

        for iy in 0..(ny - 1) {
            let y0 = min_y + (iy as Real) * dy;
            let y1 = min_y + ((iy + 1) as Real) * dy;

            for ix in 0..(nx - 1) {
                let x0 = min_x + (ix as Real) * dx;
                let x1 = min_x + ((ix + 1) as Real) * dx;

                let v0 = grid[index(ix, iy)];
                let v1 = grid[index(ix + 1, iy)];
                let v2 = grid[index(ix + 1, iy + 1)];
                let v3 = grid[index(ix, iy + 1)];

                // classification
                let mut c = 0u8;
                if v0 >= 0.0 {
                    c |= 1;
                }
                if v1 >= 0.0 {
                    c |= 2;
                }
                if v2 >= 0.0 {
                    c |= 4;
                }
                if v3 >= 0.0 {
                    c |= 8;
                }
                if c == 0 || c == 15 {
                    continue; // no crossing
                }

                let corners = [(x0, y0, v0), (x1, y0, v1), (x1, y1, v2), (x0, y1, v3)];

                let mut pts = Vec::new();
                // function to check each edge
                let mut check_edge = |mask_a: u8, mask_b: u8, a: usize, b: usize| {
                    let inside_a = (c & mask_a) != 0;
                    let inside_b = (c & mask_b) != 0;
                    if inside_a != inside_b {
                        let (px, py) = interpolate(corners[a], corners[b]);
                        pts.push((px, py));
                    }
                };

                check_edge(1, 2, 0, 1);
                check_edge(2, 4, 1, 2);
                check_edge(4, 8, 2, 3);
                check_edge(8, 1, 3, 0);

                // we might get 2 intersection points => single line segment
                // or 4 => two line segments, etc.
                // For simplicity, we just store them in a small open polyline:
                if pts.len() >= 2 {
                    let mut pl = LineString::new(vec![]);
                    for &(px, py) in &pts {
                        pl.0.push(coord! {x: px, y: py});
                    }
                    // Do not close. These are just line segments from this cell.
                    contours.push(pl);
                }
            }
        }

        // 4) Convert these line segments into geo::LineStrings or geo::Polygons if closed.
        //    We store them in a GeometryCollection.
        let mut gc = GeometryCollection::default();

        let stitched = stitch(&contours);

        for pl in stitched {
            if pl.is_closed() && pl.coords_count() >= 4 {
                let polygon = GeoPolygon::new(pl, vec![]);
                gc.0.push(Geometry::Polygon(polygon));
            }
        }

        Sketch::from_geo(gc, metadata)
    }
}

// helper – quantise to avoid FP noise
#[inline]
fn key(x: Real, y: Real) -> (i64, i64) {
    ((x * 1e8).round() as i64, (y * 1e8).round() as i64)
}

/// stitch all 2-point segments into longer polylines,
/// close them when the ends meet
fn stitch(contours: &[LineString<Real>]) -> Vec<LineString<Real>> {
    // adjacency map  endpoint -> (line index, end-id 0|1)
    let mut adj: HashMap<(i64, i64), Vec<(usize, usize)>> = HashMap::new();
    for (idx, ls) in contours.iter().enumerate() {
        let p0 = ls[0]; // first point
        let p1 = ls[1]; // second point
        adj.entry(key(p0.x, p0.y)).or_default().push((idx, 0));
        adj.entry(key(p1.x, p1.y)).or_default().push((idx, 1));
    }

    let mut used = vec![false; contours.len()];
    let mut chains = Vec::new();

    for start in 0..contours.len() {
        if used[start] {
            continue;
        }
        used[start] = true;

        // current chain of points
        let mut chain = Vec::<Coord<Real>>::from(contours[start].0.clone());

        // walk forward
        loop {
            let last = *chain.last().unwrap();
            let Some(cands) = adj.get(&key(last.x, last.y)) else {
                break;
            };
            let mut found = None;
            for &(idx, end_id) in cands {
                if used[idx] {
                    continue;
                }
                used[idx] = true;
                // choose the *other* endpoint
                let other = contours[idx][1 - end_id];
                chain.push(other);
                found = Some(());
                break;
            }
            if found.is_none() {
                break;
            }
        }

        // close if ends coincide
        if chain.len() >= 3 && (chain[0] == *chain.last().unwrap()) == false {
            chain.push(chain[0]);
        }
        chains.push(LineString::new(chain));
    }
    chains
}
