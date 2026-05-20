//! Provides a `MetaBall` struct and functions for creating a `Profile` from [MetaBalls](https://en.wikipedia.org/wiki/Metaballs)

use crate::float_types::{Real, hreal_from_f64, hreal_to_f64, tolerance};
use crate::sketch::Profile;
use hashbrown::HashMap;
use hypercurve::{Contour2, Point2, Region2};
use std::fmt::Debug;

type SamplePoint = [Real; 2];
type Segment = [SamplePoint; 2];

impl<M: Clone + Debug + Send + Sync> Profile<M> {
    /// Create a 2D metaball iso-contour in XY plane from hypercurve centers.
    /// - `balls`: array of (center, radius).
    /// - `resolution`: (nx, ny) grid resolution for marching squares.
    /// - `iso_value`: threshold for the iso-surface.
    /// - `padding`: extra boundary beyond each ball's radius.
    /// - `metadata`: optional user metadata.
    ///
    /// This samples a Wyvill-style soft object field and extracts 2D
    /// isocontours with the marching-squares analogue of Lorensen and Cline's
    /// marching-cubes cell traversal. Centers are native `hypercurve::Point2`
    /// values and influence distances are evaluated with hyperreal squared
    /// point distance before exporting finite samples to the marching-squares
    /// boundary. Closed contour loops are promoted directly into
    /// `hypercurve::Region2`, so CAD topology is carried by hyper geometry
    /// instead of by a temporary finite polygon. See Wyvill, McPheeters, and
    /// Wyvill, "Data structure for soft objects", *The Visual Computer* 2(4),
    /// 1986, DOI: 10.1007/BF01900346; Lorensen and Cline, "Marching Cubes: A
    /// High Resolution 3D Surface Construction Algorithm", SIGGRAPH 1987, DOI:
    /// 10.1145/37401.37422; and Yap, "Towards Exact Geometric Computation",
    /// *Computational Geometry* 7(1-2), 1997,
    /// <https://doi.org/10.1016/0925-7721(95)00040-2>.
    pub fn metaballs(
        balls: &[(Point2, Real)],
        resolution: (usize, usize),
        iso_value: Real,
        padding: Real,
        metadata: M,
    ) -> Profile<M> {
        let (nx, ny) = resolution;
        if balls.is_empty() || nx < 2 || ny < 2 {
            return Profile::empty(metadata);
        }

        // 1) Compute bounding box around all metaballs
        let mut min_x = Real::MAX;
        let mut min_y = Real::MAX;
        let mut max_x = -Real::MAX;
        let mut max_y = -Real::MAX;
        for (center, r) in balls {
            if !r.is_finite() || *r <= 0.0 {
                continue;
            }
            let Some([cx, cy]) = finite_point2_coords(center) else {
                continue;
            };
            let rr = *r + padding;
            if cx - rr < min_x {
                min_x = cx - rr;
            }
            if cx + rr > max_x {
                max_x = cx + rr;
            }
            if cy - rr < min_y {
                min_y = cy - rr;
            }
            if cy + rr > max_y {
                max_y = cy + rr;
            }
        }
        if !(min_x.is_finite() && min_y.is_finite() && max_x.is_finite() && max_y.is_finite())
        {
            return Profile::empty(metadata);
        }

        let dx = (max_x - min_x) / (nx as Real - 1.0);
        let dy = (max_y - min_y) / (ny as Real - 1.0);

        // 2) Fill a grid with the summed "influence" minus iso_value
        /// **Mathematical Foundation**: 2D metaball influence I(p) = r²/(|p-c|² + ε)
        /// **Optimization**: Iterator-based computation with early termination for distant points.
        fn scalar_field(balls: &[(Point2, Real)], x: Real, y: Real) -> Real {
            let Some(sample) = finite_point2(x, y) else {
                return 0.0;
            };
            balls
                .iter()
                .map(|(center, radius)| {
                    if !radius.is_finite() || *radius <= 0.0 {
                        return 0.0;
                    }
                    let Some(distance_sq) = hreal_to_f64(&sample.distance_squared(center))
                    else {
                        return 0.0;
                    };

                    // Early termination for very distant points
                    let threshold_distance_sq = radius * radius * 1000.0;
                    if distance_sq > threshold_distance_sq {
                        0.0
                    } else {
                        let denominator = distance_sq + tolerance();
                        (radius * radius) / denominator
                    }
                })
                .sum()
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

        // 3) Marching squares -> native finite line segments.
        //
        // The samples are finite because marching squares is an extraction
        // boundary for a sampled implicit field; closed stitched loops are
        // promoted immediately to hypercurve contours below.
        let mut contours = Vec::<Segment>::new();

        // Interpolator:
        let interpolate = |(x1, y1, v1): (Real, Real, Real),
                           (x2, y2, v2): (Real, Real, Real)|
         -> (Real, Real) {
            let denom = (v2 - v1).abs();
            if denom < tolerance() {
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

                // 2 intersections => one segment; 4 intersections in ambiguous
                // cells => two segments. Keep these as native point pairs so
                // Finite polygon samples are not the source representation for metaball topology.
                for pair in pts.chunks_exact(2) {
                    contours.push([[pair[0].0, pair[0].1], [pair[1].0, pair[1].1]]);
                }
            }
        }

        // 4) Stitch line segments and promote closed loops directly to Region2.
        let stitched = stitch(&contours);
        let material = stitched
            .iter()
            .filter(|line| line.len() >= 4 && same_point(line[0], *line.last().unwrap()))
            .filter_map(|line| Contour2::from_finite_ring(line).ok())
            .collect::<Vec<_>>();

        if material.is_empty() {
            return Profile::empty(metadata);
        }

        Profile::from_region(Region2::from_material_contours(material), metadata)
    }
}

fn finite_point2(x: Real, y: Real) -> Option<Point2> {
    Some(Point2::new(hreal_from_f64(x).ok()?, hreal_from_f64(y).ok()?))
}

fn finite_point2_coords(point: &Point2) -> Option<[Real; 2]> {
    Some([hreal_to_f64(point.x())?, hreal_to_f64(point.y())?])
}

// helper – quantise to avoid FP noise
#[inline]
fn key(point: SamplePoint) -> (i64, i64) {
    (
        (point[0] * 1e8).round() as i64,
        (point[1] * 1e8).round() as i64,
    )
}

fn same_point(a: SamplePoint, b: SamplePoint) -> bool {
    key(a) == key(b)
}

/// stitch all 2-point segments into longer polylines,
/// close them when the ends meet
fn stitch(contours: &[Segment]) -> Vec<Vec<SamplePoint>> {
    // adjacency map  endpoint -> (line index, end-id 0|1)
    let mut adj: HashMap<(i64, i64), Vec<(usize, usize)>> = HashMap::new();
    for (idx, segment) in contours.iter().enumerate() {
        adj.entry(key(segment[0])).or_default().push((idx, 0));
        adj.entry(key(segment[1])).or_default().push((idx, 1));
    }

    let mut used = vec![false; contours.len()];
    let mut chains = Vec::new();

    for start in 0..contours.len() {
        if used[start] {
            continue;
        }
        used[start] = true;

        // current chain of points
        let mut chain = vec![contours[start][0], contours[start][1]];

        extend_chain(&mut chain, contours, &adj, &mut used, true);
        extend_chain(&mut chain, contours, &adj, &mut used, false);

        if chain.len() >= 3 && !same_point(chain[0], *chain.last().unwrap()) {
            chain.push(chain[0]);
        }
        chains.push(chain);
    }
    chains
}

fn extend_chain(
    chain: &mut Vec<SamplePoint>,
    contours: &[Segment],
    adj: &HashMap<(i64, i64), Vec<(usize, usize)>>,
    used: &mut [bool],
    forward: bool,
) {
    loop {
        let endpoint = if forward {
            *chain.last().unwrap()
        } else {
            chain[0]
        };
        let Some(cands) = adj.get(&key(endpoint)) else {
            break;
        };
        let mut found = None;
        for &(idx, end_id) in cands {
            if used[idx] {
                continue;
            }
            used[idx] = true;
            let other = contours[idx][1 - end_id];
            found = Some(other);
            break;
        }
        let Some(other) = found else {
            break;
        };
        if forward {
            chain.push(other);
        } else {
            chain.insert(0, other);
        }
    }
}
