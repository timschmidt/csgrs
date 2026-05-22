//! Provides a `MetaBall` struct and functions for creating a `Profile` from [MetaBalls](https://en.wikipedia.org/wiki/Metaballs)

use crate::float_types::{
    Real, hreal_from_f64, hreal_gt_f64, hreal_max_pair, hreal_min_pair, hreal_sign,
    hreal_to_f64,
};
use crate::sketch::Profile;
use hashbrown::HashMap;
use hypercurve::{Contour2, Point2, Region2};
use hyperreal::RealSign;
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

        let Some(padding_h) = hreal_from_f64(padding).ok() else {
            return Profile::empty(metadata);
        };
        if matches!(hreal_sign(&padding_h), Some(RealSign::Negative)) {
            return Profile::empty(metadata);
        }
        let Some(iso_value_h) = hreal_from_f64(iso_value).ok() else {
            return Profile::empty(metadata);
        };

        let valid_balls = balls
            .iter()
            .filter_map(|(center, radius)| {
                let radius = hreal_from_f64(*radius).ok()?;
                matches!(hreal_sign(&radius), Some(RealSign::Positive))
                    .then_some((center, radius))
            })
            .collect::<Vec<_>>();
        if valid_balls.is_empty() {
            return Profile::empty(metadata);
        }

        // 1) Compute bounding box around all metaballs in hyperreal space.
        let Some((min_x, min_y, max_x, max_y)) =
            metaball_bounds_hreal(&valid_balls, &padding_h)
        else {
            return Profile::empty(metadata);
        };

        let Some(nx_step_count) = hreal_from_f64(nx as Real - 1.0).ok() else {
            return Profile::empty(metadata);
        };
        let Some(ny_step_count) = hreal_from_f64(ny as Real - 1.0).ok() else {
            return Profile::empty(metadata);
        };
        let Some(dx) = ((max_x.clone() - min_x.clone()) / nx_step_count).ok() else {
            return Profile::empty(metadata);
        };
        let Some(dy) = ((max_y.clone() - min_y.clone()) / ny_step_count).ok() else {
            return Profile::empty(metadata);
        };
        let Some(x_coords) = grid_axis_coords(&min_x, &dx, nx) else {
            return Profile::empty(metadata);
        };
        let Some(y_coords) = grid_axis_coords(&min_y, &dy, ny) else {
            return Profile::empty(metadata);
        };

        // 2) Fill a grid with the summed "influence" minus iso_value
        /// **Mathematical Foundation**: 2D metaball influence I(p) = r²/|p-c|².
        /// Exact center singularities are mapped to a finite extraction sentinel
        /// instead of perturbing every denominator with a tolerance.
        /// **Optimization**: Iterator-based computation with early termination for distant points.
        fn scalar_field(
            balls: &[(&Point2, hyperreal::Real)],
            sample: &Point2,
        ) -> Option<hyperreal::Real> {
            let mut sum = hyperreal::Real::zero();
            for (center, radius) in balls {
                let radius_squared = radius.clone() * radius.clone();
                let distance_sq = sample.distance_squared(center);

                // Early termination for very distant points.
                let threshold_distance_sq =
                    radius_squared.clone() * hreal_from_f64(1000.0).ok()?;
                if hreal_gt_f64(&(distance_sq.clone() - threshold_distance_sq), 0.0) {
                    continue;
                }

                if matches!(hreal_sign(&distance_sq), Some(RealSign::Zero)) {
                    sum = sum + hreal_from_f64(1.0e10).ok()?;
                    continue;
                }
                let denominator = distance_sq;
                sum = sum + (radius_squared / denominator).ok()?;
            }
            Some(sum)
        }

        let mut grid = vec![hyperreal::Real::zero(); nx * ny];
        let index = |ix: usize, iy: usize| -> usize { iy * nx + ix };
        for iy in 0..ny {
            let yv = &y_coords[iy];
            for ix in 0..nx {
                let xv = &x_coords[ix];
                let sample = Point2::new(xv.clone(), yv.clone());
                let val = scalar_field(&valid_balls, &sample)
                    .unwrap_or_else(hyperreal::Real::zero)
                    - iso_value_h.clone();
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
        let interpolate =
            |(x1, y1, v1): (&hyperreal::Real, &hyperreal::Real, &hyperreal::Real),
             (x2, y2, v2): (&hyperreal::Real, &hyperreal::Real, &hyperreal::Real)|
             -> Option<(Real, Real)> {
                let delta = v2.clone() - v1.clone();
                if matches!(hreal_sign(&delta), Some(RealSign::Zero)) {
                    Some((hreal_to_f64(x1)?, hreal_to_f64(y1)?))
                } else {
                    let t = (hyperreal::Real::zero() - v1.clone()) / delta; // crossing at 0
                    let t = t.ok()?;
                    let x = x1.clone() + t.clone() * (x2.clone() - x1.clone());
                    let y = y1.clone() + t * (y2.clone() - y1.clone());
                    Some((hreal_to_f64(&x)?, hreal_to_f64(&y)?))
                }
            };

        for iy in 0..(ny - 1) {
            let y0 = &y_coords[iy];
            let y1 = &y_coords[iy + 1];

            for ix in 0..(nx - 1) {
                let x0 = &x_coords[ix];
                let x1 = &x_coords[ix + 1];

                let v0 = &grid[index(ix, iy)];
                let v1 = &grid[index(ix + 1, iy)];
                let v2 = &grid[index(ix + 1, iy + 1)];
                let v3 = &grid[index(ix, iy + 1)];

                // classification
                let mut c = 0u8;
                if !matches!(hreal_sign(v0), Some(RealSign::Negative) | None) {
                    c |= 1;
                }
                if !matches!(hreal_sign(v1), Some(RealSign::Negative) | None) {
                    c |= 2;
                }
                if !matches!(hreal_sign(v2), Some(RealSign::Negative) | None) {
                    c |= 4;
                }
                if !matches!(hreal_sign(v3), Some(RealSign::Negative) | None) {
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
                        if let Some((px, py)) = interpolate(corners[a], corners[b]) {
                            pts.push((px, py));
                        }
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

fn metaball_bounds_hreal(
    balls: &[(&Point2, hyperreal::Real)],
    padding: &hyperreal::Real,
) -> Option<(
    hyperreal::Real,
    hyperreal::Real,
    hyperreal::Real,
    hyperreal::Real,
)> {
    let mut bounds = balls.iter().map(|(center, radius)| {
        let extent = radius.clone() + padding.clone();
        Some((
            center.x().clone() - extent.clone(),
            center.y().clone() - extent.clone(),
            center.x().clone() + extent.clone(),
            center.y().clone() + extent,
        ))
    });
    let (mut min_x, mut min_y, mut max_x, mut max_y) = bounds.next()??;
    for bounds in bounds {
        let (next_min_x, next_min_y, next_max_x, next_max_y) = bounds?;
        min_x = hreal_min_pair(&min_x, &next_min_x)?;
        min_y = hreal_min_pair(&min_y, &next_min_y)?;
        max_x = hreal_max_pair(&max_x, &next_max_x)?;
        max_y = hreal_max_pair(&max_y, &next_max_y)?;
    }
    Some((min_x, min_y, max_x, max_y))
}

fn grid_axis_coords(
    origin: &hyperreal::Real,
    step: &hyperreal::Real,
    count: usize,
) -> Option<Vec<hyperreal::Real>> {
    (0..count)
        .map(|index| {
            let index = hreal_from_f64(index as Real).ok()?;
            Some(origin.clone() + step.clone() * index)
        })
        .collect()
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
