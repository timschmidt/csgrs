//! Two-dimensional metaball contours as native Hypercurve regions.

use crate::hyper_math::{hreal_from_f64, hreal_gt_f64, hreal_sign};
use hashbrown::HashMap;
use hypercurve::{Contour2, CurvePolicy, CurveRegion2, Point2};
use hyperlattice::Real;
use hyperlimit::{real_max, real_min};
use hyperreal::RealSign;

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
enum GridEdge {
    Horizontal(usize, usize),
    Vertical(usize, usize),
}

#[derive(Clone, Debug)]
struct SamplePoint {
    coordinates: [Real; 2],
    edge: GridEdge,
}

type Segment = [SamplePoint; 2];

const fn ambiguous_edge_pairs(case_index: u8, center_inside: bool) -> [(usize, usize); 2] {
    if (case_index == 5) == center_inside {
        [(0, 1), (2, 3)]
    } else {
        [(0, 3), (1, 2)]
    }
}

/// Create a 2D metaball iso-contour in XY plane from hypercurve centers.
/// - `balls`: array of (center, radius).
/// - `resolution`: (nx, ny) grid resolution for marching squares.
/// - `iso_value`: threshold for the iso-surface.
/// - `padding`: extra boundary beyond each ball's radius.
///
/// This samples a Wyvill-style soft object field and extracts 2D
/// isocontours with the marching-squares analogue of Lorensen and Cline's
/// marching-cubes cell traversal. Centers are native `hypercurve::Point2`
/// values and influence distances are evaluated with hyperreal squared
/// point distance before exporting finite samples to the marching-squares
/// boundary. Closed contour loops are promoted directly into
/// `hypercurve::CurveRegion2`, so CAD topology is carried by hyper geometry
/// instead of by a temporary finite polygon. See Wyvill, McPheeters, and
/// Wyvill, "Data structure for soft objects", *The Visual Computer* 2(4),
/// 1986, DOI: 10.1007/BF01900346; Lorensen and Cline, "Marching Cubes: A
/// High Resolution 3D Surface Construction Algorithm", SIGGRAPH 1987, DOI:
/// 10.1145/37401.37422; and Yap, "Towards Exact Geometric Computation",
/// *Computational Geometry* 7(1-2), 1997,
/// <https://doi.org/10.1016/0925-7721(95)00040-2>.
pub(crate) fn metaballs(
    balls: &[(Point2, Real)],
    resolution: (usize, usize),
    iso_value: Real,
    padding: Real,
) -> CurveRegion2 {
    let (nx, ny) = resolution;
    if balls.is_empty() || nx < 2 || ny < 2 {
        return CurveRegion2::empty();
    }
    let Some(sample_count) = nx.checked_mul(ny) else {
        return CurveRegion2::empty();
    };

    let Some(padding_h) = hreal_from_f64(&padding).ok() else {
        return CurveRegion2::empty();
    };
    if matches!(hreal_sign(&padding_h), Some(RealSign::Negative)) {
        return CurveRegion2::empty();
    }
    let Some(iso_value_h) = hreal_from_f64(&iso_value).ok() else {
        return CurveRegion2::empty();
    };
    if !matches!(hreal_sign(&iso_value_h), Some(RealSign::Positive)) {
        return CurveRegion2::empty();
    }

    let valid_balls = balls
        .iter()
        .filter_map(|(center, radius)| {
            let radius = hreal_from_f64(radius).ok()?;
            matches!(hreal_sign(&radius), Some(RealSign::Positive)).then_some((center, radius))
        })
        .collect::<Vec<_>>();
    if valid_balls.is_empty() {
        return CurveRegion2::empty();
    }

    // 1) Compute bounding box around all metaballs in hyperreal space.
    let Some((min_x, min_y, max_x, max_y)) =
        metaball_bounds_hreal(&valid_balls, &iso_value_h, &padding_h)
    else {
        return CurveRegion2::empty();
    };

    let Some(nx_step_count) = hreal_from_f64(nx - 1).ok() else {
        return CurveRegion2::empty();
    };
    let Some(ny_step_count) = hreal_from_f64(ny - 1).ok() else {
        return CurveRegion2::empty();
    };
    let Some(dx) = ((max_x.clone() - min_x.clone()) / nx_step_count).ok() else {
        return CurveRegion2::empty();
    };
    let Some(dy) = ((max_y.clone() - min_y.clone()) / ny_step_count).ok() else {
        return CurveRegion2::empty();
    };
    let Some(x_coords) = grid_axis_coords(&min_x, &dx, nx) else {
        return CurveRegion2::empty();
    };
    let Some(y_coords) = grid_axis_coords(&min_y, &dy, ny) else {
        return CurveRegion2::empty();
    };

    // 2) Fill a grid with the summed "influence" minus iso_value
    /// **Mathematical Foundation**: 2D metaball influence I(p) = r²/|p-c|².
    /// Exact center singularities are mapped to a finite extraction sentinel
    /// instead of perturbing every denominator with a tolerance.
    /// **Optimization**: Iterator-based computation with early termination for distant points.
    fn scalar_field(balls: &[(&Point2, Real)], sample: &Point2) -> Option<Real> {
        let mut sum = Real::zero();
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
                sum += hreal_from_f64(1.0e10).ok()?;
                continue;
            }
            let denominator = distance_sq;
            sum += (radius_squared / denominator).ok()?;
        }
        Some(sum)
    }

    let mut grid = vec![Real::zero(); sample_count];
    let index = |ix: usize, iy: usize| -> usize { iy * nx + ix };
    for iy in 0..ny {
        let yv = &y_coords[iy];
        for ix in 0..nx {
            let xv = &x_coords[ix];
            let sample = Point2::new(xv.clone(), yv.clone());
            let val = scalar_field(&valid_balls, &sample).unwrap_or_else(Real::zero)
                - iso_value_h.clone();
            grid[index(ix, iy)] = val;
        }
    }

    // 3) Marching squares -> exact interpolated line segments.
    let mut contours = Vec::<Segment>::new();

    // Interpolator:
    let interpolate = |(x1, y1, v1): (&Real, &Real, &Real),
                       (x2, y2, v2): (&Real, &Real, &Real)|
     -> Option<[Real; 2]> {
        let delta = v2.clone() - v1.clone();
        if matches!(hreal_sign(&delta), Some(RealSign::Zero)) {
            Some([x1.clone(), y1.clone()])
        } else {
            let t = (Real::zero() - v1.clone()) / delta; // crossing at 0
            let t = t.ok()?;
            let x = x1.clone() + t.clone() * (x2.clone() - x1.clone());
            let y = y1.clone() + t * (y2.clone() - y1.clone());
            Some([x, y])
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
            let mut check_edge =
                |mask_a: u8, mask_b: u8, a: usize, b: usize, edge: GridEdge| {
                    let inside_a = (c & mask_a) != 0;
                    let inside_b = (c & mask_b) != 0;
                    if inside_a != inside_b
                        && let Some(coordinates) = interpolate(corners[a], corners[b])
                    {
                        pts.push(SamplePoint { coordinates, edge });
                    }
                };

            check_edge(1, 2, 0, 1, GridEdge::Horizontal(ix, iy));
            check_edge(2, 4, 1, 2, GridEdge::Vertical(ix + 1, iy));
            check_edge(4, 8, 2, 3, GridEdge::Horizontal(ix, iy + 1));
            check_edge(8, 1, 3, 0, GridEdge::Vertical(ix, iy));

            // Resolve diagonal cases from the bilinear cell-center sign.
            // A fixed edge-order pairing changes topology when the center is
            // on the other side of the isocontour.
            if matches!(c, 5 | 10) && pts.len() == 4 {
                let center = v0.clone() + v1.clone() + v2.clone() + v3.clone();
                let center_inside =
                    matches!(hreal_sign(&center), Some(RealSign::Positive | RealSign::Zero));
                for (a, b) in ambiguous_edge_pairs(c, center_inside) {
                    contours.push([pts[a].clone(), pts[b].clone()]);
                }
            } else if pts.len() == 2 {
                contours.push([pts[0].clone(), pts[1].clone()]);
            }
        }
    }

    // 4) Stitch line segments and promote closed loops directly to CurveRegion2.
    let stitched = stitch(&contours);
    let material = stitched
        .iter()
        .filter(|line| line.len() >= 4 && same_point(&line[0], line.last().unwrap()))
        .filter_map(|line| {
            let coordinates = line
                .iter()
                .map(|point| point.coordinates.clone())
                .collect::<Vec<_>>();
            Contour2::from_real_ring(&coordinates).ok()
        })
        .collect::<Vec<_>>();

    if material.is_empty() {
        return CurveRegion2::empty();
    }

    CurveRegion2::try_from_native_material_contours(material, &CurvePolicy::certified())
        .unwrap_or_else(|_| CurveRegion2::empty())
}

fn metaball_bounds_hreal(
    balls: &[(&Point2, Real)],
    iso_value: &Real,
    padding: &Real,
) -> Option<(Real, Real, Real, Real)> {
    let count_over_iso = (Real::from(balls.len() as u64) / iso_value.clone()).ok()?;
    let influence_extent_scale =
        ((count_over_iso.sqrt().ok()? * Real::from(101_u8)) / Real::from(100_u8)).ok()?;
    let mut bounds = balls.iter().map(|(center, radius)| {
        let extent = radius.clone() * influence_extent_scale.clone() + padding.clone();
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
        min_x = real_min(&min_x, &next_min_x).value().cloned()?;
        min_y = real_min(&min_y, &next_min_y).value().cloned()?;
        max_x = real_max(&max_x, &next_max_x).value().cloned()?;
        max_y = real_max(&max_y, &next_max_y).value().cloned()?;
    }
    Some((min_x, min_y, max_x, max_y))
}

fn grid_axis_coords(origin: &Real, step: &Real, count: usize) -> Option<Vec<Real>> {
    (0..count)
        .map(|index| {
            let index = hreal_from_f64(index).ok()?;
            Some(origin.clone() + step.clone() * index)
        })
        .collect()
}

fn same_point(a: &SamplePoint, b: &SamplePoint) -> bool {
    a.edge == b.edge
}

/// Stitch all 2-point segments into longer polylines.
///
/// Chains that reach the sampling boundary remain open. Closing them with a
/// synthetic chord invents material that was not present in the scalar field.
fn stitch(contours: &[Segment]) -> Vec<Vec<SamplePoint>> {
    // adjacency map  endpoint -> (line index, end-id 0|1)
    let mut adj: HashMap<GridEdge, Vec<(usize, usize)>> = HashMap::new();
    for (idx, segment) in contours.iter().enumerate() {
        adj.entry(segment[0].edge).or_default().push((idx, 0));
        adj.entry(segment[1].edge).or_default().push((idx, 1));
    }

    let mut used = vec![false; contours.len()];
    let mut chains = Vec::new();

    for start in 0..contours.len() {
        if used[start] {
            continue;
        }
        used[start] = true;

        // current chain of points
        let mut chain = vec![contours[start][0].clone(), contours[start][1].clone()];

        extend_chain(&mut chain, contours, &adj, &mut used, true);
        extend_chain(&mut chain, contours, &adj, &mut used, false);

        chains.push(chain);
    }
    chains
}

fn extend_chain(
    chain: &mut Vec<SamplePoint>,
    contours: &[Segment],
    adj: &HashMap<GridEdge, Vec<(usize, usize)>>,
    used: &mut [bool],
    forward: bool,
) {
    loop {
        let endpoint = if forward {
            chain.last().unwrap()
        } else {
            &chain[0]
        };
        let Some(cands) = adj.get(&endpoint.edge) else {
            break;
        };
        let mut found = None;
        for &(idx, end_id) in cands {
            if used[idx] {
                continue;
            }
            used[idx] = true;
            let other = contours[idx][1 - end_id].clone();
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

#[cfg(test)]
mod tests {
    use super::*;

    fn sample(edge: GridEdge) -> SamplePoint {
        SamplePoint {
            coordinates: [Real::zero(), Real::zero()],
            edge,
        }
    }

    #[test]
    fn ambiguous_cases_follow_the_cell_center_decider() {
        assert_eq!(ambiguous_edge_pairs(5, true), [(0, 1), (2, 3)]);
        assert_eq!(ambiguous_edge_pairs(5, false), [(0, 3), (1, 2)]);
        assert_eq!(ambiguous_edge_pairs(10, true), [(0, 3), (1, 2)]);
        assert_eq!(ambiguous_edge_pairs(10, false), [(0, 1), (2, 3)]);
    }

    #[test]
    fn stitching_does_not_invent_a_chord_across_an_open_contour() {
        let segments = [
            [
                sample(GridEdge::Horizontal(0, 0)),
                sample(GridEdge::Vertical(1, 0)),
            ],
            [
                sample(GridEdge::Vertical(1, 0)),
                sample(GridEdge::Horizontal(0, 1)),
            ],
        ];
        let chains = stitch(&segments);
        assert_eq!(chains.len(), 1);
        assert_eq!(chains[0].len(), 3);
        assert!(!same_point(&chains[0][0], chains[0].last().unwrap()));
    }

    #[test]
    fn conservative_sampling_produces_closed_material_without_boundary_chords() {
        let balls = [(Point2::new(Real::zero(), Real::zero()), Real::one())];
        assert!(!metaballs(&balls, (12, 12), Real::one(), Real::zero()).is_empty());
        assert!(metaballs(&balls, (12, 12), Real::zero(), Real::zero()).is_empty());
        assert!(metaballs(&balls, (usize::MAX, 2), Real::one(), Real::zero()).is_empty());
    }
}
