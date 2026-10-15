//! Exact-coordinate marching squares, promoted through the native region arrangement.

use crate::{
    CurveContext, CurveError, CurveRegion2, ExactCurveError, FillRule, LineSeg2, Point2, Segment2,
};
use hyperreal::{Problem, Real, RealSign};
use std::{cmp::Ordering, error::Error, fmt};

/// Borrowed rectangular scalar grid, with strictly increasing exact axes.
///
/// Values are x-fast. Nonpositive samples are inside; positive samples are
/// outside. The boundary must be strictly exterior so extraction cannot invent
/// a closing chord or silently clip an unbounded/open region.
#[derive(Clone, Copy, Debug)]
pub struct MarchingSquaresGrid<'a> {
    pub x: &'a [Real],
    pub y: &'a [Real],
    pub values: &'a [Real],
}

impl MarchingSquaresGrid<'_> {
    /// Validate dimensions and the representable primary scalar-buffer capacity.
    pub fn checked_sample_count(dimensions: [usize; 2]) -> Result<usize, MarchingSquaresError> {
        if dimensions.iter().any(|&n| n < 2) {
            return Err(MarchingSquaresError::GridTooSmall { dimensions });
        }
        let count = dimensions[0]
            .checked_mul(dimensions[1])
            .ok_or(MarchingSquaresError::CapacityOverflow)?;
        checked_capacity::<Real>(count)?;
        Ok(count)
    }
}

/// A failed sampled-region construction. No partial region is published.
#[derive(Clone, Debug, PartialEq)]
pub enum MarchingSquaresError {
    GridTooSmall {
        dimensions: [usize; 2],
    },
    SampleCountMismatch {
        expected: usize,
        actual: usize,
    },
    CapacityOverflow,
    AllocationFailed {
        elements: usize,
    },
    AxisNotIncreasing {
        axis: usize,
        index: usize,
    },
    PredicateUndecided {
        operation: &'static str,
        index: usize,
    },
    BoundaryNotExterior {
        sample: [usize; 2],
    },
    Scalar(Problem),
    Curve(CurveError),
    Region(ExactCurveError),
}

impl fmt::Display for MarchingSquaresError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::GridTooSmall { dimensions } => write!(
                f,
                "marching-squares dimensions must each be at least two: {dimensions:?}"
            ),
            Self::SampleCountMismatch { expected, actual } => write!(
                f,
                "marching squares expected {expected} samples, got {actual}"
            ),
            Self::CapacityOverflow => write!(
                f,
                "marching-squares buffer capacity exceeds platform limits"
            ),
            Self::AllocationFailed { elements } => write!(
                f,
                "could not reserve marching-squares buffer for {elements} elements"
            ),
            Self::AxisNotIncreasing { axis, index } => write!(
                f,
                "marching-squares axis {axis} is not increasing at {index}"
            ),
            Self::PredicateUndecided { operation, index } => {
                write!(f, "marching-squares {operation} is undecided at {index}")
            }
            Self::BoundaryNotExterior { sample } => write!(
                f,
                "marching-squares boundary sample {sample:?} is not strictly exterior"
            ),
            Self::Scalar(source) => {
                write!(f, "marching-squares scalar construction failed: {source}")
            }
            Self::Curve(source) => {
                write!(f, "marching-squares segment construction failed: {source}")
            }
            Self::Region(source) => write!(f, "marching-squares region admission failed: {source}"),
        }
    }
}

impl Error for MarchingSquaresError {
    fn source(&self) -> Option<&(dyn Error + 'static)> {
        match self {
            Self::Scalar(source) => Some(source),
            Self::Curve(source) => Some(source),
            Self::Region(source) => Some(source),
            _ => None,
        }
    }
}

fn checked_capacity<T>(count: usize) -> Result<(), MarchingSquaresError> {
    count
        .checked_mul(size_of::<T>())
        .filter(|&bytes| bytes <= isize::MAX as usize)
        .ok_or(MarchingSquaresError::CapacityOverflow)
        .map(|_| ())
}

fn buffer<T>(count: usize) -> Result<Vec<T>, MarchingSquaresError> {
    checked_capacity::<T>(count)?;
    let mut values = Vec::new();
    values
        .try_reserve_exact(count)
        .map_err(|_| MarchingSquaresError::AllocationFailed { elements: count })?;
    Ok(values)
}

/// Construct the exact filled region of a marching-squares polygonal proposal.
///
/// Crossings linearly interpolate the exact edge samples. Diagonal cases use
/// the sign of the four-corner sum (the bilinear center), with zero inside.
/// This defines a deterministic chord proposal, not the exact bilinear level
/// curve or a fidelity certificate for an underlying continuous field.
/// The native arrangement assigns material and holes using nesting parity.
/// Open boundaries, undecided decisions and failed admission stay typed.
///
/// Like other principal curve operations, this is strict by default; use
/// [`crate::evaluate_under`] or [`crate::provisional`] to select and observe
/// approximate predicate consumption. Preview tolerances are suspended.
/// Primary buffers use checked reservations. Scalar and arrangement work,
/// secondary allocations and cancellation are not internally bounded.
pub fn marching_squares(
    grid: &MarchingSquaresGrid<'_>,
) -> Result<CurveRegion2, MarchingSquaresError> {
    let policy = if crate::policy::principal_context().selects_approximate_512() {
        CurveContext::APPROXIMATE_512
    } else {
        CurveContext::STRICT
    };
    crate::evaluate_under(policy.predicate_policy(), || extract(grid, &policy)).into_unverified()
}

fn sign(
    value: &Real,
    policy: &CurveContext,
    operation: &'static str,
    index: usize,
) -> Result<RealSign, MarchingSquaresError> {
    crate::classify::real_sign(value, policy)
        .ok_or(MarchingSquaresError::PredicateUndecided { operation, index })
}

fn inside(sign: RealSign) -> bool {
    sign != RealSign::Positive
}

fn crossing(
    a: Point2,
    b: Point2,
    va: &Real,
    vb: &Real,
    sa: RealSign,
    sb: RealSign,
) -> Result<Option<Point2>, MarchingSquaresError> {
    if inside(sa) == inside(sb) {
        return Ok(None);
    }
    if sa == RealSign::Zero {
        return Ok(Some(a));
    }
    if sb == RealSign::Zero {
        return Ok(Some(b));
    }
    let t = (-va / (vb - va)).map_err(MarchingSquaresError::Scalar)?;
    Ok(Some(Point2::new(
        a.x() + &t * (b.x() - a.x()),
        a.y() + t * (b.y() - a.y()),
    )))
}

const fn ambiguous_edge_pairs(case_index: u8, center_inside: bool) -> [(usize, usize); 2] {
    if (case_index == 5) == center_inside {
        [(0, 1), (2, 3)]
    } else {
        [(0, 3), (1, 2)]
    }
}

fn extract(
    grid: &MarchingSquaresGrid<'_>,
    policy: &CurveContext,
) -> Result<CurveRegion2, MarchingSquaresError> {
    let [nx, ny] = [grid.x.len(), grid.y.len()];
    let count = MarchingSquaresGrid::checked_sample_count([nx, ny])?;
    if grid.values.len() != count {
        return Err(MarchingSquaresError::SampleCountMismatch {
            expected: count,
            actual: grid.values.len(),
        });
    }
    for (axis, values) in [grid.x, grid.y].into_iter().enumerate() {
        for (index, pair) in values.windows(2).enumerate() {
            match crate::classify::compare_reals(&pair[0], &pair[1], &CurveContext::STRICT) {
                Some(Ordering::Less) => {}
                Some(_) => return Err(MarchingSquaresError::AxisNotIncreasing { axis, index }),
                None => {
                    return Err(MarchingSquaresError::PredicateUndecided {
                        operation: "axis order",
                        index,
                    });
                }
            }
        }
    }
    let mut signs = buffer(count)?;
    for (index, value) in grid.values.iter().enumerate() {
        signs.push(sign(value, policy, "sample sign", index)?);
    }
    let index = |x: usize, y: usize| y * nx + x;
    for y in 0..ny {
        for x in 0..nx {
            if (x == 0 || y == 0 || x == nx - 1 || y == ny - 1) && inside(signs[index(x, y)]) {
                return Err(MarchingSquaresError::BoundaryNotExterior { sample: [x, y] });
            }
        }
    }
    // Each grid edge owns one exact crossing, shared by its incident cells.
    // This retains endpoint identity instead of reconstructing opposite traversals.
    let mut horizontal = buffer((nx - 1) * ny)?;
    let mut vertical = buffer(nx * (ny - 1))?;
    let point = |x: usize, y: usize| Point2::new(grid.x[x].clone(), grid.y[y].clone());
    for y in 0..ny {
        for x in 0..nx - 1 {
            let [a, b] = [index(x, y), index(x + 1, y)];
            horizontal.push(crossing(
                point(x, y),
                point(x + 1, y),
                &grid.values[a],
                &grid.values[b],
                signs[a],
                signs[b],
            )?);
        }
    }
    for y in 0..ny - 1 {
        for x in 0..nx {
            let [a, b] = [index(x, y), index(x, y + 1)];
            vertical.push(crossing(
                point(x, y),
                point(x, y + 1),
                &grid.values[a],
                &grid.values[b],
                signs[a],
                signs[b],
            )?);
        }
    }
    let max_segments = (nx - 1)
        .checked_mul(ny - 1)
        .and_then(|n| n.checked_mul(2))
        .ok_or(MarchingSquaresError::CapacityOverflow)?;
    let mut segments = buffer(max_segments)?;
    let mut emit = |a: &Point2, b: &Point2| -> Result<(), MarchingSquaresError> {
        // A zero corner can produce a zero-length proposal chord. Removing
        // that isolated point is regularization, not a tolerance merge.
        if a != b {
            segments.push(Segment2::Line(
                LineSeg2::try_new(a.clone(), b.clone()).map_err(MarchingSquaresError::Curve)?,
            ));
        }
        Ok(())
    };
    for y in 0..ny - 1 {
        for x in 0..nx - 1 {
            let corners = [
                index(x, y),
                index(x + 1, y),
                index(x + 1, y + 1),
                index(x, y + 1),
            ];
            let case = corners.iter().enumerate().fold(0u8, |bits, (i, &corner)| {
                bits | (u8::from(inside(signs[corner])) << i)
            });
            let edges = [
                &horizontal[y * (nx - 1) + x],
                &vertical[y * nx + x + 1],
                &horizontal[(y + 1) * (nx - 1) + x],
                &vertical[y * nx + x],
            ];
            let mut crossings = edges.into_iter().flatten();
            let Some(a) = crossings.next() else {
                continue;
            };
            let b = crossings
                .next()
                .expect("a closed cell has an even crossing count");
            if matches!(case, 5 | 10) {
                let c = crossings
                    .next()
                    .expect("diagonal cells have four crossings");
                let d = crossings
                    .next()
                    .expect("diagonal cells have four crossings");
                let points = [a, b, c, d];
                let center = corners
                    .into_iter()
                    .fold(Real::zero(), |sum, i| sum + &grid.values[i]);
                let center_inside = inside(sign(&center, policy, "cell center", index(x, y))?);
                for (a, b) in ambiguous_edge_pairs(case, center_inside) {
                    emit(points[a], points[b])?;
                }
            } else {
                emit(a, b)?;
            }
        }
    }
    CurveRegion2::arrange_unordered_segments(&segments, FillRule::EvenOdd)
        .map_err(MarchingSquaresError::Region)
}

#[cfg(test)]
mod tests {
    use super::ambiguous_edge_pairs;
    #[test]
    fn retained_diagonal_rule_uses_the_cell_center() {
        assert_eq!(ambiguous_edge_pairs(5, true), [(0, 1), (2, 3)]);
        assert_eq!(ambiguous_edge_pairs(5, false), [(0, 3), (1, 2)]);
        assert_eq!(ambiguous_edge_pairs(10, true), [(0, 3), (1, 2)]);
        assert_eq!(ambiguous_edge_pairs(10, false), [(0, 1), (2, 3)]);
    }
}
