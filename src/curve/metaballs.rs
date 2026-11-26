//! Planar sampling of the native Hypersdf field into native Hypercurve contours.
//!
//! Samples the normalized inverse-square field on a bounded XY grid and adopts
//! marching-squares chords as native filled geometry. Exact sampling decisions
//! do not certify those chords' fidelity to the continuous level set.

use crate::{
    GeometryContext, GeometryOutcome, context::GeometryDecisions, errors::ValidationError,
};
use hypercurve::{CurveRegion2, MarchingSquaresGrid, Point2};
use hyperlattice::{Point3, Real};
use hypersdf::{Metaball, MetaballField};

/// Builds a planar field grid, then extracts native contours under the requested policy.
///
/// Checks grid dimensions/count before allocating sources; native field creation
/// validates iso and radii strictly, then bounds validate padding even for no
/// sources. An empty field returns an empty region only after those checks.
/// Samples cover both ends of each axis, with X varying fastest and no extra
/// border. Field errors retain `[ix, iy, 0]` and the native cause.
///
/// Only marching squares/region admission runs in the selected curve decision
/// scope; field construction, evaluation and bounds use Hypersdf's strict
/// construction contract. The generic outcome carries extraction certainty,
/// not a continuous-field fidelity certificate or mesh construction receipt.
pub(crate) fn sample(
    balls: &[(Point2, Real)],
    resolution: (usize, usize),
    iso_value: Real,
    padding: Real,
    context: GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let (nx, ny) = resolution;
    let count = MarchingSquaresGrid::checked_sample_count([nx, ny])?;
    let decisions = GeometryDecisions::new(context);
    let mut sources = buffer(balls.len())?;
    sources.extend(balls.iter().map(|(center, radius)| {
        Metaball::new(
            Point3::new(center.x().clone(), center.y().clone(), Real::zero()),
            radius.clone(),
        )
    }));
    let field = MetaballField::new(&sources, iso_value)?;
    let Some(bounds) = field.sampling_bounds(&padding)? else {
        return Ok(decisions.finish(CurveRegion2::empty()));
    };
    // Retain the original 2D core grid, without the 3D sampler's extra border.
    let x = axis(&bounds.mins.x, &bounds.maxs.x, nx)?;
    let y = axis(&bounds.mins.y, &bounds.maxs.y, ny)?;
    let mut values = buffer(count)?;
    for (iy, y) in y.iter().enumerate() {
        for (ix, x) in x.iter().enumerate() {
            values.push(
                field
                    .evaluate_point(&Point3::new(x.clone(), y.clone(), Real::zero()))
                    .map_err(|source| ValidationError::SdfSample {
                        sample: [ix, iy, 0],
                        source,
                    })?,
            );
        }
    }
    let region = decisions.exact_curve(|| {
        hypercurve::marching_squares(&MarchingSquaresGrid {
            x: &x,
            y: &y,
            values: &values,
        })
    })?;
    Ok(decisions.finish(region))
}

/// Reserves an empty buffer after checking its requested addressable byte count.
///
/// Multiplication overflow or more than `isize::MAX` bytes is `CapacityOverflow`;
/// reservation failure is `AllocationFailed` with the element count. Length
/// remains zero, and allocator rounding or subsequent scalar allocations are
/// not bounded by this local capacity check.
fn buffer<T>(count: usize) -> Result<Vec<T>, ValidationError> {
    count
        .checked_mul(size_of::<T>())
        .filter(|&bytes| bytes <= isize::MAX as usize)
        .ok_or(ValidationError::CapacityOverflow)?;
    let mut result = Vec::new();
    result
        .try_reserve_exact(count)
        .map_err(|_| ValidationError::AllocationFailed { elements: count })?;
    Ok(result)
}

/// Equally spaced exact coordinates including `min` and `max`.
///
/// The grid validator must already establish `count >= 2`; this helper does
/// not guard the `count - 1` subtraction itself. Computes the step before
/// reserving storage and preserves conversion, scalar and allocation failures.
/// Bounds order is the caller's responsibility; no finite-coordinate projection
/// or independent interval validation is performed here.
fn axis(min: &Real, max: &Real, count: usize) -> Result<Vec<Real>, ValidationError> {
    let step = ((max - min)
        / Real::from(
            u64::try_from(count - 1).map_err(|_| ValidationError::CapacityOverflow)?,
        ))?;
    let mut values = buffer(count)?;
    for i in 0..count {
        values.push(
            min + &step
                * Real::from(u64::try_from(i).map_err(|_| ValidationError::CapacityOverflow)?),
        );
    }
    Ok(values)
}
