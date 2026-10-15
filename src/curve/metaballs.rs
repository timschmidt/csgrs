//! Planar sampling of the native Hypersdf field into native Hypercurve contours.

use crate::{
    GeometryContext, GeometryOutcome, context::GeometryDecisions, errors::ValidationError,
};
use hypercurve::{CurveRegion2, MarchingSquaresGrid, Point2};
use hyperlattice::{Point3, Real};
use hypersdf::{Metaball, MetaballField};

pub(crate) fn sample(
    balls: &[(Point2, Real)],
    resolution: (usize, usize),
    iso_value: Real,
    padding: Real,
    context: &GeometryContext,
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
