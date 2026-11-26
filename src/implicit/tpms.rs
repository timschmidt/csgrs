//! Sampled trigonometric level fields inspired by triply periodic minimal surfaces.
//!
//! The gyroid, Schwarz-P and Schwarz-D formulas below define the modeled fields;
//! no exact minimal-surface or signed-distance property is asserted. Level
//! surfaces use native Surface Nets. Solid variants sample a field-value band
//! clipped to a box and extract it by tetrahedral interpolation. Both paths keep
//! exact sample arithmetic and sampling provenance, without a continuous-field
//! topology or geometric-error certificate. All predicates here are strict.

use crate::hyper_math::tau;
use crate::solid::{capacity, reserved};
use crate::{
    GeometryContext, GeometryOutcome, context::GeometryDecisions, errors::ValidationError,
};
use hyperlattice::{Aabb, Point3, Real};
use hypermesh::{Triangle, TriangleMesh};
use hyperreal::RealSign;
use std::collections::HashMap;
use std::sync::OnceLock;

/// Classify under strict policy, retaining undecided failure and its stage label.
fn sign(value: &Real, predicate: &'static str) -> Result<RealSign, ValidationError> {
    GeometryDecisions::new(GeometryContext::STRICT).sign(value, predicate)
}

/// Select a maximum by strict comparisons in slice order; reject an empty slice.
/// Used for box half-space clipping and intersection with the field-value band.
fn maximum(values: &[Real]) -> Result<Real, ValidationError> {
    let (first, rest) = values
        .split_first()
        .ok_or(ValidationError::InvalidArguments)?;
    let decisions = GeometryDecisions::new(GeometryContext::STRICT);
    rest.iter().try_fold(first.clone(), |best, value| {
        decisions
            .decide(
                hyperlimit::real_max(&best, value, decisions.predicate_policy()),
                "TPMS clipping maximum",
            )
            .cloned()
    })
}

/// Require resolution >= 2 and its cube to fit u32, returning equal axis counts.
/// This checks count arithmetic, not the storage needed by bordered solid grids.
fn cubic_resolution(resolution: usize) -> Result<(usize, usize, usize), ValidationError> {
    if resolution < 2 {
        return Err(ValidationError::InvalidResolution {
            name: "resolution",
            min: 2,
            actual: resolution,
        });
    }
    let finite = u32::try_from(resolution).map_err(|_| ValidationError::CapacityOverflow)?;
    finite
        .checked_mul(finite)
        .and_then(|count| count.checked_mul(finite))
        .ok_or(ValidationError::CapacityOverflow)?;
    Ok((resolution, resolution, resolution))
}

/// Forward an indexed field to strict Surface Nets without diagnostic collection.
/// Discards the separate receipt after retaining mesh construction qualifications;
/// grid boundaries remain uncapped and callback evaluations use exact coordinates.
#[inline]
fn tpms_from_indexed_sdf(
    bounds: &Aabb,
    mut sdf_fn: impl FnMut(usize, usize, usize, &Real, &Real, &Real) -> Real,
    resolution: (usize, usize, usize),
    iso_value: Real,
) -> Result<TriangleMesh, ValidationError> {
    super::sdf::sample_indexed_with_context(
        |ix, iy, iz, x, y, z| Ok(sdf_fn(ix, iy, iz, x, y, z)),
        resolution,
        bounds.mins.clone(),
        bounds.maxs.clone(),
        iso_value,
        false,
        GeometryContext::STRICT,
    )
    .map(GeometryOutcome::into_value)
    .map(|(mesh, _)| mesh)
}

/// Sample `max(abs(f - iso) - thickness / 2, box_clip)` and extract its zero set.
///
/// `box_clip` is the maximum of the six signed axis-plane distances, so its
/// nonpositive set is the supplied box. Thickness measures field-value width,
/// not Euclidean distance. For axis count `n`, uses spacing `span / n` and `n + 2`
/// samples starting half a step below the minimum; the last is half a step above
/// the maximum. This exterior sampling supports caps from the clipping field.
///
/// Callers validate resolution first. Negative/undecided thickness fails before
/// the absent-bounds shortcut; absent bounds or zero thickness yields canonical
/// empty geometry. Checked primary sample buffers do not bound all extraction
/// allocations, scalar work or memory. No whole-mesh closure check is performed.
fn tpms_solid_from_sdf(
    bounds: Option<&Aabb>,
    sdf_fn: impl Fn(&Point3) -> Real,
    resolution: (usize, usize, usize),
    iso_value: Real,
    thickness: Real,
) -> Result<TriangleMesh, ValidationError> {
    let thickness_sign = sign(&thickness, "TPMS wall thickness")?;
    if thickness_sign == RealSign::Negative {
        return Err(ValidationError::NegativeParameter {
            name: "wall_thickness",
        });
    }
    let Some(bounds) = bounds else {
        return Ok(crate::solid::empty());
    };
    if thickness_sign == RealSign::Zero {
        return Ok(crate::solid::empty());
    }
    let half_thickness = (thickness / Real::from(2))?;
    let counts = [resolution.0, resolution.1, resolution.2];
    let spans = [
        &bounds.maxs.x - &bounds.mins.x,
        &bounds.maxs.y - &bounds.mins.y,
        &bounds.maxs.z - &bounds.mins.z,
    ];
    let steps = (0..3)
        .map(|axis| &spans[axis] / Real::from(counts[axis] as u64))
        .collect::<Result<Vec<_>, _>>()?;
    let sample_counts = counts.map(|count| count.checked_add(2));
    let [Some(sample_x), Some(sample_y), Some(sample_z)] = sample_counts else {
        return Err(ValidationError::CapacityOverflow);
    };
    let sample_counts = [sample_x, sample_y, sample_z];
    let half_steps = steps
        .iter()
        .map(|step| step / Real::from(2))
        .collect::<Result<Vec<_>, _>>()?;
    let origins = [
        &bounds.mins.x - &half_steps[0],
        &bounds.mins.y - &half_steps[1],
        &bounds.mins.z - &half_steps[2],
    ];
    let sample_capacity = capacity::<Point3>(
        sample_counts
            .iter()
            .try_fold(1_usize, |product, count| product.checked_mul(*count)),
    )?;
    capacity::<Real>(Some(sample_capacity))?;
    let mut samples = reserved(sample_capacity)?;
    let mut values = reserved(sample_capacity)?;
    for z in 0..sample_counts[2] {
        for y in 0..sample_counts[1] {
            for x in 0..sample_counts[0] {
                let point = Point3::new(
                    &origins[0] + &steps[0] * Real::from(x as u64),
                    &origins[1] + &steps[1] * Real::from(y as u64),
                    &origins[2] + &steps[2] * Real::from(z as u64),
                );
                let sheet = (sdf_fn(&point) - &iso_value).abs() - &half_thickness;
                let clip = maximum(&[
                    &bounds.mins.x - &point.x,
                    &point.x - &bounds.maxs.x,
                    &bounds.mins.y - &point.y,
                    &point.y - &bounds.maxs.y,
                    &bounds.mins.z - &point.z,
                    &point.z - &bounds.maxs.z,
                ])?;
                let value = maximum(&[sheet, clip])?;
                samples.push(point);
                values.push(value);
            }
        }
    }
    tetrahedral_isosurface(&samples, &values, sample_counts)
}

/// Extract a sampled zero set from six tetrahedra per regular-grid cell.
///
/// Validates dimensions and slice lengths, then strictly classifies all samples.
/// Negative values are inside and zero is outside. Linear edge crossings share
/// indices by undirected sample edge; distinct edges meeting at a zero sample
/// are not welded by coordinate equality. Each emitted triangle is oriented
/// toward the outside and a zero orientation returns a typed degeneracy error.
///
/// Successful output retains sampling provenance, without independent closure,
/// embedding or analytic-fidelity validation. Output buffers grow without a
/// declared work/byte budget; no partial mesh is returned on a reported error.
fn tetrahedral_isosurface(
    samples: &[Point3],
    values: &[Real],
    dimensions: [usize; 3],
) -> Result<TriangleMesh, ValidationError> {
    /// Reuse or construct the exact linear crossing at `t = va / (va - vb)`.
    /// Callers supply valid endpoints on opposite sides of the inside rule;
    /// interpolation failures propagate before a new vertex/cache entry is added.
    fn crossing_vertex(
        a: usize,
        b: usize,
        samples: &[Point3],
        values: &[Real],
        positions: &mut Vec<Point3>,
        edge_vertices: &mut HashMap<(usize, usize), usize>,
    ) -> Result<usize, ValidationError> {
        let key = (a.min(b), a.max(b));
        if let Some(&index) = edge_vertices.get(&key) {
            return Ok(index);
        }
        let denominator = values[a].clone() - values[b].clone();
        let t = (values[a].clone() / denominator)?;
        let point = Point3::new(
            samples[a].x.clone() + t.clone() * (samples[b].x.clone() - samples[a].x.clone()),
            samples[a].y.clone() + t.clone() * (samples[b].y.clone() - samples[a].y.clone()),
            samples[a].z.clone() + t * (samples[b].z.clone() - samples[a].z.clone()),
        );
        let index = positions.len();
        positions.push(point);
        edge_vertices.insert(key, index);
        Ok(index)
    }

    /// Orient one triangle so its normal has positive dot product with the
    /// supplied inside-to-outside direction. Reject zero or undecided orientation.
    fn emit_oriented(
        indices: [usize; 3],
        direction: &hyperlattice::Vector3,
        positions: &[Point3],
        triangles: &mut Vec<Triangle>,
    ) -> Result<(), ValidationError> {
        let ab = &positions[indices[1]] - &positions[indices[0]];
        let ac = &positions[indices[2]] - &positions[indices[0]];
        let reverse = match sign(&ab.cross(&ac).dot(direction), "TPMS triangle orientation")? {
            RealSign::Negative => true,
            RealSign::Positive => false,
            RealSign::Zero => {
                return Err(ValidationError::DegenerateOutputTriangle {
                    triangle_index: triangles.len(),
                });
            },
        };
        triangles.push(if reverse {
            Triangle::new(indices[0], indices[2], indices[1])
        } else {
            Triangle::new(indices[0], indices[1], indices[2])
        });
        Ok(())
    }

    /// Six tetrahedra around the cube's 0-to-7 body diagonal, using XYZ bit corners.
    const TETRAHEDRA: [[usize; 4]; 6] = [
        [0, 1, 3, 7],
        [0, 3, 2, 7],
        [0, 2, 6, 7],
        [0, 6, 4, 7],
        [0, 4, 5, 7],
        [0, 5, 1, 7],
    ];
    if dimensions.iter().any(|count| *count < 2) {
        return Err(ValidationError::InvalidArguments);
    }
    let expected_samples = dimensions
        .iter()
        .try_fold(1_usize, |product, count| product.checked_mul(*count))
        .ok_or(ValidationError::CapacityOverflow)?;
    if samples.len() != expected_samples || values.len() != expected_samples {
        return Err(ValidationError::InvalidArguments);
    }
    let signs = values
        .iter()
        .map(|value| sign(value, "TPMS sample sign"))
        .collect::<Result<Vec<_>, _>>()?;
    let sample_index =
        |x: usize, y: usize, z: usize| (z * dimensions[1] + y) * dimensions[0] + x;
    let mut positions = Vec::new();
    let mut triangles = Vec::new();
    let mut edge_vertices = HashMap::<(usize, usize), usize>::new();

    for z in 0..dimensions[2] - 1 {
        for y in 0..dimensions[1] - 1 {
            for x in 0..dimensions[0] - 1 {
                let corners = [
                    sample_index(x, y, z),
                    sample_index(x + 1, y, z),
                    sample_index(x, y + 1, z),
                    sample_index(x + 1, y + 1, z),
                    sample_index(x, y, z + 1),
                    sample_index(x + 1, y, z + 1),
                    sample_index(x, y + 1, z + 1),
                    sample_index(x + 1, y + 1, z + 1),
                ];
                for tetrahedron in TETRAHEDRA {
                    let vertices = tetrahedron.map(|corner| corners[corner]);
                    let (inside, outside): (Vec<_>, Vec<_>) = vertices
                        .into_iter()
                        .partition(|&index| signs[index] == RealSign::Negative);
                    if inside.is_empty() || outside.is_empty() {
                        continue;
                    }
                    let direction = &samples[outside[0]] - &samples[inside[0]];
                    match inside.len() {
                        1 => {
                            let i = inside[0];
                            let indices = outside
                                .iter()
                                .map(|&o| {
                                    crossing_vertex(
                                        i,
                                        o,
                                        samples,
                                        values,
                                        &mut positions,
                                        &mut edge_vertices,
                                    )
                                })
                                .collect::<Result<Vec<_>, _>>()?;
                            emit_oriented(
                                [indices[0], indices[1], indices[2]],
                                &direction,
                                &positions,
                                &mut triangles,
                            )?;
                        },
                        3 => {
                            let o = outside[0];
                            let indices = inside
                                .iter()
                                .map(|&i| {
                                    crossing_vertex(
                                        i,
                                        o,
                                        samples,
                                        values,
                                        &mut positions,
                                        &mut edge_vertices,
                                    )
                                })
                                .collect::<Result<Vec<_>, _>>()?;
                            emit_oriented(
                                [indices[0], indices[1], indices[2]],
                                &direction,
                                &positions,
                                &mut triangles,
                            )?;
                        },
                        2 => {
                            let a = crossing_vertex(
                                inside[0],
                                outside[0],
                                samples,
                                values,
                                &mut positions,
                                &mut edge_vertices,
                            )?;
                            let b = crossing_vertex(
                                inside[0],
                                outside[1],
                                samples,
                                values,
                                &mut positions,
                                &mut edge_vertices,
                            )?;
                            let c = crossing_vertex(
                                inside[1],
                                outside[1],
                                samples,
                                values,
                                &mut positions,
                                &mut edge_vertices,
                            )?;
                            let d = crossing_vertex(
                                inside[1],
                                outside[0],
                                samples,
                                values,
                                &mut positions,
                                &mut edge_vertices,
                            )?;
                            emit_oriented([a, b, c], &direction, &positions, &mut triangles)?;
                            emit_oriented([a, c, d], &direction, &positions, &mut triangles)?;
                        },
                        _ => unreachable!("a tetrahedron has four vertices"),
                    }
                }
            }
        }
    }
    Ok(TriangleMesh::from_construction(
        positions,
        triangles,
        hypermesh::ConstructionEvidence::DECLARED_COORDINATES.with_sampling(),
    ))
}

// ------------  Specific minimal‑surface flavours  --------------------

/// Sample the gyroid level field `sin x cos y + sin y cos z + sin z cos x = iso`.
///
/// Coordinates are scaled by `2π / period`, where period is a positive wavelength
/// in model units. Requires resolution >= 2 with a u32-representable cube, even
/// when bounds are absent. Absent bounds then returns canonical empty geometry.
/// A present box supplies `resolution` samples per axis including its endpoints.
///
/// Caches sine/cosine expressions per axis index for this call. Surface Nets
/// leaves the grid boundary uncapped. This trigonometric level field and its
/// sampled mesh carry no exact-minimality or analytic-fidelity guarantee.
pub fn gyroid(
    bounds: Option<&Aabb>,
    resolution: usize,
    period: Real,
    iso_value: Real,
) -> Result<TriangleMesh, ValidationError> {
    let res = cubic_resolution(resolution)?;
    let scale = tpms_scale(period)?;
    let Some(bounds) = bounds else {
        return Ok(crate::solid::empty());
    };
    let x_trig = empty_axis_cache(res.0);
    let y_trig = empty_axis_cache(res.1);
    let z_trig = empty_axis_cache(res.2);
    tpms_from_indexed_sdf(
        bounds,
        move |ix, iy, iz, x, y, z| {
            let (sin_x, cos_x) = cached_scaled_sin_cos(&x_trig, ix, x, &scale);
            let (sin_y, cos_y) = cached_scaled_sin_cos(&y_trig, iy, y, &scale);
            let (sin_z, cos_z) = cached_scaled_sin_cos(&z_trig, iz, z, &scale);
            sin_x * cos_y + sin_y * cos_z + sin_z * cos_x
        },
        res,
        iso_value,
    )
}

/// Sample a gyroid field-value band clipped to the supplied box.
///
/// Uses [`gyroid`]'s field and positive-period/resolution validation, then
/// [`tpms_solid_from_sdf`]'s bordered grid and tetrahedral extraction. Thickness
/// is nonnegative field-value width, not Euclidean wall thickness. Missing bounds
/// or zero thickness yields empty geometry after parameter validation.
pub fn gyroid_solid(
    bounds: Option<&Aabb>,
    resolution: usize,
    period: Real,
    iso_value: Real,
    thickness: Real,
) -> Result<TriangleMesh, ValidationError> {
    let res = cubic_resolution(resolution)?;
    let scale = tpms_scale(period)?;
    tpms_solid_from_sdf(
        bounds,
        move |p: &Point3| tpms_gyroid_value(p, scale.clone()),
        res,
        iso_value,
        thickness,
    )
}

/// Sample the Schwarz-P level field `cos x + cos y + cos z = iso`.
///
/// Coordinates are scaled by `2π / period`, with the same period, resolution,
/// absent-bounds and uncapped Surface Nets rules as [`gyroid`]. The iso value is
/// required explicitly. Cosine expressions are cached by axis index for this call;
/// no constant-mean-curvature or exact-minimality certificate is supplied.
pub fn schwarz_p(
    bounds: Option<&Aabb>,
    resolution: usize,
    period: Real,
    iso_value: Real,
) -> Result<TriangleMesh, ValidationError> {
    let res = cubic_resolution(resolution)?;
    let scale = tpms_scale(period)?;
    let Some(bounds) = bounds else {
        return Ok(crate::solid::empty());
    };
    let x_cos = empty_axis_cache(res.0);
    let y_cos = empty_axis_cache(res.1);
    let z_cos = empty_axis_cache(res.2);
    tpms_from_indexed_sdf(
        bounds,
        move |ix, iy, iz, x, y, z| {
            cached_scaled_cos(&x_cos, ix, x, &scale)
                + cached_scaled_cos(&y_cos, iy, y, &scale)
                + cached_scaled_cos(&z_cos, iz, z, &scale)
        },
        res,
        iso_value,
    )
}

/// Sample a Schwarz-P field-value band clipped to the supplied box.
///
/// Uses [`schwarz_p`]'s field and validation and [`tpms_solid_from_sdf`]'s
/// bordered tetrahedral extraction. Thickness is a nonnegative field-value width;
/// absent bounds or zero thickness returns empty after parameter validation.
pub fn schwarz_p_solid(
    bounds: Option<&Aabb>,
    resolution: usize,
    period: Real,
    iso_value: Real,
    thickness: Real,
) -> Result<TriangleMesh, ValidationError> {
    let res = cubic_resolution(resolution)?;
    let scale = tpms_scale(period)?;
    tpms_solid_from_sdf(
        bounds,
        move |p: &Point3| tpms_schwarz_p_value(p, scale.clone()),
        res,
        iso_value,
        thickness,
    )
}

/// Sample the Schwarz-D field `sin x sin y sin z + sin x cos y cos z
/// + cos x sin y cos z + cos x cos y sin z = iso`.
///
/// Coordinates are scaled by `2π / period`, with the same period, resolution,
/// absent-bounds and uncapped Surface Nets rules as [`gyroid`]. Sine/cosine
/// expressions are cached by axis index for this call. No claim about exact
/// minimality or absence/presence of self-intersection is inferred from the name.
pub fn schwarz_d(
    bounds: Option<&Aabb>,
    resolution: usize,
    period: Real,
    iso_value: Real,
) -> Result<TriangleMesh, ValidationError> {
    let res = cubic_resolution(resolution)?;
    let scale = tpms_scale(period)?;
    let Some(bounds) = bounds else {
        return Ok(crate::solid::empty());
    };
    let x_trig = empty_axis_cache(res.0);
    let y_trig = empty_axis_cache(res.1);
    let z_trig = empty_axis_cache(res.2);
    tpms_from_indexed_sdf(
        bounds,
        move |ix, iy, iz, x, y, z| {
            let (sin_x, cos_x) = cached_scaled_sin_cos(&x_trig, ix, x, &scale);
            let (sin_y, cos_y) = cached_scaled_sin_cos(&y_trig, iy, y, &scale);
            let (sin_z, cos_z) = cached_scaled_sin_cos(&z_trig, iz, z, &scale);
            sin_x.clone() * sin_y.clone() * sin_z.clone()
                + sin_x * cos_y.clone() * cos_z.clone()
                + cos_x.clone() * sin_y * cos_z
                + cos_x * cos_y * sin_z
        },
        res,
        iso_value,
    )
}

/// Sample a Schwarz-D field-value band clipped to the supplied box.
///
/// Uses [`schwarz_d`]'s field and validation and [`tpms_solid_from_sdf`]'s
/// bordered tetrahedral extraction. Thickness is a nonnegative field-value width;
/// absent bounds or zero thickness returns empty after parameter validation.
pub fn schwarz_d_solid(
    bounds: Option<&Aabb>,
    resolution: usize,
    period: Real,
    iso_value: Real,
    thickness: Real,
) -> Result<TriangleMesh, ValidationError> {
    let res = cubic_resolution(resolution)?;
    let scale = tpms_scale(period)?;
    tpms_solid_from_sdf(
        bounds,
        move |p: &Point3| tpms_schwarz_d_value(p, scale.clone()),
        res,
        iso_value,
        thickness,
    )
}

/// Return the TPMS angular scale `2π / period` in hyperreal space.
///
/// Requires a strict positive sign before division. There is no minimum period
/// tolerance and no projection to a primitive float; unresolved signs and scalar
/// failures remain errors.
fn tpms_scale(period: Real) -> Result<Real, ValidationError> {
    if sign(&period, "TPMS period")? != RealSign::Positive {
        return Err(ValidationError::NonPositiveParameter { name: "period" });
    }
    Ok((tau() / period)?)
}

/// Multiply model coordinates by the common angular frequency for field evaluation.
fn tpms_scaled_axes(point: &Point3, scale: Real) -> (Real, Real, Real) {
    (
        point.x.clone() * scale.clone(),
        point.y.clone() * scale.clone(),
        point.z.clone() * scale,
    )
}

/// Allocate one initially empty trigonometric cache cell per axis sample index.
/// The cache belongs to one extraction; allocation is not covered by a byte budget.
fn empty_axis_cache<T>(length: usize) -> Vec<OnceLock<T>> {
    std::iter::repeat_with(OnceLock::new).take(length).collect()
}

/// Initialize or reuse sine/cosine expressions for one axis sample.
/// The caller must keep coordinate and scale fixed for each valid index; they
/// are not part of a checked key and changing them would reuse stale expressions.
fn cached_scaled_sin_cos(
    cache: &[OnceLock<(Real, Real)>],
    index: usize,
    coordinate: &Real,
    scale: &Real,
) -> (Real, Real) {
    cache[index]
        .get_or_init(|| {
            let scaled = coordinate.clone() * scale.clone();
            (scaled.clone().sin(), scaled.cos())
        })
        .clone()
}

/// Initialize or reuse a cosine expression for one axis sample.
/// Requires an in-range index permanently associated with this coordinate/scale
/// pair during the extraction; no coordinate equality check is performed.
fn cached_scaled_cos(
    cache: &[OnceLock<Real>],
    index: usize,
    coordinate: &Real,
    scale: &Real,
) -> Real {
    cache[index]
        .get_or_init(|| (coordinate.clone() * scale.clone()).cos())
        .clone()
}

/// Evaluate `sin x cos y + sin y cos z + sin z cos x` at scaled coordinates.
fn tpms_gyroid_value(point: &Point3, scale: Real) -> Real {
    let (x, y, z) = tpms_scaled_axes(point, scale);
    let sin_x = x.clone().sin();
    let cos_x = x.cos();
    let sin_y = y.clone().sin();
    let cos_y = y.cos();
    let sin_z = z.clone().sin();
    let cos_z = z.cos();
    sin_x * cos_y + sin_y * cos_z + sin_z * cos_x
}

/// Evaluate the Schwarz-P field `cos x + cos y + cos z` at scaled coordinates.
fn tpms_schwarz_p_value(point: &Point3, scale: Real) -> Real {
    let (x, y, z) = tpms_scaled_axes(point, scale);
    x.cos() + y.cos() + z.cos()
}

/// Evaluate [`schwarz_d`]'s four-term trigonometric field at scaled coordinates.
fn tpms_schwarz_d_value(point: &Point3, scale: Real) -> Real {
    let (x, y, z) = tpms_scaled_axes(point, scale);
    let sin_x = x.clone().sin();
    let cos_x = x.cos();
    let sin_y = y.clone().sin();
    let cos_y = y.cos();
    let sin_z = z.clone().sin();
    let cos_z = z.cos();
    sin_x.clone() * sin_y.clone() * sin_z.clone()
        + sin_x * cos_y.clone() * cos_z.clone()
        + cos_x.clone() * sin_y * cos_z
        + cos_x * cos_y * sin_z
}

#[cfg(test)]
mod tests {
    //! Strict period-domain witnesses for the angular-frequency conversion.
    use super::*;
    use crate::hyper_math::tolerance;

    /// Positive fixtures below, at and above the legacy tolerance are admitted.
    #[test]
    fn tpms_scale_accepts_any_exact_positive_period() {
        assert!(tpms_scale(tolerance() * 0.25).is_ok());
        assert!(tpms_scale(tolerance()).is_ok());
        assert!(tpms_scale(Real::one()).is_ok());
    }

    /// Zero and a negative legacy-tolerance fixture fail period validation.
    #[test]
    fn tpms_scale_rejects_zero_and_negative_periods() {
        assert!(tpms_scale(Real::zero()).is_err());
        assert!(tpms_scale(-tolerance()).is_err());
    }
}
