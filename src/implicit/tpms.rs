//! Triply‑Periodic Minimal Surfaces generated from signed-distance fields.

use crate::hyper_math::{
    hreal_abs, hreal_from_f64, hreal_gt_f64, hreal_max, hreal_sign, hreal_sub, tau,
};
use hyperlattice::{Aabb, Point3, Real};
use hypermesh::{Triangle, TriangleMesh};
use hyperreal::RealSign;
use std::collections::HashMap;
use std::sync::OnceLock;

fn invalid_sdf_value() -> Real {
    hreal_from_f64(1.0e10).expect("finite SDF sentinel")
}

fn cubic_resolution(resolution: usize) -> Option<(usize, usize, usize)> {
    let resolution = resolution.max(2);
    let finite = u32::try_from(resolution).ok()?;
    finite.checked_mul(finite)?.checked_mul(finite)?;
    Some((resolution, resolution, resolution))
}

#[inline]
fn tpms_from_indexed_sdf(
    bounds: &Aabb,
    sdf_fn: impl FnMut(usize, usize, usize, &Real, &Real, &Real) -> Real,
    resolution: (usize, usize, usize),
    iso_value: Real,
) -> TriangleMesh {
    super::sdf::sdf_indexed(
        sdf_fn,
        resolution,
        bounds.mins.clone(),
        bounds.maxs.clone(),
        iso_value,
    )
}

/// Build a capped, finite-thickness TPMS solid inside `self`'s bounding box.
///
/// The implicit solid is:
///
/// `max(abs(f(p) - iso_value) - thickness / 2, box_sdf(p)) <= 0`
///
/// This creates a wall around the TPMS sheet and caps it where it meets the
/// bounding box. That makes a closed mesh suitable for solid workflows,
/// unlike the raw zero-level TPMS sheet.
#[inline]
fn tpms_solid_from_sdf(
    bounds: &Aabb,
    sdf_fn: impl Fn(&Point3) -> Real,
    resolution: (usize, usize, usize),
    iso_value: Real,
    thickness: Real,
) -> TriangleMesh {
    let Some(thickness_h) = hreal_from_f64(thickness).ok() else {
        return hypermesh::TriangleMesh::new(Vec::new(), Vec::new());
    };
    if !hreal_gt_f64(&thickness_h, 0.0) {
        return hypermesh::TriangleMesh::new(Vec::new(), Vec::new());
    }

    let Some(half) = hreal_from_f64(0.5).ok() else {
        return hypermesh::TriangleMesh::new(Vec::new(), Vec::new());
    };
    let half_thickness = thickness_h.clone() * half;
    let counts = [
        resolution.0.max(2),
        resolution.1.max(2),
        resolution.2.max(2),
    ];
    let spans = [
        bounds.maxs.x.clone() - bounds.mins.x.clone(),
        bounds.maxs.y.clone() - bounds.mins.y.clone(),
        bounds.maxs.z.clone() - bounds.mins.z.clone(),
    ];
    let Some(steps) = (0..3)
        .map(|axis| (spans[axis].clone() / Real::from(counts[axis] as u64)).ok())
        .collect::<Option<Vec<_>>>()
    else {
        return TriangleMesh::new(Vec::new(), Vec::new());
    };
    let [Some(sample_x), Some(sample_y), Some(sample_z)] =
        counts.map(|count| count.checked_add(2))
    else {
        return TriangleMesh::new(Vec::new(), Vec::new());
    };
    let sample_counts = [sample_x, sample_y, sample_z];
    let Some(half_steps) = steps
        .iter()
        .map(|step| (step.clone() / Real::from(2_u8)).ok())
        .collect::<Option<Vec<_>>>()
    else {
        return TriangleMesh::new(Vec::new(), Vec::new());
    };
    let origins = [
        bounds.mins.x.clone() - half_steps[0].clone(),
        bounds.mins.y.clone() - half_steps[1].clone(),
        bounds.mins.z.clone() - half_steps[2].clone(),
    ];
    let Some(sample_capacity) = sample_counts
        .iter()
        .try_fold(1_usize, |product, count| product.checked_mul(*count))
    else {
        return TriangleMesh::new(Vec::new(), Vec::new());
    };
    let mut samples = Vec::with_capacity(sample_capacity);
    let mut values = Vec::with_capacity(samples.capacity());
    for z in 0..sample_counts[2] {
        for y in 0..sample_counts[1] {
            for x in 0..sample_counts[0] {
                let point = Point3::new(
                    origins[0].clone() + steps[0].clone() * Real::from(x as u64),
                    origins[1].clone() + steps[1].clone() * Real::from(y as u64),
                    origins[2].clone() + steps[2].clone() * Real::from(z as u64),
                );
                let sheet = hreal_sub(sdf_fn(&point), iso_value.clone())
                    .and_then(hreal_abs)
                    .and_then(|distance| hreal_sub(distance, half_thickness.clone()))
                    .unwrap_or_else(invalid_sdf_value);
                let clip = hreal_max(&[
                    bounds.mins.x.clone() - point.x.clone(),
                    point.x.clone() - bounds.maxs.x.clone(),
                    bounds.mins.y.clone() - point.y.clone(),
                    point.y.clone() - bounds.maxs.y.clone(),
                    bounds.mins.z.clone() - point.z.clone(),
                    point.z.clone() - bounds.maxs.z.clone(),
                ])
                .unwrap_or_else(invalid_sdf_value);
                samples.push(point);
                values.push(hreal_max(&[sheet, clip]).unwrap_or_else(invalid_sdf_value));
            }
        }
    }
    tetrahedral_isosurface(&samples, &values, sample_counts)
}

fn tetrahedral_isosurface(
    samples: &[Point3],
    values: &[Real],
    dimensions: [usize; 3],
) -> TriangleMesh {
    fn crossing_vertex(
        a: usize,
        b: usize,
        samples: &[Point3],
        values: &[Real],
        positions: &mut Vec<Point3>,
        edge_vertices: &mut HashMap<(usize, usize), usize>,
    ) -> Option<usize> {
        let key = (a.min(b), a.max(b));
        if let Some(&index) = edge_vertices.get(&key) {
            return Some(index);
        }
        let denominator = values[a].clone() - values[b].clone();
        let t = (values[a].clone() / denominator).ok()?;
        let point = Point3::new(
            samples[a].x.clone() + t.clone() * (samples[b].x.clone() - samples[a].x.clone()),
            samples[a].y.clone() + t.clone() * (samples[b].y.clone() - samples[a].y.clone()),
            samples[a].z.clone() + t * (samples[b].z.clone() - samples[a].z.clone()),
        );
        let index = positions.len();
        positions.push(point);
        edge_vertices.insert(key, index);
        Some(index)
    }

    fn emit_oriented(
        indices: [usize; 3],
        direction: &hyperlattice::Vector3,
        positions: &[Point3],
        triangles: &mut Vec<Triangle>,
    ) {
        let ab = &positions[indices[1]] - &positions[indices[0]];
        let ac = &positions[indices[2]] - &positions[indices[0]];
        let reverse = hreal_sign(&ab.cross(&ac).dot(direction)) == Some(RealSign::Negative);
        triangles.push(if reverse {
            Triangle::new(indices[0], indices[2], indices[1])
        } else {
            Triangle::new(indices[0], indices[1], indices[2])
        });
    }

    const TETRAHEDRA: [[usize; 4]; 6] = [
        [0, 1, 3, 7],
        [0, 3, 2, 7],
        [0, 2, 6, 7],
        [0, 6, 4, 7],
        [0, 4, 5, 7],
        [0, 5, 1, 7],
    ];
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
                    let (inside, outside): (Vec<_>, Vec<_>) =
                        vertices.into_iter().partition(|&index| {
                            hreal_sign(&values[index]) == Some(RealSign::Negative)
                        });
                    if inside.is_empty() || outside.is_empty() {
                        continue;
                    }
                    let direction = &samples[outside[0]] - &samples[inside[0]];
                    match inside.len() {
                        1 => {
                            let i = inside[0];
                            let Some(indices) = outside
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
                                .collect::<Option<Vec<_>>>()
                            else {
                                continue;
                            };
                            emit_oriented(
                                [indices[0], indices[1], indices[2]],
                                &direction,
                                &positions,
                                &mut triangles,
                            );
                        },
                        3 => {
                            let o = outside[0];
                            let Some(indices) = inside
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
                                .collect::<Option<Vec<_>>>()
                            else {
                                continue;
                            };
                            emit_oriented(
                                [indices[0], indices[1], indices[2]],
                                &direction,
                                &positions,
                                &mut triangles,
                            );
                        },
                        2 => {
                            let Some(a) = crossing_vertex(
                                inside[0],
                                outside[0],
                                samples,
                                values,
                                &mut positions,
                                &mut edge_vertices,
                            ) else {
                                continue;
                            };
                            let Some(b) = crossing_vertex(
                                inside[0],
                                outside[1],
                                samples,
                                values,
                                &mut positions,
                                &mut edge_vertices,
                            ) else {
                                continue;
                            };
                            let Some(c) = crossing_vertex(
                                inside[1],
                                outside[1],
                                samples,
                                values,
                                &mut positions,
                                &mut edge_vertices,
                            ) else {
                                continue;
                            };
                            let Some(d) = crossing_vertex(
                                inside[1],
                                outside[0],
                                samples,
                                values,
                                &mut positions,
                                &mut edge_vertices,
                            ) else {
                                continue;
                            };
                            emit_oriented([a, b, c], &direction, &positions, &mut triangles);
                            emit_oriented([a, c, d], &direction, &positions, &mut triangles);
                        },
                        _ => unreachable!("a tetrahedron has four vertices"),
                    }
                }
            }
        }
    }
    TriangleMesh::new(positions, triangles)
}

// ------------  Specific minimal‑surface flavours  --------------------

/// Gyroid surface:  `sin x cos y + sin y cos z + sin z cos x = iso`
/// after scaling coordinates by `2π / period`.
///
/// `period` is a spatial wavelength in model units; larger values repeat
/// more slowly.
/// **Mathematical Foundation**: Gyroid is a triply periodic minimal surface with zero mean curvature.
/// **Optimization**: Pre-compute trigonometric values for better performance.
pub fn gyroid(
    bounds: &Aabb,
    resolution: usize,
    period: Real,
    iso_value: Real,
) -> TriangleMesh {
    let Some(res) = cubic_resolution(resolution) else {
        return TriangleMesh::new(Vec::new(), Vec::new());
    };
    let Some(scale) = tpms_scale(period) else {
        return hypermesh::TriangleMesh::new(Vec::new(), Vec::new());
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

/// Generate a capped, finite-thickness gyroid solid inside `self`'s
/// bounding box.
pub fn gyroid_solid(
    bounds: &Aabb,
    resolution: usize,
    period: Real,
    iso_value: Real,
    thickness: Real,
) -> TriangleMesh {
    let Some(res) = cubic_resolution(resolution) else {
        return TriangleMesh::new(Vec::new(), Vec::new());
    };
    let Some(scale) = tpms_scale(period) else {
        return hypermesh::TriangleMesh::new(Vec::new(), Vec::new());
    };
    tpms_solid_from_sdf(
        bounds,
        move |p: &Point3| {
            tpms_gyroid_value(p, scale.clone()).unwrap_or_else(invalid_sdf_value)
        },
        res,
        iso_value,
        thickness,
    )
}

/// Schwarz‑P surface:  `cos x + cos y + cos z = iso`  (default iso = 0)
/// after scaling coordinates by `2π / period`.
/// **Mathematical Foundation**: Schwarz P-surface has constant mean curvature and cubic symmetry.
/// **Optimization**: Use direct cosine computation for this simpler surface equation.
pub fn schwarz_p(
    bounds: &Aabb,
    resolution: usize,
    period: Real,
    iso_value: Real,
) -> TriangleMesh {
    let Some(res) = cubic_resolution(resolution) else {
        return TriangleMesh::new(Vec::new(), Vec::new());
    };
    let Some(scale) = tpms_scale(period) else {
        return hypermesh::TriangleMesh::new(Vec::new(), Vec::new());
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

/// Generate a capped, finite-thickness Schwarz-P solid inside `self`'s
/// bounding box.
pub fn schwarz_p_solid(
    bounds: &Aabb,
    resolution: usize,
    period: Real,
    iso_value: Real,
    thickness: Real,
) -> TriangleMesh {
    let Some(res) = cubic_resolution(resolution) else {
        return TriangleMesh::new(Vec::new(), Vec::new());
    };
    let Some(scale) = tpms_scale(period) else {
        return hypermesh::TriangleMesh::new(Vec::new(), Vec::new());
    };
    tpms_solid_from_sdf(
        bounds,
        move |p: &Point3| {
            tpms_schwarz_p_value(p, scale.clone()).unwrap_or_else(invalid_sdf_value)
        },
        res,
        iso_value,
        thickness,
    )
}

/// Schwarz‑D (Diamond) surface:  `sin x sin y sin z + sin x cos y cos z + ... = iso`
/// after scaling coordinates by `2π / period`.
/// **Mathematical Foundation**: Diamond surface exhibits tetrahedral symmetry and is self-intersecting.
/// **Optimization**: Pre-compute all trigonometric values for maximum efficiency.
pub fn schwarz_d(
    bounds: &Aabb,
    resolution: usize,
    period: Real,
    iso_value: Real,
) -> TriangleMesh {
    let Some(res) = cubic_resolution(resolution) else {
        return TriangleMesh::new(Vec::new(), Vec::new());
    };
    let Some(scale) = tpms_scale(period) else {
        return hypermesh::TriangleMesh::new(Vec::new(), Vec::new());
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

/// Generate a capped, finite-thickness Schwarz-D solid inside `self`'s
/// bounding box.
pub fn schwarz_d_solid(
    bounds: &Aabb,
    resolution: usize,
    period: Real,
    iso_value: Real,
    thickness: Real,
) -> TriangleMesh {
    let Some(res) = cubic_resolution(resolution) else {
        return TriangleMesh::new(Vec::new(), Vec::new());
    };
    let Some(scale) = tpms_scale(period) else {
        return hypermesh::TriangleMesh::new(Vec::new(), Vec::new());
    };
    tpms_solid_from_sdf(
        bounds,
        move |p: &Point3| {
            tpms_schwarz_d_value(p, scale.clone()).unwrap_or_else(invalid_sdf_value)
        },
        res,
        iso_value,
        thickness,
    )
}

/// Return the TPMS angular scale `2π / period` in hyperreal space.
///
/// The integer/grid APIs still expose primitive periods, but the wavelength
/// validation and reciprocal are promoted before any implicit-field samples are
/// evaluated. This follows Yap's exact-geometric-computation split between
/// primitive input boundaries and exact-aware predicates
/// (<https://doi.org/10.1016/0925-7721(95)00040-2>). The TPMS families here
/// follow Schoen, "Infinite Periodic Minimal Surfaces Without Self-
/// Intersections," NASA Technical Note D-5541, 1970.
fn tpms_scale(period: Real) -> Option<Real> {
    let period = hreal_from_f64(period).ok()?;
    if !hreal_gt_f64(&period, 0.0) {
        return None;
    }
    (tau() / period).ok()
}

fn tpms_scaled_axes(point: &Point3, scale: Real) -> Option<(Real, Real, Real)> {
    Some((
        point.x.clone() * scale.clone(),
        point.y.clone() * scale.clone(),
        point.z.clone() * scale,
    ))
}

fn empty_axis_cache<T>(length: usize) -> Vec<OnceLock<T>> {
    std::iter::repeat_with(OnceLock::new).take(length).collect()
}

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

/// Evaluate Schoen's gyroid approximation in hyperreal space.
fn tpms_gyroid_value(point: &Point3, scale: Real) -> Option<Real> {
    let (x, y, z) = tpms_scaled_axes(point, scale)?;
    let sin_x = x.clone().sin();
    let cos_x = x.cos();
    let sin_y = y.clone().sin();
    let cos_y = y.cos();
    let sin_z = z.clone().sin();
    let cos_z = z.cos();
    Some(sin_x * cos_y + sin_y * cos_z + sin_z * cos_x)
}

/// Evaluate Schwarz's primitive cubic surface approximation in hyperreal space.
fn tpms_schwarz_p_value(point: &Point3, scale: Real) -> Option<Real> {
    let (x, y, z) = tpms_scaled_axes(point, scale)?;
    Some(x.cos() + y.cos() + z.cos())
}

/// Evaluate Schwarz's diamond surface approximation in hyperreal space.
fn tpms_schwarz_d_value(point: &Point3, scale: Real) -> Option<Real> {
    let (x, y, z) = tpms_scaled_axes(point, scale)?;
    let sin_x = x.clone().sin();
    let cos_x = x.cos();
    let sin_y = y.clone().sin();
    let cos_y = y.cos();
    let sin_z = z.clone().sin();
    let cos_z = z.cos();
    Some(
        sin_x.clone() * sin_y.clone() * sin_z.clone()
            + sin_x * cos_y.clone() * cos_z.clone()
            + cos_x.clone() * sin_y * cos_z
            + cos_x * cos_y * sin_z,
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::hyper_math::tolerance;

    #[test]
    fn tpms_scale_accepts_any_exact_positive_period() {
        assert!(tpms_scale(tolerance() * 0.25).is_some());
        assert!(tpms_scale(tolerance()).is_some());
        assert!(tpms_scale(Real::one()).is_some());
    }

    #[test]
    fn tpms_scale_rejects_zero_and_negative_periods() {
        assert!(tpms_scale(Real::zero()).is_none());
        assert!(tpms_scale(-tolerance()).is_none());
    }
}
