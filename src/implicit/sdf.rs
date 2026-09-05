//! Sample signed-distance fields with native-real Surface Nets into
//! [`hypermesh::TriangleMesh`] geometry.

use crate::hyper_math::{
    hreal_from_f32, hreal_from_f64, hreal_sign, hreal_to_f64, hvector3_from_point3,
};
use crate::{
    GeometryContext, GeometryOutcome, context::GeometryDecisions, errors::ValidationError,
};
use hyperlattice::{Point3, Real};
use hyperlimit::Point3 as HPoint3;
use hypermesh::{SurfaceNetsError, SurfaceNetsGrid, TriangleMesh, surface_nets};
use hyperreal::RealSign;
use hypersdf::{
    Sdf, SdfExpr, SdfGridSamplingReport, SdfMeshPreviewReport, SdfMetricStatus,
    SdfPreviewGrid, SdfPreviewSample, SdfSampleTopologyStatus, SdfSamplingPrecision,
    SdfSamplingReport,
};

/// Diagnostics captured while sampling and meshing an SDF.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SdfDiagnostics {
    pub resolution: (u32, u32, u32),
    pub sample_count: usize,
    pub finite_sample_count: usize,
    pub non_finite_sample_count: usize,
    pub negative_sample_count: usize,
    pub zero_sample_count: usize,
    pub positive_sample_count: usize,
    pub min_finite_value: Option<Real>,
    pub max_finite_value: Option<Real>,
    pub crossing_cell_count: usize,
    pub surface_nets_vertex_count: usize,
    pub surface_nets_index_count: usize,
    pub emitted_triangle_count: usize,
    pub skipped_non_finite_triangle_count: usize,
    pub degenerate_triangle_count: usize,
    /// Lossy Hypersdf preview of the shifted field `f(point) - iso_value`.
    pub hypersdf_preview: Option<SdfMeshPreviewReport>,
}

#[derive(Debug)]
struct SdfPreviewSamples {
    samples: Vec<SdfPreviewSample>,
    non_finite_count: usize,
    negative_count: usize,
    zero_count: usize,
    positive_count: usize,
    unknown_sign_count: usize,
}

impl SdfPreviewSamples {
    fn with_capacity(capacity: usize) -> Self {
        Self {
            samples: Vec::with_capacity(capacity),
            non_finite_count: 0,
            negative_count: 0,
            zero_count: 0,
            positive_count: 0,
            unknown_sign_count: 0,
        }
    }

    fn push(&mut self, point: HPoint3, shifted: Option<&Real>) {
        match shifted.and_then(hreal_sign) {
            Some(RealSign::Negative) => self.negative_count += 1,
            Some(RealSign::Zero) => self.zero_count += 1,
            Some(RealSign::Positive) => self.positive_count += 1,
            None => self.unknown_sign_count += 1,
        }
        // Preserve exact signs across the preview boundary so its active-cell
        // partition matches the native-real mesh proposal.
        let value = shifted.and_then(surface_nets_scalar).map(f64::from);
        if value.is_none() {
            self.non_finite_count += 1;
        }
        self.samples.push(SdfPreviewSample { point, value });
    }

    fn into_grid_report(
        self,
        grid: SdfPreviewGrid,
        metric_status: SdfMetricStatus,
    ) -> SdfGridSamplingReport {
        let sample_count = self.samples.len();
        SdfGridSamplingReport {
            grid,
            samples: SdfSamplingReport {
                precision: SdfSamplingPrecision::F32,
                metric_status,
                topology_status: SdfSampleTopologyStatus::PreviewOnly,
                sample_count,
                non_finite_count: self.non_finite_count,
                negative_count: self.negative_count,
                zero_count: self.zero_count,
                positive_count: self.positive_count,
                unknown_sign_count: self.unknown_sign_count,
                samples: self.samples,
            },
        }
    }
}

fn validated_resolution(resolution: (usize, usize, usize)) -> Option<(u32, u32, u32, usize)> {
    let nx = u32::try_from(resolution.0.max(2)).ok()?;
    let ny = u32::try_from(resolution.1.max(2)).ok()?;
    let nz = u32::try_from(resolution.2.max(2)).ok()?;
    let sample_count = nx.checked_mul(ny)?.checked_mul(nz)?;
    Some((nx, ny, nz, sample_count as usize))
}

pub(crate) fn sample_with_context(
    mut field: impl FnMut(&Point3) -> Result<Real, ValidationError>,
    resolution: (usize, usize, usize),
    min: Point3,
    max: Point3,
    iso_value: Real,
    collect_diagnostics: bool,
    context: &GeometryContext,
) -> Result<GeometryOutcome<(TriangleMesh, SdfDiagnostics)>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let (nx, ny, nz, sample_count) =
        validated_resolution(resolution).ok_or(ValidationError::InvalidArguments)?;
    let grid = SamplingGrid::from_bounds(min, max, nx, ny, nz, iso_value)
        .ok_or(ValidationError::InvalidArguments)?;
    let mut diagnostics = SdfDiagnostics {
        resolution: (nx, ny, nz),
        sample_count,
        ..SdfDiagnostics::default()
    };
    let mut values = Vec::with_capacity(sample_count);
    let x_coordinates = grid.axis_coordinates(&grid.origin.x, &grid.step.x, nx);
    let y_coordinates = grid.axis_coordinates(&grid.origin.y, &grid.step.y, ny);
    let z_coordinates = grid.axis_coordinates(&grid.origin.z, &grid.step.z, nz);
    for z in &z_coordinates {
        for y in &y_coordinates {
            for x in &x_coordinates {
                let value = field(&Point3::new(x.clone(), y.clone(), z.clone()))?;
                let shifted = &value - &grid.iso;
                if collect_diagnostics {
                    match decisions.sign(&shifted, "SDF sample sign")? {
                        RealSign::Negative => diagnostics.negative_sample_count += 1,
                        RealSign::Zero => diagnostics.zero_sample_count += 1,
                        RealSign::Positive => diagnostics.positive_sample_count += 1,
                    }
                    diagnostics.finite_sample_count += 1;
                    diagnostics.min_finite_value =
                        Some(match diagnostics.min_finite_value.take() {
                            Some(current) => decisions
                                .decide(
                                    hyperlimit::real_min(
                                        &current,
                                        &value,
                                        decisions.predicate_policy(),
                                    ),
                                    "SDF minimum sample",
                                )?
                                .clone(),
                            None => value.clone(),
                        });
                    diagnostics.max_finite_value =
                        Some(match diagnostics.max_finite_value.take() {
                            Some(current) => decisions
                                .decide(
                                    hyperlimit::real_max(
                                        &current,
                                        &value,
                                        decisions.predicate_policy(),
                                    ),
                                    "SDF maximum sample",
                                )?
                                .clone(),
                            None => value,
                        });
                }
                values.push(shifted);
            }
        }
    }
    let step = grid.step.to_vector();
    let output = decisions.consume_mesh(
        surface_nets(
            decisions.mesh_context(),
            SurfaceNetsGrid::new(&grid.origin, &step, [nx, ny, nz], &values),
        )
        .map_err(|error| ValidationError::Geometry(error.to_string()))?,
    );
    diagnostics.crossing_cell_count = output.active_cell_count;
    diagnostics.surface_nets_vertex_count = output.mesh.positions.len();
    diagnostics.surface_nets_index_count = output.mesh.triangles.len() * 3;
    diagnostics.emitted_triangle_count = output.mesh.triangles.len();
    Ok(decisions.finish((output.mesh, diagnostics)))
}

pub(crate) fn sdf(
    field: impl Fn(&Point3) -> Real,
    resolution: (usize, usize, usize),
    min_pt: Point3,
    max_pt: Point3,
    iso_value: Real,
) -> TriangleMesh {
    sdf_with_indexed_sampler(
        |_, _, _, x, y, z| field(&Point3::new(x.clone(), y.clone(), z.clone())),
        resolution,
        min_pt,
        max_pt,
        iso_value,
        false,
    )
    .0
}

pub(super) fn sdf_indexed(
    field: impl FnMut(usize, usize, usize, &Real, &Real, &Real) -> Real,
    resolution: (usize, usize, usize),
    min_pt: Point3,
    max_pt: Point3,
    iso_value: Real,
) -> TriangleMesh {
    sdf_with_indexed_sampler(field, resolution, min_pt, max_pt, iso_value, false).0
}

pub(crate) fn sdf_expr(
    expression: SdfExpr,
    resolution: (usize, usize, usize),
    min_pt: Point3,
    max_pt: Point3,
    iso_value: Real,
) -> TriangleMesh {
    sdf_expr_sampled(expression, resolution, min_pt, max_pt, iso_value, false).0
}

pub(crate) fn sdf_expr_with_diagnostics(
    expr: SdfExpr,
    resolution: (usize, usize, usize),
    min_pt: Point3,
    max_pt: Point3,
    iso_value: Real,
) -> (TriangleMesh, SdfDiagnostics) {
    sdf_expr_sampled(expr, resolution, min_pt, max_pt, iso_value, true)
}

fn sdf_expr_sampled(
    expression: SdfExpr,
    resolution: (usize, usize, usize),
    min_pt: Point3,
    max_pt: Point3,
    iso_value: Real,
    collect_diagnostics: bool,
) -> (TriangleMesh, SdfDiagnostics) {
    let Some((nx, ny, nz, sample_count)) = validated_resolution(resolution) else {
        return (
            TriangleMesh::new(Vec::new(), Vec::new()),
            SdfDiagnostics::default(),
        );
    };
    let mut diagnostics = SdfDiagnostics {
        resolution: (nx, ny, nz),
        sample_count,
        ..SdfDiagnostics::default()
    };

    let preview_metric_status = collect_diagnostics.then(|| {
        if matches!(hreal_sign(&iso_value), Some(RealSign::Zero)) {
            expression.metric_status()
        } else {
            expression.clone().offset(iso_value.clone()).metric_status()
        }
    });
    let sdf = Sdf::new(expression);
    let Some(grid) =
        SamplingGrid::from_bounds(min_pt.clone(), max_pt.clone(), nx, ny, nz, iso_value)
    else {
        diagnostics.non_finite_sample_count = diagnostics.sample_count;
        return (TriangleMesh::new(Vec::new(), Vec::new()), diagnostics);
    };
    let preview_grid = collect_diagnostics
        .then(|| SdfPreviewGrid::new(grid.origin.clone(), grid.step.clone(), [nx, ny, nz]));
    let mut preview_samples =
        collect_diagnostics.then(|| SdfPreviewSamples::with_capacity(sample_count));
    let mut field_values = SdfSampleField::with_capacity(sample_count);
    let x_coordinates = grid.axis_coordinates(&grid.origin.x, &grid.step.x, nx);
    let y_coordinates = grid.axis_coordinates(&grid.origin.y, &grid.step.y, ny);
    let z_coordinates = grid.axis_coordinates(&grid.origin.z, &grid.step.z, nz);

    for z in &z_coordinates {
        for y in &y_coordinates {
            for x in &x_coordinates {
                let point = HPoint3::new(x.clone(), y.clone(), z.clone());
                let value = sdf.classify_point(&point).scalar_value;
                if let Some(samples) = &mut preview_samples {
                    let shifted = value.as_ref().map(|value| value.clone() - grid.iso.clone());
                    samples.push(point, shifted.as_ref());
                    push_sdf_sample(&mut diagnostics, &mut field_values, value, &grid.iso);
                } else {
                    push_sdf_sample_without_diagnostics(&mut field_values, value, &grid.iso);
                }
            }
        }
    }

    if let (Some(samples), Some(preview_grid), Some(metric_status)) =
        (preview_samples, preview_grid, preview_metric_status)
    {
        diagnostics.hypersdf_preview = Some(SdfMeshPreviewReport::surface_nets_diagnostic(
            samples.into_grid_report(preview_grid, metric_status),
        ));
    }
    mesh_from_sampled_field(
        field_values,
        min_pt,
        max_pt,
        nx,
        ny,
        nz,
        diagnostics,
        collect_diagnostics,
    )
}

pub(crate) fn sdf_with_diagnostics<F>(
    sdf: F,
    resolution: (usize, usize, usize),
    min_pt: Point3,
    max_pt: Point3,
    iso_value: Real,
) -> (TriangleMesh, SdfDiagnostics)
where
    F: Fn(&Point3) -> Real,
{
    sdf_with_indexed_sampler(
        |_, _, _, x, y, z| sdf(&Point3::new(x.clone(), y.clone(), z.clone())),
        resolution,
        min_pt,
        max_pt,
        iso_value,
        true,
    )
}

fn sdf_with_indexed_sampler<F>(
    mut sdf: F,
    resolution: (usize, usize, usize),
    min_pt: Point3,
    max_pt: Point3,
    iso_value: Real,
    collect_diagnostics: bool,
) -> (TriangleMesh, SdfDiagnostics)
where
    F: FnMut(usize, usize, usize, &Real, &Real, &Real) -> Real,
{
    let Some((nx, ny, nz, sample_count)) = validated_resolution(resolution) else {
        return (
            TriangleMesh::new(Vec::new(), Vec::new()),
            SdfDiagnostics::default(),
        );
    };
    let mut diagnostics = SdfDiagnostics {
        resolution: (nx, ny, nz),
        sample_count,
        ..SdfDiagnostics::default()
    };

    if hvector3_from_point3(&min_pt).is_none()
        || hvector3_from_point3(&max_pt).is_none()
        || hreal_from_f64(iso_value.clone()).is_err()
    {
        diagnostics.non_finite_sample_count = diagnostics.sample_count;
        return (TriangleMesh::new(Vec::new(), Vec::new()), diagnostics);
    }

    let array_size = sample_count;
    let mut field_values = SdfSampleField::with_capacity(array_size);
    let Some(grid) =
        SamplingGrid::from_bounds(min_pt.clone(), max_pt.clone(), nx, ny, nz, iso_value)
    else {
        diagnostics.non_finite_sample_count = diagnostics.sample_count;
        return (TriangleMesh::new(Vec::new(), Vec::new()), diagnostics);
    };
    let x_coordinates = grid.axis_coordinates(&grid.origin.x, &grid.step.x, nx);
    let y_coordinates = grid.axis_coordinates(&grid.origin.y, &grid.step.y, ny);
    let z_coordinates = grid.axis_coordinates(&grid.origin.z, &grid.step.z, nz);

    // Store `f(p) - iso` in x-major order, matching `GridShape::linearize`.
    for (iz, z) in z_coordinates.iter().enumerate() {
        for (iy, y) in y_coordinates.iter().enumerate() {
            for (ix, x) in x_coordinates.iter().enumerate() {
                let sdf_val = sdf(ix, iy, iz, x, y, z);
                let value = hreal_from_f64(sdf_val).ok();
                if collect_diagnostics {
                    push_sdf_sample(&mut diagnostics, &mut field_values, value, &grid.iso);
                } else {
                    push_sdf_sample_without_diagnostics(&mut field_values, value, &grid.iso);
                }
            }
        }
    }

    mesh_from_sampled_field(
        field_values,
        min_pt,
        max_pt,
        nx,
        ny,
        nz,
        diagnostics,
        collect_diagnostics,
    )
}

#[derive(Clone, Debug)]
struct SdfSampleField {
    values: Vec<Real>,
    failed: bool,
}

impl SdfSampleField {
    fn with_capacity(capacity: usize) -> Self {
        Self {
            values: Vec::with_capacity(capacity),
            failed: false,
        }
    }

    fn push_hyper_sample(&mut self, shifted: Real) {
        self.values.push(shifted);
    }

    const fn push_nonfinite_sample(&mut self) {
        self.failed = true;
    }
}

#[derive(Clone, Debug)]
struct SamplingGrid {
    origin: HPoint3,
    step: HPoint3,
    iso: Real,
}

impl SamplingGrid {
    fn from_bounds(
        min_pt: Point3,
        max_pt: Point3,
        nx: u32,
        ny: u32,
        nz: u32,
        iso_value: Real,
    ) -> Option<Self> {
        let origin = HPoint3::new(min_pt.x.clone(), min_pt.y.clone(), min_pt.z.clone());
        let max = HPoint3::new(max_pt.x.clone(), max_pt.y.clone(), max_pt.z.clone());
        let step = HPoint3::new(
            ((max.x - origin.x.clone()) / Real::from(u64::from(nx - 1))).ok()?,
            ((max.y - origin.y.clone()) / Real::from(u64::from(ny - 1))).ok()?,
            ((max.z - origin.z.clone()) / Real::from(u64::from(nz - 1))).ok()?,
        );
        Some(Self {
            origin,
            step,
            iso: hreal_from_f64(iso_value).ok()?,
        })
    }

    fn axis_coordinates(&self, origin: &Real, step: &Real, count: u32) -> Vec<Real> {
        (0..count)
            .map(|index| origin.clone() + step.clone() * Real::from(u64::from(index)))
            .collect()
    }
}

fn push_sdf_sample(
    diagnostics: &mut SdfDiagnostics,
    field_values: &mut SdfSampleField,
    value: Option<Real>,
    iso_value: &Real,
) {
    if let Some(sdf_val) = value {
        let shifted = sdf_val.clone() - iso_value.clone();
        let Some(sign) = hreal_sign(&shifted) else {
            field_values.failed = true;
            diagnostics.non_finite_sample_count += 1;
            return;
        };
        field_values.push_hyper_sample(shifted);
        diagnostics.finite_sample_count += 1;
        record_sdf_finite_sample(diagnostics, &sdf_val);
        match sign {
            RealSign::Negative => diagnostics.negative_sample_count += 1,
            RealSign::Positive => diagnostics.positive_sample_count += 1,
            RealSign::Zero => diagnostics.zero_sample_count += 1,
        }
    } else {
        diagnostics.non_finite_sample_count += 1;
        field_values.push_nonfinite_sample();
    }
}

fn push_sdf_sample_without_diagnostics(
    field_values: &mut SdfSampleField,
    value: Option<Real>,
    iso_value: &Real,
) {
    if let Some(sdf_val) = value {
        field_values.push_hyper_sample(sdf_val - iso_value.clone());
    } else {
        field_values.push_nonfinite_sample();
    }
}

fn record_sdf_finite_sample(diagnostics: &mut SdfDiagnostics, value: &Real) {
    diagnostics.min_finite_value = match diagnostics.min_finite_value.take() {
        Some(current) => hyperlimit::real_min(&current, value, crate::PREDICATE_POLICY)
            .value()
            .cloned(),
        None if diagnostics.finite_sample_count == 1 => Some(value.clone()),
        None => None,
    };
    diagnostics.max_finite_value = match diagnostics.max_finite_value.take() {
        Some(current) => hyperlimit::real_max(&current, value, crate::PREDICATE_POLICY)
            .value()
            .cloned(),
        None if diagnostics.finite_sample_count == 1 => Some(value.clone()),
        None => None,
    };
}

fn mesh_from_sampled_field(
    field_values: SdfSampleField,
    min_pt: Point3,
    max_pt: Point3,
    nx: u32,
    ny: u32,
    nz: u32,
    mut diagnostics: SdfDiagnostics,
    collect_diagnostics: bool,
) -> (TriangleMesh, SdfDiagnostics) {
    let expected_samples = (nx as usize)
        .checked_mul(ny as usize)
        .and_then(|count| count.checked_mul(nz as usize));
    if field_values.failed || expected_samples != Some(field_values.values.len()) {
        if diagnostics.non_finite_sample_count == 0 {
            diagnostics.non_finite_sample_count = diagnostics.sample_count;
        }
        return (TriangleMesh::new(Vec::new(), Vec::new()), diagnostics);
    }
    let Some(grid) = SamplingGrid::from_bounds(min_pt, max_pt, nx, ny, nz, Real::zero())
    else {
        diagnostics.non_finite_sample_count = diagnostics.sample_count;
        return (TriangleMesh::new(Vec::new(), Vec::new()), diagnostics);
    };

    let step = grid.step.to_vector();
    let sampled_grid =
        SurfaceNetsGrid::new(&grid.origin, &step, [nx, ny, nz], &field_values.values);
    let output = match surface_nets(&crate::MESH_CONTEXT, sampled_grid) {
        Ok(outcome) => outcome.value,
        Err(SurfaceNetsError::DegenerateTriangle { .. }) => {
            if collect_diagnostics {
                diagnostics.degenerate_triangle_count += 1;
            }
            return (TriangleMesh::new(Vec::new(), Vec::new()), diagnostics);
        },
        Err(_) => return (TriangleMesh::new(Vec::new(), Vec::new()), diagnostics),
    };
    diagnostics.crossing_cell_count = output.active_cell_count;
    diagnostics.surface_nets_vertex_count = output.mesh.positions.len();
    diagnostics.surface_nets_index_count = output.mesh.triangles.len() * 3;
    diagnostics.emitted_triangle_count = output.mesh.triangles.len();
    (output.mesh, diagnostics)
}

fn surface_nets_scalar(value: &Real) -> Option<f32> {
    let boundary = hreal_to_f64(value)?;
    // The diagnostic preview preserves the native proposal's exact sign
    // partition even when its primitive magnitude underflows or overflows f32.
    let sign = hreal_sign(value)?;
    let value = boundary as f32;
    let value = if value == 0.0 {
        match sign {
            RealSign::Negative => -f32::MIN_POSITIVE,
            RealSign::Positive => f32::MIN_POSITIVE,
            RealSign::Zero => 0.0,
        }
    } else if value.is_infinite() {
        match sign {
            RealSign::Negative => -f32::MAX,
            RealSign::Positive => f32::MAX,
            RealSign::Zero => 0.0,
        }
    } else {
        value
    };
    hreal_from_f32(value).ok()?;
    Some(value)
}
