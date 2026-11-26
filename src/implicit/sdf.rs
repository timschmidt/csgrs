//! Sample scalar fields with native-real Surface Nets into triangle geometry.
//!
//! Field values need not be signed distances. The finite grid defines a sampled
//! proposal, with sampling provenance and policy-qualified decisions; it does
//! not certify continuous-field topology or cap the grid boundary. Optional
//! Hypersdf previews cross a separate lossy boundary and never drive extraction.

use crate::hyper_math::{hreal_from_f32, hreal_sign, hreal_to_f64};
use crate::{
    GeometryContext, GeometryOutcome, context::GeometryDecisions, errors::ValidationError,
};
use hyperlattice::{Point3, Real};
use hyperlimit::Point3 as HPoint3;
use hypermesh::{SurfaceNetsGrid, TriangleMesh, surface_nets_standalone};
use hyperreal::RealSign;
#[cfg(feature = "sdf")]
use hypersdf::{Sdf, SdfExpr};
use hypersdf::{
    SdfGridSamplingReport, SdfMeshPreviewReport, SdfMetricStatus, SdfPreviewGrid,
    SdfPreviewSample, SdfSampleTopologyStatus, SdfSamplingPrecision, SdfSamplingReport,
};

/// Diagnostics captured while sampling and meshing an SDF.
///
/// Sign buckets classify `field(point) - iso_value` under the meshing policy;
/// extrema select unshifted field values under that same policy. These are
/// sampling statistics, not a field-fidelity or topology certificate. Native
/// sampling returns an error rather than a partial report on failure.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SdfDiagnostics {
    /// Actual sample dimensions, including any border supplied by the caller.
    pub resolution: (u32, u32, u32),
    /// Product of the validated sample dimensions, even without diagnostics.
    pub sample_count: usize,
    /// Samples whose diagnostic sign classification succeeded. This does not
    /// test binary64 representability; zero when diagnostics are disabled.
    pub finite_sample_count: usize,
    /// Retained compatibility counter; native sampling leaves it zero and fails
    /// on undecided samples instead of discarding them as nonfinite.
    pub non_finite_sample_count: usize,
    /// Samples classified below the iso value when diagnostics are enabled.
    pub negative_sample_count: usize,
    /// Samples classified at the iso value when diagnostics are enabled.
    pub zero_sample_count: usize,
    /// Samples classified above the iso value when diagnostics are enabled.
    pub positive_sample_count: usize,
    /// Selected minimum unshifted field value, or `None` without diagnostics.
    pub min_finite_value: Option<Real>,
    /// Selected maximum unshifted field value, or `None` without diagnostics.
    pub max_finite_value: Option<Real>,
    /// Cells with both negative and nonnegative sample corners.
    pub crossing_cell_count: usize,
    /// Position rows returned by native Surface Nets, including unused rows.
    pub surface_nets_vertex_count: usize,
    /// Three times the number of emitted triangles.
    pub surface_nets_index_count: usize,
    /// Triangle rows returned by native Surface Nets.
    pub emitted_triangle_count: usize,
    /// Always zero in successful native sampling; triangles are not skipped on
    /// projection failure because extraction never projects to floating point.
    pub skipped_non_finite_triangle_count: usize,
    /// Always zero in successful native sampling; detected degeneracy is an error.
    pub degenerate_triangle_count: usize,
    /// Lossy expression preview of `f(point) - iso_value`, present only when
    /// expression diagnostics are requested. Its sign buckets require strict
    /// proofs independently of the main counts' policy; coordinates remain exact.
    pub hypersdf_preview: Option<SdfMeshPreviewReport>,
}

/// Optional expression-preview samples and strict-sign counters, separate from meshing.
#[derive(Debug)]
struct SdfPreviewSamples {
    /// X-fast sample sequence with exact locations and optional projected values.
    samples: Vec<SdfPreviewSample>,
    /// Missing preview values, including undecided signs and failed projection.
    non_finite_count: usize,
    /// Shifted field samples proved negative under strict policy.
    negative_count: usize,
    /// Shifted field samples proved zero under strict policy.
    zero_count: usize,
    /// Shifted field samples proved positive under strict policy.
    positive_count: usize,
    /// Samples with no supplied value or no strict sign proof.
    unknown_sign_count: usize,
}

impl SdfPreviewSamples {
    /// Reserve the preview rows with checked inline storage and clear all counts.
    fn with_capacity(capacity: usize) -> Result<Self, ValidationError> {
        Ok(Self {
            samples: crate::solid::reserved(crate::solid::capacity::<SdfPreviewSample>(
                Some(capacity),
            )?)?,
            non_finite_count: 0,
            negative_count: 0,
            zero_count: 0,
            positive_count: 0,
            unknown_sign_count: 0,
        })
    }

    /// Record a strict sign and a sign-preserving lossy preview, without failing
    /// meshing when preview data is unavailable. Missing values and unknown signs
    /// remain explicit counts; the preview never supplies a geometric decision.
    fn push(&mut self, point: HPoint3, shifted: Option<&Real>) {
        match shifted.and_then(hreal_sign) {
            Some(RealSign::Negative) => self.negative_count += 1,
            Some(RealSign::Zero) => self.zero_count += 1,
            Some(RealSign::Positive) => self.positive_count += 1,
            None => self.unknown_sign_count += 1,
        }
        // Preserve strict signs across the preview boundary. A meshing policy
        // may admit approximate signs, but this report's sign buckets promise
        // strict evidence. Unknown signs therefore withhold preview samples.
        let value = shifted.and_then(surface_nets_scalar).map(f64::from);
        if value.is_none() {
            self.non_finite_count += 1;
        }
        self.samples.push(SdfPreviewSample { point, value });
    }

    /// Package collected rows with their supplied grid and shifted-field metric
    /// status. Values use F32 precision stored as f64; topology stays preview-only.
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

/// Require at least two samples per axis and a total count representable in u32.
///
/// Returns dimensions and their product as a host index. This checks count
/// arithmetic only; element storage and downstream allocations need separate
/// checks. Validation runs before field evaluation, including for empty output.
pub(super) fn validated_resolution(
    resolution: (usize, usize, usize),
) -> Result<(u32, u32, u32, usize), ValidationError> {
    for (name, actual) in [
        ("x resolution", resolution.0),
        ("y resolution", resolution.1),
        ("z resolution", resolution.2),
    ] {
        if actual < 2 {
            return Err(ValidationError::InvalidResolution {
                name,
                min: 2,
                actual,
            });
        }
    }
    let nx = u32::try_from(resolution.0).map_err(|_| ValidationError::CapacityOverflow)?;
    let ny = u32::try_from(resolution.1).map_err(|_| ValidationError::CapacityOverflow)?;
    let nz = u32::try_from(resolution.2).map_err(|_| ValidationError::CapacityOverflow)?;
    // Surface Nets indexes the grid with u32 products.
    let count = nx
        .checked_mul(ny)
        .and_then(|n| n.checked_mul(nz))
        .ok_or(ValidationError::CapacityOverflow)?;
    let count = usize::try_from(count).map_err(|_| ValidationError::CapacityOverflow)?;
    Ok((nx, ny, nz, count))
}

/// Evaluate an owned Hypersdf expression strictly, then mesh with the selected policy.
///
/// Evaluation failures retain the sample's `[x, y, z]` indices and native cause.
/// Requested diagnostics include the shifted expression's metric status and
/// strict-sign preview. A nonzero or undecided iso value uses offset-expression
/// metadata; this metadata does not certify the extracted surface.
#[cfg(feature = "sdf")]
pub(crate) fn sample_expr_with_context(
    expression: SdfExpr,
    resolution: (usize, usize, usize),
    min: Point3,
    max: Point3,
    iso_value: Real,
    collect_diagnostics: bool,
    context: GeometryContext,
) -> Result<GeometryOutcome<(TriangleMesh, SdfDiagnostics)>, ValidationError> {
    let preview_metric_status = collect_diagnostics.then(|| {
        if matches!(hreal_sign(&iso_value), Some(RealSign::Zero)) {
            expression.metric_status()
        } else {
            expression.clone().offset(iso_value.clone()).metric_status()
        }
    });
    let sdf = Sdf::new(expression);
    sample_indexed(
        |ix, iy, iz, x, y, z| {
            sdf.evaluate_point(&HPoint3::new(x.clone(), y.clone(), z.clone()))
                .map_err(|source| ValidationError::SdfSample {
                    sample: [ix, iy, iz],
                    source,
                })
        },
        resolution,
        min,
        max,
        iso_value,
        collect_diagnostics,
        preview_metric_status,
        context,
    )
}

/// Adapt a fallible point callback to the shared indexed sampler.
///
/// Callback errors propagate as supplied; unlike the expression adapter, this
/// adds no sample-index wrapper or Hypersdf metric/preview report.
#[cfg(feature = "sdf")]
pub(crate) fn sample_with_context(
    mut field: impl FnMut(&Point3) -> Result<Real, ValidationError>,
    resolution: (usize, usize, usize),
    min: Point3,
    max: Point3,
    iso_value: Real,
    collect_diagnostics: bool,
    context: GeometryContext,
) -> Result<GeometryOutcome<(TriangleMesh, SdfDiagnostics)>, ValidationError> {
    sample_indexed_with_context(
        |_, _, _, x, y, z| field(&Point3::new(x.clone(), y.clone(), z.clone())),
        resolution,
        min,
        max,
        iso_value,
        collect_diagnostics,
        context,
    )
}

/// Sample a fallible indexed callback without expression-specific preview metadata.
///
/// The callback receives zero-based grid indices and exact coordinates in X-fast,
/// then Y, then Z order. Diagnostics and extraction use `context`; the callback
/// itself owns its evaluation/validation policy and failures propagate unchanged.
pub(super) fn sample_indexed_with_context(
    field: impl FnMut(usize, usize, usize, &Real, &Real, &Real) -> Result<Real, ValidationError>,
    resolution: (usize, usize, usize),
    min: Point3,
    max: Point3,
    iso_value: Real,
    collect_diagnostics: bool,
    context: GeometryContext,
) -> Result<GeometryOutcome<(TriangleMesh, SdfDiagnostics)>, ValidationError> {
    sample_indexed(
        field,
        resolution,
        min,
        max,
        iso_value,
        collect_diagnostics,
        None,
        context,
    )
}

/// Evaluate the complete regular grid and extract a standalone Surface Nets proposal.
///
/// Validates sample counts and reserves values before invoking the callback once
/// per grid point, in X-fast order. Bounds are converted to steps without checking
/// their ordering or nonzero extent. Values are shifted by `iso_value` for meshing;
/// diagnostic extrema use the original values. Optional comparisons can fail or
/// consume approximate certainty, which joins extraction certainty in the receipt.
///
/// Native extraction preserves exact coordinates and sampling provenance, without
/// capping or independently certifying closure. Any failure returns no partial
/// mesh/report. Checked reservations do not impose a total-work, memory or
/// cancellation budget on callback evaluation or downstream extraction.
fn sample_indexed(
    mut field: impl FnMut(
        usize,
        usize,
        usize,
        &Real,
        &Real,
        &Real,
    ) -> Result<Real, ValidationError>,
    resolution: (usize, usize, usize),
    min: Point3,
    max: Point3,
    iso_value: Real,
    collect_diagnostics: bool,
    preview_metric_status: Option<SdfMetricStatus>,
    context: GeometryContext,
) -> Result<GeometryOutcome<(TriangleMesh, SdfDiagnostics)>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let (nx, ny, nz, sample_count) = validated_resolution(resolution)?;
    let grid = SamplingGrid::from_bounds(min, max, nx, ny, nz, iso_value)?;
    let mut preview_samples = preview_metric_status
        .map(|_| SdfPreviewSamples::with_capacity(sample_count))
        .transpose()?;
    let mut diagnostics = SdfDiagnostics {
        resolution: (nx, ny, nz),
        sample_count,
        ..SdfDiagnostics::default()
    };
    let mut values =
        crate::solid::reserved(crate::solid::capacity::<Real>(Some(sample_count))?)?;
    let x_coordinates = grid.axis_coordinates(&grid.origin.x, &grid.step.x, nx)?;
    let y_coordinates = grid.axis_coordinates(&grid.origin.y, &grid.step.y, ny)?;
    let z_coordinates = grid.axis_coordinates(&grid.origin.z, &grid.step.z, nz)?;
    for (iz, z) in z_coordinates.iter().enumerate() {
        for (iy, y) in y_coordinates.iter().enumerate() {
            for (ix, x) in x_coordinates.iter().enumerate() {
                let value = field(ix, iy, iz, x, y, z)?;
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
                if let Some(samples) = &mut preview_samples {
                    samples
                        .push(HPoint3::new(x.clone(), y.clone(), z.clone()), Some(&shifted));
                }
                values.push(shifted);
            }
        }
    }
    let step = grid.step.to_vector();
    let output = decisions.consume_mesh(surface_nets_standalone(
        decisions.mesh_context(),
        SurfaceNetsGrid::new(&grid.origin, &step, [nx, ny, nz], &values),
    )?);
    diagnostics.crossing_cell_count = output.active_cell_count;
    diagnostics.surface_nets_vertex_count = output.mesh.positions.len();
    diagnostics.surface_nets_index_count = output.mesh.triangles.len() * 3;
    diagnostics.emitted_triangle_count = output.mesh.triangles.len();
    if let (Some(samples), Some(metric_status)) = (preview_samples, preview_metric_status) {
        let preview_grid = SdfPreviewGrid::new(grid.origin, grid.step, [nx, ny, nz]);
        diagnostics.hypersdf_preview = Some(SdfMeshPreviewReport::surface_nets_diagnostic(
            samples.into_grid_report(preview_grid, metric_status),
        ));
    }
    Ok(decisions.finish((decisions.qualify_mesh(output.mesh), diagnostics)))
}

/// Exact axis-aligned sample placement and iso shift, without bounds validation.
#[derive(Clone, Debug)]
struct SamplingGrid {
    /// Coordinate of sample `[0, 0, 0]`.
    origin: HPoint3,
    /// Per-axis displacement between adjacent samples; may be zero or negative.
    step: HPoint3,
    /// Value subtracted from each field sample before extraction.
    iso: Real,
}

impl SamplingGrid {
    /// Compute steps `(max - min) / (count - 1)` for caller-validated counts >= 2.
    /// Does not reorder bounds or classify their extents; scalar failures propagate.
    fn from_bounds(
        min_pt: Point3,
        max_pt: Point3,
        nx: u32,
        ny: u32,
        nz: u32,
        iso_value: Real,
    ) -> Result<Self, ValidationError> {
        let origin = HPoint3::new(min_pt.x, min_pt.y, min_pt.z);
        let step = HPoint3::new(
            ((max_pt.x - &origin.x) / Real::from(u64::from(nx - 1)))?,
            ((max_pt.y - &origin.y) / Real::from(u64::from(ny - 1)))?,
            ((max_pt.z - &origin.z) / Real::from(u64::from(nz - 1)))?,
        );
        Ok(Self {
            origin,
            step,
            iso: iso_value,
        })
    }

    /// Reserve and generate `origin + index * step` for the supplied axis.
    /// Includes both bounds when called with the grid's original dimension.
    fn axis_coordinates(
        &self,
        origin: &Real,
        step: &Real,
        count: u32,
    ) -> Result<Vec<Real>, ValidationError> {
        let mut values =
            crate::solid::reserved(crate::solid::capacity::<Real>(Some(count as usize))?)?;
        values.extend((0..count).map(|index| origin + &(step * &Real::from(index))));
        Ok(values)
    }
}

/// Project a diagnostic value through binary64 to F32 while preserving a strict sign.
///
/// A rounded zero with nonzero sign becomes signed `f32::MIN_POSITIVE`; F32
/// infinity becomes signed `f32::MAX`. Missing binary64 projection, undecided
/// strict sign or failed finite promotion returns `None`. These replacements
/// are preview values only, with no magnitude-error guarantee or meshing role.
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
