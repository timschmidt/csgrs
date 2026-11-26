//! Metaball modeling over Hypersdf's retained field and the shared native sampler.

use crate::{
    GeometryContext, GeometryOutcome, context::GeometryDecisions, errors::ValidationError,
};
use hyperlattice::{Aabb, Point3, Real};
use hypermesh::TriangleMesh;
use hypersdf::MetaballField;

/// Retained inverse-square source. Evaluation is fallible and never uses a cutoff or pole sentinel.
pub use hypersdf::Metaball as MetaBall;

/// Diagnostics for the normalized field `(iso - I)/(iso + I)` and its native mesh.
/// Negative samples are inside. Extrema describe this continuous normalized field,
/// including its exact value -1 at centers, rather than singular inverse-square influence.
/// Counts and extrema use the selected meshing policy. They do not certify the
/// finite grid's fidelity to the continuous field; failed sampling returns no report.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct MetaballDiagnostics {
    /// Actual bordered dimensions for nonempty sources; requested dimensions
    /// when an empty source list bypasses sampling.
    pub resolution: (u32, u32, u32),
    /// Number of retained inverse-square sources.
    pub ball_count: usize,
    /// Actual bordered grid point count, or zero for no sources.
    pub sample_count: usize,
    /// Samples successfully sign-classified for diagnostics; no binary64
    /// representability test is performed. Zero when diagnostics are disabled.
    pub finite_sample_count: usize,
    /// Compatibility counter left zero; undecided samples return an error.
    pub non_finite_sample_count: usize,
    /// Normalized samples classified below zero when diagnostics are enabled.
    pub negative_sample_count: usize,
    /// Normalized samples classified as zero when diagnostics are enabled.
    pub zero_sample_count: usize,
    /// Normalized samples classified above zero when diagnostics are enabled.
    pub positive_sample_count: usize,
    /// Selected normalized-field minimum; `None` without sampled diagnostics.
    pub min_finite_value: Option<Real>,
    /// Selected normalized-field maximum; `None` without sampled diagnostics.
    pub max_finite_value: Option<Real>,
    /// Grid cells with both negative and nonnegative sample corners.
    pub crossing_cell_count: usize,
    /// Position rows returned by Surface Nets, including unused rows.
    pub surface_nets_vertex_count: usize,
    /// Three times the number of emitted triangles.
    pub surface_nets_index_count: usize,
    /// Triangle rows in the native sampled proposal.
    pub emitted_triangle_count: usize,
    /// Always zero for successful native extraction, which never skips a
    /// triangle because of failed floating-point projection.
    pub skipped_non_finite_triangle_count: usize,
    /// Always zero for successful extraction; detected degeneracy returns an error.
    pub degenerate_triangle_count: usize,
}

impl MetaballDiagnostics {
    /// Transfer shared sampler counts and extrema, attaching the source count.
    /// Metaball sampling does not request the expression-only Hypersdf preview.
    fn from_sampling(ball_count: usize, samples: super::sdf::SdfDiagnostics) -> Self {
        Self {
            ball_count,
            resolution: samples.resolution,
            sample_count: samples.sample_count,
            finite_sample_count: samples.finite_sample_count,
            non_finite_sample_count: samples.non_finite_sample_count,
            negative_sample_count: samples.negative_sample_count,
            zero_sample_count: samples.zero_sample_count,
            positive_sample_count: samples.positive_sample_count,
            min_finite_value: samples.min_finite_value,
            max_finite_value: samples.max_finite_value,
            crossing_cell_count: samples.crossing_cell_count,
            surface_nets_vertex_count: samples.surface_nets_vertex_count,
            surface_nets_index_count: samples.surface_nets_index_count,
            emitted_triangle_count: samples.emitted_triangle_count,
            skipped_non_finite_triangle_count: samples.skipped_non_finite_triangle_count,
            degenerate_triangle_count: samples.degenerate_triangle_count,
        }
    }
}

/// Validate a normalized inverse-square field and sample its conservative bounds.
///
/// Checks requested and bordered grid counts before strict field/padding
/// validation, even for no sources. Nonempty fields add one exterior sample on
/// each side of every axis, preserving the requested grid spacing, and evaluate
/// strictly before policy-selected diagnostics and Surface Nets extraction.
/// Evaluation errors carry bordered grid indices and their Hypersdf cause.
///
/// No sources yields canonical empty geometry with requested resolution and
/// zero diagnostic counts after validation. Successful meshes retain sampling
/// provenance; the local receipt is not a continuous-field topology certificate.
pub(crate) fn sample(
    balls: &[MetaBall],
    resolution: (usize, usize, usize),
    iso_value: Real,
    padding: Real,
    collect_diagnostics: bool,
    context: GeometryContext,
) -> Result<GeometryOutcome<(TriangleMesh, MetaballDiagnostics)>, ValidationError> {
    let (nx, ny, nz, _) = super::sdf::validated_resolution(resolution)?;
    let bordered =
        [nx, ny, nz].map(|n| n.checked_add(2).ok_or(ValidationError::CapacityOverflow));
    let [bx, by, bz] = bordered;
    let resolution = (bx? as usize, by? as usize, bz? as usize);
    super::sdf::validated_resolution(resolution)?;
    let field = MetaballField::new(balls, iso_value)?;
    let Some(bounds) = field.sampling_bounds(&padding)? else {
        return Ok(GeometryDecisions::new(context).finish((
            crate::solid::empty(),
            MetaballDiagnostics {
                resolution: (nx, ny, nz),
                ..MetaballDiagnostics::default()
            },
        )));
    };
    let bounds = add_sampling_border(bounds, [nx, ny, nz])?;
    // Retain the established grid locations, including its one-cell exterior
    // border. Extraction now uses native Real coordinates and standalone faces.
    super::sdf::sample_indexed_with_context(
        |ix, iy, iz, x, y, z| {
            field
                .evaluate_point(&Point3::new(x.clone(), y.clone(), z.clone()))
                .map_err(|source| ValidationError::SdfSample {
                    sample: [ix, iy, iz],
                    source,
                })
        },
        resolution,
        bounds.mins,
        bounds.maxs,
        Real::zero(),
        collect_diagnostics,
        context,
    )
    .map(|outcome| {
        outcome.map(|(mesh, samples)| {
            (mesh, MetaballDiagnostics::from_sampling(balls.len(), samples))
        })
    })
}

/// Expand each bound by one original grid step `(max - min) / (count - 1)`.
/// Counts must already be >= 2. Sampling the new bounds with two extra samples
/// per axis preserves the original locations and spacing exactly.
fn add_sampling_border(bounds: Aabb, [nx, ny, nz]: [u32; 3]) -> Result<Aabb, ValidationError> {
    let step = |min: &Real, max: &Real, count: u32| (max - min) / Real::from(count - 1);
    let dx = step(&bounds.mins.x, &bounds.maxs.x, nx)?;
    let dy = step(&bounds.mins.y, &bounds.maxs.y, ny)?;
    let dz = step(&bounds.mins.z, &bounds.maxs.z, nz)?;
    Ok(Aabb::new(
        Point3::new(bounds.mins.x - &dx, bounds.mins.y - &dy, bounds.mins.z - &dz),
        Point3::new(bounds.maxs.x + dx, bounds.maxs.y + dy, bounds.maxs.z + dz),
    ))
}
