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
#[derive(Clone, Debug, Default, PartialEq)]
pub struct MetaballDiagnostics {
    pub resolution: (u32, u32, u32),
    pub ball_count: usize,
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
}

impl MetaballDiagnostics {
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

pub(crate) fn sample(
    balls: &[MetaBall],
    resolution: (usize, usize, usize),
    iso_value: Real,
    padding: Real,
    collect_diagnostics: bool,
    context: &GeometryContext,
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
