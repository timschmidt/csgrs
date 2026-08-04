//! Canonical exact surface-arrangement Boolean entry point.

use crate::context::{DecisionContext, MeshContext, MeshOutcome};
use crate::error::{HypermeshError, HypermeshResult};
use crate::mesh::{TriangleMeshRef, build_polygon_soup_internal};
use crate::output::BooleanMeshBatch;
use crate::surface_arrangement::{build_surface_arrangement, validate_surface_boolean_program};
use crate::winding::BooleanProgram;

/// Evaluates one or more Boolean expressions from one exact surface
/// arrangement of borrowed mesh views.
///
/// Input validation, corefinement, radial-cell construction, winding
/// propagation, expression evaluation, boundary selection, and output
/// certification all consume the policy selected by `context`. The returned
/// aggregate certainty reports whether `APPROXIMATE_512`'s terminal decision
/// was consumed anywhere in the batch. `STRICT` never consumes that terminal.
///
/// Every requested result shares one compact exact vertex arena. Supplying
/// additional roots does not repeat input intersection or corefinement.
pub fn boolean(
    context: &MeshContext,
    meshes: &[TriangleMeshRef<'_>],
    program: BooleanProgram<'_>,
) -> HypermeshResult<MeshOutcome<BooleanMeshBatch>> {
    if meshes.is_empty() {
        return Err(HypermeshError::EmptyInput);
    }
    validate_surface_boolean_program(program, meshes.len())?;

    let decisions = DecisionContext::new(context);
    crate::trace_dispatch!("boolean", "start");
    let soup = build_polygon_soup_internal(&decisions, meshes)?;
    crate::trace_dispatch!("boolean", "input-certified");
    let arrangement = build_surface_arrangement(&decisions, &soup.polygons)?;
    crate::trace_dispatch!("boolean", "surface-arranged");
    let output = arrangement.materialize_program(&decisions, &soup.polygons, program)?;
    crate::trace_dispatch!("boolean", "output-certified");
    crate::trace_dispatch!("boolean", "complete");
    Ok(decisions.finish(output))
}
