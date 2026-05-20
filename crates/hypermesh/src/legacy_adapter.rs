//--- Copyright (C) 2025 Saki Komikado <komietty@gmail.com>,
//--- This Source Code Form is subject to the terms of the Mozilla Public License v.2.0.

//! Compatibility entry points for the gated legacy boolean adapter.
//!
//! This module owns the boolmesh-derived primitive-float path. It is kept
//! separate from the crate root so the public surface can clearly distinguish
//! approximate compatibility from exact, replayable topology. Yap, "Towards
//! Exact Geometric Computation," *Computational Geometry* 7.1-2 (1997),
//! treats exact geometric decisions as a separate contract from numerical edge
//! adapters; these entry points therefore return an explicit report whenever
//! callers still cross the legacy adapter boundary.

use crate::boolean03::boolean03;
use crate::boolean45::boolean45;
use crate::common::{OpType, Vec3u};
use crate::legacy_report::{LegacyBooleanReport, LegacyBooleanResult};
use crate::manifold::{Manifold, cleanup_unused_verts};
use crate::simplification::simplify_topology;
use crate::triangulation::triangulate;

/// Compute a legacy mesh boolean and return an explicit adapter report.
///
/// This is the preferred compatibility entry point for callers that still need
/// the boolmesh-derived path while the exact pipeline is being finished. The
/// output is not a Yap-style exact certificate; it is an approximate adapter
/// result with a report that validates the chosen primitive-float boundary.
pub fn compute_boolean_with_report(
    mp: &Manifold,
    mq: &Manifold,
    op: OpType,
) -> Result<LegacyBooleanResult, String> {
    let eps = mp.eps.max(mq.eps);
    let tol = mp.tol.max(mq.tol);

    let b03 = boolean03(mp, mq, &op);
    let mut b45 = boolean45(mp, mq, &b03, &op);
    let mut trg = triangulate(mp, mq, &b45, eps)?;

    simplify_topology(
        &mut trg.hs,
        &mut b45.ps,
        &mut trg.ns,
        &mut trg.rs,
        b45.nv_from_p,
        b45.nv_from_q,
        eps,
    );

    cleanup_unused_verts(&mut b45.ps, &mut trg.hs);

    let mesh = Manifold::new_impl(
        b45.ps,
        trg.hs
            .chunks(3)
            .map(|hs| Vec3u::new(hs[0].tail, hs[1].tail, hs[2].tail))
            .collect(),
        Some(eps),
        Some(tol),
    )?;
    let result = LegacyBooleanResult {
        report: LegacyBooleanReport {
            operation: op,
            left_vertices: mp.nv,
            left_faces: mp.nf,
            right_vertices: mq.nv,
            right_faces: mq.nf,
            output_vertices: mesh.nv,
            output_faces: mesh.nf,
            epsilon: eps,
            tolerance: tol,
            used_primitive_float_adapter: true,
        },
        mesh,
    };
    result
        .validate_operation_against_inputs(mp, mq, op)
        .map_err(|error| format!("legacy boolean adapter report validation failed: {error:?}"))?;
    Ok(result)
}
