//--- Copyright (C) 2025 Saki Komikado <komietty@gmail.com>,
//--- This Source Code Form is subject to the terms of the Mozilla Public License v.2.0.

#![allow(clippy::too_many_arguments)]
#![allow(clippy::cast_abs_to_unsigned)]
#![allow(unused_braces)]

#[cfg(feature = "legacy-boolean")]
mod boolean03;
#[cfg(feature = "legacy-boolean")]
mod boolean45;
#[cfg(feature = "legacy-boolean")]
mod common;
#[cfg(feature = "exact")]
pub mod exact;
#[cfg(feature = "legacy-boolean")]
mod manifold;
#[cfg(feature = "legacy-boolean")]
mod simplification;
#[cfg(all(test, feature = "legacy-boolean"))]
mod tests;
#[cfg(feature = "legacy-boolean")]
mod triangulation;

#[cfg(feature = "legacy-boolean")]
use crate::boolean03::boolean03;
#[cfg(feature = "legacy-boolean")]
use crate::boolean45::boolean45;
#[cfg(feature = "legacy-boolean")]
use crate::common::*;
#[cfg(feature = "legacy-boolean")]
use crate::manifold::*;
#[cfg(feature = "legacy-boolean")]
use crate::simplification::simplify_topology;
#[cfg(feature = "legacy-boolean")]
use crate::triangulation::triangulate;

#[cfg(feature = "legacy-boolean")]
pub use crate::common::{K_PRECISION, Mat3, Real, Vec2, Vec3, Vec4};

pub mod prelude {
    #[cfg(feature = "legacy-boolean")]
    pub use crate::common::OpType;
    #[cfg(feature = "legacy-boolean")]
    pub use crate::compute_boolean;
    #[cfg(feature = "exact")]
    pub use crate::exact::{ExactMesh, ExactPoint3, MeshFacts, Triangle};
    #[cfg(feature = "legacy-boolean")]
    pub use crate::manifold::Manifold;
}

/// Compute a legacy mesh boolean over closed manifold triangle meshes.
///
/// This entry point is compiled only with the `legacy-boolean` feature. It is
/// the boolmesh-derived adapter and still uses tolerance-based construction
/// internally; exact-topology callers should use `crate::exact::ExactMesh`
/// validation and the future exact boolean pipeline instead. Keeping this path
/// feature-gated makes approximate runtime topology an explicit opt-in, in the
/// spirit of Yap's exact-geometric-computation split between edge adapters and
/// certified decisions.
#[cfg(feature = "legacy-boolean")]
pub fn compute_boolean(mp: &Manifold, mq: &Manifold, op: OpType) -> Result<Manifold, String> {
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

    Manifold::new_impl(
        b45.ps,
        trg.hs
            .chunks(3)
            .map(|hs| Vec3u::new(hs[0].tail, hs[1].tail, hs[2].tail))
            .collect(),
        Some(eps),
        Some(tol),
    )
}
