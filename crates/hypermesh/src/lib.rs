//--- Copyright (C) 2025 Saki Komikado <komietty@gmail.com>,
//--- This Source Code Form is subject to the terms of the Mozilla Public License v.2.0.

#![allow(clippy::too_many_arguments)]
#![allow(clippy::cast_abs_to_unsigned)]
#![allow(unused_braces)]

mod manifold;
mod triangulation;
mod simplification;
mod common;
mod boolean03;
mod boolean45;
mod tests;

use crate::boolean03::boolean03;
use crate::boolean45::boolean45;
use crate::simplification::simplify_topology;
use crate::triangulation::triangulate;
use crate::common::*;
use crate::manifold::*;

pub use crate::common::{Real, Vec2, Vec3, Vec4, Mat3, K_PRECISION};

pub mod prelude {
    pub use crate::common::OpType;
    pub use crate::manifold::Manifold;
    pub use crate::compute_boolean;
}

pub fn compute_boolean(
    mp: &Manifold,
    mq: &Manifold,
    op: OpType,
) -> Result<Manifold, String> {
    let eps = mp.eps.max(mq.eps);
    let tol = mp.tol.max(mq.tol);

    let     b03 = boolean03(mp, mq, &op);
    let mut b45 = boolean45(mp, mq, &b03, &op);
    let mut trg = triangulate(mp, mq, &b45, eps)?;

    simplify_topology(
        &mut trg.hs,
        &mut b45.ps,
        &mut trg.ns,
        &mut trg.rs,
        b45.nv_from_p,
        b45.nv_from_q,
        eps
    );

    cleanup_unused_verts(
        &mut b45.ps,
        &mut trg.hs
    );

    Manifold::new_impl(
        b45.ps,
        trg.hs
            .chunks(3)
            .map(|hs| Vec3u::new(hs[0].tail, hs[1].tail, hs[2].tail))
            .collect(),
        Some(eps),
        Some(tol)
    )
}





