//--- Copyright (C) 2025 Saki Komikado <komietty@gmail.com>,
//--- This Source Code Form is subject to the terms of the Mozilla Public License v.2.0.

use super::kernel02::Kernel02;
use crate::bounds::{BPos, Query};
use crate::{Real, Vec2, Manifold};

pub fn winding03(
    mp: &Manifold,
    mq: &Manifold,
    expand: Real,
    fwd: bool
) -> Vec<i32> {
    let ma = if fwd { mp } else { mq };
    let mb = if fwd { mq } else { mp };

    let mut w03 = vec![0; ma.nv];
    let k02 = Kernel02 {
        ps_p: &ma.ps,
        ps_q: &mb.ps,
        hs_q: &mb.hs,
        ns: &mp.vert_normals,
        expand,
        fwd,
    };

    mb.collider.collision(
        &ma.ps.iter().enumerate()
            .map(|(i, p)| Query::Pt(BPos{id: Some(i), pos: Vec2::new(p.x, p.y)}))
            .collect::<Vec<_>>(),
        &mut |a, b| if let Some((s, _)) = k02.op(a, b) { w03[a] += s * if fwd { 1 } else { -1 }; }
    );

    w03
}
