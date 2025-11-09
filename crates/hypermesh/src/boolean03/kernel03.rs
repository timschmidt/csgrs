use super::kernel02::Kernel02;
use crate::bounds::{BPos, Query};
use crate::common::Row2f;
use crate::Manifold;

pub fn winding03(
    mp: &Manifold,
    mq: &Manifold,
    expand: f64,
    fwd: bool
) -> Vec<i32> {
    let a = if fwd { mp } else { mq };
    let b = if fwd { mq } else { mp };

    let mut w03 = vec![0; a.nv];
    let k02 = Kernel02 {
        ps_p: &a.ps,
        ps_q: &b.ps,
        hs_q: &b.hs,
        ns: &mp.vns,
        expand,
        fwd,
    };

    b.collider.collision(
        & a.ps.iter().enumerate()
            .map(|(i, p)| Query::Pt(BPos{id: Some(i), pos: Row2f::new(p.x, p.y)}))
            .collect::<Vec<_>>(),
        &mut |a, b| if let Some((s, _)) = k02.op(a, b) { w03[a] += s * if fwd { 1 } else { -1 }; }
    );

    w03
}
