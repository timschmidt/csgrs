#![no_main]

use hyperreal::Real;
use hypervoxel::{
    ExactTriangle3, ExactTriangleSolidMesh, ExactTriangleSurfaceMesh, GridFrame, GridSource,
    MaterialRegionId, VoxelizationPolicy, voxelize_exact_triangle_solid_mesh,
};
use libfuzzer_sys::fuzz_target;

fn r(value: u64) -> Real {
    Real::from(value)
}

fn tri(vertices: [[Real; 3]; 3]) -> ExactTriangle3 {
    ExactTriangle3::new(vertices, Some(0))
}

fuzz_target!(|data: (u8, u8, u8, bool)| {
    let (depth_raw, lo_raw, span_raw, closed_replay) = data;
    let depth = (depth_raw % 3) + 2;
    let frame = GridFrame::builder()
        .depth(depth)
        .source(GridSource::new("fuzz:triangle-solid", 1))
        .build()
        .unwrap();
    let cells = 1_u64 << depth;
    let lo = 1 + (u64::from(lo_raw) % (cells - 1));
    let hi = (lo + 1 + (u64::from(span_raw) % (cells - lo))).min(cells);
    let p = |x, y, z| [r(x), r(y), r(z)];
    let surface = ExactTriangleSurfaceMesh::new(
        vec![
            tri([p(lo, lo, lo), p(lo, hi, hi), p(lo, hi, lo)]),
            tri([p(lo, lo, lo), p(lo, lo, hi), p(lo, hi, hi)]),
            tri([p(hi, lo, lo), p(hi, hi, lo), p(hi, lo, hi)]),
            tri([p(hi, hi, lo), p(hi, hi, hi), p(hi, lo, hi)]),
            tri([p(lo, lo, lo), p(hi, lo, lo), p(lo, lo, hi)]),
            tri([p(hi, lo, lo), p(hi, lo, hi), p(lo, lo, hi)]),
            tri([p(lo, hi, lo), p(lo, hi, hi), p(hi, hi, lo)]),
            tri([p(hi, hi, lo), p(lo, hi, hi), p(hi, hi, hi)]),
            tri([p(lo, lo, lo), p(lo, hi, lo), p(hi, lo, lo)]),
            tri([p(hi, lo, lo), p(lo, hi, lo), p(hi, hi, lo)]),
            tri([p(lo, lo, hi), p(hi, lo, hi), p(lo, hi, hi)]),
            tri([p(hi, lo, hi), p(hi, hi, hi), p(lo, hi, hi)]),
        ],
        frame.source().cloned(),
        true,
    );
    let solid = ExactTriangleSolidMesh::new(surface, closed_replay);
    let result = voxelize_exact_triangle_solid_mesh(
        frame,
        &solid,
        MaterialRegionId(1),
        VoxelizationPolicy::conservative_cover(),
    );
    if closed_replay {
        let (grid, report) = result.unwrap();
        assert_eq!(report.unknown_cells, 0);
        assert!(grid.len() <= report.aggregate.child_count);
        assert!(report.predicate_certificates.is_fully_certified());
    } else {
        assert!(result.is_err());
    }
});
