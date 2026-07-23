#![no_main]

use hyperreal::Real;
use hypervoxel::{
    ExactTriangle3, ExactTriangleSurfaceMesh, GridFrame, MaterialRegionId, VoxelizationPolicy,
    voxelize_exact_triangle_surface_mesh,
};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|data: (u8, [u8; 9], bool)| {
    let (depth_raw, coords, _source_replay) = data;
    let depth = (depth_raw % 3) + 1;
    let frame = GridFrame::builder().depth(depth).build().unwrap();
    let cells = 1_u64 << depth;
    let vertex = |i: usize| -> [Real; 3] {
        [
            Real::from(u64::from(coords[i]) % (cells + 1)),
            Real::from(u64::from(coords[i + 1]) % (cells + 1)),
            Real::from(u64::from(coords[i + 2]) % (cells + 1)),
        ]
    };
    let mesh = ExactTriangleSurfaceMesh::new(vec![ExactTriangle3::new(
        [vertex(0), vertex(3), vertex(6)],
        Some(0),
    )]);
    let source_report = mesh.report();
    let result = voxelize_exact_triangle_surface_mesh(
        frame,
        &mesh,
        MaterialRegionId(1),
        VoxelizationPolicy::conservative_cover(),
    );
    if !source_report.exact_triangle_source_ready {
        assert!(result.is_err());
    } else if let Ok((grid, report)) = result {
        assert_eq!(report.unknown_cells, 0);
        assert_eq!(report.boundary_cells, grid.len());
        assert!(report.predicate_certificates.is_fully_certified());
    }
});
