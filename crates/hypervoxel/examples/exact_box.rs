use hyperreal::Real;
use hypervoxel::{
    ExactBox, GridFrame, HypervoxelResult, LengthUnit, MaterialRegionId, VoxelizationPolicy,
    voxelize_exact_box,
};

fn main() -> HypervoxelResult<()> {
    let frame = GridFrame::new(
        [0.into(), 0.into(), 0.into()],
        [1.into(), 1.into(), 1.into()],
        3,
        LengthUnit::Millimeter,
    )?;
    let solid = ExactBox::new(
        [Real::from(1), Real::from(1), Real::from(1)],
        [Real::from(3), Real::from(3), Real::from(3)],
    );

    let (grid, report) = voxelize_exact_box(
        frame,
        &solid,
        MaterialRegionId(7),
        VoxelizationPolicy::conservative_cover(),
    )?;

    assert_eq!(grid.len(), 8);
    assert!(report.exact_topology_ready());
    println!("stored {} exact cells", grid.len());
    Ok(())
}
