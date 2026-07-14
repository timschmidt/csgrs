use hyperreal::Real;
use hypervoxel::{
    ExactBox, GridFrame, GridSource, HypervoxelResult, LengthUnit, MaterialRegionId,
    VoxelizationPolicy, voxelize_exact_box,
};

fn main() -> HypervoxelResult<()> {
    let source = GridSource::new("example:box", 1);
    let frame = GridFrame::builder()
        .units(LengthUnit::Millimeter)
        .origin([0.into(), 0.into(), 0.into()])
        .pitch([1.into(), 1.into(), 1.into()])
        .depth(3)
        .source(source.clone())
        .build()?;
    let solid = ExactBox::new(
        [Real::from(1), Real::from(1), Real::from(1)],
        [Real::from(3), Real::from(3), Real::from(3)],
        Some(source),
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
