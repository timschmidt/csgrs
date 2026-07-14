use hypervoxel::{
    GridFrame, HypervoxelResult, MaterialRegionId, SparseVoxelGrid, VoxelAddress, VoxelCell,
    VoxelPayload,
};

fn main() -> HypervoxelResult<()> {
    let frame = GridFrame::builder()
        .pitch([1.into(), 1.into(), 1.into()])
        .depth(3)
        .build()?;
    let mut grid = SparseVoxelGrid::new(frame);
    let address = VoxelAddress::new(3, [2, 1, 0])?;

    let edit = grid.set(address, VoxelCell::material(MaterialRegionId(4)))?;
    assert!(edit.exact_edit_replay_ready);
    assert_eq!(
        grid.get(address)?.payload,
        VoxelPayload::MaterialRegion(MaterialRegionId(4))
    );

    let bounds = address.bounds(grid.frame())?;
    println!("cell bounds: {:?} .. {:?}", bounds.min, bounds.max);
    Ok(())
}
