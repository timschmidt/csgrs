use hypervoxel::{
    GridFrame, HypervoxelResult, MaterialRegionId, SparseVoxelGrid, VoxelAddress, VoxelCell,
    VoxelPayload,
};

fn main() -> HypervoxelResult<()> {
    let frame = GridFrame::unit(3)?;
    let mut grid = SparseVoxelGrid::new(frame);
    let address = VoxelAddress::new(3, [2, 1, 0])?;

    grid.set(address, VoxelCell::material(MaterialRegionId(4)))?;
    assert_eq!(
        grid.get(address)?.payload,
        VoxelPayload::MaterialRegion(MaterialRegionId(4))
    );

    let bounds = address.bounds(grid.frame())?;
    println!("cell bounds: {:?} .. {:?}", bounds.min, bounds.max);
    Ok(())
}
