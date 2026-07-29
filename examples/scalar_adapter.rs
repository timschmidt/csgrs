use csgrs::adapter::{F64, ScalarMesh};

type MeshF64 = ScalarMesh<F64>;

fn main() -> Result<(), csgrs::adapter::AdapterError> {
    let cube = MeshF64::cube(2.0)?;
    let shifted = cube.translate(1.0, 0.0, 0.0)?;
    let bounds = shifted.bounding_box()?;
    assert_eq!(bounds.mins, [1.0, 0.0, 0.0]);
    assert_eq!(bounds.maxs, [3.0, 2.0, 2.0]);
    Ok(())
}
