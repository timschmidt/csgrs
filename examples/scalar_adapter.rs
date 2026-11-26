//! Exact finite binary64 ingress and projected bounds through the scalar mesh adapter.
//! Integer-valued coordinates in this fixture are exactly representable on both
//! sides of the adapter; the bounds equality assertions do not imply lossless
//! export for arbitrary native scalars. No output files are written.

use csgrs::adapter::{F64, ScalarMesh};

/// Native mesh operations exposed with the adapter's binary64 input/output scalar.
type MeshF64 = ScalarMesh<F64>;

/// Translates a two-unit cube by +1 on X and checks both projected bound corners.
/// Adapter failures propagate; missing bounds or unequal fixture values panic.
fn main() -> Result<(), csgrs::adapter::AdapterError> {
    let cube = MeshF64::cube(2.0)?;
    let shifted = cube.translate(1.0, 0.0, 0.0)?;
    let bounds = shifted.bounding_box()?.expect("nonempty cube has bounds");
    assert_eq!(bounds.mins, [1.0, 0.0, 0.0]);
    assert_eq!(bounds.maxs, [3.0, 2.0, 2.0]);
    Ok(())
}
