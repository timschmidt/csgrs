//! Exact voxel addresses and cell bounds.

use hyperreal::{Rational, Real};

use crate::{GridFrame, HypervoxelError, HypervoxelResult};

/// Integer address of a voxel cell at a specific depth.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct VoxelAddress {
    /// Depth of this address. Depth zero is the root cell.
    pub depth: u8,
    /// Integer coordinates at `depth`.
    pub xyz: [u64; 3],
}

impl VoxelAddress {
    /// Creates an address after checking it lies inside the complete tree.
    pub fn new(depth: u8, xyz: [u64; 3]) -> HypervoxelResult<Self> {
        if depth > crate::frame::MAX_ADDRESS_DEPTH {
            return Err(HypervoxelError::DepthTooLarge {
                depth,
                max_supported: crate::frame::MAX_ADDRESS_DEPTH,
            });
        }
        let cells = 1_u64 << depth;
        if xyz.iter().any(|&v| v >= cells) {
            return Err(HypervoxelError::AddressOverflow);
        }
        Ok(Self { depth, xyz })
    }

    /// Returns the root address.
    pub fn root() -> Self {
        Self {
            depth: 0,
            xyz: [0, 0, 0],
        }
    }

    /// Returns this cell's child address for an octree child index in `0..8`.
    pub fn child(self, child_index: u8) -> HypervoxelResult<Self> {
        if child_index >= 8 {
            return Err(HypervoxelError::InvalidChildIndex(child_index));
        }
        let x = (child_index & 0b001) as u64;
        let y = ((child_index & 0b010) >> 1) as u64;
        let z = ((child_index & 0b100) >> 2) as u64;
        Self::new(
            self.depth + 1,
            [
                self.xyz[0] * 2 + x,
                self.xyz[1] * 2 + y,
                self.xyz[2] * 2 + z,
            ],
        )
    }

    /// Returns the parent address, if this is not the root.
    pub fn parent(self) -> Option<Self> {
        (self.depth > 0).then_some(Self {
            depth: self.depth - 1,
            xyz: [self.xyz[0] / 2, self.xyz[1] / 2, self.xyz[2] / 2],
        })
    }

    /// Returns this address as a Morton/Z-order code at its own depth.
    ///
    /// Morton ordering is a storage key, not a geometric predicate. It is
    /// exposed here because SVO-DAG interning and chunk paging need a stable
    /// exact integer path code, while Yap's EGC boundary keeps metric
    /// coordinates in [`Real`] cell bounds instead of conflating them with this
    /// layout code.
    pub fn morton_code(self) -> u64 {
        let mut code = 0_u64;
        for bit in 0..self.depth {
            let shift = u32::from(bit);
            code |= ((self.xyz[0] >> shift) & 1) << (3 * shift);
            code |= ((self.xyz[1] >> shift) & 1) << (3 * shift + 1);
            code |= ((self.xyz[2] >> shift) & 1) << (3 * shift + 2);
        }
        code
    }

    /// Reconstructs an address from a Morton/Z-order code and depth.
    pub fn from_morton_code(depth: u8, code: u64) -> HypervoxelResult<Self> {
        let mut xyz = [0_u64; 3];
        for bit in 0..depth {
            let shift = u32::from(bit);
            xyz[0] |= ((code >> (3 * shift)) & 1) << shift;
            xyz[1] |= ((code >> (3 * shift + 1)) & 1) << shift;
            xyz[2] |= ((code >> (3 * shift + 2)) & 1) << shift;
        }
        Self::new(depth, xyz)
    }

    /// Returns the octree child-index path from the root to this address.
    pub fn child_path(self) -> Vec<u8> {
        (0..self.depth)
            .rev()
            .map(|level| {
                let x = ((self.xyz[0] >> level) & 1) as u8;
                let y = ((self.xyz[1] >> level) & 1) as u8;
                let z = ((self.xyz[2] >> level) & 1) as u8;
                x | (y << 1) | (z << 2)
            })
            .collect()
    }

    /// Reconstructs an address from an octree child-index path.
    pub fn from_child_path(path: &[u8]) -> HypervoxelResult<Self> {
        if path.len() > usize::from(crate::frame::MAX_ADDRESS_DEPTH) {
            return Err(HypervoxelError::DepthTooLarge {
                depth: path.len() as u8,
                max_supported: crate::frame::MAX_ADDRESS_DEPTH,
            });
        }
        let mut address = Self::root();
        for &child in path {
            address = address.child(child)?;
        }
        Ok(address)
    }

    /// Computes exact cell bounds in a grid frame.
    pub fn bounds(self, frame: &GridFrame) -> HypervoxelResult<CellBounds> {
        if self.depth > frame.depth() {
            return Err(HypervoxelError::DepthOutsideFrame {
                depth: self.depth,
                frame_depth: frame.depth(),
            });
        }

        let span = 1_u64 << (frame.depth() - self.depth);
        let mut min = frame.origin().clone();
        let mut max = frame.origin().clone();

        for axis in 0..3 {
            let fine_min = self.xyz[axis]
                .checked_mul(span)
                .ok_or(HypervoxelError::AddressOverflow)?;
            let fine_max = fine_min
                .checked_add(span)
                .ok_or(HypervoxelError::AddressOverflow)?;
            min[axis] = min[axis].clone() + frame.pitch(axis).clone() * Real::from(fine_min);
            max[axis] = max[axis].clone() + frame.pitch(axis).clone() * Real::from(fine_max);
        }

        Ok(CellBounds { min, max })
    }
}

/// Exact axis-aligned bounds of a voxel cell.
#[derive(Clone, Debug, PartialEq)]
pub struct CellBounds {
    /// Minimum exact corner.
    pub min: [Real; 3],
    /// Maximum exact corner.
    pub max: [Real; 3],
}

impl CellBounds {
    /// Returns the exact extent along one axis.
    pub fn extent(&self, axis: usize) -> Real {
        self.max[axis].clone() - self.min[axis].clone()
    }

    /// Returns the exact center point.
    ///
    /// Center points are exact views over the grid frame. Rendering adapters may
    /// lower them to primitive floats, but that lowering is a lossy export edge
    /// rather than a topology predicate.
    pub fn center(&self) -> [Real; 3] {
        let half: Real = Rational::fraction(1, 2)
            .expect("positive literal denominator")
            .into();
        [
            (&self.min[0] + &self.max[0]) * &half,
            (&self.min[1] + &self.max[1]) * &half,
            (&self.min[2] + &self.max[2]) * &half,
        ]
    }

    /// Returns the eight exact corners in octant order.
    pub fn corners(&self) -> [[Real; 3]; 8] {
        [
            [
                self.min[0].clone(),
                self.min[1].clone(),
                self.min[2].clone(),
            ],
            [
                self.max[0].clone(),
                self.min[1].clone(),
                self.min[2].clone(),
            ],
            [
                self.min[0].clone(),
                self.max[1].clone(),
                self.min[2].clone(),
            ],
            [
                self.max[0].clone(),
                self.max[1].clone(),
                self.min[2].clone(),
            ],
            [
                self.min[0].clone(),
                self.min[1].clone(),
                self.max[2].clone(),
            ],
            [
                self.max[0].clone(),
                self.min[1].clone(),
                self.max[2].clone(),
            ],
            [
                self.min[0].clone(),
                self.max[1].clone(),
                self.max[2].clone(),
            ],
            [
                self.max[0].clone(),
                self.max[1].clone(),
                self.max[2].clone(),
            ],
        ]
    }
}
