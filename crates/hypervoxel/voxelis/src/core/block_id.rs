/// # BlockId
///
/// Represents a node in an octree structure used for voxel storage.
///
/// ## Bit Layout (64-bit structure)
/// ```ignore
///      63     62-55  54-47     46-32      31-0
/// +---------+-------+------+------------+-------+
/// | is_leaf | types | mask | generation | index |
/// +---------+-------+------+------------+-------+
///    1 bit   8 bits  8 bits    15 bits   32 bits
/// ```
///
/// - Bit 63: Node type flag
///   - 1 = Leaf node (contains actual voxel data)
///   - 0 = Branch node (internal node with children)
///
/// - Bits 62-55 (types): For branch nodes, each bit represents one of 8 children
///   - Bit position corresponds to octree child index (0-7)
///   - 1 = Child at this position is a leaf node
///   - 0 = Child at this position is a branch node
///
/// - Bits 54-47 (mask): Child presence mask, each bit represents one of 8 children
///   - Bit position corresponds to octree child index (0-7)
///   - 1 = Child exists at this position
///   - 0 = No child at this position
///
/// - Bits 46-32: Generation (15 bits)
///   - Tracks version/generation of the node for memory management
///   - Maximum value is 0x7FFE (32,766)
///
/// - Bits 31-0: Index (32 bits)
///   - Unique identifier for the node within its generation
///   - Full range of u32 is available (0 to 4,294,967,295)
#[repr(transparent)]
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub struct BlockId(u64);

impl BlockId {
    /// Represents an invalid block ID (all bits set to 1)
    pub const INVALID: BlockId = BlockId(u64::MAX);

    /// Represents an empty block ID (all bits set to 0)
    pub const EMPTY: BlockId = BlockId(0);

    /// Maximum allowed index value (2^32 - 1)
    pub const MAX_INDEX: u32 = u32::MAX;

    /// Maximum allowed generation value (0x7FFE = 32766)
    /// The highest bit is used for the leaf/branch flag, so the maximum generation is 0x7FFE.
    pub const MAX_GENERATION: u16 = 0x7FFE;

    /// Mask for accessing the generation bits (15 bits)
    pub const GENERATION_MASK: u64 = 0x7FFF;

    /// Mask for accessing the index bits (32 bits)
    pub const INDEX_MASK: u64 = 0xFFFF_FFFF;

    /// Creates a BlockId from a raw 64-bit value
    ///
    /// # Parameters
    /// * `raw` - The raw 64-bit value to create the BlockId from
    #[inline(always)]
    pub const fn from_raw(raw: u64) -> Self {
        BlockId(raw)
    }

    /// Creates a new leaf node BlockId with the specified index and generation
    ///
    /// Leaf nodes represent actual voxel data in the octree (terminal nodes).
    ///
    /// # Parameters
    /// * `index` - Unique identifier for this node (32 bits)
    /// * `generation` - Generation/version of this node (15 bits)
    #[inline(always)]
    pub const fn new_leaf(index: u32, generation: u16) -> Self {
        Self::new_extended(index, generation, 0, 0, true)
    }

    /// Creates a new branch node BlockId with the specified parameters
    ///
    /// # Parameters
    /// * `index` - Unique identifier for this node
    /// * `generation` - Generation/version of this node
    /// * `types` - 8-bit value where each bit indicates if the corresponding child is a leaf (1) or branch (0)
    /// * `mask` - 8-bit value where each bit indicates if a child exists (1) or not (0) at the corresponding position
    #[inline(always)]
    pub const fn new_branch(index: u32, generation: u16, types: u8, mask: u8) -> Self {
        Self::new_extended(index, generation, types, mask, false)
    }

    /// Internal function to create a BlockId with all parameters specified
    ///
    /// # Parameters
    /// * `index` - The 32-bit index value (bits 31-0)
    /// * `generation` - The generation value (bits 46-32, only 15 bits used)
    /// * `types` - The 8-bit types field indicating child types (bits 62-55)
    /// * `mask` - The 8-bit mask field indicating child presence (bits 54-47)
    /// * `is_leaf` - Whether this is a leaf node (bit 63)
    #[inline(always)]
    const fn new_extended(index: u32, generation: u16, types: u8, mask: u8, is_leaf: bool) -> Self {
        assert!(generation <= Self::MAX_GENERATION);

        BlockId(
            ((is_leaf as u64) << 63) // Bit 63
                | ((types as u64) << 55) // Bits 62-55
                | ((mask as u64) << 47) // Bits 54-47
                | ((generation as u64 & Self::GENERATION_MASK) << 32) // Bits 46-32
                | (index as u64 & Self::INDEX_MASK), // Bits 31-0
        )
    }

    /// Retrieves the 32-bit index component of this BlockId
    #[inline(always)]
    pub const fn index(&self) -> u32 {
        (self.0 & Self::INDEX_MASK) as u32
    }

    /// Retrieves the 15-bit generation component of this BlockId
    #[inline(always)]
    pub const fn generation(&self) -> u16 {
        ((self.0 >> 32) & Self::GENERATION_MASK) as u16
    }

    /// Retrieves the 8-bit types field of this BlockId
    /// Each bit indicates whether the corresponding child is a leaf (1) or branch (0)
    #[inline(always)]
    pub const fn types(&self) -> u8 {
        assert!(self.is_branch());
        ((self.0 >> 55) & 0xFF) as u8
    }

    /// Retrieves the 8-bit mask field of this BlockId
    /// Each bit indicates whether a child exists (1) or not (0) at the corresponding position
    #[inline(always)]
    pub const fn mask(&self) -> u8 {
        assert!(self.is_branch());
        ((self.0 >> 47) & 0xFF) as u8
    }

    /// Checks if this branch node has a child at the specified index (0-7)
    ///
    /// # Parameters
    /// * `child_index` - Index of the child to check (0-7, corresponding to octree position)
    ///
    /// # Returns
    /// `true` if a child exists at the specified position, `false` otherwise
    #[inline(always)]
    pub const fn has_child(&self, child_index: u8) -> bool {
        assert!(self.is_branch());
        assert!(child_index < 8);

        ((self.0 >> 47) & 0xFF) as u8 & (1 << child_index) != 0
    }

    /// Checks if this BlockId represents a leaf node
    /// Leaf nodes contain actual voxel data
    #[inline(always)]
    pub const fn is_leaf(&self) -> bool {
        (self.0 >> 63) == 1
    }

    /// Checks if this BlockId represents a branch node
    /// Branch nodes are internal nodes that may have children
    #[inline(always)]
    pub const fn is_branch(&self) -> bool {
        (self.0 >> 63) == 0
    }

    /// Checks if this BlockId is invalid (equals INVALID constant)
    #[inline(always)]
    pub const fn is_invalid(&self) -> bool {
        self.0 == Self::INVALID.0
    }

    /// Checks if this BlockId is valid (not equal to INVALID constant)
    #[inline(always)]
    pub const fn is_valid(&self) -> bool {
        self.0 != Self::INVALID.0
    }

    /// Checks if this BlockId is empty (all zeros)
    #[inline(always)]
    pub const fn is_empty(&self) -> bool {
        self.0 == 0
    }

    /// Returns a reference to the raw 64-bit value of this BlockId
    #[inline(always)]
    pub const fn raw(&self) -> &u64 {
        &self.0
    }
}

/// Display implementation for BlockId that provides a human-readable representation
impl std::fmt::Display for BlockId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if self.is_invalid() {
            write!(f, "Id(INVALID)")
        } else if self.is_empty() {
            write!(f, "Id(EMPTY)")
        } else if self.is_leaf() {
            write!(
                f,
                "Id(L, i: {:08X}, g: {:04X})",
                self.index(),
                self.generation(),
            )
        } else {
            write!(
                f,
                "Id(B, i: {:08X}, g: {:04X}, m: {:02X}, t: {:02X})",
                self.index(),
                self.generation(),
                self.mask(),
                self.types(),
            )
        }
    }
}

/// Debug implementation for BlockId that provides a human-readable representation
impl std::fmt::Debug for BlockId {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if self.is_invalid() {
            write!(f, "Id(INVALID)")
        } else if self.is_empty() {
            write!(f, "Id(EMPTY)")
        } else if self.is_leaf() {
            write!(
                f,
                "Id(L, i: {:08X}, g: {:04X})",
                self.index(),
                self.generation(),
            )
        } else {
            write!(
                f,
                "Id(B, i: {:08X}, g: {:04X}, m: {:02X}, t: {:02X})",
                self.index(),
                self.generation(),
                self.mask(),
                self.types(),
            )
        }
    }
}

#[cfg(test)]
mod tests {
    use crate::BlockId;

    #[test]
    fn test_invalid() {
        assert!(BlockId::INVALID.is_invalid());
        assert!(!BlockId::INVALID.is_valid());
    }

    #[test]
    fn test_empty() {
        let id = BlockId::EMPTY;
        assert_eq!(id.index(), 0);
        assert_eq!(id.generation(), 0);
        assert_eq!(id.types(), 0);
        assert_eq!(id.mask(), 0);
        assert!(id.is_branch());
        assert!(id.is_empty());
        assert!(id.is_valid());
        assert!(!id.is_invalid());
    }

    #[test]
    fn test_leaf() {
        let id = BlockId::new_leaf(123, 456);
        assert_eq!(id.index(), 123);
        assert_eq!(id.generation(), 456);
        assert!(id.is_leaf());
        assert!(id.is_valid());
    }

    #[test]
    fn test_branch() {
        let id = BlockId::new_branch(123, 456, 0xAB, 0xCD);
        assert_eq!(id.index(), 123);
        assert_eq!(id.generation(), 456);
        assert_eq!(id.types(), 0xAB);
        assert_eq!(id.mask(), 0xCD);
        assert!(id.is_branch());
        assert!(id.is_valid());
    }

    #[test]
    fn test_max_values() {
        let id = BlockId::new_extended(
            BlockId::MAX_INDEX,
            BlockId::MAX_GENERATION,
            0xFF,
            0xFF,
            true,
        );
        assert!(id.is_valid());
        assert!(!id.is_invalid());
        assert_ne!(
            id,
            BlockId::INVALID,
            "Max values should not be equal to INVALID",
        );
    }
}
