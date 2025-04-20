//! Module `core::block_id`
//!
//! This module defines the [`BlockId`] struct, a compact 64-bit identifier for nodes in an octree.
//! It encodes node type (leaf/branch), child types, presence mask, generation, and index.
//!
//! # Examples
//!
//! ```rust
//! use voxelis::BlockId;
//!
//! let leaf_id = BlockId::new_leaf(123, 456);
//! println!("{}", leaf_id);
//!
//! let branch_id = BlockId::new_branch(789, 1011, 0xAB, 0xCD);
//! println!("{}", branch_id);
//! ```

/// Shift for the leaf/branch flag (1 bit)
const LEAF_SHIFT: u64 = 63;
/// Shift for the types bits (8 bits)
const TYPES_SHIFT: u32 = 55;
/// Shift for the mask bits (8 bits)
const MASK_SHIFT: u32 = 47;
/// Shift for the generation bits (15 bits)
const GENERATION_SHIFT: u32 = 32;

/// Mask for accessing the generation bits (15 bits)
const GENERATION_MASK: u64 = 0x7FFF;
/// Mask for accessing the index bits (32 bits)
const INDEX_MASK: u64 = 0xFFFF_FFFF;

/// # [`BlockId`]
///
/// Represents a node in an octree structure used for voxel storage.
///
/// ## Bit Layout (64-bit structure)
///
/// ```text
/// ┌63──────63┬62───────55┬54──────47┬46─────────────32┬31─────────0┐
/// │ LEAF (1) │ TYPES (8) │ MASK (8) │ GENERATION (15) │ INDEX (32) │
/// └──────────┴───────────┴──────────┴─────────────────┴────────────┘
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
///
/// # Examples
///
/// Create a new leaf node [`BlockId`]
///
/// ```
/// use voxelis::BlockId;
///
/// let leaf_id = BlockId::new_leaf(123, 456);
/// assert_eq!(leaf_id.index(), 123);
/// assert_eq!(leaf_id.generation(), 456);
/// assert!(leaf_id.is_leaf());
///```
///
/// Create a new branch node [`BlockId`]
/// ```
/// use voxelis::BlockId;
///
/// let branch_id = BlockId::new_branch(789, 1011, 0xAB, 0xCD);
/// assert_eq!(branch_id.index(), 789);
/// assert_eq!(branch_id.generation(), 1011);
/// assert_eq!(branch_id.types(), 0xAB);
/// assert_eq!(branch_id.mask(), 0xCD);
/// assert!(branch_id.is_branch());
///
/// // Check if a child exists at a specific index
/// assert!(branch_id.has_child(0)); // Check if child at index 0 exists
/// assert!(!branch_id.has_child(1)); // Check if child at index 1 exists
/// ```
#[repr(transparent)]
#[derive(Clone, Copy, PartialEq, Eq, Hash)]
pub struct BlockId(u64);

impl From<BlockId> for u64 {
    #[inline]
    fn from(id: BlockId) -> u64 {
        id.0
    }
}

impl From<u64> for BlockId {
    #[inline]
    fn from(raw: u64) -> Self {
        Self(raw)
    }
}

impl Default for BlockId {
    fn default() -> Self {
        Self::INVALID
    }
}

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

    /// Creates a [`BlockId`] from a raw 64-bit value
    ///
    /// # Parameters
    ///
    /// * `raw` - The raw 64-bit value to create the [`BlockId`] from
    ///
    /// # Examples
    ///
    /// ```
    /// use voxelis::BlockId;
    ///
    /// let raw_value = 0x123456789ABCDEF0;
    /// let block_id = BlockId::from_raw(raw_value);
    /// assert_eq!(block_id.raw(), raw_value);
    /// ```
    #[must_use]
    #[inline(always)]
    pub const fn from_raw(raw: u64) -> Self {
        BlockId(raw)
    }

    /// Creates a new leaf node [`BlockId`] with the specified index and generation
    ///
    /// Leaf nodes represent actual voxel data in the octree (terminal nodes).
    ///
    /// # Parameters
    ///
    /// * `index` - Unique identifier for this node (32 bits)
    /// * `generation` - Generation/version of this node (15 bits)
    ///
    /// # Examples
    ///
    /// ```
    /// use voxelis::BlockId;
    ///
    /// let leaf_id = BlockId::new_leaf(123, 456);
    /// assert_eq!(leaf_id.index(), 123);
    /// assert_eq!(leaf_id.generation(), 456);
    /// assert!(leaf_id.is_leaf());
    /// ```
    #[must_use]
    #[inline(always)]
    pub const fn new_leaf(index: u32, generation: u16) -> Self {
        Self::new_extended(index, generation, 0, 0, true)
    }

    /// Creates a new branch node [`BlockId`] with the specified parameters
    ///
    /// # Parameters
    ///
    /// * `index` - Unique identifier for this node
    /// * `generation` - Generation/version of this node
    /// * `types` - 8-bit value where each bit indicates if the corresponding child is a leaf (1) or branch (0)
    /// * `mask` - 8-bit value where each bit indicates if a child exists (1) or not (0) at the corresponding position
    ///
    /// # Examples
    ///
    /// ```
    /// use voxelis::BlockId;
    ///
    /// let branch_id = BlockId::new_branch(789, 1011, 0xAB, 0xCD);
    /// assert_eq!(branch_id.index(), 789);
    /// assert_eq!(branch_id.generation(), 1011);
    /// assert_eq!(branch_id.types(), 0xAB);
    /// assert_eq!(branch_id.mask(), 0xCD);
    /// assert!(branch_id.is_branch());
    /// ```
    #[must_use]
    #[inline(always)]
    pub const fn new_branch(index: u32, generation: u16, types: u8, mask: u8) -> Self {
        Self::new_extended(index, generation, types, mask, false)
    }

    /// Internal function to create a [`BlockId`] with all parameters specified
    ///
    /// # Parameters
    ///
    /// * `index` - The 32-bit index value (bits 31-0)
    /// * `generation` - The generation value (bits 46-32, only 15 bits used)
    /// * `types` - The 8-bit types field indicating child types (bits 62-55)
    /// * `mask` - The 8-bit mask field indicating child presence (bits 54-47)
    /// * `is_leaf` - Whether this is a leaf node (bit 63)
    #[must_use]
    #[inline(always)]
    const fn new_extended(index: u32, generation: u16, types: u8, mask: u8, is_leaf: bool) -> Self {
        assert!(generation <= Self::MAX_GENERATION, "Generation overflow");

        BlockId(
            ((is_leaf as u64) << LEAF_SHIFT)
                | ((types as u64) << TYPES_SHIFT)
                | ((mask as u64) << MASK_SHIFT)
                | ((generation as u64 & GENERATION_MASK) << GENERATION_SHIFT)
                | (index as u64 & INDEX_MASK),
        )
    }

    /// Retrieves the 32-bit index component of this [`BlockId`]
    ///
    /// # Examples
    ///
    /// ```
    /// use voxelis::BlockId;
    ///
    /// let block_id = BlockId::new_leaf(123, 456);
    /// assert_eq!(block_id.index(), 123);
    /// ```
    #[must_use]
    #[inline(always)]
    pub const fn index(&self) -> u32 {
        (self.0 & INDEX_MASK) as u32
    }

    /// Retrieves the 15-bit generation component of this [`BlockId`]
    ///
    /// # Examples
    ///
    /// ```
    /// use voxelis::BlockId;
    ///
    /// let block_id = BlockId::new_leaf(123, 456);
    /// assert_eq!(block_id.generation(), 456);
    /// ```
    #[must_use]
    #[inline(always)]
    pub const fn generation(&self) -> u16 {
        ((self.0 >> GENERATION_SHIFT) & GENERATION_MASK) as u16
    }

    /// Retrieves the 8-bit types field of this [`BlockId`]
    /// Each bit indicates whether the corresponding child is a leaf (1) or branch (0)
    ///
    /// # Examples
    ///
    /// ```
    /// use voxelis::BlockId;
    ///
    /// let block_id = BlockId::new_branch(123, 456, 0xAB, 0xCD);
    /// assert_eq!(block_id.types(), 0xAB);
    /// ```
    #[must_use]
    #[inline(always)]
    pub const fn types(&self) -> u8 {
        assert!(self.is_branch(), "Cannot get types from a leaf node");
        ((self.0 >> TYPES_SHIFT) & 0xFF) as u8
    }

    /// Retrieves the 8-bit mask field of this [`BlockId`]
    /// Each bit indicates whether a child exists (1) or not (0) at the corresponding position
    ///
    /// # Examples
    ///
    /// ```
    /// use voxelis::BlockId;
    ///
    /// let block_id = BlockId::new_branch(123, 456, 0xAB, 0xCD);
    /// assert_eq!(block_id.mask(), 0xCD);
    /// ```
    #[must_use]
    #[inline(always)]
    pub const fn mask(&self) -> u8 {
        assert!(self.is_branch(), "Cannot get mask from a leaf node");
        ((self.0 >> MASK_SHIFT) & 0xFF) as u8
    }

    /// Checks if this branch node has a child at the specified index (0-7)
    /// Returns `true` if a child exists at the specified position, `false` otherwise
    ///
    /// # Parameters
    ///
    /// * `child_index` - Index of the child to check (0-7, corresponding to octree position)
    ///
    /// # Examples
    ///
    /// ```
    /// use voxelis::BlockId;
    ///
    /// let block_id = BlockId::new_branch(123, 456, 0xAB, 0xCD);
    /// assert!(block_id.has_child(0)); // Check if child at index 0 exists
    /// assert!(!block_id.has_child(1)); // Check if child at index 1 exists
    /// ```
    #[must_use]
    #[inline(always)]
    pub const fn has_child(&self, child_index: u8) -> bool {
        assert!(self.is_branch(), "Cannot check child on a leaf node");
        assert!(child_index < 8, "Child index out of bounds (0-7)");

        (((self.0 >> MASK_SHIFT) & 0xFF) as u8 & (1 << child_index)) != 0
    }

    /// Checks if this [`BlockId`] represents a leaf node
    /// Leaf nodes contain actual voxel data
    ///
    /// # Examples
    ///
    /// ```
    /// use voxelis::BlockId;
    ///
    /// let leaf_id = BlockId::new_leaf(123, 456);
    /// assert!(leaf_id.is_leaf());
    /// ```
    #[must_use]
    #[inline(always)]
    pub const fn is_leaf(&self) -> bool {
        (self.0 >> LEAF_SHIFT) == 1
    }

    /// Checks if this [`BlockId`] represents a branch node
    /// Branch nodes are internal nodes that may have children
    ///
    /// # Examples
    ///
    /// ```
    /// use voxelis::BlockId;
    ///
    /// let branch_id = BlockId::new_branch(123, 456, 0xAB, 0xCD);
    /// assert!(branch_id.is_branch());
    /// ```
    #[must_use]
    #[inline(always)]
    pub const fn is_branch(&self) -> bool {
        (self.0 >> LEAF_SHIFT) == 0
    }

    /// Checks if this [`BlockId`] is invalid (equals INVALID constant)
    ///
    /// # Examples
    ///
    /// ```
    /// use voxelis::BlockId;
    ///
    /// let invalid_id = BlockId::INVALID;
    /// assert!(invalid_id.is_invalid());
    /// ```
    #[must_use]
    #[inline(always)]
    pub const fn is_invalid(&self) -> bool {
        self.0 == Self::INVALID.0
    }

    /// Checks if this [`BlockId`] is valid (not equal to INVALID constant)
    ///
    /// # Examples
    ///
    /// ```
    /// use voxelis::BlockId;
    ///
    /// let valid_id = BlockId::new_leaf(123, 456);
    /// assert!(valid_id.is_valid());
    /// ```
    #[must_use]
    #[inline(always)]
    pub const fn is_valid(&self) -> bool {
        self.0 != Self::INVALID.0
    }

    /// Checks if this [`BlockId`] is empty (all zeros)
    ///
    /// # Examples
    ///
    /// ```
    /// use voxelis::BlockId;
    ///
    /// let empty_id = BlockId::EMPTY;
    /// assert!(empty_id.is_empty());
    /// ```
    #[must_use]
    #[inline(always)]
    pub const fn is_empty(&self) -> bool {
        self.0 == 0
    }

    /// Returns raw 64-bit value of this [`BlockId`]
    ///
    /// # Examples
    ///
    /// ```
    /// use voxelis::BlockId;
    ///
    /// let block_id = BlockId::new_leaf(123, 456);
    /// assert_eq!(block_id.raw(), 0x800001C80000007B);
    /// ```
    #[must_use]
    #[inline(always)]
    pub const fn raw(&self) -> u64 {
        self.0
    }
}

/// Display implementation for [`BlockId`] that provides a human-readable representation
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

/// Debug implementation for [`BlockId`] that provides a human-readable representation
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

    #[test]
    fn test_raw_roundtrip() {
        let branch = BlockId::new_branch(123, 456, 0xAA, 0x55);
        let raw: u64 = branch.into();
        assert_eq!(raw, branch.raw());
        assert_eq!(BlockId::from_raw(raw), branch);
    }

    #[test]
    fn test_display_debug_variants() {
        let invalid = BlockId::INVALID;
        assert_eq!(format!("{invalid}"), "Id(INVALID)");
        assert_eq!(format!("{invalid:?}"), "Id(INVALID)");

        let empty = BlockId::EMPTY;
        assert_eq!(format!("{empty}"), "Id(EMPTY)");
        assert_eq!(format!("{empty:?}"), "Id(EMPTY)");

        let leaf = BlockId::new_leaf(1, 2);
        assert_eq!(format!("{leaf}"), format!("{leaf:?}"));
        assert!(leaf.is_leaf());

        let branch = BlockId::new_branch(3, 4, 0x0F, 0xF0);
        assert_eq!(format!("{branch}",), format!("{branch:?}"));
        assert!(branch.is_branch());
    }

    #[test]
    #[cfg(debug_assertions)]
    #[should_panic(expected = "Generation overflow")]
    fn test_generation_overflow() {
        let _ = BlockId::new_leaf(0, BlockId::MAX_GENERATION + 1);
    }

    #[test]
    #[cfg(debug_assertions)]
    #[should_panic(expected = "Cannot get types from a leaf node")]
    fn test_types_on_leaf_panic() {
        let leaf = BlockId::new_leaf(0, 0);
        let _ = leaf.types();
    }

    #[test]
    #[cfg(debug_assertions)]
    #[should_panic(expected = "Cannot get mask from a leaf node")]
    fn test_mask_on_leaf_panic() {
        let leaf = BlockId::new_leaf(0, 0);
        let _ = leaf.mask();
    }

    #[test]
    #[cfg(debug_assertions)]
    #[should_panic(expected = "Cannot check child on a leaf node")]
    fn test_has_child_on_leaf_panic() {
        let leaf = BlockId::new_leaf(0, 0);
        let _ = leaf.has_child(0);
    }

    #[test]
    #[cfg(debug_assertions)]
    #[should_panic(expected = "Child index out of bounds (0-7)")]
    fn test_has_child_index_out_of_bounds() {
        let branch = BlockId::new_branch(0, 0, 0xFF, 0xFF);
        let _ = branch.has_child(8);
    }

    #[test]
    fn test_has_child_logic() {
        let branch = BlockId::new_branch(0, 0, 0, 0b1010_1010);
        assert!(branch.has_child(1));
        assert!(!branch.has_child(0));
        assert!(branch.has_child(3));
        assert!(!branch.has_child(2));
    }
}
