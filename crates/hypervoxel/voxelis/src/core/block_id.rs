//! Compact identifiers for interned leaf and branch nodes.

// Packed-field offsets and masks. The layout is part of serialized node IDs,
// so changes require an explicit compatibility migration.
const LEAF_SHIFT: u64 = 63;
const TYPES_SHIFT: u32 = 55;
const MASK_SHIFT: u32 = 47;
const GENERATION_SHIFT: u32 = 32;
const GENERATION_MASK: u64 = 0x7FFF;
const INDEX_MASK: u64 = 0xFFFF_FFFF;

/// Packed identity and branch metadata for a node owned by [`crate::VoxInterner`].
///
/// # Bit layout
///
/// ```text
/// ┌63──────63┬62───────55┬54──────47┬46─────────────32┬31─────────0┐
/// │ LEAF (1) │ TYPES (8) │ MASK (8) │ GENERATION (15) │ INDEX (32) │
/// └──────────┴───────────┴──────────┴─────────────────┴────────────┘
/// ```
///
/// The top bit distinguishes leaves from branches. For a branch, `types` marks
/// leaf children and `mask` marks present children. The generation detects a
/// stale index after the interner recycles its storage slot.
///
/// A value being different from [`Self::INVALID`] does not prove that it is
/// live in a particular interner; use the interner's validity checks when
/// validating externally retained IDs.
///
/// ```
/// use voxelis::BlockId;
///
/// let leaf_id = BlockId::new_leaf(123, 456);
/// assert_eq!(leaf_id.index(), 123);
/// assert_eq!(leaf_id.generation(), 456);
/// assert!(leaf_id.is_leaf());
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
    /// Invalid sentinel with every bit set.
    pub const INVALID: BlockId = BlockId(u64::MAX);

    /// Canonical empty-branch sentinel with every bit clear.
    pub const EMPTY: BlockId = BlockId(0);

    /// Largest encodable storage index.
    pub const MAX_INDEX: u32 = u32::MAX;

    /// Largest generation accepted by the constructors.
    ///
    /// The interner uses this value as its recycling threshold and therefore
    /// does not normally issue it. The all-ones generation remains reserved
    /// for the invalid sentinel.
    pub const MAX_GENERATION: u16 = 0x7FFE;

    /// Wraps an unvalidated packed value.
    ///
    /// This accepts reserved and internally inconsistent bit patterns. It is
    /// intended for trusted serialization code that validates in context.
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

    /// Creates a leaf ID for an interner storage slot and generation.
    ///
    /// # Panics
    ///
    /// Panics if `generation` exceeds [`Self::MAX_GENERATION`].
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

    /// Creates a branch ID with packed child type and presence masks.
    ///
    /// Bit `n` in `types` is set when child `n` is a leaf; bit `n` in `mask`
    /// is set when that child exists.
    ///
    /// # Panics
    ///
    /// Panics if `generation` exceeds [`Self::MAX_GENERATION`].
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

    /// Packs all identifier fields after validating the generation.
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

    /// Returns the 32-bit storage index.
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

    /// Returns the 15-bit storage generation.
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

    /// Returns the branch's child-type mask.
    ///
    /// # Panics
    ///
    /// Panics when called on a leaf.
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

    /// Returns the branch's child-presence mask.
    ///
    /// # Panics
    ///
    /// Panics when called on a leaf.
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

    /// Returns whether branch child `child_index` is present.
    ///
    /// # Panics
    ///
    /// Panics when called on a leaf or with an index outside `0..8`.
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

    /// Returns whether the packed node-kind bit denotes a leaf.
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

    /// Returns whether the packed node-kind bit denotes a branch.
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

    /// Returns whether this is the invalid sentinel.
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

    /// Returns whether this is not the invalid sentinel.
    ///
    /// This does not check that the index and generation are live in an
    /// interner.
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

    /// Returns whether this is the canonical empty-branch sentinel.
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

    /// Returns the packed 64-bit representation.
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
