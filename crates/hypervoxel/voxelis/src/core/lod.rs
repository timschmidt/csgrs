//! Module `core::lod`
//!
//! Defines the [`Lod`] struct, a compact and type-safe representation of Level of Detail (LOD) for voxel/octree structures.
//!
//! # Usage
//!
//! LOD is used to represent the resolution or coarseness of voxel data, where higher values typically mean lower resolution (coarser data).
//!
//! # Examples
//!
//! ```rust
//! use voxelis::Lod;
//!
//! let lod = Lod::new(2);
//! assert_eq!(lod.lod(), 2);
//! ```

#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct Lod(u8);

impl From<Lod> for u8 {
    #[inline]
    fn from(id: Lod) -> u8 {
        id.0
    }
}

impl From<u8> for Lod {
    #[inline]
    fn from(raw: u8) -> Self {
        Self(raw)
    }
}

/// Display implementation for [`Lod`] that provides a human-readable representation
impl std::fmt::Display for Lod {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}", self.lod())
    }
}

impl Lod {
    /// Creates a new [`Lod`].
    ///
    /// # Parameters
    ///
    /// - `lod`: Level of detail (`u8`).
    ///
    /// # Examples
    ///
    /// ```rust
    /// use voxelis::Lod;
    ///
    /// let lod = Lod::new(3);
    /// assert_eq!(lod.lod(), 3);
    /// ```
    #[must_use]
    #[inline(always)]
    pub const fn new(lod: u8) -> Self {
        Self(lod)
    }

    /// Returns the level of detail as `u8`.
    ///
    /// # Examples
    ///
    /// ```
    /// use voxelis::Lod;
    ///
    /// let lod = Lod::new(2);
    /// assert_eq!(lod.lod(), 2);
    /// ```
    #[must_use]
    #[inline(always)]
    pub const fn lod(&self) -> u8 {
        self.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_new() {
        let lod = Lod::new(5);
        assert_eq!(lod.lod(), 5);
    }

    #[test]
    fn test_display() {
        let lod = Lod::new(7);
        assert_eq!(format!("{lod}"), "7");
    }

    #[test]
    fn test_roundtrip() {
        let val: u8 = 8;
        let lod = Lod::new(val);
        let back: u8 = lod.into();
        assert_eq!(val, back);
    }

    #[test]
    fn test_zero_lod() {
        let lod = Lod::new(0);
        assert_eq!(lod.lod(), 0);
    }

    #[test]
    fn test_max_lod() {
        let lod = Lod::new(u8::MAX);
        assert_eq!(lod.lod(), 255);
    }
}
