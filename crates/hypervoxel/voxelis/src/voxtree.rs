use rayon::prelude::*;
use rustc_hash::FxHashMap;

use crate::voxtree_iterator::VoxTreeIterator;

/// Calculates the total number of voxels along one axis at a given level of detail (LOD).
///
/// # Parameters
///
/// - `lod_level`: The level of detail for which to calculate the number of voxels per axis.
///
/// # Returns
///
/// The number of voxels along one axis at the specified LOD as a `usize`.
///
/// # Examples
///
/// ```
/// use voxelis::voxtree::calculate_voxels_per_axis;
/// assert_eq!(calculate_voxels_per_axis(0), 1);
/// assert_eq!(calculate_voxels_per_axis(1), 2);
/// assert_eq!(calculate_voxels_per_axis(2), 4);
/// assert_eq!(calculate_voxels_per_axis(3), 8);
/// assert_eq!(calculate_voxels_per_axis(4), 16);
/// assert_eq!(calculate_voxels_per_axis(5), 32);
/// assert_eq!(calculate_voxels_per_axis(6), 64);
/// ```
pub const fn calculate_voxels_per_axis(lod_level: usize) -> usize {
    1 << lod_level
}

/// Calculates the area of voxels along one axis at a given level of detail (LOD).
///
/// # Parameters
///
/// - `lod_level`: The level of detail for which to calculate the voxel area.
///
/// # Returns
///
/// The area of the voxels along one axis at the specified LOD as a `usize`.
///
/// # Example
///
/// ```
/// use voxelis::voxtree::calculate_voxel_area;
/// assert_eq!(calculate_voxel_area(2), 16);
/// ```
///
/// # Note
///
/// This function relies on the [calculate_voxels_per_axis] function to determine the number of voxels per axis.
pub const fn calculate_voxel_area(lod_level: usize) -> usize {
    let voxels_per_axis = calculate_voxels_per_axis(lod_level);
    voxels_per_axis * voxels_per_axis
}

/// Calculates the volume of voxels at a given level of detail (LOD).
///
/// # Parameters
///
/// - `lod_level`: The level of detail for which to calculate the voxel volume.
///
/// # Returns
///
/// The volume of the voxels at the specified LOD as a `usize`.
///
/// # Example
///
/// ```
/// use voxelis::voxtree::calculate_voxel_volume;
/// assert_eq!(calculate_voxel_volume(2), 64);
/// ```
///
/// # Note
///
/// This function relies on the [calculate_voxels_per_axis] function to determine the number of voxels per axis.
pub const fn calculate_voxel_volume(lod_level: usize) -> usize {
    let voxels_per_axis = calculate_voxels_per_axis(lod_level);
    voxels_per_axis * voxels_per_axis * voxels_per_axis
}

/// Calculates the total number of voxels needed to keep data for all LOD levels up to the maximum level of detail (LOD).
///
/// # Parameters
///
/// - `max_lod_level`: The maximum level of detail, which determines the size of the chunk.
///
/// # Returns
///
/// The total number of voxels in the chunk for all LOD levels up to the specified maximum LOD as a `usize`.
///
/// # Example
///
/// ```
/// use voxelis::voxtree::calculate_total_voxel_count;
/// assert_eq!(calculate_total_voxel_count(2), 73);
/// ```
///
/// # Note
///
/// This function uses a formula to determine the total voxel count based on the maximum LOD, for example:
/// For `max_lod_level = 2`:
/// - On LOD 0: we have 64 voxels
/// - On LOD 1: we have 8 voxels
/// - On LOD 2: we have 1 voxel
/// - Total: 64 + 8 + 1 = 73 voxels
///
/// The formula used is:
/// - Sum of powers: 1 + 8 + 64 + ... + (8^max_lod_level)
/// - Simplified to: (8^(max_lod_level + 1) - 1) / (8 - 1)
/// - Further simplified to: (1 << (3 * (max_lod_level + 1))) / 7
pub const fn calculate_total_voxel_count(max_lod_level: usize) -> usize {
    // 1 + 8 + 64 + ... + (8^max_lod_level)
    // (8^(max_lod_level + 1) - 1) / (8 - 1)
    // (8^(max_lod_level + 1) - 1) / 7
    // ((2^3)^(max_lod_level + 1))) / 7
    // (2^(3 * (max_lod_level + 1))) / 7
    ((1 << (3 * (max_lod_level + 1))) - 1) / 7
}

pub const fn calculate_lod_data_index(lod_level: usize, max_lod_level: usize) -> usize {
    // Map the external LOD (where lod = 0 is highest detail) to internal LOD (for calculations)
    let internal_lod = max_lod_level - lod_level;

    // Calculate the index at which data starts for given LOD level
    if internal_lod == 0 {
        0
    } else {
        calculate_total_voxel_count(internal_lod - 1)
    }
}

/// A Sparse-Voxel-Octree (SVO) structure representing a voxel tree that stores voxel data for multiple levels of detail (LODs).
///
/// The [VoxTree] struct is parameterized by a constant `MAX_LOD_LEVEL`, which determines the maximum level of detail
/// and consequently the total size of the data storage required to keep voxels for all LODs.
///
/// # Type Parameters
///
/// - `MAX_LOD_LEVEL`: The maximum level of detail for the voxel tree.
///
/// # Fields
///
/// - `data`: A hashmap storing the voxel data. The maximum size of this hashmap is determined by the `MAX_LOD_LEVEL`.
///
/// # Constants
///
/// - [MAX_VOXEL_COUNT](VoxTree::MAX_VOXEL_COUNT): The maximum size of the data hashmap, calculated using the [calculate_total_voxel_count] function.
///
/// # Methods
///
/// - [new](VoxTree::new)`()`: Creates a new [VoxTree] instance with the empty data hashmap.
/// - [set_value](VoxTree::set_value)`(lod, x, y, z, value)`: Sets the value of a voxel at the specified LOD and coordinates.
/// - [get_value](VoxTree::get_value)`(lod, x, y, z)`: Gets the value of a voxel at the specified LOD and coordinates.
/// - [update_lods](VoxTree::update_lods)`()`: Updates the voxel data for all LODs based on the current voxel values at maximum LOD, 0.
///
/// # Explanation
///
/// The [VoxTree] holds data for `MAX_LOD_LEVEL` LODs. For example:
/// - For `MAX_LOD_LEVEL = 0`, there will be only one LOD level with a single voxel.
/// - For `MAX_LOD_LEVEL = 1`, there will be two LOD levels: 8 voxels at level 0, and 1 voxel at level 1, totaling 9 voxels.
/// - For `MAX_LOD_LEVEL = 2`, there will be three LOD levels: 64 voxels at level 0, 8 voxels at level 1, and 1 voxel at level 2, totaling 73 voxels.
///
/// # Example
///
/// ```
/// use voxelis::voxtree::VoxTree;
/// const MAX_LOD: usize = 2;
/// let mut voxtree = VoxTree::<MAX_LOD>::new();
/// voxtree.set_value(1, 1, 1, 1, 42);
/// ```
pub struct VoxTree<const MAX_LOD_LEVEL: usize> {
    data: FxHashMap<usize, i32>,
}

impl<const MAX_LOD_LEVEL: usize> VoxTree<MAX_LOD_LEVEL> {
    pub const MAX_VOXEL_COUNT: usize = calculate_total_voxel_count(MAX_LOD_LEVEL);

    // Creates a new [VoxTree] instance.
    ///
    /// This method initializes the `data` vector with zeros, with the size [MAX_VOXEL_COUNT](VoxTree::MAX_VOXEL_COUNT), determined by the `MAX_LOD_LEVEL`.
    ///
    /// # Returns
    ///
    /// A new [VoxTree] instance with the data vector initialized to zeros.
    ///
    /// # Example
    ///
    /// ```
    /// use voxelis::voxtree::VoxTree;
    /// const MAX_LOD: usize = 2;
    /// let voxtree = VoxTree::<MAX_LOD>::new();
    /// ```
    pub fn new() -> Self {
        Self {
            data: FxHashMap::default(),
        }
    }

    pub fn is_empty(&self) -> bool {
        self.data.is_empty()
    }

    pub fn clear(&mut self) {
        self.data.clear();
    }

    /// Sets the value of a voxel at the specified level of detail (LOD) and coordinates.
    ///
    /// # Parameters
    ///
    /// - `lod`: The level of detail (0 is the maximum detail, `MAX_LOD_LEVEL` lowest).
    /// - `x`: The x-coordinate of the voxel at the specified LOD.
    /// - `y`: The y-coordinate of the voxel at the specified LOD.
    /// - `z`: The z-coordinate of the voxel at the specified LOD.
    /// - `value`: The value to set for the voxel.
    ///
    /// # Example
    ///
    /// ```
    /// use voxelis::voxtree::VoxTree;
    /// const MAX_LOD: usize = 2;
    /// let mut voxtree = VoxTree::<MAX_LOD>::new();
    /// voxtree.set_value(1, 1, 1, 1, 42);
    /// ```
    pub fn set_value(&mut self, lod: usize, x: u8, y: u8, z: u8, value: i32) {
        let index = Self::get_index_of(lod, x, y, z);
        if value != 0 {
            self.data.insert(index, value);
        } else {
            self.data.remove(&index);
        }
    }

    fn set_value_for_index(&mut self, index: usize, value: i32) {
        if value != 0 {
            self.data.insert(index, value);
        } else {
            self.data.remove(&index);
        }
    }

    /// Gets the value of a voxel at the specified level of detail (LOD) and coordinates.
    ///
    /// # Parameters
    ///
    /// - `lod`: The level of detail (0 is the maximum detail, `MAX_LOD_LEVEL` lowest).
    /// - `x`: The x-coordinate of the voxel at the specified LOD.
    /// - `y`: The y-coordinate of the voxel at the specified LOD.
    /// - `z`: The z-coordinate of the voxel at the specified LOD.
    ///
    /// # Returns
    ///
    /// The value of the voxel at the specified LOD and coordinates as an `i32`.
    ///
    /// # Example
    ///
    /// ```
    /// use voxelis::voxtree::VoxTree;
    /// const MAX_LOD: usize = 2;
    /// let voxtree = VoxTree::<MAX_LOD>::new();
    /// let value = voxtree.get_value(1, 1, 1, 1);
    /// ```
    pub fn get_value(&self, lod: usize, x: u8, y: u8, z: u8) -> i32 {
        let index = Self::get_index_of(lod, x, y, z);
        *self.data.get(&index).unwrap_or(&0)
    }

    fn get_value_for_index(&self, index: usize) -> i32 {
        *self.data.get(&index).unwrap_or(&0)
    }

    /// Updates the voxel data for all LODs based on the current voxel values at maximum LOD, 0.
    ///
    /// This method propagates the voxel values from higher levels of detail (lower LOD values) to lower levels of detail (higher LOD values).
    /// It ensures that the voxel data at each LOD level is consistent with the data at the higher levels of detail.
    ///
    /// # Example
    ///
    /// ```
    /// use voxelis::voxtree::VoxTree;
    /// const MAX_LOD: usize = 2;
    /// let mut voxtree = VoxTree::<MAX_LOD>::new();
    /// voxtree.set_value(0, 0, 0, 0, 42);
    /// voxtree.update_lods();
    /// ```
    pub fn update_lods(&mut self) {
        self.update_lods_parallel_clone();
    }

    // Sequential implementation
    pub fn update_lods_sequential(&mut self) {
        for lod in 1..=MAX_LOD_LEVEL {
            let voxels_per_axis = calculate_voxels_per_axis(MAX_LOD_LEVEL - lod) as u8;

            for y in 0..voxels_per_axis {
                for z in 0..voxels_per_axis {
                    for x in 0..voxels_per_axis {
                        let index = Self::get_index_of(lod, x, y, z);
                        let child_indices = Self::get_lod_child_indices(lod, x, y, z);

                        let sum = child_indices
                            .iter()
                            .map(|&child_index| self.data.get(&child_index).unwrap_or(&0))
                            .sum::<i32>();

                        let average_value = (sum + 4) / 8; // Adding 4 ensures proper rounding
                        self.set_value_for_index(index, average_value);
                    }
                }
            }
        }
    }

    // Parallel implementation with cloning
    pub fn update_lods_parallel_clone(&mut self) {
        for lod in 1..=MAX_LOD_LEVEL {
            let voxels_per_axis = calculate_voxels_per_axis(MAX_LOD_LEVEL - lod) as u8;

            let data_snapshot = self.data.clone();

            let indices: Vec<(u8, u8, u8)> = (0..voxels_per_axis)
                .flat_map(|y| {
                    (0..voxels_per_axis)
                        .flat_map(move |z| (0..voxels_per_axis).map(move |x| (x, y, z)))
                })
                .collect();

            let updates: Vec<(usize, i32)> = indices
                .into_par_iter()
                .map(|(x, y, z)| {
                    let index = Self::get_index_of(lod, x, y, z);
                    let child_indices = Self::get_lod_child_indices(lod, x, y, z);

                    let sum = child_indices
                        .iter()
                        .map(|&child_index| *data_snapshot.get(&child_index).unwrap_or(&0))
                        .sum::<i32>();

                    let average_value = (sum + 4) / 8; // Adding 4 ensures proper rounding
                    (index, average_value)
                })
                .collect();

            for (index, value) in updates {
                self.set_value_for_index(index, value);
            }
        }
    }

    /// Calculates the index of a voxel in the data vector based on the level of detail (LOD) and coordinates.
    ///
    /// # Parameters
    ///
    /// - `lod`: The level of detail (0 is the maximum detail, `MAX_LOD_LEVEL` lowest).
    /// - `x`: The x-coordinate of the voxel at the specified LOD.
    /// - `y`: The y-coordinate of the voxel at the specified LOD.
    /// - `z`: The z-coordinate of the voxel at the specified LOD.
    ///
    /// # Returns
    ///
    /// The index of the voxel in the data vector as a `usize`.
    ///
    /// # Note
    ///
    /// This method adjusts the LOD level to account for the inverse relationship between LOD and detail level.
    /// A higher LOD level corresponds to a lower detail level, for example, for `MAX_LOD_LEVEL = 2`:
    /// - LOD 0: Highest detail level is LOD level 2 internally
    /// - LOD 1: Lower detail level is LOD level 1 internally
    /// - LOD 2: Lowest detail level is LOD level 0 internally
    fn get_index_of(lod: usize, x: u8, y: u8, z: u8) -> usize {
        // Validate LOD level
        assert!(
            lod <= MAX_LOD_LEVEL,
            "LOD level {} exceeds MAX_LOD_LEVEL {}",
            lod,
            MAX_LOD_LEVEL
        );

        // Map the external LOD (where lod = 0 is highest detail) to internal LOD (for calculations)
        let internal_lod = MAX_LOD_LEVEL - lod;

        // Calculate the number of voxels per axis at the given LOD
        let voxels_per_axis = calculate_voxels_per_axis(internal_lod);

        // Validate voxel coordinates
        let x = x as usize;
        let y = y as usize;
        let z = z as usize;

        assert!(
            x < voxels_per_axis && y < voxels_per_axis && z < voxels_per_axis,
            "Voxel coordinates ({}, {}, {}) are out of bounds for LOD {} with size {}",
            x,
            y,
            z,
            lod,
            voxels_per_axis
        );

        // Calculate the total number of voxels before the current LOD
        let lod_data_offset = if internal_lod == 0 {
            0
        } else {
            calculate_total_voxel_count(internal_lod - 1)
        };

        // Calculate the index within the current LOD
        let index_in_lod = x + z * voxels_per_axis + y * voxels_per_axis * voxels_per_axis;

        // Total index is the sum of the offset and the index within the LOD
        let index = lod_data_offset + index_in_lod;

        assert!(
            index < Self::MAX_VOXEL_COUNT,
            "Calculated index {} exceeds MAX_VOXEL_COUNT {}",
            index,
            Self::MAX_VOXEL_COUNT
        );

        index
    }

    /// Calculates the indices of the child voxels at the next level of detail (LOD) for a given voxel.
    ///
    /// # Parameters
    ///
    /// - `lod`: The current level of detail (must be greater than 0).
    /// - `x`: The x-coordinate of the voxel at the current LOD.
    /// - `y`: The y-coordinate of the voxel at the current LOD.
    /// - `z`: The z-coordinate of the voxel at the current LOD.
    ///
    /// # Returns
    ///
    /// An array of 8 `usize` values representing the indices of the child voxels in the data vector.
    ///
    /// # Panics
    ///
    /// This method will panic if `lod` is 0, as there are no child voxels at the maximum level of detail.
    fn get_lod_child_indices(lod: usize, x: u8, y: u8, z: u8) -> [usize; 8] {
        // At the maximum internal LOD (lowest detail), there are no child indices
        assert!(
            lod > 0,
            "LOD {} has no child indices (maximum LOD is {})",
            lod,
            MAX_LOD_LEVEL
        );

        // Calculate the child LOD
        let child_lod = lod - 1;

        // Each voxel splits into 8 child voxels at higher resolution
        let x = x * 2;
        let y = y * 2;
        let z = z * 2;

        [
            Self::get_index_of(child_lod, x, y, z),
            Self::get_index_of(child_lod, x + 1, y, z),
            Self::get_index_of(child_lod, x, y, z + 1),
            Self::get_index_of(child_lod, x + 1, y, z + 1),
            Self::get_index_of(child_lod, x, y + 1, z),
            Self::get_index_of(child_lod, x + 1, y + 1, z),
            Self::get_index_of(child_lod, x, y + 1, z + 1),
            Self::get_index_of(child_lod, x + 1, y + 1, z + 1),
        ]
    }

    pub fn iter(&self) -> VoxTreeIterator<MAX_LOD_LEVEL> {
        VoxTreeIterator::new(&self.data)
    }

    pub fn for_each_mut<F>(&mut self, mut f: F)
    where
        F: FnMut(usize, &mut i32),
    {
        let voxels_per_axis = calculate_voxels_per_axis(MAX_LOD_LEVEL);
        let max_size = voxels_per_axis * voxels_per_axis * voxels_per_axis;
        let min_index = calculate_lod_data_index(0, MAX_LOD_LEVEL);
        let max_index = min_index + max_size;

        for index in min_index..max_index {
            let mut value = 0;

            f(index - min_index, &mut value);

            self.set_value_for_index(index, value);
        }
    }

    pub fn to_vec(&self, lod: usize) -> Vec<i32> {
        let voxels_per_axis = calculate_voxels_per_axis(MAX_LOD_LEVEL - lod);
        let size = voxels_per_axis * voxels_per_axis * voxels_per_axis;
        let min_index = calculate_lod_data_index(lod, MAX_LOD_LEVEL);
        let max_index = min_index + size;

        let mut data = vec![0; size];

        for index in min_index..max_index {
            let value = self.get_value_for_index(index);
            data[index - min_index] = value;
        }

        data
    }
}

impl<const MAX_LOD_LEVEL: usize> Default for VoxTree<MAX_LOD_LEVEL> {
    /// Creates a default [VoxTree] instance.
    ///
    /// This method is equivalent to calling [new](VoxTree::new).
    ///
    /// # Returns
    ///
    /// A new [VoxTree] instance with the data vector initialized to zeros.
    ///
    /// # Example
    ///
    /// ```
    /// use voxelis::voxtree::VoxTree;
    /// const MAX_LOD: usize = 2;
    /// let voxtree = VoxTree::<MAX_LOD>::default();
    /// ```
    fn default() -> Self {
        Self::new()
    }
}

#[cfg(test)]
mod tests {
    use crate::voxtree::*;

    #[test]
    fn voxels_per_axis() {
        assert_eq!(calculate_voxels_per_axis(0), 1);
        assert_eq!(calculate_voxels_per_axis(1), 2);
        assert_eq!(calculate_voxels_per_axis(2), 4);
        assert_eq!(calculate_voxels_per_axis(3), 8);
        assert_eq!(calculate_voxels_per_axis(4), 16);
        assert_eq!(calculate_voxels_per_axis(5), 32);
        assert_eq!(calculate_voxels_per_axis(6), 64);
    }

    #[test]
    fn voxel_area() {
        assert_eq!(calculate_voxel_area(0), 1);
        assert_eq!(calculate_voxel_area(1), 4);
        assert_eq!(calculate_voxel_area(2), 16);
        assert_eq!(calculate_voxel_area(3), 64);
        assert_eq!(calculate_voxel_area(4), 256);
        assert_eq!(calculate_voxel_area(5), 1024);
        assert_eq!(calculate_voxel_area(6), 4096);
    }

    #[test]
    fn voxel_volume() {
        assert_eq!(calculate_voxel_volume(0), 1);
        assert_eq!(calculate_voxel_volume(1), 8);
        assert_eq!(calculate_voxel_volume(2), 64);
        assert_eq!(calculate_voxel_volume(3), 512);
        assert_eq!(calculate_voxel_volume(4), 4096);
        assert_eq!(calculate_voxel_volume(5), 32_768);
        assert_eq!(calculate_voxel_volume(6), 262_144);
    }

    #[test]
    fn total_voxel_count() {
        assert_eq!(calculate_total_voxel_count(0), 1);
        assert_eq!(calculate_total_voxel_count(1), 9);
        assert_eq!(calculate_total_voxel_count(2), 73);
        assert_eq!(calculate_total_voxel_count(3), 585);
        assert_eq!(calculate_total_voxel_count(4), 4681);
        assert_eq!(calculate_total_voxel_count(5), 37_449);
        assert_eq!(calculate_total_voxel_count(6), 299_593);
    }

    #[test]
    fn voxtree_new() {
        let voxtree = VoxTree::<2>::new();
        assert_eq!(voxtree.data.len(), 0);
    }

    #[test]
    fn voxtree_default() {
        let voxtree = VoxTree::<2>::default();
        assert_eq!(voxtree.data.len(), 0);
    }

    #[test]
    fn voxtree_get_index_of() {
        // lod 2: 1 voxel
        assert_eq!(VoxTree::<2>::get_index_of(2, 0, 0, 0), 0);
        // lod 1: 8 voxels
        assert_eq!(VoxTree::<2>::get_index_of(1, 0, 0, 0), 1);
        // lod 0: 64 voxels
        assert_eq!(VoxTree::<2>::get_index_of(0, 0, 0, 0), 9);
        assert_eq!(VoxTree::<2>::get_index_of(0, 3, 3, 3), 72);
    }

    #[test]
    fn voxtree_get_set_value() {
        let mut voxtree = VoxTree::<2>::new();

        assert_eq!(voxtree.get_value_for_index(0), 0);
        assert_eq!(voxtree.get_value(2, 0, 0, 0), 0);

        voxtree.set_value(2, 0, 0, 0, 2);

        assert_eq!(voxtree.get_value_for_index(0), 2);
        assert_eq!(voxtree.get_value(2, 0, 0, 0), 2);

        assert_eq!(voxtree.get_value_for_index(9), 0);
        assert_eq!(voxtree.get_value(0, 0, 0, 0), 0);

        voxtree.set_value(0, 0, 0, 0, 1);

        assert_eq!(voxtree.get_value_for_index(9), 1);
        assert_eq!(voxtree.get_value(0, 0, 0, 0), 1);

        assert_eq!(voxtree.get_value_for_index(72), 0);
        assert_eq!(voxtree.get_value(0, 3, 3, 3), 0);

        voxtree.set_value(0, 3, 3, 3, 3);

        assert_eq!(voxtree.get_value_for_index(72), 3);
        assert_eq!(voxtree.get_value(0, 3, 3, 3), 3);
    }

    #[test]
    fn voxtree_get_lod_child_indices() {
        assert_eq!(
            VoxTree::<2>::get_lod_child_indices(2, 0, 0, 0),
            [1, 2, 3, 4, 5, 6, 7, 8]
        );

        assert_eq!(
            VoxTree::<2>::get_lod_child_indices(1, 0, 0, 0),
            [9, 10, 13, 14, 25, 26, 29, 30]
        );
        assert_eq!(
            VoxTree::<2>::get_lod_child_indices(1, 1, 0, 0),
            [11, 12, 15, 16, 27, 28, 31, 32]
        );
        assert_eq!(
            VoxTree::<2>::get_lod_child_indices(1, 0, 0, 1),
            [17, 18, 21, 22, 33, 34, 37, 38]
        );
        assert_eq!(
            VoxTree::<2>::get_lod_child_indices(1, 1, 0, 1),
            [19, 20, 23, 24, 35, 36, 39, 40]
        );
        assert_eq!(
            VoxTree::<2>::get_lod_child_indices(1, 0, 1, 0),
            [41, 42, 45, 46, 57, 58, 61, 62]
        );
        assert_eq!(
            VoxTree::<2>::get_lod_child_indices(1, 1, 1, 0),
            [43, 44, 47, 48, 59, 60, 63, 64]
        );
        assert_eq!(
            VoxTree::<2>::get_lod_child_indices(1, 0, 1, 1),
            [49, 50, 53, 54, 65, 66, 69, 70]
        );
        assert_eq!(
            VoxTree::<2>::get_lod_child_indices(1, 1, 1, 1),
            [51, 52, 55, 56, 67, 68, 71, 72]
        );
    }

    #[test]
    fn voxtree_update_lods() {
        let mut voxtree = VoxTree::<2>::new();

        // lod 0: 64 voxels
        voxtree.set_value(0, 0, 0, 0, 1);
        voxtree.set_value(0, 1, 0, 0, 1);
        voxtree.set_value(0, 0, 0, 1, 1);
        voxtree.set_value(0, 1, 0, 1, 1);
        voxtree.set_value(0, 0, 1, 0, 1);
        voxtree.set_value(0, 1, 1, 0, 1);
        voxtree.set_value(0, 0, 1, 1, 1);
        voxtree.set_value(0, 1, 1, 1, 1);

        voxtree.set_value(0, 2, 0, 0, 1);
        voxtree.set_value(0, 3, 0, 0, 1);
        voxtree.set_value(0, 2, 0, 1, 1);
        voxtree.set_value(0, 3, 0, 1, 1);
        voxtree.set_value(0, 2, 1, 0, 1);
        voxtree.set_value(0, 3, 1, 0, 1);
        voxtree.set_value(0, 2, 1, 1, 1);

        voxtree.set_value(0, 0, 0, 2, 1);
        voxtree.set_value(0, 1, 0, 2, 1);
        voxtree.set_value(0, 0, 0, 3, 1);
        voxtree.set_value(0, 1, 0, 3, 1);
        voxtree.set_value(0, 0, 1, 2, 1);
        voxtree.set_value(0, 1, 1, 2, 1);

        voxtree.set_value(0, 2, 0, 2, 1);
        voxtree.set_value(0, 3, 0, 2, 1);
        voxtree.set_value(0, 2, 0, 3, 1);
        voxtree.set_value(0, 3, 0, 3, 1);
        voxtree.set_value(0, 2, 1, 2, 1);

        voxtree.set_value(0, 0, 2, 0, 1);
        voxtree.set_value(0, 1, 2, 0, 1);
        voxtree.set_value(0, 0, 2, 1, 1);
        voxtree.set_value(0, 1, 2, 1, 1);

        voxtree.set_value(0, 2, 2, 0, 1);
        voxtree.set_value(0, 3, 2, 0, 1);
        voxtree.set_value(0, 2, 2, 1, 1);

        voxtree.set_value(0, 0, 2, 2, 1);
        voxtree.set_value(0, 1, 2, 2, 1);

        voxtree.set_value(0, 2, 2, 2, 1);

        voxtree.update_lods();

        // lod 1: 8 voxels
        assert_eq!(voxtree.get_value(1, 0, 0, 0), 1);
        assert_eq!(voxtree.get_value(1, 1, 0, 0), 1);
        assert_eq!(voxtree.get_value(1, 0, 0, 1), 1);
        assert_eq!(voxtree.get_value(1, 1, 0, 1), 1);
        assert_eq!(voxtree.get_value(1, 0, 1, 0), 1);
        assert_eq!(voxtree.get_value(1, 1, 1, 0), 0);
        assert_eq!(voxtree.get_value(1, 0, 1, 1), 0);
        assert_eq!(voxtree.get_value(1, 1, 1, 1), 0);

        // lod 2: 1 voxel
        assert_eq!(voxtree.get_value(2, 0, 0, 0), 1);
    }
}
