use bevy::utils::default;

const fn calculate_voxel_size(lod: usize) -> usize
{
    1 << lod
}

const fn calculate_chunk_area(lod: usize) -> usize
{
    let voxel_size = calculate_voxel_size(lod);
    voxel_size * voxel_size
}
const fn calculate_chunk_volume(lod: usize) -> usize
{
    let voxel_size = calculate_voxel_size(lod);
    voxel_size * voxel_size * voxel_size
}

const fn calculate_chunk_size(max_lod: usize) -> usize
{
    // On
    //  lod 0: we have 1 voxel
    //  lod 1: we have 8 voxels
    //  lod 2: we have 64 voxels
    //  ...
    // So, we can use the formula for the sum of powers
    // 1 + 8 + 64 + ... + (8^max_lod)
    // (8^(max_lod + 1) - 1) / (8 - 1)
    // (8^(max_lod + 1) - 1) / 7
    // ((2^3)^(max_lod + 1))) / 7
    // (2^(3 * (max_lod + 1))) / 7
    // (1 << (3 * (max_lod + 1))) / 7
    (1 << (3 * (max_lod + 1))) / 7
}

struct VoxTree<const MAX_LOD_LEVEL: usize> {
    data: Vec<i32>,
}

impl<const MAX_LOD_LEVEL: usize> VoxTree<MAX_LOD_LEVEL> {
    const MAX_SIZE: usize = calculate_chunk_size(MAX_LOD_LEVEL);

    fn new() -> Self {
        Self { data: vec![0; Self::MAX_SIZE] }
    }

    fn set_value(&mut self, lod: usize, x: u8, y: u8, z: u8, value: i32)
    {
        let lod = MAX_LOD_LEVEL - lod;

        assert!(lod <= MAX_LOD_LEVEL);
        assert!(x < calculate_voxel_size(lod) as u8);
        assert!(y < calculate_voxel_size(lod) as u8);
        assert!(z < calculate_voxel_size(lod) as u8);

        let index = Self::get_index_of(lod, x, y, z);
        self.data[index] = value;
    }

    fn get_value(&self, lod: usize, x: u8, y: u8, z: u8) -> i32
    {
        let lod = MAX_LOD_LEVEL - lod;

        assert!(lod <= MAX_LOD_LEVEL);
        assert!(x < calculate_voxel_size(lod) as u8);
        assert!(y < calculate_voxel_size(lod) as u8);
        assert!(z < calculate_voxel_size(lod) as u8);

        let index = Self::get_index_of(lod, x, y, z);
        self.data[index]
    }

    fn update_lods(&mut self)
    {
        for lod in 0..MAX_LOD_LEVEL
        {
            let lod = MAX_LOD_LEVEL - lod - 1;
            let voxel_size = calculate_voxel_size(lod) as u8;

            for y in 0..voxel_size {
                for z in 0..voxel_size {
                    for x in 0..voxel_size {
                        let index = Self::get_index_of(lod, x, y, z);
                        assert!(index < Self::MAX_SIZE);

                        let child_indices = Self::get_lod_child_indices(lod, x, y, z);

                        let mut average_value = 0;

                        for child_index in child_indices {
                            average_value += self.data[child_index];
                        }

                        self.data[index] = (average_value as f64 / 8.0).round() as i32;
                    }
                }
            }
        }
    }

    fn get_index_of(lod: usize, x: u8, y: u8, z: u8) -> usize
    {
        assert!(lod <= MAX_LOD_LEVEL);

        if lod == 0 {
            return 0;
        }

        let lod_data_offset = calculate_chunk_size(lod - 1);
        let chunk_area = calculate_chunk_area(lod);
        let voxel_size = calculate_voxel_size(lod);

        lod_data_offset + y as usize * chunk_area + z as usize * voxel_size + x as usize
    }

    fn get_lod_child_indices(lod: usize, x: u8, y: u8, z: u8) -> [usize; 8]
    {
        let child_lod = lod + 1;
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
}

#[cfg(test)]
mod tests {
    use crate::voxtree::*;

    #[test]
    fn voxel_size() {
        assert_eq!(calculate_voxel_size(0), 1);
        assert_eq!(calculate_voxel_size(1), 2);
        assert_eq!(calculate_voxel_size(2), 4);
        assert_eq!(calculate_voxel_size(3), 8);
        assert_eq!(calculate_voxel_size(4), 16);
        assert_eq!(calculate_voxel_size(5), 32);
        assert_eq!(calculate_voxel_size(6), 64);
    }

    #[test]
    fn chunk_area() {
        assert_eq!(calculate_chunk_area(0), 1);
        assert_eq!(calculate_chunk_area(1), 4);
        assert_eq!(calculate_chunk_area(2), 16);
        assert_eq!(calculate_chunk_area(3), 64);
        assert_eq!(calculate_chunk_area(4), 256);
        assert_eq!(calculate_chunk_area(5), 1024);
        assert_eq!(calculate_chunk_area(6), 4096);
    }

    #[test]
    fn chunk_volume() {
        assert_eq!(calculate_chunk_volume(0), 1);
        assert_eq!(calculate_chunk_volume(1), 8);
        assert_eq!(calculate_chunk_volume(2), 64);
        assert_eq!(calculate_chunk_volume(3), 512);
        assert_eq!(calculate_chunk_volume(4), 4096);
        assert_eq!(calculate_chunk_volume(5), 32_768);
        assert_eq!(calculate_chunk_volume(6), 262_144);
    }

    #[test]
    fn chunk_size() {
        assert_eq!(calculate_chunk_size(0), 1);
        assert_eq!(calculate_chunk_size(1), 9);
        assert_eq!(calculate_chunk_size(2), 73);
        assert_eq!(calculate_chunk_size(3), 585);
        assert_eq!(calculate_chunk_size(4), 4681);
        assert_eq!(calculate_chunk_size(5), 37_449);
        assert_eq!(calculate_chunk_size(6), 299_593);
    }

    #[test]
    fn voxtree_new() {
        let voxtree = VoxTree::<2>::new();
        assert_eq!(voxtree.data.len(), 73);
    }

    #[test]
    fn voxtree_get_index_of() {
        assert_eq!(VoxTree::<2>::get_index_of(0, 0, 0, 0), 0);
        assert_eq!(VoxTree::<2>::get_index_of(1, 0, 0, 0), 1);
        assert_eq!(VoxTree::<2>::get_index_of(2, 0, 0, 0), 9);
        assert_eq!(VoxTree::<2>::get_index_of(2, 3, 3, 3), 72);
    }

    #[test]
    fn voxtree_get_set_value() {
        let mut voxtree = VoxTree::<2>::new();

        assert_eq!(voxtree.data[9], 0);
        assert_eq!(voxtree.get_value(0, 0, 0, 0), 0);

        voxtree.set_value(0, 0, 0, 0, 1);

        assert_eq!(voxtree.data[9], 1);
        assert_eq!(voxtree.get_value(0, 0, 0, 0), 1);
    }

    #[test]
    fn voxtree_get_lod_child_indices() {
        assert_eq!(VoxTree::<2>::get_lod_child_indices(1, 0, 0, 0), [9, 10, 13, 14, 25, 26, 29, 30]);
        assert_eq!(VoxTree::<2>::get_lod_child_indices(1, 1, 0, 0), [11, 12, 15, 16, 27, 28, 31, 32]);
        assert_eq!(VoxTree::<2>::get_lod_child_indices(1, 0, 0, 1), [17, 18, 21, 22, 33, 34, 37, 38]);
        assert_eq!(VoxTree::<2>::get_lod_child_indices(1, 1, 0, 1), [19, 20, 23, 24, 35, 36, 39, 40]);
        assert_eq!(VoxTree::<2>::get_lod_child_indices(1, 0, 1, 0), [41, 42, 45, 46, 57, 58, 61, 62]);
        assert_eq!(VoxTree::<2>::get_lod_child_indices(1, 1, 1, 0), [43, 44, 47, 48, 59, 60, 63, 64]);
        assert_eq!(VoxTree::<2>::get_lod_child_indices(1, 0, 1, 1), [49, 50, 53, 54, 65, 66, 69, 70]);
        assert_eq!(VoxTree::<2>::get_lod_child_indices(1, 1, 1, 1), [51, 52, 55, 56, 67, 68, 71, 72]);
    }

    #[test]
    fn voxtree_update_lods()
    {
        let mut voxtree = VoxTree::<2>::new();

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

        assert_eq!(voxtree.get_value(1, 0, 0, 0), 1);
        assert_eq!(voxtree.get_value(1, 1, 0, 0), 1);
        assert_eq!(voxtree.get_value(1, 0, 0, 1), 1);
        assert_eq!(voxtree.get_value(1, 1, 0, 1), 1);
        assert_eq!(voxtree.get_value(1, 0, 1, 0), 1);
        assert_eq!(voxtree.get_value(1, 1, 1, 0), 0);
        assert_eq!(voxtree.get_value(1, 0, 1, 1), 0);
        assert_eq!(voxtree.get_value(1, 1, 1, 1), 0);

        assert_eq!(voxtree.get_value(2, 0, 0, 0), 1);
    }
}
