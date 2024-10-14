use glam::IVec3;

use crate::chunk::{SHIFT_Y, SHIFT_Z, VOXELS_PER_AXIS};

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct Voxel {
    pub value: u8,
}

pub enum OctreeNode {
    Root {
        max_depth: u32,
        child: Option<Box<OctreeNode>>,
    },
    Branch(Box<[Option<OctreeNode>; 8]>),
    Leaf(Voxel),
}

impl OctreeNode {
    pub fn create(max_depth: u32) -> Self {
        OctreeNode::Root {
            max_depth,
            child: None,
        }
    }

    fn new_branch() -> Self {
        OctreeNode::Branch(Box::new([None, None, None, None, None, None, None, None]))
    }

    pub fn insert(&mut self, position: IVec3, voxel: Voxel) {
        match self {
            OctreeNode::Root { max_depth, child } => {
                let max_depth = *max_depth;
                if child.is_none() {
                    *child = Some(Box::new(OctreeNode::new_branch()));
                }

                if let Some(child_node) = child {
                    child_node.insert_at_depth(position, 0, max_depth, voxel);
                }
            }
            _ => panic!("OctreeNode::insert called on non-root node"),
        }
    }

    fn insert_at_depth(&mut self, position: IVec3, depth: u32, max_depth: u32, voxel: Voxel) {
        if depth == max_depth {
            *self = OctreeNode::Leaf(voxel);
        } else {
            match self {
                OctreeNode::Leaf(existing_voxel) => {
                    // If the existing leaf has the same value, no action is needed
                    if *existing_voxel == voxel {
                        return;
                    }

                    // Split the leaf into a branch
                    let mut branch = OctreeNode::new_branch();

                    // Initialize all children with the existing voxel value
                    for i in 0..8 {
                        branch.set_child(i, OctreeNode::Leaf(*existing_voxel));
                    }

                    // Replace self with the new branch
                    *self = branch;

                    // Now, insert the new voxel into the tree
                    self.insert_at_depth(position, depth, max_depth, voxel);
                }
                OctreeNode::Branch(children) => {
                    let index = Self::child_index(position, depth, max_depth);

                    if children[index].is_none() {
                        if depth + 1 == max_depth {
                            // We're at the level just before max_depth
                            children[index] = Some(OctreeNode::Leaf(voxel));
                        } else {
                            // Create a new branch node
                            children[index] = Some(OctreeNode::new_branch());
                            children[index].as_mut().unwrap().insert_at_depth(
                                position,
                                depth + 1,
                                max_depth,
                                voxel,
                            );
                        }
                    } else {
                        children[index].as_mut().unwrap().insert_at_depth(
                            position,
                            depth + 1,
                            max_depth,
                            voxel,
                        );
                    }

                    // After insertion, check if we can merge this branch into a leaf
                    if let Some(merged_voxel) = self.try_merge_children_into_leaf() {
                        *self = OctreeNode::Leaf(merged_voxel);
                    }
                }
                _ => panic!("OctreeNode::insert_at_depth called on non-leaf or branch node"),
            }
        }
    }

    fn set_child(&mut self, index: usize, child: OctreeNode) {
        if let OctreeNode::Branch(children) = self {
            children[index] = Some(child);
        } else {
            panic!("OctreeNode::set_child called on non-branch node");
        }
    }

    fn try_merge_children_into_leaf(&self) -> Option<Voxel> {
        if let OctreeNode::Branch(children) = self {
            // Get the first child (octant 0)
            let first_child = &children[0];

            // The first child must be a Leaf node to consider merging
            let first_voxel = match first_child {
                Some(OctreeNode::Leaf(voxel)) => *voxel,
                _ => return None, // Cannot merge if the first child is not a leaf
            };

            // Check if all other children are leaves with the same voxel value
            for child in children.iter() {
                match child {
                    Some(OctreeNode::Leaf(voxel)) if *voxel == first_voxel => continue,
                    _ => return None, // Cannot merge if any child is not a matching Leaf node
                }
            }

            return Some(first_voxel);
        }

        None
    }

    pub fn get(&self, position: IVec3) -> Option<&Voxel> {
        match self {
            OctreeNode::Root { max_depth, child } => {
                if let Some(child_node) = child {
                    child_node.get_at_depth(position, 0, *max_depth)
                } else {
                    None
                }
            }
            _ => panic!("OctreeNode::get called on non-root node"),
        }
    }

    fn get_at_depth(&self, position: IVec3, depth: u32, max_depth: u32) -> Option<&Voxel> {
        match self {
            OctreeNode::Leaf(voxel) => Some(voxel),
            OctreeNode::Branch(children) => {
                let index = Self::child_index(position, depth, max_depth);
                if let Some(child) = &children[index] {
                    child.get_at_depth(position, depth + 1, max_depth)
                } else {
                    None
                }
            }
            _ => None,
        }
    }

    fn child_index(position: IVec3, depth: u32, max_depth: u32) -> usize {
        let shift = max_depth - depth - 1;

        let mut index = 0;

        if ((position.x >> shift) & 1) == 1 {
            index |= 1;
        }
        if ((position.y >> shift) & 1) == 1 {
            index |= 2;
        }
        if ((position.z >> shift) & 1) == 1 {
            index |= 4;
        }

        index
    }

    pub fn is_empty(&self) -> bool {
        match self {
            OctreeNode::Leaf(_) => false,
            OctreeNode::Branch(children) => children
                .iter()
                .all(|child| child.as_ref().map_or(true, |child| child.is_empty())),
            OctreeNode::Root {
                max_depth: _,
                child,
            } => child.as_ref().map_or(true, |child| child.is_empty()),
        }
    }

    pub fn is_full(&self) -> bool {
        match self {
            OctreeNode::Leaf(_) => true,
            OctreeNode::Branch(children) => children
                .iter()
                .all(|child| child.as_ref().map_or(false, |child| child.is_full())),
            OctreeNode::Root { child, .. } => child.as_ref().map_or(false, |child| child.is_full()),
        }
    }

    pub fn clear(&mut self) {
        match self {
            OctreeNode::Leaf(_) => *self = OctreeNode::Leaf(Voxel::default()),
            OctreeNode::Branch(children) => {
                children.iter_mut().for_each(|child| {
                    if let Some(child) = child {
                        child.clear();
                    }
                });

                // After clearing, attempt to merge the branch into a leaf
                if let Some(merged_voxel) = self.try_merge_children_into_leaf() {
                    *self = OctreeNode::Leaf(merged_voxel);
                }
            }
            OctreeNode::Root { child, .. } => {
                if let Some(child) = child {
                    child.clear();

                    // After clearing, attempt to merge the child
                    if let Some(merged_voxel) = child.try_merge_children_into_leaf() {
                        *child = Box::new(OctreeNode::Leaf(merged_voxel));
                    }
                }

                // If the child is now empty, set it to None
                if child.as_ref().map_or(true, |child| child.is_empty()) {
                    *child = None;
                }
            }
        }
    }

    /// Calculates the total memory size occupied by the octree.
    pub fn total_memory_size(&self) -> usize {
        // Assuming the discriminant size is 1 byte (since we have only a few variants)
        let discriminant_size = size_of::<u8>();

        match self {
            OctreeNode::Root { child, .. } => {
                let mut size = discriminant_size;
                size += size_of::<u32>(); // max_depth
                size += size_of::<Option<Box<OctreeNode>>>(); // child pointer

                if let Some(child_node) = child {
                    // The size of the Box pointer is included in the Option<Box<_>>
                    // Add the size of the child node recursively
                    size += child_node.total_memory_size();
                }

                size
            }
            OctreeNode::Branch(children_box) => {
                let mut size = discriminant_size;
                size += size_of::<Box<[Option<OctreeNode>; 8]>>(); // Box pointer (8 bytes)

                // Add the size of the heap allocation
                size += size_of::<[Option<OctreeNode>; 8]>(); // Size of the array on the heap

                // Recursively add the sizes of existing child nodes
                children_box.iter().for_each(|child_option| {
                    if let Some(child_node) = child_option {
                        size += child_node.total_memory_size();
                    }
                });

                size
            }
            OctreeNode::Leaf(_) => {
                let mut size = discriminant_size;
                size += size_of::<Voxel>(); // Size of the Voxel

                size
            }
        }
    }

    pub fn to_vec(&self, lod: usize) -> Vec<i32> {
        let mut data =
            vec![0; VOXELS_PER_AXIS as usize * VOXELS_PER_AXIS as usize * VOXELS_PER_AXIS as usize];

        for y in 0..VOXELS_PER_AXIS {
            let base_index_y = (y as usize) * SHIFT_Y;
            for z in 0..VOXELS_PER_AXIS {
                let base_index_z = base_index_y + (z as usize) * SHIFT_Z;
                for x in 0..VOXELS_PER_AXIS {
                    let index = base_index_z + x as usize;

                    if let Some(voxel) = self.get(IVec3::new(x as i32, y as i32, z as i32)) {
                        // println!("index: {} value: {}", index, voxel.value);
                        data[index] = voxel.value as i32;
                    } else {
                        // println!("index: {} value: 0", index);
                        data[index] = 0;
                    }
                }
            }
        }

        data
    }
}

#[cfg(test)]
mod tests {
    use crate::svo::{OctreeNode, Voxel};
    use glam::IVec3;

    #[test]
    fn test_create() {
        let octree = OctreeNode::create(3);
        if let OctreeNode::Root { max_depth, child } = octree {
            assert_eq!(max_depth, 3);
            assert!(child.is_none());
        } else {
            panic!("OctreeNode::create did not create a root node");
        }
    }

    #[test]
    fn test_insert_and_get() {
        let mut octree = OctreeNode::create(3);

        let positions = [
            IVec3::new(0, 0, 0),
            IVec3::new(0, 0, 1),
            IVec3::new(0, 1, 0),
            IVec3::new(0, 1, 1),
            IVec3::new(1, 0, 0),
            IVec3::new(1, 0, 1),
            IVec3::new(1, 1, 0),
            IVec3::new(1, 1, 1),
        ];

        for (i, &pos) in positions.iter().enumerate() {
            octree.insert(
                pos,
                Voxel {
                    value: (i + 1) as u8,
                },
            );
        }

        for (i, &pos) in positions.iter().enumerate() {
            assert_eq!(octree.get(pos).unwrap().value, (i + 1) as u8);
        }
    }

    #[test]
    fn test_memory_usage() {
        let mut octree = OctreeNode::create(3);
        assert_eq!(octree.total_memory_size(), 1 + 4 + 8);

        let positions = [
            IVec3::new(0, 0, 0),
            IVec3::new(0, 0, 1),
            IVec3::new(0, 1, 0),
            IVec3::new(0, 1, 1),
            IVec3::new(1, 0, 0),
            IVec3::new(1, 0, 1),
            IVec3::new(1, 1, 0),
            IVec3::new(1, 1, 1),
        ];

        for (i, &pos) in positions.iter().enumerate() {
            octree.insert(
                pos,
                Voxel {
                    value: (i + 1) as u8,
                },
            );
        }

        assert_eq!(octree.total_memory_size(), 440);

        for &pos in positions.iter() {
            octree.insert(pos, Voxel { value: 0 });
        }

        // assert_eq!(octree.total_memory_size(), 1 + 4 + 8);
        assert_eq!(octree.total_memory_size(), 289);
    }

    #[test]
    fn test_is_empty_and_is_full() {
        let mut octree = OctreeNode::create(3);
        assert!(octree.is_empty());
        assert!(!octree.is_full());

        octree.insert(IVec3::new(0, 0, 0), Voxel { value: 1 });
        assert!(!octree.is_empty());
        assert!(!octree.is_full());

        for x in 0..8 {
            for y in 0..8 {
                for z in 0..8 {
                    octree.insert(IVec3::new(x, y, z), Voxel { value: 1 });
                }
            }
        }

        assert!(!octree.is_empty());
        assert!(octree.is_full());
    }

    #[test]
    fn test_iter_voxels() {
        let mut octree = OctreeNode::create(3);

        let positions = [
            IVec3::new(0, 0, 0),
            IVec3::new(0, 0, 1),
            IVec3::new(0, 1, 0),
            IVec3::new(0, 1, 1),
            IVec3::new(1, 0, 0),
            IVec3::new(1, 0, 1),
            IVec3::new(1, 1, 0),
            IVec3::new(1, 1, 1),
        ];

        for (i, &pos) in positions.iter().enumerate() {
            octree.insert(
                pos,
                Voxel {
                    value: (i + 1) as u8,
                },
            );
        }
    }

    #[test]
    fn test_clear() {
        let mut octree = OctreeNode::create(3);

        let positions = [
            IVec3::new(0, 0, 0),
            IVec3::new(0, 0, 1),
            IVec3::new(0, 1, 0),
            IVec3::new(0, 1, 1),
            IVec3::new(1, 0, 0),
            IVec3::new(1, 0, 1),
            IVec3::new(1, 1, 0),
            IVec3::new(1, 1, 1),
        ];

        for (i, &pos) in positions.iter().enumerate() {
            octree.insert(
                pos,
                Voxel {
                    value: (i + 1) as u8,
                },
            );
        }

        octree.clear();
        assert!(octree.is_empty());
        for &pos in positions.iter() {
            assert!(octree.get(pos).is_none());
        }
    }
}
