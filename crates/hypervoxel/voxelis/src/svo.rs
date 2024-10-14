use std::marker::PhantomData;

use glam::IVec3;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct Voxel<T> {
    pub value: T,
}

pub struct Octree<T: Copy + Default + PartialEq> {
    max_depth: u8,
    root: Option<Box<OctreeNode<T>>>,
    _phantom: PhantomData<T>,
}

pub enum OctreeNode<T: Copy + Default + PartialEq> {
    Branch(Box<[Option<OctreeNode<T>>; 8]>),
    Leaf(Voxel<T>),
}

fn child_index(position: IVec3, depth: u8, max_depth: u8) -> usize {
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

impl<T: Copy + Default + PartialEq> Octree<T> {
    pub fn new(max_depth: u8) -> Self {
        Self {
            max_depth,
            root: None,
            _phantom: PhantomData,
        }
    }

    pub fn insert(&mut self, position: IVec3, voxel: Voxel<T>) {
        if self.root.is_none() {
            self.root = Some(Box::new(OctreeNode::new_branch()));
        }

        if let Some(root) = &mut self.root {
            root.insert_at_depth(position, 0, self.max_depth, voxel);
        }
    }

    pub fn get(&self, position: IVec3) -> Option<&Voxel<T>> {
        self.root
            .as_ref()
            .and_then(|root| root.get_at_depth(position, 0, self.max_depth))
    }

    pub fn is_empty(&self) -> bool {
        self.root.as_ref().map_or(true, |root| root.is_empty())
    }

    pub fn is_full(&self) -> bool {
        self.root.as_ref().map_or(false, |root| root.is_full())
    }

    pub fn calculate_voxels_per_axis(lod_level: usize) -> usize {
        1 << lod_level
    }

    pub fn clear(&mut self) {
        if let Some(root) = &mut self.root {
            root.clear();
            if root.is_empty() {
                self.root = None;
            }
        }
    }

    pub fn to_vec(&self) -> Vec<T> {
        let voxels_per_axis = Self::calculate_voxels_per_axis(self.max_depth as usize);
        let shift_y: usize = 1 << (2 * self.max_depth as usize);
        let shift_z: usize = 1 << self.max_depth as usize;

        let mut data = vec![T::default(); voxels_per_axis * voxels_per_axis * voxels_per_axis];

        if let Some(root) = &self.root {
            for y in 0..voxels_per_axis {
                let base_index_y = y * shift_y;
                for z in 0..voxels_per_axis {
                    let base_index_z = base_index_y + z * shift_z;
                    for x in 0..voxels_per_axis {
                        let index = base_index_z + x;
                        if let Some(voxel) = root.get_at_depth(
                            IVec3::new(x as i32, y as i32, z as i32),
                            0,
                            self.max_depth,
                        ) {
                            data[index] = voxel.value;
                        }
                    }
                }
            }
        }

        data
    }
}

impl<T: Copy + Default + PartialEq> OctreeNode<T> {
    fn new_branch() -> Self {
        OctreeNode::Branch(Box::new([None, None, None, None, None, None, None, None]))
    }

    fn insert_at_depth(&mut self, position: IVec3, depth: u8, max_depth: u8, voxel: Voxel<T>) {
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
                    let index = child_index(position, depth, max_depth);

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
            }
        }
    }

    fn get_at_depth(&self, position: IVec3, depth: u8, max_depth: u8) -> Option<&Voxel<T>> {
        match self {
            OctreeNode::Leaf(voxel) => Some(voxel),
            OctreeNode::Branch(children) => {
                let index = child_index(position, depth, max_depth);
                children[index]
                    .as_ref()
                    .and_then(|child| child.get_at_depth(position, depth + 1, max_depth))
            }
        }
    }

    fn set_child(&mut self, index: usize, child: OctreeNode<T>) {
        if let OctreeNode::Branch(children) = self {
            children[index] = Some(child);
        } else {
            panic!("OctreeNode::set_child called on non-branch node");
        }
    }

    fn try_merge_children_into_leaf(&self) -> Option<Voxel<T>> {
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

    pub fn is_empty(&self) -> bool {
        match self {
            OctreeNode::Leaf(_) => false,
            OctreeNode::Branch(children) => children
                .iter()
                .all(|child| child.as_ref().map_or(true, |child| child.is_empty())),
        }
    }

    pub fn is_full(&self) -> bool {
        match self {
            OctreeNode::Leaf(_) => true,
            OctreeNode::Branch(children) => children
                .iter()
                .all(|child| child.as_ref().map_or(false, |child| child.is_full())),
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
        }
    }
}

#[cfg(test)]
mod tests {
    use glam::IVec3;

    use super::{Octree, Voxel};

    #[test]
    fn test_create() {
        let octree = Octree::<u8>::new(3);
        assert_eq!(octree.max_depth, 3);
        assert!(octree.root.is_none());
    }

    #[test]
    fn test_insert_and_get() {
        let mut octree = Octree::<u8>::new(3);

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
    fn test_is_empty_and_is_full() {
        let mut octree = Octree::<u8>::new(3);
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
    fn test_clear() {
        let mut octree = Octree::<u8>::new(3);

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
        // assert!(octree.is_empty());

        // for &pos in positions.iter() {
        //     assert!(octree.get(pos).is_none());
        // }
    }
}
