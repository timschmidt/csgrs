use std::marker::PhantomData;

use glam::IVec3;

use crate::voxel::VoxelTrait;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct Voxel<T: VoxelTrait> {
    pub value: T,
}

#[derive(Default, Debug)]
pub struct Octree<T: VoxelTrait> {
    max_depth: u8,
    root: Option<Box<OctreeNode<T>>>,
    _phantom: PhantomData<T>,
}

#[derive(Debug)]
pub enum OctreeNode<T: VoxelTrait> {
    Branch(Box<[Option<OctreeNode<T>>; 8]>),
    Leaf(Voxel<T>),
}

pub struct OctreeIterator<'a, T: VoxelTrait> {
    stack: Vec<&'a OctreeNode<T>>,
}

fn octree_child_index(position: IVec3, depth: u8, max_depth: u8) -> usize {
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

pub fn octree_calculate_voxels_per_axis(lod_level: usize) -> usize {
    1 << lod_level
}

fn octree_get_at_depth<T: VoxelTrait>(
    node: &OctreeNode<T>,
    position: IVec3,
    depth: u8,
    max_depth: u8,
) -> Option<&Voxel<T>> {
    match node {
        OctreeNode::Leaf(voxel) => Some(voxel),
        OctreeNode::Branch(children) => {
            let index = octree_child_index(position, depth, max_depth);
            children[index]
                .as_ref()
                .and_then(|child| octree_get_at_depth(child, position, depth + 1, max_depth))
        }
    }
}

fn octree_set_at_depth<T: VoxelTrait>(
    node: &mut OctreeNode<T>,
    position: IVec3,
    depth: u8,
    max_depth: u8,
    voxel: Voxel<T>,
) -> bool {
    if depth == max_depth {
        *node = OctreeNode::Leaf(voxel);
        voxel.value == T::default()
    } else {
        match node {
            OctreeNode::Leaf(existing_voxel) => {
                // If the existing leaf has the same value, no action is needed
                if *existing_voxel == voxel {
                    return false; // Do not remove this node
                }

                // Split the leaf into a branch
                let mut branch = OctreeNode::new_branch();

                // Initialize all children with the existing voxel value
                for i in 0..8 {
                    octree_set_child(&mut branch, i, OctreeNode::Leaf(*existing_voxel));
                }

                // Replace self with the new branch
                *node = branch;

                // Now, insert the new voxel into the tree
                let should_remove = octree_set_at_depth(node, position, depth, max_depth, voxel);

                // After insertion, check if we can merge this branch into a Leaf
                if let Some(merged_voxel) = octree_try_merge_children_into_leaf(node) {
                    *node = OctreeNode::Leaf(merged_voxel);
                    return merged_voxel.value == T::default(); // Indicate if this node should be removed
                }

                should_remove
            }
            OctreeNode::Branch(children) => {
                let index = octree_child_index(position, depth, max_depth);

                if let Some(child) = &mut children[index] {
                    let should_remove =
                        octree_set_at_depth(child, position, depth + 1, max_depth, voxel);
                    if should_remove {
                        children[index] = None;
                    }
                } else if voxel.value != T::default() {
                    // Create a new child node
                    let mut child = OctreeNode::new_branch();
                    let should_remove =
                        octree_set_at_depth(&mut child, position, depth + 1, max_depth, voxel);
                    if !should_remove {
                        children[index] = Some(child);
                    }
                }

                // After insertion, check if we can merge this branch into a Leaf
                if let Some(merged_voxel) = octree_try_merge_children_into_leaf(node) {
                    *node = OctreeNode::Leaf(merged_voxel);
                    return merged_voxel.value == T::default(); // Indicate if this node should be removed
                }

                if let OctreeNode::Branch(children) = node {
                    // If all children are None, remove this node
                    if children.iter().all(|child| child.is_none()) {
                        return true;
                    }
                }

                false
            }
        }
    }
}

fn octree_set_child<T: VoxelTrait>(node: &mut OctreeNode<T>, index: usize, child: OctreeNode<T>) {
    if let OctreeNode::Branch(children) = node {
        children[index] = Some(child);
    }
}

fn octree_try_merge_children_into_leaf<T: VoxelTrait>(node: &OctreeNode<T>) -> Option<Voxel<T>> {
    if let OctreeNode::Branch(children) = node {
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
                Some(OctreeNode::Leaf(voxel)) if voxel.value == T::default() => return None,
                _ => return None, // Cannot merge if any child is not a matching Leaf node
            }
        }

        return Some(first_voxel);
    }

    None
}

fn octree_is_node_empty<T: VoxelTrait>(node: &OctreeNode<T>) -> bool {
    match node {
        OctreeNode::Leaf(_) => false,
        OctreeNode::Branch(children) => children.iter().all(|child| {
            child
                .as_ref()
                .map_or(true, |child| octree_is_node_empty(child))
        }),
    }
}

fn octree_is_node_full<T: VoxelTrait>(node: &OctreeNode<T>) -> bool {
    match node {
        OctreeNode::Leaf(_) => true,
        OctreeNode::Branch(children) => children.iter().all(|child| {
            child
                .as_ref()
                .map_or(false, |child| octree_is_node_full(child))
        }),
    }
}

fn octree_node_memory_size<T: VoxelTrait>(node: &OctreeNode<T>) -> usize {
    match node {
        OctreeNode::Leaf(_) => std::mem::size_of::<OctreeNode<T>>(),
        OctreeNode::Branch(children) => {
            let children_size: usize = children
                .iter()
                .filter_map(|child| child.as_ref())
                .map(|child| octree_node_memory_size(child))
                .sum();
            std::mem::size_of::<OctreeNode<T>>() + children_size
        }
    }
}

impl<T: VoxelTrait> Octree<T> {
    pub fn new(max_depth: u8) -> Self {
        Self {
            max_depth,
            root: None,
            _phantom: PhantomData,
        }
    }

    pub fn set(&mut self, position: IVec3, voxel: Voxel<T>) {
        if self.root.is_none() {
            if voxel.value == T::default() {
                return;
            }

            self.root = Some(Box::new(OctreeNode::new_branch()));
        }

        if let Some(root) = &mut self.root {
            let should_remove = octree_set_at_depth(root, position, 0, self.max_depth, voxel);
            if should_remove {
                self.root = None;
            }
        }
    }

    pub fn get(&self, position: IVec3) -> Option<&Voxel<T>> {
        self.root
            .as_ref()
            .and_then(|root| octree_get_at_depth(root, position, 0, self.max_depth))
    }

    pub fn is_empty(&self) -> bool {
        self.root.is_none()
    }

    pub fn is_full(&self) -> bool {
        if let Some(root) = &self.root {
            return match root.as_ref() {
                OctreeNode::Leaf(_) => true,
                OctreeNode::Branch(_) => octree_is_node_full(root),
            };
        }

        false
    }

    pub fn clear(&mut self) {
        self.root = None;
    }

    pub fn iter(&self) -> OctreeIterator<'_, T> {
        OctreeIterator::new(self.root.as_deref())
    }

    pub fn to_vec(&self) -> Vec<T> {
        let voxels_per_axis = octree_calculate_voxels_per_axis(self.max_depth as usize);
        let shift_y: usize = 1 << (2 * self.max_depth as usize);
        let shift_z: usize = 1 << self.max_depth as usize;
        let size: usize = voxels_per_axis * voxels_per_axis * voxels_per_axis;

        if let Some(root) = &self.root {
            match root.as_ref() {
                OctreeNode::Leaf(voxel) => {
                    return vec![voxel.value; size];
                }
                OctreeNode::Branch(_) => {
                    let mut data = vec![T::default(); size];
                    for y in 0..voxels_per_axis {
                        let base_index_y = y * shift_y;
                        for z in 0..voxels_per_axis {
                            let base_index_z = base_index_y + z * shift_z;
                            for x in 0..voxels_per_axis {
                                let index = base_index_z + x;
                                if let Some(voxel) = octree_get_at_depth(
                                    root,
                                    IVec3::new(x as i32, y as i32, z as i32),
                                    0,
                                    self.max_depth,
                                ) {
                                    data[index] = voxel.value;
                                }
                            }
                        }
                    }

                    return data;
                }
            }
        }

        vec![T::default(); size]
    }

    pub fn for_each_mut<F>(&mut self, mut f: F)
    where
        F: FnMut(usize, &mut T),
    {
        let voxels_per_axis = octree_calculate_voxels_per_axis(self.max_depth as usize);

        for y in 0..voxels_per_axis {
            for z in 0..voxels_per_axis {
                for x in 0..voxels_per_axis {
                    let index = y * voxels_per_axis * voxels_per_axis + z * voxels_per_axis + x;
                    let position = IVec3::new(x as i32, y as i32, z as i32);

                    // Get the current value at the position or default if not present
                    let mut value = self.get(position).map_or(T::default(), |voxel| voxel.value);

                    // Pass the index and mutable reference to the value to the closure
                    f(index, &mut value);

                    // Update the octree with the new value
                    if value != T::default() {
                        self.set(position, Voxel { value });
                    } else {
                        // Remove the voxel by inserting the default value
                        self.set(
                            position,
                            Voxel {
                                value: T::default(),
                            },
                        );
                    }
                }
            }
        }
    }

    pub fn total_memory_size(&self) -> usize {
        self.root
            .as_ref()
            .map_or(0, |root| octree_node_memory_size(root))
            + std::mem::size_of::<Self>()
    }
}

impl<T: VoxelTrait> OctreeNode<T> {
    fn new_branch() -> Self {
        OctreeNode::Branch(Box::new([None, None, None, None, None, None, None, None]))
    }
}

impl<'a, T: VoxelTrait> OctreeIterator<'a, T> {
    pub fn new(root: Option<&'a OctreeNode<T>>) -> Self {
        let stack = root.into_iter().collect();
        Self { stack }
    }
}

impl<'a, T: VoxelTrait> Iterator for OctreeIterator<'a, T> {
    type Item = &'a Voxel<T>;

    fn next(&mut self) -> Option<Self::Item> {
        while let Some(node) = self.stack.pop() {
            match node {
                OctreeNode::Leaf(voxel) => return Some(voxel),
                OctreeNode::Branch(children) => {
                    for child in children.iter().rev().filter_map(|child| child.as_ref()) {
                        self.stack.push(child);
                    }
                }
            }
        }

        None
    }
}

#[cfg(test)]
mod tests {
    use glam::IVec3;

    use crate::{
        VoxelTrait,
        spatial::octree::svo::{OctreeNode, octree_calculate_voxels_per_axis},
    };

    use super::{Octree, Voxel};

    #[test]
    fn test_create() {
        let octree = Octree::<u8>::new(3);
        assert_eq!(octree.max_depth, 3);
        assert!(octree.root.is_none());
    }

    #[test]
    fn test_set_and_get() {
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
            octree.set(
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

        octree.set(IVec3::new(0, 0, 0), Voxel { value: 1 });
        assert!(!octree.is_empty());
        assert!(!octree.is_full());

        for x in 0..8 {
            for y in 0..8 {
                for z in 0..8 {
                    octree.set(IVec3::new(x, y, z), Voxel { value: 1 });
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
            octree.set(
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

    #[test]
    fn test_no_default_leaf_nodes() {
        let mut octree = Octree::<u8>::new(3);

        // Insert some voxels with non-default values
        octree.set(IVec3::new(0, 0, 0), Voxel { value: 1 });
        octree.set(IVec3::new(1, 1, 1), Voxel { value: 2 });

        // Set a voxel to the default value, which should remove the node
        octree.set(IVec3::new(0, 0, 0), Voxel { value: 0 });

        // Traverse the tree to ensure no Leaf nodes have default value
        fn check_no_default_leaf<T: VoxelTrait>(node: &OctreeNode<T>) {
            match node {
                OctreeNode::Leaf(voxel) => {
                    assert!(
                        voxel.value != T::default(),
                        "Found a Leaf node with default value!"
                    );
                }
                OctreeNode::Branch(children) => {
                    for child in children.iter().flatten() {
                        check_no_default_leaf(child);
                    }
                }
            }
        }

        if let Some(root) = &octree.root {
            check_no_default_leaf(root);
        }
    }

    #[test]
    fn test_total_memory_size() {
        let mut octree = Octree::<u8>::new(3);
        assert_eq!(
            octree.total_memory_size(),
            std::mem::size_of::<Octree<u8>>()
        );

        octree.set(IVec3::new(0, 0, 0), Voxel { value: 1 });
        let size_with_one_voxel = octree.total_memory_size();
        assert!(size_with_one_voxel > std::mem::size_of::<Octree<u8>>());
        assert_eq!(size_with_one_voxel, 80);

        octree.clear();
        assert_eq!(
            octree.total_memory_size(),
            std::mem::size_of::<Octree<u8>>()
        );
    }

    #[test]
    fn test_iterator() {
        let mut octree = Octree::<u8>::new(3);

        octree.set(IVec3::new(0, 0, 0), Voxel { value: 1 });
        octree.set(IVec3::new(1, 1, 1), Voxel { value: 2 });

        let voxels: Vec<&Voxel<u8>> = octree.iter().collect();
        assert_eq!(voxels.len(), 2);
        assert_eq!(voxels[0].value, 1);
        assert_eq!(voxels[1].value, 2);
    }

    #[test]
    fn test_fill() {
        let max_depth = 2;
        let mut octree = Octree::<u8>::new(max_depth);
        assert!(!octree.is_full());

        let voxels_per_axis = octree_calculate_voxels_per_axis(max_depth as usize) as i32;

        for y in 0..voxels_per_axis {
            for z in 0..voxels_per_axis {
                for x in 0..voxels_per_axis {
                    octree.set(IVec3::new(x, y, z), Voxel { value: 1 });
                }
            }
        }

        assert!(octree.is_full());

        octree.set(IVec3::new(0, 0, 0), Voxel { value: 0 });

        assert!(!octree.is_full());
    }
}
