use glam::IVec3;

use crate::{BlockId, MaxDepth, TraversalDepth, VoxInterner, VoxelTrait};

#[inline(always)]
pub const fn child_index(position: &IVec3, depth: &TraversalDepth) -> usize {
    let shift = depth.max() - depth.current() - 1;

    ((position.x as usize >> shift) & 1)
        | (((position.y as usize >> shift) & 1) << 1)
        | (((position.z as usize >> shift) & 1) << 2)
}

#[inline(always)]
pub const fn child_index2(position: &IVec3, current: usize, max: usize) -> usize {
    let shift = max - current - 1;

    ((position.x as usize >> shift) & 1)
        | (((position.y as usize >> shift) & 1) << 1)
        | (((position.z as usize >> shift) & 1) << 2)
}

#[inline(always)]
pub const fn encode_child_index_path(position: &IVec3) -> u32 {
    const MASK_10_BITS: u32 = 0x000003FF; // Mask for lower 10 bits
    const MASK_1: u32 = 0x30000FF;
    const MASK_2: u32 = 0x300F00F;
    const MASK_3: u32 = 0x30C30C3;
    const MASK_4: u32 = 0x9249249;

    let x = (position.x as u32) & MASK_10_BITS;
    let y = (position.y as u32) & MASK_10_BITS;
    let z = (position.z as u32) & MASK_10_BITS;

    // Split x bits using magic shifts and masks
    let x = (x | (x << 16)) & MASK_1;
    let x = (x | (x << 8)) & MASK_2;
    let x = (x | (x << 4)) & MASK_3;
    let x = (x | (x << 2)) & MASK_4;

    // Split y bits
    let y = (y | (y << 16)) & MASK_1;
    let y = (y | (y << 8)) & MASK_2;
    let y = (y | (y << 4)) & MASK_3;
    let y = (y | (y << 2)) & MASK_4;

    // Split z bits
    let z = (z | (z << 16)) & MASK_1;
    let z = (z | (z << 8)) & MASK_2;
    let z = (z | (z << 4)) & MASK_3;
    let z = (z | (z << 2)) & MASK_4;

    // Combine results - x at positions 3n, y at 3n+1, z at 3n+2
    x | (y << 1) | (z << 2)
}

#[macro_export]
macro_rules! encode_child_index_path_macro {
    ($position:expr) => {{
        const MASK_10_BITS: u32 = 0x000003FF; // Mask for lower 10 bits
        const MASK_1: u32 = 0x30000FF;
        const MASK_2: u32 = 0x300F00F;
        const MASK_3: u32 = 0x30C30C3;
        const MASK_4: u32 = 0x9249249;

        let x = ($position.x as u32) & MASK_10_BITS;
        let y = ($position.y as u32) & MASK_10_BITS;
        let z = ($position.z as u32) & MASK_10_BITS;

        // Split x bits using magic shifts and masks
        let x = (x | (x << 16)) & MASK_1;
        let x = (x | (x << 8)) & MASK_2;
        let x = (x | (x << 4)) & MASK_3;
        let x = (x | (x << 2)) & MASK_4;

        // Split y bits
        let y = (y | (y << 16)) & MASK_1;
        let y = (y | (y << 8)) & MASK_2;
        let y = (y | (y << 4)) & MASK_3;
        let y = (y | (y << 2)) & MASK_4;

        // Split z bits
        let z = (z | (z << 16)) & MASK_1;
        let z = (z | (z << 8)) & MASK_2;
        let z = (z | (z << 4)) & MASK_3;
        let z = (z | (z << 2)) & MASK_4;

        // Combine results - x at positions 3n, y at 3n+1, z at 3n+2
        x | (y << 1) | (z << 2)
    }};
}

#[macro_export]
macro_rules! child_index_macro {
    ($position:expr, $depth:expr) => {{
        let shift = ($depth.max() - $depth.current() - 1) as usize;
        (((($position).x as usize >> shift) & 1)
            | (((($position).y as usize >> shift) & 1) << 1)
            | (((($position).z as usize >> shift) & 1) << 2))
    }};
}

#[macro_export]
macro_rules! child_index_macro_2 {
    ($position:expr, $current_depth:expr, $max_depth:expr) => {{
        let shift = ($max_depth - $current_depth - 1);
        (((($position).x as usize >> shift) & 1)
            | (((($position).y as usize >> shift) & 1) << 1)
            | (((($position).z as usize >> shift) & 1) << 2))
    }};
}

#[macro_export]
macro_rules! child_index_macro_2d {
    ($position:expr, $current_depth:expr, $max_depth:expr) => {{
        let shift = ($max_depth - $current_depth - 1);
        (((($position).x as usize >> shift) & 1) | (((($position).y as usize >> shift) & 1) << 1))
    }};
}

#[inline(always)]
pub fn get_at_depth<T: VoxelTrait>(
    interner: &VoxInterner<T>,
    mut node_id: BlockId,
    position: &IVec3,
    depth: &TraversalDepth,
) -> Option<T> {
    let max_depth = depth.max();
    let mut depth = depth.current();

    while !node_id.is_empty() {
        if depth >= max_depth {
            return Some(*interner.get_value(&node_id));
        }

        if node_id.is_branch() {
            let index = child_index_macro_2!(position, depth, max_depth);
            node_id = interner.get_child_id(&node_id, index);
            depth += 1;
        } else {
            return Some(*interner.get_value(&node_id));
        }
    }

    None
}

pub fn to_vec<T: VoxelTrait>(
    interner: &VoxInterner<T>,
    root_id: &BlockId,
    max_depth: MaxDepth,
) -> Vec<T> {
    let max_depth = max_depth.max();
    let voxels_per_axis = 1 << max_depth;
    let size: usize = voxels_per_axis * voxels_per_axis * voxels_per_axis;

    if !root_id.is_empty() {
        if root_id.is_branch() {
            let mut data = vec![T::default(); size];
            let shift_y: usize = 1 << (2 * max_depth);
            let shift_z: usize = voxels_per_axis;
            let depth = TraversalDepth::new(0, max_depth);
            let mut pos = IVec3::default();

            for y in 0..voxels_per_axis {
                pos.y = y as i32;
                let base_index_y = y * shift_y;
                for z in 0..voxels_per_axis {
                    pos.z = z as i32;

                    let base_index_z = base_index_y + z * shift_z;
                    for x in 0..voxels_per_axis {
                        pos.x = x as i32;

                        if let Some(voxel) = get_at_depth(interner, *root_id, &pos, &depth) {
                            data[base_index_z + x] = voxel;
                        }
                    }
                }
            }

            data
        } else {
            vec![*interner.get_value(root_id); size]
        }
    } else {
        vec![T::default(); size]
    }
}

pub fn dump_structure<T: VoxelTrait>(
    interner: &VoxInterner<T>,
    root_id: BlockId,
    max_depth: usize,
) {
    println!("\n=== Octree Structure Dump ===");
    println!("Max depth: {max_depth}");

    if !root_id.is_empty() {
        interner.dump_node(root_id, 0, "  ");
    } else {
        println!("Empty octree (no root)");
    }
    println!("=== End of Structure Dump ===\n");
}

pub fn dump_root<T: VoxelTrait>(interner: &VoxInterner<T>, root_id: BlockId) {
    println!("\n=== Octree Root Dump ===");
    if !root_id.is_empty() {
        interner.dump_node(root_id, 0, "");
    } else {
        println!("Empty octree (no root)");
    }
    println!("=== End of Root Dump ===\n");
}

#[derive(Default)]
struct OctreeStats {
    total_nodes: usize,
    branch_nodes: usize,
    leaf_nodes: usize,
    max_depth_reached: u8,
    nodes_by_depth: Vec<usize>,
}

pub fn dump_statistics<T: VoxelTrait>(interner: &VoxInterner<T>, root_id: BlockId) {
    println!("\n=== Octree Statistics ===");
    if !root_id.is_empty() {
        let mut stats = OctreeStats::default();
        collect_stats(interner, root_id, 0, &mut stats);
        println!("Total nodes: {}", stats.total_nodes);
        println!("Branch nodes: {}", stats.branch_nodes);
        println!("Leaf nodes: {}", stats.leaf_nodes);
        println!("Max depth reached: {}", stats.max_depth_reached);
        println!("Nodes by depth:");
        for (depth, count) in stats.nodes_by_depth.iter().enumerate() {
            println!("  Depth {depth}: {count} nodes");
        }
    } else {
        println!("Empty octree (no statistics available)");
    }
    println!("=== End of Statistics ===\n");
}

fn collect_stats<T: VoxelTrait>(
    interner: &VoxInterner<T>,
    node_id: BlockId,
    depth: u8,
    stats: &mut OctreeStats,
) {
    stats.total_nodes += 1;

    // Ensure we have enough space in nodes_by_depth
    while stats.nodes_by_depth.len() <= depth as usize {
        stats.nodes_by_depth.push(0);
    }
    stats.nodes_by_depth[depth as usize] += 1;

    // Update max depth
    stats.max_depth_reached = stats.max_depth_reached.max(depth);

    match node_id.is_leaf() {
        true => {
            stats.leaf_nodes += 1;
        }
        false => {
            stats.branch_nodes += 1;
            let children = interner.get_children(&node_id);
            for child in children.iter() {
                if !child.is_empty() {
                    collect_stats(interner, *child, depth + 1, stats);
                }
            }
        }
    }
}
