use std::{
    io::{BufReader, Read, Write},
    sync::Arc,
};

use byteorder::{BigEndian, ReadBytesExt, WriteBytesExt};
use glam::IVec3;
use parking_lot::RwLock;

use rayon::prelude::*;
use rustc_hash::FxHashMap;

#[cfg(feature = "memory_stats")]
use crate::storage::node::StoreStats;
use crate::{
    BlockId, MaxDepth, NodeStore,
    io::varint::{decode_varint_u32_from_reader, encode_varint_u32},
    storage::node::EMPTY_CHILD,
    world::Chunk,
};

pub struct Model {
    pub max_depth: MaxDepth,
    pub voxels_per_axis: usize,
    pub chunk_size: f32,
    pub chunks_size: IVec3,
    pub chunks_len: usize,
    pub chunks: Vec<Chunk>,
    pub store: Arc<RwLock<NodeStore<i32>>>,
}

impl Model {
    pub fn new(max_depth: MaxDepth, chunk_size: f32) -> Self {
        let store = Arc::new(RwLock::new(NodeStore::with_memory_budget(
            1024 * 1024 * 256,
        )));
        let chunks_size = IVec3::new(32, 32, 32);
        let chunks_len = chunks_size.x as usize * chunks_size.y as usize * chunks_size.z as usize;
        let chunks = Self::init_chunks(max_depth, chunk_size, chunks_size, chunks_len);
        let voxels_per_axis = 1 << max_depth.max();

        Self {
            max_depth,
            voxels_per_axis,
            chunk_size,
            chunks_size,
            chunks_len,
            chunks,
            store,
        }
    }

    pub fn with_size(max_depth: MaxDepth, chunk_size: f32, chunks_size: IVec3) -> Self {
        println!(
            "Creating model with size {:?}, chunk: {} depth: {}",
            chunks_size, chunk_size, max_depth
        );
        let store = Arc::new(RwLock::new(NodeStore::with_memory_budget(
            1024 * 1024 * 1024,
        )));
        let chunks_len = chunks_size.x as usize * chunks_size.y as usize * chunks_size.z as usize;
        let chunks = Self::init_chunks(max_depth, chunk_size, chunks_size, chunks_len);
        let voxels_per_axis = 1 << max_depth.max();

        Self {
            max_depth,
            voxels_per_axis,
            chunk_size,
            chunks_size,
            chunks_len,
            chunks,
            store,
        }
    }

    pub fn get_store(&self) -> Arc<RwLock<NodeStore<i32>>> {
        self.store.clone()
    }

    pub fn clear(&mut self) {
        self.chunks_size = IVec3::ZERO;
        self.chunks_len = 0;
        self.chunks.clear();
    }

    pub fn resize(&mut self, size: IVec3) {
        self.chunks.clear();

        self.chunks_size = size;
        self.chunks_len = size.x as usize * size.y as usize * size.z as usize;
        self.chunks = Self::init_chunks(
            self.max_depth,
            self.chunk_size,
            self.chunks_size,
            self.chunks_len,
        );
    }

    fn init_chunks(max_depth: MaxDepth, chunk_size: f32, size: IVec3, len: usize) -> Vec<Chunk> {
        let mut chunks = Vec::with_capacity(len);

        for y in 0..size.y {
            for z in 0..size.z {
                for x in 0..size.x {
                    chunks.push(Chunk::with_position(chunk_size, max_depth, x, y, z));
                }
            }
        }

        chunks
    }

    pub fn serialize(&self, data: &mut Vec<u8>) {
        const BUFFER_SIZE: usize = 256;

        let storage = self.store.read();

        let leaf_patterns = storage.leaf_patterns();
        let branch_patterns = storage.branch_patterns();

        let mut id_map: FxHashMap<u32, u32> = FxHashMap::default();
        id_map.insert(0, 0);

        let mut leaf_patterns = leaf_patterns.iter().map(|(_, id)| *id).collect::<Vec<_>>();
        let mut branch_patterns = branch_patterns
            .iter()
            .map(|(_, id)| *id)
            .collect::<Vec<_>>();

        let mut next_id = 1;

        leaf_patterns.sort_by_key(|id| id.index());
        branch_patterns.sort_by_key(|id| id.index());

        leaf_patterns.iter().for_each(|id| {
            id_map.insert(id.index(), next_id);
            next_id += 1;
        });

        branch_patterns.iter().for_each(|id| {
            let index = id.index();
            if index == 0 {
                return;
            }

            id_map.insert(id.index(), next_id);
            next_id += 1;
        });

        let leaf_size = leaf_patterns.len();
        assert!(leaf_size <= u32::MAX as usize);
        let branch_size = branch_patterns.len();
        assert!(branch_size <= u32::MAX as usize);
        let size = leaf_size + branch_size;
        assert!(size <= u32::MAX as usize);

        let leaf_size = leaf_size as u32;
        let branch_size = branch_size as u32;

        let mut writer = std::io::BufWriter::new(data);

        writer.write_u32::<BigEndian>(leaf_size).unwrap();
        for id in leaf_patterns.iter() {
            let new_id = *id_map.get(&id.index()).unwrap();
            // println!(" leaf id: {} -> {}", id.index(), new_id);
            let new_id_bytes = encode_varint_u32(new_id);
            // writer.write_u32::<BigEndian>(new_id).unwrap();
            writer.write_all(&new_id_bytes).unwrap();
            let value = storage.get_value(id);
            writer.write_all(&value.to_be_bytes()).unwrap();
        }

        writer.write_u32::<BigEndian>(branch_size - 1).unwrap();
        for id in branch_patterns.iter() {
            if id.index() == 0 {
                continue;
            }

            let new_id = *id_map.get(&id.index()).unwrap();
            // println!("branch id: {} -> {}", id.index(), new_id);
            let new_id_bytes = encode_varint_u32(new_id);
            // writer.write_u32::<BigEndian>(new_id).unwrap();
            writer.write_all(&new_id_bytes).unwrap();
            writer.write_u8(id.mask()).unwrap();
            let branch = storage.get_children_ref(id);
            for child in branch.iter() {
                if child.is_empty() {
                    // println!(" empty child");
                    continue;
                }
                let new_id = *id_map.get(&child.index()).unwrap();
                // println!("  child id: {} -> {}", child.index(), new_id);
                let new_id_bytes = encode_varint_u32(new_id);
                // writer.write_u32::<BigEndian>(new_id).unwrap();
                writer.write_all(&new_id_bytes).unwrap();
            }
        }

        let chunks_data: Vec<Vec<u8>> = self
            .chunks
            .par_iter()
            .map(|chunk| {
                let mut buffer = Vec::with_capacity(BUFFER_SIZE);
                chunk.serialize(&id_map, &mut buffer);
                buffer
            })
            .collect();

        for chunk_data in chunks_data.iter() {
            writer.write_all(chunk_data).unwrap();
        }
    }

    pub fn deserialize(&mut self, data: &[u8]) {
        println!("Deserializing chunks...");

        let now = std::time::Instant::now();

        let mut reader = BufReader::new(data);

        let leaf_size = reader.read_u32::<BigEndian>().unwrap();
        // let mut leaf_patterns: HashMap<u32, (BlockId, i32)> =
        //     HashMap<K, V, FxBuildHasher>(leaf_size as usize);
        let mut leaf_patterns: FxHashMap<u32, (BlockId, i32)> = FxHashMap::default();

        let mut storage = self.store.write();

        for _ in 0..leaf_size {
            let id = decode_varint_u32_from_reader(&mut reader).unwrap();
            let mut bytes = [0u8; std::mem::size_of::<i32>()];
            reader.read_exact(&mut bytes).unwrap();
            let value = i32::from_be_bytes(bytes);

            let block_id = storage.deserialize_leaf(id, value);
            leaf_patterns.insert(id, (block_id, value));

            println!(" leaf id: {:?} -> {}", block_id, value);
        }

        let branch_size = reader.read_u32::<BigEndian>().unwrap();
        let mut branch_patterns: FxHashMap<u32, (BlockId, [u32; 8])> =
        // FxHashMap::with_capacity(branch_size as usize);
            FxHashMap::default();

        branch_patterns.insert(0, (BlockId::EMPTY, [0u32; 8]));

        for _ in 0..branch_size {
            let id = decode_varint_u32_from_reader(&mut reader).unwrap();
            assert_ne!(id, 0);

            let mask = reader.read_u8().unwrap();
            // println!("id: {} mask: {:08b}", id, mask);
            let mut types: u8 = 0;
            let mut children = [0u32; 8];
            for child_id in 0..8 {
                if mask & (1 << child_id) == 0 {
                    // println!(" skipping child {}", child_id);
                    continue;
                }
                // println!(" reading child {}", child_id);
                children[child_id] = decode_varint_u32_from_reader(&mut reader).unwrap();
                if leaf_patterns.contains_key(&children[child_id]) {
                    types |= 1 << child_id;
                }
            }

            let block_id = storage.preallocate_branch_id(id, types, mask);

            branch_patterns.insert(id, (block_id, children));
            // println!(
            //     " branch: mask: {:08b} types: {:08b} id: {:08X} [{:?}] -> {:08X?}",
            //     mask, types, id, block_id, children
            // );
            assert_ne!(mask, 0);
        }

        branch_patterns
            .iter()
            .for_each(|(id, (block_id, children))| {
                if *id == 0 {
                    return;
                }

                let types = block_id.types();
                let mask = block_id.mask();

                let mut branch = EMPTY_CHILD;
                for child_idx in 0..8 {
                    if mask & (1 << child_idx) == 0 {
                        continue;
                    }

                    let child_id = children[child_idx];
                    if types & (1 << child_idx) != 0 {
                        let (leaf_id, _) = leaf_patterns.get(&child_id).unwrap();
                        branch[child_idx] = *leaf_id;
                    } else {
                        branch[child_idx] = branch_patterns.get(&child_id).unwrap().0;
                    }
                }

                // println!("branch: {:?} -> {:?}", block_id, branch);
                storage.deserialize_branch(*block_id, branch, types, mask);
            });

        // drop(storage);

        let mut branch_ids = branch_patterns
            .iter()
            .map(|(_, (block_id, _))| *block_id)
            .collect::<Vec<_>>();
        branch_ids.sort_by_key(|id| id.index());

        // for branch_id in branch_ids.iter() {
        //     println!("Branch id: {:?}", branch_id);
        //     storage.dump_node(*branch_id, 0, "  ");
        // }

        self.chunks.iter_mut().for_each(|chunk| {
            chunk.deserialize(&mut storage, &leaf_patterns, &branch_patterns, &mut reader);
        });

        let elapsed = now.elapsed();
        println!("Deserializing chunks took {:?}", elapsed);
    }

    pub fn max_depth(&self) -> MaxDepth {
        self.max_depth
    }

    pub fn voxels_per_axis(&self) -> usize {
        self.voxels_per_axis
    }

    #[cfg(feature = "memory_stats")]
    pub fn storage_stats(&self) -> StoreStats {
        self.store.read().stats()
    }

    // #[cfg(feature = "memory_stats")]
    // pub fn allocator_stats(&self) -> StorageStats {
    //     if let Some(shared_node_cache) = self.shared_node_cache.as_ref() {
    //         shared_node_cache.as_ref().read().stats().clone()
    //     } else {
    //         StorageStats::default()
    //     }
    // }

    // pub fn storage_stats(&self) -> StorageStats {
    //     if let Some(shared_node_cache) = self.shared_node_cache.as_ref() {
    //         shared_node_cache.storage_stats()
    //     } else {
    //         StorageStats::default()
    //     }
    // }

    // pub fn cache_statistics(&self) -> CacheStatistics {
    //     CacheStatistics::default()
    //     // if let Some(shared_node_cache) = self.shared_node_cache.as_ref() {
    //     //     get_cache_statistics(shared_node_cache)
    //     // } else {
    //     //     self.chunks
    //     //         .par_iter()
    //     //         .map(|chunk| chunk.cache_statistics())
    //     //         .sum()
    //     // }
    // }
}
