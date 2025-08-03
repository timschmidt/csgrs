use std::{io::Read, path::Path};

use byteorder::{BigEndian, ReadBytesExt};
use glam::IVec3;
use md5::{Digest, Md5};

use crate::{MaxDepth, world::VoxModel};

use super::{
    Flags,
    consts::{VTM_MAGIC, VTM_VERSION},
};

pub fn import_model_from_vtm<P: AsRef<Path>>(
    path: &P,
    memory_budget: usize,
    target_chunk_world_size: Option<f32>,
) -> VoxModel {
    let mut vox_file = std::fs::File::open(path).unwrap();
    let mut reader = std::io::BufReader::new(&mut vox_file);

    let mut magic = [0u8; VTM_MAGIC.len()];
    reader.read_exact(&mut magic).unwrap();
    assert_eq!(magic, VTM_MAGIC);

    let version = reader.read_u16::<BigEndian>().unwrap();
    assert_eq!(version, VTM_VERSION);

    let flags = reader.read_u16::<BigEndian>().unwrap();
    let flags = Flags::from_bits(flags).unwrap();
    println!("Flags: {flags:?}");

    let lod_level = reader.read_u8().unwrap();
    println!("LOD Level: {lod_level}");

    let chunk_world_size = reader.read_f32::<BigEndian>().unwrap();
    println!("Chunk Size: {chunk_world_size}m");
    println!(
        "Voxel Size: {}cm",
        chunk_world_size / (1 << lod_level) as f32 * 100.0
    );

    let _reserved_1 = reader.read_u32::<BigEndian>().unwrap();
    let _reserved_2 = reader.read_u32::<BigEndian>().unwrap();

    let world_bounds_x = reader.read_i32::<BigEndian>().unwrap();
    let world_bounds_y = reader.read_i32::<BigEndian>().unwrap();
    let world_bounds_z = reader.read_i32::<BigEndian>().unwrap();
    let world_bounds = IVec3::new(world_bounds_x, world_bounds_y, world_bounds_z);

    println!("World bounds: {world_bounds:?}");

    let name_len = reader.read_u8().unwrap();
    let mut name = vec![0u8; name_len as usize];
    reader.read_exact(&mut name).unwrap();

    println!("Name: {:?}", std::str::from_utf8(&name).unwrap());

    let mut md5_hash = [0u8; 16];
    reader.read_exact(&mut md5_hash).unwrap();

    println!("MD5 Hash: {md5_hash:0X?}");

    let data_size = reader.read_u32::<BigEndian>().unwrap();
    let mut data = vec![0u8; data_size as usize];
    reader.read_exact(&mut data).unwrap();

    println!("Data: {data_size:?}");

    let data = if flags.contains(Flags::COMPRESSED) {
        let mut decoder = zstd::stream::Decoder::new(&data[..]).unwrap();
        let mut data = Vec::new();
        std::io::copy(&mut decoder, &mut data).unwrap();

        data
    } else {
        data
    };

    let mut md5_hasher = Md5::new();
    md5_hasher.update(&data);
    let md5_hash_calculated = md5_hasher.finalize();

    assert_eq!(md5_hash, md5_hash_calculated.as_slice());

    println!("MD5 Hash calculated: {md5_hash_calculated:0X?}");

    let chunk_world_size = target_chunk_world_size.unwrap_or(chunk_world_size);

    let mut model = VoxModel::empty(MaxDepth::new(lod_level), chunk_world_size, memory_budget);
    model.deserialize(&data);

    model
}
