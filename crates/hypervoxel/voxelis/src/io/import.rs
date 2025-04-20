use std::{io::Read, path::Path};

use byteorder::{BigEndian, ReadBytesExt};
use glam::IVec3;
use md5::{Digest, Md5};

use crate::{MaxDepth, model::Model};

use super::{
    Flags,
    consts::{VTM_MAGIC, VTM_VERSION},
};

pub fn import_model_from_vtm<P: AsRef<Path>>(path: &P) -> Model {
    let mut vox_file = std::fs::File::open(path).unwrap();
    let mut reader = std::io::BufReader::new(&mut vox_file);

    let mut magic = [0u8; VTM_MAGIC.len()];
    reader.read_exact(&mut magic).unwrap();
    assert_eq!(magic, VTM_MAGIC);

    let version = reader.read_u16::<BigEndian>().unwrap();
    assert_eq!(version, VTM_VERSION);

    let flags = reader.read_u16::<BigEndian>().unwrap();
    let flags = Flags::from_bits(flags).unwrap();
    println!("Flags: {:?}", flags);

    let lod_level = reader.read_u8().unwrap();
    println!("LOD Level: {}", lod_level);

    let chunk_size = reader.read_u32::<BigEndian>().unwrap() as i32;
    println!("Chunk Size: {}cm", chunk_size);

    let _reserved_1 = reader.read_u32::<BigEndian>().unwrap();
    let _reserved_2 = reader.read_u32::<BigEndian>().unwrap();

    let size_x = reader.read_u16::<BigEndian>().unwrap();
    let size_y = reader.read_u16::<BigEndian>().unwrap();
    let size_z = reader.read_u16::<BigEndian>().unwrap();
    let size = IVec3::new(size_x as i32, size_y as i32, size_z as i32);

    println!("Size: {:?}", size);

    let name_len = reader.read_u8().unwrap();
    let mut name = vec![0u8; name_len as usize];
    reader.read_exact(&mut name).unwrap();

    println!("Name: {:?}", std::str::from_utf8(&name).unwrap());

    let mut md5_hash = [0u8; 16];
    reader.read_exact(&mut md5_hash).unwrap();

    println!("MD5 Hash: {:0X?}", md5_hash);

    let data_size = reader.read_u32::<BigEndian>().unwrap();
    let mut data = vec![0u8; data_size as usize];
    reader.read_exact(&mut data).unwrap();

    println!("Data: {:?}", data_size);

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

    println!("MD5 Hash calculated: {:0X?}", md5_hash_calculated);

    let chunk_size = 1.28;

    let mut model = Model::with_size(MaxDepth::new(lod_level), chunk_size, size);
    model.deserialize(&data);

    model
}
