use std::{io::Read, path::Path};

use byteorder::{BigEndian, ReadBytesExt};
use glam::IVec3;
use md5::{Digest, Md5};

use crate::io::consts::{VTM_MAGIC, VTM_VERSION};
use crate::io::Flags;
use crate::model::Model;
use crate::world::MAX_LOD_LEVEL;

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

    let lod_level = reader.read_u8().unwrap();
    assert_eq!(lod_level, MAX_LOD_LEVEL as u8);

    let _reserved_1 = reader.read_u32::<BigEndian>().unwrap();
    let _reserved_2 = reader.read_u32::<BigEndian>().unwrap();

    let size_x = reader.read_u16::<BigEndian>().unwrap();
    let size_y = reader.read_u16::<BigEndian>().unwrap();
    let size_z = reader.read_u16::<BigEndian>().unwrap();
    let size = IVec3::new(size_x as i32, size_y as i32, size_z as i32);

    let name_len = reader.read_u8().unwrap();
    let mut name = vec![0u8; name_len as usize];
    reader.read_exact(&mut name).unwrap();

    let mut md5_hash = [0u8; 16];
    reader.read_exact(&mut md5_hash).unwrap();

    let sizes_len = reader.read_u32::<BigEndian>().unwrap();
    let mut sizes = vec![0u8; sizes_len as usize];
    reader.read_exact(&mut sizes).unwrap();

    let data_size = reader.read_u32::<BigEndian>().unwrap();
    let mut data = vec![0u8; data_size as usize];
    reader.read_exact(&mut data).unwrap();

    let (sizes, data) = if flags.contains(Flags::COMPRESSED) {
        let mut decoder = zstd::stream::Decoder::new(&sizes[..]).unwrap();
        let mut sizes = Vec::new();
        std::io::copy(&mut decoder, &mut sizes).unwrap();

        let mut decoder = zstd::stream::Decoder::new(&data[..]).unwrap();
        let mut data = Vec::new();
        std::io::copy(&mut decoder, &mut data).unwrap();

        (sizes, data)
    } else {
        (sizes, data)
    };

    let mut sizes_data = Vec::new();

    let mut sizes_reader = std::io::BufReader::new(sizes.as_slice());

    for _ in 0..sizes.len() / 4 {
        let size = sizes_reader.read_u32::<BigEndian>().unwrap();
        sizes_data.push(size);
    }

    let mut offsets = Vec::new();
    let mut offset = 0;
    for chunk_size in sizes_data.iter() {
        offsets.push(offset);
        offset += *chunk_size as usize;
    }

    let mut md5_hasher = Md5::new();
    md5_hasher.update(&data);
    let md5_hash_calculated = md5_hasher.finalize();

    assert_eq!(md5_hash, md5_hash_calculated.as_slice());

    let mut model = Model::with_size(size);
    model.deserialize(&data, &offsets);

    model
}
