use std::{
    io::{Read, Write},
    path::PathBuf,
};

use bevy::math::IVec3;
use byteorder::{BigEndian, ReadBytesExt, WriteBytesExt};
use md5::{Digest, Md5};

use crate::{
    chunk::MAX_LOD_LEVEL,
    io::{Flags, RESERVED_1, RESERVED_2, VTM_MAGIC, VTM_VERSION},
    Model,
};

pub fn encode_varint(mut value: usize) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(8);

    while value >= 0x80 {
        // Set the MSB to indicate more bytes follow
        bytes.push((value as u8 & 0x7F) | 0x80);
        value >>= 7;
    }

    // Last byte with MSB unset
    bytes.push(value as u8);

    bytes
}

pub fn decode_varint(iter: &mut std::slice::Iter<u8>) -> Option<usize> {
    let mut result = 0usize;
    let mut shift = 0;

    loop {
        let byte = *iter.next()?;

        result |= ((byte & 0x7F) as usize) << shift;

        if byte & 0x80 == 0 {
            break;
        }

        shift += 7;
    }

    Some(result)
}

pub fn export_model_to_obj(name: String, path: PathBuf, model: &Model) {
    let mut vertices: Vec<bevy::math::Vec3> = Vec::new();
    let mut normals: Vec<bevy::math::Vec3> = Vec::new();
    let mut indices: Vec<u32> = Vec::new();

    for chunk in model.chunks.iter() {
        if chunk.is_empty() {
            continue;
        }

        let offset = chunk.get_position().as_vec3();

        chunk.generate_mesh_arrays(&mut vertices, &mut normals, &mut indices, offset);
    }

    let obj_file = std::fs::File::create(path).unwrap();
    let mut writer = std::io::BufWriter::new(obj_file);

    writer
        .write_all(format!("o {}\n", name).as_bytes())
        .unwrap();

    for vertex in vertices.iter() {
        writer
            .write_fmt(format_args!("v {} {} {}\n", vertex.x, vertex.y, vertex.z))
            .unwrap();
    }

    for normal in normals.iter() {
        writer
            .write_fmt(format_args!("vn {} {} {}\n", normal.x, normal.y, normal.z))
            .unwrap();
    }

    for index in indices.chunks(3) {
        writer
            .write_fmt(format_args!(
                "f {} {} {}\n",
                index[0] + 1,
                index[1] + 1,
                index[2] + 1
            ))
            .unwrap();
    }
}

pub fn export_model_to_vtm(name: String, path: PathBuf, model: &Model) {
    let mut vox_file = std::fs::File::create(path).unwrap();
    let mut writer = std::io::BufWriter::new(&mut vox_file);

    let flags = Flags::DEFAULT;
    // let flags = Flags::NONE;

    writer.write_all(&VTM_MAGIC).unwrap();
    writer.write_u16::<BigEndian>(VTM_VERSION).unwrap();
    writer.write_u16::<BigEndian>(flags.bits()).unwrap();
    writer.write_u8(MAX_LOD_LEVEL as u8).unwrap();
    writer.write_u32::<BigEndian>(RESERVED_1).unwrap();
    writer.write_u32::<BigEndian>(RESERVED_2).unwrap();

    let size = model.chunks_size;
    writer
        .write_u16::<BigEndian>(size.x.try_into().unwrap())
        .unwrap();
    writer
        .write_u16::<BigEndian>(size.y.try_into().unwrap())
        .unwrap();
    writer
        .write_u16::<BigEndian>(size.z.try_into().unwrap())
        .unwrap();

    writer.write_u8(name.len().try_into().unwrap()).unwrap();
    writer.write_all(name.as_bytes()).unwrap();

    let mut data = Vec::new();
    let mut sizes = Vec::new();
    model.serialize(&mut data, &mut sizes);

    let mut md5_hasher = Md5::new();
    md5_hasher.update(&data);
    let md5_hash = md5_hasher.finalize();

    writer.write_all(&md5_hash).unwrap();

    let mut sizes_data = Vec::new();
    for size in sizes.iter() {
        sizes_data.extend(size.to_be_bytes());
    }

    let (sizes, data) = if flags.contains(Flags::COMPRESSED) {
        let mut encoder = zstd::stream::Encoder::new(Vec::new(), 7).unwrap();
        std::io::copy(&mut sizes_data.as_slice(), &mut encoder).unwrap();
        let sizes_data = encoder.finish().unwrap();

        let mut encoder = zstd::stream::Encoder::new(Vec::new(), 7).unwrap();
        std::io::copy(&mut data.as_slice(), &mut encoder).unwrap();
        let data = encoder.finish().unwrap();

        (sizes_data, data)
    } else {
        (sizes_data, data)
    };

    writer
        .write_u32::<BigEndian>(sizes.len().try_into().unwrap())
        .unwrap();
    writer.write_all(&sizes).unwrap();

    writer
        .write_u32::<BigEndian>(data.len().try_into().unwrap())
        .unwrap();
    writer.write_all(&data).unwrap();
}

pub fn import_model_from_vtm(path: PathBuf) -> Model {
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

    for _ in 0..sizes.len() / 2 {
        let size = sizes_reader.read_u16::<BigEndian>().unwrap();
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
