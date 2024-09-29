use std::{io::Write, path::PathBuf};

use byteorder::{BigEndian, WriteBytesExt};
use md5::{Digest, Md5};

use crate::{
    chunk::MAX_LOD_LEVEL,
    io::{DEFAULT_FLAGS, RESERVED_1, RESERVED_2, VTM_MAGIC, VTM_VERSION},
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

    writer.write_all(&VTM_MAGIC).unwrap();
    writer.write_u16::<BigEndian>(VTM_VERSION).unwrap();
    writer.write_u16::<BigEndian>(DEFAULT_FLAGS.bits()).unwrap();
    writer.write_u8(MAX_LOD_LEVEL as u8).unwrap();
    writer.write_u32::<BigEndian>(RESERVED_1).unwrap();
    writer.write_u32::<BigEndian>(RESERVED_2).unwrap();

    let size = model.chunks_size;
    writer.write_u16::<BigEndian>(size.x as u16).unwrap();
    writer.write_u16::<BigEndian>(size.y as u16).unwrap();
    writer.write_u16::<BigEndian>(size.z as u16).unwrap();

    writer.write_u8(name.len() as u8).unwrap();
    writer.write_all(name.as_bytes()).unwrap();

    let mut data = Vec::new();
    model.serialize(&mut data);

    let mut encoder = zstd::stream::Encoder::new(Vec::new(), 7).unwrap();
    std::io::copy(&mut data.as_slice(), &mut encoder).unwrap();
    let compressed_data = encoder.finish().unwrap();

    let mut md5_hasher = Md5::new();
    md5_hasher.update(&compressed_data);
    let md5_hash = md5_hasher.finalize();

    writer.write_all(&md5_hash).unwrap();

    writer
        .write_u32::<BigEndian>(compressed_data.len() as u32)
        .unwrap();
    writer.write_all(&compressed_data).unwrap();
}
