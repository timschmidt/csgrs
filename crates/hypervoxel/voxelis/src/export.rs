use std::{
    io::Write,
    path::{Path, PathBuf},
};

use byteorder::{BigEndian, LittleEndian};

use crate::{chunk::MAX_LOD_LEVEL, Model};

const VTM_VERSION: u16 = 1;

pub fn encode_varint(mut value: usize) -> Vec<u8> {
    let mut bytes = Vec::new();

    while value >= 0x80 {
        // Set the MSB to indicate more bytes follow
        bytes.push((value as u8 & 0x7F) | 0x80);
        value >>= 7;
    }

    // Last byte with MSB unset
    bytes.push(value as u8);

    bytes
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
    let mut data: Vec<u8> = Vec::new();

    model.run_length_encode(&mut data);

    let mut encoder = zstd::stream::Encoder::new(Vec::new(), 7).unwrap();
    std::io::copy(&mut data.as_slice(), &mut encoder).unwrap();
    let compressed_data = encoder.finish().unwrap();

    let mut vox_file = std::fs::File::create(path).unwrap();
    let mut writer = std::io::BufWriter::new(&mut vox_file);

    writer.write_all("VoxTreeModel".as_bytes()).unwrap();
    writer.write_u16::<LittleEndian>(VTM_VERSION).unwrap();
    writer.write_u8(MAX_LOD_LEVEL as u8).unwrap();

    let size = model.chunks_size;
    writer.write_u16::<LittleEndian>(size.x as u16).unwrap();
    writer.write_u16::<LittleEndian>(size.y as u16).unwrap();
    writer.write_u16::<LittleEndian>(size.z as u16).unwrap();

    writer.write_u8(name.len() as u8).unwrap();
    writer.write_all(name.as_bytes()).unwrap();

    writer.write_all(&compressed_data).unwrap();
}
