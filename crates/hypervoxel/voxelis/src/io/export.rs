use std::{io::Write, path::Path};

use byteorder::{BigEndian, WriteBytesExt};
use glam::Vec3;
use md5::{Digest, Md5};

use crate::io::Flags;
use crate::model::Model;
use crate::world::Chunk;

use super::consts::{RESERVED_1, RESERVED_2, VTM_MAGIC, VTM_VERSION};

const MAX_LOD_LEVEL: usize = 5;

pub fn export_model_to_obj<P: AsRef<Path>>(name: String, path: &P, model: &Model) {
    let mut vertices: Vec<Vec3> = Vec::new();
    let mut normals: Vec<Vec3> = Vec::new();
    let mut indices: Vec<u32> = Vec::new();

    for chunk in model.chunks.iter() {
        if chunk.is_empty() {
            continue;
        }

        let offset = chunk.get_position().as_vec3();

        let data = chunk.to_vec(0);

        chunk.generate_mesh_arrays(&data, &mut vertices, &mut normals, &mut indices, offset);
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

pub fn export_model_to_vtm<P: AsRef<Path>>(name: String, path: &P, model: &Model) {
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
