use std::{io::Write, path::Path};

use byteorder::{BigEndian, WriteBytesExt};
use glam::Vec3;
use md5::{Digest, Md5};

use crate::{Lod, model::Model, spatial::OctreeOpsState};

use super::{
    Flags,
    consts::{RESERVED_1, RESERVED_2, VTM_MAGIC, VTM_VERSION},
};

pub fn export_model_to_obj<P: AsRef<Path>>(name: String, path: &P, model: &Model, lod: Lod) {
    let mut vertices: Vec<Vec3> = Vec::new();
    let mut normals: Vec<Vec3> = Vec::new();
    let mut indices: Vec<u32> = Vec::new();

    let store = model.get_store();
    let store = store.read();

    for chunk in model.chunks.iter() {
        if chunk.is_empty() {
            continue;
        }

        chunk.generate_mesh_arrays(
            &store,
            &mut vertices,
            &mut normals,
            &mut indices,
            chunk.get_position().as_vec3(),
            lod,
        );
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

    let max_depth = model.max_depth();

    writer.write_all(&VTM_MAGIC).unwrap();
    writer.write_u16::<BigEndian>(VTM_VERSION).unwrap();
    writer.write_u16::<BigEndian>(flags.bits()).unwrap();
    writer.write_u8(max_depth.max()).unwrap();
    writer
        .write_f32::<BigEndian>(model.chunk_world_size)
        .unwrap();
    writer.write_u32::<BigEndian>(RESERVED_1).unwrap();
    writer.write_u32::<BigEndian>(RESERVED_2).unwrap();

    let world_bounds = model.world_bounds;
    writer.write_i32::<BigEndian>(world_bounds.x).unwrap();
    writer.write_i32::<BigEndian>(world_bounds.y).unwrap();
    writer.write_i32::<BigEndian>(world_bounds.z).unwrap();

    writer.write_u8(name.len().try_into().unwrap()).unwrap();
    writer.write_all(name.as_bytes()).unwrap();

    let mut data = Vec::new();
    model.serialize(&mut data);

    let mut md5_hasher = Md5::new();
    md5_hasher.update(&data);
    let md5_hash = md5_hasher.finalize();

    writer.write_all(&md5_hash).unwrap();

    let data = if flags.contains(Flags::COMPRESSED) {
        let mut encoder = zstd::stream::Encoder::new(Vec::new(), 7).unwrap();
        std::io::copy(&mut data.as_slice(), &mut encoder).unwrap();
        encoder.finish().unwrap()
    } else {
        data
    };

    writer
        .write_u32::<BigEndian>(data.len().try_into().unwrap())
        .unwrap();
    writer.write_all(&data).unwrap();
}
