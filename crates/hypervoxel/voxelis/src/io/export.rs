use std::{io::Write, path::Path};

use byteorder::{BigEndian, WriteBytesExt};
use glam::Vec3;
use md5::{Digest, Md5};

use crate::{Lod, spatial::VoxOpsState, world::VoxModel};

use super::{
    Flags,
    consts::{RESERVED_1, RESERVED_2, VTM_MAGIC, VTM_VERSION},
};

pub fn export_model_to_obj<P: AsRef<Path>>(name: String, path: &P, model: &VoxModel, lod: Lod) {
    let mut vertices: Vec<Vec3> = Vec::new();
    let mut normals: Vec<Vec3> = Vec::new();
    let mut indices: Vec<u32> = Vec::new();

    let interner = model.get_interner();
    let interner = interner.read();

    for (_, chunk) in model.chunks.iter() {
        if chunk.is_empty() {
            continue;
        }

        chunk.generate_mesh_arrays(
            &interner,
            &mut vertices,
            &mut normals,
            &mut indices,
            chunk.get_world_position(),
            lod,
        );
    }

    let obj_file = std::fs::File::create(path).unwrap();
    let mut writer = std::io::BufWriter::new(obj_file);

    writer.write_all(format!("o {name}\n").as_bytes()).unwrap();

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

pub struct ByteSize(pub usize);

impl std::fmt::Display for ByteSize {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        if self.0 < 1024 {
            write!(f, "{} B", self.0)
        } else if self.0 < 1024 * 1024 {
            write!(f, "{:.3} KB", self.0 as f64 / 1024.0)
        } else if self.0 < 1024 * 1024 * 1024 {
            write!(f, "{:.3} MB", self.0 as f64 / (1024.0 * 1024.0))
        } else {
            write!(f, "{:.3} GB", self.0 as f64 / (1024.0 * 1024.0 * 1024.0))
        }
    }
}

pub fn export_model_to_vtm<P: AsRef<Path>>(name: String, path: &P, model: &VoxModel) {
    print!("Exporting VTM model to {}", path.as_ref().display(),);

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

    let file_len = writer.get_ref().metadata().unwrap().len();

    println!(" ({})", ByteSize(file_len as usize));
}
