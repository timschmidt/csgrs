#![doc = " glTF 2.0 file format support"]
#![doc = ""]
#![doc = " This module provides export functionality for glTF 2.0 files,"]
#![doc = " a modern, efficient, and widely supported 3D asset format."]

use crate::triangulated::IndexedTriangulated3D;
use crate::vertex::Vertex;
use base64::Engine;
use base64::engine::general_purpose::STANDARD as BASE64_ENGINE;
use hashbrown::HashMap;
use std::fmt::Debug;
use std::io::Write;

fn real_f32(value: &hyperlattice::Real) -> f32 {
    value
        .to_f32_lossy()
        .filter(|value| value.is_finite())
        .unwrap_or(0.0)
}

fn build_gltf_buffers<T: IndexedTriangulated3D>(shape: &T) -> (Vec<Vertex>, Vec<u32>) {
    let indexed = shape.indexed_triangles();
    let mut vertices = Vec::new();
    let mut vertex_map = HashMap::<(usize, usize), u32>::new();
    let indices = indexed
        .faces
        .into_iter()
        .flat_map(|face| {
            face.map(|key @ (position, normal)| {
                *vertex_map.entry(key).or_insert_with(|| {
                    let index = vertices.len() as u32;
                    vertices.push(Vertex {
                        position: indexed.positions[position].clone(),
                        normal: indexed.normals[normal].clone(),
                    });
                    index
                })
            })
        })
        .collect();
    (vertices, indices)
}

/// Build a glTF 2.0 JSON document with a single mesh & single scene,
/// using POSITION and NORMAL attributes and UNSIGNED_INT indices.
///
/// All binary data is stored in a single buffer as a base64-embedded data URI.
fn gltf_from_vertices(vertices: &[Vertex], indices: &[u32], object_name: &str) -> String {
    // Pack positions, normals and indices into binary buffers
    let mut position_bytes = Vec::with_capacity(vertices.len() * 3 * 4);
    let mut normal_bytes = Vec::with_capacity(vertices.len() * 3 * 4);
    let mut index_bytes = Vec::with_capacity(indices.len() * 4);

    #[allow(clippy::unnecessary_cast)]
    {
        for v in vertices {
            let px = real_f32(&v.position.x);
            let py = real_f32(&v.position.y);
            let pz = real_f32(&v.position.z);

            position_bytes.extend_from_slice(&px.to_le_bytes());
            position_bytes.extend_from_slice(&py.to_le_bytes());
            position_bytes.extend_from_slice(&pz.to_le_bytes());

            let nx = real_f32(&v.normal.0[0]);
            let ny = real_f32(&v.normal.0[1]);
            let nz = real_f32(&v.normal.0[2]);

            normal_bytes.extend_from_slice(&nx.to_le_bytes());
            normal_bytes.extend_from_slice(&ny.to_le_bytes());
            normal_bytes.extend_from_slice(&nz.to_le_bytes());
        }

        for &idx in indices {
            index_bytes.extend_from_slice(&idx.to_le_bytes());
        }
    }

    let positions_len = position_bytes.len() as u32;
    let normals_len = normal_bytes.len() as u32;
    let indices_len = index_bytes.len() as u32;

    let positions_offset: u32 = 0;
    let normals_offset: u32 = positions_offset + positions_len;
    let indices_offset: u32 = normals_offset + normals_len;

    let mut buffer_data = Vec::with_capacity(
        positions_len as usize + normals_len as usize + indices_len as usize,
    );
    buffer_data.extend_from_slice(&position_bytes);
    buffer_data.extend_from_slice(&normal_bytes);
    buffer_data.extend_from_slice(&index_bytes);

    let buffer_byte_length = buffer_data.len() as u32;
    let buffer_base64 = BASE64_ENGINE.encode(&buffer_data);

    let vertex_count = vertices.len();
    let index_count = indices.len();

    // Minimal glTF 2.0 JSON with one mesh, one node, one scene.
    // We do not emit `min`/`max` for accessors to keep it simple.
    let mut json = String::new();
    json.push_str("{\n");
    json.push_str("  \"asset\": {\n");
    json.push_str("    \"version\": \"2.0\",\n");
    json.push_str("    \"generator\": \"csgrs\"\n");
    json.push_str("  },\n");
    json.push_str("  \"buffers\": [\n");
    json.push_str(&format!(
        "    {{\"byteLength\": {}, \"uri\": \"data:application/octet-stream;base64,{}\"}}\n",
        buffer_byte_length, buffer_base64
    ));
    json.push_str("  ],\n");
    json.push_str("  \"bufferViews\": [\n");
    json.push_str(&format!(
        "    {{\"buffer\": 0, \"byteOffset\": {}, \"byteLength\": {}, \"target\": 34962}},\n",
        positions_offset, positions_len
    ));
    json.push_str(&format!(
        "    {{\"buffer\": 0, \"byteOffset\": {}, \"byteLength\": {}, \"target\": 34962}},\n",
        normals_offset, normals_len
    ));
    json.push_str(&format!(
        "    {{\"buffer\": 0, \"byteOffset\": {}, \"byteLength\": {}, \"target\": 34963}}\n",
        indices_offset, indices_len
    ));
    json.push_str("  ],\n");
    json.push_str("  \"accessors\": [\n");
    json.push_str(&format!(
        "    {{\"bufferView\": 0, \"componentType\": 5126, \"count\": {}, \"type\": \"VEC3\"}},\n",
        vertex_count
    ));
    json.push_str(&format!(
        "    {{\"bufferView\": 1, \"componentType\": 5126, \"count\": {}, \"type\": \"VEC3\"}},\n",
        vertex_count
    ));
    json.push_str(&format!(
        "    {{\"bufferView\": 2, \"componentType\": 5125, \"count\": {}, \"type\": \"SCALAR\"}}\n",
        index_count
    ));
    json.push_str("  ],\n");
    json.push_str("  \"meshes\": [\n");
    json.push_str(&format!(
        "    {{\"name\": \"{}\", \"primitives\": [{{\"attributes\": {{\"POSITION\": 0, \"NORMAL\": 1}}, \"indices\": 2}}]}}\n",
        object_name
    ));
    json.push_str("  ],\n");
    json.push_str("  \"nodes\": [\n");
    json.push_str("    {\"mesh\": 0}\n");
    json.push_str("  ],\n");
    json.push_str("  \"scenes\": [\n");
    json.push_str("    {\"nodes\": [0]}\n");
    json.push_str("  ],\n");
    json.push_str("  \"scene\": 0\n");
    json.push_str("}\n");

    json
}

#[doc = " Export any `Triangulated3D` shape to glTF 2.0 JSON as a string"]
pub fn to_gltf<T: IndexedTriangulated3D>(shape: &T, object_name: &str) -> String {
    let (vertices, indices) = build_gltf_buffers(shape);
    gltf_from_vertices(&vertices, &indices, object_name)
}

#[doc = " Export any `Triangulated3D` shape to a glTF 2.0 JSON writer"]
pub fn write_gltf<T: IndexedTriangulated3D, W: Write>(
    shape: &T,
    writer: &mut W,
    object_name: &str,
) -> std::io::Result<()> {
    let gltf_content = to_gltf(shape, object_name);
    writer.write_all(gltf_content.as_bytes())
}

impl<M: Clone + Debug + Send + Sync> crate::mesh::Mesh<M> {
    pub fn to_gltf(&self, object_name: &str) -> String {
        self::to_gltf(self, object_name)
    }

    pub fn write_gltf<W: Write>(
        &self,
        writer: &mut W,
        object_name: &str,
    ) -> std::io::Result<()> {
        self::write_gltf(self, writer, object_name)
    }
}

#[cfg(feature = "sketch")]
impl<M: Clone + Debug + Send + Sync> crate::sketch::Profile<M> {
    pub fn to_gltf(&self, object_name: &str) -> String {
        self::to_gltf(self, object_name)
    }

    pub fn write_gltf<W: Write>(
        &self,
        writer: &mut W,
        object_name: &str,
    ) -> std::io::Result<()> {
        self::write_gltf(self, writer, object_name)
    }
}
