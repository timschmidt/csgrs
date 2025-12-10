#![doc = " glTF 2.0 file format support"]
#![doc = ""]
#![doc = " This module provides export functionality for glTF 2.0 files,"]
#![doc = " a modern, efficient, and widely supported 3D asset format."]

use crate::float_types::{tolerance, Real};
use crate::triangulated::Triangulated3D;
use nalgebra::{Point3, Vector3};
use std::fmt::Debug;
use std::io::Write;
use base64::engine::general_purpose::STANDARD as BASE64_ENGINE;
use base64::Engine;

#[derive(Clone)]
struct GltfVertex {
    position: Point3<Real>,
    normal: Vector3<Real>,
}

/// Add a vertex to the list, reusing an existing one if position and normal
/// are within `tolerance()`.
fn add_unique_vertex_gltf(
    vertices: &mut Vec<GltfVertex>,
    position: Point3<Real>,
    normal: Vector3<Real>,
) -> u32 {
    for (i, existing) in vertices.iter().enumerate() {
        if (existing.position.coords - position.coords).norm() < tolerance()
            && (existing.normal - normal).norm() < tolerance()
        {
            return i as u32;
        }
    }
    vertices.push(GltfVertex { position, normal });
    (vertices.len() - 1) as u32
}

fn build_gltf_buffers<T: Triangulated3D>(
    shape: &T,
) -> (Vec<GltfVertex>, Vec<u32>) {
    let mut vertices = Vec::<GltfVertex>::new();
    let mut indices  = Vec::<u32>::new();

    shape.visit_triangles(|tri| {
        for v in tri {
            let idx = add_unique_vertex_gltf(
                &mut vertices,
                v.position,
                v.normal,
            );
            indices.push(idx);
        }
    });

    (vertices, indices)
}

/// Build a glTF 2.0 JSON document with a single mesh & single scene,
/// using POSITION and NORMAL attributes and UNSIGNED_INT indices.
///
/// All binary data is stored in a single buffer as a base64-embedded data URI.
fn gltf_from_vertices(
    vertices: &[GltfVertex],
    indices: &[u32],
    object_name: &str,
) -> String {
    // Pack positions, normals and indices into binary buffers
    let mut position_bytes = Vec::with_capacity(vertices.len() * 3 * 4);
    let mut normal_bytes = Vec::with_capacity(vertices.len() * 3 * 4);
    let mut index_bytes = Vec::with_capacity(indices.len() * 4);

    #[allow(clippy::unnecessary_cast)]
    {
        for v in vertices {
            let px = v.position.x as f32;
            let py = v.position.y as f32;
            let pz = v.position.z as f32;

            position_bytes.extend_from_slice(&px.to_le_bytes());
            position_bytes.extend_from_slice(&py.to_le_bytes());
            position_bytes.extend_from_slice(&pz.to_le_bytes());

            let nx = v.normal.x as f32;
            let ny = v.normal.y as f32;
            let nz = v.normal.z as f32;

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

impl<S: Clone + Debug + Send + Sync> crate::mesh::Mesh<S> {
    pub fn to_gltf(&self, object_name: &str) -> String {
        let (vertices, indices) = build_gltf_buffers(self);
        gltf_from_vertices(&vertices, &indices, object_name)
    }

    pub fn write_gltf<W: Write>(
        &self,
        writer: &mut W,
        object_name: &str,
    ) -> std::io::Result<()> {
        let gltf_content = self.to_gltf(object_name);
        writer.write_all(gltf_content.as_bytes())
    }
}

impl<S: Clone + Debug + Send + Sync> crate::sketch::Sketch<S> {
    pub fn to_gltf(&self, object_name: &str) -> String {
        let (vertices, indices) = build_gltf_buffers(self);
        gltf_from_vertices(&vertices, &indices, object_name)
    }

    pub fn write_gltf<W: Write>(
        &self,
        writer: &mut W,
        object_name: &str,
    ) -> std::io::Result<()> {
        let gltf_content = self.to_gltf(object_name);
        writer.write_all(gltf_content.as_bytes())
    }
}

impl<S: Clone + Debug + Send + Sync> crate::bmesh::BMesh<S> {
    pub fn to_gltf(&self, object_name: &str) -> String {
        let (vertices, indices) = build_gltf_buffers(self);
        gltf_from_vertices(&vertices, &indices, object_name)
    }

    pub fn write_gltf<W: Write>(
        &self,
        writer: &mut W,
        object_name: &str,
    ) -> std::io::Result<()> {
        let gltf_content = self.to_gltf(object_name);
        writer.write_all(gltf_content.as_bytes())
    }
}
