//! glTF 2.0 JSON export with an embedded binary buffer.

use crate::io::{IoError, finite_f32};
use crate::triangulated::IndexedTriangulated3D;
use base64::Engine;
use base64::engine::general_purpose::STANDARD as BASE64_ENGINE;
use hashbrown::HashMap;
use serde_json::json;
use std::fmt::Debug;
use std::io::Write;

fn checked_u32(value: usize, limit: &'static str) -> Result<u32, IoError> {
    u32::try_from(value).map_err(|_| IoError::SizeOverflow {
        format: "glTF",
        limit,
    })
}

pub fn to_gltf<T: IndexedTriangulated3D>(
    shape: &T,
    object_name: &str,
) -> Result<String, IoError> {
    let indexed = shape.indexed_triangles();
    if indexed.faces.is_empty() {
        return Err(IoError::Geometry {
            format: "glTF",
            detail: "cannot export an empty primitive".into(),
        });
    }

    let mut vertices = Vec::<([f32; 3], [f32; 3])>::new();
    let mut vertex_map = HashMap::<(usize, usize), u32>::new();
    let index_capacity = indexed
        .faces
        .len()
        .checked_mul(3)
        .ok_or(IoError::SizeOverflow {
            format: "glTF",
            limit: "index buffer capacity",
        })?;
    let mut indices = Vec::<u32>::with_capacity(index_capacity);
    for face in indexed.faces {
        for (position_index, normal_index) in face {
            if position_index >= indexed.positions.len()
                || normal_index >= indexed.normals.len()
            {
                return Err(IoError::Geometry {
                    format: "glTF",
                    detail: "indexed triangle references missing vertex data".into(),
                });
            }
            let key = (position_index, normal_index);
            let index = if let Some(index) = vertex_map.get(&key) {
                *index
            } else {
                let index = checked_u32(vertices.len(), "u32 vertex index")?;
                let position = &indexed.positions[position_index];
                let normal = &indexed.normals[normal_index];
                vertices.push((
                    [
                        finite_f32(&position.x, "glTF", "position x")?,
                        finite_f32(&position.y, "glTF", "position y")?,
                        finite_f32(&position.z, "glTF", "position z")?,
                    ],
                    [
                        finite_f32(&normal.0[0], "glTF", "normal x")?,
                        finite_f32(&normal.0[1], "glTF", "normal y")?,
                        finite_f32(&normal.0[2], "glTF", "normal z")?,
                    ],
                ));
                vertex_map.insert(key, index);
                index
            };
            indices.push(index);
        }
    }

    let vertex_byte_capacity =
        vertices.len().checked_mul(12).ok_or(IoError::SizeOverflow {
            format: "glTF",
            limit: "vertex buffer capacity",
        })?;
    let mut position_bytes = Vec::with_capacity(vertex_byte_capacity);
    let mut normal_bytes = Vec::with_capacity(vertex_byte_capacity);
    let mut minimum = [f32::INFINITY; 3];
    let mut maximum = [f32::NEG_INFINITY; 3];
    for (position, normal) in &vertices {
        for axis in 0..3 {
            minimum[axis] = minimum[axis].min(position[axis]);
            maximum[axis] = maximum[axis].max(position[axis]);
            position_bytes.extend_from_slice(&position[axis].to_le_bytes());
            normal_bytes.extend_from_slice(&normal[axis].to_le_bytes());
        }
    }
    let index_byte_capacity = indices.len().checked_mul(4).ok_or(IoError::SizeOverflow {
        format: "glTF",
        limit: "index byte-buffer capacity",
    })?;
    let mut index_bytes = Vec::with_capacity(index_byte_capacity);
    for index in &indices {
        index_bytes.extend_from_slice(&index.to_le_bytes());
    }

    let positions_len = checked_u32(position_bytes.len(), "u32 buffer-view length")?;
    let normals_len = checked_u32(normal_bytes.len(), "u32 buffer-view length")?;
    let indices_len = checked_u32(index_bytes.len(), "u32 buffer-view length")?;
    let normals_offset = positions_len;
    let indices_offset =
        normals_offset
            .checked_add(normals_len)
            .ok_or(IoError::SizeOverflow {
                format: "glTF",
                limit: "u32 buffer offset",
            })?;

    let mut buffer = Vec::new();
    buffer.extend_from_slice(&position_bytes);
    buffer.extend_from_slice(&normal_bytes);
    buffer.extend_from_slice(&index_bytes);
    let byte_length = checked_u32(buffer.len(), "u32 buffer length")?;
    let vertex_count = checked_u32(vertices.len(), "u32 accessor vertex count")?;
    let index_count = checked_u32(indices.len(), "u32 accessor index count")?;
    let uri = format!(
        "data:application/octet-stream;base64,{}",
        BASE64_ENGINE.encode(buffer)
    );

    let document = json!({
        "asset": {"version": "2.0", "generator": "csgrs"},
        "buffers": [{"byteLength": byte_length, "uri": uri}],
        "bufferViews": [
            {"buffer": 0, "byteOffset": 0, "byteLength": positions_len, "target": 34962},
            {"buffer": 0, "byteOffset": normals_offset, "byteLength": normals_len, "target": 34962},
            {"buffer": 0, "byteOffset": indices_offset, "byteLength": indices_len, "target": 34963}
        ],
        "accessors": [
            {"bufferView": 0, "componentType": 5126, "count": vertex_count, "type": "VEC3", "min": minimum, "max": maximum},
            {"bufferView": 1, "componentType": 5126, "count": vertex_count, "type": "VEC3"},
            {"bufferView": 2, "componentType": 5125, "count": index_count, "type": "SCALAR"}
        ],
        "meshes": [{"name": object_name, "primitives": [{"attributes": {"POSITION": 0, "NORMAL": 1}, "indices": 2}]}],
        "nodes": [{"mesh": 0}],
        "scenes": [{"nodes": [0]}],
        "scene": 0
    });
    serde_json::to_string_pretty(&document).map_err(|error| IoError::Geometry {
        format: "glTF",
        detail: error.to_string(),
    })
}

pub fn write_gltf<T: IndexedTriangulated3D, W: Write>(
    shape: &T,
    writer: &mut W,
    object_name: &str,
) -> Result<(), IoError> {
    writer.write_all(to_gltf(shape, object_name)?.as_bytes())?;
    Ok(())
}

macro_rules! impl_gltf_export {
    ($type:ty) => {
        impl<M: Clone + Debug + Send + Sync> $type {
            pub fn to_gltf(&self, name: &str) -> Result<String, IoError> {
                to_gltf(self, name)
            }

            pub fn write_gltf<W: Write>(
                &self,
                writer: &mut W,
                name: &str,
            ) -> Result<(), IoError> {
                write_gltf(self, writer, name)
            }
        }
    };
}

impl_gltf_export!(crate::mesh::Mesh<M>);
#[cfg(feature = "sketch")]
impl crate::sketch::Profile {
    pub fn to_gltf(&self, name: &str) -> Result<String, IoError> {
        to_gltf(self, name)
    }

    pub fn write_gltf<W: Write>(&self, writer: &mut W, name: &str) -> Result<(), IoError> {
        write_gltf(self, writer, name)
    }
}

#[cfg(test)]
mod tests {
    use super::{checked_u32, to_gltf};
    use crate::io::{IoError, test_support::InvalidIndexed};
    use crate::mesh::Mesh;
    use hyperlattice::Real;

    #[test]
    fn output_is_valid_json_and_escapes_names() {
        let mesh = Mesh::<()>::cube(Real::one(), ());
        let output = mesh.to_gltf("quoted \" name\n").unwrap();
        let parsed: serde_json::Value = serde_json::from_str(&output).unwrap();
        assert_eq!(parsed["meshes"][0]["name"], "quoted \" name\n");
        assert!(parsed["accessors"][0].get("min").is_some());
        let document = gltf::Gltf::from_slice(output.as_bytes()).unwrap();
        assert_eq!(document.meshes().count(), 1);
        assert_eq!(document.accessors().count(), 3);
    }

    #[test]
    fn rejects_empty_invalid_and_overflowing_inputs() {
        assert!(Mesh::<()>::empty().to_gltf("empty").is_err());
        assert!(matches!(
            to_gltf(&InvalidIndexed, "invalid"),
            Err(IoError::Geometry { format: "glTF", .. })
        ));
        assert!(matches!(
            checked_u32(usize::MAX, "test count"),
            Err(IoError::SizeOverflow { format: "glTF", .. })
        ));
    }
}
