//! glTF 2.0 triangle-scene import and embedded-buffer JSON export.

use crate::io::{IoError, finite_f32};
use crate::mesh::Mesh;
use crate::triangulated::{IndexedTriangleMesh3D, IndexedTriangulated3D};
use base64::Engine;
use base64::engine::general_purpose::STANDARD as BASE64_ENGINE;
use hashbrown::{HashMap, HashSet};
use hyperlattice::{Point3, Real};
use serde_json::json;
use std::fmt::Debug;
use std::io::Write;

type Matrix4 = [[f64; 4]; 4];

fn checked_u32(value: usize, limit: &'static str) -> Result<u32, IoError> {
    u32::try_from(value).map_err(|_| IoError::SizeOverflow {
        format: "glTF",
        limit,
    })
}

/// Geometry-only evidence produced while flattening one glTF scene.
#[derive(Clone, Debug)]
pub struct GltfMeshImport<M: Clone + Debug + Send + Sync> {
    /// Flattened exact-aware triangle mesh.
    pub mesh: Mesh<M>,
    /// Selected default scene, or the first scene when no default is declared.
    pub scene_index: usize,
    /// Number of scene nodes that instantiated a mesh.
    pub mesh_node_count: usize,
    /// Number of triangle primitives consumed across those nodes.
    pub primitive_count: usize,
}

/// Imports the default (or first) self-contained glTF 2.0 scene.
///
/// JSON glTF buffers must use embedded base64 data URIs; GLB binary buffers are
/// read from the BIN chunk. Filesystem and network resolution remain caller
/// policy. Node transforms are flattened into positions before exact promotion.
/// Skins, morph targets, and non-triangle primitives fail explicitly instead
/// of being silently ignored.
pub fn from_gltf<M: Clone + Debug + Send + Sync>(
    bytes: &[u8],
    metadata: M,
) -> Result<GltfMeshImport<M>, IoError> {
    let asset = gltf::Gltf::from_slice(bytes).map_err(|error| {
        IoError::MalformedInput(format!("glTF document validation failed: {error}"))
    })?;
    let buffers = gltf_buffers(&asset)?;
    let scene = asset
        .default_scene()
        .or_else(|| asset.scenes().next())
        .ok_or_else(|| IoError::MalformedInput("glTF document has no scene".into()))?;
    let scene_index = scene.index();
    let mut positions = Vec::new();
    let mut triangles = Vec::new();
    let mut mesh_node_count = 0usize;
    let mut primitive_count = 0usize;
    for node in scene.nodes() {
        import_gltf_node(
            node,
            &identity_matrix(),
            &buffers,
            &mut positions,
            &mut triangles,
            &mut mesh_node_count,
            &mut primitive_count,
        )?;
    }
    if triangles.is_empty() {
        return Err(IoError::Geometry {
            format: "glTF",
            detail: "selected scene contains no triangle primitives".into(),
        });
    }

    let mut normals = Vec::with_capacity(triangles.len());
    let mut faces = Vec::with_capacity(triangles.len());
    for triangle in triangles {
        let [a, b, c] = triangle;
        let ab = &positions[b] - &positions[a];
        let ac = &positions[c] - &positions[a];
        let normal = ab
            .unit_cross_checked(&ac)
            .map_err(|error| IoError::Geometry {
                format: "glTF",
                detail: format!("triangle has no certifiable nondegenerate normal: {error}"),
            })?;
        let normal_index = normals.len();
        normals.push(normal);
        faces.push([(a, normal_index), (b, normal_index), (c, normal_index)]);
    }
    let mesh = Mesh::from_indexed_triangles(
        IndexedTriangleMesh3D {
            positions,
            normals,
            faces,
        },
        metadata,
    )
    .map_err(|error| IoError::Geometry {
        format: "glTF",
        detail: error.to_string(),
    })?;

    Ok(GltfMeshImport {
        mesh,
        scene_index,
        mesh_node_count,
        primitive_count,
    })
}

fn gltf_buffers(asset: &gltf::Gltf) -> Result<Vec<Vec<u8>>, IoError> {
    asset
        .buffers()
        .map(|buffer| {
            let bytes = match buffer.source() {
                gltf::buffer::Source::Bin => asset.blob.clone().ok_or_else(|| {
                    IoError::MalformedInput(format!(
                        "glTF buffer {} references an absent GLB BIN chunk",
                        buffer.index()
                    ))
                })?,
                gltf::buffer::Source::Uri(uri) => decode_gltf_data_uri(uri)?,
            };
            if bytes.len() < buffer.length() {
                return Err(IoError::MalformedInput(format!(
                    "glTF buffer {} declares {} bytes but provides {}",
                    buffer.index(),
                    buffer.length(),
                    bytes.len()
                )));
            }
            Ok(bytes)
        })
        .collect()
}

fn decode_gltf_data_uri(uri: &str) -> Result<Vec<u8>, IoError> {
    let Some(payload) = uri.strip_prefix("data:") else {
        return Err(IoError::Unsupported {
            format: "glTF",
            detail: format!("external buffer URI {uri:?} requires caller resolution"),
        });
    };
    let (media, encoded) = payload.split_once(',').ok_or_else(|| {
        IoError::MalformedInput("glTF data URI has no comma separator".into())
    })?;
    if !media.split(';').any(|part| part == "base64") {
        return Err(IoError::Unsupported {
            format: "glTF",
            detail: "only base64-encoded embedded buffer data URIs are supported".into(),
        });
    }
    BASE64_ENGINE.decode(encoded).map_err(|error| {
        IoError::MalformedInput(format!("glTF embedded buffer is not valid base64: {error}"))
    })
}

#[allow(clippy::too_many_arguments)]
fn import_gltf_node(
    node: gltf::Node<'_>,
    parent: &Matrix4,
    buffers: &[Vec<u8>],
    positions: &mut Vec<Point3>,
    triangles: &mut Vec<[usize; 3]>,
    mesh_node_count: &mut usize,
    primitive_count: &mut usize,
) -> Result<(), IoError> {
    if node.skin().is_some() {
        return Err(IoError::Unsupported {
            format: "glTF",
            detail: format!("node {} uses skinning", node.index()),
        });
    }
    let local = f32_matrix_to_f64(node.transform().matrix())?;
    let transform = multiply_matrix(parent, &local);
    if let Some(mesh) = node.mesh() {
        *mesh_node_count = (*mesh_node_count)
            .checked_add(1)
            .ok_or(IoError::SizeOverflow {
                format: "glTF",
                limit: "mesh-node count",
            })?;
        for primitive in mesh.primitives() {
            if primitive.mode() != gltf::mesh::Mode::Triangles {
                return Err(IoError::Unsupported {
                    format: "glTF",
                    detail: format!(
                        "mesh {} primitive {} uses {:?} topology",
                        mesh.index(),
                        primitive.index(),
                        primitive.mode()
                    ),
                });
            }
            if primitive.morph_targets().next().is_some() {
                return Err(IoError::Unsupported {
                    format: "glTF",
                    detail: format!(
                        "mesh {} primitive {} uses morph targets",
                        mesh.index(),
                        primitive.index()
                    ),
                });
            }
            let reader =
                primitive.reader(|buffer| buffers.get(buffer.index()).map(Vec::as_slice));
            let source_positions = reader.read_positions().ok_or_else(|| {
                IoError::MalformedInput(format!(
                    "mesh {} primitive {} has no POSITION attribute",
                    mesh.index(),
                    primitive.index()
                ))
            })?;
            let base = positions.len();
            for position in source_positions {
                positions.push(transform_gltf_position(&transform, position)?);
            }
            let vertex_count = positions.len() - base;
            let indices = reader
                .read_indices()
                .map(|indices| indices.into_u32().collect::<Vec<_>>())
                .unwrap_or_else(|| {
                    (0..vertex_count)
                        .map_while(|index| u32::try_from(index).ok())
                        .collect()
                });
            if indices.len() != vertex_count && primitive.indices().is_none() {
                return Err(IoError::SizeOverflow {
                    format: "glTF",
                    limit: "u32 implicit vertex index",
                });
            }
            let chunks = indices.chunks_exact(3);
            if !chunks.remainder().is_empty() {
                return Err(IoError::MalformedInput(format!(
                    "mesh {} primitive {} triangle index count is not divisible by three",
                    mesh.index(),
                    primitive.index()
                )));
            }
            for chunk in chunks {
                let mut triangle = [0usize; 3];
                for (corner, index) in chunk.iter().copied().enumerate() {
                    let index = usize::try_from(index).map_err(|_| IoError::SizeOverflow {
                        format: "glTF",
                        limit: "platform vertex index",
                    })?;
                    if index >= vertex_count {
                        return Err(IoError::MalformedInput(format!(
                            "mesh {} primitive {} references vertex {index} but has {vertex_count}",
                            mesh.index(),
                            primitive.index()
                        )));
                    }
                    triangle[corner] =
                        base.checked_add(index).ok_or(IoError::SizeOverflow {
                            format: "glTF",
                            limit: "combined vertex index",
                        })?;
                }
                triangles.push(triangle);
            }
            *primitive_count =
                (*primitive_count)
                    .checked_add(1)
                    .ok_or(IoError::SizeOverflow {
                        format: "glTF",
                        limit: "primitive count",
                    })?;
        }
    }
    for child in node.children() {
        import_gltf_node(
            child,
            &transform,
            buffers,
            positions,
            triangles,
            mesh_node_count,
            primitive_count,
        )?;
    }
    Ok(())
}

const fn identity_matrix() -> Matrix4 {
    [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
}

fn f32_matrix_to_f64(matrix: [[f32; 4]; 4]) -> Result<Matrix4, IoError> {
    let mut result = [[0.0; 4]; 4];
    for column in 0..4 {
        for row in 0..4 {
            let value = f64::from(matrix[column][row]);
            if !value.is_finite() {
                return Err(IoError::MalformedInput(
                    "glTF node transform contains a non-finite value".into(),
                ));
            }
            result[column][row] = value;
        }
    }
    Ok(result)
}

fn multiply_matrix(left: &Matrix4, right: &Matrix4) -> Matrix4 {
    let mut result = [[0.0; 4]; 4];
    for column in 0..4 {
        for row in 0..4 {
            result[column][row] =
                (0..4).map(|slot| left[slot][row] * right[column][slot]).sum();
        }
    }
    result
}

fn transform_gltf_position(matrix: &Matrix4, position: [f32; 3]) -> Result<Point3, IoError> {
    let [x, y, z] = position.map(f64::from);
    let transformed = [
        matrix[0][0] * x + matrix[1][0] * y + matrix[2][0] * z + matrix[3][0],
        matrix[0][1] * x + matrix[1][1] * y + matrix[2][1] * z + matrix[3][1],
        matrix[0][2] * x + matrix[1][2] * y + matrix[2][2] * z + matrix[3][2],
    ];
    let w = matrix[0][3] * x + matrix[1][3] * y + matrix[2][3] * z + matrix[3][3];
    if !transformed.iter().all(|value| value.is_finite()) || w != 1.0 {
        return Err(IoError::Unsupported {
            format: "glTF",
            detail: "node transform is non-finite or non-affine".into(),
        });
    }
    Ok(Point3::new(
        exact_gltf_real(transformed[0], "position x")?,
        exact_gltf_real(transformed[1], "position y")?,
        exact_gltf_real(transformed[2], "position z")?,
    ))
}

fn exact_gltf_real(value: f64, field: &'static str) -> Result<Real, IoError> {
    Real::try_from(value).map_err(|error| IoError::Geometry {
        format: "glTF",
        detail: format!("{field} cannot promote to an exact scalar: {error}"),
    })
}

pub fn to_gltf<T: IndexedTriangulated3D>(
    shape: &T,
    object_name: &str,
) -> Result<String, IoError> {
    to_gltf_scene(object_name, &[GltfSceneObject::new(object_name, shape)])
}

/// One named geometry object in a glTF scene.
///
/// The object carries no application semantics: callers retain those in their
/// own scene manifest and provide only render-safe names plus triangulated
/// geometry at this format boundary.
pub struct GltfSceneObject {
    name: String,
    indexed: IndexedTriangleMesh3D,
}

impl GltfSceneObject {
    /// Captures one named indexed-triangle source for scene serialization.
    pub fn new<T: IndexedTriangulated3D>(name: impl Into<String>, shape: &T) -> Self {
        Self {
            name: name.into(),
            indexed: shape.indexed_triangles(),
        }
    }
}

/// Serializes multiple independently named geometry objects into one glTF 2.0
/// scene with an embedded binary buffer.
pub fn to_gltf_scene(
    scene_name: &str,
    objects: &[GltfSceneObject],
) -> Result<String, IoError> {
    if objects.is_empty() {
        return Err(IoError::Geometry {
            format: "glTF",
            detail: "cannot export an empty scene".into(),
        });
    }

    let mut buffer = Vec::new();
    let mut buffer_views = Vec::with_capacity(objects.len() * 3);
    let mut accessors = Vec::with_capacity(objects.len() * 3);
    let mut meshes = Vec::with_capacity(objects.len());
    let mut nodes = Vec::with_capacity(objects.len());
    let mut scene_nodes = Vec::with_capacity(objects.len());
    let mut names = HashSet::with_capacity(objects.len());

    for object in objects {
        if !names.insert(object.name.as_str()) {
            return Err(IoError::Geometry {
                format: "glTF",
                detail: format!("duplicate scene object name {:?}", object.name),
            });
        }
        let indexed = &object.indexed;
        if indexed.faces.is_empty() {
            return Err(IoError::Geometry {
                format: "glTF",
                detail: format!("cannot export empty scene object {:?}", object.name),
            });
        }

        let mut vertices = Vec::<([f32; 3], [f32; 3])>::new();
        let mut vertex_map = HashMap::<(usize, usize), u32>::new();
        let index_capacity =
            indexed
                .faces
                .len()
                .checked_mul(3)
                .ok_or(IoError::SizeOverflow {
                    format: "glTF",
                    limit: "index buffer capacity",
                })?;
        let mut indices = Vec::<u32>::with_capacity(index_capacity);
        for face in &indexed.faces {
            for (position_index, normal_index) in face {
                if *position_index >= indexed.positions.len()
                    || *normal_index >= indexed.normals.len()
                {
                    return Err(IoError::Geometry {
                        format: "glTF",
                        detail: format!(
                            "scene object {:?} references missing indexed vertex data",
                            object.name
                        ),
                    });
                }
                let key = (*position_index, *normal_index);
                let index = if let Some(index) = vertex_map.get(&key) {
                    *index
                } else {
                    let index = checked_u32(vertices.len(), "u32 vertex index")?;
                    let position = &indexed.positions[*position_index];
                    let normal = &indexed.normals[*normal_index];
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
        let index_byte_capacity =
            indices.len().checked_mul(4).ok_or(IoError::SizeOverflow {
                format: "glTF",
                limit: "index byte-buffer capacity",
            })?;
        let mut index_bytes = Vec::with_capacity(index_byte_capacity);
        for index in &indices {
            index_bytes.extend_from_slice(&index.to_le_bytes());
        }

        let positions_offset = checked_u32(buffer.len(), "u32 buffer offset")?;
        let positions_len = checked_u32(position_bytes.len(), "u32 buffer-view length")?;
        buffer.extend_from_slice(&position_bytes);
        let normals_offset = checked_u32(buffer.len(), "u32 buffer offset")?;
        let normals_len = checked_u32(normal_bytes.len(), "u32 buffer-view length")?;
        buffer.extend_from_slice(&normal_bytes);
        let indices_offset = checked_u32(buffer.len(), "u32 buffer offset")?;
        let indices_len = checked_u32(index_bytes.len(), "u32 buffer-view length")?;
        buffer.extend_from_slice(&index_bytes);

        let position_view = checked_u32(buffer_views.len(), "u32 buffer-view index")?;
        buffer_views.push(json!({
            "buffer": 0,
            "byteOffset": positions_offset,
            "byteLength": positions_len,
            "target": 34962
        }));
        let normal_view = checked_u32(buffer_views.len(), "u32 buffer-view index")?;
        buffer_views.push(json!({
            "buffer": 0,
            "byteOffset": normals_offset,
            "byteLength": normals_len,
            "target": 34962
        }));
        let index_view = checked_u32(buffer_views.len(), "u32 buffer-view index")?;
        buffer_views.push(json!({
            "buffer": 0,
            "byteOffset": indices_offset,
            "byteLength": indices_len,
            "target": 34963
        }));

        let position_accessor = checked_u32(accessors.len(), "u32 accessor index")?;
        accessors.push(json!({
            "bufferView": position_view,
            "componentType": 5126,
            "count": checked_u32(vertices.len(), "u32 accessor vertex count")?,
            "type": "VEC3",
            "min": minimum,
            "max": maximum
        }));
        let normal_accessor = checked_u32(accessors.len(), "u32 accessor index")?;
        accessors.push(json!({
            "bufferView": normal_view,
            "componentType": 5126,
            "count": checked_u32(vertices.len(), "u32 accessor vertex count")?,
            "type": "VEC3"
        }));
        let index_accessor = checked_u32(accessors.len(), "u32 accessor index")?;
        accessors.push(json!({
            "bufferView": index_view,
            "componentType": 5125,
            "count": checked_u32(indices.len(), "u32 accessor index count")?,
            "type": "SCALAR"
        }));

        let mesh_index = checked_u32(meshes.len(), "u32 mesh index")?;
        meshes.push(json!({
            "name": object.name,
            "primitives": [{
                "attributes": {
                    "POSITION": position_accessor,
                    "NORMAL": normal_accessor
                },
                "indices": index_accessor
            }]
        }));
        let node_index = checked_u32(nodes.len(), "u32 node index")?;
        nodes.push(json!({"name": object.name, "mesh": mesh_index}));
        scene_nodes.push(node_index);
    }

    let byte_length = checked_u32(buffer.len(), "u32 buffer length")?;
    let uri = format!(
        "data:application/octet-stream;base64,{}",
        BASE64_ENGINE.encode(buffer)
    );

    let document = json!({
        "asset": {"version": "2.0", "generator": "csgrs"},
        "buffers": [{"byteLength": byte_length, "uri": uri}],
        "bufferViews": buffer_views,
        "accessors": accessors,
        "meshes": meshes,
        "nodes": nodes,
        "scenes": [{"name": scene_name, "nodes": scene_nodes}],
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

/// Writes a named multi-object glTF scene to an arbitrary byte sink.
pub fn write_gltf_scene<W: Write>(
    writer: &mut W,
    scene_name: &str,
    objects: &[GltfSceneObject],
) -> Result<(), IoError> {
    writer.write_all(to_gltf_scene(scene_name, objects)?.as_bytes())?;
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
    use super::{GltfSceneObject, checked_u32, from_gltf, to_gltf, to_gltf_scene};
    use crate::csg::CSG;
    use crate::io::{IoError, test_support::InvalidIndexed};
    use crate::mesh::Mesh;
    use base64::Engine;
    use hyperlattice::{Aabb, Point3, Real};
    use serde_json::json;
    use std::borrow::Cow;

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

    #[test]
    fn public_writer_matches_string_serializer() {
        let mesh = Mesh::<()>::cube(Real::one(), ());
        let expected = to_gltf(&mesh, "cube").unwrap();
        let mut written = Vec::new();
        crate::io::gltf::write_gltf(&mesh, &mut written, "cube").unwrap();
        assert_eq!(written, expected.as_bytes());
    }

    #[test]
    fn named_multi_object_scene_is_valid_and_retains_object_boundaries() {
        let left = Mesh::<()>::cube(Real::one(), ());
        let right = Mesh::<()>::cube(Real::one(), ()).translate(
            Real::from(2),
            Real::zero(),
            Real::zero(),
        );
        let output = to_gltf_scene(
            "assembly",
            &[
                GltfSceneObject::new("left", &left),
                GltfSceneObject::new("right", &right),
            ],
        )
        .unwrap();
        let parsed: serde_json::Value = serde_json::from_str(&output).unwrap();
        assert_eq!(parsed["scenes"][0]["name"], "assembly");
        assert_eq!(parsed["meshes"].as_array().unwrap().len(), 2);
        assert_eq!(parsed["nodes"].as_array().unwrap().len(), 2);
        assert_eq!(parsed["meshes"][0]["name"], "left");
        assert_eq!(parsed["meshes"][1]["name"], "right");
        let document = gltf::Gltf::from_slice(output.as_bytes()).unwrap();
        assert_eq!(document.meshes().count(), 2);
        assert_eq!(document.nodes().count(), 2);
        assert_eq!(document.accessors().count(), 6);
    }

    #[test]
    fn self_contained_scene_import_flattens_nested_node_transforms() {
        let cube = Mesh::<()>::cube(Real::one(), ());
        let mut document: serde_json::Value =
            serde_json::from_str(&to_gltf(&cube, "cube").unwrap()).unwrap();
        document["nodes"][0]["translation"] = json!([1, 2, 3]);
        document["nodes"][0]["children"] = json!([1]);
        document["nodes"]
            .as_array_mut()
            .unwrap()
            .push(json!({"mesh": 0, "translation": [4, 0, 0]}));
        document["scenes"][0]["nodes"] = json!([0]);

        let imported =
            from_gltf(serde_json::to_string(&document).unwrap().as_bytes(), ()).unwrap();
        assert_eq!(imported.scene_index, 0);
        assert_eq!(imported.mesh_node_count, 2);
        assert_eq!(imported.primitive_count, 2);
        assert_eq!(imported.mesh.triangles().len(), 24);
        assert_eq!(
            imported.mesh.bounding_box(),
            Aabb::new(
                Point3::new(Real::from(1), Real::from(2), Real::from(3)),
                Point3::new(Real::from(6), Real::from(3), Real::from(4)),
            )
        );
    }

    #[test]
    fn binary_glb_import_reads_its_bin_chunk() {
        let cube = Mesh::<()>::cube(Real::one(), ());
        let mut document: serde_json::Value =
            serde_json::from_str(&to_gltf(&cube, "cube").unwrap()).unwrap();
        let uri = document["buffers"][0]["uri"].as_str().unwrap();
        let encoded = uri.split_once(',').unwrap().1;
        let bin = base64::engine::general_purpose::STANDARD
            .decode(encoded)
            .unwrap();
        document["buffers"][0].as_object_mut().unwrap().remove("uri");
        let glb = gltf::binary::Glb {
            header: gltf::binary::Header {
                magic: *b"glTF",
                version: 2,
                length: 0,
            },
            json: Cow::Owned(serde_json::to_vec(&document).unwrap()),
            bin: Some(Cow::Owned(bin)),
        }
        .to_vec()
        .unwrap();

        let imported = from_gltf(&glb, ()).unwrap();
        assert_eq!(imported.mesh_node_count, 1);
        assert_eq!(imported.primitive_count, 1);
        assert_eq!(imported.mesh.triangles().len(), 12);
    }

    #[test]
    fn import_rejects_external_buffers_and_non_triangle_topology() {
        let cube = Mesh::<()>::cube(Real::one(), ());
        let mut external: serde_json::Value =
            serde_json::from_str(&to_gltf(&cube, "cube").unwrap()).unwrap();
        external["buffers"][0]["uri"] = json!("cube.bin");
        assert!(matches!(
            from_gltf(serde_json::to_string(&external).unwrap().as_bytes(), ()),
            Err(IoError::Unsupported { format: "glTF", .. })
        ));

        let mut lines: serde_json::Value =
            serde_json::from_str(&to_gltf(&cube, "cube").unwrap()).unwrap();
        lines["meshes"][0]["primitives"][0]["mode"] = json!(1);
        assert!(matches!(
            from_gltf(serde_json::to_string(&lines).unwrap().as_bytes(), ()),
            Err(IoError::Unsupported { format: "glTF", .. })
        ));
    }

    #[test]
    fn scene_rejects_empty_objects_and_duplicate_names() {
        let empty = Mesh::<()>::empty();
        let cube = Mesh::<()>::cube(Real::one(), ());
        assert!(to_gltf_scene("none", &[]).is_err());
        assert!(to_gltf_scene("empty", &[GltfSceneObject::new("empty", &empty)]).is_err());
        assert!(
            to_gltf_scene(
                "duplicates",
                &[
                    GltfSceneObject::new("same", &cube),
                    GltfSceneObject::new("same", &cube),
                ],
            )
            .is_err()
        );
    }
}
