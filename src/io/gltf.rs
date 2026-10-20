//! glTF 2.0 triangle-scene import and embedded-buffer JSON export.

use crate::io::{IoError, finite_f32};
use base64::Engine;
use base64::engine::general_purpose::STANDARD as BASE64_ENGINE;
use hashbrown::{HashMap, HashSet};
use hyperlattice::{Point3, Real, Vector3};
use hypermesh::{Triangle, TriangleMesh};
use serde_json::json;
use std::borrow::Cow;
use std::cell::RefCell;
use std::io::Write;
use std::sync::Arc;

type Matrix4 = [[f64; 4]; 4];

#[derive(Clone)]
struct CachedGltf {
    positions: Arc<[Point3]>,
    triangles: Arc<[Triangle]>,
    object_name: String,
    output: String,
}

thread_local! {
    static GLTF_CACHE: RefCell<Vec<CachedGltf>> = const { RefCell::new(Vec::new()) };
}

#[derive(Clone)]
struct IndexedTriangleMesh3D {
    positions: Vec<Point3>,
    normals: Vec<Vector3>,
    faces: Vec<[(usize, usize); 3]>,
}

fn indexed_geometry(mesh: &TriangleMesh) -> Result<IndexedTriangleMesh3D, IoError> {
    if mesh.triangles.is_empty() && !mesh.positions.is_empty() {
        return Err(IoError::Geometry {
            format: "glTF",
            detail: "positions without triangles are not canonical empty geometry".into(),
        });
    }
    let buffers = mesh
        .to_exact_gpu_mesh_buffers()
        .map_err(|error| IoError::Construction {
            format: "glTF",
            source: error.into(),
        })?;
    let positions = buffers
        .vertices
        .iter()
        .map(|(position, _)| {
            Point3::new(position[0].clone(), position[1].clone(), position[2].clone())
        })
        .collect::<Vec<_>>();
    let normals = buffers
        .vertices
        .iter()
        .map(|(_, normal)| {
            Vector3::from_xyz(normal[0].clone(), normal[1].clone(), normal[2].clone())
        })
        .collect::<Vec<_>>();
    let faces = buffers
        .indices
        .as_chunks::<3>()
        .0
        .iter()
        .map(|row| {
            [
                (row[0] as usize, row[0] as usize),
                (row[1] as usize, row[1] as usize),
                (row[2] as usize, row[2] as usize),
            ]
        })
        .collect();
    Ok(IndexedTriangleMesh3D {
        positions,
        normals,
        faces,
    })
}

fn checked_u32(value: usize, limit: &'static str) -> Result<u32, IoError> {
    u32::try_from(value).map_err(|_| IoError::SizeOverflow {
        format: "glTF",
        limit,
    })
}

/// Geometry-only evidence produced while flattening one glTF scene.
#[derive(Clone, Debug)]
pub struct GltfMeshImport {
    /// Flattened native triangle mesh.
    pub mesh: hypermesh::TriangleMesh,
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
/// Scenes without mesh instances import as canonical empty geometry. Missing
/// source history remains explicit, including after subsequent certified work.
pub fn from_gltf(bytes: &[u8]) -> Result<GltfMeshImport, IoError> {
    let asset = parse_gltf(bytes)?;
    validate_node_forest(&asset)?;
    let buffers = gltf_buffers(&asset)?;
    let scene = asset
        .default_scene()
        .or_else(|| asset.scenes().next())
        .ok_or_else(|| IoError::Unsupported {
            format: "glTF",
            detail: "document has no scene to flatten".into(),
        })?;
    let scene_index = scene.index();
    let mut positions = Vec::new();
    let mut triangles = Vec::new();
    let mut mesh_node_count = 0usize;
    let mut primitive_count = 0usize;
    let mut pending = scene
        .nodes()
        .map(|node| (node, identity_matrix()))
        .collect::<Vec<_>>();
    pending.reverse();
    while let Some((node, parent)) = pending.pop() {
        let transform = import_gltf_node(
            node.clone(),
            &parent,
            &buffers,
            &mut positions,
            &mut triangles,
            &mut mesh_node_count,
            &mut primitive_count,
        )?;
        let first_child = pending.len();
        pending.extend(node.children().map(|child| (child, transform)));
        pending[first_child..].reverse();
    }

    let mut native_triangles = Vec::with_capacity(triangles.len());
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
        let _ = normal;
        native_triangles.push(Triangle::new(a, b, c));
    }
    let evidence = if positions.is_empty() {
        hypermesh::ConstructionEvidence::UNTRACKED
    } else {
        hypermesh::ConstructionEvidence::UNTRACKED.with_coordinate_projection()
    };
    let mesh =
        TriangleMesh::new(positions, native_triangles).with_inherited_construction(evidence);

    Ok(GltfMeshImport {
        mesh,
        scene_index,
        mesh_node_count,
        primitive_count,
    })
}

fn gltf_document_error(error: impl std::fmt::Display) -> IoError {
    IoError::MalformedInput(format!("glTF document validation failed: {error}"))
}

/// Bridge the parser's required Scene.nodes field to glTF's optional property,
/// then retain its native document validation. Never serialize the internal
/// empty array: a present scene.nodes array must contain at least one entry.
fn parse_gltf(bytes: &[u8]) -> Result<gltf::Gltf, IoError> {
    let (json_bytes, blob) = if bytes.starts_with(b"glTF") {
        let glb = gltf::binary::Glb::from_slice(bytes).map_err(gltf_document_error)?;
        (glb.json, glb.bin.map(Cow::into_owned))
    } else {
        (Cow::Borrowed(bytes), None)
    };
    let mut value: serde_json::Value =
        serde_json::from_slice(&json_bytes).map_err(gltf_document_error)?;
    for field in [
        "scenes",
        "nodes",
        "meshes",
        "accessors",
        "buffers",
        "bufferViews",
    ] {
        if value
            .get(field)
            .and_then(serde_json::Value::as_array)
            .is_some_and(Vec::is_empty)
        {
            return Err(gltf_document_error(format!(
                "{field} must be omitted or nonempty"
            )));
        }
    }
    if let Some(scenes) = value
        .get_mut("scenes")
        .and_then(serde_json::Value::as_array_mut)
    {
        for scene in scenes {
            if let Some(object) = scene.as_object_mut() {
                if object
                    .get("nodes")
                    .and_then(serde_json::Value::as_array)
                    .is_some_and(Vec::is_empty)
                {
                    return Err(gltf_document_error(
                        "scene.nodes must be omitted or nonempty",
                    ));
                }
                object.entry("nodes").or_insert_with(|| json!([]));
            }
        }
    }
    let root: gltf::json::Root = serde_json::from_value(value).map_err(gltf_document_error)?;
    // The dependency's primitive validation hook dereferences POSITION before
    // returning index-validation errors. Check that precondition first.
    for (mesh_index, mesh) in root.meshes.iter().enumerate() {
        if mesh.primitives.is_empty() {
            return Err(gltf_document_error(format!(
                "mesh {mesh_index} has no primitives"
            )));
        }
        for primitive in &mesh.primitives {
            for index in primitive.attributes.values() {
                if index.value() >= root.accessors.len() {
                    return Err(gltf_document_error(format!(
                        "mesh {mesh_index} references missing accessor {}",
                        index.value()
                    )));
                }
            }
        }
    }
    for accessor in &root.accessors {
        if accessor.count.0 == 0 {
            return Err(gltf_document_error("accessor count must be positive"));
        }
        platform_size(accessor.count.0)?;
        platform_size(accessor.byte_offset.map_or(0, |value| value.0))?;
        if let Some(sparse) = &accessor.sparse {
            if sparse.count.0 == 0 || sparse.count.0 > accessor.count.0 {
                return Err(gltf_document_error(
                    "sparse count must be positive and no greater than accessor count",
                ));
            }
            platform_size(sparse.count.0)?;
            platform_size(sparse.indices.byte_offset.0)?;
            platform_size(sparse.values.byte_offset.0)?;
        }
    }
    for buffer in &root.buffers {
        if buffer.byte_length.0 == 0 {
            return Err(gltf_document_error("buffer byteLength must be positive"));
        }
        platform_size(buffer.byte_length.0)?;
    }
    for view in &root.buffer_views {
        if view.byte_length.0 == 0 {
            return Err(gltf_document_error("bufferView byteLength must be positive"));
        }
        platform_size(view.byte_length.0)?;
        platform_size(view.byte_offset.map_or(0, |value| value.0))?;
    }
    Ok(gltf::Gltf {
        document: gltf::Document::from_json(root).map_err(gltf_document_error)?,
        blob,
    })
}

fn platform_size(value: u64) -> Result<usize, IoError> {
    usize::try_from(value).map_err(|_| IoError::SizeOverflow {
        format: "glTF",
        limit: "platform buffer/accessor size",
    })
}

/// Validate the document's forest without recursive calls or repeated traversal
/// of shared subtrees. The same root may appear in different scenes.
fn validate_node_forest(asset: &gltf::Gltf) -> Result<(), IoError> {
    let mut parents = vec![None; asset.nodes().len()];
    for node in asset.nodes() {
        for child in node.children() {
            if parents[child.index()].replace(node.index()).is_some() {
                return Err(gltf_document_error(format!(
                    "node {} has multiple parent references",
                    child.index()
                )));
            }
        }
    }
    let mut pending = parents
        .iter()
        .enumerate()
        .filter_map(|(index, parent)| parent.is_none().then_some(index))
        .collect::<Vec<_>>();
    let mut visited = 0;
    while let Some(index) = pending.pop() {
        visited += 1;
        pending.extend(
            asset
                .nodes()
                .nth(index)
                .expect("validated node index")
                .children()
                .map(|child| child.index()),
        );
    }
    if visited != parents.len() {
        return Err(gltf_document_error("node hierarchy contains a cycle"));
    }
    for scene in asset.scenes() {
        let mut roots = HashSet::new();
        for node in scene.nodes() {
            if parents[node.index()].is_some() || !roots.insert(node.index()) {
                return Err(gltf_document_error(format!(
                    "scene {} contains a duplicate or non-root node {}",
                    scene.index(),
                    node.index()
                )));
            }
        }
    }
    Ok(())
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

fn validate_accessor_layout(
    accessor: &gltf::Accessor<'_>,
    buffers: &[Vec<u8>],
    item_size: usize,
    alignment: usize,
) -> Result<(), IoError> {
    if let Some(view) = accessor.view() {
        validate_view_layout(
            &view,
            accessor.offset(),
            accessor.count(),
            item_size,
            alignment,
        )?;
    }
    if let Some(sparse) = accessor.sparse() {
        let indices = sparse.indices();
        let index_view = indices.view();
        let index_size = indices.index_type().size();
        let values = sparse.values();
        if index_view.stride().is_some() || values.view().stride().is_some() {
            return Err(gltf_document_error(
                "sparse buffer views must not specify byteStride",
            ));
        }
        validate_view_layout(
            &index_view,
            indices.offset(),
            sparse.count(),
            index_size,
            index_size,
        )?;
        validate_view_layout(
            &values.view(),
            values.offset(),
            sparse.count(),
            item_size,
            alignment,
        )?;
        let start = index_view.offset() + indices.offset();
        let bytes = &buffers[index_view.buffer().index()];
        let mut previous = None;
        for slot in 0..sparse.count() {
            let offset = start + slot * index_size;
            let index = match indices.index_type() {
                gltf::accessor::sparse::IndexType::U8 => u32::from(bytes[offset]),
                gltf::accessor::sparse::IndexType::U16 => u32::from(u16::from_le_bytes(
                    bytes[offset..offset + 2]
                        .try_into()
                        .expect("validated sparse index width"),
                )),
                gltf::accessor::sparse::IndexType::U32 => u32::from_le_bytes(
                    bytes[offset..offset + 4]
                        .try_into()
                        .expect("validated sparse index width"),
                ),
            };
            if index as usize >= accessor.count() || previous.is_some_and(|last| index <= last)
            {
                return Err(gltf_document_error(
                    "sparse indices must increase strictly within accessor bounds",
                ));
            }
            previous = Some(index);
        }
    }
    Ok(())
}

fn validate_view_layout(
    view: &gltf::buffer::View<'_>,
    offset: usize,
    count: usize,
    item_size: usize,
    alignment: usize,
) -> Result<(), IoError> {
    let overflow = || IoError::SizeOverflow {
        format: "glTF",
        limit: "buffer-view/accessor byte range",
    };
    let view_end = view
        .offset()
        .checked_add(view.length())
        .ok_or_else(overflow)?;
    let stride = view.stride().unwrap_or(item_size);
    let end = count
        .checked_sub(1)
        .and_then(|last| last.checked_mul(stride))
        .and_then(|last| last.checked_add(offset))
        .and_then(|last| last.checked_add(item_size))
        .ok_or_else(overflow)?;
    let absolute_offset = view.offset().checked_add(offset).ok_or_else(overflow)?;
    if view_end > view.buffer().length()
        || end > view.length()
        || stride < item_size
        || !stride.is_multiple_of(alignment)
        || !offset.is_multiple_of(alignment)
        || !absolute_offset.is_multiple_of(alignment)
    {
        return Err(gltf_document_error(format!(
            "buffer view {} does not contain the aligned accessor range",
            view.index()
        )));
    }
    Ok(())
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
) -> Result<Matrix4, IoError> {
    if node.skin().is_some() {
        return Err(IoError::Unsupported {
            format: "glTF",
            detail: format!("node {} uses skinning", node.index()),
        });
    }
    let local = f32_matrix_to_f64(node.transform().matrix())?;
    let transform = multiply_matrix(parent, &local);
    if !transform.iter().flatten().all(|value| value.is_finite())
        || [
            transform[0][3],
            transform[1][3],
            transform[2][3],
            transform[3][3],
        ] != [0.0, 0.0, 0.0, 1.0]
    {
        return Err(IoError::Unsupported {
            format: "glTF",
            detail: "node transform is non-finite or non-affine".into(),
        });
    }
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
            let position_accessor =
                primitive.get(&gltf::Semantic::Positions).ok_or_else(|| {
                    gltf_document_error("triangle primitive has no POSITION attribute")
                })?;
            if position_accessor.dimensions() != gltf::accessor::Dimensions::Vec3 {
                return Err(gltf_document_error("POSITION accessor must have VEC3 type"));
            }
            if position_accessor.data_type() != gltf::accessor::DataType::F32 {
                return Err(IoError::Unsupported {
                    format: "glTF",
                    detail: "only FLOAT POSITION accessors are supported".into(),
                });
            }
            if position_accessor.normalized() {
                return Err(gltf_document_error(
                    "FLOAT POSITION accessors must not be normalized",
                ));
            }
            validate_accessor_layout(&position_accessor, buffers, 12, 4)?;
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
            let indices = if let Some(accessor) = primitive.indices() {
                if accessor.dimensions() != gltf::accessor::Dimensions::Scalar
                    || !matches!(
                        accessor.data_type(),
                        gltf::accessor::DataType::U8
                            | gltf::accessor::DataType::U16
                            | gltf::accessor::DataType::U32
                    )
                    || accessor.normalized()
                {
                    return Err(gltf_document_error(
                        "triangle indices must be non-normalized unsigned SCALAR values",
                    ));
                }
                if accessor.view().is_some_and(|view| view.stride().is_some()) {
                    return Err(gltf_document_error(
                        "index buffer view must not specify byteStride",
                    ));
                }
                let size = accessor.data_type().size();
                validate_accessor_layout(&accessor, buffers, size, size)?;
                reader
                    .read_indices()
                    .ok_or_else(|| {
                        gltf_document_error("triangle index accessor cannot be read")
                    })?
                    .into_u32()
                    .collect::<Vec<_>>()
            } else {
                (0..vertex_count)
                    .map_while(|index| u32::try_from(index).ok())
                    .collect()
            };
            if indices.len() != vertex_count && primitive.indices().is_none() {
                return Err(IoError::SizeOverflow {
                    format: "glTF",
                    limit: "u32 implicit vertex index",
                });
            }
            let (chunks, remainder) = indices.as_chunks::<3>();
            if !remainder.is_empty() {
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
    Ok(transform)
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

/// Export native triangles with binary32 position and normal attributes.
///
/// Native index and normal failures retain their category and cause through
/// [`IoError::Construction`]. Only successful output is cached, allowing retry
/// after an undecided normal gains stronger scalar evidence.
/// Canonical empty geometry exports as a named node without a mesh. Positions
/// without triangles remain invalid input, rather than becoming an empty node.
pub fn to_gltf(mesh: &TriangleMesh, object_name: &str) -> Result<String, IoError> {
    if let Some(output) = GLTF_CACHE.with_borrow(|entries| {
        entries
            .iter()
            .find(|entry| {
                Arc::ptr_eq(&entry.positions, &mesh.positions)
                    && Arc::ptr_eq(&entry.triangles, &mesh.triangles)
                    && entry.object_name == object_name
            })
            .map(|entry| entry.output.clone())
    }) {
        return Ok(output);
    }
    let output = to_gltf_scene(object_name, &[GltfSceneObject::new(object_name, mesh)?])?;
    GLTF_CACHE.with_borrow_mut(|entries| {
        const CAPACITY: usize = 4;
        if entries.len() == CAPACITY {
            entries.remove(0);
        }
        entries.push(CachedGltf {
            positions: Arc::clone(&mesh.positions),
            triangles: Arc::clone(&mesh.triangles),
            object_name: object_name.to_owned(),
            output: output.clone(),
        });
    });
    Ok(output)
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
    /// Native failures use the same typed construction cause as [`to_gltf`].
    pub fn new(name: impl Into<String>, mesh: &TriangleMesh) -> Result<Self, IoError> {
        Ok(Self {
            name: name.into(),
            indexed: indexed_geometry(mesh)?,
        })
    }
}

/// Serializes multiple independently named geometry objects into one glTF 2.0
/// scene with an embedded binary buffer when geometry is present. Empty objects
/// retain their names as meshless nodes; a scene with no objects omits its nodes.
pub fn to_gltf_scene(
    scene_name: &str,
    objects: &[GltfSceneObject],
) -> Result<String, IoError> {
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
            let node_index = checked_u32(nodes.len(), "u32 node index")?;
            nodes.push(json!({"name": object.name}));
            scene_nodes.push(node_index);
            continue;
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

    let mut document = json!({
        "asset": {"version": "2.0", "generator": "csgrs"},
        "scenes": [{"name": scene_name}],
        "scene": 0
    });
    if !nodes.is_empty() {
        document["nodes"] = json!(nodes);
        document["scenes"][0]["nodes"] = json!(scene_nodes);
    }
    if !buffer.is_empty() {
        let byte_length = checked_u32(buffer.len(), "u32 buffer length")?;
        let uri = format!(
            "data:application/octet-stream;base64,{}",
            BASE64_ENGINE.encode(buffer)
        );
        document["buffers"] = json!([{"byteLength": byte_length, "uri": uri}]);
        document["bufferViews"] = json!(buffer_views);
        document["accessors"] = json!(accessors);
        document["meshes"] = json!(meshes);
    }
    serde_json::to_string_pretty(&document).map_err(|error| IoError::Geometry {
        format: "glTF",
        detail: error.to_string(),
    })
}

pub fn write_gltf<W: Write>(
    mesh: &TriangleMesh,
    writer: &mut W,
    object_name: &str,
) -> Result<(), IoError> {
    writer.write_all(to_gltf(mesh, object_name)?.as_bytes())?;
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
