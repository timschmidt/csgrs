#![doc = " OBJ file format support"]
#![doc = ""]
#![doc = " This module provides import and export functionality for Wavefront OBJ files,"]
#![doc = " a widely-supported 3D file format used by many modeling and rendering applications."]

use crate::mesh::Mesh;
use crate::mesh::Polygon;
use crate::mesh::polygon::{
    LazySubdivisionVertexPool, fresh_plane_id, triangulate_indexed_positions_into,
};
#[cfg(feature = "sketch")]
use crate::sketch::Profile;
use crate::triangulated::IndexedTriangulated3D;
use crate::vertex::{Vertex, reserve_position_ids};
use hashbrown::HashMap;
use hyperlattice::{Aabb, Point3, Real, Vector3};
use std::fmt::Debug;
use std::io::{BufRead, Write};
use std::sync::Arc;

use super::{IoError, finite_f64, single_line_metadata};

type ObjFace = Vec<(usize, usize)>;
type ObjBuffers = (Vec<Point3>, Vec<Vector3>, Vec<ObjFace>);

#[derive(Clone, Copy, Debug, Eq, Hash, PartialEq)]
struct ObjPositionKey([Option<u64>; 3]);

impl ObjPositionKey {
    fn new(position: &Point3) -> Self {
        fn coordinate_key(value: &Real) -> Option<u64> {
            value.to_f64_lossy().map(|value| {
                if value == 0.0 {
                    0.0_f64.to_bits()
                } else {
                    value.to_bits()
                }
            })
        }

        Self([
            coordinate_key(&position.x),
            coordinate_key(&position.y),
            coordinate_key(&position.z),
        ])
    }
}

#[derive(Debug)]
struct ObjTriangle {
    corners: [(usize, usize); 3],
    plane_id: u64,
}

fn checked_obj_index(index: usize, limit: &'static str) -> Result<usize, IoError> {
    index.checked_add(1).ok_or(IoError::SizeOverflow {
        format: "OBJ",
        limit,
    })
}

fn invalid_obj_data(message: impl Into<String>) -> IoError {
    IoError::MalformedInput(format!("OBJ: {}", message.into()))
}

fn invalid_obj_data_at_line(line: usize, message: impl Into<String>) -> IoError {
    invalid_obj_data(format!("line {line}: {}", message.into()))
}

fn obj_line_error(line: usize, error: IoError) -> IoError {
    invalid_obj_data_at_line(line, error.to_string())
}

fn parse_obj_real(text: &str) -> Result<Real, hyperlattice::Problem> {
    const MAX_EXPANDED_DECIMAL_LEN: usize = 1_000_000;

    if !text.contains(['e', 'E']) {
        return text.parse();
    }

    let (mantissa, exponent) = text
        .split_once(['e', 'E'])
        .ok_or(hyperlattice::Problem::BadDecimal)?;
    let exponent = exponent
        .parse::<i64>()
        .map_err(|_| hyperlattice::Problem::BadDecimal)?;
    let (sign, magnitude) = if let Some(magnitude) = mantissa.strip_prefix('-') {
        ("-", magnitude)
    } else if let Some(magnitude) = mantissa.strip_prefix('+') {
        ("", magnitude)
    } else {
        ("", mantissa)
    };
    let (whole, fraction) = magnitude
        .split_once('.')
        .map_or((magnitude, ""), |parts| parts);
    if (whole.is_empty() && fraction.is_empty())
        || !whole.bytes().all(|byte| byte.is_ascii_digit())
        || !fraction.bytes().all(|byte| byte.is_ascii_digit())
    {
        return Err(hyperlattice::Problem::BadDecimal);
    }

    let mut digits = String::with_capacity(whole.len().saturating_add(fraction.len()));
    digits.push_str(whole);
    digits.push_str(fraction);
    let decimal_position = i64::try_from(whole.len())
        .ok()
        .and_then(|whole_len| whole_len.checked_add(exponent))
        .ok_or(hyperlattice::Problem::OutOfRange)?;
    let digits_len =
        i64::try_from(digits.len()).map_err(|_| hyperlattice::Problem::OutOfRange)?;

    let mut normalized = String::new();
    normalized.push_str(sign);
    if decimal_position <= 0 {
        normalized.push_str("0.");
        let zero_count = usize::try_from(
            decimal_position
                .checked_neg()
                .ok_or(hyperlattice::Problem::OutOfRange)?,
        )
        .map_err(|_| hyperlattice::Problem::OutOfRange)?;
        if zero_count.saturating_add(digits.len()) > MAX_EXPANDED_DECIMAL_LEN {
            return Err(hyperlattice::Problem::OutOfRange);
        }
        normalized.extend(std::iter::repeat_n('0', zero_count));
        normalized.push_str(&digits);
    } else if decimal_position >= digits_len {
        normalized.push_str(&digits);
        let zero_count = usize::try_from(decimal_position - digits_len)
            .map_err(|_| hyperlattice::Problem::OutOfRange)?;
        if zero_count.saturating_add(digits.len()) > MAX_EXPANDED_DECIMAL_LEN {
            return Err(hyperlattice::Problem::OutOfRange);
        }
        normalized.extend(std::iter::repeat_n('0', zero_count));
    } else {
        let decimal_position = usize::try_from(decimal_position)
            .map_err(|_| hyperlattice::Problem::OutOfRange)?;
        normalized.push_str(&digits[..decimal_position]);
        normalized.push('.');
        normalized.push_str(&digits[decimal_position..]);
    }
    normalized.parse()
}

fn parse_obj_normal(coordinates: [&str; 3], line_number: usize) -> Result<Vector3, IoError> {
    let mut values = [0.0_f64; 3];
    for (axis, (text, label)) in coordinates.into_iter().zip(["x", "y", "z"]).enumerate() {
        values[axis] = text.parse::<f64>().map_err(|error| {
            invalid_obj_data_at_line(
                line_number,
                format!("Invalid normal {label} coordinate: {error}"),
            )
        })?;
    }
    let length = values[0].hypot(values[1]).hypot(values[2]);
    if !values.iter().all(|value| value.is_finite()) || !length.is_finite() || length == 0.0 {
        return Err(invalid_obj_data_at_line(
            line_number,
            "Invalid normal coordinate: OBJ normals must be finite and non-zero",
        ));
    }

    let normalized = values.map(|value| value / length);
    Ok(Vector3::from_xyz(
        Real::try_from(normalized[0]).map_err(|error| {
            invalid_obj_data_at_line(
                line_number,
                format!("Invalid normal x coordinate: {error}"),
            )
        })?,
        Real::try_from(normalized[1]).map_err(|error| {
            invalid_obj_data_at_line(
                line_number,
                format!("Invalid normal y coordinate: {error}"),
            )
        })?,
        Real::try_from(normalized[2]).map_err(|error| {
            invalid_obj_data_at_line(
                line_number,
                format!("Invalid normal z coordinate: {error}"),
            )
        })?,
    ))
}

fn include_obj_coordinate(value: &Real, minimum: &mut Real, maximum: &mut Real) {
    if matches!(
        hyperlimit::compare_reals(value, minimum).value(),
        Some(std::cmp::Ordering::Less)
    ) {
        *minimum = value.clone();
    }
    if matches!(
        hyperlimit::compare_reals(value, maximum).value(),
        Some(std::cmp::Ordering::Greater)
    ) {
        *maximum = value.clone();
    }
}

fn obj_bounds(positions: &[Point3]) -> Option<Aabb> {
    let first = positions.first()?.clone();
    let mut minimum = first.clone();
    let mut maximum = first;
    for position in &positions[1..] {
        include_obj_coordinate(&position.x, &mut minimum.x, &mut maximum.x);
        include_obj_coordinate(&position.y, &mut minimum.y, &mut maximum.y);
        include_obj_coordinate(&position.z, &mut minimum.z, &mut maximum.z);
    }
    Some(Aabb::new(minimum, maximum))
}

fn build_obj_buffers<T: IndexedTriangulated3D>(shape: &T) -> Result<ObjBuffers, IoError> {
    let indexed = shape.indexed_triangles();
    for face in &indexed.faces {
        for &(position, normal) in face {
            if position >= indexed.positions.len() || normal >= indexed.normals.len() {
                return Err(IoError::Geometry {
                    format: "OBJ",
                    detail: "indexed triangle references missing vertex data".into(),
                });
            }
            checked_obj_index(position, "one-based vertex index")?;
            checked_obj_index(normal, "one-based normal index")?;
        }
    }
    let faces = indexed.faces.into_iter().map(Vec::from).collect();
    Ok((indexed.positions, indexed.normals, faces))
}

#[doc = " Export any `Triangulated3D` shape to OBJ format as a string"]
#[doc = ""]
#[doc = " Creates a Wavefront OBJ file containing all triangles from `shape`."]
#[doc = ""]
#[doc = " # Arguments"]
#[doc = " * `object_name` - Name for the object in the OBJ file"]
pub fn to_obj<T: IndexedTriangulated3D>(
    shape: &T,
    object_name: &str,
) -> Result<String, IoError> {
    let object_name = single_line_metadata(object_name, "OBJ", "object name")?;
    if object_name.contains('#') {
        return Err(IoError::InvalidMetadata {
            format: "OBJ",
            field: "object name",
        });
    }
    let (vertices, normals, faces) = build_obj_buffers(shape)?;

    let mut obj_content = String::new();
    obj_content.push_str("# Generated by csgrs library\n");
    obj_content.push_str(&format!("# Object: {object_name}\n"));
    obj_content.push_str("# https://github.com/timschmidt/csgrs\n\n");
    obj_content.push_str(&format!("o {object_name}\n\n"));

    for vertex in &vertices {
        obj_content.push_str(&format!(
            "v {:.17} {:.17} {:.17}\n",
            finite_f64(&vertex.x, "OBJ", "vertex x")?,
            finite_f64(&vertex.y, "OBJ", "vertex y")?,
            finite_f64(&vertex.z, "OBJ", "vertex z")?
        ));
    }
    obj_content.push('\n');

    for normal in &normals {
        obj_content.push_str(&format!(
            "vn {:.17} {:.17} {:.17}\n",
            finite_f64(&normal.0[0], "OBJ", "normal x")?,
            finite_f64(&normal.0[1], "OBJ", "normal y")?,
            finite_f64(&normal.0[2], "OBJ", "normal z")?
        ));
    }
    obj_content.push('\n');

    for face in &faces {
        #[allow(clippy::single_char_add_str)]
        obj_content.push_str("f");
        for (vertex_idx, normal_idx) in face {
            obj_content.push_str(&format!(
                " {}//{}",
                checked_obj_index(*vertex_idx, "one-based vertex index")?,
                checked_obj_index(*normal_idx, "one-based normal index")?
            ));
        }
        obj_content.push('\n');
    }

    Ok(obj_content)
}

#[doc = " Export any `Triangulated3D` shape to an OBJ file"]
#[doc = ""]
#[doc = " # Arguments"]
#[doc = " * `writer` - Where to write the OBJ data"]
#[doc = " * `object_name` - Name for the object in the OBJ file"]
pub fn write_obj<T: IndexedTriangulated3D, W: Write>(
    shape: &T,
    writer: &mut W,
    object_name: &str,
) -> Result<(), IoError> {
    let obj_content = to_obj(shape, object_name)?;
    writer.write_all(obj_content.as_bytes())?;
    Ok(())
}

impl<M: Clone + Debug + Send + Sync> Mesh<M> {
    #[doc = " Export this Mesh to OBJ format as a string"]
    pub fn to_obj(&self, object_name: &str) -> Result<String, IoError> {
        self::to_obj(self, object_name)
    }

    #[doc = " Export this Mesh to an OBJ file"]
    pub fn write_obj<W: Write>(
        &self,
        writer: &mut W,
        object_name: &str,
    ) -> Result<(), IoError> {
        self::write_obj(self, writer, object_name)
    }

    #[doc = " Import a Mesh from OBJ file data"]
    #[doc = ""]
    #[doc = " # Arguments"]
    #[doc = " * `reader` - Source of OBJ data"]
    #[doc = " * `metadata` - Optional metadata to attach to all polygons"]
    ///
    /// Primitive OBJ coordinates are accepted only after they promote through
    /// the shared hyperlattice boundary adapters. Vertex normals cross the
    /// finite file-format boundary as binary64 values and are checked-normalized
    /// there before promotion, so non-finite or zero normals fail as malformed
    /// OBJ data rather than being silently sanitized by [`Vertex::new`]. This
    /// keeps exact positions aligned with Yap, "Towards
    /// Exact Geometric Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>), while retaining the
    /// Wavefront OBJ `v`/`vn` finite boundary format.
    pub fn from_obj<R: BufRead>(reader: R, metadata: M) -> Result<Mesh<M>, IoError> {
        let mut vertices = Vec::new();
        let mut normals = Vec::new();
        let mut triangles = Vec::<ObjTriangle>::new();
        let mut face_corners = Vec::new();
        let mut face_position_indices = Vec::new();
        let mut face_normal_indices = Vec::new();
        let mut face_triangles = Vec::new();
        let mut face_projected = Vec::new();
        let mut triangles_nondegenerate = true;
        let mut reader = reader;
        let mut line_buffer = String::new();
        let mut line_number = 0usize;

        loop {
            line_buffer.clear();
            if reader.read_line(&mut line_buffer)? == 0 {
                break;
            }
            line_number += 1;
            let line = line_buffer.as_str();
            let line = line.split('#').next().unwrap_or("").trim();
            if line.is_empty() {
                continue;
            }
            let mut parts = line.split_whitespace();
            let Some(directive) = parts.next() else {
                continue;
            };
            match directive {
                "v" => {
                    let coordinates = [parts.next(), parts.next(), parts.next()];
                    let [Some(x), Some(y), Some(z)] = coordinates else {
                        return Err(invalid_obj_data_at_line(
                            line_number,
                            "vertex line needs three coordinates",
                        ));
                    };
                    let x: Real = parse_obj_real(x).map_err(|e| {
                        invalid_obj_data_at_line(
                            line_number,
                            format!("Invalid vertex x coordinate: {e}"),
                        )
                    })?;
                    let y: Real = parse_obj_real(y).map_err(|e| {
                        invalid_obj_data_at_line(
                            line_number,
                            format!("Invalid vertex y coordinate: {e}"),
                        )
                    })?;
                    let z: Real = parse_obj_real(z).map_err(|e| {
                        invalid_obj_data_at_line(
                            line_number,
                            format!("Invalid vertex z coordinate: {e}"),
                        )
                    })?;
                    let weight = parts.next();
                    if parts.next().is_some() {
                        return Err(IoError::Unsupported {
                            format: "OBJ",
                            detail: format!(
                                "line {line_number}: vertex color extensions are not supported"
                            ),
                        });
                    }
                    let point = if let Some(weight) = weight {
                        let weight: Real = parse_obj_real(weight).map_err(|error| {
                            IoError::MalformedInput(format!(
                                "OBJ line {line_number}: invalid homogeneous weight: {error}"
                            ))
                        })?;
                        Point3::new(
                            (x / &weight).map_err(|error| IoError::MalformedInput(format!(
                                "OBJ line {line_number}: invalid homogeneous weight: {error}"
                            )))?,
                            (y / &weight).map_err(|error| IoError::MalformedInput(format!(
                                "OBJ line {line_number}: invalid homogeneous weight: {error}"
                            )))?,
                            (z / &weight).map_err(|error| IoError::MalformedInput(format!(
                                "OBJ line {line_number}: invalid homogeneous weight: {error}"
                            )))?,
                        )
                    } else {
                        Point3::new(x, y, z)
                    };
                    vertices.push(point);
                },
                "vn" => {
                    let coordinates = [parts.next(), parts.next(), parts.next()];
                    let [Some(x), Some(y), Some(z)] = coordinates else {
                        return Err(invalid_obj_data_at_line(
                            line_number,
                            "normal line needs three coordinates",
                        ));
                    };
                    if parts.next().is_some() {
                        return Err(invalid_obj_data_at_line(
                            line_number,
                            "normal line needs three coordinates",
                        ));
                    }
                    normals.push(parse_obj_normal([x, y, z], line_number)?);
                },
                "f" => {
                    Self::parse_obj_face(
                        parts,
                        vertices.len(),
                        normals.len(),
                        line_number,
                        &mut face_corners,
                    )?;
                    let corners = &face_corners;
                    if corners.len() < 3 {
                        return Err(invalid_obj_data_at_line(
                            line_number,
                            "face line needs at least three vertices",
                        ));
                    }
                    let has_normal = corners[0].1.is_some();
                    if corners
                        .iter()
                        .any(|(_, normal)| normal.is_some() != has_normal)
                    {
                        return Err(invalid_obj_data_at_line(
                            line_number,
                            "face mixes vertices with and without normals",
                        ));
                    }
                    let plane_id = if has_normal {
                        face_position_indices.clear();
                        face_position_indices
                            .extend(corners.iter().map(|&(position, _)| position));
                        face_normal_indices.clear();
                        face_normal_indices.extend(corners.iter().map(|(_, normal)| {
                            normal.expect("face normal presence was checked")
                        }));
                        triangulate_indexed_positions_into(
                            &vertices,
                            &face_position_indices,
                            &mut face_triangles,
                            &mut face_projected,
                        );
                        fresh_plane_id()
                    } else {
                        let face_vertices = corners
                            .iter()
                            .map(|&(position, _)| {
                                Vertex::new(vertices[position].clone(), Vector3::z())
                            })
                            .collect();
                        let mut face = Polygon::new(face_vertices, metadata.clone());
                        face.set_new_normal();
                        let normal = face.vertices[0].normal.clone();
                        let normal_index = normals.len();
                        normals.push(normal);
                        face_normal_indices.clear();
                        face_normal_indices.resize(corners.len(), normal_index);
                        face.triangulate_indices_into(
                            &mut face_triangles,
                            &mut face_projected,
                        );
                        face.plane_id
                    };
                    if face_triangles.is_empty() {
                        return Err(invalid_obj_data_at_line(
                            line_number,
                            "face could not be triangulated",
                        ));
                    }
                    // Exact triangulation certifies emitted triangles for
                    // polygonal faces. A three-corner face bypasses that
                    // kernel, so certify only that direct carrier here.
                    if triangles_nondegenerate && corners.len() == 3 {
                        let [a, b, c] = [
                            &vertices[corners[0].0],
                            &vertices[corners[1].0],
                            &vertices[corners[2].0],
                        ];
                        triangles_nondegenerate =
                            ::hypermesh::Plane::points_are_nondegenerate(a, b, c);
                    }
                    for &triangle in &face_triangles {
                        triangles.push(ObjTriangle {
                            corners: triangle.map(|corner| {
                                (corners[corner].0, face_normal_indices[corner])
                            }),
                            plane_id,
                        });
                    }
                },
                "o" | "g" | "s" | "usemtl" | "mtllib" => {},
                "vt" => {
                    return Err(IoError::Unsupported {
                        format: "OBJ",
                        detail: format!(
                            "line {line_number}: texture coordinates are not supported"
                        ),
                    });
                },
                directive => {
                    return Err(IoError::Unsupported {
                        format: "OBJ",
                        detail: format!("line {line_number}: directive {directive:?}"),
                    });
                },
            }
        }

        if triangles.is_empty() {
            return Err(IoError::MalformedInput(
                "OBJ input contains no polygonal faces".into(),
            ));
        }

        // Canonicalize exact position rows once while their source indices are
        // still available. The retained slots then serve transforms,
        // connectivity, and hypermesh adapters without re-hashing every
        // triangle corner.
        let mut source_position_slots = vec![None; vertices.len()];
        let mut position_bucket_heads = HashMap::<ObjPositionKey, usize>::new();
        let mut position_bucket_next = Vec::<Option<usize>>::new();
        let mut canonical_source_positions = Vec::<usize>::new();
        for triangle in &mut triangles {
            for (source_position, _) in &mut triangle.corners {
                let slot = if let Some(slot) = source_position_slots[*source_position] {
                    slot
                } else {
                    let position = &vertices[*source_position];
                    let key = ObjPositionKey::new(position);
                    let mut candidate = position_bucket_heads.get(&key).copied();
                    let mut slot = None;
                    while let Some(current) = candidate {
                        if vertices[canonical_source_positions[current]] == *position {
                            slot = Some(current);
                            break;
                        }
                        candidate = position_bucket_next[current];
                    }
                    let slot = slot.unwrap_or_else(|| {
                        let slot = canonical_source_positions.len();
                        canonical_source_positions.push(*source_position);
                        position_bucket_next.push(position_bucket_heads.insert(key, slot));
                        slot
                    });
                    source_position_slots[*source_position] = Some(slot);
                    slot
                };
                *source_position = slot;
            }
        }

        source_position_slots.fill(None);
        for (slot, &source_position) in canonical_source_positions.iter().enumerate() {
            source_position_slots[source_position] = Some(slot);
        }
        let mut source_position = 0;
        let mut final_slot = 0;
        let mut positions_remapped = false;
        vertices.retain(|_| {
            let retained = if let Some(slot) = source_position_slots[source_position] {
                positions_remapped |= slot != final_slot;
                canonical_source_positions[slot] = final_slot;
                final_slot += 1;
                true
            } else {
                false
            };
            source_position += 1;
            retained
        });
        debug_assert_eq!(final_slot, canonical_source_positions.len());
        if positions_remapped {
            for triangle in &mut triangles {
                for (position, _) in &mut triangle.corners {
                    *position = canonical_source_positions[*position];
                }
            }
        }
        let positions = vertices;
        drop(source_position_slots);
        drop(position_bucket_heads);
        drop(position_bucket_next);
        drop(canonical_source_positions);

        let mut position_normals = vec![None; positions.len()];
        let mut normals_match_positions = true;
        for triangle in &triangles {
            for &(position, normal) in &triangle.corners {
                match position_normals[position] {
                    Some(existing) if existing != normal => normals_match_positions = false,
                    Some(_) => {},
                    None => position_normals[position] = Some(normal),
                }
            }
        }

        let identity_count = positions.len().checked_mul(4).ok_or(IoError::SizeOverflow {
            format: "OBJ",
            limit: "vertex identity count",
        })?;
        let first_vertex_identity = reserve_position_ids(identity_count);
        let mut pool_triangles = Vec::with_capacity(triangles.len());
        let pool_vertices = if normals_match_positions {
            pool_triangles.extend(
                triangles
                    .iter()
                    .map(|triangle| triangle.corners.map(|(position, _)| position)),
            );
            positions
                .iter()
                .cloned()
                .zip(position_normals)
                .enumerate()
                .map(|(slot, (position, normal))| {
                    Vertex::new_with_reserved_identity(
                        position,
                        normals[normal.expect("used OBJ position has a normal")].clone(),
                        first_vertex_identity,
                        slot,
                    )
                })
                .collect()
        } else {
            drop(position_normals);
            let base_vertices = positions
                .iter()
                .cloned()
                .enumerate()
                .map(|(slot, position)| {
                    Vertex::new_with_reserved_identity(
                        position,
                        Vector3::z(),
                        first_vertex_identity,
                        slot,
                    )
                })
                .collect::<Vec<_>>();
            let mut pool_vertices = Vec::with_capacity(positions.len());
            let mut vertex_slots =
                HashMap::<(usize, usize), usize>::with_capacity(positions.len());
            for triangle in &triangles {
                pool_triangles.push(triangle.corners.map(|(position, normal)| {
                    *vertex_slots.entry((position, normal)).or_insert_with(|| {
                        let slot = pool_vertices.len();
                        let mut vertex = base_vertices[position].clone();
                        vertex.normal = normals[normal].clone();
                        pool_vertices.push(vertex);
                        slot
                    })
                }));
            }
            pool_vertices
        };
        drop(normals);

        let triangle_count = triangles.len();
        let vertex_pool = Arc::new(LazySubdivisionVertexPool::new(
            pool_vertices,
            Vec::new(),
            triangle_count,
            0,
        ));
        let polygons = pool_triangles
            .into_iter()
            .zip(&triangles)
            .enumerate()
            .map(|(triangle_slot, (indices, triangle))| {
                Polygon::from_lazy_subdivision_triangle(
                    Arc::clone(&vertex_pool),
                    triangle_slot,
                    indices,
                    metadata.clone(),
                    triangle.plane_id,
                )
            })
            .collect();
        let mesh = Mesh::from_polygons_with_topology(
            polygons,
            (triangle_count, triangle_count.saturating_mul(3), true),
        );
        let corner_position_slots = triangles
            .iter()
            .flat_map(|triangle| triangle.corners.map(|(position, _)| position))
            .collect::<Vec<_>>();
        let position_f64 = positions
            .iter()
            .map(|position| {
                Some([
                    position.x.to_f64_lossy()?,
                    position.y.to_f64_lossy()?,
                    position.z.to_f64_lossy()?,
                ])
            })
            .collect::<Option<Vec<_>>>()
            .map(Arc::new);
        mesh.retain_shared_position_transform_layout(
            corner_position_slots,
            positions.len(),
            position_f64,
            normals_match_positions.then(|| Arc::clone(&vertex_pool)),
            None,
            normals_match_positions,
        );

        let mut adjacent = vec![false; positions.len()];
        for triangle in &triangles {
            let positions = triangle.corners.map(|(position, _)| position);
            for [left, right] in [
                [positions[0], positions[1]],
                [positions[1], positions[2]],
                [positions[2], positions[0]],
            ] {
                if left != right {
                    adjacent[left] = true;
                    adjacent[right] = true;
                }
            }
        }
        mesh.polygons.retain_connectivity_counts((
            positions.len(),
            adjacent.into_iter().filter(|x| *x).count(),
        ));
        mesh.polygons
            .retain_nondegenerate_triangles_fact(triangles_nondegenerate);
        if let Some(bounds) = obj_bounds(&positions) {
            let _ = mesh.bounding_box.set(bounds.clone());
            mesh.polygons.retain_axis_aligned_box_fact(bounds);
        }
        Ok(mesh)
    }

    fn parse_obj_face<'a>(
        face_parts: impl Iterator<Item = &'a str>,
        vertex_count: usize,
        normal_count: usize,
        line_number: usize,
        face_vertices: &mut Vec<(usize, Option<usize>)>,
    ) -> Result<(), IoError> {
        face_vertices.clear();
        for part in face_parts {
            let mut indices = part.split('/');
            let position = indices.next().unwrap_or("");
            let texture = indices.next();
            let normal = indices.next();
            if indices.next().is_some() {
                return Err(IoError::MalformedInput(format!(
                    "OBJ line {line_number}: face token has too many index fields"
                )));
            }
            if texture.is_some_and(|index| !index.is_empty()) {
                return Err(IoError::Unsupported {
                    format: "OBJ",
                    detail: format!(
                        "line {line_number}: texture-coordinate indices are not supported"
                    ),
                });
            }
            let vertex_idx = parse_obj_index(position, "vertex", vertex_count)
                .map_err(|error| obj_line_error(line_number, error))?;
            let normal_idx = normal
                .filter(|index| !index.is_empty())
                .map(|index| {
                    parse_obj_index(index, "normal", normal_count)
                        .map_err(|error| obj_line_error(line_number, error))
                })
                .transpose()?;
            face_vertices.push((vertex_idx, normal_idx));
        }
        Ok(())
    }
}

fn parse_obj_index(raw: &str, label: &str, value_count: usize) -> Result<usize, IoError> {
    if raw.is_empty() {
        return Err(invalid_obj_data(format!(
            "face token is missing a {label} index"
        )));
    }

    let index = raw
        .parse::<isize>()
        .map_err(|error| invalid_obj_data(format!("Invalid {label} index: {error}")))?;

    let resolved = if index > 0 {
        usize::try_from(index).map_err(|_| invalid_obj_data("face index exceeds usize"))? - 1
    } else if index < 0 {
        value_count.checked_sub(index.unsigned_abs()).ok_or_else(|| {
            invalid_obj_data(format!("negative {label} index is out of range"))
        })?
    } else {
        return Err(invalid_obj_data(format!(
            "Invalid {label} index: OBJ indices are one-based"
        )));
    };

    if resolved >= value_count {
        return Err(invalid_obj_data(format!("{label} index is out of range")));
    }

    Ok(resolved)
}

#[cfg(feature = "sketch")]
impl Profile {
    #[doc = " Export this Profile to OBJ format as a string"]
    pub fn to_obj(&self, object_name: &str) -> Result<String, IoError> {
        self::to_obj(self, object_name)
    }

    #[doc = " Export this Profile to an OBJ file"]
    pub fn write_obj<W: Write>(
        &self,
        writer: &mut W,
        object_name: &str,
    ) -> Result<(), IoError> {
        self::write_obj(self, writer, object_name)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::io::test_support::InvalidIndexed;
    use std::io::Cursor;

    #[test]
    fn public_writer_matches_string_serializer() {
        let mesh = Mesh::<()>::cube(Real::one(), ());
        let expected = to_obj(&mesh, "cube").unwrap();
        let mut written = Vec::new();
        crate::io::obj::write_obj(&mesh, &mut written, "cube").unwrap();
        assert_eq!(written, expected.as_bytes());
    }

    #[test]
    fn from_obj_accepts_relative_negative_face_indices() {
        let obj = "\
v 0 0 0
v 1 0 0
v 0 1 0
f -3 -2 -1
";

        let mesh = Mesh::<()>::from_obj(Cursor::new(obj), ()).unwrap();

        assert_eq!(mesh.polygons.len(), 1);
        assert_eq!(mesh.polygons[0].vertices.len(), 3);
    }

    #[test]
    fn from_obj_accepts_scientific_notation_without_losing_exactness() {
        let obj = "v 7.78437e-005 0 0\nv 0 +.1E+1 0\nv 0 0 2e0\nf 1 2 3\n";

        let mesh = Mesh::<()>::from_obj(Cursor::new(obj), ()).unwrap();

        let expected = (Real::from(778_437_u64) / Real::from(10_000_000_000_u64)).unwrap();
        assert_eq!(mesh.polygons[0].vertices[0].position.x, expected);
        assert_eq!(mesh.polygons[0].vertices[1].position.y, Real::one());
        assert_eq!(mesh.polygons[0].vertices[2].position.z, Real::from(2_u8));
    }

    #[test]
    fn from_obj_keeps_exact_positions_that_share_a_binary64_bucket_distinct() {
        let obj = "\
v 9007199254740992 0 0
v 0 1 0
v 0 0 1
v 9007199254740993 0 0
v 0 -1 0
v 0 0 -1
f 1 2 3
f 4 5 6
";

        let mesh = Mesh::<()>::from_obj(Cursor::new(obj), ()).unwrap();
        let indexed = mesh.indexed_triangles();
        let left: Real = "9007199254740992".parse().unwrap();
        let right: Real = "9007199254740993".parse().unwrap();

        assert_eq!(indexed.positions.len(), 6);
        assert!(indexed.positions.iter().any(|point| point.x == left));
        assert!(indexed.positions.iter().any(|point| point.x == right));
    }

    #[test]
    fn from_obj_compacts_duplicate_rows_when_first_used_out_of_order() {
        let obj = "\
v 0 0 0
v 1 0 0
v 0 1 0
v 0 0 0
v 1 0 0
v 0 1 0
vn 0 0 1
f 4//1 5//1 6//1
f 1//1 2//1 3//1
";

        let mesh = Mesh::<()>::from_obj(Cursor::new(obj), ()).unwrap();

        assert_eq!(mesh.polygons.len(), 2);
        assert_eq!(mesh.indexed_triangles().positions.len(), 3);
    }

    #[test]
    fn from_obj_reports_malformed_vertex_line() {
        let error = Mesh::<()>::from_obj(Cursor::new("v 0 0\n"), ()).unwrap_err();

        assert!(error.to_string().contains("line 1"));
        assert!(
            error
                .to_string()
                .contains("vertex line needs three coordinates")
        );
    }

    #[test]
    fn from_obj_reports_malformed_face_line() {
        let obj = "\
v 0 0 0
v 1 0 0
f 1 2
";
        let error = Mesh::<()>::from_obj(Cursor::new(obj), ()).unwrap_err();

        assert!(error.to_string().contains("line 3"));
        assert!(
            error
                .to_string()
                .contains("face line needs at least three vertices")
        );
    }

    #[test]
    fn from_obj_applies_homogeneous_weights() {
        let obj = "v 2 0 0 2\nv 0 2 0 2\nv 0 0 2 2\nf 1 2 3\n";
        let mesh = Mesh::<()>::from_obj(Cursor::new(obj), ()).unwrap();
        assert_eq!(mesh.polygons[0].vertices[0].position.x, Real::one());
    }

    #[test]
    fn from_obj_rejects_texture_data_deliberately() {
        let obj = "v 0 0 0\nv 1 0 0\nv 0 1 0\nvt 0 0\nf 1/1 2/1 3/1\n";
        let error = Mesh::<()>::from_obj(Cursor::new(obj), ()).unwrap_err();
        assert!(matches!(error, IoError::Unsupported { format: "OBJ", .. }));
    }

    #[test]
    fn object_name_cannot_inject_records() {
        let mesh = Mesh::<()>::cube(Real::one(), ());
        assert!(mesh.to_obj("safe\nf 1 2 3").is_err());
        assert!(mesh.to_obj("name#silently truncated").is_err());
    }

    #[test]
    fn to_obj_emits_round_trip_f64_precision() {
        let obj = "\
v 0 0 0
v 1 0 0
v 0 1 0
f 1 2 3
";
        let mesh = Mesh::<()>::from_obj(Cursor::new(obj), ()).unwrap();

        let exported = mesh.to_obj("triangle").unwrap();

        assert!(
            exported.contains("v 1.00000000000000000 0.00000000000000000 0.00000000000000000")
        );
        assert!(exported.contains("o triangle"));
        let reparsed = Mesh::<()>::from_obj(Cursor::new(exported), ()).unwrap();
        assert_eq!(reparsed.polygons.len(), 1);
    }

    #[test]
    fn from_obj_triangulates_concave_faces_without_a_triangle_fan() {
        let obj = "\
v 0 0 0
v 2 0 0
v 2 2 0
v 1 1 0
v 0 2 0
f 1 2 3 4 5
";

        let mesh = Mesh::<()>::from_obj(Cursor::new(obj), ()).unwrap();

        assert_eq!(mesh.polygons.len(), 3);
        assert!(
            mesh.polygons
                .iter()
                .all(|polygon| polygon.vertices.len() == 3)
        );
    }

    #[test]
    fn from_obj_retains_direct_triangle_degeneracy() {
        let obj = "v 0 0 0\nv 1 0 0\nv 2 0 0\nf 1 2 3\n";

        let mesh = Mesh::<()>::from_obj(Cursor::new(obj), ()).unwrap();

        assert!(!mesh.is_manifold());
    }

    #[test]
    fn from_obj_rejects_nonfinite_and_zero_normals_at_boundary() {
        for normal in ["0 0 0", "NaN 0 1", "inf 0 1"] {
            let obj = format!("v 0 0 0\nv 1 0 0\nv 0 1 0\nvn {normal}\nf 1//1 2//1 3//1\n");
            assert!(Mesh::<()>::from_obj(Cursor::new(obj), ()).is_err());
        }
    }

    #[test]
    fn export_rejects_invalid_indices_and_index_overflow() {
        assert!(matches!(
            to_obj(&InvalidIndexed, "invalid"),
            Err(IoError::Geometry { format: "OBJ", .. })
        ));
        assert!(matches!(
            checked_obj_index(usize::MAX, "test index"),
            Err(IoError::SizeOverflow { format: "OBJ", .. })
        ));
    }
}
