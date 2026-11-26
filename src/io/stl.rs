//! STL import and export for native Hypermesh triangle geometry.

use crate::io::{
    IoError, decimal_f64, finite_f32, finite_f64, finite_triangle_normal, single_line_metadata,
};
use hyperlattice::{Point3, Real};
use hypermesh::{ConstructionEvidence, Triangle, TriangleMesh};
use std::cell::RefCell;
use std::io::Cursor;
use std::sync::Arc;

/// Successful binary output retained with both source buffer allocations.
#[derive(Clone)]
struct CachedBinaryStl {
    /// Position identity; ownership prevents reuse of the allocation while cached.
    positions: Arc<[Point3]>,
    /// Triangle identity, paired with positions rather than a geometric equality test.
    triangles: Arc<[Triangle]>,
    /// Complete immutable bytes, copied into an independent vector for each caller.
    bytes: Arc<[u8]>,
}

thread_local! {
    /// Four successful exports per thread, with insertion-order eviction.
    /// Hits do not refresh order; this count is not a retained-memory byte bound.
    static BINARY_STL_CACHE: RefCell<Vec<CachedBinaryStl>> = const { RefCell::new(Vec::new()) };
}

/// Immutable binary STL bytes bound to their source construction qualifications.
///
/// Coordinates were projected by the STL exporter to binary32; the exporter
/// proves nondegenerate encoded triangles, not manifoldness or a geometric
/// error bound. The receipt is an in-process summary, not a serializable proof.
/// There is intentionally no constructor accepting arbitrary bytes and a flag.
#[derive(Clone, Debug)]
pub struct BinaryStlArtifact {
    /// Immutable exported bytes shared by artifact clones, without serialized evidence.
    bytes: Arc<[u8]>,
    /// Source qualification captured separately from the geometry-only byte cache.
    source: ConstructionEvidence,
}

impl BinaryStlArtifact {
    /// Borrow the encoded geometry without changing the bound artifact.
    pub fn bytes(&self) -> &[u8] {
        &self.bytes
    }

    /// Qualifications of the authoritative source before export.
    pub const fn source_construction_evidence(&self) -> ConstructionEvidence {
        self.source
    }

    /// Qualifications of the encoded geometry, including binary32 projection.
    pub const fn construction_evidence(&self) -> ConstructionEvidence {
        self.source.with_coordinate_projection()
    }

    /// Validate and import these exact bytes while retaining their bound summary.
    /// The topology proof concerns the parsed geometry only.
    /// An exported open surface can fail this stricter closed-PWN admission.
    pub fn import(&self) -> Result<TriangleMesh, IoError> {
        from_stl_with_construction(self.bytes(), self.construction_evidence())
    }

    /// Extract plain bytes, explicitly discarding the bound source receipt.
    /// Copies the shared storage into a vector even when this consumes the last artifact.
    pub fn into_bytes(self) -> Vec<u8> {
        self.bytes.to_vec()
    }
}

/// Export binary32 STL while retaining a bound source construction summary.
/// Geometry bytes are identical to [`to_stl_binary`]. The native source remains
/// authoritative and unchanged; the artifact does not assert lossless export.
/// The summary is taken from this mesh even if geometry bytes come from a cache hit.
pub fn to_stl_binary_artifact(
    mesh: &TriangleMesh,
    name: &str,
) -> Result<BinaryStlArtifact, IoError> {
    Ok(BinaryStlArtifact {
        bytes: to_stl_binary(mesh, name)?.into(),
        source: mesh.construction_evidence(),
    })
}

/// Clones triangle coordinates in corner order, rejecting missing indices as text.
/// This checks references only, not geometric nondegeneracy or mesh topology.
fn triangle_points(mesh: &TriangleMesh, triangle: &Triangle) -> Result<[Point3; 3], IoError> {
    let [a, b, c] = triangle.indices();
    let point = |index| {
        mesh.positions
            .get(index)
            .cloned()
            .ok_or_else(|| IoError::Geometry {
                format: "STL",
                detail: format!("triangle references missing position {index}"),
            })
    };
    Ok([point(a)?, point(b)?, point(c)?])
}

/// Computes the shared finite binary64 normal, retaining its STL-labeled failures.
/// Cross-product underflow/overflow can fail even for an exact nondegenerate face.
fn triangle_normal(points: &[Point3; 3]) -> Result<[f64; 3], IoError> {
    finite_triangle_normal(points.each_ref(), "STL")
}

/// Checks the binary header's unsigned 32-bit face count before encoding or caching.
/// This is a format limit, not a bound on allocation size or total conversion work.
fn validate_binary_triangle_count(count: usize) -> Result<(), IoError> {
    u32::try_from(count)
        .map(|_| ())
        .map_err(|_| IoError::SizeOverflow {
            format: "STL",
            limit: "32-bit binary triangle count",
        })
}

/// Serialize native triangle geometry as ASCII STL.
///
/// Finite binary64 coordinates and computed normals use exact decimal spelling.
/// Reimport through [`from_stl`] subsequently quantizes coordinates to binary32;
/// its proof concerns that parsed geometry, not the original exact model.
///
/// Rejects control characters in the solid name before visiting triangles.
/// Each face requires valid references and a finite nonzero binary64 cross-product
/// length. This numerical normal check can reject exact nondegenerate faces;
/// it is not a native topology certificate. Unused positions are omitted, open
/// surfaces are allowed and empty meshes emit matching solid/end-solid lines.
pub fn to_stl_ascii(mesh: &TriangleMesh, name: &str) -> Result<String, IoError> {
    let name = single_line_metadata(name, "STL", "solid name")?;
    let mut output = format!("solid {name}\n");
    for triangle in mesh.triangles.iter() {
        let points = triangle_points(mesh, triangle)?;
        let normal = triangle_normal(&points)?;
        output.push_str(&format!(
            "  facet normal {} {} {}\n",
            decimal_f64(normal[0]),
            decimal_f64(normal[1]),
            decimal_f64(normal[2]),
        ));
        output.push_str("    outer loop\n");
        for point in &points {
            output.push_str(&format!(
                "      vertex {} {} {}\n",
                decimal_f64(finite_f64(&point.x, "STL", "vertex x")?),
                decimal_f64(finite_f64(&point.y, "STL", "vertex y")?),
                decimal_f64(finite_f64(&point.z, "STL", "vertex z")?),
            ));
        }
        output.push_str("    endloop\n  endfacet\n");
    }
    output.push_str(&format!("endsolid {name}\n"));
    Ok(output)
}

/// Serialize native triangle geometry as binary STL.
///
/// Coordinates are projected to finite binary32 values. Each encoded triangle
/// is certified nondegenerate under the strict native policy before complete
/// bytes are published or cached. Open triangle surfaces remain exportable;
/// closed topology and fidelity before projection are separate claims.
/// Plain bytes omit construction history. Use [`to_stl_binary_artifact`] when
/// the next operation must retain a bound source summary.
///
/// Validates the name and 32-bit triangle count before cache lookup. The name
/// is not encoded: the binary header is zero-filled. Unused positions are omitted.
/// Normals come from binary64-projected source points and are then cast to f32;
/// they are not recomputed from the encoded vertices. The preliminary normal
/// calculation can fail by arithmetic underflow/overflow before native validation.
///
/// Only complete successful bytes enter the per-thread four-entry FIFO cache,
/// keyed by both source array allocations. Each return owns a separate byte vector.
/// Empty geometry emits a zero-count header. No whole-mesh closure check is made.
pub fn to_stl_binary(mesh: &TriangleMesh, name: &str) -> Result<Vec<u8>, IoError> {
    use stl_io::{Normal, Triangle as StlTriangle, Vertex, write_stl};

    single_line_metadata(name, "STL", "solid name")?;
    validate_binary_triangle_count(mesh.triangles.len())?;
    if let Some(bytes) = BINARY_STL_CACHE.with_borrow(|entries| {
        entries
            .iter()
            .find(|entry| {
                Arc::ptr_eq(&entry.positions, &mesh.positions)
                    && Arc::ptr_eq(&entry.triangles, &mesh.triangles)
            })
            .map(|entry| entry.bytes.to_vec())
    }) {
        return Ok(bytes);
    }
    let triangles = mesh
        .triangles
        .iter()
        .map(|triangle| {
            let points = triangle_points(mesh, triangle)?;
            let normal = triangle_normal(&points)?;
            let point = |point: &Point3| {
                Ok::<_, IoError>(Vertex::new([
                    finite_f32(&point.x, "STL", "vertex x")?,
                    finite_f32(&point.y, "STL", "vertex y")?,
                    finite_f32(&point.z, "STL", "vertex z")?,
                ]))
            };
            let vertices = [point(&points[0])?, point(&points[1])?, point(&points[2])?];
            let projected = vertices.each_ref().map(|vertex| {
                Point3::new(
                    Real::try_from(vertex[0]).expect("finite_f32 established a finite value"),
                    Real::try_from(vertex[1]).expect("finite_f32 established a finite value"),
                    Real::try_from(vertex[2]).expect("finite_f32 established a finite value"),
                )
            });
            if !hypermesh::Plane::points_are_nondegenerate(
                &crate::GeometryContext::STRICT.mesh_context(),
                &projected[0],
                &projected[1],
                &projected[2],
            )
            .map_err(|error| IoError::Construction {
                format: "STL",
                source: error.into(),
            })?
            .value
            {
                return Err(IoError::Geometry {
                    format: "STL",
                    detail: "degenerate triangle after binary32 projection".into(),
                });
            }
            Ok(StlTriangle {
                normal: Normal::new([normal[0] as f32, normal[1] as f32, normal[2] as f32]),
                vertices,
            })
        })
        .collect::<Result<Vec<_>, IoError>>()?;
    let mut cursor = Cursor::new(Vec::new());
    write_stl(&mut cursor, triangles.iter())?;
    let bytes = cursor.into_inner();
    BINARY_STL_CACHE.with_borrow_mut(|entries| {
        /// Maximum successful entries per thread, not a byte or work budget.
        const CAPACITY: usize = 4;
        if entries.len() == CAPACITY {
            entries.remove(0);
        }
        entries.push(CachedBinaryStl {
            positions: Arc::clone(&mesh.positions),
            triangles: Arc::clone(&mesh.triangles),
            bytes: Arc::from(bytes.as_slice()),
        });
    });
    Ok(bytes)
}

/// Import STL triangles with certified closed-PWN topology into Hypermesh.
///
/// Both encodings parse coordinates as `f32`, then promote finite values exactly
/// to native scalars. Strict native validation proves nondegenerate faces and
/// balanced geometric edge multiplicities, retaining that proof for later
/// operations. Canonical empty geometry is also accepted. Disconnected, nested,
/// inverted and multiply wound closed surfaces need not be manifolds.
///
/// This certifies the parsed geometry, not fidelity to an earlier exact model
/// or analytic surface. Invalid topology returns [`IoError::Construction`] with
/// the native mesh error preserved as its cause; parsing errors remain IO errors.
/// Nonfinite parsed coordinates become textual malformed-input errors. File
/// normals are ignored. Plain input is marked as projected with missing source
/// history, even when every encoded coordinate happened to preserve its source.
pub fn from_stl(data: &[u8]) -> Result<TriangleMesh, IoError> {
    from_stl_with_construction(
        data,
        ConstructionEvidence::UNTRACKED.with_coordinate_projection(),
    )
}

/// Parses with `stl_io`, promotes binary32 coordinates and attaches trusted local evidence.
///
/// Parser indexing shares bit-identical vertex triples; native geometric edge
/// validation remains authoritative. File normals are discarded. Nonempty output
/// must pass strict closed-PWN admission, caching that local proof without
/// upgrading inherited qualification. Canonical empty output bypasses nonempty
/// validation. Only this module can supply the artifact's bound evidence summary.
fn from_stl_with_construction(
    data: &[u8],
    evidence: ConstructionEvidence,
) -> Result<TriangleMesh, IoError> {
    let mut cursor = Cursor::new(data);
    let indexed = stl_io::read_stl(&mut cursor)?;
    let positions = indexed
        .vertices
        .iter()
        .map(|vertex| {
            Ok(Point3::new(
                Real::try_from(vertex[0]).map_err(|error| {
                    IoError::MalformedInput(format!("STL vertex x is invalid: {error}"))
                })?,
                Real::try_from(vertex[1]).map_err(|error| {
                    IoError::MalformedInput(format!("STL vertex y is invalid: {error}"))
                })?,
                Real::try_from(vertex[2]).map_err(|error| {
                    IoError::MalformedInput(format!("STL vertex z is invalid: {error}"))
                })?,
            ))
        })
        .collect::<Result<Vec<_>, IoError>>()?;
    let mut triangles = Vec::with_capacity(indexed.faces.len());
    for face in indexed.faces {
        triangles.push(Triangle::new(
            face.vertices[0],
            face.vertices[1],
            face.vertices[2],
        ));
    }
    let mesh = TriangleMesh::from_construction(positions, triangles, evidence);
    if !(mesh.positions.is_empty() && mesh.triangles.is_empty()) {
        mesh.validate_closed_pwn(&crate::GeometryContext::STRICT.mesh_context())
            .map_err(|error| IoError::Construction {
                format: "STL",
                source: error.into(),
            })?;
    }
    Ok(mesh)
}

#[cfg(test)]
/// STL boundary smoke witnesses; topology checks occur inside the importer.
mod tests {
    use super::*;

    #[test]
    /// Requires validated binary cube replay with twelve resulting triangles.
    fn binary_cube_round_trips_through_validated_import() {
        let cube = crate::solid::cube(Real::from(2_u8)).expect("valid primitive fixture");
        let bytes = to_stl_binary(&cube, "cube").unwrap();
        assert_eq!(from_stl(&bytes).unwrap().triangles.len(), 12);
    }

    #[test]
    /// Checks successful sampled-sphere binary replay and preservation of triangle count.
    fn sampled_sphere_is_representable_at_the_stl_boundary() {
        let sphere =
            crate::solid::sphere(Real::from(10_u8), 32, 16).expect("valid primitive fixture");
        let bytes = to_stl_binary(&sphere, "sphere").unwrap();
        assert_eq!(
            from_stl(&bytes).unwrap().triangles.len(),
            sphere.triangles.len()
        );
    }

    #[test]
    /// Rejects name injection, a repeated-point ASCII facet and unparseable STL bytes.
    fn rejects_injected_names_and_degenerate_geometry() {
        let cube = crate::solid::cube(Real::one()).expect("valid primitive fixture");
        assert!(to_stl_ascii(&cube, "safe\nendsolid forged").is_err());
        let point = Point3::origin();
        let degenerate = TriangleMesh::new(
            vec![point.clone(), point.clone(), point],
            vec![Triangle::new(0, 1, 2)],
        );
        assert!(to_stl_ascii(&degenerate, "degenerate").is_err());
        assert!(from_stl(b"not an STL").is_err());
    }
}
