//! STL import and export helpers for triangulated 3D geometry.

use crate::triangulated::Triangulated3D;
use std::fmt::Debug;
use std::io::Cursor;
use stl_io;

fn real_f32(value: &hyperlattice::Real) -> f32 {
    value
        .to_f32_lossy()
        .filter(|value| value.is_finite())
        .unwrap_or(0.0)
}

fn real_f64(value: &hyperlattice::Real) -> f64 {
    value
        .to_f64_lossy()
        .filter(|value| value.is_finite())
        .unwrap_or(0.0)
}

/// Export to ASCII STL
/// Convert this Mesh to an **ASCII STL** string with the given `name`.
///
/// ```ignore
/// # use csgrs::mesh::Mesh;
/// # use std::error::Error;
/// # fn main() -> Result<(), Box<dyn Error>> {
/// let mesh  = Mesh::<()>::cube(1.0, ());
/// let bytes = mesh.to_stl_ascii("my_solid");
/// assert!(bytes.starts_with("solid my_solid"));
/// # Ok(())
/// # }
/// ```
pub fn to_stl_ascii<T: Triangulated3D>(shape: &T, name: &str) -> String {
    let mut out = String::new();
    out.push_str(&format!("solid {name}\n"));

    shape.visit_triangles(|tri| {
        let n = &tri[0].normal; // or recompute if you want per-facet normals
        out.push_str(&format!(
            "  facet normal {:.6} {:.6} {:.6}\n",
            real_f64(&n.0[0]),
            real_f64(&n.0[1]),
            real_f64(&n.0[2])
        ));
        out.push_str("    outer loop\n");
        for v in &tri {
            let p = &v.position;
            out.push_str(&format!(
                "      vertex {:.6} {:.6} {:.6}\n",
                real_f64(&p.x),
                real_f64(&p.y),
                real_f64(&p.z)
            ));
        }
        out.push_str("    endloop\n");
        out.push_str("  endfacet\n");
    });

    out.push_str(&format!("endsolid {name}\n"));
    out
}

/// Export to BINARY STL (returns `Vec<u8>`)
///
/// Convert this Mesh to a **binary STL** byte vector with the given `name`.
///
/// The resulting `Vec<u8>` can then be written to a file or handled in memory:
///
/// ```ignore
/// # use csgrs::mesh::Mesh;
/// # use std::error::Error;
/// # fn main() -> Result<(), Box<dyn Error>> {
/// let object = Mesh::<()>::cube(1.0, ());
/// let bytes  = object.to_stl_binary("my_solid")?;
/// assert!(!bytes.is_empty());
/// # Ok(())
/// # }
/// ```
pub fn to_stl_binary<T: Triangulated3D>(shape: &T, _name: &str) -> std::io::Result<Vec<u8>> {
    use stl_io::{Normal, Triangle, Vertex, write_stl};

    let mut triangles = Vec::<Triangle>::new();

    shape.visit_triangles(|tri| {
        let n = &tri[0].normal;
        #[allow(clippy::unnecessary_cast)]
        {
            triangles.push(Triangle {
                normal: Normal::new([real_f32(&n.0[0]), real_f32(&n.0[1]), real_f32(&n.0[2])]),
                vertices: tri.map(|v| {
                    let p = v.position;
                    Vertex::new([real_f32(&p.x), real_f32(&p.y), real_f32(&p.z)])
                }),
            });
        }
    });

    let mut cursor = Cursor::new(Vec::new());
    write_stl(&mut cursor, triangles.iter())?;
    Ok(cursor.into_inner())
}

impl<M: Clone + Debug + Send + Sync> crate::mesh::Mesh<M> {
    pub fn to_stl_ascii(&self, name: &str) -> String {
        self::to_stl_ascii(self, name)
    }
    pub fn to_stl_binary(&self, name: &str) -> std::io::Result<Vec<u8>> {
        self::to_stl_binary(self, name)
    }
}

#[cfg(feature = "sketch")]
impl<M: Clone + Debug + Send + Sync> crate::sketch::Profile<M> {
    pub fn to_stl_ascii(&self, name: &str) -> String {
        self::to_stl_ascii(self, name)
    }
    pub fn to_stl_binary(&self, name: &str) -> std::io::Result<Vec<u8>> {
        self::to_stl_binary(self, name)
    }
}

// TODO: re-impl Mesh::from_stl
// was used in crate::tests::test_csg_to_stl_and_from_stl_file
