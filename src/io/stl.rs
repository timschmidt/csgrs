use crate::triangulated::Triangulated3D;
use std::fmt::Debug;
use core2::io::Cursor;
use stl_io;

/// Export to ASCII STL
/// Convert this Mesh to an **ASCII STL** string with the given `name`.
///
/// ```rust
/// # use csgrs::mesh::Mesh;
/// # use std::error::Error;
/// # fn main() -> Result<(), Box<dyn Error>> {
/// let mesh  = Mesh::<()>::cube(1.0, None);
/// let bytes = mesh.to_stl_ascii("my_solid");
/// std::fs::write("stl/my_solid.stl", bytes)?;
/// # Ok(())
/// # }
/// ```
pub fn to_stl_ascii<T: Triangulated3D>(
	shape: &T,
	name: &str,
) -> String {
	let mut out = String::new();
	out.push_str(&format!("solid {name}\n"));

	shape.visit_triangles(|tri| {
		let n = tri[0].normal; // or recompute if you want per-facet normals
		out.push_str(&format!(
			"  facet normal {:.6} {:.6} {:.6}\n",
			n.x, n.y, n.z
		));
		out.push_str("    outer loop\n");
		for v in &tri {
			let p = v.position;
			out.push_str(&format!(
				"      vertex {:.6} {:.6} {:.6}\n",
				p.x, p.y, p.z
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
/// ```rust
/// # use csgrs::mesh::Mesh;
/// # use std::error::Error;
/// # fn main() -> Result<(), Box<dyn Error>> {
/// let object = Mesh::<()>::cube(1.0, None);
/// let bytes  = object.to_stl_binary("my_solid")?;
/// std::fs::write("stl/my_solid.stl", bytes)?;
/// # Ok(())
/// # }
/// ```
pub fn to_stl_binary<T: Triangulated3D>(
	shape: &T,
	_name: &str,
) -> std::io::Result<Vec<u8>> {
	use stl_io::{Normal, Triangle, Vertex, write_stl};

	let mut triangles = Vec::<Triangle>::new();

	shape.visit_triangles(|tri| {
		let n = tri[0].normal;
		#[allow(clippy::unnecessary_cast)]
		{
			triangles.push(Triangle {
				normal: Normal::new([n.x as f32, n.y as f32, n.z as f32]),
				vertices: tri.map(|v| {
					let p = v.position;
					Vertex::new([p.x as f32, p.y as f32, p.z as f32])
				}),
			});
		}
	});

	let mut cursor = Cursor::new(Vec::new());
	write_stl(&mut cursor, triangles.iter())?;
	Ok(cursor.into_inner())
}

impl<S: Clone + Debug + Send + Sync> crate::mesh::Mesh<S> {
    pub fn to_stl_ascii(&self, name: &str) -> String {
        self::to_stl_ascii(self, name)
    }
    pub fn to_stl_binary(&self, name: &str) -> std::io::Result<Vec<u8>> {
        self::to_stl_binary(self, name)
    }
}

impl<S: Clone + Debug + Send + Sync> crate::sketch::Sketch<S> {
    pub fn to_stl_ascii(&self, name: &str) -> String {
        self::to_stl_ascii(self, name)
    }
    pub fn to_stl_binary(&self, name: &str) -> std::io::Result<Vec<u8>> {
        self::to_stl_binary(self, name)
    }
}

impl<S: Clone + Debug + Send + Sync> crate::bmesh::BMesh<S> {
    pub fn to_stl_ascii(&self, name: &str) -> String {
        self::to_stl_ascii(self, name)
    }
    pub fn to_stl_binary(&self, name: &str) -> std::io::Result<Vec<u8>> {
        self::to_stl_binary(self, name)
    }
}

// TODO: re-impl Mesh::from_stl
// was used in crate::tests::test_csg_to_stl_and_from_stl_file
