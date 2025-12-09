
#![doc = " glTF 2.0 file format support for Mesh and Sketch objects"]
#![doc = ""]
#![doc = " This module provides export functionality for glTF 2.0 files,"]
#![doc = " a modern, efficient, and widely supported 3D asset format."]

use crate::float_types::{tolerance, Real};
use crate::mesh::Mesh;
use crate::sketch::Sketch;
use geo::CoordsIter;
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

impl<S: Clone + Debug + Send + Sync> Mesh<S> {
    #[doc = " Export this Mesh to glTF 2.0 format as a string"]
    #[doc = ""]
    #[doc = " Creates a glTF 2.0 (.gltf) JSON file containing:"]
    #[doc = " 1. All 3D polygons from `self.polygons` (tessellated to triangles)"]
    #[doc = " 2. POSITION and NORMAL attributes and triangle indices"]
    #[doc = " 3. A single mesh / single node / single scene"]
    #[doc = ""]
    #[doc = " The binary data is embedded as a base64 buffer in the JSON file."]
    #[doc = ""]
    #[doc = " # Arguments"]
    #[doc = " * `object_name` - Name for the mesh object in the glTF file"]
    #[doc = ""]
    #[doc = " # Example"]
    #[doc = " ```"]
    #[doc = " use csgrs::mesh::Mesh;"]
    #[doc = " let csg: Mesh<()> = Mesh::cube(10.0, None);"]
    #[doc = " let gltf_content = csg.to_gltf(\"my_cube\");"]
    #[doc = " println!(\"{}\", gltf_content);"]
    #[doc = " ```"]
    pub fn to_gltf(&self, object_name: &str) -> String {
        let mut vertices = Vec::<GltfVertex>::new();
        let mut indices = Vec::<u32>::new();

        for poly in &self.polygons {
            let triangles = poly.triangulate();
            let normal = poly.plane.normal().normalize();

            for triangle in triangles {
                for vertex in triangle {
                    let idx =
                        add_unique_vertex_gltf(&mut vertices, vertex.pos, normal);
                    indices.push(idx);
                }
            }
        }

        gltf_from_vertices(&vertices, &indices, object_name)
    }

    #[doc = " Export this Mesh to a glTF 2.0 file"]
    #[doc = ""]
    #[doc = " # Arguments"]
    #[doc = " * `writer` - Where to write the glTF JSON data"]
    #[doc = " * `object_name` - Name for the object in the glTF file"]
    #[doc = ""]
    #[doc = " # Example"]
    #[doc = " ```"]
    #[doc = " use csgrs::mesh::Mesh;"]
    #[doc = " use std::fs::File;"]
    #[doc = " # fn main() -> Result<(), Box<dyn std::error::Error>> {"]
    #[doc = " let csg: Mesh<()> = Mesh::cube(10.0, None);"]
    #[doc = " let mut file = File::create(\"stl/output.gltf\")?;"]
    #[doc = " csg.write_gltf(&mut file, \"my_mesh\")?;"]
    #[doc = " # Ok(())"]
    #[doc = " # }"]
    #[doc = " ```"]
    pub fn write_gltf<W: Write>(
        &self,
        writer: &mut W,
        object_name: &str,
    ) -> std::io::Result<()> {
        let gltf_content = self.to_gltf(object_name);
        writer.write_all(gltf_content.as_bytes())
    }
}

impl<S: Clone + Debug + Send + Sync> Sketch<S> {
    #[doc = " Export this Sketch to glTF 2.0 format as a string"]
    #[doc = ""]
    #[doc = " Creates a glTF 2.0 (.gltf) JSON file containing:"]
    #[doc = " 1. All 2D polygons from `self.geometry` triangulated and placed at Z=0"]
    #[doc = " 2. POSITION and NORMAL attributes and triangle indices"]
    #[doc = " 3. A single mesh / single node / single scene"]
    #[doc = ""]
    #[doc = " The binary data is embedded as a base64 buffer in the JSON file."]
    #[doc = ""]
    #[doc = " # Arguments"]
    #[doc = " * `object_name` - Name for the object in the glTF file"]
    #[doc = ""]
    #[doc = " # Example"]
    #[doc = " ```"]
    #[doc = " use csgrs::sketch::Sketch;"]
    #[doc = " let sketch: Sketch<()> = Sketch::square(2.0, None);"]
    #[doc = " let gltf_content = sketch.to_gltf(\"my_sketch\");"]
    #[doc = " println!(\"{}\", gltf_content);"]
    #[doc = " ```"]
    pub fn to_gltf(&self, object_name: &str) -> String {
        let mut vertices = Vec::<GltfVertex>::new();
        let mut indices = Vec::<u32>::new();

        for geom in &self.geometry.0 {
            match geom {
                geo::Geometry::Polygon(poly2d) => {
                    self.add_2d_polygon_to_gltf(poly2d, &mut vertices, &mut indices);
                }
                geo::Geometry::MultiPolygon(mp) => {
                    for poly2d in &mp.0 {
                        self.add_2d_polygon_to_gltf(
                            poly2d,
                            &mut vertices,
                            &mut indices,
                        );
                    }
                }
                _ => {}
            }
        }

        gltf_from_vertices(&vertices, &indices, object_name)
    }

    #[doc = " Export this Sketch to a glTF 2.0 file"]
    #[doc = ""]
    #[doc = " # Arguments"]
    #[doc = " * `writer` - Where to write the glTF JSON data"]
    #[doc = " * `object_name` - Name for the object in the glTF file"]
    #[doc = ""]
    #[doc = " # Example"]
    #[doc = " ```"]
    #[doc = " use csgrs::sketch::Sketch;"]
    #[doc = " use std::fs::File;"]
    #[doc = " # fn main() -> Result<(), Box<dyn std::error::Error>> {"]
    #[doc = " let sketch: Sketch<()> = Sketch::square(2.0, None);"]
    #[doc = " let mut file = File::create(\"stl/output.gltf\")?;"]
    #[doc = " sketch.write_gltf(&mut file, \"my_sketch\")?;"]
    #[doc = " # Ok(())"]
    #[doc = " # }"]
    #[doc = " ```"]
    pub fn write_gltf<W: Write>(
        &self,
        writer: &mut W,
        object_name: &str,
    ) -> std::io::Result<()> {
        let gltf_content = self.to_gltf(object_name);
        writer.write_all(gltf_content.as_bytes())
    }

    fn add_2d_polygon_to_gltf(
        &self,
        poly2d: &geo::Polygon<Real>,
        vertices: &mut Vec<GltfVertex>,
        indices: &mut Vec<u32>,
    ) {
        let exterior: Vec<[Real; 2]> = poly2d
            .exterior()
            .coords_iter()
            .map(|c| [c.x, c.y])
            .collect();

        let holes_vec: Vec<Vec<[Real; 2]>> = poly2d
            .interiors()
            .iter()
            .map(|ring| ring.coords_iter().map(|c| [c.x, c.y]).collect())
            .collect();

        let hole_refs: Vec<&[[Real; 2]]> =
            holes_vec.iter().map(|h| &h[..]).collect();

        let triangles_2d = Self::triangulate_with_holes(&exterior, &hole_refs);
        let normal = Vector3::new(0.0, 0.0, 1.0);

        for triangle in triangles_2d {
            for point in triangle {
                let vertex_3d = Point3::new(point.x, point.y, point.z);
                let idx =
                    add_unique_vertex_gltf(vertices, vertex_3d, normal);
                indices.push(idx);
            }
        }
    }
}
