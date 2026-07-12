//! JavaScript wrapper for polygons.

use crate::mesh::Polygon;
use crate::vertex::Vertex;
use crate::wasm::real_to_js;
use crate::wasm::{
    js_metadata, metadata_to_js, plane_js::PlaneJs, point_js::Point3Js, vector_js::Vector3Js,
    vertex_js::VertexJs,
};
use js_sys::{Object, Reflect};
use serde_json::Value as JsonValue;
use wasm_bindgen::prelude::*;

const fn validate_polygon_vertices(vertices: &[VertexJs]) -> Result<(), &'static str> {
    if vertices.len() < 3 {
        return Err("Polygon.fromVertices requires at least 3 vertices");
    }
    Ok(())
}

#[wasm_bindgen]
pub struct PolygonJs {
    pub(crate) inner: Polygon<Option<JsonValue>>,
}

#[wasm_bindgen]
impl PolygonJs {
    /// Construct a polygon from a list of vertices and optional metadata.
    ///
    /// Metadata may be any JSON-serializable value; it is stored as a JSON string
    /// in the underlying Rust `Polygon<JsonValue>`.
    #[wasm_bindgen(constructor)]
    pub fn new(vertices: Vec<VertexJs>, metadata: JsValue) -> Result<PolygonJs, JsValue> {
        PolygonJs::from_vertices(vertices, metadata)
    }

    /// Construct from vertices (same as constructor, but named).
    #[wasm_bindgen(js_name = fromVertices)]
    pub fn from_vertices(
        vertices: Vec<VertexJs>,
        metadata: JsValue,
    ) -> Result<PolygonJs, JsValue> {
        validate_polygon_vertices(&vertices).map_err(JsValue::from_str)?;

        // Vertices are finite wasm boundary wrappers; polygon plane selection
        // is delegated to hyperlattice/hyperreal predicates in `Polygon`/`Plane`,
        // following Yap, "Towards Exact Geometric Computation," Computational
        // Geometry 7(1-2), 1997
        // (<https://doi.org/10.1016/0925-7721(95)00040-2>).
        let verts: Vec<Vertex> = vertices.into_iter().map(|v| v.inner).collect();
        let meta = js_metadata(metadata)?;

        Ok(PolygonJs {
            inner: Polygon::new(verts, meta),
        })
    }

    /// Get the vertices as `VertexJs[]`.
    #[wasm_bindgen(js_name = vertices)]
    pub fn vertices(&self) -> JsValue {
        let arr = js_sys::Array::new();

        for v in &self.inner.vertices {
            let js_vert = VertexJs { inner: v.clone() };
            arr.push(&JsValue::from(js_vert));
        }

        arr.into()
    }

    /// Get the polygon's plane as a `PlaneJs`.
    #[wasm_bindgen(js_name = plane)]
    pub fn plane(&self) -> PlaneJs {
        PlaneJs::from(self.inner.plane.clone())
    }

    /// Flip winding order and vertex normals in place.
    #[wasm_bindgen(js_name = flip)]
    pub fn flip(&mut self) {
        self.inner.flip();
    }

    /// Axis-aligned bounding box of this polygon as `{ min: Point3Js, max: Point3Js }`.
    #[wasm_bindgen(js_name = boundingBox)]
    pub fn bounding_box(&self) -> JsValue {
        let bb = self.inner.bounding_box();
        let min_js = Point3Js::from(bb.mins.clone());
        let max_js = Point3Js::from(bb.maxs.clone());

        let obj = Object::new();
        Reflect::set(&obj, &"min".into(), &JsValue::from(min_js)).unwrap();
        Reflect::set(&obj, &"max".into(), &JsValue::from(max_js)).unwrap();

        obj.into()
    }

    /// Get metadata as its original JSON-compatible JavaScript value.
    #[wasm_bindgen(js_name = metadata)]
    pub fn metadata(&self) -> Result<JsValue, JsValue> {
        metadata_to_js(&self.inner.metadata)
    }

    /// Set metadata from any JSON-serializable JS value.
    #[wasm_bindgen(js_name = setMetadata)]
    pub fn set_metadata(&mut self, metadata: JsValue) -> Result<(), JsValue> {
        let meta = js_metadata(metadata)?;
        self.inner.set_metadata(meta);
        Ok(())
    }

    /// Recalculate a normal from all vertices and return it.
    #[wasm_bindgen(js_name = calculateNewNormal)]
    pub fn calculate_new_normal(&self) -> Vector3Js {
        let n = self.inner.calculate_new_normal();
        Vector3Js::from(n)
    }

    /// Recompute and assign a new flat normal to all vertices.
    #[wasm_bindgen(js_name = setNewNormal)]
    pub fn set_new_normal(&mut self) {
        self.inner.set_new_normal();
    }

    /// Flatten all vertices to a single Float64 array:
    /// `[x, y, z, nx, ny, nz, x, y, z, nx, ny, nz, ...]`
    #[wasm_bindgen(js_name = toArray)]
    pub fn to_array(&self) -> Vec<f64> {
        let mut data = Vec::with_capacity(self.inner.vertices.len() * 6);

        for v in &self.inner.vertices {
            data.push(real_to_js(&v.position.x));
            data.push(real_to_js(&v.position.y));
            data.push(real_to_js(&v.position.z));
            data.push(real_to_js(&v.normal.0[0]));
            data.push(real_to_js(&v.normal.0[1]));
            data.push(real_to_js(&v.normal.0[2]));
        }

        data
    }

    /// Triangulate this polygon into a list of triangular polygons.
    ///
    /// Returns `PolygonJs[]`, each of which is a triangle.
    #[wasm_bindgen(js_name = triangulate)]
    pub fn triangulate(&self) -> Vec<PolygonJs> {
        let tris = self.inner.triangulate();
        let mut out = Vec::with_capacity(tris.len());

        for tri in tris {
            let vertices = tri.to_vec();
            let poly = Polygon::new(vertices, self.inner.metadata.clone());
            out.push(PolygonJs { inner: poly });
        }

        out
    }

    /// Subdivide this polygon's triangles, returning the refined triangular polygons.
    ///
    /// If `levels` is 0, returns a single-element array containing this polygon.
    #[wasm_bindgen(js_name = subdivideTriangles)]
    pub fn subdivide_triangles(&self, levels: u32) -> Vec<PolygonJs> {
        if levels == 0 {
            return vec![PolygonJs {
                inner: self.inner.clone(),
            }];
        }

        let levels_nz = std::num::NonZeroU32::new(levels).unwrap();
        let polys = self.inner.subdivide_to_polygons(levels_nz);

        polys.into_iter().map(|p| PolygonJs { inner: p }).collect()
    }
}

// Optional conversions for convenience
impl From<Polygon<Option<JsonValue>>> for PolygonJs {
    fn from(p: Polygon<Option<JsonValue>>) -> Self {
        PolygonJs { inner: p }
    }
}

impl From<&PolygonJs> for Polygon<Option<JsonValue>> {
    fn from(p: &PolygonJs) -> Self {
        p.inner.clone()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn polygon_js_from_vertices_rejects_short_input_without_panic() {
        let result = std::panic::catch_unwind(|| validate_polygon_vertices(&[]));
        assert!(result.is_ok());
        assert!(result.unwrap().is_err());
    }
}
