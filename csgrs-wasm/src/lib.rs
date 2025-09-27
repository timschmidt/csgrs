use wasm_bindgen::prelude::*;
use js_sys::{Float64Array, Uint32Array, Reflect, Object};

use csgrs::float_types::Real;
use csgrs::mesh::{Mesh, vertex::Vertex};
use csgrs::sketch::Sketch;
use csgrs::traits::CSG;
use nalgebra::{Point3, Vector3};

// Optional: better panic messages in the browser console.
#[cfg(feature = "console_error_panic_hook")]
#[wasm_bindgen(start)]
pub fn init_panic_hook() {
    console_error_panic_hook::set_once();
}

#[wasm_bindgen]
pub struct VertexJs {
    inner: Vertex,
}

#[wasm_bindgen]
impl VertexJs {
    #[wasm_bindgen(constructor)]
    pub fn new(x: f64, y: f64, z: f64) -> VertexJs {
        VertexJs {
            inner: Vertex::new(
                Point3::new(x as Real, y as Real, z as Real),
                Vector3::new(0.0, 0.0, 1.0), // default normal
            ),
        }
    }

    pub fn to_array(&self) -> Vec<f64> {
        vec![self.inner.pos.x as f64, 
             self.inner.pos.y as f64, 
             self.inner.pos.z as f64,
             self.inner.normal.x as f64, 
             self.inner.normal.y as f64, 
             self.inner.normal.z as f64]
    }
}

#[wasm_bindgen]
pub struct MeshJs {
    inner: Mesh<()>,
}

#[wasm_bindgen]
impl MeshJs {
    /// Return an interleaved array of vertex positions (x,y,z)*.
    #[wasm_bindgen(js_name = positions)]
	pub fn positions(&self) -> Float64Array {
		let obj = self.to_arrays();
		let pos = Reflect::get(&obj, &"positions".into()).unwrap();
		pos.dyn_into::<Float64Array>().unwrap()
	}

    /// Return an interleaved array of vertex normals (nx,ny,nz)*.
	#[wasm_bindgen(js_name = normals)]
	pub fn normals(&self) -> Float64Array {
		let obj = self.to_arrays();
		let norms = Reflect::get(&obj, &"normals".into()).unwrap();
		norms.dyn_into::<Float64Array>().unwrap()
	}

    /// Return triangle indices (u32).
	#[wasm_bindgen(js_name = indices)]
	pub fn indices(&self) -> Uint32Array {
		let obj = self.to_arrays();
		let idx = Reflect::get(&obj, &"indices".into()).unwrap();
		idx.dyn_into::<Uint32Array>().unwrap()
	}
    /// Number of triangles (handy to sanity-check).
    #[wasm_bindgen(js_name = triangleCount)]
    pub fn triangle_count(&self) -> u32 {
        self.inner.triangulate().polygons.len() as u32
    }
    
    /// Convert a mesh to arrays of positions, normals, and indices
	#[wasm_bindgen(js_name = to_arrays)]
    pub fn to_arrays(&self) -> js_sys::Object {
        let tri = &self.inner.triangulate();

        let tri_count = tri.polygons.len();
        let mut positions = Vec::with_capacity(tri_count * 3 * 3);
        let mut normals   = Vec::with_capacity(tri_count * 3 * 3);
        let mut indices   = Vec::with_capacity(tri_count * 3);

        let mut idx: u32 = 0;
        for p in &tri.polygons {
            for v in &p.vertices {
                positions.extend_from_slice(&[v.pos.x as f64, v.pos.y as f64, v.pos.z as f64]);
                normals.extend_from_slice(&[v.normal.x as f64, v.normal.y as f64, v.normal.z as f64]);
            }
            indices.extend_from_slice(&[idx, idx + 1, idx + 2]);
            idx += 3;
        }

        let obj = Object::new();
        Reflect::set(
            &obj,
            &"positions".into(),
            &Float64Array::from(positions.as_slice()).into(),
        )
        .unwrap();
        Reflect::set(
            &obj,
            &"normals".into(),
            &Float64Array::from(normals.as_slice()).into(),
        )
        .unwrap();
        Reflect::set(
            &obj,
            &"indices".into(),
            &Uint32Array::from(indices.as_slice()).into(),
        )
        .unwrap();

        obj
    }
}
