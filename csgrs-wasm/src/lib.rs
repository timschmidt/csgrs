use wasm_bindgen::prelude::*;
use js_sys::{Float64Array, Uint32Array, Reflect, Object};
use csgrs::float_types::Real;
use csgrs::mesh::{Mesh, plane::Plane, polygon::Polygon, vertex::Vertex};
use csgrs::sketch::Sketch;
use csgrs::io::svg::{FromSVG, ToSVG};
use csgrs::traits::CSG;
use nalgebra::{Point3, Matrix4, Vector3};
use serde_wasm_bindgen::from_value;
use geo::{Geometry, GeometryCollection};

// Optional: better panic messages in the browser console.
#[cfg(feature = "console_error_panic_hook")]
#[wasm_bindgen(start)]
pub fn init_panic_hook() {
    console_error_panic_hook::set_once();
}

#[wasm_bindgen]
pub struct Matrix4Js {
    inner: Matrix4<f64>,
}

#[wasm_bindgen]
pub struct PolygonJs {
    inner: Polygon<()>,
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
pub struct SketchJs {
    inner: Sketch<()>,
}

#[wasm_bindgen]
impl SketchJs {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        Self {
            inner: Sketch::new(),
        }
    }
    
    #[wasm_bindgen(js_name = isEmpty)]
    pub fn is_empty(&self) -> bool {
        self.inner.geometry.0.is_empty()
    }

    #[wasm_bindgen(js_name = toArrays)]
    pub fn to_arrays(&self) -> JsValue {
        let mut positions = Vec::new();
        let mut indices = Vec::new();
        let mut normals = Vec::new();

        // Convert 2D geometry to 3D triangles for visualization
        let triangulated = self.inner.triangulate();
        for tri in triangulated {
			let [a, b, c] = tri;
			
			// Push vertices (Z=0 for 2D)
			positions.push(a.x);
			positions.push(a.y);
			positions.push(0.0);
			positions.push(b.x);
			positions.push(b.y);
			positions.push(0.0);
			positions.push(c.x);
			positions.push(c.y);
			positions.push(0.0);

			// Push normals (upwards for 2D)
			normals.push(0.0);
			normals.push(0.0);
			normals.push(1.0);
			normals.push(0.0);
			normals.push(0.0);
			normals.push(1.0);
			normals.push(0.0);
			normals.push(0.0);
			normals.push(1.0);

			// Push indices
			let base_idx = indices.len() / 3;
			indices.push(base_idx as u32);
			indices.push((base_idx + 1) as u32);
			indices.push((base_idx + 2) as u32);
        }

        let pos_array = Float64Array::from(positions.as_slice());
        let norm_array = Float64Array::from(normals.as_slice());
        let idx_array = Uint32Array::from(indices.as_slice());

        let obj = Object::new();
        Reflect::set(&obj, &"positions".into(), &pos_array).unwrap();
        Reflect::set(&obj, &"normals".into(), &norm_array).unwrap();
        Reflect::set(&obj, &"indices".into(), &idx_array).unwrap();
        obj.into()
    }

	#[wasm_bindgen(js_name = polygon)]
	pub fn polygon(points: JsValue) -> Result<Self, JsValue> {
		let points_vec: Vec<[f64; 2]> = from_value(points)
			.map_err(|e| JsValue::from_str(&format!("Failed to parse points: {:?}", e)))?;

		let points_2d: Vec<[Real; 2]> = points_vec
			.into_iter()
			.map(|[x, y]| [x as Real, y as Real])
			.collect();

		Ok(Self {
			inner: Sketch::polygon(&points_2d, None),
		})
	}
	
	/* fix to work with outer, holes as input
	#[wasm_bindgen(js_name=triangulateWithHoles)]
    pub fn triangulate_with_holes(outer, holes) -> Vec<JsValue> {
        let tris = Sketch::<()>::triangulate_with_holes(outer, holes);
        tris.into_iter()
            .map(|tri| {
                let points: Vec<[f64; 3]> = tri
                    .iter()
                    .map(|v| [v.pos.x, v.pos.y, v.pos.z])
                    .collect();
                JsValue::from_serde(&points).unwrap_or(JsValue::NULL)
            })
            .collect()
    }
    */
    
	/*
	
	error[E0609]: no field `pos` on type `&OPoint<f64, Const<3>>`
   --> src/lib.rs:159:33
    |
159 |                     .map(|v| [v.pos.x, v.pos.y, v.pos.z])
    |                                 ^^^ unknown field
    |
    = note: available field is: `coords`
    = note: available fields are: `x`, `y`, `z`
	
	#[wasm_bindgen(js_name=triangulate)]
    pub fn triangulate(&self) -> Vec<JsValue> {
        let tris = self.inner.triangulate();
        tris.into_iter()
            .map(|tri| {
                let points: Vec<[f64; 3]> = tri
                    .iter()
                    .map(|v| [v.pos.x, v.pos.y, v.pos.z])
                    .collect();
                JsValue::from_serde(&points).unwrap_or(JsValue::NULL)
            })
            .collect()
    }
    */

	// IO operations
    #[wasm_bindgen(js_name = fromSVG)]
    pub fn from_svg(svg_data: &str) -> Result<Self, JsValue> {
        let sketch = Sketch::from_svg(svg_data)
            .map_err(|e| JsValue::from_str(&format!("SVG parsing error: {:?}", e)))?;
        Ok(Self { inner: sketch })
    }

    #[wasm_bindgen(js_name = toSVG)]
    pub fn to_svg(&self) -> String {
        self.inner.to_svg()
    }
    
    #[wasm_bindgen(js_name=fromGeo)]
    pub fn from_geo(geo_json: &str) -> Result<SketchJs, JsValue> {
        let geometry: Geometry<Real> = serde_json::from_str(geo_json)
            .map_err(|e| JsValue::from_str(&format!("Failed to parse GeoJSON: {}", e)))?;
        let sketch = Sketch::from_geo(GeometryCollection(vec![geometry]), None);
        Ok(SketchJs { inner: sketch })
    }
    
	#[wasm_bindgen(js_name=toMultiPolygon)]
    pub fn to_multipolygon(&self) -> String {
        let mp = self.inner.to_multipolygon();
        serde_json::to_string(&mp).unwrap_or_else(|_| "null".to_string())
    }
    
    #[wasm_bindgen(js_name=fromMesh)]
    pub fn from_mesh(mesh_js: &MeshJs) -> SketchJs {
        let sketch = Sketch::from(mesh_js.inner.clone());
        SketchJs { inner: sketch }
    }

    // Boolean Operations
    #[wasm_bindgen(js_name = union)]
    pub fn union(&self, other: &SketchJs) -> Self {
        Self {
            inner: self.inner.union(&other.inner),
        }
    }

    #[wasm_bindgen(js_name = difference)]
    pub fn difference(&self, other: &SketchJs) -> Self {
        Self {
            inner: self.inner.difference(&other.inner),
        }
    }

    #[wasm_bindgen(js_name = intersection)]
    pub fn intersection(&self, other: &SketchJs) -> Self {
        Self {
            inner: self.inner.intersection(&other.inner),
        }
    }

    #[wasm_bindgen(js_name = xor)]
    pub fn xor(&self, other: &SketchJs) -> Self {
        Self {
            inner: self.inner.xor(&other.inner),
        }
    }

    // Transformations
	#[wasm_bindgen(js_name=transform)]
    pub fn transform(&self, mat: &Matrix4Js) -> SketchJs {
        Self { inner: self.inner.transform(&mat.inner), }
    }
    
    #[wasm_bindgen(js_name = translate)]
    pub fn translate(&self, dx: Real, dy: Real, dz: Real) -> Self {
        Self {
            inner: self.inner.translate(dx, dy, dz),
        }
    }

    #[wasm_bindgen(js_name = rotate)]
    pub fn rotate(&self, rx: Real, ry: Real, rz: Real) -> Self {
        Self {
            inner: self.inner.rotate(rx, ry, rz),
        }
    }

    #[wasm_bindgen(js_name = scale)]
    pub fn scale(&self, sx: Real, sy: Real, sz: Real) -> Self {
        Self {
            inner: self.inner.scale(sx, sy, sz),
        }
    }

    #[wasm_bindgen(js_name = center)]
    pub fn center(&self) -> Self {
        Self {
            inner: self.inner.center(),
        }
    }
    
    #[wasm_bindgen(js_name=inverse)]
    pub fn inverse(&self) -> SketchJs {
        let sketch = self.inner.inverse();
        Self { inner: sketch }
    }
    
	#[wasm_bindgen(js_name=renormalize)]
    pub fn renormalize(&self) -> SketchJs {
        let sketch = self.inner.renormalize();
        Self { inner: sketch }
    }

    // Extrusion and 3D Operations
    #[wasm_bindgen(js_name = extrude)]
    pub fn extrude(&self, height: Real) -> MeshJs {
        let mesh = self.inner.extrude(height);
        MeshJs { inner: mesh }
    }

    #[wasm_bindgen(js_name = revolve)]
    pub fn revolve(&self, angle_degrees: Real, segments: usize) -> Result<MeshJs, JsValue> {
        let mesh = self.inner.revolve(angle_degrees, segments)
            .map_err(|e| JsValue::from_str(&format!("Revolve failed: {:?}", e)))?;
        Ok(MeshJs { inner: mesh })
    }

    // Offset Operations (if offset feature is enabled)
    #[cfg(feature = "offset")]
    #[wasm_bindgen(js_name = offset)]
    pub fn offset(&self, distance: Real) -> Self {
        Self {
            inner: self.inner.offset(distance),
        }
    }

    #[cfg(feature = "offset")]
    #[wasm_bindgen(js_name = offsetRounded)]
    pub fn offset_rounded(&self, distance: Real) -> Self {
        Self {
            inner: self.inner.offset_rounded(distance),
        }
    }
    
    #[cfg(feature = "offset")]
	#[wasm_bindgen(js_name=straightSkeleton)]
    pub fn straight_skeleton(&self, orientation: f64) -> SketchJs {
        let sketch = self.inner.straight_skeleton(orientation);
        Self { inner: sketch }
    }

    // Bounding Box
	#[wasm_bindgen(js_name = boundingBox)]
	pub fn bounding_box(&self) -> JsValue {
		let bb = self.inner.bounding_box();
		let min = Point3::new(bb.mins.x, bb.mins.y, bb.mins.z);
		let max = Point3::new(bb.maxs.x, bb.maxs.y, bb.maxs.z);

		let obj = Object::new();
		let min_arr = js_sys::Array::of3(&min.x.into(), &min.y.into(), &min.z.into());
		let max_arr = js_sys::Array::of3(&max.x.into(), &max.y.into(), &max.z.into());

		Reflect::set(&obj, &"min".into(), &min_arr).unwrap();
		Reflect::set(&obj, &"max".into(), &max_arr).unwrap();

		obj.into()
	}
	
	#[wasm_bindgen(js_name=invalidateBoundingBox)]
    pub fn invalidate_bounding_box(&mut self) {
        self.inner.invalidate_bounding_box();
    }
    
	// 2D Shapes
    #[wasm_bindgen(js_name = square)]
    pub fn square(width: Real) -> Self {
        Self {
            inner: Sketch::square(width, None),
        }
    }

    #[wasm_bindgen(js_name = circle)]
    pub fn circle(radius: Real, segments: usize) -> Self {
        Self {
            inner: Sketch::circle(radius, segments, None),
        }
    }
    
	#[cfg(feature = "offset")]
	#[wasm_bindgen(js_name = hilbertCurve)]
	pub fn hilbert_curve(&self, order: usize, padding: Real) -> Self {
		Self { inner: self.inner.hilbert_curve(order, padding) }
	}
}


#[wasm_bindgen]
pub struct MeshJs {
    inner: Mesh<()>,
}

#[wasm_bindgen]
impl MeshJs {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        Self {
            inner: Mesh::new(),
        }
    }
    
    #[wasm_bindgen(js_name=fromPolygons)]
    pub fn from_polygons(polygons: Vec<PolygonJs>) -> MeshJs { // second parameter: Option<(&str)>
        let poly_vec: Vec<_> = polygons.iter().map(|p| p.inner.clone()).collect();
        //let meta_opt = metadata.map(|s| s.to_string());
        let mesh = Mesh::from_polygons(&poly_vec, None); // add meta_opt in place of None once metadata is bound
        MeshJs { inner: mesh }
    }

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

    #[wasm_bindgen(js_name = vertexCount)]
	pub fn vertex_count(&self) -> u32 {
		let mut vertices = Vec::new();
		for poly in &self.inner.polygons {
			for vertex in &poly.vertices {
				if !vertices.iter().any(|v: &Point3<f64>| {
					(v.x - vertex.pos.x).abs() < 1e-8 &&
					(v.y - vertex.pos.y).abs() < 1e-8 &&
					(v.z - vertex.pos.z).abs() < 1e-8
				}) {
					vertices.push(vertex.pos);
				}
			}
		}
		vertices.len() as u32
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
    
    #[wasm_bindgen(js_name=vertices)]
	pub fn vertices(&self) -> JsValue {
		let verts = self.inner.vertices();
		let js_array = js_sys::Array::new();
		for v in verts {
			let js_vert = Object::new();
			Reflect::set(&js_vert, &"x".into(), &v.pos.x.into()).unwrap();
			Reflect::set(&js_vert, &"y".into(), &v.pos.y.into()).unwrap();
			Reflect::set(&js_vert, &"z".into(), &v.pos.z.into()).unwrap();
			Reflect::set(&js_vert, &"nx".into(), &v.normal.x.into()).unwrap();
			Reflect::set(&js_vert, &"ny".into(), &v.normal.y.into()).unwrap();
			Reflect::set(&js_vert, &"nz".into(), &v.normal.z.into()).unwrap();
			js_array.push(&js_vert);
		}
		js_array.into()
	}
    
    #[wasm_bindgen(js_name=containsVertex)]
	pub fn contains_vertex(&self, x: Real, y: Real, z: Real) -> bool {
		let point = Point3::new(x, y, z);
		self.inner.contains_vertex(&point)
	}

    // Boolean Operations
    #[wasm_bindgen(js_name = union)]
    pub fn union(&self, other: &MeshJs) -> Self {
        Self {
            inner: self.inner.union(&other.inner),
        }
    }

    #[wasm_bindgen(js_name = difference)]
    pub fn difference(&self, other: &MeshJs) -> Self {
        Self {
            inner: self.inner.difference(&other.inner),
        }
    }

    #[wasm_bindgen(js_name = intersection)]
    pub fn intersection(&self, other: &MeshJs) -> Self {
        Self {
            inner: self.inner.intersection(&other.inner),
        }
    }
    
	#[wasm_bindgen(js_name = xor)]
    pub fn xor(&self, other: &MeshJs) -> Self {
        Self {
            inner: self.inner.xor(&other.inner),
        }
    }

    // Transformations
    #[wasm_bindgen(js_name=transform)]
	pub fn transform(&self, m00: Real, m01: Real, m02: Real, m03: Real,
					 m10: Real, m11: Real, m12: Real, m13: Real,
					 m20: Real, m21: Real, m22: Real, m23: Real,
					 m30: Real, m31: Real, m32: Real, m33: Real) -> Self {
		let matrix = Matrix4::new(m00, m01, m02, m03,
								  m10, m11, m12, m13,
								  m20, m21, m22, m23,
								  m30, m31, m32, m33);
		Self { inner: self.inner.transform(&matrix) }
	}
    
    #[wasm_bindgen(js_name = translate)]
    pub fn translate(&self, dx: Real, dy: Real, dz: Real) -> Self {
        Self {
            inner: self.inner.translate(dx, dy, dz),
        }
    }

    #[wasm_bindgen(js_name = rotate)]
    pub fn rotate(&self, rx: Real, ry: Real, rz: Real) -> Self {
        Self {
            inner: self.inner.rotate(rx, ry, rz),
        }
    }

    #[wasm_bindgen(js_name = scale)]
    pub fn scale(&self, sx: Real, sy: Real, sz: Real) -> Self {
        Self {
            inner: self.inner.scale(sx, sy, sz),
        }
    }

    #[wasm_bindgen(js_name = center)]
    pub fn center(&self) -> Self {
        Self {
            inner: self.inner.center(),
        }
    }
    
	#[wasm_bindgen(js_name = float)]
    pub fn float(&self) -> Self {
        Self {
            inner: self.inner.float(),
        }
    }
    
    #[wasm_bindgen(js_name = inverse)]
    pub fn inverse(&self) -> Self {
        Self {
            inner: self.inner.inverse(),
        }
    }
    
    #[cfg(feature = "chull-io")]
	#[wasm_bindgen(js_name=convexHull)]
	pub fn convex_hull(&self) -> Self {
		Self { inner: self.inner.convex_hull() }
	}

	#[cfg(feature = "chull-io")]
	#[wasm_bindgen(js_name=minkowskiSum)]
	pub fn minkowski_sum(&self, other: &MeshJs) -> Self {
		Self { inner: self.inner.minkowski_sum(&other.inner) }
	}
	
	// Distribute functions
	#[wasm_bindgen(js_name=distributeLinear)]
	pub fn distribute_linear(&self, count: usize, dx: Real, dy: Real, dz: Real, spacing: Real) -> Self {
		let direction = Vector3::new(dx, dy, dz);
		Self { inner: self.inner.distribute_linear(count, direction, spacing) }
	}

	#[wasm_bindgen(js_name=distributeArc)]
	pub fn distribute_arc(&self, count: usize, radius: Real, start_angle: Real, end_angle: Real) -> Self {
		Self { inner: self.inner.distribute_arc(count, radius, start_angle, end_angle) }
	}

	#[wasm_bindgen(js_name=distributeGrid)]
	pub fn distribute_grid(&self, rows: usize, cols: usize, row_spacing: Real, col_spacing: Real) -> Self {
		Self { inner: self.inner.distribute_grid(rows, cols, row_spacing, col_spacing) }
	}

    // Bounding Box
    #[wasm_bindgen(js_name = boundingBox)]
    pub fn bounding_box(&self) -> JsValue {
        let bb = self.inner.bounding_box();
        let min = Point3::new(bb.mins.x, bb.mins.y, bb.mins.z);
        let max = Point3::new(bb.maxs.x, bb.maxs.y, bb.maxs.z);
        let obj = Object::new();
        Reflect::set(&obj, &"min".into(), &serde_wasm_bindgen::to_value(&min).unwrap()).unwrap();
        Reflect::set(&obj, &"max".into(), &serde_wasm_bindgen::to_value(&max).unwrap()).unwrap();
        obj.into()
    }
    
	#[wasm_bindgen(js_name=invalidateBoundingBox)]
    pub fn invalidate_bounding_box(&mut self) {
        self.inner.invalidate_bounding_box();
    }

    // IO Operations
    #[wasm_bindgen(js_name = toSTLBinary)]
    pub fn to_stl_binary(&self) -> Result<Vec<u8>, JsValue> {
        self.inner.to_stl_binary("mesh")
            .map_err(|e| JsValue::from_str(&format!("STL export failed: {:?}", e)))
    }

    #[wasm_bindgen(js_name = toSTLASCII)]
    pub fn to_stl_ascii(&self) -> Result<String, JsValue> {
        let stl_content = self.inner.to_stl_ascii("mesh");
        Ok(stl_content)
    }

    #[wasm_bindgen(js_name = toAMF)]
    pub fn to_amf(&self, object_name: &str, units: &str) -> String {
        self.inner.to_amf(object_name, units)
    }

    #[wasm_bindgen(js_name = toAMFWithColor)]
    pub fn to_amf_with_color(&self, object_name: &str, units: &str, r: f64, g: f64, b: f64) -> String {
        self.inner.to_amf_with_color(object_name, units, (r as Real, g as Real, b as Real))
    }
    
    #[wasm_bindgen(js_name=fromSketch)]
    pub fn from_sketch(sketch_js: &SketchJs) -> MeshJs {
        let mesh = Mesh::from(sketch_js.inner.clone());
        Self { inner: mesh }
    }
    
    /*
    // Metadata
    #[wasm_bindgen(js_name=sameMetadata)]
    pub fn same_metadata(&self, metadata: Option<&str>) -> bool {
        let meta_opt = metadata.map(|s| s.to_string());
        self.inner.same_metadata(meta_opt)
    }
    
    #[wasm_bindgen(js_name=filterPolygonsByMetadata)]
    pub fn filter_polygons_by_metadata(&self, needle: &str) -> MeshJs {
        let mesh = self.inner.filter_polygons_by_metadata(needle);
        MeshJs { inner: mesh }
    }
    */

    // Mass Properties
    #[wasm_bindgen(js_name = massProperties)]
    pub fn mass_properties(&self, density: Real) -> JsValue {
        let (mass, com, _frame) = self.inner.mass_properties(density);
        let obj = Object::new();
        Reflect::set(&obj, &"mass".into(), &mass.into()).unwrap();
        Reflect::set(&obj, &"centerOfMass".into(), &serde_wasm_bindgen::to_value(&com).unwrap()).unwrap();
        obj.into()
    }

    // Slicing
    #[wasm_bindgen(js_name = slice)]
    pub fn slice(&self, normal_x: Real, normal_y: Real, normal_z: Real, offset: Real) -> SketchJs {
        let plane = Plane::from_normal(Vector3::new(normal_x, normal_y, normal_z), offset);
        let sketch = self.inner.slice(plane);
        SketchJs { inner: sketch }
    }

    // Subdivision
    #[wasm_bindgen(js_name = subdivideTriangles)]
	pub fn subdivide_triangles(&self, levels: u32) -> Self {
		if levels == 0 {
			return Self {
				inner: self.inner.clone(),
			};
		}
		let levels_nonzero = std::num::NonZeroU32::new(levels).unwrap();
		Self {
			inner: self.inner.subdivide_triangles(levels_nonzero),
		}
	}
	
	#[wasm_bindgen(js_name = renormalize)]
	pub fn renormalize(&self) -> Self {
		let mut inner = self.inner.clone();
		inner.renormalize();
		Self { inner }
	}
	
	#[wasm_bindgen(js_name = triangulate)]
	pub fn triangulate(&self) -> Self {
		Self { inner: self.inner.triangulate() }
	}
	
    // 3D Shapes
    #[wasm_bindgen(js_name = cube)]
    pub fn cube(size: Real) -> Self {
        Self {
            inner: Mesh::cube(size, None),
        }
    }
    
    #[wasm_bindgen(js_name = sphere)]
    pub fn sphere(radius: Real, segments_u: usize, segments_v: usize) -> Self {
        Self {
            inner: Mesh::sphere(radius, segments_u, segments_v, None),
        }
    }

    #[wasm_bindgen(js_name = cylinder)]
    pub fn cylinder(radius: Real, height: Real, segments: usize) -> Self {
        Self {
            inner: Mesh::cylinder(radius, height, segments, None),
        }
    }
}
