type Real = f64;
use crate::io::svg::{FromSVG, ToSVG};
//use crate::mesh::metaballs::MetaBall;
use crate::mesh::{Mesh, plane::Plane, polygon::Polygon, vertex::Vertex};
use crate::sketch::Sketch;
use crate::traits::CSG;
use geo::{Geometry, GeometryCollection};
use js_sys::{Float64Array, Object, Reflect, Uint32Array};
use crate::math_ndsp::{Matrix4, Point3, Vector3};
//use serde::{Deserialize, Serialize};
use serde_wasm_bindgen::from_value; //, to_value};
use wasm_bindgen::prelude::*;

// Optional: better panic messages in the browser console.
#[cfg(feature = "console_error_panic_hook")]
#[wasm_bindgen(start)]
pub fn init_panic_hook() {
    console_error_panic_hook::set_once();
}

/*
#[wasm_bindgen]
#[derive(Serialize, Deserialize)]
pub struct MetaBallJs {
    inner: MetaBall,
}

#[wasm_bindgen]
impl MetaBallJs {
    #[wasm_bindgen(constructor)]
    pub fn new(center_x: Real, center_y: Real, center_z: Real, radius: Real) -> Self {
        let center = Point3::new(center_x, center_y, center_z);
        let meta_ball = MetaBall::new(center, radius);
        Self { inner: meta_ball }
    }

    #[wasm_bindgen(getter)]
    pub fn center_x(&self) -> Real {
        self.inner.center.x
    }

    #[wasm_bindgen(getter)]
    pub fn center_y(&self) -> Real {
        self.inner.center.y
    }

    #[wasm_bindgen(getter)]
    pub fn center_z(&self) -> Real {
        self.inner.center.z
    }

    #[wasm_bindgen(getter)]
    pub fn radius(&self) -> Real {
        self.inner.radius
    }
}
*/

#[wasm_bindgen]
pub struct Matrix4Js {
    inner: Matrix4<f64>,
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
        vec![
            self.inner.pos.x as f64,
            self.inner.pos.y as f64,
            self.inner.pos.z as f64,
            self.inner.normal.x as f64,
            self.inner.normal.y as f64,
            self.inner.normal.z as f64,
        ]
    }
}

#[wasm_bindgen]
pub struct PlaneJs {
    inner: Plane,
}

#[wasm_bindgen]
impl PlaneJs {
    // Constructor: Create a plane from three vertices
    #[wasm_bindgen(constructor)]
    pub fn new_from_vertices(
        ax: f64,
        ay: f64,
        az: f64,
        bx: f64,
        by: f64,
        bz: f64,
        cx: f64,
        cy: f64,
        cz: f64,
    ) -> Self {
        let point_a = Point3::new(ax, ay, az);
        let point_b = Point3::new(bx, by, bz);
        let point_c = Point3::new(cx, cy, cz);

        let normal = (point_b - point_a).cross(&(point_c - point_a)).normalize();

        // Convert Points to Vertices with default normals
        let vertex_a = Vertex::new(point_a, normal);
        let vertex_b = Vertex::new(point_b, normal);
        let vertex_c = Vertex::new(point_c, normal);

        let plane = Plane::from_vertices(vec![vertex_a, vertex_b, vertex_c]);
        Self { inner: plane }
    }

    // Constructor: Create a plane from a normal vector and an offset
    #[wasm_bindgen(js_name=newFromNormal)]
    pub fn new_from_normal(nx: f64, ny: f64, nz: f64, offset: f64) -> Self {
        let normal = Vector3::new(nx, ny, nz);
        let plane = Plane::from_normal(normal, offset);
        Self { inner: plane }
    }

    // Get the plane's normal vector as an array [nx, ny, nz]
    #[wasm_bindgen(js_name=normal)]
    pub fn normal(&self) -> JsValue {
        let n = self.inner.normal();
        serde_wasm_bindgen::to_value(&[n.x, n.y, n.z]).unwrap()
    }

    // Get the plane's offset (distance from origin along the normal)
    #[wasm_bindgen(js_name=offset)]
    pub fn offset(&self) -> f64 {
        self.inner.offset()
    }

    // Flip the plane's orientation (negate normal and offset)
    #[wasm_bindgen(js_name=flip)]
    pub fn flip(&mut self) {
        self.inner.flip();
    }

    // Orient a point relative to the plane (FRONT, BACK, COPLANAR)
    #[wasm_bindgen(js_name=orientPoint)]
    pub fn orient_point(&self, x: f64, y: f64, z: f64) -> i8 {
        let point = Point3::new(x, y, z);
        self.inner.orient_point(&point)
    }

    // Orient another plane relative to this plane (FRONT, BACK, COPLANAR, SPANNING)
    #[wasm_bindgen(js_name=orientPlane)]
    pub fn orient_plane(&self, other: &PlaneJs) -> i8 {
        self.inner.orient_plane(&other.inner)
    }

    // Classify a polygon relative to the plane
    #[wasm_bindgen(js_name=classifyPolygon)]
    pub fn classify_polygon(&self, polygon_js: &PolygonJs) -> i8 {
        self.inner.classify_polygon(&polygon_js.inner)
    }

    // Split a polygon with the plane, returning the result as an object
    //#[wasm_bindgen(js_name=splitPolygon)]
    //pub fn split_polygon(&self, polygon_js: &PolygonJs) -> JsValue {}

    // Get the transformation matrices to project this plane onto the XY-plane and back
    #[wasm_bindgen(js_name=toXYTransform)]
    pub fn to_xy_transform(&self) -> JsValue {
        let (to_xy, from_xy) = self.inner.to_xy_transform();

        // Convert Matrix4 to flat arrays for easier JS consumption
        let to_xy_flat: Vec<f64> =
            to_xy.as_slice().iter().copied().map(|v| v as f64).collect();
        let from_xy_flat: Vec<f64> =
            from_xy.as_slice().iter().copied().map(|v| v as f64).collect();

        let obj = js_sys::Object::new();
        js_sys::Reflect::set(
            &obj,
            &"toXY".into(),
            &js_sys::Float64Array::from(to_xy_flat.as_slice()),
        )
        .unwrap();
        js_sys::Reflect::set(
            &obj,
            &"fromXY".into(),
            &js_sys::Float64Array::from(from_xy_flat.as_slice()),
        )
        .unwrap();

        obj.into()
    }
}

#[wasm_bindgen]
pub struct PolygonJs {
    inner: Polygon<()>,
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

    /*
    #[wasm_bindgen(js_name=triangulateWithHoles)]
    pub fn triangulate_with_holes(outer, holes) -> Vec<JsValue> {
        let tris = Sketch::<()>::triangulate_with_holes(outer, holes);
        tris.into_iter()
            .map(|tri| {
                let points: Vec<[f64; 3]> = tri
                    .iter()
                    .map(|v| [v.x, v.y, v.z])
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
        Self {
            inner: self.inner.transform(&mat.inner),
        }
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
        let mesh = self
            .inner
            .revolve(angle_degrees, segments)
            .map_err(|e| JsValue::from_str(&format!("Revolve failed: {:?}", e)))?;
        Ok(MeshJs { inner: mesh })
    }

    #[wasm_bindgen(js_name=extrudeVector)]
    pub fn extrude_vector(&self, dx: Real, dy: Real, dz: Real) -> MeshJs {
        let direction = Vector3::new(dx, dy, dz);
        let mesh = self.inner.extrude_vector(direction);
        MeshJs { inner: mesh }
    }

    #[wasm_bindgen(js_name=sweep)]
    pub fn sweep(&self, path: JsValue) -> MeshJs {
        // Parse the path from a JS array of [x, y, z] coordinates.
        let path_vec: Vec<[f64; 3]> = from_value(path).unwrap_or_else(|_| vec![]);
        let path_points: Vec<Point3<Real>> = path_vec
            .into_iter()
            .map(|[x, y, z]| Point3::new(x as Real, y as Real, z as Real))
            .collect();
        let mesh = self.inner.sweep(&path_points);
        MeshJs { inner: mesh }
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
    pub fn straight_skeleton(&self, orientation: bool) -> SketchJs {
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

    #[wasm_bindgen(js_name = rectangle)]
    pub fn rectangle(width: Real, length: Real) -> Self {
        Self {
            inner: Sketch::rectangle(width, length, None),
        }
    }

    #[wasm_bindgen(js_name = rightTriangle)]
    pub fn right_triangle(width: Real, height: Real) -> Self {
        Self {
            inner: Sketch::right_triangle(width, height, None),
        }
    }

    #[wasm_bindgen(js_name = ellipse)]
    pub fn ellipse(width: Real, height: Real, segments: usize) -> Self {
        Self {
            inner: Sketch::ellipse(width, height, segments, None),
        }
    }

    #[wasm_bindgen(js_name = regularNGon)]
    pub fn regular_ngon(sides: usize, radius: Real) -> Self {
        Self {
            inner: Sketch::regular_ngon(sides, radius, None),
        }
    }
    
    #[wasm_bindgen(js_name = arrow)]
	pub fn arrow(
        shaft_length: Real,
        shaft_width: Real,
        head_length: Real,
        head_width: Real,
    ) -> Self {
		Self {
            inner: Sketch::arrow(shaft_length, shaft_width, head_length, head_width, None),
        }
	}

    #[wasm_bindgen(js_name = trapezoid)]
    pub fn trapezoid(
        top_width: Real,
        bottom_width: Real,
        height: Real,
        top_offset: Real,
    ) -> Self {
        Self {
            inner: Sketch::trapezoid(top_width, bottom_width, height, top_offset, None),
        }
    }

    #[wasm_bindgen(js_name = star)]
    pub fn star(num_points: usize, outer_radius: Real, inner_radius: Real) -> Self {
        Self {
            inner: Sketch::star(num_points, outer_radius, inner_radius, None),
        }
    }

    #[wasm_bindgen(js_name = teardrop)]
    pub fn teardrop(width: Real, length: Real, segments: usize) -> Self {
        Self {
            inner: Sketch::teardrop(width, length, segments, None),
        }
    }

    #[wasm_bindgen(js_name = egg)]
    pub fn egg(width: Real, length: Real, segments: usize) -> Self {
        Self {
            inner: Sketch::egg(width, length, segments, None),
        }
    }

    #[wasm_bindgen(js_name = roundedRectangle)]
    pub fn rounded_rectangle(
        width: Real,
        height: Real,
        corner_radius: Real,
        corner_segments: usize,
    ) -> Self {
        Self {
            inner: Sketch::rounded_rectangle(
                width,
                height,
                corner_radius,
                corner_segments,
                None,
            ),
        }
    }

    #[wasm_bindgen(js_name = squircle)]
    pub fn squircle(width: Real, height: Real, segments: usize) -> Self {
        Self {
            inner: Sketch::squircle(width, height, segments, None),
        }
    }

    #[wasm_bindgen(js_name = keyhole)]
    pub fn keyhole(
        circle_radius: Real,
        handle_width: Real,
        handle_height: Real,
        segments: usize,
    ) -> Self {
        Self {
            inner: Sketch::keyhole(circle_radius, handle_width, handle_height, segments, None),
        }
    }

    #[wasm_bindgen(js_name = reuleaux)]
    pub fn reuleaux(sides: usize, diameter: Real, circle_segments: usize) -> Self {
        Self {
            inner: Sketch::reuleaux(sides, diameter, circle_segments, None),
        }
    }

    #[wasm_bindgen(js_name = ring)]
    pub fn ring(id: Real, thickness: Real, segments: usize) -> Self {
        Self {
            inner: Sketch::ring(id, thickness, segments, None),
        }
    }

    #[wasm_bindgen(js_name = pieSlice)]
    pub fn pie_slice(
        radius: Real,
        start_angle_deg: Real,
        end_angle_deg: Real,
        segments: usize,
    ) -> Self {
        Self {
            inner: Sketch::pie_slice(radius, start_angle_deg, end_angle_deg, segments, None),
        }
    }

    #[wasm_bindgen(js_name = supershape)]
    pub fn supershape(
        a: Real,
        b: Real,
        m: Real,
        n1: Real,
        n2: Real,
        n3: Real,
        segments: usize,
    ) -> Self {
        Self {
            inner: Sketch::supershape(a, b, m, n1, n2, n3, segments, None),
        }
    }

    #[wasm_bindgen(js_name = circleWithKeyway)]
    pub fn circle_with_keyway(
        radius: Real,
        segments: usize,
        key_width: Real,
        key_depth: Real,
    ) -> Self {
        Self {
            inner: Sketch::circle_with_keyway(radius, segments, key_width, key_depth, None),
        }
    }

    #[wasm_bindgen(js_name = circleWithFlat)]
    pub fn circle_with_flat(radius: Real, segments: usize, flat_dist: Real) -> Self {
        Self {
            inner: Sketch::circle_with_flat(radius, segments, flat_dist, None),
        }
    }

    #[wasm_bindgen(js_name = circleWithTwoFlats)]
    pub fn circle_with_two_flats(radius: Real, segments: usize, flat_dist: Real) -> Self {
        Self {
            inner: Sketch::circle_with_two_flats(radius, segments, flat_dist, None),
        }
    }

    #[wasm_bindgen(js_name = bezier)]
    pub fn bezier(control: JsValue, segments: usize) -> Result<Self, JsValue> {
        let control_vec: Vec<[f64; 2]> = from_value(control).map_err(|e| {
            JsValue::from_str(&format!("Failed to parse control points: {:?}", e))
        })?;

        let control_2d: Vec<[Real; 2]> = control_vec
            .into_iter()
            .map(|[x, y]| [x as Real, y as Real])
            .collect();

        Ok(Self {
            inner: Sketch::bezier(&control_2d, segments, None),
        })
    }

    #[wasm_bindgen(js_name = bspline)]
    pub fn bspline(
        control: JsValue,
        p: usize,
        segments_per_span: usize,
    ) -> Result<Self, JsValue> {
        let control_vec: Vec<[f64; 2]> = from_value(control).map_err(|e| {
            JsValue::from_str(&format!("Failed to parse control points: {:?}", e))
        })?;

        let control_2d: Vec<[Real; 2]> = control_vec
            .into_iter()
            .map(|[x, y]| [x as Real, y as Real])
            .collect();

        Ok(Self {
            inner: Sketch::bspline(&control_2d, p, segments_per_span, None),
        })
    }

    #[wasm_bindgen(js_name = heart)]
    pub fn heart(width: Real, height: Real, segments: usize) -> Self {
        Self {
            inner: Sketch::heart(width, height, segments, None),
        }
    }

    #[wasm_bindgen(js_name = crescent)]
    pub fn crescent(outer_r: Real, inner_r: Real, offset: Real, segments: usize) -> Self {
        Self {
            inner: Sketch::crescent(outer_r, inner_r, offset, segments, None),
        }
    }

    #[wasm_bindgen(js_name = involuteGear)]
    pub fn involute_gear(
        module_: Real,
        teeth: usize,
        pressure_angle_deg: Real,
        clearance: Real,
        backlash: Real,
        segments_per_flank: usize,
    ) -> Self {
        Self {
            inner: Sketch::involute_gear(
                module_,
                teeth,
                pressure_angle_deg,
                clearance,
                backlash,
                segments_per_flank,
                None,
            ),
        }
    }

    #[wasm_bindgen(js_name = airfoilNACA4)]
    pub fn airfoil_naca4(
        max_camber: Real,
        camber_position: Real,
        thickness: Real,
        chord: Real,
        samples: usize,
    ) -> Self {
        Self {
            inner: Sketch::airfoil_naca4(
                max_camber,
                camber_position,
                thickness,
                chord,
                samples,
                None,
            ),
        }
    }

    #[cfg(feature = "offset")]
    #[wasm_bindgen(js_name = hilbertCurve)]
    pub fn hilbert_curve(&self, order: usize, padding: Real) -> Self {
        Self {
            inner: self.inner.hilbert_curve(order, padding),
        }
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
        Self { inner: Mesh::new() }
    }

    #[wasm_bindgen(js_name=fromPolygons)]
    pub fn from_polygons(polygons: Vec<PolygonJs>) -> MeshJs {
        // second parameter: Option<(&str)>
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
                    (v.x - vertex.pos.x).abs() < 1e-8
                        && (v.y - vertex.pos.y).abs() < 1e-8
                        && (v.z - vertex.pos.z).abs() < 1e-8
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
        let mut normals = Vec::with_capacity(tri_count * 3 * 3);
        let mut indices = Vec::with_capacity(tri_count * 3);

        let mut idx: u32 = 0;
        for p in &tri.polygons {
            for v in &p.vertices {
                positions.extend_from_slice(&[v.pos.x as f64, v.pos.y as f64, v.pos.z as f64]);
                normals.extend_from_slice(&[
                    v.normal.x as f64,
                    v.normal.y as f64,
                    v.normal.z as f64,
                ]);
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
    pub fn transform(
        &self,
        m00: Real,
        m01: Real,
        m02: Real,
        m03: Real,
        m10: Real,
        m11: Real,
        m12: Real,
        m13: Real,
        m20: Real,
        m21: Real,
        m22: Real,
        m23: Real,
        m30: Real,
        m31: Real,
        m32: Real,
        m33: Real,
    ) -> Self {
        let matrix = Matrix4::new(
            m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33,
        );
        Self {
            inner: self.inner.transform(&matrix),
        }
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
        Self {
            inner: self.inner.convex_hull(),
        }
    }

    #[cfg(feature = "chull-io")]
    #[wasm_bindgen(js_name=minkowskiSum)]
    pub fn minkowski_sum(&self, other: &MeshJs) -> Self {
        Self {
            inner: self.inner.minkowski_sum(&other.inner),
        }
    }

    #[wasm_bindgen(js_name=flatten)]
    pub fn flatten(&self) -> SketchJs {
        let sketch = self.inner.flatten();
        SketchJs { inner: sketch }
    }

    #[wasm_bindgen(js_name=slice)]
    pub fn slice(
        &self,
        normal_x: Real,
        normal_y: Real,
        normal_z: Real,
        offset: Real,
    ) -> SketchJs {
        let plane = Plane::from_normal(Vector3::new(normal_x, normal_y, normal_z), offset);
        let sketch = self.inner.slice(plane);
        SketchJs { inner: sketch }
    }

    #[wasm_bindgen(js_name=laplacianSmooth)]
    pub fn laplacian_smooth(
        &self,
        lambda: Real,
        iterations: usize,
        preserve_boundaries: bool,
    ) -> Self {
        let smoothed = self
            .inner
            .laplacian_smooth(lambda, iterations, preserve_boundaries);
        Self { inner: smoothed }
    }

    #[wasm_bindgen(js_name=taubinSmooth)]
    pub fn taubin_smooth(
        &self,
        lambda: Real,
        mu: Real,
        iterations: usize,
        preserve_boundaries: bool,
    ) -> Self {
        let smoothed = self
            .inner
            .taubin_smooth(lambda, mu, iterations, preserve_boundaries);
        Self { inner: smoothed }
    }

    #[wasm_bindgen(js_name=adaptiveRefine)]
    pub fn adaptive_refine(
        &self,
        quality_threshold: Real,
        max_edge_length: Real,
        curvature_threshold_deg: Real,
    ) -> Self {
        let refined = self.inner.adaptive_refine(
            quality_threshold,
            max_edge_length,
            curvature_threshold_deg,
        );
        Self { inner: refined }
    }

    #[wasm_bindgen(js_name=removePoorTriangles)]
    pub fn remove_poor_triangles(&self, min_quality: Real) -> Self {
        let cleaned = self.inner.remove_poor_triangles(min_quality);
        Self { inner: cleaned }
    }

    // Distribute functions
    #[wasm_bindgen(js_name=distributeLinear)]
    pub fn distribute_linear(
        &self,
        count: usize,
        dx: Real,
        dy: Real,
        dz: Real,
        spacing: Real,
    ) -> Self {
        let direction = Vector3::new(dx, dy, dz);
        Self {
            inner: self.inner.distribute_linear(count, direction, spacing),
        }
    }

    #[wasm_bindgen(js_name=distributeArc)]
    pub fn distribute_arc(
        &self,
        count: usize,
        radius: Real,
        start_angle: Real,
        end_angle: Real,
    ) -> Self {
        Self {
            inner: self
                .inner
                .distribute_arc(count, radius, start_angle, end_angle),
        }
    }

    #[wasm_bindgen(js_name=distributeGrid)]
    pub fn distribute_grid(
        &self,
        rows: usize,
        cols: usize,
        row_spacing: Real,
        col_spacing: Real,
    ) -> Self {
        Self {
            inner: self
                .inner
                .distribute_grid(rows, cols, row_spacing, col_spacing),
        }
    }

    // Bounding Box
    #[wasm_bindgen(js_name = boundingBox)]
    pub fn bounding_box(&self) -> JsValue {
        let bb = self.inner.bounding_box();
        let min = Point3::new(bb.mins.x, bb.mins.y, bb.mins.z);
        let max = Point3::new(bb.maxs.x, bb.maxs.y, bb.maxs.z);
        let obj = Object::new();
        Reflect::set(
            &obj,
            &"min".into(),
            &serde_wasm_bindgen::to_value(&min).unwrap(),
        )
        .unwrap();
        Reflect::set(
            &obj,
            &"max".into(),
            &serde_wasm_bindgen::to_value(&max).unwrap(),
        )
        .unwrap();
        obj.into()
    }

    #[wasm_bindgen(js_name=invalidateBoundingBox)]
    pub fn invalidate_bounding_box(&mut self) {
        self.inner.invalidate_bounding_box();
    }

    // IO Operations
    #[wasm_bindgen(js_name = toSTLBinary)]
    pub fn to_stl_binary(&self) -> Result<Vec<u8>, JsValue> {
        self.inner
            .to_stl_binary("mesh")
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
    pub fn to_amf_with_color(
        &self,
        object_name: &str,
        units: &str,
        r: f64,
        g: f64,
        b: f64,
    ) -> String {
        self.inner
            .to_amf_with_color(object_name, units, (r as Real, g as Real, b as Real))
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
        Reflect::set(
            &obj,
            &"centerOfMass".into(),
            &serde_wasm_bindgen::to_value(&com).unwrap(),
        )
        .unwrap();
        obj.into()
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
        Self {
            inner: self.inner.triangulate(),
        }
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

    #[wasm_bindgen(js_name = cuboid)]
    pub fn cuboid(width: Real, length: Real, height: Real) -> Self {
        Self {
            inner: Mesh::cuboid(width, length, height, None),
        }
    }

    #[wasm_bindgen(js_name = frustum_ptp)]
    pub fn frustum_ptp(
        start_x: Real,
        start_y: Real,
        start_z: Real,
        end_x: Real,
        end_y: Real,
        end_z: Real,
        radius1: Real,
        radius2: Real,
        segments: usize,
    ) -> Self {
        let start = Point3::new(start_x, start_y, start_z);
        let end = Point3::new(end_x, end_y, end_z);
        Self {
            inner: Mesh::frustum_ptp(start, end, radius1, radius2, segments, None),
        }
    }

    #[wasm_bindgen(js_name = frustum)]
    pub fn frustum(radius1: Real, radius2: Real, height: Real, segments: usize) -> Self {
        Self {
            inner: Mesh::frustum(radius1, radius2, height, segments, None),
        }
    }

    #[wasm_bindgen(js_name = polyhedron)]
    pub fn polyhedron(points: JsValue, faces: JsValue) -> Result<Self, JsValue> {
        let points_vec: Vec<[f64; 3]> = from_value(points)
            .map_err(|e| JsValue::from_str(&format!("Failed to parse points: {:?}", e)))?;
        let faces_vec: Vec<Vec<usize>> = from_value(faces)
            .map_err(|e| JsValue::from_str(&format!("Failed to parse faces: {:?}", e)))?;

        let points_3d: Vec<[Real; 3]> = points_vec
            .into_iter()
            .map(|[x, y, z]| [x as Real, y as Real, z as Real])
            .collect();

        let faces_ref: Vec<&[usize]> = faces_vec.iter().map(|f| f.as_slice()).collect();

        let mesh = Mesh::polyhedron(&points_3d, &faces_ref, None)
            .map_err(|e| JsValue::from_str(&format!("Polyhedron creation failed: {:?}", e)))?;

        Ok(Self { inner: mesh })
    }

    #[wasm_bindgen(js_name = egg)]
    pub fn egg(
        width: Real,
        length: Real,
        revolve_segments: usize,
        outline_segments: usize,
    ) -> Self {
        Self {
            inner: Mesh::egg(width, length, revolve_segments, outline_segments, None),
        }
    }

    #[wasm_bindgen(js_name = teardrop)]
    pub fn teardrop(
        width: Real,
        length: Real,
        revolve_segments: usize,
        shape_segments: usize,
    ) -> Self {
        Self {
            inner: Mesh::teardrop(width, length, revolve_segments, shape_segments, None),
        }
    }

    #[wasm_bindgen(js_name = teardrop_cylinder)]
    pub fn teardrop_cylinder(
        width: Real,
        length: Real,
        height: Real,
        shape_segments: usize,
    ) -> Self {
        Self {
            inner: Mesh::teardrop_cylinder(width, length, height, shape_segments, None),
        }
    }

    #[wasm_bindgen(js_name = ellipsoid)]
    pub fn ellipsoid(rx: Real, ry: Real, rz: Real, segments: usize, stacks: usize) -> Self {
        Self {
            inner: Mesh::ellipsoid(rx, ry, rz, segments, stacks, None),
        }
    }

    #[wasm_bindgen(js_name = arrow)]
    pub fn arrow(
        start_x: Real,
        start_y: Real,
        start_z: Real,
        dir_x: Real,
        dir_y: Real,
        dir_z: Real,
        segments: usize,
        orientation: bool,
    ) -> Self {
        let start = Point3::new(start_x, start_y, start_z);
        let direction = Vector3::new(dir_x, dir_y, dir_z);
        Self {
            inner: Mesh::arrow(start, direction, segments, orientation, None),
        }
    }

    #[wasm_bindgen(js_name = octahedron)]
    pub fn octahedron(radius: Real) -> Self {
        Self {
            inner: Mesh::octahedron(radius, None),
        }
    }

    #[wasm_bindgen(js_name = icosahedron)]
    pub fn icosahedron(radius: Real) -> Self {
        Self {
            inner: Mesh::icosahedron(radius, None),
        }
    }

    #[wasm_bindgen(js_name = torus)]
    pub fn torus(
        major_r: Real,
        minor_r: Real,
        segments_major: usize,
        segments_minor: usize,
    ) -> Self {
        Self {
            inner: Mesh::torus(major_r, minor_r, segments_major, segments_minor, None),
        }
    }

    #[wasm_bindgen(js_name = spur_gear_involute)]
    pub fn spur_gear_involute(
        module_: Real,
        teeth: usize,
        pressure_angle_deg: Real,
        clearance: Real,
        backlash: Real,
        segments_per_flank: usize,
        thickness: Real,
    ) -> Self {
        Self {
            inner: Mesh::spur_gear_involute(
                module_,
                teeth,
                pressure_angle_deg,
                clearance,
                backlash,
                segments_per_flank,
                thickness,
                None,
            ),
        }
    }

    #[wasm_bindgen(js_name=gyroid)]
    pub fn gyroid(&self, resolution: u32, scale: Real, iso_value: Real) -> Self {
        let gyroid_mesh =
            self.inner
                .gyroid(resolution.try_into().unwrap(), scale, iso_value, None);
        Self { inner: gyroid_mesh }
    }

    #[wasm_bindgen(js_name=schwarzP)]
    pub fn schwarz_p(&self, resolution: u32, scale: Real, iso_value: Real) -> Self {
        let schwarzp_mesh =
            self.inner
                .schwarz_p(resolution.try_into().unwrap(), scale, iso_value, None);
        Self {
            inner: schwarzp_mesh,
        }
    }

    #[wasm_bindgen(js_name=schwarzD)]
    pub fn schwarz_d(&self, resolution: u32, scale: Real, iso_value: Real) -> Self {
        let schwarzd_mesh =
            self.inner
                .schwarz_d(resolution.try_into().unwrap(), scale, iso_value, None);
        Self {
            inner: schwarzd_mesh,
        }
    }

    /*
    #[wasm_bindgen(js_name=metaballs)]
    pub fn metaballs(balls: JsValue, resolution_x: u32, resolution_y: u32, resolution_z: u32, iso_value: Real, padding: Real) -> Self {
        // Parse the list of MetaBallJs objects or raw data.
        let balls_vec: Vec<MetaBallJs> = from_value(balls).unwrap_or_else(|_| vec![]);
        let meta_balls: Vec<MetaBall> = balls_vec.into_iter().map(|b| b.inner).collect();

        let resolution = (resolution_x.try_into().unwrap(), resolution_y.try_into().unwrap(), resolution_z.try_into().unwrap());
        let metaball_mesh = Mesh::metaballs(&meta_balls, resolution, iso_value, padding, None);
        Self { inner: metaball_mesh }
    }
    */
}
