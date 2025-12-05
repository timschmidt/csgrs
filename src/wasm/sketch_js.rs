use crate::float_types::Real;
use crate::io::svg::{FromSVG, ToSVG};
use crate::sketch::Sketch;
use crate::traits::CSG;
use crate::wasm::{
    js_metadata_to_string, matrix_js::Matrix4Js, mesh_js::MeshJs, vector_js::Vector3Js,
};
use geo::{Geometry, GeometryCollection};
use js_sys::{Float64Array, Object, Reflect, Uint32Array};
use nalgebra::{Point3, Vector3};
use serde_wasm_bindgen::from_value;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct SketchJs {
    pub(crate) inner: Sketch<String>,
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
    pub fn polygon(points: JsValue, metadata: JsValue) -> Result<Self, JsValue> {
        let points_vec: Vec<[f64; 2]> = from_value(points)
            .map_err(|e| JsValue::from_str(&format!("Failed to parse points: {:?}", e)))?;

        let points_2d: Vec<[Real; 2]> = points_vec
            .into_iter()
            .map(|[x, y]| [x as Real, y as Real])
            .collect();

        let meta = js_metadata_to_string(metadata).unwrap_or(None);

        Ok(Self {
            inner: Sketch::polygon(&points_2d, meta),
        })
    }

    // #[wasm_bindgen(js_name=triangulateWithHoles)]
    // pub fn triangulate_with_holes(outer, holes) -> Vec<JsValue> {
    // let tris = Sketch::<()>::triangulate_with_holes(outer, holes);
    // tris.into_iter()
    // .map(|tri| {
    // let points: Vec<[f64; 3]> = tri
    // .iter()
    // .map(|v| [v.x, v.y, v.z])
    // .collect();
    // JsValue::from_serde(&points).unwrap_or(JsValue::NULL)
    // })
    // .collect()
    // }

    // error[E0609]: no field `pos` on type `&OPoint<f64, Const<3>>`
    // --> src/lib.rs:159:33
    // |
    // 159 |                     .map(|v| [v.pos.x, v.pos.y, v.pos.z])
    // |                                 ^^^ unknown field
    // |
    // = note: available field is: `coords`
    // = note: available fields are: `x`, `y`, `z`
    //
    // #[wasm_bindgen(js_name=triangulate)]
    // pub fn triangulate(&self) -> Vec<JsValue> {
    // let tris = self.inner.triangulate();
    // tris.into_iter()
    // .map(|tri| {
    // let points: Vec<[f64; 3]> = tri
    // .iter()
    // .map(|v| [v.pos.x, v.pos.y, v.pos.z])
    // .collect();
    // JsValue::from_serde(&points).unwrap_or(JsValue::NULL)
    // })
    // .collect()
    // }

    // IO operations
    #[wasm_bindgen(js_name = fromSVG)]
    pub fn from_svg(svg_data: &str, metadata: JsValue) -> Result<Self, JsValue> {
        let meta = js_metadata_to_string(metadata).unwrap_or(None);
        let sketch = Sketch::from_svg(svg_data, meta)
            .map_err(|e| JsValue::from_str(&format!("SVG parsing error: {:?}", e)))?;
        Ok(Self { inner: sketch })
    }

    #[wasm_bindgen(js_name = toSVG)]
    pub fn to_svg(&self) -> String {
        self.inner.to_svg()
    }

    #[wasm_bindgen(js_name=fromGeo)]
    pub fn from_geo(geo_json: &str, metadata: JsValue) -> Result<SketchJs, JsValue> {
        let geometry: Geometry<Real> = serde_json::from_str(geo_json)
            .map_err(|e| JsValue::from_str(&format!("Failed to parse GeoJSON: {}", e)))?;
        let meta = js_metadata_to_string(metadata).unwrap_or(None);
        let sketch = Sketch::from_geo(GeometryCollection(vec![geometry]), meta);
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

    #[wasm_bindgen(js_name=extrudeVectorComponents)]
    pub fn extrude_vector_components(&self, dx: Real, dy: Real, dz: Real) -> MeshJs {
        let direction = Vector3::new(dx, dy, dz);
        let mesh = self.inner.extrude_vector(direction);
        MeshJs { inner: mesh }
    }

    #[wasm_bindgen(js_name = extrudeVector)]
    pub fn extrude_vector(&self, dir: &Vector3Js) -> MeshJs {
        let direction: Vector3<Real> = dir.into();
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
    pub fn square(width: Real, metadata: JsValue) -> Self {
        let meta = js_metadata_to_string(metadata).unwrap_or(None);
        Self {
            inner: Sketch::square(width, meta),
        }
    }

    #[wasm_bindgen(js_name = circle)]
    pub fn circle(radius: Real, segments: usize, metadata: JsValue) -> Self {
        let meta = js_metadata_to_string(metadata).unwrap_or(None);
        Self {
            inner: Sketch::circle(radius, segments, meta),
        }
    }

    #[wasm_bindgen(js_name = rectangle)]
    pub fn rectangle(width: Real, length: Real, metadata: JsValue) -> Self {
        let meta = js_metadata_to_string(metadata).unwrap_or(None);
        Self {
            inner: Sketch::rectangle(width, length, meta),
        }
    }

    #[wasm_bindgen(js_name = rightTriangle)]
    pub fn right_triangle(width: Real, height: Real, metadata: JsValue) -> Self {
        let meta = js_metadata_to_string(metadata).unwrap_or(None);
        Self {
            inner: Sketch::right_triangle(width, height, meta),
        }
    }

    #[wasm_bindgen(js_name = ellipse)]
    pub fn ellipse(width: Real, height: Real, segments: usize, metadata: JsValue) -> Self {
        let meta = js_metadata_to_string(metadata).unwrap_or(None);
        Self {
            inner: Sketch::ellipse(width, height, segments, meta),
        }
    }

    #[wasm_bindgen(js_name = regularNGon)]
    pub fn regular_ngon(sides: usize, radius: Real, metadata: JsValue) -> Self {
        let meta = js_metadata_to_string(metadata).unwrap_or(None);
        Self {
            inner: Sketch::regular_ngon(sides, radius, meta),
        }
    }

    #[wasm_bindgen(js_name = arrow)]
    pub fn arrow(
        shaft_length: Real,
        shaft_width: Real,
        head_length: Real,
        head_width: Real,
        metadata: JsValue,
    ) -> Self {
        let meta = js_metadata_to_string(metadata).unwrap_or(None);
        Self {
            inner: Sketch::arrow(shaft_length, shaft_width, head_length, head_width, meta),
        }
    }

    #[wasm_bindgen(js_name = trapezoid)]
    pub fn trapezoid(
        top_width: Real,
        bottom_width: Real,
        height: Real,
        top_offset: Real,
        metadata: JsValue,
    ) -> Self {
        let meta = js_metadata_to_string(metadata).unwrap_or(None);
        Self {
            inner: Sketch::trapezoid(top_width, bottom_width, height, top_offset, meta),
        }
    }

    #[wasm_bindgen(js_name = star)]
    pub fn star(
        num_points: usize,
        outer_radius: Real,
        inner_radius: Real,
        metadata: JsValue,
    ) -> Self {
        let meta = js_metadata_to_string(metadata).unwrap_or(None);
        Self {
            inner: Sketch::star(num_points, outer_radius, inner_radius, meta),
        }
    }

    #[wasm_bindgen(js_name = teardrop)]
    pub fn teardrop(width: Real, length: Real, segments: usize, metadata: JsValue) -> Self {
        let meta = js_metadata_to_string(metadata).unwrap_or(None);
        Self {
            inner: Sketch::teardrop(width, length, segments, meta),
        }
    }

    #[wasm_bindgen(js_name = egg)]
    pub fn egg(width: Real, length: Real, segments: usize, metadata: JsValue) -> Self {
        let meta = js_metadata_to_string(metadata).unwrap_or(None);
        Self {
            inner: Sketch::egg(width, length, segments, meta),
        }
    }

    #[wasm_bindgen(js_name = roundedRectangle)]
    pub fn rounded_rectangle(
        width: Real,
        height: Real,
        corner_radius: Real,
        corner_segments: usize,
        metadata: JsValue,
    ) -> Self {
        let meta = js_metadata_to_string(metadata).unwrap_or(None);
        Self {
            inner: Sketch::rounded_rectangle(
                width,
                height,
                corner_radius,
                corner_segments,
                meta,
            ),
        }
    }

    #[wasm_bindgen(js_name = squircle)]
    pub fn squircle(width: Real, height: Real, segments: usize, metadata: JsValue) -> Self {
        let meta = js_metadata_to_string(metadata).unwrap_or(None);
        Self {
            inner: Sketch::squircle(width, height, segments, meta),
        }
    }

    #[wasm_bindgen(js_name = keyhole)]
    pub fn keyhole(
        circle_radius: Real,
        handle_width: Real,
        handle_height: Real,
        segments: usize,
        metadata: JsValue,
    ) -> Self {
        let meta = js_metadata_to_string(metadata).unwrap_or(None);
        Self {
            inner: Sketch::keyhole(circle_radius, handle_width, handle_height, segments, meta),
        }
    }

    #[wasm_bindgen(js_name = reuleaux)]
    pub fn reuleaux(
        sides: usize,
        diameter: Real,
        circle_segments: usize,
        metadata: JsValue,
    ) -> Self {
        let meta = js_metadata_to_string(metadata).unwrap_or(None);
        Self {
            inner: Sketch::reuleaux(sides, diameter, circle_segments, meta),
        }
    }

    #[wasm_bindgen(js_name = ring)]
    pub fn ring(id: Real, thickness: Real, segments: usize, metadata: JsValue) -> Self {
        let meta = js_metadata_to_string(metadata).unwrap_or(None);
        Self {
            inner: Sketch::ring(id, thickness, segments, meta),
        }
    }

    #[wasm_bindgen(js_name = pieSlice)]
    pub fn pie_slice(
        radius: Real,
        start_angle_deg: Real,
        end_angle_deg: Real,
        segments: usize,
        metadata: JsValue,
    ) -> Self {
        let meta = js_metadata_to_string(metadata).unwrap_or(None);
        Self {
            inner: Sketch::pie_slice(radius, start_angle_deg, end_angle_deg, segments, meta),
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
        metadata: JsValue,
    ) -> Self {
        let meta = js_metadata_to_string(metadata).unwrap_or(None);
        Self {
            inner: Sketch::supershape(a, b, m, n1, n2, n3, segments, meta),
        }
    }

    #[wasm_bindgen(js_name = circleWithKeyway)]
    pub fn circle_with_keyway(
        radius: Real,
        segments: usize,
        key_width: Real,
        key_depth: Real,
        metadata: JsValue,
    ) -> Self {
        let meta = js_metadata_to_string(metadata).unwrap_or(None);
        Self {
            inner: Sketch::circle_with_keyway(radius, segments, key_width, key_depth, meta),
        }
    }

    #[wasm_bindgen(js_name = circleWithFlat)]
    pub fn circle_with_flat(
        radius: Real,
        segments: usize,
        flat_dist: Real,
        metadata: JsValue,
    ) -> Self {
        let meta = js_metadata_to_string(metadata).unwrap_or(None);
        Self {
            inner: Sketch::circle_with_flat(radius, segments, flat_dist, meta),
        }
    }

    #[wasm_bindgen(js_name = circleWithTwoFlats)]
    pub fn circle_with_two_flats(
        radius: Real,
        segments: usize,
        flat_dist: Real,
        metadata: JsValue,
    ) -> Self {
        let meta = js_metadata_to_string(metadata).unwrap_or(None);
        Self {
            inner: Sketch::circle_with_two_flats(radius, segments, flat_dist, meta),
        }
    }

    #[wasm_bindgen(js_name = bezier)]
    pub fn bezier(
        control: JsValue,
        segments: usize,
        metadata: JsValue,
    ) -> Result<Self, JsValue> {
        let control_vec: Vec<[f64; 2]> = from_value(control).map_err(|e| {
            JsValue::from_str(&format!("Failed to parse control points: {:?}", e))
        })?;

        let control_2d: Vec<[Real; 2]> = control_vec
            .into_iter()
            .map(|[x, y]| [x as Real, y as Real])
            .collect();

        let meta = js_metadata_to_string(metadata).unwrap_or(None);

        Ok(Self {
            inner: Sketch::bezier(&control_2d, segments, meta),
        })
    }

    #[wasm_bindgen(js_name = bspline)]
    pub fn bspline(
        control: JsValue,
        p: usize,
        segments_per_span: usize,
        metadata: JsValue,
    ) -> Result<Self, JsValue> {
        let control_vec: Vec<[f64; 2]> = from_value(control).map_err(|e| {
            JsValue::from_str(&format!("Failed to parse control points: {:?}", e))
        })?;

        let control_2d: Vec<[Real; 2]> = control_vec
            .into_iter()
            .map(|[x, y]| [x as Real, y as Real])
            .collect();

        let meta = js_metadata_to_string(metadata).unwrap_or(None);

        Ok(Self {
            inner: Sketch::bspline(&control_2d, p, segments_per_span, meta),
        })
    }

    #[wasm_bindgen(js_name = heart)]
    pub fn heart(width: Real, height: Real, segments: usize, metadata: JsValue) -> Self {
        let meta = js_metadata_to_string(metadata).unwrap_or(None);
        Self {
            inner: Sketch::heart(width, height, segments, meta),
        }
    }

    #[wasm_bindgen(js_name = crescent)]
    pub fn crescent(
        outer_r: Real,
        inner_r: Real,
        offset: Real,
        segments: usize,
        metadata: JsValue,
    ) -> Self {
        let meta = js_metadata_to_string(metadata).unwrap_or(None);
        Self {
            inner: Sketch::crescent(outer_r, inner_r, offset, segments, meta),
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
        metadata: JsValue,
    ) -> Self {
        let meta = js_metadata_to_string(metadata).unwrap_or(None);
        Self {
            inner: Sketch::involute_gear(
                module_,
                teeth,
                pressure_angle_deg,
                clearance,
                backlash,
                segments_per_flank,
                meta,
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
        metadata: JsValue,
    ) -> Self {
        let meta = js_metadata_to_string(metadata).unwrap_or(None);
        Self {
            inner: Sketch::airfoil_naca4(
                max_camber,
                camber_position,
                thickness,
                chord,
                samples,
                meta,
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
