//! JavaScript wrapper for [`Profile`].

use crate::csg::CSG;
use crate::hyper_math::Real;
use crate::io::svg::{FromSVG, ToSVG};
use crate::sketch::Profile;
use crate::wasm::{
    finite_matrix4, js_metadata, matrix_js::Matrix4Js, mesh_js::MeshJs, point_js::Point3Js,
    vector_js::Vector3Js,
};
use crate::wasm::{real_from_js, real_from_js_or_zero, real_to_js};
use hypercurve::{Contour2, CurveString2};
use hyperlattice::{Point3, Vector3};
use js_sys::{Float64Array, Object, Reflect, Uint32Array};
use serde::{Deserialize, Serialize};
use serde_wasm_bindgen::{from_value, to_value};
use wasm_bindgen::prelude::*;

#[derive(Debug, Deserialize, Serialize)]
#[serde(rename_all = "camelCase")]
struct RegionProfileJs {
    material: Vec<[f64; 2]>,
    holes: Vec<Vec<[f64; 2]>>,
}

fn promote_ring(points: Vec<[f64; 2]>, label: &str) -> Result<Vec<[Real; 2]>, JsValue> {
    points
        .into_iter()
        .enumerate()
        .map(|(index, [x, y])| {
            let x = real_from_js(x).ok_or_else(|| {
                JsValue::from_str(&format!("{label}[{index}] has invalid x"))
            })?;
            let y = real_from_js(y).ok_or_else(|| {
                JsValue::from_str(&format!("{label}[{index}] has invalid y"))
            })?;
            Ok([x, y])
        })
        .collect()
}

fn promote_polyline(points: Vec<[f64; 2]>, label: &str) -> Result<Vec<[Real; 2]>, JsValue> {
    points
        .into_iter()
        .enumerate()
        .map(|(index, [x, y])| {
            let x = real_from_js(x).ok_or_else(|| {
                JsValue::from_str(&format!("{label}[{index}] has invalid x"))
            })?;
            let y = real_from_js(y).ok_or_else(|| {
                JsValue::from_str(&format!("{label}[{index}] has invalid y"))
            })?;
            Ok([x, y])
        })
        .collect()
}

#[wasm_bindgen(js_name = ProfileJs)]
pub struct SketchJs {
    pub(crate) inner: Profile,
}

#[wasm_bindgen]
impl SketchJs {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        Self {
            inner: Profile::empty(),
        }
    }

    #[wasm_bindgen(js_name = isEmpty)]
    pub fn is_empty(&self) -> bool {
        self.inner.is_empty()
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
            positions.push(real_to_js(&a.x));
            positions.push(real_to_js(&a.y));
            positions.push(0.0);
            positions.push(real_to_js(&b.x));
            positions.push(real_to_js(&b.y));
            positions.push(0.0);
            positions.push(real_to_js(&c.x));
            positions.push(real_to_js(&c.y));
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

    /// Project the native hypercurve filled region to JS polygon profiles.
    ///
    /// The result is `[{ material: [[x, y], ...], holes: [[[x, y], ...], ...] }]`.
    /// It is the wasm-facing form of hypercurve's finite projection records, not
    /// a compatibility-cache object. The split between exact internal topology and finite
    /// boundary output follows Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>), and the finite-output
    /// discipline in Hobby, "Practical Segment Intersection with Finite Precision
    /// Output," *Computational Geometry* 13(4), 1999
    /// (<https://doi.org/10.1016/S0925-7721(99)00021-8>).
    #[wasm_bindgen(js_name = regionProfiles)]
    pub fn region_profiles(&self) -> Result<JsValue, JsValue> {
        let profiles = self
            .inner
            .region_profiles()
            .into_iter()
            .map(|profile| RegionProfileJs {
                material: profile.material().points().to_vec(),
                holes: profile
                    .holes()
                    .iter()
                    .map(|hole| hole.points().to_vec())
                    .collect(),
            })
            .collect::<Vec<_>>();
        to_value(&profiles).map_err(|e| {
            JsValue::from_str(&format!("Failed to serialize region profiles: {e}"))
        })
    }

    /// Project native hypercurve wires to JS polylines.
    ///
    /// The result is `[[[x, y], ...], ...]`, sampled from `CurveString2` via
    /// hypercurve's finite projection machinery. Keeping path ownership in
    /// hypercurve and projecting only at the wasm edge follows Yap, "Towards
    /// Exact Geometric Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    #[wasm_bindgen(js_name = wirePolylines)]
    pub fn wire_polylines(&self) -> Result<JsValue, JsValue> {
        let polylines = self
            .inner
            .wire_polylines()
            .into_iter()
            .map(|polyline| {
                polyline
                    .into_iter()
                    .map(|point| [real_to_js(&point[0]), real_to_js(&point[1])])
                    .collect::<Vec<_>>()
            })
            .collect::<Vec<_>>();
        to_value(&polylines).map_err(|e| {
            JsValue::from_str(&format!("Failed to serialize wire polylines: {e}"))
        })
    }

    /// Project native hypercurve wires to flat render arrays.
    ///
    /// The returned object is `{ positions, indices }`, where `positions` is a
    /// flat `Float64Array` of XYZ points and `indices` is a `Uint32Array` of
    /// line segment endpoints. It is a finite wasm/rendering boundary over
    /// [`hypercurve::CurveString2`] ownership, not a compatibility geometry
    /// cache. That keeps path topology in hyperreal-backed hypercurve objects
    /// until the JS edge, following Yap, "Towards Exact Geometric Computation,"
    /// *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>), and Hobby,
    /// "Practical Segment Intersection with Finite Precision Output,"
    /// *Computational Geometry* 13(4), 1999
    /// (<https://doi.org/10.1016/S0925-7721(99)00021-8>).
    #[wasm_bindgen(js_name = wireArrays)]
    pub fn wire_arrays(&self) -> JsValue {
        let mut positions = Vec::new();
        let mut indices = Vec::new();
        let mut next_index = 0_u32;

        for polyline in self.inner.wire_polylines() {
            if polyline.len() < 2 {
                continue;
            }
            for point in &polyline {
                positions.push(real_to_js(&point[0]));
                positions.push(real_to_js(&point[1]));
                positions.push(0.0);
            }
            for segment in 0..polyline.len().saturating_sub(1) {
                let start = next_index + segment as u32;
                indices.push(start);
                indices.push(start + 1);
            }
            next_index += polyline.len() as u32;
        }

        let pos_array = Float64Array::from(positions.as_slice());
        let idx_array = Uint32Array::from(indices.as_slice());

        let obj = Object::new();
        Reflect::set(&obj, &"positions".into(), &pos_array).unwrap();
        Reflect::set(&obj, &"indices".into(), &idx_array).unwrap();
        obj.into()
    }

    #[wasm_bindgen(js_name = polygon)]
    pub fn polygon(points: JsValue) -> Result<Self, JsValue> {
        let points_vec: Vec<[f64; 2]> = from_value(points)
            .map_err(|e| JsValue::from_str(&format!("Failed to parse points: {:?}", e)))?;

        let points_2d: Vec<[Real; 2]> = points_vec
            .into_iter()
            .map(|[x, y]| {
                let x = real_from_js(x)
                    .ok_or_else(|| JsValue::from_str("Polygon point contains invalid x"))?;
                let y = real_from_js(y)
                    .ok_or_else(|| JsValue::from_str("Polygon point contains invalid y"))?;
                Ok([x, y])
            })
            .collect::<Result<_, JsValue>>()?;
        let contour = Contour2::from_real_ring(&points_2d)
            .map_err(|_| JsValue::from_str("Invalid polygon ring"))?;

        Ok(Self {
            inner: Profile::from_contour(contour),
        })
    }

    /// Build a Profile directly from hypercurve-style finite region profiles.
    ///
    /// Input shape is `[{ material: [[x, y], ...], holes: [[[x, y], ...], ...] }]`.
    /// Each finite ring is immediately promoted to a `hypercurve::Contour2`, and
    /// the Profile is backed by `hypercurve::Region2` rather than a GeoJSON cache.
    /// This keeps ordinary `f64` values at the wasm boundary while internal CAD
    /// topology is owned by hyper geometry, following Yap, "Towards Exact
    /// Geometric Computation," *Computational Geometry* 7(1-2), 1997
    /// (<https://doi.org/10.1016/0925-7721(95)00040-2>).
    #[wasm_bindgen(js_name = fromRegionProfiles)]
    pub fn from_region_profiles(profiles: JsValue) -> Result<Self, JsValue> {
        let profiles: Vec<RegionProfileJs> = from_value(profiles).map_err(|e| {
            JsValue::from_str(&format!("Failed to parse region profiles: {e}"))
        })?;
        let mut material = Vec::new();
        let mut holes = Vec::new();
        for profile in profiles {
            let material_ring = promote_ring(profile.material, "material")?;
            let contour = Contour2::from_real_ring(material_ring.as_slice())
                .map_err(|_| JsValue::from_str("Invalid material ring"))?;
            material.push(contour);
            for (hole_index, hole) in profile.holes.into_iter().enumerate() {
                let hole_ring = promote_ring(hole, &format!("holes[{hole_index}]"))?;
                let contour = Contour2::from_real_ring(hole_ring.as_slice())
                    .map_err(|_| JsValue::from_str("Invalid hole ring"))?;
                holes.push(contour);
            }
        }
        Ok(Self {
            inner: Profile::from_region(hypercurve::Region2::new(material, holes)),
        })
    }

    /// Build a wire-only Profile directly from finite JS polylines.
    ///
    /// Input shape is `[[[x, y], ...], ...]`. Each finite polyline is immediately
    /// promoted to `hypercurve::CurveString2`; unsupported or degenerate
    /// polylines are rejected instead of being retained as hidden compatibility
    /// compatibility state.
    #[wasm_bindgen(js_name = fromWirePolylines)]
    pub fn from_wire_polylines(polylines: JsValue) -> Result<Self, JsValue> {
        let polylines: Vec<Vec<[f64; 2]>> = from_value(polylines)
            .map_err(|e| JsValue::from_str(&format!("Failed to parse wire polylines: {e}")))?;
        let mut wires = Vec::with_capacity(polylines.len());
        for (polyline_index, polyline) in polylines.into_iter().enumerate() {
            let polyline =
                promote_polyline(polyline, &format!("polylines[{polyline_index}]"))?;
            let wire = CurveString2::from_real_point_iter(polyline)
                .map_err(|_| JsValue::from_str("Invalid wire polyline"))?;
            wires.push(wire);
        }
        Ok(Self {
            inner: Profile::from_wires(wires),
        })
    }

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
    pub fn from_svg(svg_data: &str) -> Result<Self, JsValue> {
        let sketch = Profile::from_svg(svg_data)
            .map_err(|e| JsValue::from_str(&format!("SVG parsing error: {:?}", e)))?;
        Ok(Self { inner: sketch })
    }

    #[wasm_bindgen(js_name = toSVG)]
    pub fn to_svg(&self) -> Result<String, JsValue> {
        self.inner
            .to_svg()
            .map_err(|error| JsValue::from_str(&error.to_string()))
    }

    #[wasm_bindgen(js_name=fromMesh)]
    pub fn from_mesh(mesh_js: &MeshJs) -> SketchJs {
        let sketch = Profile::from(mesh_js.inner.clone());
        SketchJs { inner: sketch }
    }

    // Boolean Operations
    #[wasm_bindgen(js_name = union)]
    pub fn union(&self, other: &SketchJs) -> Result<Self, JsValue> {
        self.inner
            .try_union(&other.inner)
            .map(|inner| Self { inner })
            .map_err(|error| JsValue::from_str(&error.to_string()))
    }

    #[wasm_bindgen(js_name = difference)]
    pub fn difference(&self, other: &SketchJs) -> Result<Self, JsValue> {
        self.inner
            .try_difference(&other.inner)
            .map(|inner| Self { inner })
            .map_err(|error| JsValue::from_str(&error.to_string()))
    }

    #[wasm_bindgen(js_name = intersection)]
    pub fn intersection(&self, other: &SketchJs) -> Result<Self, JsValue> {
        self.inner
            .try_intersection(&other.inner)
            .map(|inner| Self { inner })
            .map_err(|error| JsValue::from_str(&error.to_string()))
    }

    #[wasm_bindgen(js_name = xor)]
    pub fn xor(&self, other: &SketchJs) -> Result<Self, JsValue> {
        self.inner
            .try_xor(&other.inner)
            .map(|inner| Self { inner })
            .map_err(|error| JsValue::from_str(&error.to_string()))
    }

    // Transformations
    #[wasm_bindgen(js_name=transform)]
    pub fn transform(&self, mat: &Matrix4Js) -> SketchJs {
        Self {
            inner: self.inner.transform(&mat.inner),
        }
    }

    #[wasm_bindgen(js_name = transformComponents)]
    pub fn transform_components(
        &self,
        m00: f64,
        m01: f64,
        m02: f64,
        m03: f64,
        m10: f64,
        m11: f64,
        m12: f64,
        m13: f64,
        m20: f64,
        m21: f64,
        m22: f64,
        m23: f64,
        m30: f64,
        m31: f64,
        m32: f64,
        m33: f64,
    ) -> SketchJs {
        let raw = [
            m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23, m30, m31, m32, m33,
        ];
        let mut values: [Real; 16] = std::array::from_fn(|_| Real::zero());
        for (slot, value) in values.iter_mut().zip(raw) {
            let Some(value) = real_from_js(value) else {
                return SketchJs {
                    inner: self.inner.clone(),
                };
            };
            *slot = value;
        }
        let Some(matrix) = finite_matrix4(values) else {
            return SketchJs {
                inner: self.inner.clone(),
            };
        };
        SketchJs {
            inner: self.inner.transform(&matrix),
        }
    }

    #[wasm_bindgen(js_name = translate)]
    pub fn translate(&self, offset: &Vector3Js) -> Self {
        let v: Vector3 = offset.into();
        Self {
            inner: self
                .inner
                .translate(v.0[0].clone(), v.0[1].clone(), v.0[2].clone()),
        }
    }

    #[wasm_bindgen(js_name = translateComponents)]
    pub fn translate_components(&self, dx: f64, dy: f64, dz: f64) -> Self {
        Self {
            inner: self.inner.translate(
                real_from_js_or_zero(dx),
                real_from_js_or_zero(dy),
                real_from_js_or_zero(dz),
            ),
        }
    }

    #[wasm_bindgen(js_name = rotate)]
    pub fn rotate(&self, rx: f64, ry: f64, rz: f64) -> Self {
        Self {
            inner: self.inner.rotate(
                real_from_js_or_zero(rx),
                real_from_js_or_zero(ry),
                real_from_js_or_zero(rz),
            ),
        }
    }

    #[wasm_bindgen(js_name = scale)]
    pub fn scale(&self, sx: f64, sy: f64, sz: f64) -> Self {
        Self {
            inner: self.inner.scale(
                real_from_js_or_zero(sx),
                real_from_js_or_zero(sy),
                real_from_js_or_zero(sz),
            ),
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
    pub fn extrude(&self, height: f64, metadata: JsValue) -> Result<MeshJs, JsValue> {
        let metadata = js_metadata(metadata)?;
        let mesh = self.inner.extrude(real_from_js_or_zero(height), metadata);
        Ok(MeshJs { inner: mesh })
    }

    #[wasm_bindgen(js_name = revolve)]
    pub fn revolve(
        &self,
        angle_degrees: f64,
        segments: usize,
        metadata: JsValue,
    ) -> Result<MeshJs, JsValue> {
        let metadata = js_metadata(metadata)?;
        let mesh = self
            .inner
            .revolve(real_from_js_or_zero(angle_degrees), segments, metadata)
            .map_err(|e| JsValue::from_str(&format!("Revolve failed: {:?}", e)))?;
        Ok(MeshJs { inner: mesh })
    }

    #[wasm_bindgen(js_name=extrudeVectorComponents)]
    pub fn extrude_vector_components(
        &self,
        dx: f64,
        dy: f64,
        dz: f64,
        metadata: JsValue,
    ) -> Result<MeshJs, JsValue> {
        let metadata = js_metadata(metadata)?;
        let direction = Vector3Js::new(dx, dy, dz);
        let mesh = self.inner.extrude_vector(direction.inner, metadata);
        Ok(MeshJs { inner: mesh })
    }

    #[wasm_bindgen(js_name = extrudeVector)]
    pub fn extrude_vector(
        &self,
        dir: &Vector3Js,
        metadata: JsValue,
    ) -> Result<MeshJs, JsValue> {
        let metadata = js_metadata(metadata)?;
        let direction: Vector3 = dir.into();
        let mesh = self.inner.extrude_vector(direction, metadata);
        Ok(MeshJs { inner: mesh })
    }

    #[wasm_bindgen(js_name = sweep)]
    pub fn sweep(&self, path: Vec<Point3Js>, metadata: JsValue) -> Result<MeshJs, JsValue> {
        let metadata = js_metadata(metadata)?;
        // Move the inner hyperlattice points out of the wrappers.
        let path_points: Vec<Point3> = path.into_iter().map(|p| p.inner).collect();
        let mesh = self.inner.sweep(&path_points, metadata);
        Ok(MeshJs { inner: mesh })
    }

    #[wasm_bindgen(js_name=sweepComponents)]
    pub fn sweep_components(
        &self,
        path: JsValue,
        metadata: JsValue,
    ) -> Result<MeshJs, JsValue> {
        let metadata = js_metadata(metadata)?;
        // Parse the path from a JS array of [x, y, z] coordinates.
        let path_vec: Vec<[f64; 3]> = from_value(path)
            .map_err(|error| JsValue::from_str(&format!("Invalid sweep path: {error}")))?;
        let path_points: Vec<Point3> = path_vec
            .into_iter()
            .map(|[x, y, z]| Point3Js::new(x, y, z).inner)
            .collect();
        let mesh = self.inner.sweep(&path_points, metadata);
        Ok(MeshJs { inner: mesh })
    }

    // Offset Operations (if offset feature is enabled)
    #[cfg(feature = "offset")]
    #[wasm_bindgen(js_name = offset)]
    pub fn offset(&self, distance: f64) -> Self {
        Self {
            inner: self.inner.offset(real_from_js_or_zero(distance)),
        }
    }

    #[cfg(feature = "offset")]
    #[wasm_bindgen(js_name = offsetRounded)]
    pub fn offset_rounded(&self, distance: f64) -> Self {
        Self {
            inner: self.inner.offset_rounded(real_from_js_or_zero(distance)),
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

        let min_js = Point3Js::from(bb.mins);
        let max_js = Point3Js::from(bb.maxs);

        let obj = Object::new();
        Reflect::set(&obj, &"min".into(), &JsValue::from(min_js)).unwrap();
        Reflect::set(&obj, &"max".into(), &JsValue::from(max_js)).unwrap();
        obj.into()
    }

    #[wasm_bindgen(js_name=invalidateBoundingBox)]
    pub fn invalidate_bounding_box(&mut self) {
        self.inner.invalidate_bounding_box();
    }

    // 2D Shapes
    #[wasm_bindgen(js_name = square)]
    pub fn square(width: f64) -> Self {
        Self {
            inner: Profile::square(real_from_js_or_zero(width)),
        }
    }

    #[wasm_bindgen(js_name = circle)]
    pub fn circle(radius: f64, segments: usize) -> Self {
        Self {
            inner: Profile::circle(real_from_js_or_zero(radius), segments),
        }
    }

    #[wasm_bindgen(js_name = rectangle)]
    pub fn rectangle(width: f64, length: f64) -> Self {
        Self {
            inner: Profile::rectangle(
                real_from_js_or_zero(width),
                real_from_js_or_zero(length),
            ),
        }
    }

    #[wasm_bindgen(js_name = rightTriangle)]
    pub fn right_triangle(width: f64, height: f64) -> Self {
        Self {
            inner: Profile::right_triangle(
                real_from_js_or_zero(width),
                real_from_js_or_zero(height),
            ),
        }
    }

    #[wasm_bindgen(js_name = ellipse)]
    pub fn ellipse(width: f64, height: f64, segments: usize) -> Self {
        Self {
            inner: Profile::ellipse(
                real_from_js_or_zero(width),
                real_from_js_or_zero(height),
                segments,
            ),
        }
    }

    #[wasm_bindgen(js_name = regularNGon)]
    pub fn regular_ngon(sides: usize, radius: f64) -> Self {
        Self {
            inner: Profile::regular_ngon(sides, real_from_js_or_zero(radius)),
        }
    }

    #[wasm_bindgen(js_name = arrow)]
    pub fn arrow(
        shaft_length: f64,
        shaft_width: f64,
        head_length: f64,
        head_width: f64,
    ) -> Self {
        Self {
            inner: Profile::arrow(
                real_from_js_or_zero(shaft_length),
                real_from_js_or_zero(shaft_width),
                real_from_js_or_zero(head_length),
                real_from_js_or_zero(head_width),
            ),
        }
    }

    #[wasm_bindgen(js_name = trapezoid)]
    pub fn trapezoid(top_width: f64, bottom_width: f64, height: f64, top_offset: f64) -> Self {
        Self {
            inner: Profile::trapezoid(
                real_from_js_or_zero(top_width),
                real_from_js_or_zero(bottom_width),
                real_from_js_or_zero(height),
                real_from_js_or_zero(top_offset),
            ),
        }
    }

    #[wasm_bindgen(js_name = star)]
    pub fn star(num_points: usize, outer_radius: f64, inner_radius: f64) -> Self {
        Self {
            inner: Profile::star(
                num_points,
                real_from_js_or_zero(outer_radius),
                real_from_js_or_zero(inner_radius),
            ),
        }
    }

    #[wasm_bindgen(js_name = teardrop)]
    pub fn teardrop(width: f64, length: f64, segments: usize) -> Self {
        Self {
            inner: Profile::teardrop(
                real_from_js_or_zero(width),
                real_from_js_or_zero(length),
                segments,
            ),
        }
    }

    #[wasm_bindgen(js_name = egg)]
    pub fn egg(width: f64, length: f64, segments: usize) -> Self {
        Self {
            inner: Profile::egg(
                real_from_js_or_zero(width),
                real_from_js_or_zero(length),
                segments,
            ),
        }
    }

    #[wasm_bindgen(js_name = roundedRectangle)]
    pub fn rounded_rectangle(
        width: f64,
        height: f64,
        corner_radius: f64,
        corner_segments: usize,
    ) -> Self {
        Self {
            inner: Profile::rounded_rectangle(
                real_from_js_or_zero(width),
                real_from_js_or_zero(height),
                real_from_js_or_zero(corner_radius),
                corner_segments,
            ),
        }
    }

    #[wasm_bindgen(js_name = squircle)]
    pub fn squircle(width: f64, height: f64, segments: usize) -> Self {
        Self {
            inner: Profile::squircle(
                real_from_js_or_zero(width),
                real_from_js_or_zero(height),
                segments,
            ),
        }
    }

    #[wasm_bindgen(js_name = keyhole)]
    pub fn keyhole(
        circle_radius: f64,
        handle_width: f64,
        handle_height: f64,
        segments: usize,
    ) -> Self {
        Self {
            inner: Profile::keyhole(
                real_from_js_or_zero(circle_radius),
                real_from_js_or_zero(handle_width),
                real_from_js_or_zero(handle_height),
                segments,
            ),
        }
    }

    #[wasm_bindgen(js_name = reuleaux)]
    pub fn reuleaux(sides: usize, diameter: f64, circle_segments: usize) -> Self {
        Self {
            inner: Profile::reuleaux(sides, real_from_js_or_zero(diameter), circle_segments),
        }
    }

    #[wasm_bindgen(js_name = ring)]
    pub fn ring(id: f64, thickness: f64, segments: usize) -> Self {
        Self {
            inner: Profile::ring(
                real_from_js_or_zero(id),
                real_from_js_or_zero(thickness),
                segments,
            ),
        }
    }

    #[wasm_bindgen(js_name = pieSlice)]
    pub fn pie_slice(
        radius: f64,
        start_angle_deg: f64,
        end_angle_deg: f64,
        segments: usize,
    ) -> Self {
        Self {
            inner: Profile::pie_slice(
                real_from_js_or_zero(radius),
                real_from_js_or_zero(start_angle_deg),
                real_from_js_or_zero(end_angle_deg),
                segments,
            ),
        }
    }

    #[wasm_bindgen(js_name = supershape)]
    pub fn supershape(
        a: f64,
        b: f64,
        m: f64,
        n1: f64,
        n2: f64,
        n3: f64,
        segments: usize,
    ) -> Self {
        Self {
            inner: Profile::supershape(
                real_from_js_or_zero(a),
                real_from_js_or_zero(b),
                real_from_js_or_zero(m),
                real_from_js_or_zero(n1),
                real_from_js_or_zero(n2),
                real_from_js_or_zero(n3),
                segments,
            ),
        }
    }

    #[wasm_bindgen(js_name = circleWithKeyway)]
    pub fn circle_with_keyway(
        radius: f64,
        segments: usize,
        key_width: f64,
        key_depth: f64,
    ) -> Self {
        Self {
            inner: Profile::circle_with_keyway(
                real_from_js_or_zero(radius),
                segments,
                real_from_js_or_zero(key_width),
                real_from_js_or_zero(key_depth),
            ),
        }
    }

    #[wasm_bindgen(js_name = circleWithFlat)]
    pub fn circle_with_flat(radius: f64, segments: usize, flat_dist: f64) -> Self {
        Self {
            inner: Profile::circle_with_flat(
                real_from_js_or_zero(radius),
                segments,
                real_from_js_or_zero(flat_dist),
            ),
        }
    }

    #[wasm_bindgen(js_name = circleWithTwoFlats)]
    pub fn circle_with_two_flats(radius: f64, segments: usize, flat_dist: f64) -> Self {
        Self {
            inner: Profile::circle_with_two_flats(
                real_from_js_or_zero(radius),
                segments,
                real_from_js_or_zero(flat_dist),
            ),
        }
    }

    #[wasm_bindgen(js_name = bezier)]
    pub fn bezier(control: JsValue, segments: usize) -> Result<Self, JsValue> {
        let control_vec: Vec<[f64; 2]> = from_value(control).map_err(|e| {
            JsValue::from_str(&format!("Failed to parse control points: {:?}", e))
        })?;

        let control_2d: Vec<[Real; 2]> = control_vec
            .into_iter()
            .map(|[x, y]| {
                let x = real_from_js(x).ok_or_else(|| {
                    JsValue::from_str("Bezier control point contains invalid x")
                })?;
                let y = real_from_js(y).ok_or_else(|| {
                    JsValue::from_str("Bezier control point contains invalid y")
                })?;
                Ok([x, y])
            })
            .collect::<Result<_, JsValue>>()?;

        Ok(Self {
            inner: Profile::bezier(&control_2d, segments),
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
            .map(|[x, y]| {
                let x = real_from_js(x).ok_or_else(|| {
                    JsValue::from_str("B-spline control point contains invalid x")
                })?;
                let y = real_from_js(y).ok_or_else(|| {
                    JsValue::from_str("B-spline control point contains invalid y")
                })?;
                Ok([x, y])
            })
            .collect::<Result<_, JsValue>>()?;

        Ok(Self {
            inner: Profile::bspline(&control_2d, p, segments_per_span),
        })
    }

    #[wasm_bindgen(js_name = heart)]
    pub fn heart(width: f64, height: f64, segments: usize) -> Self {
        Self {
            inner: Profile::heart(
                real_from_js_or_zero(width),
                real_from_js_or_zero(height),
                segments,
            ),
        }
    }

    #[wasm_bindgen(js_name = crescent)]
    pub fn crescent(outer_r: f64, inner_r: f64, offset: f64, segments: usize) -> Self {
        Self {
            inner: Profile::crescent(
                real_from_js_or_zero(outer_r),
                real_from_js_or_zero(inner_r),
                real_from_js_or_zero(offset),
                segments,
            ),
        }
    }

    #[wasm_bindgen(js_name = involuteGear)]
    pub fn involute_gear(
        module_: f64,
        teeth: usize,
        pressure_angle_deg: f64,
        clearance: f64,
        backlash: f64,
        segments_per_flank: usize,
    ) -> Self {
        Self {
            inner: Profile::involute_gear(
                real_from_js_or_zero(module_),
                teeth,
                real_from_js_or_zero(pressure_angle_deg),
                real_from_js_or_zero(clearance),
                real_from_js_or_zero(backlash),
                segments_per_flank,
            ),
        }
    }

    #[wasm_bindgen(js_name = airfoilNACA4)]
    pub fn airfoil_naca4(
        max_camber: f64,
        camber_position: f64,
        thickness: f64,
        chord: f64,
        samples: usize,
    ) -> Self {
        Self {
            inner: Profile::airfoil_naca4(
                real_from_js_or_zero(max_camber),
                real_from_js_or_zero(camber_position),
                real_from_js_or_zero(thickness),
                real_from_js_or_zero(chord),
                samples,
            ),
        }
    }

    #[cfg(feature = "offset")]
    #[wasm_bindgen(js_name = hilbertCurve)]
    pub fn hilbert_curve(&self, order: usize, padding: f64) -> Self {
        Self {
            inner: self.inner.hilbert_curve(order, real_from_js_or_zero(padding)),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sketch_js_transform_components_rejects_nonfinite_matrix() {
        let sketch = SketchJs {
            inner: Profile::square(Real::one()),
        };
        let original = sketch.inner.bounding_box();

        let transformed = sketch.transform_components(
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            f64::INFINITY,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
            0.0,
            0.0,
            0.0,
            0.0,
            1.0,
        );

        assert_eq!(transformed.inner.bounding_box(), original);
    }

    #[test]
    fn sketch_js_component_paths_reuse_hyper_boundary_wrappers() {
        let sketch = SketchJs {
            inner: Profile::square(Real::one()),
        };

        let mesh = sketch.inner.extrude_vector(
            Vector3::from_xyz(
                real_from_js_or_zero(f64::NAN),
                real_from_js_or_zero(f64::INFINITY),
                real_from_js_or_zero(0.0),
            ),
            None::<serde_json::Value>,
        );
        assert!(mesh.polygons.is_empty());
    }
}
