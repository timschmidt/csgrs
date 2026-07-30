//! JavaScript wrapper around native [`hypermesh::TriangleMesh`].

use crate::solid::{self, SolidExt};
use crate::wasm::{
    matrix_js::Matrix4Js, plane_js::PlaneJs, point_js::Point3Js, real_from_js,
    real_from_js_named, real_to_js,
};
use hyperlattice::Point3;
use hypermesh::{Triangle, TriangleMesh};
use js_sys::{Float64Array, Object, Reflect, Uint32Array};
use wasm_bindgen::prelude::*;

fn js_error(error: impl std::fmt::Display) -> JsValue {
    JsValue::from_str(&error.to_string())
}

fn face_normal(points: &[Point3; 3]) -> Option<hyperlattice::Vector3> {
    (&points[1] - &points[0])
        .unit_cross_checked(&(&points[2] - &points[0]))
        .ok()
}

#[wasm_bindgen]
pub struct MeshJs {
    pub(crate) inner: TriangleMesh,
}

impl From<TriangleMesh> for MeshJs {
    fn from(inner: TriangleMesh) -> Self {
        Self { inner }
    }
}

#[wasm_bindgen]
#[allow(clippy::missing_const_for_fn)]
impl MeshJs {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        solid::empty().into()
    }

    #[wasm_bindgen(js_name = fromArrays)]
    pub fn from_arrays(positions: Vec<f64>, indices: Vec<u32>) -> Result<Self, JsValue> {
        if !positions.len().is_multiple_of(3) || !indices.len().is_multiple_of(3) {
            return Err(JsValue::from_str(
                "positions and indices must contain complete triples",
            ));
        }
        let positions = positions
            .chunks_exact(3)
            .map(|row| {
                Some(Point3::new(
                    real_from_js(row[0])?,
                    real_from_js(row[1])?,
                    real_from_js(row[2])?,
                ))
            })
            .collect::<Option<Vec<_>>>()
            .ok_or_else(|| JsValue::from_str("positions must be finite"))?;
        let triangles = indices
            .chunks_exact(3)
            .map(|row| {
                let [a, b, c] = [
                    usize::try_from(row[0]).ok()?,
                    usize::try_from(row[1]).ok()?,
                    usize::try_from(row[2]).ok()?,
                ];
                (a < positions.len() && b < positions.len() && c < positions.len())
                    .then(|| Triangle::new(a, b, c))
            })
            .collect::<Option<Vec<_>>>()
            .ok_or_else(|| JsValue::from_str("an index is outside the position array"))?;
        Ok(TriangleMesh::new(positions, triangles).into())
    }

    #[wasm_bindgen(js_name = isEmpty)]
    pub fn is_empty(&self) -> bool {
        self.inner.triangles.is_empty()
    }

    #[wasm_bindgen(js_name = triangleCount)]
    pub fn triangle_count(&self) -> u32 {
        u32::try_from(self.inner.triangles.len()).unwrap_or(u32::MAX)
    }

    #[wasm_bindgen(js_name = vertexCount)]
    pub fn vertex_count(&self) -> u32 {
        u32::try_from(self.inner.positions.len()).unwrap_or(u32::MAX)
    }

    #[wasm_bindgen(js_name = toArrays)]
    pub fn to_arrays(&self) -> Object {
        let positions = self
            .inner
            .positions
            .iter()
            .flat_map(|point| [&point.x, &point.y, &point.z].map(real_to_js))
            .collect::<Vec<_>>();
        let indices = self
            .inner
            .triangles
            .iter()
            .flat_map(|triangle| triangle.indices().map(|index| index as u32))
            .collect::<Vec<_>>();
        let mut normals = vec![0.0; positions.len()];
        let mut counts = vec![0_u32; self.inner.positions.len()];
        for triangle in self.inner.triangles.iter() {
            let [a, b, c] = triangle.indices();
            let points = [
                self.inner.positions[a].clone(),
                self.inner.positions[b].clone(),
                self.inner.positions[c].clone(),
            ];
            let Some(normal) = face_normal(&points) else {
                continue;
            };
            for index in [a, b, c] {
                normals[index * 3] += real_to_js(&normal.0[0]);
                normals[index * 3 + 1] += real_to_js(&normal.0[1]);
                normals[index * 3 + 2] += real_to_js(&normal.0[2]);
                counts[index] += 1;
            }
        }
        for (normal, count) in normals.chunks_exact_mut(3).zip(counts) {
            if count != 0 {
                let length =
                    (normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2])
                        .sqrt();
                if length > 0.0 {
                    normal.iter_mut().for_each(|component| *component /= length);
                }
            }
        }
        let object = Object::new();
        Reflect::set(
            &object,
            &"positions".into(),
            &Float64Array::from(positions.as_slice()),
        )
        .expect("plain object accepts positions");
        Reflect::set(
            &object,
            &"normals".into(),
            &Float64Array::from(normals.as_slice()),
        )
        .expect("plain object accepts normals");
        Reflect::set(
            &object,
            &"indices".into(),
            &Uint32Array::from(indices.as_slice()),
        )
        .expect("plain object accepts indices");
        object
    }

    pub fn positions(&self) -> Float64Array {
        Reflect::get(&self.to_arrays(), &"positions".into())
            .expect("positions exist")
            .unchecked_into()
    }

    pub fn normals(&self) -> Float64Array {
        Reflect::get(&self.to_arrays(), &"normals".into())
            .expect("normals exist")
            .unchecked_into()
    }

    pub fn indices(&self) -> Uint32Array {
        Reflect::get(&self.to_arrays(), &"indices".into())
            .expect("indices exist")
            .unchecked_into()
    }

    #[wasm_bindgen(js_name = containsPoint)]
    pub fn contains_point(&self, point: &Point3Js) -> Result<bool, JsValue> {
        solid::contains_point(&self.inner, &point.inner).map_err(js_error)
    }

    pub fn union(&self, other: &Self) -> Result<Self, JsValue> {
        self.inner
            .try_union(&other.inner)
            .map(Into::into)
            .map_err(js_error)
    }

    pub fn difference(&self, other: &Self) -> Result<Self, JsValue> {
        self.inner
            .try_difference(&other.inner)
            .map(Into::into)
            .map_err(js_error)
    }

    pub fn intersection(&self, other: &Self) -> Result<Self, JsValue> {
        self.inner
            .try_intersection(&other.inner)
            .map(Into::into)
            .map_err(js_error)
    }

    pub fn xor(&self, other: &Self) -> Result<Self, JsValue> {
        self.inner
            .try_xor(&other.inner)
            .map(Into::into)
            .map_err(js_error)
    }

    pub fn transform(&self, matrix: &Matrix4Js) -> Result<Self, JsValue> {
        solid::try_transform(&self.inner, &matrix.inner)
            .map(Into::into)
            .map_err(js_error)
    }

    pub fn translate(&self, x: f64, y: f64, z: f64) -> Result<Self, JsValue> {
        Ok(self
            .inner
            .translated(
                real_from_js_named(x, "x")?,
                real_from_js_named(y, "y")?,
                real_from_js_named(z, "z")?,
            )
            .into())
    }

    pub fn rotate(&self, x: f64, y: f64, z: f64) -> Result<Self, JsValue> {
        solid::try_rotate(
            &self.inner,
            real_from_js_named(x, "x")?,
            real_from_js_named(y, "y")?,
            real_from_js_named(z, "z")?,
        )
        .map(Into::into)
        .map_err(js_error)
    }

    pub fn scale(&self, x: f64, y: f64, z: f64) -> Result<Self, JsValue> {
        solid::try_scale(
            &self.inner,
            real_from_js_named(x, "x")?,
            real_from_js_named(y, "y")?,
            real_from_js_named(z, "z")?,
        )
        .map(Into::into)
        .map_err(js_error)
    }

    pub fn center(&self) -> Result<Self, JsValue> {
        solid::try_center(&self.inner)
            .map(Into::into)
            .map_err(js_error)
    }

    #[wasm_bindgen(js_name = floatOnZ)]
    pub fn float_on_z(&self) -> Result<Self, JsValue> {
        solid::try_float(&self.inner)
            .map(Into::into)
            .map_err(js_error)
    }

    pub fn inverse(&self) -> Self {
        solid::inverse(&self.inner).into()
    }

    pub fn mirror(&self, plane: &PlaneJs) -> Result<Self, JsValue> {
        solid::try_mirror(&self.inner, &plane.inner)
            .map(Into::into)
            .map_err(js_error)
    }

    #[wasm_bindgen(js_name = convexHull)]
    pub fn convex_hull(&self) -> Result<Self, JsValue> {
        solid::convex_hull(&self.inner)
            .map(Into::into)
            .map_err(js_error)
    }

    #[wasm_bindgen(js_name = minkowskiSum)]
    pub fn minkowski_sum(&self, other: &Self) -> Result<Self, JsValue> {
        solid::minkowski_sum(&self.inner, &other.inner)
            .map(Into::into)
            .map_err(js_error)
    }

    #[wasm_bindgen(js_name = boundingBox)]
    pub fn bounding_box(&self) -> Result<JsValue, JsValue> {
        let bounds = solid::try_bounding_box(&self.inner).map_err(js_error)?;
        let object = Object::new();
        Reflect::set(
            &object,
            &"min".into(),
            &JsValue::from(Point3Js::from(bounds.mins.clone())),
        )
        .expect("plain object accepts minimum");
        Reflect::set(
            &object,
            &"max".into(),
            &JsValue::from(Point3Js::from(bounds.maxs.clone())),
        )
        .expect("plain object accepts maximum");
        Ok(object.into())
    }

    #[wasm_bindgen(js_name = toSTLBinary)]
    pub fn to_stl_binary(&self) -> Result<Vec<u8>, JsValue> {
        crate::io::stl::to_stl_binary(&self.inner, "mesh").map_err(js_error)
    }

    #[wasm_bindgen(js_name = toSTLASCII)]
    pub fn to_stl_ascii(&self) -> Result<String, JsValue> {
        crate::io::stl::to_stl_ascii(&self.inner, "mesh").map_err(js_error)
    }

    pub fn subdivide(&self, levels: u32) -> Result<Self, JsValue> {
        let levels = std::num::NonZeroU32::new(levels)
            .ok_or_else(|| JsValue::from_str("levels must be positive"))?;
        solid::subdivide(&self.inner, levels)
            .map(Into::into)
            .map_err(js_error)
    }

    pub fn cube(size: f64) -> Result<Self, JsValue> {
        Ok(solid::cube(real_from_js_named(size, "size")?).into())
    }

    pub fn cuboid(width: f64, length: f64, height: f64) -> Result<Self, JsValue> {
        Ok(solid::cuboid(
            real_from_js_named(width, "width")?,
            real_from_js_named(length, "length")?,
            real_from_js_named(height, "height")?,
        )
        .into())
    }

    pub fn sphere(radius: f64, segments: usize, stacks: usize) -> Result<Self, JsValue> {
        Ok(solid::sphere(real_from_js_named(radius, "radius")?, segments, stacks).into())
    }

    pub fn cylinder(radius: f64, height: f64, segments: usize) -> Result<Self, JsValue> {
        Ok(solid::cylinder(
            real_from_js_named(radius, "radius")?,
            real_from_js_named(height, "height")?,
            segments,
        )
        .into())
    }

    pub fn frustum(
        bottom_radius: f64,
        top_radius: f64,
        height: f64,
        segments: usize,
    ) -> Result<Self, JsValue> {
        Ok(solid::frustum(
            real_from_js_named(bottom_radius, "bottomRadius")?,
            real_from_js_named(top_radius, "topRadius")?,
            real_from_js_named(height, "height")?,
            segments,
        )
        .into())
    }

    pub fn ellipsoid(
        rx: f64,
        ry: f64,
        rz: f64,
        segments: usize,
        stacks: usize,
    ) -> Result<Self, JsValue> {
        Ok(solid::ellipsoid(
            real_from_js_named(rx, "rx")?,
            real_from_js_named(ry, "ry")?,
            real_from_js_named(rz, "rz")?,
            segments,
            stacks,
        )
        .into())
    }

    pub fn torus(
        major_radius: f64,
        minor_radius: f64,
        major_segments: usize,
        minor_segments: usize,
    ) -> Result<Self, JsValue> {
        Ok(solid::torus(
            real_from_js_named(major_radius, "majorRadius")?,
            real_from_js_named(minor_radius, "minorRadius")?,
            major_segments,
            minor_segments,
        )
        .into())
    }
}

impl Default for MeshJs {
    fn default() -> Self {
        Self::new()
    }
}
