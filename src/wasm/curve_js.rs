//! JavaScript wrapper around native [`hypercurve::CurveRegion2`].

use crate::curve::{self, CurveRegionExt};
use crate::wasm::{
    matrix_js::Matrix4Js, mesh_js::MeshJs, point_js::Point3Js, real_from_js,
    real_from_js_or_zero, real_to_js,
};
use hypercurve::{Contour2, CurvePolicy, CurveRegion2};
use hyperlattice::Real;
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
            Ok([
                real_from_js(x).ok_or_else(|| {
                    JsValue::from_str(&format!("{label}[{index}] has invalid x"))
                })?,
                real_from_js(y).ok_or_else(|| {
                    JsValue::from_str(&format!("{label}[{index}] has invalid y"))
                })?,
            ])
        })
        .collect()
}

fn js_error(error: impl std::fmt::Display) -> JsValue {
    JsValue::from_str(&error.to_string())
}

#[wasm_bindgen]
pub struct CurveRegionJs {
    pub(crate) inner: CurveRegion2,
}

impl From<CurveRegion2> for CurveRegionJs {
    fn from(inner: CurveRegion2) -> Self {
        Self { inner }
    }
}

#[wasm_bindgen]
impl CurveRegionJs {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        curve::empty().into()
    }

    #[wasm_bindgen(js_name = fromRegionProfiles)]
    pub fn from_region_profiles(value: JsValue) -> Result<Self, JsValue> {
        let profiles: Vec<RegionProfileJs> = from_value(value).map_err(js_error)?;
        let mut materials = Vec::with_capacity(profiles.len());
        let mut holes = Vec::new();
        for (profile_index, profile) in profiles.into_iter().enumerate() {
            let material = promote_ring(
                profile.material,
                &format!("profiles[{profile_index}].material"),
            )?;
            materials.push(Contour2::from_real_ring(&material).map_err(js_error)?);
            for (hole_index, hole) in profile.holes.into_iter().enumerate() {
                let hole = promote_ring(
                    hole,
                    &format!("profiles[{profile_index}].holes[{hole_index}]"),
                )?;
                holes.push(Contour2::from_real_ring(&hole).map_err(js_error)?);
            }
        }
        CurveRegion2::try_from_native_contours(materials, holes, &CurvePolicy::certified())
            .map(Into::into)
            .map_err(js_error)
    }

    #[wasm_bindgen(js_name = toRegionProfiles)]
    pub fn to_region_profiles(&self) -> Result<JsValue, JsValue> {
        let profiles = curve::finite_profiles(&self.inner)
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
        to_value(&profiles).map_err(js_error)
    }

    #[wasm_bindgen(js_name = isEmpty)]
    pub fn is_empty(&self) -> bool {
        self.inner.is_empty()
    }

    #[wasm_bindgen(js_name = toArrays)]
    pub fn to_arrays(&self) -> Object {
        let mesh = curve::triangulate(&self.inner);
        let positions = mesh
            .positions
            .iter()
            .flat_map(|point| [real_to_js(&point.x), real_to_js(&point.y), 0.0])
            .collect::<Vec<_>>();
        let normals = (0..mesh.positions.len())
            .flat_map(|_| [0.0, 0.0, 1.0])
            .collect::<Vec<_>>();
        let indices = mesh
            .triangles
            .iter()
            .flat_map(|triangle| triangle.indices().map(|index| index as u32))
            .collect::<Vec<_>>();
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

    pub fn transform(&self, matrix: &Matrix4Js) -> Self {
        curve::transformed(&self.inner, &matrix.inner).into()
    }

    pub fn translate(&self, x: f64, y: f64) -> Self {
        curve::translated(&self.inner, real_from_js_or_zero(x), real_from_js_or_zero(y)).into()
    }

    pub fn rotate(&self, degrees: f64) -> Self {
        curve::rotated(&self.inner, real_from_js_or_zero(degrees)).into()
    }

    pub fn scale(&self, x: f64, y: f64) -> Self {
        curve::scaled(&self.inner, real_from_js_or_zero(x), real_from_js_or_zero(y)).into()
    }

    #[wasm_bindgen(js_name = containsXY)]
    pub fn contains_xy(&self, x: f64, y: f64) -> Option<bool> {
        curve::contains_xy(&self.inner, real_from_js_or_zero(x), real_from_js_or_zero(y))
    }

    pub fn extrude(&self, height: f64) -> MeshJs {
        curve::extrude(&self.inner, real_from_js_or_zero(height)).into()
    }

    pub fn revolve(&self, angle_degrees: f64, segments: usize) -> Result<MeshJs, JsValue> {
        curve::revolve(&self.inner, real_from_js_or_zero(angle_degrees), segments)
            .map(Into::into)
            .map_err(js_error)
    }

    pub fn sweep(&self, path: Vec<Point3Js>) -> MeshJs {
        let path = path.into_iter().map(|point| point.inner).collect::<Vec<_>>();
        curve::sweep(&self.inner, &path).into()
    }

    #[wasm_bindgen(js_name = extrudeTwisted)]
    pub fn extrude_twisted(
        &self,
        height: f64,
        twist_degrees: f64,
        end_scale_x: f64,
        end_scale_y: f64,
        slices: usize,
    ) -> Result<MeshJs, JsValue> {
        curve::extrude_twisted(
            &self.inner,
            real_from_js_or_zero(height),
            real_from_js_or_zero(twist_degrees),
            [
                real_from_js_or_zero(end_scale_x),
                real_from_js_or_zero(end_scale_y),
            ],
            slices,
        )
        .map(Into::into)
        .map_err(js_error)
    }

    pub fn rectangle(width: f64, height: f64) -> Self {
        curve::rectangle(real_from_js_or_zero(width), real_from_js_or_zero(height)).into()
    }

    pub fn square(width: f64) -> Self {
        curve::square(real_from_js_or_zero(width)).into()
    }

    pub fn circle(radius: f64, segments: usize) -> Self {
        curve::circle(real_from_js_or_zero(radius), segments).into()
    }

    pub fn ellipse(width: f64, height: f64, segments: usize) -> Self {
        curve::ellipse(
            real_from_js_or_zero(width),
            real_from_js_or_zero(height),
            segments,
        )
        .into()
    }

    #[wasm_bindgen(js_name = rightTriangle)]
    pub fn right_triangle(width: f64, height: f64) -> Self {
        curve::right_triangle(real_from_js_or_zero(width), real_from_js_or_zero(height)).into()
    }

    #[wasm_bindgen(js_name = regularNgon)]
    pub fn regular_ngon(sides: usize, radius: f64) -> Self {
        curve::regular_ngon(sides, real_from_js_or_zero(radius)).into()
    }

    pub fn star(points: usize, outer_radius: f64, inner_radius: f64) -> Self {
        curve::star(
            points,
            real_from_js_or_zero(outer_radius),
            real_from_js_or_zero(inner_radius),
        )
        .into()
    }

    pub fn teardrop(width: f64, length: f64, segments: usize) -> Self {
        curve::teardrop(
            real_from_js_or_zero(width),
            real_from_js_or_zero(length),
            segments,
        )
        .into()
    }

    pub fn egg(width: f64, length: f64, segments: usize) -> Self {
        curve::egg(
            real_from_js_or_zero(width),
            real_from_js_or_zero(length),
            segments,
        )
        .into()
    }

    pub fn ring(inner_diameter: f64, thickness: f64, segments: usize) -> Self {
        curve::ring(
            real_from_js_or_zero(inner_diameter),
            real_from_js_or_zero(thickness),
            segments,
        )
        .into()
    }

    pub fn heart(width: f64, height: f64, segments: usize) -> Self {
        curve::heart(
            real_from_js_or_zero(width),
            real_from_js_or_zero(height),
            segments,
        )
        .into()
    }

    pub fn crescent(
        outer_radius: f64,
        inner_radius: f64,
        offset: f64,
        segments: usize,
    ) -> Self {
        curve::crescent(
            real_from_js_or_zero(outer_radius),
            real_from_js_or_zero(inner_radius),
            real_from_js_or_zero(offset),
            segments,
        )
        .into()
    }
}

impl Default for CurveRegionJs {
    fn default() -> Self {
        Self::new()
    }
}
