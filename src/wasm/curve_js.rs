//! JavaScript wrapper around native [`hypercurve::CurveRegion2`].

use crate::curve::{self, CurveRegionExt};
use crate::wasm::{
    matrix_js::Matrix4Js, mesh_js::MeshJs, point_js::Point3Js, real_from_js,
    real_from_js_named, real_to_js,
};
use crate::{GeometryCertainty, GeometryContext, GeometryOutcome, TriangleMesh};
use hypercurve::{Contour2, CurveCertainty, CurveOutcome, CurvePolicy, CurveRegion2};
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

/// JavaScript-facing curve Boolean value and aggregate predicate certainty.
#[wasm_bindgen]
pub struct CurveBooleanResultJs {
    region: CurveRegion2,
    certainty: CurveCertainty,
}

/// JavaScript-facing mesh value and aggregate predicate certainty.
#[wasm_bindgen]
pub struct GeometryMeshResultJs {
    mesh: TriangleMesh,
    certainty: GeometryCertainty,
}

impl From<CurveRegion2> for CurveRegionJs {
    fn from(inner: CurveRegion2) -> Self {
        Self { inner }
    }
}

impl From<CurveOutcome<CurveRegion2>> for CurveBooleanResultJs {
    fn from(outcome: CurveOutcome<CurveRegion2>) -> Self {
        Self {
            region: outcome.value,
            certainty: outcome.certainty,
        }
    }
}

impl From<GeometryOutcome<CurveRegion2>> for CurveBooleanResultJs {
    fn from(outcome: GeometryOutcome<CurveRegion2>) -> Self {
        Self {
            region: outcome.value,
            certainty: match outcome.certainty {
                GeometryCertainty::Certified => CurveCertainty::Certified,
                GeometryCertainty::Approximate512Consumed => {
                    CurveCertainty::Approximate512Consumed
                },
            },
        }
    }
}

impl From<GeometryOutcome<TriangleMesh>> for GeometryMeshResultJs {
    fn from(outcome: GeometryOutcome<TriangleMesh>) -> Self {
        Self {
            mesh: outcome.value,
            certainty: outcome.certainty,
        }
    }
}

#[wasm_bindgen]
impl CurveBooleanResultJs {
    #[wasm_bindgen(getter, js_name = approximate512Consumed)]
    pub fn approximate_512_consumed(&self) -> bool {
        self.certainty == CurveCertainty::Approximate512Consumed
    }

    #[wasm_bindgen(js_name = intoRegion)]
    pub fn into_region(self) -> CurveRegionJs {
        self.region.into()
    }
}

#[wasm_bindgen]
impl GeometryMeshResultJs {
    #[wasm_bindgen(getter, js_name = approximate512Consumed)]
    pub fn approximate_512_consumed(&self) -> bool {
        self.certainty == GeometryCertainty::Approximate512Consumed
    }

    #[wasm_bindgen(js_name = intoMesh)]
    pub fn into_mesh(self) -> MeshJs {
        self.mesh.into()
    }
}

const fn boolean_policy(approximate_512: bool) -> CurvePolicy {
    if approximate_512 {
        CurvePolicy::APPROXIMATE_512
    } else {
        CurvePolicy::STRICT
    }
}

const fn geometry_context(approximate_512: bool) -> GeometryContext {
    if approximate_512 {
        GeometryContext::APPROXIMATE_512
    } else {
        GeometryContext::STRICT
    }
}

#[wasm_bindgen]
impl CurveRegionJs {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        curve::empty().into()
    }

    #[wasm_bindgen(js_name = fromRegionProfiles)]
    pub fn from_region_profiles(
        value: JsValue,
        approximate_512: bool,
    ) -> Result<Self, JsValue> {
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
        CurveRegion2::try_from_native_contours(
            materials,
            holes,
            &geometry_context(approximate_512).curve_policy(),
        )
        .map(Into::into)
        .map_err(js_error)
    }

    #[wasm_bindgen(js_name = toRegionProfiles)]
    pub fn to_region_profiles(&self, approximate_512: bool) -> Result<JsValue, JsValue> {
        let outcome =
            curve::try_finite_profiles(&self.inner, &geometry_context(approximate_512))
                .map_err(js_error)?;
        let profiles = outcome
            .value
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
        let result = Object::new();
        Reflect::set(
            &result,
            &"profiles".into(),
            &to_value(&profiles).map_err(js_error)?,
        )
        .expect("plain object accepts profiles");
        Reflect::set(
            &result,
            &"approximate512Consumed".into(),
            &(outcome.certainty == GeometryCertainty::Approximate512Consumed).into(),
        )
        .expect("plain object accepts certainty");
        Ok(result.into())
    }

    #[wasm_bindgen(js_name = isEmpty)]
    pub fn is_empty(&self) -> bool {
        self.inner.is_empty()
    }

    #[wasm_bindgen(js_name = toArrays)]
    pub fn to_arrays(&self, approximate_512: bool) -> Result<Object, JsValue> {
        let outcome = curve::try_triangulate(&self.inner, &geometry_context(approximate_512))
            .map_err(js_error)?;
        let mesh = outcome.value;
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
        Reflect::set(
            &object,
            &"approximate512Consumed".into(),
            &(outcome.certainty == GeometryCertainty::Approximate512Consumed).into(),
        )
        .expect("plain object accepts certainty");
        Ok(object)
    }

    pub fn union(
        &self,
        other: &Self,
        approximate_512: bool,
    ) -> Result<CurveBooleanResultJs, JsValue> {
        let policy = boolean_policy(approximate_512);
        self.inner
            .try_union(&other.inner, &policy)
            .map(Into::into)
            .map_err(js_error)
    }

    pub fn difference(
        &self,
        other: &Self,
        approximate_512: bool,
    ) -> Result<CurveBooleanResultJs, JsValue> {
        let policy = boolean_policy(approximate_512);
        self.inner
            .try_difference(&other.inner, &policy)
            .map(Into::into)
            .map_err(js_error)
    }

    pub fn intersection(
        &self,
        other: &Self,
        approximate_512: bool,
    ) -> Result<CurveBooleanResultJs, JsValue> {
        let policy = boolean_policy(approximate_512);
        self.inner
            .try_intersection(&other.inner, &policy)
            .map(Into::into)
            .map_err(js_error)
    }

    pub fn xor(
        &self,
        other: &Self,
        approximate_512: bool,
    ) -> Result<CurveBooleanResultJs, JsValue> {
        let policy = boolean_policy(approximate_512);
        self.inner
            .try_xor(&other.inner, &policy)
            .map(Into::into)
            .map_err(js_error)
    }

    pub fn transform(&self, matrix: &Matrix4Js) -> Result<Self, JsValue> {
        curve::try_transformed(&self.inner, &matrix.inner)
            .map(Into::into)
            .map_err(js_error)
    }

    pub fn translate(&self, x: f64, y: f64) -> Result<Self, JsValue> {
        curve::try_translated(
            &self.inner,
            real_from_js_named(x, "x")?,
            real_from_js_named(y, "y")?,
        )
        .map(Into::into)
        .map_err(js_error)
    }

    pub fn rotate(&self, degrees: f64) -> Result<Self, JsValue> {
        curve::try_rotated(&self.inner, real_from_js_named(degrees, "degrees")?)
            .map(Into::into)
            .map_err(js_error)
    }

    pub fn scale(&self, x: f64, y: f64) -> Result<Self, JsValue> {
        curve::try_scaled(
            &self.inner,
            real_from_js_named(x, "x")?,
            real_from_js_named(y, "y")?,
        )
        .map(Into::into)
        .map_err(js_error)
    }

    #[wasm_bindgen(js_name = containsXY)]
    pub fn contains_xy(&self, x: f64, y: f64) -> Result<Option<bool>, JsValue> {
        Ok(curve::contains_xy(
            &self.inner,
            real_from_js_named(x, "x")?,
            real_from_js_named(y, "y")?,
        ))
    }

    pub fn extrude(
        &self,
        height: f64,
        approximate_512: bool,
    ) -> Result<GeometryMeshResultJs, JsValue> {
        curve::try_extrude(
            &self.inner,
            real_from_js_named(height, "height")?,
            &geometry_context(approximate_512),
        )
        .map(Into::into)
        .map_err(js_error)
    }

    pub fn revolve(
        &self,
        angle_degrees: f64,
        segments: usize,
        approximate_512: bool,
    ) -> Result<GeometryMeshResultJs, JsValue> {
        curve::revolve(
            &self.inner,
            real_from_js_named(angle_degrees, "angleDegrees")?,
            segments,
            &geometry_context(approximate_512),
        )
        .map(Into::into)
        .map_err(js_error)
    }

    pub fn sweep(
        &self,
        path: Vec<Point3Js>,
        approximate_512: bool,
    ) -> Result<GeometryMeshResultJs, JsValue> {
        let path = path.into_iter().map(|point| point.inner).collect::<Vec<_>>();
        curve::try_sweep(&self.inner, &path, &geometry_context(approximate_512))
            .map(Into::into)
            .map_err(js_error)
    }

    #[wasm_bindgen(js_name = extrudeTwisted)]
    pub fn extrude_twisted(
        &self,
        height: f64,
        twist_degrees: f64,
        end_scale_x: f64,
        end_scale_y: f64,
        slices: usize,
        approximate_512: bool,
    ) -> Result<GeometryMeshResultJs, JsValue> {
        curve::extrude_twisted(
            &self.inner,
            real_from_js_named(height, "height")?,
            real_from_js_named(twist_degrees, "twistDegrees")?,
            [
                real_from_js_named(end_scale_x, "endScaleX")?,
                real_from_js_named(end_scale_y, "endScaleY")?,
            ],
            slices,
            &geometry_context(approximate_512),
        )
        .map(Into::into)
        .map_err(js_error)
    }

    pub fn rectangle(width: f64, height: f64) -> Result<Self, JsValue> {
        Ok(curve::rectangle(
            real_from_js_named(width, "width")?,
            real_from_js_named(height, "height")?,
        )
        .into())
    }

    pub fn square(width: f64) -> Result<Self, JsValue> {
        Ok(curve::square(real_from_js_named(width, "width")?).into())
    }

    pub fn circle(radius: f64, segments: usize) -> Result<Self, JsValue> {
        Ok(curve::circle(real_from_js_named(radius, "radius")?, segments).into())
    }

    pub fn ellipse(width: f64, height: f64, segments: usize) -> Result<Self, JsValue> {
        Ok(curve::ellipse(
            real_from_js_named(width, "width")?,
            real_from_js_named(height, "height")?,
            segments,
        )
        .into())
    }

    #[wasm_bindgen(js_name = rightTriangle)]
    pub fn right_triangle(width: f64, height: f64) -> Result<Self, JsValue> {
        Ok(curve::right_triangle(
            real_from_js_named(width, "width")?,
            real_from_js_named(height, "height")?,
        )
        .into())
    }

    #[wasm_bindgen(js_name = regularNgon)]
    pub fn regular_ngon(sides: usize, radius: f64) -> Result<Self, JsValue> {
        Ok(curve::regular_ngon(sides, real_from_js_named(radius, "radius")?).into())
    }

    pub fn star(points: usize, outer_radius: f64, inner_radius: f64) -> Result<Self, JsValue> {
        Ok(curve::star(
            points,
            real_from_js_named(outer_radius, "outerRadius")?,
            real_from_js_named(inner_radius, "innerRadius")?,
        )
        .into())
    }

    pub fn teardrop(width: f64, length: f64, segments: usize) -> Result<Self, JsValue> {
        Ok(curve::teardrop(
            real_from_js_named(width, "width")?,
            real_from_js_named(length, "length")?,
            segments,
        )
        .into())
    }

    pub fn egg(width: f64, length: f64, segments: usize) -> Result<Self, JsValue> {
        Ok(curve::egg(
            real_from_js_named(width, "width")?,
            real_from_js_named(length, "length")?,
            segments,
        )
        .into())
    }

    pub fn ring(
        inner_diameter: f64,
        thickness: f64,
        segments: usize,
        approximate_512: bool,
    ) -> Result<CurveBooleanResultJs, JsValue> {
        curve::ring(
            real_from_js_named(inner_diameter, "innerDiameter")?,
            real_from_js_named(thickness, "thickness")?,
            segments,
            &geometry_context(approximate_512),
        )
        .map(Into::into)
        .map_err(js_error)
    }

    pub fn heart(width: f64, height: f64, segments: usize) -> Result<Self, JsValue> {
        Ok(curve::heart(
            real_from_js_named(width, "width")?,
            real_from_js_named(height, "height")?,
            segments,
        )
        .into())
    }

    pub fn crescent(
        outer_radius: f64,
        inner_radius: f64,
        offset: f64,
        segments: usize,
        approximate_512: bool,
    ) -> Result<CurveBooleanResultJs, JsValue> {
        curve::crescent(
            real_from_js_named(outer_radius, "outerRadius")?,
            real_from_js_named(inner_radius, "innerRadius")?,
            real_from_js_named(offset, "offset")?,
            segments,
            &geometry_context(approximate_512),
        )
        .map(Into::into)
        .map_err(js_error)
    }
}

impl Default for CurveRegionJs {
    fn default() -> Self {
        Self::new()
    }
}
