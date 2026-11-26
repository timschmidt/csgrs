//! JavaScript wrapper around native [`hypercurve::CurveRegion2`].
//!
//! Native regions remain authoritative through modeling and Boolean operations.
//! JS numbers enter as exact finite binary64 values; sampled constructors and
//! finite profile/array exports have their own source-fidelity limits. Counts
//! typed as `usize` use the binding's integer ABI before native validation.
//! Context outcomes expose consumed predicate certainty, not an independent proof
//! of source fidelity. Most modeling failures become `GeometryError` objects;
//! profile/Boolean helpers and some scalar arguments still return JS strings.

use crate::curve;
use crate::wasm::{
    context_js::{
        GeometryBoolResultJs, GeometryBoundsResultJs, GeometryCertaintyJs,
        GeometryQueryStatusJs,
    },
    geometry_context,
};
use crate::wasm::{
    matrix_js::Matrix4Js, mesh_js::MeshJs, point_js::Point3Js, real_from_js,
    real_from_js_named, real_to_js,
};
use crate::{GeometryCertainty, GeometryOutcome, TriangleMesh};
use hypercurve::{Contour2, CurveCertainty, CurveRegion2};
use hyperlattice::Real;
use hyperlimit::PredicatePolicy;
use js_sys::{Float64Array, Object, Reflect, Uint32Array};
use serde::{Deserialize, Serialize};
use serde_wasm_bindgen::{from_value, to_value};
use wasm_bindgen::prelude::*;

/// Serialized finite polygon-with-holes record used by profile ingress and egress.
/// No exact curve definition, root selection, chord-error metadata or receipt is stored here.
#[derive(Debug, Deserialize, Serialize)]
#[serde(rename_all = "camelCase")]
struct RegionProfileJs {
    /// Finite XY ring declared as material; native admission owns topology validation.
    material: Vec<[f64; 2]>,
    /// Finite XY rings declared as holes; ingress flattens these across profiles.
    holes: Vec<Vec<[f64; 2]>>,
}

/// Promotes finite ring coordinates exactly in vertex/X/Y order, with indexed string errors.
/// Does not itself close or validate a contour, remove duplicates or recover a sampled source.
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

/// Copies a native diagnostic into a JS string, discarding typed categories and source payloads.
fn js_error(error: impl std::fmt::Display) -> JsValue {
    JsValue::from_str(&error.to_string())
}

/// Owned native filled region, including holes, disconnected components and canonical emptiness.
/// Finite exports are views; native conversions and operations do not replace this carrier with them.
#[wasm_bindgen]
pub struct CurveRegionJs {
    /// Authoritative region and its retained native evidence/caches.
    pub(crate) inner: CurveRegion2,
}

/// JavaScript-facing curve Boolean value and aggregate predicate certainty.
#[wasm_bindgen]
pub struct CurveBooleanResultJs {
    /// Completed native region, including successful empty geometry.
    region: CurveRegion2,
    /// Consumed operation certainty, separate from fidelity to an authored source.
    certainty: CurveCertainty,
}

/// JavaScript-facing mesh value and aggregate predicate certainty.
#[wasm_bindgen]
pub struct GeometryMeshResultJs {
    /// Completed mesh, retaining its own native construction qualifications.
    mesh: TriangleMesh,
    /// Operation-local certainty, independent of the mesh's inherited history.
    certainty: GeometryCertainty,
}

impl From<CurveRegion2> for CurveRegionJs {
    /// Moves a native region into the wrapper without projection or additional validation.
    fn from(inner: CurveRegion2) -> Self {
        Self { inner }
    }
}

impl From<GeometryOutcome<CurveRegion2>> for CurveBooleanResultJs {
    /// Moves region and local receipt together, mapping both certainty variants without upgrading them.
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
    /// Moves mesh and local receipt together; does not synthesize mesh history from that receipt.
    fn from(outcome: GeometryOutcome<TriangleMesh>) -> Self {
        Self {
            mesh: outcome.value,
            certainty: outcome.certainty,
        }
    }
}

#[wasm_bindgen]
impl CurveBooleanResultJs {
    /// Returns the retained operation certainty.
    #[wasm_bindgen(getter)]
    pub fn certainty(&self) -> GeometryCertaintyJs {
        self.certainty.into()
    }

    /// Reports actual terminal consumption, not merely selection of the approximate policy.
    #[wasm_bindgen(getter, js_name = approximate512Consumed)]
    pub fn approximate_512_consumed(&self) -> bool {
        self.certainty == CurveCertainty::Approximate512Consumed
    }

    /// Consumes this outcome handle and returns the native region without the separate receipt.
    /// Inspect certainty first; retained region evidence is moved unchanged.
    #[wasm_bindgen(js_name = intoRegion)]
    pub fn into_region(self) -> CurveRegionJs {
        self.region.into()
    }
}

#[wasm_bindgen]
impl GeometryMeshResultJs {
    /// Returns the mesh operation's local predicate certainty.
    #[wasm_bindgen(getter)]
    pub fn certainty(&self) -> GeometryCertaintyJs {
        self.certainty.into()
    }

    /// Reports whether this operation consumed the authorized approximate terminal.
    #[wasm_bindgen(getter, js_name = approximate512Consumed)]
    pub fn approximate_512_consumed(&self) -> bool {
        self.certainty == GeometryCertainty::Approximate512Consumed
    }

    /// Consumes the outcome handle and returns its mesh, dropping the separate local receipt.
    /// The mesh's existing construction qualifications remain queryable through `constructionEvidence`.
    #[wasm_bindgen(js_name = intoMesh)]
    pub fn into_mesh(self) -> MeshJs {
        self.mesh.into()
    }
}

/// Runs a region closure through [`hypercurve::evaluate_under`] with the selected policy.
/// On success retains the evaluated certainty; on failure returns only diagnostic
/// text. The approximate flag authorizes terminal consumption but does not imply it occurred.
fn region_result(
    approximate_512: bool,
    operation: impl FnOnce() -> hypercurve::ExactCurveResult<CurveRegion2>,
) -> Result<CurveBooleanResultJs, JsValue> {
    let policy = if approximate_512 {
        PredicatePolicy::APPROXIMATE_512
    } else {
        PredicatePolicy::STRICT
    };
    let provisional = hypercurve::evaluate_under(policy, operation);
    let certainty = provisional.certainty();
    let region = provisional.into_unverified();
    region
        .map(|region| CurveBooleanResultJs { region, certainty })
        .map_err(js_error)
}

#[wasm_bindgen]
impl CurveRegionJs {
    /// Constructs the canonical empty filled region, usable by subsequent native operations.
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        curve::empty().into()
    }

    /// Imports an array of `{material, holes}` finite XY-ring records as new polygonal geometry.
    /// Coordinates are promoted exactly, then native contour validation runs before
    /// the selected-policy region closure. Material/hole roles are flattened across
    /// records; native admission validates and reconstructs their topology.
    /// An empty list yields an empty region. Failures are JS strings; serialized
    /// records contain no evidence of a preceding analytic or exact-curve source.
    #[wasm_bindgen(js_name = fromRegionProfiles)]
    pub fn from_region_profiles(
        value: JsValue,
        approximate_512: bool,
    ) -> Result<CurveBooleanResultJs, JsValue> {
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
        region_result(approximate_512, || {
            CurveRegion2::try_from_native_contours(materials, holes)
        })
    }

    /// Returns `{profiles, certainty, approximate512Consumed}` with finite XY rings and native ownership.
    /// Uses [`curve::try_finite_profiles`]'s `1e-3` chord-error request and projects
    /// every coordinate, including exact line vertices. Record conversion omits
    /// chord-error metadata and source history; the receipt covers native decisions,
    /// not post-projection topology/fidelity. Empty geometry yields no profiles.
    /// Native/serialization failures become strings; property-setting exceptions panic.
    #[wasm_bindgen(js_name = toRegionProfiles)]
    pub fn to_region_profiles(&self, approximate_512: bool) -> Result<JsValue, JsValue> {
        let outcome =
            curve::try_finite_profiles(&self.inner, geometry_context(approximate_512))
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
        Reflect::set(
            &result,
            &"certainty".into(),
            &GeometryCertaintyJs::from(outcome.certainty).into(),
        )
        .expect("plain object accepts certainty");
        Ok(result.into())
    }

    /// Reports native region emptiness without sampling or generating display triangles.
    #[wasm_bindgen(js_name = isEmpty)]
    pub fn is_empty(&self) -> bool {
        self.inner.is_empty()
    }

    /// Triangulates the modeled XY region and copies display positions, normals and indices into JS arrays.
    /// Returns Float64Array XYZ positions with Z zero, constant +Z normals,
    /// Uint32Array indices and local certainty fields. Native triangulation follows
    /// [`curve::try_triangulate`]'s mixed exact/sampled boundary contract.
    /// Position views can round or become NaN; index casts are unchecked on a
    /// wider host. Mesh construction history is not exported in this object, and
    /// no post-projection topology check occurs. Property-setting exceptions panic.
    #[wasm_bindgen(js_name = toArrays)]
    pub fn to_arrays(&self, approximate_512: bool) -> Result<Object, JsValue> {
        let outcome = curve::try_triangulate(&self.inner, geometry_context(approximate_512))
            .map_err(crate::wasm::mesh_js::geometry_error)?;
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
        Reflect::set(
            &object,
            &"certainty".into(),
            &GeometryCertaintyJs::from(outcome.certainty).into(),
        )
        .expect("plain object accepts certainty");
        Ok(object)
    }

    /// Returns the regularized native union with selected-policy certainty; failures become strings.
    pub fn union(
        &self,
        other: &Self,
        approximate_512: bool,
    ) -> Result<CurveBooleanResultJs, JsValue> {
        region_result(approximate_512, || {
            self.inner
                .boolean_region(&other.inner, hypercurve::BooleanOp::Union)
        })
    }

    /// Returns regularized `self` minus `other` with selected-policy certainty; failures become strings.
    pub fn difference(
        &self,
        other: &Self,
        approximate_512: bool,
    ) -> Result<CurveBooleanResultJs, JsValue> {
        region_result(approximate_512, || {
            self.inner
                .boolean_region(&other.inner, hypercurve::BooleanOp::Difference)
        })
    }

    /// Returns the regularized native intersection with selected-policy certainty; failures become strings.
    pub fn intersection(
        &self,
        other: &Self,
        approximate_512: bool,
    ) -> Result<CurveBooleanResultJs, JsValue> {
        region_result(approximate_512, || {
            self.inner
                .boolean_region(&other.inner, hypercurve::BooleanOp::Intersection)
        })
    }

    /// Returns the regularized native symmetric difference with selected-policy certainty; failures become strings.
    pub fn xor(
        &self,
        other: &Self,
        approximate_512: bool,
    ) -> Result<CurveBooleanResultJs, JsValue> {
        region_result(approximate_512, || {
            self.inner
                .boolean_region(&other.inner, hypercurve::BooleanOp::Xor)
        })
    }

    /// Applies [`curve::transformed`]'s nonsingular planar restriction of a homogeneous matrix strictly.
    /// Constant nonzero weight is divided out; perspective/out-of-plane maps fail,
    /// including for empty input. Returns no separate operation receipt.
    pub fn transform(&self, matrix: &Matrix4Js) -> Result<Self, JsValue> {
        curve::transformed(&self.inner, &matrix.inner)
            .map(Into::into)
            .map_err(crate::wasm::mesh_js::geometry_error)
    }

    /// Translates by exactly promoted finite XY offsets under strict native decisions, without a separate receipt.
    pub fn translate(&self, x: f64, y: f64) -> Result<Self, JsValue> {
        curve::translated(
            &self.inner,
            crate::wasm::mesh_js::geometry_scalar(x, "x")?,
            crate::wasm::mesh_js::geometry_scalar(y, "y")?,
        )
        .map(Into::into)
        .map_err(crate::wasm::mesh_js::geometry_error)
    }

    /// Rotates counterclockwise about the origin by finite degrees with native trig and strict decisions.
    /// Returns geometry without a separate receipt; it does not rotate display chords in binary64.
    pub fn rotate(&self, degrees: f64) -> Result<Self, JsValue> {
        curve::rotated(
            &self.inner,
            crate::wasm::mesh_js::geometry_scalar(degrees, "degrees")?,
        )
        .map(Into::into)
        .map_err(crate::wasm::mesh_js::geometry_error)
    }

    /// Scales about the origin strictly; finite factors must be nonzero even for empty input.
    /// Negative factors reflect the region with native filled-side handling; no separate receipt is returned.
    pub fn scale(&self, x: f64, y: f64) -> Result<Self, JsValue> {
        curve::scaled(
            &self.inner,
            crate::wasm::mesh_js::geometry_scalar(x, "x")?,
            crate::wasm::mesh_js::geometry_scalar(y, "y")?,
        )
        .map(Into::into)
        .map_err(crate::wasm::mesh_js::geometry_error)
    }

    #[wasm_bindgen(js_name = containsXY)]
    /// Strict membership. Only a certified boundary returns an absent value;
    /// an undecidable query throws. Use `containsXYWithContext` for typed status.
    pub fn contains_xy(&self, x: f64, y: f64) -> Result<Option<bool>, JsValue> {
        curve::contains_xy_with_context(
            &self.inner,
            crate::wasm::mesh_js::geometry_scalar(x, "x")?,
            crate::wasm::mesh_js::geometry_scalar(y, "y")?,
            crate::GeometryContext::STRICT,
        )
        .map(GeometryOutcome::into_value)
        .map_err(crate::wasm::mesh_js::geometry_error)
    }

    /// Applies [`curve::transformed_with_context`]'s planar matrix contract and retains operation certainty.
    /// The full 4-by-4 matrix need not be invertible if its XY-plane restriction is nonsingular.
    #[wasm_bindgen(js_name = transformWithContext)]
    pub fn transform_with_context(
        &self,
        matrix: &Matrix4Js,
        approximate_512: bool,
    ) -> Result<CurveBooleanResultJs, JsValue> {
        let context = geometry_context(approximate_512);
        curve::transformed_with_context(&self.inner, &matrix.inner, context)
            .map(Into::into)
            .map_err(crate::wasm::mesh_js::geometry_error)
    }

    /// Translates by finite XY offsets under the chosen native policy and returns the consumed certainty.
    #[wasm_bindgen(js_name = translateWithContext)]
    pub fn translate_with_context(
        &self,
        x: f64,
        y: f64,
        approximate_512: bool,
    ) -> Result<CurveBooleanResultJs, JsValue> {
        let context = geometry_context(approximate_512);
        curve::translated_with_context(
            &self.inner,
            crate::wasm::mesh_js::geometry_scalar(x, "x")?,
            crate::wasm::mesh_js::geometry_scalar(y, "y")?,
            context,
        )
        .map(Into::into)
        .map_err(crate::wasm::mesh_js::geometry_error)
    }

    /// Rotates by finite counterclockwise degrees about the origin and retains selected-policy certainty.
    #[wasm_bindgen(js_name = rotateWithContext)]
    pub fn rotate_with_context(
        &self,
        degrees: f64,
        approximate_512: bool,
    ) -> Result<CurveBooleanResultJs, JsValue> {
        let context = geometry_context(approximate_512);
        curve::rotated_with_context(
            &self.inner,
            crate::wasm::mesh_js::geometry_scalar(degrees, "degrees")?,
            context,
        )
        .map(Into::into)
        .map_err(crate::wasm::mesh_js::geometry_error)
    }

    /// Applies finite nonzero axis factors, including reflections, and retains native decision certainty.
    /// A zero factor fails even for an empty region.
    #[wasm_bindgen(js_name = scaleWithContext)]
    pub fn scale_with_context(
        &self,
        x: f64,
        y: f64,
        approximate_512: bool,
    ) -> Result<CurveBooleanResultJs, JsValue> {
        let context = geometry_context(approximate_512);
        curve::scaled_with_context(
            &self.inner,
            crate::wasm::mesh_js::geometry_scalar(x, "x")?,
            crate::wasm::mesh_js::geometry_scalar(y, "y")?,
            context,
        )
        .map(Into::into)
        .map_err(crate::wasm::mesh_js::geometry_error)
    }

    /// Classifies an exactly promoted finite XY point, preserving boundary and unresolved statuses.
    /// Inside/outside supply a Boolean; a completed boundary has no Boolean but
    /// retains certainty. Native blockers become `Uncertain` with debug reason text
    /// and no receipt; other failures throw `GeometryError`. Empty geometry is outside.
    #[wasm_bindgen(js_name = containsXYWithContext)]
    pub fn contains_xy_with_context(
        &self,
        x: f64,
        y: f64,
        approximate_512: bool,
    ) -> Result<GeometryBoolResultJs, JsValue> {
        let point = hypercurve::Point2::new(
            crate::wasm::mesh_js::geometry_scalar(x, "x")?,
            crate::wasm::mesh_js::geometry_scalar(y, "y")?,
        );
        let evaluated = hypercurve::evaluate_under(
            geometry_context(approximate_512).predicate_policy(),
            || self.inner.classify_point(&point.into()),
        );
        let certainty = match evaluated.certainty() {
            CurveCertainty::Certified => GeometryCertainty::Certified,
            CurveCertainty::Approximate512Consumed => {
                GeometryCertainty::Approximate512Consumed
            },
        };
        match evaluated.into_unverified() {
            Ok(location) => Ok(GeometryOutcome {
                value: match location {
                    hypercurve::RegionPointLocation::Inside => Some(true),
                    hypercurve::RegionPointLocation::Outside => Some(false),
                    hypercurve::RegionPointLocation::Boundary => None,
                },
                certainty,
            }
            .into()),
            Err(hypercurve::ExactCurveError::Blocked(blocker)) => {
                Ok(GeometryBoolResultJs::unresolved(
                    GeometryQueryStatusJs::Uncertain,
                    format!("{:?}", blocker.reason()),
                ))
            },
            Err(error) => Err(crate::wasm::mesh_js::geometry_error(error.into())),
        }
    }

    /// Returns native bounds and local certainty; successful empty geometry has no corner values.
    /// Numeric getters on the returned native corners are separate lossy views.
    #[wasm_bindgen(js_name = boundingBoxWithContext)]
    pub fn bounding_box_with_context(
        &self,
        approximate_512: bool,
    ) -> Result<GeometryBoundsResultJs, JsValue> {
        curve::bounding_box_with_context(&self.inner, geometry_context(approximate_512))
            .map(Into::into)
            .map_err(crate::wasm::mesh_js::geometry_error)
    }

    /// Extrudes from Z zero to finite signed nonzero height, retaining local certainty and mesh history.
    /// Validates height even for empty input; uses [`curve::try_extrude`]'s mixed
    /// exact/sampled profiles and mesh checks, not an analytic-source fidelity proof.
    pub fn extrude(
        &self,
        height: f64,
        approximate_512: bool,
    ) -> Result<GeometryMeshResultJs, JsValue> {
        curve::try_extrude(
            &self.inner,
            crate::wasm::mesh_js::geometry_scalar(height, "height")?,
            geometry_context(approximate_512),
        )
        .map(Into::into)
        .map_err(crate::wasm::mesh_js::geometry_error)
    }

    /// Builds [`curve::revolve`]'s segmented swept boundary about Y from finite signed degrees.
    /// X is signed radius. Requires at least two segments for a partial turn or
    /// three for full coverage; zero and oversized noninteger turns fail before
    /// empty handling. Partial loops crossing the axis fail. The sampled result
    /// and its receipt do not imply an analytic or embedded solid; use `revolveSolid`
    /// for regularized partial material. Invalid angle ingress returns a string.
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
            geometry_context(approximate_512),
        )
        .map(Into::into)
        .map_err(crate::wasm::mesh_js::geometry_error)
    }

    /// Builds [`curve::revolve_solid`]'s regularized material from a segmented Y-axis revolution.
    /// Admits axis-crossing profiles and resolves overlapping pieces through native
    /// arrangement. Shares `revolve`'s angle/count domain; closed-PWN output can
    /// have a nonmanifold boundary. Invalid angle ingress returns a string.
    #[wasm_bindgen(js_name = revolveSolid)]
    pub fn revolve_solid(
        &self,
        angle_degrees: f64,
        segments: usize,
        approximate_512: bool,
    ) -> Result<GeometryMeshResultJs, JsValue> {
        curve::revolve_solid(
            &self.inner,
            real_from_js_named(angle_degrees, "angleDegrees")?,
            segments,
            geometry_context(approximate_512),
        )
        .map(Into::into)
        .map_err(crate::wasm::mesh_js::geometry_error)
    }

    /// Consumes the supplied point handles and sweeps along their native 3D positions.
    /// Uses [`curve::try_sweep`]'s minimal-rotation frames, adjacent-point coalescing,
    /// closed-loop seam and fixed triangle bands. Revisited points/reversals fail;
    /// parameters are checked even for empty profiles. Retains certainty and mesh
    /// history, without a total work bound or proof of an embedded analytic sweep.
    pub fn sweep(
        &self,
        path: Vec<Point3Js>,
        approximate_512: bool,
    ) -> Result<GeometryMeshResultJs, JsValue> {
        let path = path.into_iter().map(|point| point.inner).collect::<Vec<_>>();
        curve::try_sweep(&self.inner, &path, geometry_context(approximate_512))
            .map(Into::into)
            .map_err(crate::wasm::mesh_js::geometry_error)
    }

    /// Connects `slices + 1` sections with linearly varying height, degree twist and XY scale.
    /// Requires nonzero finite height, at least one slice and both end scales
    /// positive or both zero (an apex). Uses [`curve::extrude_twisted`]'s fixed
    /// triangle bands and validation; self-overlap is not regularized. Scalar
    /// ingress failures are strings. No construction-wide work/allocation bound is added.
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
            geometry_context(approximate_512),
        )
        .map(Into::into)
        .map_err(crate::wasm::mesh_js::geometry_error)
    }

    /// Constructs `[0,width] × [0,height]` strictly from finite nonnegative dimensions.
    /// A zero dimension yields canonical empty only after both inputs are validated.
    pub fn rectangle(width: f64, height: f64) -> Result<Self, JsValue> {
        curve::rectangle(
            crate::wasm::mesh_js::geometry_scalar(width, "width")?,
            crate::wasm::mesh_js::geometry_scalar(height, "height")?,
        )
        .map(Into::into)
        .map_err(crate::wasm::mesh_js::geometry_error)
    }

    /// Constructs an origin-corner square from a finite nonnegative width; zero yields canonical empty.
    pub fn square(width: f64) -> Result<Self, JsValue> {
        curve::square(crate::wasm::mesh_js::geometry_scalar(width, "width")?)
            .map(Into::into)
            .map_err(crate::wasm::mesh_js::geometry_error)
    }

    /// Constructs a regular polygon centered at the origin, starting at `(radius,0)` counterclockwise.
    /// Requires finite nonnegative radius and at least three segments, even at zero
    /// radius. Native exact samples are joined by chords, not analytic circular arcs.
    pub fn circle(radius: f64, segments: usize) -> Result<Self, JsValue> {
        curve::circle(
            crate::wasm::mesh_js::geometry_scalar(radius, "radius")?,
            segments,
        )
        .map(Into::into)
        .map_err(crate::wasm::mesh_js::geometry_error)
    }

    /// Constructs an origin-centered sampled ellipse using full finite nonnegative axis lengths.
    /// Requires at least three segments even for collapsed input; samples start at
    /// `(width/2,0)`. Straight chords carry no analytic ellipse error guarantee.
    pub fn ellipse(width: f64, height: f64, segments: usize) -> Result<Self, JsValue> {
        curve::ellipse(
            crate::wasm::mesh_js::geometry_scalar(width, "width")?,
            crate::wasm::mesh_js::geometry_scalar(height, "height")?,
            segments,
        )
        .map(Into::into)
        .map_err(crate::wasm::mesh_js::geometry_error)
    }

    /// Constructs the triangle `(0,0),(width,0),(0,height)` from finite nonnegative dimensions.
    /// Zero dimensions yield empty only after both inputs are validated strictly.
    #[wasm_bindgen(js_name = rightTriangle)]
    pub fn right_triangle(width: f64, height: f64) -> Result<Self, JsValue> {
        curve::right_triangle(
            crate::wasm::mesh_js::geometry_scalar(width, "width")?,
            crate::wasm::mesh_js::geometry_scalar(height, "height")?,
        )
        .map(Into::into)
        .map_err(crate::wasm::mesh_js::geometry_error)
    }

    /// Constructs an origin-centered regular polygon with finite nonnegative circumradius and at least three sides.
    /// Starts on positive X and traverses counterclockwise; zero radius yields empty after validation.
    #[wasm_bindgen(js_name = regularNgon)]
    pub fn regular_ngon(sides: usize, radius: f64) -> Result<Self, JsValue> {
        curve::regular_ngon(
            sides,
            crate::wasm::mesh_js::geometry_scalar(radius, "radius")?,
        )
        .map(Into::into)
        .map_err(crate::wasm::mesh_js::geometry_error)
    }

    /// Constructs a polygon alternating radii, with `points` outer tips and twice as many vertices.
    /// Requires at least three tips and finite `outer_radius > inner_radius > 0`;
    /// starts on positive X. This is a simple alternating-radius boundary, not a crossing star path.
    pub fn star(points: usize, outer_radius: f64, inner_radius: f64) -> Result<Self, JsValue> {
        curve::star(
            points,
            crate::wasm::mesh_js::geometry_scalar(outer_radius, "outerRadius")?,
            crate::wasm::mesh_js::geometry_scalar(inner_radius, "innerRadius")?,
        )
        .map(Into::into)
        .map_err(crate::wasm::mesh_js::geometry_error)
    }

    /// Joins an origin tip to a sampled upper semicircle as specified by [`curve::teardrop`].
    /// Requires positive finite dimensions, `length > width/2` and at least two
    /// semicircle chords. The top reaches length only when its angular sample is present.
    pub fn teardrop(width: f64, length: f64, segments: usize) -> Result<Self, JsValue> {
        curve::teardrop(
            crate::wasm::mesh_js::geometry_scalar(width, "width")?,
            crate::wasm::mesh_js::geometry_scalar(length, "length")?,
            segments,
        )
        .map(Into::into)
        .map_err(crate::wasm::mesh_js::geometry_error)
    }

    /// Constructs [`curve::egg`]'s chordal profile, centered by its sampled bounds.
    /// Requires positive finite dimensions and at least three samples; normalized
    /// sample extrema, not continuous extrema, establish the requested dimensions.
    pub fn egg(width: f64, length: f64, segments: usize) -> Result<Self, JsValue> {
        curve::egg(
            crate::wasm::mesh_js::geometry_scalar(width, "width")?,
            crate::wasm::mesh_js::geometry_scalar(length, "length")?,
            segments,
        )
        .map(Into::into)
        .map_err(crate::wasm::mesh_js::geometry_error)
    }

    /// Subtracts concentric sampled polygons with selected-policy certainty.
    /// Requires positive finite inner diameter/thickness and at least three segments.
    /// Thickness adds to circumradius; edge separation is `thickness*cos(pi/segments)`.
    /// See [`curve::ring`] for the polygonal annulus contract.
    pub fn ring(
        inner_diameter: f64,
        thickness: f64,
        segments: usize,
        approximate_512: bool,
    ) -> Result<CurveBooleanResultJs, JsValue> {
        curve::ring(
            crate::wasm::mesh_js::geometry_scalar(inner_diameter, "innerDiameter")?,
            crate::wasm::mesh_js::geometry_scalar(thickness, "thickness")?,
            segments,
            geometry_context(approximate_512),
        )
        .map(Into::into)
        .map_err(crate::wasm::mesh_js::geometry_error)
    }

    /// Constructs [`curve::heart`]'s sampled polygon normalized to `[0,width] × [0,height]`.
    /// Requires positive finite dimensions and at least eight samples; no analytic-curve bound is supplied.
    pub fn heart(width: f64, height: f64, segments: usize) -> Result<Self, JsValue> {
        curve::heart(
            crate::wasm::mesh_js::geometry_scalar(width, "width")?,
            crate::wasm::mesh_js::geometry_scalar(height, "height")?,
            segments,
        )
        .map(Into::into)
        .map_err(crate::wasm::mesh_js::geometry_error)
    }

    /// Subtracts an X-shifted sampled inner circle with selected-policy certainty.
    /// Requires finite `outer_radius > inner_radius > 0`, finite offset and at least
    /// six segments. Offset has no overlap/sign restriction, so the result need not
    /// look crescent-shaped; see [`curve::crescent`].
    pub fn crescent(
        outer_radius: f64,
        inner_radius: f64,
        offset: f64,
        segments: usize,
        approximate_512: bool,
    ) -> Result<CurveBooleanResultJs, JsValue> {
        curve::crescent(
            crate::wasm::mesh_js::geometry_scalar(outer_radius, "outerRadius")?,
            crate::wasm::mesh_js::geometry_scalar(inner_radius, "innerRadius")?,
            crate::wasm::mesh_js::geometry_scalar(offset, "offset")?,
            segments,
            geometry_context(approximate_512),
        )
        .map(Into::into)
        .map_err(crate::wasm::mesh_js::geometry_error)
    }
}

impl Default for CurveRegionJs {
    /// Constructs the same canonical empty region as the JS constructor.
    fn default() -> Self {
        Self::new()
    }
}
