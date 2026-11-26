//! JavaScript wrapper around native [`hypermesh::TriangleMesh`].
//!
//! Modeling retains native coordinates and construction qualifications; finite
//! display arrays and STL are explicit projection boundaries. Context outcomes
//! carry local predicate certainty independently of that history. Numeric inputs
//! are promoted from finite binary64 values exactly. Integer ABI arguments are
//! checked by native operations after binding conversion; only the explicit
//! floating-point Minkowski limit arguments receive integer/range checks here.

use crate::solid::{self, SolidExt};
use crate::wasm::{
    context_js::{GeometryBoolResultJs, GeometryBoundsResultJs, GeometryQueryStatusJs},
    curve_js::GeometryMeshResultJs,
    geometry_context,
    io_error_js::io_error,
};
use crate::wasm::{
    matrix_js::Matrix4Js, plane_js::PlaneJs, point_js::Point3Js, real_from_js, real_to_js,
};
use hyperlattice::Point3;
use hypermesh::{Triangle, TriangleMesh};
use js_sys::{Float64Array, Object, Reflect, Uint32Array};
use wasm_bindgen::prelude::*;

/// Maps a native mesh failure through modeling classification into a host-owned JS Error.
fn mesh_error(error: hypermesh::HypermeshError) -> JsValue {
    geometry_error(error.into())
}

/// Preserves a modeling failure's stable category and display message in a JS Error.
/// Nested native payloads/source objects are not retained by this conversion.
pub(super) fn geometry_error(error: crate::errors::ValidationError) -> JsValue {
    geometry_exception(&error.to_string(), error.kind())
}

/// Creates a `GeometryError` with its category in `code` and owned diagnostic text.
/// Assumes an ordinary fresh Error accepts properties: reflection exceptions panic,
/// while a false assignment result is not checked. The result has no native handle.
fn geometry_exception(message: &str, kind: crate::errors::ValidationErrorKind) -> JsValue {
    let exception = js_sys::Error::new(message);
    exception.set_name("GeometryError");
    Reflect::set(
        &exception,
        &JsValue::from_str("code"),
        &JsValue::from_str(kind.as_str()),
    )
    .expect("new Error object accepts a code property");
    exception.into()
}

/// Promotes a finite binary64 number exactly or throws an axis/field-named `invalid_input` Error.
pub(super) fn geometry_scalar(value: f64, name: &str) -> Result<crate::Real, JsValue> {
    real_from_js(value).ok_or_else(|| {
        geometry_exception(
            &format!("{name} must be finite"),
            crate::errors::ValidationErrorKind::InvalidInput,
        )
    })
}

/// Requires an integral finite count in `0..=u32::MAX`, then converts it to native `usize`.
/// Rejects fractional, negative and excessive counts before geometry/empty shortcuts.
fn geometry_count(value: f64, name: &str) -> Result<usize, JsValue> {
    if value.is_finite()
        && (0.0..=f64::from(u32::MAX)).contains(&value)
        && value.fract() == 0.0
    {
        Ok(value as usize)
    } else {
        Err(geometry_exception(
            &format!("{name} must be a finite integer between 0 and {}", u32::MAX),
            crate::errors::ValidationErrorKind::InvalidInput,
        ))
    }
}

/// Computes a checked native unit cross product for one oriented face.
/// Degenerate, unknown-zero and scalar failures all become absence, causing display
/// normal accumulation to skip that face; this is not a topology validation pass.
fn face_normal(points: &[Point3; 3]) -> Option<hyperlattice::Vector3> {
    (&points[1] - &points[0])
        .unit_cross_checked(&(&points[2] - &points[0]))
        .ok()
}

/// Native triangle geometry exposed to JavaScript.
///
/// Native modeling failures throw ordinary `GeometryError` objects with a stable
/// `code` from `GeometryValidationError` and the native diagnostic in `message`.
/// Boolean, hull, membership and subdivision errors formerly thrown as strings
/// now use this same category-bearing error contract. Errors are owned by
/// JavaScript and remain usable after mesh handles are freed. Interchange methods
/// expose their format failures as `GeometryIoError`.
#[wasm_bindgen]
pub struct MeshJs {
    /// Native geometry and dependency qualifications; public Rust conversion can wrap unvalidated meshes.
    pub(crate) inner: TriangleMesh,
}

impl From<TriangleMesh> for MeshJs {
    /// Moves geometry and existing evidence into the wrapper without additional validation or projection.
    fn from(inner: TriangleMesh) -> Self {
        Self { inner }
    }
}

#[wasm_bindgen]
#[allow(clippy::missing_const_for_fn)]
impl MeshJs {
    /// Construction qualifications remain queryable after outcome extraction.
    #[wasm_bindgen(getter, js_name = constructionEvidence)]
    pub fn construction_evidence(&self) -> super::context_js::ConstructionEvidenceJs {
        super::context_js::ConstructionEvidenceJs {
            inner: self.inner.construction_evidence(),
        }
    }

    /// Export immutable bytes with a bound construction receipt. Plain byte
    /// extraction alone cannot preserve that receipt through reimport.
    #[cfg(feature = "stl-io")]
    #[wasm_bindgen(js_name = toStlBinaryArtifact)]
    pub fn to_stl_binary_artifact(&self) -> Result<BinaryStlArtifactJs, JsValue> {
        crate::io::stl::to_stl_binary_artifact(&self.inner, "mesh")
            .map(|inner| BinaryStlArtifactJs { inner })
            .map_err(|error| io_error(error, "STL"))
    }

    /// Constructs a canonical empty mesh with no position or triangle rows.
    #[wasm_bindgen(constructor)]
    pub fn new() -> Self {
        solid::empty().into()
    }

    /// Import finite coordinate triples and in-range index triples.
    ///
    /// Invalid buffers throw `GeometryError` with code `invalid_input` before
    /// constructing a native mesh. Topology is validated by consuming operations.
    /// Promotes each finite binary coordinate exactly; repeated/degenerate faces,
    /// open surfaces and unused positions can be stored here. Raw arrays supply
    /// no preceding construction history. Canonical emptiness requires both buffers empty.
    #[wasm_bindgen(js_name = fromArrays)]
    pub fn from_arrays(positions: Vec<f64>, indices: Vec<u32>) -> Result<Self, JsValue> {
        if !positions.len().is_multiple_of(3) || !indices.len().is_multiple_of(3) {
            return Err(geometry_exception(
                "positions and indices must contain complete triples",
                crate::errors::ValidationErrorKind::InvalidInput,
            ));
        }
        let positions = positions
            .as_chunks::<3>()
            .0
            .iter()
            .map(|row| {
                Some(Point3::new(
                    real_from_js(row[0])?,
                    real_from_js(row[1])?,
                    real_from_js(row[2])?,
                ))
            })
            .collect::<Option<Vec<_>>>()
            .ok_or_else(|| {
                geometry_exception(
                    "positions must be finite",
                    crate::errors::ValidationErrorKind::InvalidInput,
                )
            })?;
        let triangles = indices
            .as_chunks::<3>()
            .0
            .iter()
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
            .ok_or_else(|| {
                geometry_exception(
                    "an index is outside the position array",
                    crate::errors::ValidationErrorKind::InvalidInput,
                )
            })?;
        Ok(TriangleMesh::new(positions, triangles).into())
    }

    /// Reports whether there are no triangles, even if unused position rows remain.
    /// This storage test is weaker than the canonical-empty contract of solid operations.
    #[wasm_bindgen(js_name = isEmpty)]
    pub fn is_empty(&self) -> bool {
        self.inner.triangles.is_empty()
    }

    /// Returns stored triangle count, saturating at `u32::MAX` on a wider host.
    #[wasm_bindgen(js_name = triangleCount)]
    pub fn triangle_count(&self) -> u32 {
        u32::try_from(self.inner.triangles.len()).unwrap_or(u32::MAX)
    }

    /// Returns all stored position rows, including unused ones, saturating at `u32::MAX` on a wider host.
    #[wasm_bindgen(js_name = vertexCount)]
    pub fn vertex_count(&self) -> u32 {
        u32::try_from(self.inner.positions.len()).unwrap_or(u32::MAX)
    }

    /// Copies display positions/normals to Float64Array and triangle indices to Uint32Array.
    /// Positions use lossy views, with NaN when a finite view is unavailable. Normals
    /// are normalized sums of unit face normals, equally weighting incident faces;
    /// failed face normalization is skipped and zero sums remain zero. Summation
    /// and final normalization are binary64. Index casts are unchecked on a wider host.
    /// No topology validation, post-projection collapse check or evidence receipt
    /// is supplied. Invalid indices can panic, as can JS property-setting exceptions.
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
        for (normal, count) in normals.as_chunks_mut::<3>().0.iter_mut().zip(counts) {
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

    /// Builds the complete display-array object and returns its copied position array.
    /// Inherits [`Self::to_arrays`]'s projection, validation and panic limits.
    pub fn positions(&self) -> Float64Array {
        Reflect::get(&self.to_arrays(), &"positions".into())
            .expect("positions exist")
            .unchecked_into()
    }

    /// Builds the complete display-array object and returns its copied smooth-normal array.
    /// Inherits [`Self::to_arrays`]'s skipped-face and finite-arithmetic behavior.
    pub fn normals(&self) -> Float64Array {
        Reflect::get(&self.to_arrays(), &"normals".into())
            .expect("normals exist")
            .unchecked_into()
    }

    /// Builds the complete display-array object and returns its copied index array.
    /// Still computes positions/normals and inherits [`Self::to_arrays`]'s panic limits.
    pub fn indices(&self) -> Uint32Array {
        Reflect::get(&self.to_arrays(), &"indices".into())
            .expect("indices exist")
            .unchecked_into()
    }

    /// Strict interior of the regularized nonzero-winding solid. Internal
    /// source faces follow represented-set membership; its boundary is excluded.
    /// Canonical empty geometry returns false. Malformed closed-solid input
    /// throws GeometryError with code invalid_input, even
    /// when the point is outside its bounds or on a source triangle.
    #[wasm_bindgen(js_name = containsPoint)]
    pub fn contains_point(&self, point: &Point3Js) -> Result<bool, JsValue> {
        solid::contains_point(&self.inner, &point.inner).map_err(mesh_error)
    }

    /// Computes strict regularized union, validating both operands even beside canonical empty geometry.
    /// Returns no separate receipt; native construction qualifications remain attached.
    pub fn union(&self, other: &Self) -> Result<Self, JsValue> {
        self.inner
            .try_union(&other.inner)
            .map(Into::into)
            .map_err(mesh_error)
    }

    /// Computes strict regularized `self` minus `other`, retaining mesh history without a separate receipt.
    pub fn difference(&self, other: &Self) -> Result<Self, JsValue> {
        self.inner
            .try_difference(&other.inner)
            .map(Into::into)
            .map_err(mesh_error)
    }

    /// Computes strict regularized intersection, retaining mesh history without a separate receipt.
    pub fn intersection(&self, other: &Self) -> Result<Self, JsValue> {
        self.inner
            .try_intersection(&other.inner)
            .map(Into::into)
            .map_err(mesh_error)
    }

    /// Computes strict regularized symmetric difference, retaining mesh history without a separate receipt.
    pub fn xor(&self, other: &Self) -> Result<Self, JsValue> {
        self.inner
            .try_xor(&other.inner)
            .map(Into::into)
            .map_err(mesh_error)
    }

    /// Computes native regularized union under the selected policy with local certainty and inherited history.
    /// Both operands are validated before applying empty identities.
    #[wasm_bindgen(js_name = unionWithContext)]
    pub fn union_with_context(
        &self,
        other: &Self,
        approximate_512: bool,
    ) -> Result<GeometryMeshResultJs, JsValue> {
        self.inner
            .try_union_with_context(&other.inner, geometry_context(approximate_512))
            .map(Into::into)
            .map_err(mesh_error)
    }

    /// Computes native regularized ordered difference with selected-policy certainty and inherited history.
    #[wasm_bindgen(js_name = differenceWithContext)]
    pub fn difference_with_context(
        &self,
        other: &Self,
        approximate_512: bool,
    ) -> Result<GeometryMeshResultJs, JsValue> {
        self.inner
            .try_difference_with_context(&other.inner, geometry_context(approximate_512))
            .map(Into::into)
            .map_err(mesh_error)
    }

    /// Computes native regularized intersection with selected-policy certainty and inherited history.
    #[wasm_bindgen(js_name = intersectionWithContext)]
    pub fn intersection_with_context(
        &self,
        other: &Self,
        approximate_512: bool,
    ) -> Result<GeometryMeshResultJs, JsValue> {
        self.inner
            .try_intersection_with_context(&other.inner, geometry_context(approximate_512))
            .map(Into::into)
            .map_err(mesh_error)
    }

    /// Computes native regularized symmetric difference with selected-policy certainty and inherited history.
    #[wasm_bindgen(js_name = xorWithContext)]
    pub fn xor_with_context(
        &self,
        other: &Self,
        approximate_512: bool,
    ) -> Result<GeometryMeshResultJs, JsValue> {
        self.inner
            .try_xor_with_context(&other.inner, geometry_context(approximate_512))
            .map(Into::into)
            .map_err(mesh_error)
    }

    /// Applies [`solid::transform_with_context`]'s nonsingular projective map and retains local certainty.
    /// Rejects zero determinant even for empty input, corrects reflected winding,
    /// and rejects positions/triangles at or crossing infinity. This validates
    /// transform/index conditions, not source closure, manifoldness or embedding.
    #[wasm_bindgen(js_name = transformWithContext)]
    pub fn transform_with_context(
        &self,
        matrix: &Matrix4Js,
        approximate_512: bool,
    ) -> Result<GeometryMeshResultJs, JsValue> {
        let context = geometry_context(approximate_512);
        solid::transform_with_context(&self.inner, &matrix.inner, context)
            .map(Into::into)
            .map_err(geometry_error)
    }

    /// Translates native positions by finite XYZ offsets under the chosen policy, preserving history and receipt.
    #[wasm_bindgen(js_name = translateWithContext)]
    pub fn translate_with_context(
        &self,
        x: f64,
        y: f64,
        z: f64,
        approximate_512: bool,
    ) -> Result<GeometryMeshResultJs, JsValue> {
        let context = geometry_context(approximate_512);
        solid::transform_with_context(
            &self.inner,
            &hyperlattice::Matrix4::affine_translation([
                geometry_scalar(x, "x")?,
                geometry_scalar(y, "y")?,
                geometry_scalar(z, "z")?,
            ]),
            context,
        )
        .map(Into::into)
        .map_err(geometry_error)
    }

    /// Applies finite Euler degrees in X-then-Y-then-Z order, retaining native trig values and certainty.
    #[wasm_bindgen(js_name = rotateWithContext)]
    pub fn rotate_with_context(
        &self,
        x: f64,
        y: f64,
        z: f64,
        approximate_512: bool,
    ) -> Result<GeometryMeshResultJs, JsValue> {
        let context = geometry_context(approximate_512);
        solid::rotate_with_context(
            &self.inner,
            geometry_scalar(x, "x")?,
            geometry_scalar(y, "y")?,
            geometry_scalar(z, "z")?,
            context,
        )
        .map(Into::into)
        .map_err(geometry_error)
    }

    /// Applies finite nonzero XYZ factors with selected-policy certainty and reflection winding correction.
    /// Factor validation precedes geometry handling, including canonical empty input.
    #[wasm_bindgen(js_name = scaleWithContext)]
    pub fn scale_with_context(
        &self,
        x: f64,
        y: f64,
        z: f64,
        approximate_512: bool,
    ) -> Result<GeometryMeshResultJs, JsValue> {
        let context = geometry_context(approximate_512);
        solid::scale_with_context(
            &self.inner,
            geometry_scalar(x, "x")?,
            geometry_scalar(y, "y")?,
            geometry_scalar(z, "z")?,
            context,
        )
        .map(Into::into)
        .map_err(geometry_error)
    }

    /// Translates the bounds midpoint to the origin with selected-policy certainty and inherited history.
    /// Bounds include unused positions; no positions returns a clone after native validation.
    #[wasm_bindgen(js_name = centerWithContext)]
    pub fn center_with_context(
        &self,
        approximate_512: bool,
    ) -> Result<GeometryMeshResultJs, JsValue> {
        let context = geometry_context(approximate_512);
        solid::center_with_context(&self.inner, context)
            .map(Into::into)
            .map_err(geometry_error)
    }

    /// Translates the selected minimum Z to zero with local certainty and inherited history.
    /// Bounds include unused positions; no positions returns a clone.
    #[wasm_bindgen(js_name = floatOnZWithContext)]
    pub fn float_on_z_with_context(
        &self,
        approximate_512: bool,
    ) -> Result<GeometryMeshResultJs, JsValue> {
        let context = geometry_context(approximate_512);
        solid::float_with_context(&self.inner, context)
            .map(Into::into)
            .map_err(geometry_error)
    }

    /// Reflects across the native plane and reverses winding, retaining reflection/transform certainty.
    /// Validates a nonzero plane normal before empty handling; unit length is not required.
    #[wasm_bindgen(js_name = mirrorWithContext)]
    pub fn mirror_with_context(
        &self,
        plane: &PlaneJs,
        approximate_512: bool,
    ) -> Result<GeometryMeshResultJs, JsValue> {
        let context = geometry_context(approximate_512);
        solid::mirror_with_context(&self.inner, &plane.inner, context)
            .map(Into::into)
            .map_err(geometry_error)
    }

    /// Constructs a 3D convex hull of all stored positions with selected-policy certainty.
    /// Source triangles are ignored; insufficient/degenerate point sets fail, including empty input.
    #[wasm_bindgen(js_name = convexHullWithContext)]
    pub fn convex_hull_with_context(
        &self,
        approximate_512: bool,
    ) -> Result<GeometryMeshResultJs, JsValue> {
        let context = geometry_context(approximate_512);
        solid::convex_hull_with_context(&self.inner, context)
            .map(Into::into)
            .map_err(mesh_error)
    }

    /// Computes the regularized represented-set sum of bounded closed-PWN solids with local certainty.
    /// Validates both operands before empty identities and preserves their history;
    /// uses native default collection limits with no cancellation flag or total-work bound.
    #[wasm_bindgen(js_name = minkowskiSumWithContext)]
    pub fn minkowski_sum_with_context(
        &self,
        other: &Self,
        approximate_512: bool,
    ) -> Result<GeometryMeshResultJs, JsValue> {
        let context = geometry_context(approximate_512);
        solid::minkowski_sum_with_context(&self.inner, &other.inner, context)
            .map(Into::into)
            .map_err(geometry_error)
    }

    /// Represented-set sum with count limits; these are not time or heap-byte bounds.
    /// Counts must be finite integers from zero through `u32::MAX`.
    /// Limits cap live decomposition cells per operand, accumulated convex pair
    /// sums and generated vertex sums per pair, respectively. All three counts
    /// are checked before operand handling. No cancellation flag is exposed;
    /// exhaustion throws a categorized error without publishing a partial mesh.
    #[wasm_bindgen(js_name = minkowskiSumWithLimits)]
    pub fn minkowski_sum_with_limits(
        &self,
        other: &Self,
        approximate_512: bool,
        max_cells: f64,
        max_pair_sums: f64,
        max_pair_points: f64,
    ) -> Result<GeometryMeshResultJs, JsValue> {
        let context = geometry_context(approximate_512);
        let options = hypermesh::MinkowskiOptions {
            max_cells: geometry_count(max_cells, "maxCells")?,
            max_pair_sums: geometry_count(max_pair_sums, "maxPairSums")?,
            max_pair_points: geometry_count(max_pair_points, "maxPairPoints")?,
            cancellation: None,
        };
        solid::minkowski_sum_with_options(&self.inner, &other.inner, &options, context)
            .map(Into::into)
            .map_err(geometry_error)
    }

    /// Convex hull of stored-position sums, including unused positions.
    /// Ignores triangle indices/closure and convexifies gaps; preserves both
    /// construction histories and local certainty. This differs from the represented-set sum.
    #[wasm_bindgen(js_name = minkowskiHullWithContext)]
    pub fn minkowski_hull_with_context(
        &self,
        other: &Self,
        approximate_512: bool,
    ) -> Result<GeometryMeshResultJs, JsValue> {
        let context = geometry_context(approximate_512);
        solid::minkowski_hull_with_context(&self.inner, &other.inner, context)
            .map(Into::into)
            .map_err(geometry_error)
    }

    /// Containment with the certainty of both source validation and the query.
    /// Malformed input throws GeometryError; unresolved predicates keep Unknown.
    #[wasm_bindgen(js_name = containsPointWithContext)]
    pub fn contains_point_with_context(
        &self,
        point: &Point3Js,
        approximate_512: bool,
    ) -> Result<GeometryBoolResultJs, JsValue> {
        match solid::contains_point_with_context(
            &self.inner,
            &point.inner,
            geometry_context(approximate_512),
        ) {
            Ok(outcome) => Ok(outcome.map(Some).into()),
            Err(
                error @ (hypermesh::HypermeshError::UnknownClassification
                | hypermesh::HypermeshError::PredicateUndecided { .. }),
            ) => Ok(GeometryBoolResultJs::unresolved(
                GeometryQueryStatusJs::Unknown,
                error.to_string(),
            )),
            Err(error) => Err(mesh_error(error)),
        }
    }

    /// Returns native bounds of all stored positions with selected-policy certainty.
    /// Validates indices; no positions yields absent bounds. Approximate bound
    /// selection is qualified, not a proved outward enclosure. The bounds outcome
    /// does not carry source mesh history, and point numeric getters are lossy views.
    #[wasm_bindgen(js_name = boundingBoxWithContext)]
    pub fn bounding_box_with_context(
        &self,
        approximate_512: bool,
    ) -> Result<GeometryBoundsResultJs, JsValue> {
        solid::bounding_box_with_context(&self.inner, geometry_context(approximate_512))
            .map(Into::into)
            .map_err(geometry_error)
    }

    /// Applies [`solid::transform`]'s nonsingular projective contract strictly, preserving mesh history.
    /// Rejects point/triangle crossings of infinity and returns no separate local receipt.
    pub fn transform(&self, matrix: &Matrix4Js) -> Result<Self, JsValue> {
        solid::transform(&self.inner, &matrix.inner)
            .map(Into::into)
            .map_err(geometry_error)
    }

    /// Applies a strict native translation by finite XYZ offsets, retaining history without a separate receipt.
    pub fn translate(&self, x: f64, y: f64, z: f64) -> Result<Self, JsValue> {
        Ok(self
            .inner
            .translated(
                geometry_scalar(x, "x")?,
                geometry_scalar(y, "y")?,
                geometry_scalar(z, "z")?,
            )
            .map_err(geometry_error)?
            .into())
    }

    /// Applies strict X-then-Y-then-Z Euler rotations in finite degrees, preserving native coordinates/history.
    pub fn rotate(&self, x: f64, y: f64, z: f64) -> Result<Self, JsValue> {
        solid::rotate(
            &self.inner,
            geometry_scalar(x, "x")?,
            geometry_scalar(y, "y")?,
            geometry_scalar(z, "z")?,
        )
        .map(Into::into)
        .map_err(geometry_error)
    }

    /// Scales strictly by finite nonzero axis factors, correcting winding for reflections and retaining history.
    pub fn scale(&self, x: f64, y: f64, z: f64) -> Result<Self, JsValue> {
        solid::scale(
            &self.inner,
            geometry_scalar(x, "x")?,
            geometry_scalar(y, "y")?,
            geometry_scalar(z, "z")?,
        )
        .map(Into::into)
        .map_err(geometry_error)
    }

    /// Centers strict bounds of all stored positions, returning a clone for absent bounds and no separate receipt.
    pub fn center(&self) -> Result<Self, JsValue> {
        solid::center(&self.inner)
            .map(Into::into)
            .map_err(geometry_error)
    }

    /// Moves the strict minimum Z to zero, including unused positions in the bounds; absent bounds leave a clone.
    #[wasm_bindgen(js_name = floatOnZ)]
    pub fn float_on_z(&self) -> Result<Self, JsValue> {
        solid::float(&self.inner)
            .map(Into::into)
            .map_err(geometry_error)
    }

    /// Reverses triangle winding while retaining positions/history; does not construct an unbounded set complement.
    pub fn inverse(&self) -> Self {
        solid::inverse(&self.inner).into()
    }

    /// Reflects strictly across a valid native plane with winding correction, returning no separate receipt.
    pub fn mirror(&self, plane: &PlaneJs) -> Result<Self, JsValue> {
        solid::mirror(&self.inner, &plane.inner)
            .map(Into::into)
            .map_err(geometry_error)
    }

    /// Constructs the strict 3D convex hull of stored positions; ignores source topology and retains history.
    /// Empty or insufficient-dimensional point sets fail rather than producing a successful solid.
    #[wasm_bindgen(js_name = convexHull)]
    pub fn convex_hull(&self) -> Result<Self, JsValue> {
        solid::convex_hull(&self.inner)
            .map(Into::into)
            .map_err(mesh_error)
    }

    /// Computes the strict regularized represented-set sum, validating both bounded closed-PWN operands.
    /// Retains native history without a separate local receipt; uses default collection limits.
    #[wasm_bindgen(js_name = minkowskiSum)]
    pub fn minkowski_sum(&self, other: &Self) -> Result<Self, JsValue> {
        solid::minkowski_sum(&self.inner, &other.inner)
            .map(Into::into)
            .map_err(geometry_error)
    }

    /// Strict convex hull of stored-position sums, explicitly convexifying gaps.
    #[wasm_bindgen(js_name = minkowskiHull)]
    pub fn minkowski_hull(&self, other: &Self) -> Result<Self, JsValue> {
        solid::minkowski_hull(&self.inner, &other.inner)
            .map(Into::into)
            .map_err(geometry_error)
    }

    /// Returns strict native corner handles as `{min,max}`, or JS undefined when there are no positions.
    /// Includes unused rows and validates indices. The caller owns both point
    /// handles; their numeric getters project separately. Property-setting exceptions panic.
    #[wasm_bindgen(js_name = boundingBox)]
    pub fn bounding_box(&self) -> Result<JsValue, JsValue> {
        let Some(bounds) = solid::bounding_box(&self.inner).map_err(geometry_error)? else {
            return Ok(JsValue::UNDEFINED);
        };
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

    /// Import binary or ASCII STL through strict native closed-PWN validation.
    ///
    /// Canonical empty geometry is accepted. Coordinates parse as binary32 and
    /// are promoted exactly; validation certifies that parsed geometry, not the
    /// original exact construction or analytic source. Manifoldness is a separate
    /// claim. Failure throws `GeometryIoError`; native validation's category and
    /// message remain in `cause`, and no mesh is returned. The byte buffer is not retained.
    #[wasm_bindgen(js_name = fromSTL)]
    pub fn from_stl(data: &[u8]) -> Result<Self, JsValue> {
        crate::io::stl::from_stl(data)
            .map(Into::into)
            .map_err(|error| io_error(error, "STL"))
    }

    /// Serialize binary32 STL, rejecting collapsed projected triangles.
    ///
    /// Failure throws an ordinary `GeometryIoError` object with a native `code`,
    /// `format`, and variant fields. This replaces the former string exception;
    /// inspect `message` for text and `code` for the machine-readable variant.
    #[wasm_bindgen(js_name = toSTLBinary)]
    pub fn to_stl_binary(&self) -> Result<Vec<u8>, JsValue> {
        crate::io::stl::to_stl_binary(&self.inner, "mesh")
            .map_err(|error| io_error(error, "STL"))
    }

    /// Serialize exact decimal spellings of the finite binary64 projection.
    ///
    /// Failure throws `GeometryIoError` with queryable native fields rather than
    /// a string. Reimport subsequently quantizes coordinates to binary32.
    #[wasm_bindgen(js_name = toSTLASCII)]
    pub fn to_stl_ascii(&self) -> Result<String, JsValue> {
        crate::io::stl::to_stl_ascii(&self.inner, "mesh")
            .map_err(|error| io_error(error, "STL"))
    }

    /// Splits every triangle into four per positive level using shared indexed-edge midpoints.
    /// Rejects zero even for empty input; preserves winding/history without smoothing
    /// or welding coincident distinct indices. Native overflow/index failures remain
    /// categorized, but no total-work or heap-byte bound is imposed; see [`solid::subdivide`].
    pub fn subdivide(&self, levels: u32) -> Result<Self, JsValue> {
        let levels = std::num::NonZeroU32::new(levels).ok_or_else(|| {
            geometry_exception(
                "levels must be positive",
                crate::errors::ValidationErrorKind::InvalidInput,
            )
        })?;
        solid::subdivide(&self.inner, levels)
            .map(Into::into)
            .map_err(mesh_error)
    }

    /// Constructs `[0,size]³` from finite nonnegative size; zero yields canonical empty after validation.
    pub fn cube(size: f64) -> Result<Self, JsValue> {
        Ok(solid::cube(geometry_scalar(size, "size")?)
            .map_err(geometry_error)?
            .into())
    }

    /// Constructs an origin-corner box with finite nonnegative XYZ dimensions.
    /// Validates all dimensions before a zero-dimension empty result; positive sizes give twelve triangles.
    pub fn cuboid(width: f64, length: f64, height: f64) -> Result<Self, JsValue> {
        Ok(solid::cuboid(
            geometry_scalar(width, "width")?,
            geometry_scalar(length, "length")?,
            geometry_scalar(height, "height")?,
        )
        .map_err(geometry_error)?
        .into())
    }

    /// Constructs an origin-centered sampled sphere with Y-axis poles.
    /// Requires finite nonnegative radius, at least three segments and two stacks,
    /// including at zero radius. Native exact samples define a polyhedron with
    /// sampling history, not an analytic sphere or a certified surface error bound.
    pub fn sphere(radius: f64, segments: usize, stacks: usize) -> Result<Self, JsValue> {
        Ok(
            solid::sphere(geometry_scalar(radius, "radius")?, segments, stacks)
                .map_err(geometry_error)?
                .into(),
        )
    }

    /// Constructs a sampled cylinder from the origin to `(0,0,height)` with finite signed height.
    /// Requires nonnegative radius and at least three segments; zero radius/height
    /// yields empty after validation. Native history records the polygonal sampling.
    pub fn cylinder(radius: f64, height: f64, segments: usize) -> Result<Self, JsValue> {
        Ok(solid::cylinder(
            geometry_scalar(radius, "radius")?,
            geometry_scalar(height, "height")?,
            segments,
        )
        .map_err(geometry_error)?
        .into())
    }

    /// Constructs a sampled conical frustum along signed Z using finite nonnegative end radii.
    /// Delegates sample phase, cone-tip, minimum-count and empty-collapse rules to [`solid::frustum`].
    pub fn frustum(
        bottom_radius: f64,
        top_radius: f64,
        height: f64,
        segments: usize,
    ) -> Result<Self, JsValue> {
        Ok(solid::frustum(
            geometry_scalar(bottom_radius, "bottomRadius")?,
            geometry_scalar(top_radius, "topRadius")?,
            geometry_scalar(height, "height")?,
            segments,
        )
        .map_err(geometry_error)?
        .into())
    }

    /// Scales the sampled unit sphere by finite nonnegative XYZ radii, retaining sampling history.
    /// Uses the sphere's segment/stack minima; validates dimensions/counts before zero-radius emptiness.
    pub fn ellipsoid(
        rx: f64,
        ry: f64,
        rz: f64,
        segments: usize,
        stacks: usize,
    ) -> Result<Self, JsValue> {
        Ok(solid::ellipsoid(
            geometry_scalar(rx, "rx")?,
            geometry_scalar(ry, "ry")?,
            geometry_scalar(rz, "rz")?,
            segments,
            stacks,
        )
        .map_err(geometry_error)?
        .into())
    }

    /// Constructs a sampled ring torus about Z from finite nonnegative major/minor radii.
    /// Both sample counts must be at least three. Zero minor radius yields empty
    /// after validation; otherwise major radius must exceed minor radius, with
    /// horn/spindle cases unsupported. Retains exact samples and sampling history.
    pub fn torus(
        major_radius: f64,
        minor_radius: f64,
        major_segments: usize,
        minor_segments: usize,
    ) -> Result<Self, JsValue> {
        Ok(solid::torus(
            geometry_scalar(major_radius, "majorRadius")?,
            geometry_scalar(minor_radius, "minorRadius")?,
            major_segments,
            minor_segments,
        )
        .map_err(geometry_error)?
        .into())
    }
}

/// In-process binary32 STL artifact; the bytes and source receipt are immutable.
#[cfg(feature = "stl-io")]
#[wasm_bindgen]
pub struct BinaryStlArtifactJs {
    /// Immutable native bytes bound to their retained projection/construction receipt.
    inner: crate::io::stl::BinaryStlArtifact,
}

#[cfg(feature = "stl-io")]
#[wasm_bindgen]
#[allow(clippy::missing_const_for_fn)]
impl BinaryStlArtifactJs {
    /// Returns a copy; editing it cannot alter the bound artifact.
    #[wasm_bindgen(getter)]
    pub fn bytes(&self) -> Vec<u8> {
        self.inner.bytes().to_vec()
    }

    /// Copies the artifact's retained source qualifications, including its coordinate projection boundary.
    #[wasm_bindgen(getter, js_name = constructionEvidence)]
    pub fn construction_evidence(&self) -> super::context_js::ConstructionEvidenceJs {
        super::context_js::ConstructionEvidenceJs {
            inner: self.inner.construction_evidence(),
        }
    }

    /// Consumes the artifact handle, strictly imports its bound bytes and preserves their construction history.
    /// Local validation concerns the binary32 geometry; it does not recover source
    /// coordinates or prove source fidelity. Failures throw `GeometryIoError`.
    #[wasm_bindgen(js_name = intoMesh)]
    pub fn into_mesh(self) -> Result<MeshJs, JsValue> {
        self.inner
            .import()
            .map(MeshJs::from)
            .map_err(|error| io_error(error, "STL"))
    }
}

impl Default for MeshJs {
    /// Constructs the canonical empty mesh, as the JS constructor does.
    fn default() -> Self {
        Self::new()
    }
}
