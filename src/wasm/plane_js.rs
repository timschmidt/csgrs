//! JavaScript wrapper for native Hypermesh planes.
//! A plane stores `normal · point + offset = 0` without unit normalization.
//! Context constructors/reflection expose operation-local certainty; extracting
//! the bare plane or matrix discards that separate receipt. These value wrappers
//! do not retain an inherited construction history. Failure paths here use JS strings.

use crate::context::GeometryDecisions;
use crate::wasm::{
    context_js::{GeometryMatrixResultJs, GeometryPlaneResultJs},
    geometry_context,
    point_js::Point3Js,
    point3_from_js, real_from_js_named, real_to_js,
    vector_js::Vector3Js,
};
use hyperlattice::Vector3;
use hypermesh::Plane;
use wasm_bindgen::prelude::*;

/// Owned native oriented plane with exact coefficients and optional construction receipts.
/// Rust conversion can wrap an unchecked native plane; query/construction methods
/// establish only their documented local preconditions.
#[wasm_bindgen]
pub struct PlaneJs {
    /// Native normal and additive offset; neither is normalized for display.
    pub(crate) inner: Plane,
}

#[wasm_bindgen]
impl PlaneJs {
    /// Constructs from three finite numeric points under strict decisions, consuming the receipt.
    #[wasm_bindgen(js_name = FromComponents)]
    pub fn from_components(
        ax: f64,
        ay: f64,
        az: f64,
        bx: f64,
        by: f64,
        bz: f64,
        cx: f64,
        cy: f64,
        cz: f64,
    ) -> Result<Self, JsValue> {
        Self::from_components_with_context(ax, ay, az, bx, by, bz, cx, cy, cz, false)
            .map(GeometryPlaneResultJs::into_plane)
    }

    /// Promotes three finite points exactly, then validates non-collinearity under the selected policy.
    /// Returns the native plane with local certainty; invalid numbers/geometry return strings.
    #[wasm_bindgen(js_name = FromComponentsWithContext)]
    pub fn from_components_with_context(
        ax: f64,
        ay: f64,
        az: f64,
        bx: f64,
        by: f64,
        bz: f64,
        cx: f64,
        cy: f64,
        cz: f64,
        approximate_512: bool,
    ) -> Result<GeometryPlaneResultJs, JsValue> {
        Self::from_points_with_context(
            &point3_from_js(ax, ay, az)?.into(),
            &point3_from_js(bx, by, bz)?.into(),
            &point3_from_js(cx, cy, cz)?.into(),
            approximate_512,
        )
    }

    /// Constructs the oriented plane through three native points strictly, consuming its receipt.
    #[wasm_bindgen(js_name = FromPoints)]
    pub fn from_points(a: &Point3Js, b: &Point3Js, c: &Point3Js) -> Result<Self, JsValue> {
        Self::from_points_with_context(a, b, c, false).map(GeometryPlaneResultJs::into_plane)
    }

    /// Validates non-collinearity, then constructs the plane oriented by `(b-a) × (c-a)`.
    /// Native coefficients may remove a common projective scale; the normal is
    /// not required to have unit length. Preserves local decision certainty and
    /// rejects a decided degeneracy or undecided predicate with a JS string.
    #[wasm_bindgen(js_name = FromPointsWithContext)]
    pub fn from_points_with_context(
        a: &Point3Js,
        b: &Point3Js,
        c: &Point3Js,
        approximate_512: bool,
    ) -> Result<GeometryPlaneResultJs, JsValue> {
        let decisions = GeometryDecisions::new(geometry_context(approximate_512));
        let nondegenerate = decisions.consume_mesh(
            Plane::points_are_nondegenerate(
                decisions.mesh_context(),
                &a.inner,
                &b.inner,
                &c.inner,
            )
            .map_err(|error| JsValue::from_str(&error.to_string()))?,
        );
        if !nondegenerate {
            return Err(JsValue::from_str("plane points must be non-collinear"));
        }
        Ok(decisions
            .finish(Plane::from_points(&a.inner, &b.inner, &c.inner))
            .into())
    }

    /// Imports finite normal/offset coefficients strictly, consuming the separate construction receipt.
    #[wasm_bindgen(js_name = FromNormalComponents)]
    pub fn from_normal_components(
        nx: f64,
        ny: f64,
        nz: f64,
        offset: f64,
    ) -> Result<Self, JsValue> {
        Self::from_normal_components_with_context(nx, ny, nz, offset, false)
            .map(GeometryPlaneResultJs::into_plane)
    }

    /// Imports finite normal components, then validates the normal and imports the additive offset.
    /// Returns local certainty under the selected policy; does not normalize coefficients.
    #[wasm_bindgen(js_name = FromNormalComponentsWithContext)]
    pub fn from_normal_components_with_context(
        nx: f64,
        ny: f64,
        nz: f64,
        offset: f64,
        approximate_512: bool,
    ) -> Result<GeometryPlaneResultJs, JsValue> {
        let normal = Vector3::from_xyz(
            real_from_js_named(nx, "nx")?,
            real_from_js_named(ny, "ny")?,
            real_from_js_named(nz, "nz")?,
        );
        Self::from_normal_with_context(&normal.into(), offset, approximate_512)
    }

    /// Combines a native nonzero normal and finite additive offset strictly, consuming the receipt.
    #[wasm_bindgen(js_name = FromNormal)]
    pub fn from_normal(normal: &Vector3Js, offset: f64) -> Result<Self, JsValue> {
        Self::from_normal_with_context(normal, offset, false)
            .map(GeometryPlaneResultJs::into_plane)
    }

    /// Requires positive squared normal magnitude before promoting the finite additive offset.
    /// The equation is `n · p + offset = 0`; offset is a signed-distance coefficient
    /// only when the normal is unit length. Retains local predicate certainty,
    /// with validation/uncertainty failures converted to JS strings.
    #[wasm_bindgen(js_name = FromNormalWithContext)]
    pub fn from_normal_with_context(
        normal: &Vector3Js,
        offset: f64,
        approximate_512: bool,
    ) -> Result<GeometryPlaneResultJs, JsValue> {
        let decisions = GeometryDecisions::new(geometry_context(approximate_512));
        let [x, y, z] = normal.inner.0.clone();
        validate_normal(&[x.clone(), y.clone(), z.clone()], &decisions)?;
        Ok(decisions
            .finish(Plane::from_coefficients(
                x,
                y,
                z,
                real_from_js_named(offset, "offset")?,
            ))
            .into())
    }

    /// Clones the native normal without normalization or finite projection.
    pub fn normal(&self) -> Vector3Js {
        Vector3Js::from(Vector3::from_xyz(
            self.inner.normal.x.clone(),
            self.inner.normal.y.clone(),
            self.inner.normal.z.clone(),
        ))
    }

    /// Returns a lossy additive-offset view, or NaN if no finite view is available.
    pub fn offset(&self) -> f64 {
        real_to_js(&self.inner.offset)
    }

    /// Negates normal and offset in place, preserving the plane image and reversing its orientation.
    pub fn flip(&mut self) {
        self.inner = self.inner.inverted();
    }

    /// Builds the native reflection matrix with strict decisions, consuming the separate receipt.
    #[wasm_bindgen(js_name = reflectionMatrix)]
    pub fn reflection_matrix(&self) -> Result<crate::wasm::matrix_js::Matrix4Js, JsValue> {
        self.reflection_matrix_with_context(false)
            .map(GeometryMatrixResultJs::into_matrix)
    }

    /// Builds the homogeneous reflection `p - 2(n·p + d)n/(n·n)` with local certainty.
    /// Delegates reciprocal/nonzero decisions to Hypermesh under the selected
    /// policy; failures become JS strings. Does not recover discarded construction history.
    #[wasm_bindgen(js_name = reflectionMatrixWithContext)]
    pub fn reflection_matrix_with_context(
        &self,
        approximate_512: bool,
    ) -> Result<GeometryMatrixResultJs, JsValue> {
        let decisions = GeometryDecisions::new(geometry_context(approximate_512));
        self.inner
            .reflection_matrix(decisions.mesh_context())
            .map(|outcome| decisions.finish(decisions.consume_mesh(outcome)).into())
            .map_err(|error| JsValue::from_str(&error.to_string()))
    }
}

impl From<Plane> for PlaneJs {
    /// Moves a native plane into the wrapper without validation or an attached receipt.
    fn from(inner: Plane) -> Self {
        Self { inner }
    }
}

/// Requires a positive squared norm through the shared selected-policy decision accumulator.
/// Zero/negative decisions and unresolved predicates become JS strings; successful
/// decisions contribute to the caller's receipt without normalizing the vector.
fn validate_normal(
    normal: &[hyperlattice::Real; 3],
    decisions: &GeometryDecisions,
) -> Result<(), JsValue> {
    let squared = normal.iter().fold(hyperlattice::Real::zero(), |sum, value| {
        sum + value.clone() * value.clone()
    });
    match decisions
        .decide(
            hyperlimit::classify_real_sign(&squared, decisions.predicate_policy()),
            "plane normal",
        )
        .map_err(|error| JsValue::from_str(&error.to_string()))?
    {
        hyperlimit::Sign::Positive => Ok(()),
        hyperlimit::Sign::Negative | hyperlimit::Sign::Zero => {
            Err(JsValue::from_str("plane normal must be non-zero"))
        },
    }
}
