//! JavaScript query results paired with predicate certainty.

use super::point_js::Point3Js;
use crate::{GeometryCertainty, GeometryOutcome};
use hyperlattice::Aabb;
use wasm_bindgen::prelude::*;

/// Weakest predicate certainty consumed by a completed geometry operation.
///
/// This describes topology decisions, not the fidelity of a float projection.
#[wasm_bindgen]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GeometryCertaintyJs {
    /// All consumed decisions have exact certificates.
    Certified,
    /// At least one decision consumed the authorized 512-bit terminal.
    Approximate512Consumed,
}

impl From<GeometryCertainty> for GeometryCertaintyJs {
    fn from(certainty: GeometryCertainty) -> Self {
        match certainty {
            GeometryCertainty::Certified => Self::Certified,
            GeometryCertainty::Approximate512Consumed => Self::Approximate512Consumed,
        }
    }
}

impl From<hypercurve::CurveCertainty> for GeometryCertaintyJs {
    fn from(certainty: hypercurve::CurveCertainty) -> Self {
        match certainty {
            hypercurve::CurveCertainty::Certified => Self::Certified,
            hypercurve::CurveCertainty::Approximate512Consumed => Self::Approximate512Consumed,
        }
    }
}

/// Outcome of a point query, separate from its predicate certainty.
#[wasm_bindgen]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GeometryQueryStatusJs {
    /// A Boolean membership value is available.
    Decided,
    /// The curve query placed the point on the boundary under the selected policy.
    Boundary,
    /// Hypercurve could not decide the query under the selected policy.
    Uncertain,
    /// Hypermesh could not decide the query under the selected policy.
    Unknown,
}

/// Point-query value, status, and optional certainty receipt.
///
/// Read `status` before interpreting an absent `value`: it can denote a curve
/// boundary, curve uncertainty, or an unknown mesh query. An unresolved query
/// has no certainty receipt. Mesh membership is strict containment, so mesh
/// boundary points have a decided `false` value.
#[wasm_bindgen]
pub struct GeometryBoolResultJs {
    value: Option<bool>,
    status: GeometryQueryStatusJs,
    certainty: Option<GeometryCertaintyJs>,
    reason: Option<String>,
}

impl From<GeometryOutcome<Option<bool>>> for GeometryBoolResultJs {
    fn from(outcome: GeometryOutcome<Option<bool>>) -> Self {
        Self {
            value: outcome.value,
            status: if outcome.value.is_some() {
                GeometryQueryStatusJs::Decided
            } else {
                GeometryQueryStatusJs::Boundary
            },
            certainty: Some(outcome.certainty.into()),
            reason: None,
        }
    }
}

impl GeometryBoolResultJs {
    pub(crate) fn unresolved(status: GeometryQueryStatusJs, reason: String) -> Self {
        debug_assert!(matches!(
            status,
            GeometryQueryStatusJs::Uncertain | GeometryQueryStatusJs::Unknown
        ));
        Self {
            value: None,
            status,
            certainty: None,
            reason: Some(reason),
        }
    }
}

#[wasm_bindgen]
impl GeometryBoolResultJs {
    #[wasm_bindgen(getter)]
    #[allow(clippy::missing_const_for_fn)] // wasm-bindgen requires a non-const export.
    pub fn value(&self) -> Option<bool> {
        self.value
    }

    #[wasm_bindgen(getter)]
    #[allow(clippy::missing_const_for_fn)]
    pub fn status(&self) -> GeometryQueryStatusJs {
        self.status
    }

    /// Absent for an uncertain or unknown query.
    #[wasm_bindgen(getter)]
    #[allow(clippy::missing_const_for_fn)]
    pub fn certainty(&self) -> Option<GeometryCertaintyJs> {
        self.certainty
    }

    /// Diagnostic reason for an unresolved query; absent for decided values.
    #[wasm_bindgen(getter)]
    pub fn reason(&self) -> Option<String> {
        self.reason.clone()
    }

    /// True only for a completed query that consumed the approximate terminal.
    /// A false value alone does not imply that the query was decided.
    #[wasm_bindgen(getter, js_name = approximate512Consumed)]
    pub fn approximate_512_consumed(&self) -> bool {
        self.certainty == Some(GeometryCertaintyJs::Approximate512Consumed)
    }
}

/// Bounds and aggregate predicate certainty.
#[wasm_bindgen]
pub struct GeometryBoundsResultJs {
    bounds: Aabb,
    certainty: GeometryCertainty,
}

impl From<GeometryOutcome<Aabb>> for GeometryBoundsResultJs {
    fn from(outcome: GeometryOutcome<Aabb>) -> Self {
        Self {
            bounds: outcome.value,
            certainty: outcome.certainty,
        }
    }
}

#[wasm_bindgen]
impl GeometryBoundsResultJs {
    #[wasm_bindgen(getter)]
    pub fn certainty(&self) -> GeometryCertaintyJs {
        self.certainty.into()
    }

    #[wasm_bindgen(getter)]
    pub fn min(&self) -> Point3Js {
        self.bounds.mins.clone().into()
    }

    #[wasm_bindgen(getter)]
    pub fn max(&self) -> Point3Js {
        self.bounds.maxs.clone().into()
    }

    #[wasm_bindgen(getter, js_name = approximate512Consumed)]
    pub fn approximate_512_consumed(&self) -> bool {
        self.certainty == GeometryCertainty::Approximate512Consumed
    }
}

/// Plane construction and aggregate predicate certainty.
#[wasm_bindgen]
pub struct GeometryPlaneResultJs {
    plane: hypermesh::Plane,
    certainty: GeometryCertainty,
}

impl From<GeometryOutcome<hypermesh::Plane>> for GeometryPlaneResultJs {
    fn from(outcome: GeometryOutcome<hypermesh::Plane>) -> Self {
        Self {
            plane: outcome.value,
            certainty: outcome.certainty,
        }
    }
}

#[wasm_bindgen]
impl GeometryPlaneResultJs {
    #[wasm_bindgen(getter)]
    pub fn certainty(&self) -> GeometryCertaintyJs {
        self.certainty.into()
    }

    #[wasm_bindgen(getter, js_name = approximate512Consumed)]
    pub fn approximate_512_consumed(&self) -> bool {
        self.certainty == GeometryCertainty::Approximate512Consumed
    }

    #[wasm_bindgen(js_name = intoPlane)]
    pub fn into_plane(self) -> super::plane_js::PlaneJs {
        self.plane.into()
    }
}

/// Reflection matrix and aggregate predicate certainty.
#[wasm_bindgen]
pub struct GeometryMatrixResultJs {
    matrix: hyperlattice::Matrix4,
    certainty: GeometryCertainty,
}

impl From<GeometryOutcome<hyperlattice::Matrix4>> for GeometryMatrixResultJs {
    fn from(outcome: GeometryOutcome<hyperlattice::Matrix4>) -> Self {
        Self {
            matrix: outcome.value,
            certainty: outcome.certainty,
        }
    }
}

#[wasm_bindgen]
impl GeometryMatrixResultJs {
    #[wasm_bindgen(getter)]
    pub fn certainty(&self) -> GeometryCertaintyJs {
        self.certainty.into()
    }

    #[wasm_bindgen(getter, js_name = approximate512Consumed)]
    pub fn approximate_512_consumed(&self) -> bool {
        self.certainty == GeometryCertainty::Approximate512Consumed
    }

    #[wasm_bindgen(js_name = intoMatrix)]
    pub fn into_matrix(self) -> super::matrix_js::Matrix4Js {
        self.matrix.into()
    }
}
