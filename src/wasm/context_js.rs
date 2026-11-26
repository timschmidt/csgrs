//! JavaScript query results paired with predicate certainty.

use super::point_js::Point3Js;
use crate::{GeometryCertainty, GeometryOutcome};
use hyperlattice::Aabb;
use wasm_bindgen::prelude::*;

/// Weakest predicate certainty consumed by a completed geometry operation.
///
/// This describes consumed predicate decisions, not a standalone topology proof
/// or the fidelity of a float projection. Separate construction evidence may
/// retain qualifications that an operation-local receipt cannot establish.
#[wasm_bindgen]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum GeometryCertaintyJs {
    /// All consumed decisions have exact certificates.
    Certified,
    /// At least one decision consumed the authorized 512-bit terminal.
    Approximate512Consumed,
}

impl From<GeometryCertainty> for GeometryCertaintyJs {
    /// Maps both modeling certainty variants without changing their strength.
    fn from(certainty: GeometryCertainty) -> Self {
        match certainty {
            GeometryCertainty::Certified => Self::Certified,
            GeometryCertainty::Approximate512Consumed => Self::Approximate512Consumed,
        }
    }
}

impl From<hypercurve::CurveCertainty> for GeometryCertaintyJs {
    /// Maps both curve certainty variants without upgrading a qualified result.
    fn from(certainty: hypercurve::CurveCertainty) -> Self {
        match certainty {
            hypercurve::CurveCertainty::Certified => Self::Certified,
            hypercurve::CurveCertainty::Approximate512Consumed => Self::Approximate512Consumed,
        }
    }
}

/// Native construction qualifications, independent of operation-local certainty.
/// A missing predicate certainty means source history is unavailable. Projection
/// and sampling flags are separate and supply no geometric error bound.
#[wasm_bindgen]
pub struct ConstructionEvidenceJs {
    /// Native dependency summary, not a replayable proof or boundary-to-source mapping.
    pub(crate) inner: hypermesh::ConstructionEvidence,
}

#[wasm_bindgen]
#[allow(clippy::missing_const_for_fn)]
impl ConstructionEvidenceJs {
    /// Reports unavailable preceding history; false alone does not certify source fidelity.
    #[wasm_bindgen(getter, js_name = sourceHistoryMissing)]
    pub fn source_history_missing(&self) -> bool {
        self.inner.source_history_missing()
    }

    /// Reports recorded terminal consumption; false does not certify missing history.
    #[wasm_bindgen(getter, js_name = approximate512Consumed)]
    pub fn approximate_512_consumed(&self) -> bool {
        self.inner.approximate_512_consumed()
    }

    /// Reports coordinate projection in the dependency chain, without supplying its error bound.
    #[wasm_bindgen(getter, js_name = coordinatesProjected)]
    pub fn coordinates_projected(&self) -> bool {
        self.inner.coordinates_projected()
    }

    /// Reports construction from samples, independently of continuous-source fidelity.
    #[wasm_bindgen(getter, js_name = sourceSampled)]
    pub fn source_sampled(&self) -> bool {
        self.inner.source_sampled()
    }

    /// Reports buffers that ceased to match their retained construction identity.
    #[wasm_bindgen(getter, js_name = buffersChanged)]
    pub fn buffers_changed(&self) -> bool {
        self.inner.buffers_changed()
    }

    /// Returns aggregate predicate certainty only when source history is available.
    /// Even `Certified` establishes neither topology nor fidelity to preceding source intent.
    #[wasm_bindgen(getter, js_name = predicateCertainty)]
    pub fn predicate_certainty(&self) -> Option<GeometryCertaintyJs> {
        self.inner
            .predicate_certainty()
            .map(|certainty| match certainty {
                hypermesh::MeshCertainty::Certified => GeometryCertaintyJs::Certified,
                hypermesh::MeshCertainty::Approximate512Consumed => {
                    GeometryCertaintyJs::Approximate512Consumed
                },
            })
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
    /// Membership when decided; absent for boundary or unresolved statuses.
    value: Option<bool>,
    /// Discriminant needed to distinguish the meanings of an absent membership value.
    status: GeometryQueryStatusJs,
    /// Consumed decision receipt for completed queries, absent for unresolved queries.
    certainty: Option<GeometryCertaintyJs>,
    /// Owned diagnostic text for unresolved queries; not a structured native blocker.
    reason: Option<String>,
}

impl From<GeometryOutcome<Option<bool>>> for GeometryBoolResultJs {
    /// Treats `Some` as decided membership and `None` as completed curve boundary, retaining certainty.
    /// An unresolved outcome must use the separate unresolved constructor instead.
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
    /// Builds a valueless, receipt-free query carrying an owned diagnostic string.
    /// Callers must supply `Uncertain` or `Unknown`; this precondition is asserted only in debug builds.
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
    /// Returns membership if present; consult status to interpret absence.
    #[wasm_bindgen(getter)]
    #[allow(clippy::missing_const_for_fn)] // wasm-bindgen requires a non-const export.
    pub fn value(&self) -> Option<bool> {
        self.value
    }

    /// Distinguishes decided membership, completed curve boundary and unresolved curve/mesh queries.
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
    /// Exact native bounds, absent for a successfully empty result.
    bounds: Option<Aabb>,
    /// Aggregate decisions consumed by the completed bounds operation.
    certainty: GeometryCertainty,
}

impl From<GeometryOutcome<Option<Aabb>>> for GeometryBoundsResultJs {
    /// Moves native bounds or successful emptiness together with the operation receipt.
    fn from(outcome: GeometryOutcome<Option<Aabb>>) -> Self {
        Self {
            bounds: outcome.value,
            certainty: outcome.certainty,
        }
    }
}

#[wasm_bindgen]
impl GeometryBoundsResultJs {
    /// True for a completed bounds query on empty geometry.
    #[wasm_bindgen(getter, js_name = isEmpty)]
    #[allow(clippy::missing_const_for_fn)] // wasm-bindgen requires a non-const export.
    pub fn is_empty(&self) -> bool {
        self.bounds.is_none()
    }

    /// Returns the retained bounds-operation receipt, including for successful empty geometry.
    #[wasm_bindgen(getter)]
    pub fn certainty(&self) -> GeometryCertaintyJs {
        self.certainty.into()
    }

    /// Clones the native minimum corner, or returns absence for empty geometry; point getters are lossy views.
    #[wasm_bindgen(getter)]
    pub fn min(&self) -> Option<Point3Js> {
        self.bounds.as_ref().map(|bounds| bounds.mins.clone().into())
    }

    /// Clones the native maximum corner, or returns absence for empty geometry; point getters are lossy views.
    #[wasm_bindgen(getter)]
    pub fn max(&self) -> Option<Point3Js> {
        self.bounds.as_ref().map(|bounds| bounds.maxs.clone().into())
    }

    /// Reports actual terminal consumption recorded by this bounds operation.
    #[wasm_bindgen(getter, js_name = approximate512Consumed)]
    pub fn approximate_512_consumed(&self) -> bool {
        self.certainty == GeometryCertainty::Approximate512Consumed
    }
}

/// Plane construction and aggregate predicate certainty.
#[wasm_bindgen]
pub struct GeometryPlaneResultJs {
    /// Native plane coefficients, independent of the separate decision receipt.
    plane: hypermesh::Plane,
    /// Local certainty consumed while constructing this plane.
    certainty: GeometryCertainty,
}

impl From<GeometryOutcome<hypermesh::Plane>> for GeometryPlaneResultJs {
    /// Moves the native plane and its local receipt into the JS-facing outcome.
    fn from(outcome: GeometryOutcome<hypermesh::Plane>) -> Self {
        Self {
            plane: outcome.value,
            certainty: outcome.certainty,
        }
    }
}

#[wasm_bindgen]
impl GeometryPlaneResultJs {
    /// Returns the plane construction's retained predicate certainty.
    #[wasm_bindgen(getter)]
    pub fn certainty(&self) -> GeometryCertaintyJs {
        self.certainty.into()
    }

    /// Reports terminal consumption during this plane construction.
    #[wasm_bindgen(getter, js_name = approximate512Consumed)]
    pub fn approximate_512_consumed(&self) -> bool {
        self.certainty == GeometryCertainty::Approximate512Consumed
    }

    /// Consumes the outcome and returns its plane, discarding the separate receipt.
    /// Inspect certainty first; `PlaneJs` stores no inherited construction summary.
    #[wasm_bindgen(js_name = intoPlane)]
    pub fn into_plane(self) -> super::plane_js::PlaneJs {
        self.plane.into()
    }
}

/// Reflection matrix and aggregate predicate certainty.
#[wasm_bindgen]
pub struct GeometryMatrixResultJs {
    /// Native reflection coefficients, without finite projection.
    matrix: hyperlattice::Matrix4,
    /// Local certainty consumed by reflection construction.
    certainty: GeometryCertainty,
}

impl From<GeometryOutcome<hyperlattice::Matrix4>> for GeometryMatrixResultJs {
    /// Moves the native matrix and local receipt into the JS-facing outcome.
    fn from(outcome: GeometryOutcome<hyperlattice::Matrix4>) -> Self {
        Self {
            matrix: outcome.value,
            certainty: outcome.certainty,
        }
    }
}

#[wasm_bindgen]
impl GeometryMatrixResultJs {
    /// Returns the reflection construction's retained predicate certainty.
    #[wasm_bindgen(getter)]
    pub fn certainty(&self) -> GeometryCertaintyJs {
        self.certainty.into()
    }

    /// Reports terminal consumption during this reflection construction.
    #[wasm_bindgen(getter, js_name = approximate512Consumed)]
    pub fn approximate_512_consumed(&self) -> bool {
        self.certainty == GeometryCertainty::Approximate512Consumed
    }

    /// Consumes the outcome and returns its matrix, discarding the separate receipt.
    /// Inspect certainty first; `Matrix4Js` does not retain construction history.
    #[wasm_bindgen(js_name = intoMatrix)]
    pub fn into_matrix(self) -> super::matrix_js::Matrix4Js {
        self.matrix.into()
    }
}
