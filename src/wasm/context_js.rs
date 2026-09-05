//! JavaScript query results paired with predicate certainty.

use super::point_js::Point3Js;
use crate::{GeometryCertainty, GeometryOutcome};
use hyperlattice::Aabb;
use wasm_bindgen::prelude::*;

/// Point classification. An absent value represents a curve boundary.
#[wasm_bindgen]
pub struct GeometryBoolResultJs {
    value: Option<bool>,
    certainty: GeometryCertainty,
}

impl From<GeometryOutcome<Option<bool>>> for GeometryBoolResultJs {
    fn from(outcome: GeometryOutcome<Option<bool>>) -> Self {
        Self {
            value: outcome.value,
            certainty: outcome.certainty,
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

    #[wasm_bindgen(getter, js_name = approximate512Consumed)]
    pub fn approximate_512_consumed(&self) -> bool {
        self.certainty == GeometryCertainty::Approximate512Consumed
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
    #[wasm_bindgen(getter, js_name = approximate512Consumed)]
    pub fn approximate_512_consumed(&self) -> bool {
        self.certainty == GeometryCertainty::Approximate512Consumed
    }

    #[wasm_bindgen(js_name = intoMatrix)]
    pub fn into_matrix(self) -> super::matrix_js::Matrix4Js {
        self.matrix.into()
    }
}
