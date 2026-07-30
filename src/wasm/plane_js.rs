//! JavaScript wrapper for native Hypermesh planes.

use crate::wasm::{
    point_js::Point3Js, point3_from_js, real_from_js_named, real_to_js, vector_js::Vector3Js,
};
use hyperlattice::Vector3;
use hypermesh::Plane;
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct PlaneJs {
    pub(crate) inner: Plane,
}

#[wasm_bindgen]
impl PlaneJs {
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
        let a = point3_from_js(ax, ay, az)?;
        let b = point3_from_js(bx, by, bz)?;
        let c = point3_from_js(cx, cy, cz)?;
        if !Plane::points_are_nondegenerate(&crate::MESH_CONTEXT, &a, &b, &c)
            .map_err(|error| JsValue::from_str(&error.to_string()))?
            .into_value()
        {
            return Err(JsValue::from_str(
                "plane points must be certifiably non-collinear",
            ));
        }
        Ok(Self {
            inner: Plane::from_points(&a, &b, &c),
        })
    }

    #[wasm_bindgen(js_name = FromPoints)]
    pub fn from_points(a: &Point3Js, b: &Point3Js, c: &Point3Js) -> Result<Self, JsValue> {
        if !Plane::points_are_nondegenerate(&crate::MESH_CONTEXT, &a.inner, &b.inner, &c.inner)
            .map_err(|error| JsValue::from_str(&error.to_string()))?
            .into_value()
        {
            return Err(JsValue::from_str(
                "plane points must be certifiably non-collinear",
            ));
        }
        Ok(Self {
            inner: Plane::from_points(&a.inner, &b.inner, &c.inner),
        })
    }

    #[wasm_bindgen(js_name = FromNormalComponents)]
    pub fn from_normal_components(
        nx: f64,
        ny: f64,
        nz: f64,
        offset: f64,
    ) -> Result<Self, JsValue> {
        let normal = [
            real_from_js_named(nx, "nx")?,
            real_from_js_named(ny, "ny")?,
            real_from_js_named(nz, "nz")?,
        ];
        validate_normal(&normal)?;
        Ok(Self {
            inner: Plane::from_coefficients(
                normal[0].clone(),
                normal[1].clone(),
                normal[2].clone(),
                real_from_js_named(offset, "offset")?,
            ),
        })
    }

    #[wasm_bindgen(js_name = FromNormal)]
    pub fn from_normal(normal: &Vector3Js, offset: f64) -> Result<Self, JsValue> {
        let [x, y, z] = normal.inner.0.clone();
        validate_normal(&[x.clone(), y.clone(), z.clone()])?;
        Ok(Self {
            inner: Plane::from_coefficients(x, y, z, real_from_js_named(offset, "offset")?),
        })
    }

    pub fn normal(&self) -> Vector3Js {
        Vector3Js::from(Vector3::from_xyz(
            self.inner.normal.x.clone(),
            self.inner.normal.y.clone(),
            self.inner.normal.z.clone(),
        ))
    }

    pub fn offset(&self) -> f64 {
        real_to_js(&self.inner.offset)
    }

    pub fn flip(&mut self) {
        self.inner = self.inner.inverted();
    }

    #[wasm_bindgen(js_name = reflectionMatrix)]
    pub fn reflection_matrix(&self) -> Result<crate::wasm::matrix_js::Matrix4Js, JsValue> {
        self.inner
            .reflection_matrix(&crate::MESH_CONTEXT)
            .map(|outcome| outcome.into_value().into())
            .map_err(|error| JsValue::from_str(&error.to_string()))
    }
}

fn validate_normal(normal: &[hyperlattice::Real; 3]) -> Result<(), JsValue> {
    let squared = normal.iter().fold(hyperlattice::Real::zero(), |sum, value| {
        sum + value.clone() * value.clone()
    });
    match hyperlimit::classify_real_sign(&squared, crate::PREDICATE_POLICY).value() {
        Some(hyperlimit::Sign::Positive) => Ok(()),
        Some(hyperlimit::Sign::Negative | hyperlimit::Sign::Zero) | None => {
            Err(JsValue::from_str("plane normal must be certifiably non-zero"))
        },
    }
}
