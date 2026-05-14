//! JavaScript wrapper for 3D vectors.

use crate::float_types::Real;
use nalgebra::{Quaternion, Rotation3, Unit, UnitQuaternion, Vector3};
use wasm_bindgen::prelude::*;

#[wasm_bindgen]
pub struct Vector3Js {
    pub(crate) inner: Vector3<Real>,
}

#[wasm_bindgen]
impl Vector3Js {
    #[wasm_bindgen(constructor)]
    pub fn new(x: f64, y: f64, z: f64) -> Vector3Js {
        Vector3Js {
            inner: Vector3::new(x as Real, y as Real, z as Real),
        }
    }

    #[wasm_bindgen(getter)]
    pub fn x(&self) -> f64 {
        self.inner.x as f64
    }

    #[wasm_bindgen(getter)]
    pub fn y(&self) -> f64 {
        self.inner.y as f64
    }

    #[wasm_bindgen(getter)]
    pub fn z(&self) -> f64 {
        self.inner.z as f64
    }

    pub fn length(&self) -> f64 {
        self.inner.norm() as f64
    }

    pub fn normalize(&self) -> Vector3Js {
        let norm = self.inner.norm();
        if norm <= Real::EPSILON || !norm.is_finite() {
            return Vector3Js {
                inner: Vector3::zeros(),
            };
        }

        Vector3Js {
            inner: self.inner / norm,
        }
    }

    #[wasm_bindgen(js_name = isOrthogonal)]
    pub fn is_orthogonal(&self, tolerance: f64) -> bool {
        self.inner.is_orthogonal(tolerance as Real)
    }

    pub fn abs(&self) -> Vector3Js {
        Vector3Js {
            inner: self.inner.abs(),
        }
    }

    pub fn reverse(&self) -> Vector3Js {
        Vector3Js { inner: -self.inner }
    }

    pub fn add(&self, other: &Vector3Js) -> Vector3Js {
        Vector3Js {
            inner: self.inner + other.inner,
        }
    }

    pub fn subtract(&self, other: &Vector3Js) -> Vector3Js {
        Vector3Js {
            inner: self.inner - other.inner,
        }
    }

    pub fn dot(&self, other: &Vector3Js) -> f64 {
        self.inner.dot(&other.inner) as f64
    }

    pub fn equals(&self, other: &Vector3Js) -> bool {
        self.inner == other.inner
    }

    pub fn angle(&self, other: &Vector3Js) -> f64 {
        self.inner.angle(&other.inner) as f64
    }

    pub fn scale(&self, factor: f64) -> Vector3Js {
        Vector3Js {
            inner: self.inner * factor as Real,
        }
    }

    pub fn cross(&self, other: &Vector3Js) -> Vector3Js {
        Vector3Js {
            inner: self.inner.cross(&other.inner),
        }
    }

    pub fn rotate(&self, axis: &Vector3Js, angle: f64) -> Vector3Js {
        let axis_norm = axis.inner.norm();
        if axis_norm <= Real::EPSILON || !axis_norm.is_finite() {
            return Vector3Js { inner: self.inner };
        }

        let axis = Unit::new_unchecked(axis.inner / axis_norm);
        let rotation = Rotation3::from_axis_angle(&axis, angle as Real);
        Vector3Js {
            inner: rotation * self.inner,
        }
    }

    #[wasm_bindgen(js_name = rotateQuaternion)]
    pub fn rotate_quaternion(&self, w: f64, x: f64, y: f64, z: f64) -> Vector3Js {
        let q = Quaternion::new(w as Real, x as Real, y as Real, z as Real);
        let norm = q.norm();
        if norm <= Real::EPSILON || !norm.is_finite() {
            return Vector3Js { inner: self.inner };
        }

        let q = UnitQuaternion::new_unchecked(q / norm);
        Vector3Js {
            inner: q * self.inner,
        }
    }

    #[wasm_bindgen(js_name = rotationBetween)]
    pub fn rotation_between(&self, other: &Vector3Js) -> Result<JsValue, JsValue> {
        let a_norm = self.inner.norm();
        let b_norm = other.inner.norm();
        if a_norm <= Real::EPSILON || b_norm <= Real::EPSILON {
            return Err(JsValue::from_str("rotationBetween requires non-zero vectors"));
        }

        let a = self.inner / a_norm;
        let b = other.inner / b_norm;
        let q = UnitQuaternion::rotation_between(&a, &b).unwrap_or_else(|| {
            let seed = if a.x.abs() < 0.9 {
                Vector3::x()
            } else {
                Vector3::y()
            };
            let axis = Unit::new_normalize(a.cross(&seed));
            UnitQuaternion::from_axis_angle(&axis, std::f64::consts::PI as Real)
        });

        let vector = q.vector();
        let obj = js_sys::Object::new();
        js_sys::Reflect::set(
            &obj,
            &JsValue::from_str("w"),
            &JsValue::from_f64(q.scalar() as f64),
        )?;
        js_sys::Reflect::set(
            &obj,
            &JsValue::from_str("x"),
            &JsValue::from_f64(vector.x as f64),
        )?;
        js_sys::Reflect::set(
            &obj,
            &JsValue::from_str("y"),
            &JsValue::from_f64(vector.y as f64),
        )?;
        js_sys::Reflect::set(
            &obj,
            &JsValue::from_str("z"),
            &JsValue::from_f64(vector.z as f64),
        )?;
        Ok(obj.into())
    }

    #[wasm_bindgen(js_name = toString)]
    pub fn to_string_js(&self) -> String {
        format!(
            "<Vector3({}, {}, {})>",
            self.inner.x, self.inner.y, self.inner.z
        )
    }
}

// Rust-only conversions
impl From<Vector3<Real>> for Vector3Js {
    fn from(v: Vector3<Real>) -> Self {
        Vector3Js { inner: v }
    }
}

impl From<&Vector3Js> for Vector3<Real> {
    fn from(v: &Vector3Js) -> Self {
        v.inner
    }
}
