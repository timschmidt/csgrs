//! JavaScript wrapper for 3D vectors.

#[cfg(test)]
use crate::wasm::tolerance;
use crate::wasm::{real_from_js, real_from_js_or_zero, real_to_js, vector3_from_js_or_zero};
use hyperlattice::{Real, Vector3};
use hyperreal::RealSign;
use wasm_bindgen::prelude::*;

fn finite_vector3(vector: &Vector3) -> Option<Vector3> {
    Some(vector.clone())
}

fn real(value: f64) -> Real {
    real_from_js(value).expect("finite wasm scalar")
}

fn quaternion_unit(w: Real, x: Real, y: Real, z: Real) -> Option<[Real; 4]> {
    let norm_squared = w.clone() * w.clone()
        + x.clone() * x.clone()
        + y.clone() * y.clone()
        + z.clone() * z.clone();
    let norm = norm_squared.sqrt().ok()?;
    Some([
        (w / norm.clone()).ok()?,
        (x / norm.clone()).ok()?,
        (y / norm.clone()).ok()?,
        (z / norm).ok()?,
    ])
}

fn rotation_between_quaternion_components(from: &Vector3, to: &Vector3) -> Option<[Real; 4]> {
    let a = from.normalize_checked().ok()?;
    let b = to.normalize_checked().ok()?;
    let dot = a.dot(&b);

    if matches!(
        (dot.clone() + Real::one()).refine_sign_until(128),
        Some(RealSign::Zero)
    ) {
        let seed = if a.0[0].abs() < real(0.9) {
            Vector3::x()
        } else {
            Vector3::y()
        };
        let axis = a.cross(&seed).normalize_checked().ok()?;
        return Some([
            Real::zero(),
            axis.0[0].clone(),
            axis.0[1].clone(),
            axis.0[2].clone(),
        ]);
    }

    let cross = a.cross(&b);
    quaternion_unit(
        Real::one() + dot,
        cross.0[0].clone(),
        cross.0[1].clone(),
        cross.0[2].clone(),
    )
}

#[wasm_bindgen]
pub struct Vector3Js {
    pub(crate) inner: Vector3,
}

#[wasm_bindgen]
impl Vector3Js {
    #[wasm_bindgen(constructor)]
    pub fn new(x: f64, y: f64, z: f64) -> Vector3Js {
        // JS coordinates are primitive boundary data. Promotion through
        // hyperlattice rejects NaN/Inf before vectors can enter CAD state; see
        // Yap, "Towards Exact Geometric Computation," Computational Geometry
        // 7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
        Vector3Js {
            inner: vector3_from_js_or_zero(x, y, z),
        }
    }

    #[wasm_bindgen(getter)]
    pub fn x(&self) -> f64 {
        real_to_js(&self.inner.0[0])
    }

    #[wasm_bindgen(getter)]
    pub fn y(&self) -> f64 {
        real_to_js(&self.inner.0[1])
    }

    #[wasm_bindgen(getter)]
    pub fn z(&self) -> f64 {
        real_to_js(&self.inner.0[2])
    }

    pub fn length(&self) -> f64 {
        real_to_js(&self.inner.norm())
    }

    pub fn normalize(&self) -> Vector3Js {
        Vector3Js {
            inner: self
                .inner
                .normalize_checked()
                .unwrap_or_else(|_| Vector3::zeros()),
        }
    }

    #[wasm_bindgen(js_name = isOrthogonal)]
    pub fn is_orthogonal(&self, other: &Vector3Js) -> bool {
        matches!(
            self.inner.dot(&other.inner).refine_sign_until(128),
            Some(RealSign::Zero)
        )
    }

    pub fn abs(&self) -> Vector3Js {
        Vector3Js {
            inner: Vector3::from_xyz(
                self.inner.0[0].abs(),
                self.inner.0[1].abs(),
                self.inner.0[2].abs(),
            ),
        }
    }

    pub fn reverse(&self) -> Vector3Js {
        Vector3Js {
            inner: -self.inner.clone(),
        }
    }

    pub fn add(&self, other: &Vector3Js) -> Vector3Js {
        Vector3Js {
            inner: self.inner.clone() + other.inner.clone(),
        }
    }

    pub fn subtract(&self, other: &Vector3Js) -> Vector3Js {
        Vector3Js {
            inner: self.inner.clone() - other.inner.clone(),
        }
    }

    pub fn dot(&self, other: &Vector3Js) -> f64 {
        real_to_js(&self.inner.dot(&other.inner))
    }

    pub fn equals(&self, other: &Vector3Js) -> bool {
        self.inner == other.inner
    }

    pub fn angle(&self, other: &Vector3Js) -> f64 {
        self.inner
            .angle_to(&other.inner)
            .ok()
            .map(|angle| real_to_js(&angle))
            .unwrap_or(0.0)
    }

    pub fn scale(&self, factor: f64) -> Vector3Js {
        let Some(factor) = real_from_js(factor) else {
            return Vector3Js {
                inner: self.inner.clone(),
            };
        };
        Vector3Js {
            inner: Vector3::weighted_sum(std::slice::from_ref(&self.inner), &[factor])
                .unwrap_or_else(|| self.inner.clone()),
        }
    }

    pub fn cross(&self, other: &Vector3Js) -> Vector3Js {
        Vector3Js {
            inner: self.inner.cross(&other.inner),
        }
    }

    pub fn rotate(&self, axis: &Vector3Js, angle: f64) -> Vector3Js {
        // Rodrigues' rotation formula is still the wasm/API boundary shape,
        // but all scalar trig and vector blends are routed through
        // hyperreal/hyperlattice helpers before returning finite components.
        // See Yap, "Towards Exact Geometric Computation," Computational
        // Geometry 7(1-2), 1997
        // (<https://doi.org/10.1016/0925-7721(95)00040-2>) and Rodrigues,
        // "Des lois géométriques qui régissent les déplacements d'un système
        // solide dans l'espace," Journal de Mathématiques Pures et Appliquées
        // 5, 1840.
        let Ok(axis) = axis.inner.normalize_checked() else {
            return Vector3Js {
                inner: self.inner.clone(),
            };
        };
        let Some(angle) = real_from_js(angle) else {
            return Vector3Js {
                inner: self.inner.clone(),
            };
        };
        let sin = angle.clone().sin();
        let cos = angle.cos();

        let cross = axis.cross(&self.inner);
        let dot = axis.dot(&self.inner);
        let axis_weight = dot * (Real::one() - cos.clone());
        Vector3Js {
            inner: Vector3::weighted_sum(
                &[self.inner.clone(), cross, axis],
                &[cos, sin, axis_weight],
            )
            .unwrap_or_else(|| self.inner.clone()),
        }
    }

    #[wasm_bindgen(js_name = rotateQuaternion)]
    pub fn rotate_quaternion(&self, w: f64, x: f64, y: f64, z: f64) -> Vector3Js {
        let Some([w, x, y, z]) = quaternion_unit(
            real_from_js_or_zero(w),
            real_from_js_or_zero(x),
            real_from_js_or_zero(y),
            real_from_js_or_zero(z),
        ) else {
            return Vector3Js {
                inner: self.inner.clone(),
            };
        };
        let u = Vector3::from_xyz(x, y, z);
        let v = self.inner.clone();
        let two = real(2.0);
        let u_dot_v = u.dot(&v);
        let u_dot_u = u.dot(&u);
        let cross = u.cross(&v);
        let weights = [
            two.clone() * u_dot_v,
            w.clone() * w.clone() - u_dot_u,
            two * w,
        ];

        Vector3Js {
            inner: Vector3::weighted_sum(&[u, v, cross], &weights)
                .unwrap_or_else(|| self.inner.clone()),
        }
    }

    #[wasm_bindgen(js_name = rotationBetween)]
    pub fn rotation_between(&self, other: &Vector3Js) -> Result<JsValue, JsValue> {
        let Some([w, x, y, z]) =
            rotation_between_quaternion_components(&self.inner, &other.inner)
        else {
            return Err(JsValue::from_str(
                "rotationBetween requires non-zero finite vectors",
            ));
        };

        let obj = js_sys::Object::new();
        js_sys::Reflect::set(
            &obj,
            &JsValue::from_str("w"),
            &JsValue::from_f64(real_to_js(&w)),
        )?;
        js_sys::Reflect::set(
            &obj,
            &JsValue::from_str("x"),
            &JsValue::from_f64(real_to_js(&x)),
        )?;
        js_sys::Reflect::set(
            &obj,
            &JsValue::from_str("y"),
            &JsValue::from_f64(real_to_js(&y)),
        )?;
        js_sys::Reflect::set(
            &obj,
            &JsValue::from_str("z"),
            &JsValue::from_f64(real_to_js(&z)),
        )?;
        Ok(obj.into())
    }

    #[wasm_bindgen(js_name = toString)]
    pub fn to_string_js(&self) -> String {
        format!(
            "<Vector3({}, {}, {})>",
            self.inner.0[0], self.inner.0[1], self.inner.0[2]
        )
    }
}

// Rust-only conversions
impl From<Vector3> for Vector3Js {
    fn from(v: Vector3) -> Self {
        Vector3Js {
            inner: finite_vector3(&v).unwrap_or_else(Vector3::zeros),
        }
    }
}

impl From<&Vector3Js> for Vector3 {
    fn from(v: &Vector3Js) -> Self {
        v.inner.clone()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vector_js_rejects_nonfinite_boundary_inputs() {
        let vector = Vector3Js::new(f64::NAN, 2.0, f64::INFINITY);
        assert_eq!(vector.inner, Vector3::zeros());

        let finite = Vector3Js::new(1.0, 2.0, 3.0);
        assert_eq!(finite.x(), 1.0);
        assert_eq!(finite.y(), 2.0);
        assert_eq!(finite.z(), 3.0);

        let scaled = finite.scale(f64::NAN);
        assert_eq!(scaled.inner, finite.inner);
    }

    #[test]
    fn vector_js_orthogonality_uses_other_vector() {
        let x = Vector3Js::new(1.0, 0.0, 0.0);
        let y = Vector3Js::new(0.0, 1.0, 0.0);
        let diagonal = Vector3Js::new(1.0, 1.0, 0.0);

        assert!(x.is_orthogonal(&y));
        assert!(!x.is_orthogonal(&diagonal));
    }

    #[test]
    fn vector_js_cross_and_conversion_reject_nonfinite_vectors() {
        let corrupted = Vector3Js::new(f64::NAN, f64::INFINITY, 0.0);
        let finite = Vector3Js::new(1.0, 0.0, 0.0);

        assert_eq!(corrupted.cross(&finite).inner, Vector3::zeros());
        assert_eq!(
            Vector3Js::new(0.0, f64::NEG_INFINITY, 1.0).inner,
            Vector3::zeros(),
        );
    }

    #[test]
    fn vector_js_rotate_uses_hyper_cross_and_rejects_hostile_axis() {
        let vector = Vector3Js::new(1.0, 0.0, 0.0);
        let z_axis = Vector3Js::new(0.0, 0.0, 1.0);
        let rotated = vector.rotate(&z_axis, std::f64::consts::FRAC_PI_2);

        assert!(rotated.inner.0[0].abs() < tolerance());
        assert!((rotated.inner.0[1].clone() - 1.0).abs() < tolerance());

        let hostile_axis = Vector3Js::new(f64::NAN, 0.0, 1.0);
        assert_eq!(vector.rotate(&hostile_axis, 1.0).inner, vector.inner);
        assert_eq!(vector.rotate(&z_axis, f64::NAN).inner, vector.inner);
    }

    #[test]
    fn vector_js_rotation_between_components_use_hyper_quaternion_path() {
        let x = Vector3::x();
        let y = Vector3::y();
        let [w, qx, qy, qz] = rotation_between_quaternion_components(&x, &y).unwrap();

        assert!((w - std::f64::consts::FRAC_1_SQRT_2).abs() < tolerance());
        assert!(qx.abs() < tolerance());
        assert!(qy.abs() < tolerance());
        assert!((qz - std::f64::consts::FRAC_1_SQRT_2).abs() < tolerance());

        let opposite = rotation_between_quaternion_components(&x, &-x).unwrap();
        assert!(opposite[0].abs() < tolerance());
        assert!(
            (opposite[1] * opposite[1]
                + opposite[2] * opposite[2]
                + opposite[3] * opposite[3]
                - 1.0)
                .abs()
                < tolerance()
        );

        assert!(
            rotation_between_quaternion_components(
                &Vector3Js::new(f64::NAN, 0.0, 0.0).inner,
                &Vector3::y()
            )
            .is_none()
        );
    }

    #[test]
    fn vector_js_rotation_between_preserves_nearly_antiparallel_direction() {
        let x = Vector3::x();
        let nearly_opposite = Vector3Js::new(-1.0, 1.0e-5, 0.0)
            .inner
            .normalize_checked()
            .unwrap();
        let [w, _qx, _qy, qz] =
            rotation_between_quaternion_components(&x, &nearly_opposite).unwrap();

        assert!(
            w > 0.0,
            "near-antiparallel rotations should not collapse to exact pi"
        );
        assert!(
            qz > 0.0,
            "quaternion axis should retain the exact positive turn direction"
        );
    }
}
