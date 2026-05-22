//! JavaScript wrapper for 3D vectors.

#[cfg(test)]
use crate::float_types::tolerance;
use crate::float_types::{
    Real, hangle_between_vectors, hangle_sin_cos, hreal_f64s_exactly_equal, hreal_mul,
    hreal_sub, hunit_quaternion, hunit_vector3, hvector3_cross, hvector3_dot,
    hvector3_from_vector3, hvector3_magnitude, hvector3_weighted_sum,
    hvectors_orthogonal_exact,
};
use nalgebra::Vector3;
use wasm_bindgen::prelude::*;

fn finite_vector3(vector: &Vector3<Real>) -> Option<Vector3<Real>> {
    let vector = hvector3_from_vector3(vector)?.to_f64_array_lossy()?;
    Some(Vector3::new(vector[0], vector[1], vector[2]))
}

fn rotation_between_quaternion_components(
    from: &Vector3<Real>,
    to: &Vector3<Real>,
) -> Option<[Real; 4]> {
    let a = hunit_vector3(from)?;
    let b = hunit_vector3(to)?;
    let dot = hvector3_dot(&a, &b)?;

    if hreal_f64s_exactly_equal(dot, -1.0) {
        let seed = if a.x.abs() < 0.9 {
            Vector3::x()
        } else {
            Vector3::y()
        };
        let axis = hunit_vector3(&hvector3_cross(&a, &seed)?)?;
        return Some([0.0, axis.x, axis.y, axis.z]);
    }

    let cross = hvector3_cross(&a, &b)?;
    let q = hunit_quaternion(1.0 + dot, cross.x, cross.y, cross.z)?;
    let vector = q.vector();
    Some([q.scalar(), vector.x, vector.y, vector.z])
}

#[wasm_bindgen]
pub struct Vector3Js {
    pub(crate) inner: Vector3<Real>,
}

#[wasm_bindgen]
impl Vector3Js {
    #[wasm_bindgen(constructor)]
    pub fn new(x: f64, y: f64, z: f64) -> Vector3Js {
        // JS coordinates are primitive boundary data. Promotion through
        // hyperlattice rejects NaN/Inf before vectors can enter CAD state; see
        // Yap, "Towards Exact Geometric Computation," Computational Geometry
        // 7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
        let vector = Vector3::new(x as Real, y as Real, z as Real);
        Vector3Js {
            inner: hvector3_from_vector3(&vector)
                .and_then(|vector| vector.to_f64_array_lossy())
                .map(|[x, y, z]| Vector3::new(x, y, z))
                .unwrap_or_else(Vector3::zeros),
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
        hvector3_magnitude(&self.inner).unwrap_or(0.0)
    }

    pub fn normalize(&self) -> Vector3Js {
        Vector3Js {
            inner: hunit_vector3(&self.inner).unwrap_or_else(Vector3::zeros),
        }
    }

    #[wasm_bindgen(js_name = isOrthogonal)]
    pub fn is_orthogonal(&self, other: &Vector3Js) -> bool {
        hvectors_orthogonal_exact(&self.inner, &other.inner)
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
        hvector3_dot(&self.inner, &other.inner).unwrap_or(0.0)
    }

    pub fn equals(&self, other: &Vector3Js) -> bool {
        self.inner == other.inner
    }

    pub fn angle(&self, other: &Vector3Js) -> f64 {
        hangle_between_vectors(&self.inner, &other.inner).unwrap_or(0.0)
    }

    pub fn scale(&self, factor: f64) -> Vector3Js {
        Vector3Js {
            inner: hvector3_weighted_sum(&[self.inner], &[factor as Real])
                .unwrap_or(self.inner),
        }
    }

    pub fn cross(&self, other: &Vector3Js) -> Vector3Js {
        Vector3Js {
            inner: hvector3_cross(&self.inner, &other.inner).unwrap_or_else(Vector3::zeros),
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
        let Some(axis) = hunit_vector3(&axis.inner) else {
            return Vector3Js { inner: self.inner };
        };
        let Some((sin, cos)) = hangle_sin_cos(angle as Real) else {
            return Vector3Js { inner: self.inner };
        };

        let Some(cross) = hvector3_cross(&axis, &self.inner) else {
            return Vector3Js { inner: self.inner };
        };
        let Some(dot) = hvector3_dot(&axis, &self.inner) else {
            return Vector3Js { inner: self.inner };
        };
        let Some(one_minus_cos) = hreal_sub(1.0, cos) else {
            return Vector3Js { inner: self.inner };
        };
        let Some(axis_weight) = hreal_mul(dot, one_minus_cos) else {
            return Vector3Js { inner: self.inner };
        };
        Vector3Js {
            inner: hvector3_weighted_sum(&[self.inner, cross, axis], &[cos, sin, axis_weight])
                .unwrap_or(self.inner),
        }
    }

    #[wasm_bindgen(js_name = rotateQuaternion)]
    pub fn rotate_quaternion(&self, w: f64, x: f64, y: f64, z: f64) -> Vector3Js {
        let Some(q) = hunit_quaternion(w as Real, x as Real, y as Real, z as Real) else {
            return Vector3Js { inner: self.inner };
        };

        Vector3Js {
            inner: q * self.inner,
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
        js_sys::Reflect::set(&obj, &JsValue::from_str("w"), &JsValue::from_f64(w as f64))?;
        js_sys::Reflect::set(&obj, &JsValue::from_str("x"), &JsValue::from_f64(x as f64))?;
        js_sys::Reflect::set(&obj, &JsValue::from_str("y"), &JsValue::from_f64(y as f64))?;
        js_sys::Reflect::set(&obj, &JsValue::from_str("z"), &JsValue::from_f64(z as f64))?;
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
        Vector3Js {
            inner: finite_vector3(&v).unwrap_or_else(Vector3::zeros),
        }
    }
}

impl From<&Vector3Js> for Vector3<Real> {
    fn from(v: &Vector3Js) -> Self {
        v.inner
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
        assert_eq!(finite.inner, Vector3::new(1.0, 2.0, 3.0));

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
        let corrupted = Vector3Js {
            inner: Vector3::new(Real::NAN, Real::INFINITY, 0.0),
        };
        let finite = Vector3Js::new(1.0, 0.0, 0.0);

        assert_eq!(corrupted.cross(&finite).inner, Vector3::zeros());
        assert_eq!(
            Vector3Js::from(Vector3::new(0.0, Real::NEG_INFINITY, 1.0)).inner,
            Vector3::zeros()
        );
    }

    #[test]
    fn vector_js_rotate_uses_hyper_cross_and_rejects_hostile_axis() {
        let vector = Vector3Js::new(1.0, 0.0, 0.0);
        let z_axis = Vector3Js::new(0.0, 0.0, 1.0);
        let rotated = vector.rotate(&z_axis, std::f64::consts::FRAC_PI_2);

        assert!(rotated.inner.x.abs() < tolerance());
        assert!((rotated.inner.y - 1.0).abs() < tolerance());

        let hostile_axis = Vector3Js {
            inner: Vector3::new(Real::NAN, 0.0, 1.0),
        };
        assert_eq!(vector.rotate(&hostile_axis, 1.0).inner, vector.inner);
        assert_eq!(vector.rotate(&z_axis, Real::NAN).inner, vector.inner);
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
                &Vector3::new(Real::NAN, 0.0, 0.0),
                &Vector3::y()
            )
            .is_none()
        );
    }

    #[test]
    fn vector_js_rotation_between_preserves_nearly_antiparallel_direction() {
        let x = Vector3::x();
        let nearly_opposite = hunit_vector3(&Vector3::new(-1.0, 1.0e-5, 0.0)).unwrap();
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
