//! JavaScript wrapper for 3D vectors.
//! Arithmetic retains native scalars; numeric getters/products/angles are lossy
//! views and can return NaN when no finite view is available. Predicates use the
//! crate's strict policy. This wrapper carries no construction-certainty history,
//! and several checked arithmetic failures are collapsed to generic JS strings.

#[cfg(test)]
use crate::wasm::tolerance;
use crate::wasm::{real_from_js, real_from_js_named, real_to_js, vector3_from_js};
use hyperlattice::{Real, Vector3};
use wasm_bindgen::prelude::*;

/// Promotes a known finite implementation literal exactly; panics if that precondition is violated.
fn real(value: f64) -> Real {
    real_from_js(value).expect("finite wasm scalar")
}

/// Normalizes scalar-first quaternion components by sqrt(w²+x²+y²+z²).
/// Returns `None` for any square-root/division failure, erasing its native reason.
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

/// Constructs a scalar-first unit quaternion rotating normalized `from` toward `to`.
/// Uses `(1 + dot, cross)` except at an exactly decided antipode, where a
/// perpendicular axis is chosen using the exact binary64 `0.9` seed threshold.
/// Checked normalization, strict sign/order uncertainty and arithmetic failure
/// all become `None`; near-antipodal directions are not merged by tolerance.
fn rotation_between_quaternion_components(from: &Vector3, to: &Vector3) -> Option<[Real; 4]> {
    let a = from.normalize_checked().ok()?;
    let b = to.normalize_checked().ok()?;
    let dot = a.dot(&b);

    match hyperlimit::classify_real_sign(&(dot.clone() + Real::one()), crate::PREDICATE_POLICY)
        .value()?
    {
        hyperlimit::Sign::Zero => {
            let seed = match hyperlimit::compare_reals(
                &a.0[0].abs(),
                &real(0.9),
                crate::PREDICATE_POLICY,
            )
            .value()?
            {
                std::cmp::Ordering::Less => Vector3::x(),
                std::cmp::Ordering::Equal | std::cmp::Ordering::Greater => Vector3::y(),
            };
            let axis = a.cross(&seed).normalize_checked().ok()?;
            return Some([
                Real::zero(),
                axis.0[0].clone(),
                axis.0[1].clone(),
                axis.0[2].clone(),
            ]);
        },
        hyperlimit::Sign::Positive => {},
        hyperlimit::Sign::Negative => return None,
    }

    let cross = a.cross(&b);
    quaternion_unit(
        Real::one() + dot,
        cross.0[0].clone(),
        cross.0[1].clone(),
        cross.0[2].clone(),
    )
}

/// Owned exact native vector with explicit finite numeric views and strict predicates.
#[wasm_bindgen]
pub struct Vector3Js {
    /// Authoritative components; arithmetic and Rust conversions avoid display projection.
    pub(crate) inner: Vector3,
}

#[wasm_bindgen]
impl Vector3Js {
    /// Imports finite X/Y/Z numbers exactly as binary values; invalid components return axis-named strings.
    #[wasm_bindgen(constructor)]
    pub fn new(x: f64, y: f64, z: f64) -> Result<Vector3Js, JsValue> {
        // JS coordinates are primitive boundary data. Promotion through
        // hyperlattice rejects NaN/Inf before vectors can enter CAD state; see
        // Yap, "Towards Exact Geometric Computation," Computational Geometry
        // 7(1-2), 1997 (<https://doi.org/10.1016/0925-7721(95)00040-2>).
        Ok(Vector3Js {
            inner: vector3_from_js(x, y, z)?,
        })
    }

    /// Returns a lossy X view, or NaN when no finite view is available.
    #[wasm_bindgen(getter)]
    pub fn x(&self) -> f64 {
        real_to_js(&self.inner.0[0])
    }

    /// Returns a lossy Y view, or NaN when no finite view is available.
    #[wasm_bindgen(getter)]
    pub fn y(&self) -> f64 {
        real_to_js(&self.inner.0[1])
    }

    /// Returns a lossy Z view, or NaN when no finite view is available.
    #[wasm_bindgen(getter)]
    pub fn z(&self) -> f64 {
        real_to_js(&self.inner.0[2])
    }

    /// Projects the native Euclidean norm to binary64, with NaN for unavailable finite output.
    /// Inherits the native norm's invariant panic if its square root unexpectedly fails.
    pub fn length(&self) -> f64 {
        real_to_js(&self.inner.norm())
    }

    /// Returns a checked unit vector; zero, unknown-zero and arithmetic failures share one JS string.
    pub fn normalize(&self) -> Result<Vector3Js, JsValue> {
        Ok(Vector3Js {
            inner: self
                .inner
                .normalize_checked()
                .map_err(|_| JsValue::from_str("vector must be non-zero"))?,
        })
    }

    /// Strictly decides whether the dot product is zero; uncertainty returns a JS string.
    /// Includes zero vectors, which have zero dot product with every vector.
    #[wasm_bindgen(js_name = isOrthogonal)]
    pub fn is_orthogonal(&self, other: &Vector3Js) -> Result<bool, JsValue> {
        hyperlimit::classify_real_sign(&self.inner.dot(&other.inner), crate::PREDICATE_POLICY)
            .value()
            .map(|sign| sign == hyperlimit::Sign::Zero)
            .ok_or_else(|| JsValue::from_str("orthogonality is indeterminate"))
    }

    /// Returns componentwise native absolute values, without changing this vector.
    pub fn abs(&self) -> Vector3Js {
        Vector3Js {
            inner: Vector3::from_xyz(
                self.inner.0[0].abs(),
                self.inner.0[1].abs(),
                self.inner.0[2].abs(),
            ),
        }
    }

    /// Returns the componentwise negation, reversing direction without normalization.
    pub fn reverse(&self) -> Vector3Js {
        Vector3Js {
            inner: -self.inner.clone(),
        }
    }

    /// Returns the native componentwise sum without finite projection.
    pub fn add(&self, other: &Vector3Js) -> Vector3Js {
        Vector3Js {
            inner: self.inner.clone() + other.inner.clone(),
        }
    }

    /// Returns `self - other` using native component arithmetic.
    pub fn subtract(&self, other: &Vector3Js) -> Vector3Js {
        Vector3Js {
            inner: self.inner.clone() - other.inner.clone(),
        }
    }

    /// Computes the native dot product, then returns a lossy view or NaN on unavailable finite output.
    pub fn dot(&self, other: &Vector3Js) -> f64 {
        real_to_js(&self.inner.dot(&other.inner))
    }

    /// Compares scalar values componentwise under strict decisions, not structural identity.
    /// Continues after a decided unequal component; any later undecided comparison
    /// still returns a JS string rather than the earlier false result.
    pub fn equals(&self, other: &Vector3Js) -> Result<bool, JsValue> {
        self.inner
            .0
            .iter()
            .zip(&other.inner.0)
            .try_fold(true, |equal, (left, right)| {
                hyperlimit::compare_reals(left, right, crate::PREDICATE_POLICY)
                    .value()
                    .map(|ordering| equal && ordering == std::cmp::Ordering::Equal)
            })
            .ok_or_else(|| JsValue::from_str("vector equality is indeterminate"))
    }

    /// Returns a lossy radian angle after checked normalization and native acos.
    /// All native failures share a nonzero-vector string; a successful native angle
    /// with unavailable finite projection becomes `Ok(NaN)`.
    pub fn angle(&self, other: &Vector3Js) -> Result<f64, JsValue> {
        self.inner
            .angle_to(&other.inner)
            .map(|angle| real_to_js(&angle))
            .map_err(|_| JsValue::from_str("angle requires non-zero vectors"))
    }

    /// Multiplies by an exactly promoted finite factor; invalid input or blend failure returns a string.
    pub fn scale(&self, factor: f64) -> Result<Vector3Js, JsValue> {
        let factor = real_from_js_named(factor, "factor")?;
        Ok(Vector3Js {
            inner: Vector3::weighted_sum(std::slice::from_ref(&self.inner), &[factor])
                .ok_or_else(|| JsValue::from_str("vector scaling failed"))?,
        })
    }

    /// Returns the right-handed native cross product without numeric projection.
    pub fn cross(&self, other: &Vector3Js) -> Vector3Js {
        Vector3Js {
            inner: self.inner.cross(&other.inner),
        }
    }

    /// Rotates by a finite radian angle using native Rodrigues arithmetic and a checked unit axis.
    /// Axis normalization precedes angle validation; native failure details are
    /// collapsed to strings. The returned vector retains native trigonometric values.
    pub fn rotate(&self, axis: &Vector3Js, angle: f64) -> Result<Vector3Js, JsValue> {
        // Rodrigues' rotation formula is still the wasm/API boundary shape,
        // but all scalar trig and vector blends are routed through
        // hyperreal/hyperlattice helpers while retaining native components.
        // See Yap, "Towards Exact Geometric Computation," Computational
        // Geometry 7(1-2), 1997
        // (<https://doi.org/10.1016/0925-7721(95)00040-2>) and Rodrigues,
        // "Des lois géométriques qui régissent les déplacements d'un système
        // solide dans l'espace," Journal de Mathématiques Pures et Appliquées
        // 5, 1840.
        let axis = axis
            .inner
            .normalize_checked()
            .map_err(|_| JsValue::from_str("axis must be non-zero"))?;
        let angle = real_from_js_named(angle, "angle")?;
        let sin = angle.clone().sin();
        let cos = angle.cos();

        let cross = axis.cross(&self.inner);
        let dot = axis.dot(&self.inner);
        let axis_weight = dot * (Real::one() - cos.clone());
        Ok(Vector3Js {
            inner: Vector3::weighted_sum(
                &[self.inner.clone(), cross, axis],
                &[cos, sin, axis_weight],
            )
            .ok_or_else(|| JsValue::from_str("vector rotation failed"))?,
        })
    }

    /// Normalizes finite scalar-first quaternion `(w,x,y,z)` and applies its native rotation.
    /// Zero or failed quaternion normalization returns a generic string; output
    /// components remain native and no certainty/history record is attached.
    #[wasm_bindgen(js_name = rotateQuaternion)]
    pub fn rotate_quaternion(
        &self,
        w: f64,
        x: f64,
        y: f64,
        z: f64,
    ) -> Result<Vector3Js, JsValue> {
        let Some([w, x, y, z]) = quaternion_unit(
            real_from_js_named(w, "w")?,
            real_from_js_named(x, "x")?,
            real_from_js_named(y, "y")?,
            real_from_js_named(z, "z")?,
        ) else {
            return Err(JsValue::from_str("quaternion must be non-zero"));
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

        Ok(Vector3Js {
            inner: Vector3::weighted_sum(&[u, v, cross], &weights)
                .ok_or_else(|| JsValue::from_str("quaternion rotation failed"))?,
        })
    }

    /// Returns a JS object `{w,x,y,z}` of lossy views of the selected unit quaternion.
    /// Native construction failures share one string; property-setting exceptions
    /// propagate. Unavailable finite component projections become NaN, with no
    /// renormalization or error guarantee for the exported quaternion.
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

    /// Formats native scalar displays for inspection, not exact replay.
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
    /// Moves a native vector into the wrapper without finite conversion or validation.
    fn from(v: Vector3) -> Self {
        Vector3Js { inner: v }
    }
}

impl From<&Vector3Js> for Vector3 {
    /// Clones native components without using the lossy coordinate getters.
    fn from(v: &Vector3Js) -> Self {
        v.inner.clone()
    }
}

/// Host-side native-arithmetic and finite-projection witnesses; not generated JS execution.
#[cfg(test)]
mod tests {
    use super::*;

    /// Requires a finite numeric fixture to construct successfully.
    fn js_vector(x: f64, y: f64, z: f64) -> Vector3Js {
        Vector3Js::new(x, y, z).expect("finite test vector")
    }

    /// Accepts only a strictly decided less-than relation; uncertainty fails the test condition.
    fn less_than(left: &Real, right: &Real) -> bool {
        hyperlimit::compare_reals(left, right, crate::PREDICATE_POLICY).value()
            == Some(std::cmp::Ordering::Less)
    }

    /// Checks invalid-number promotion and a finite constructor/getter path without JS exception execution.
    #[test]
    fn vector_js_rejects_nonfinite_boundary_inputs() {
        assert!(real_from_js(f64::NAN).is_none());
        assert!(real_from_js(f64::INFINITY).is_none());

        let finite = js_vector(1.0, 2.0, 3.0);
        assert_eq!(finite.x(), 1.0);
        assert_eq!(finite.y(), 2.0);
        assert_eq!(finite.z(), 3.0);
    }

    /// Distinguishes perpendicular coordinate axes from a nonperpendicular diagonal.
    #[test]
    fn vector_js_orthogonality_uses_other_vector() {
        let x = js_vector(1.0, 0.0, 0.0);
        let y = js_vector(0.0, 1.0, 0.0);
        let diagonal = js_vector(1.0, 1.0, 0.0);

        assert!(x.is_orthogonal(&y).unwrap());
        assert!(!x.is_orthogonal(&diagonal).unwrap());
    }

    /// Checks only NaN and signed-infinity rejection by scalar promotion, despite the historical name.
    #[test]
    fn vector_js_cross_and_conversion_reject_nonfinite_vectors() {
        assert!(real_from_js(f64::NAN).is_none());
        assert!(real_from_js(f64::INFINITY).is_none());
        assert!(real_from_js(f64::NEG_INFINITY).is_none());
    }

    /// Bounds two coordinates after a finite pi/2-literal rotation and separately checks NaN promotion.
    /// Does not invoke rotation with an invalid axis or assert an exact quarter-turn image.
    #[test]
    fn vector_js_rotate_uses_hyper_cross_and_rejects_hostile_axis() {
        let vector = js_vector(1.0, 0.0, 0.0);
        let z_axis = js_vector(0.0, 0.0, 1.0);
        let rotated = vector
            .rotate(&z_axis, std::f64::consts::FRAC_PI_2)
            .expect("valid rotation");

        assert!(less_than(&rotated.inner.0[0].abs(), &tolerance()));
        assert!(less_than(
            &(rotated.inner.0[1].clone() - 1.0).abs(),
            &tolerance()
        ));

        assert!(real_from_js(f64::NAN).is_none());
    }

    /// Checks quarter-turn quaternion component enclosures, an antipodal unit axis and zero-input failure.
    #[test]
    fn vector_js_rotation_between_components_use_hyper_quaternion_path() {
        let x = Vector3::x();
        let y = Vector3::y();
        let [w, qx, qy, qz] = rotation_between_quaternion_components(&x, &y).unwrap();

        assert!(less_than(
            &(w - std::f64::consts::FRAC_1_SQRT_2).abs(),
            &tolerance()
        ));
        assert!(less_than(&qx.abs(), &tolerance()));
        assert!(less_than(&qy.abs(), &tolerance()));
        assert!(less_than(
            &(qz - std::f64::consts::FRAC_1_SQRT_2).abs(),
            &tolerance()
        ));

        let opposite = rotation_between_quaternion_components(&x, &-x.clone()).unwrap();
        assert!(less_than(&opposite[0].abs(), &tolerance()));
        assert!(less_than(
            &(opposite[1].clone() * opposite[1].clone()
                + opposite[2].clone() * opposite[2].clone()
                + opposite[3].clone() * opposite[3].clone()
                - 1.0)
                .abs(),
            &tolerance()
        ));

        assert!(
            rotation_between_quaternion_components(&Vector3::zeros(), &Vector3::y()).is_none()
        );
    }

    /// Strictly requires positive scalar and Z quaternion components near the antipode.
    /// These sign witnesses distinguish this turn from an exactly pi rotation.
    #[test]
    fn vector_js_rotation_between_preserves_nearly_antiparallel_direction() {
        let x = Vector3::x();
        let nearly_opposite = js_vector(-1.0, 1.0e-5, 0.0)
            .inner
            .normalize_checked()
            .unwrap();
        let [w, _qx, _qy, qz] =
            rotation_between_quaternion_components(&x, &nearly_opposite).unwrap();

        assert!(
            hyperlimit::compare_reals(&w, &Real::zero(), crate::PREDICATE_POLICY).value()
                == Some(std::cmp::Ordering::Greater),
            "near-antiparallel rotations should not collapse to exact pi"
        );
        assert!(
            hyperlimit::compare_reals(&qz, &Real::zero(), crate::PREDICATE_POLICY).value()
                == Some(std::cmp::Ordering::Greater),
            "quaternion axis should retain the exact positive turn direction"
        );
    }
}
