//! Exact camera state and explicit f64 projection.

#[cfg(feature = "exact")]
use core::cmp::Ordering;

#[cfg(feature = "exact")]
use hyperlattice::{Point3, Real, pi};
#[cfg(feature = "exact")]
use hyperlimit::{PredicateOutcome, PredicatePolicy, compare_reals};
#[cfg(feature = "exact")]
use nalgebra::{Matrix4, Perspective3, Translation3, UnitQuaternion, Vector4};

use crate::error::{Error, Result};

/// Rectangular render viewport in screen coordinates.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Viewport {
    min_x: f64,
    min_y: f64,
    width: f64,
    height: f64,
}

impl Viewport {
    /// Construct a finite viewport with positive extents and a stable aspect ratio.
    pub fn new(min_x: f64, min_y: f64, width: f64, height: f64) -> Result<Self> {
        let viewport = Self {
            min_x,
            min_y,
            width,
            height,
        };
        viewport.validate()?;
        Ok(viewport)
    }

    fn validate(self) -> Result<()> {
        for (value, name) in [
            (self.min_x, "min_x"),
            (self.min_y, "min_y"),
            (self.width, "width"),
            (self.height, "height"),
        ] {
            if !value.is_finite() {
                return Err(Error::NonFinitePrimitive { value: name });
            }
        }
        if self.width <= 0.0 {
            return Err(Error::NonPositiveViewportExtent { value: "width" });
        }
        if self.height <= 0.0 {
            return Err(Error::NonPositiveViewportExtent { value: "height" });
        }
        if !(self.min_x + self.width).is_finite() {
            return Err(Error::NonFinitePrimitive {
                value: "viewport maximum x",
            });
        }
        if !(self.min_y + self.height).is_finite() {
            return Err(Error::NonFinitePrimitive {
                value: "viewport maximum y",
            });
        }
        let aspect = self.width / self.height;
        if !aspect.is_finite() || aspect <= 0.0 {
            return Err(Error::InvalidViewportAspect);
        }
        Ok(())
    }

    /// Return the minimum screen x coordinate.
    pub const fn min_x(self) -> f64 {
        self.min_x
    }

    /// Return the minimum screen y coordinate.
    pub const fn min_y(self) -> f64 {
        self.min_y
    }

    /// Return the viewport width in pixels.
    pub const fn width(self) -> f64 {
        self.width
    }

    /// Return the viewport height in pixels.
    pub const fn height(self) -> f64 {
        self.height
    }

    /// Return the viewport aspect ratio.
    pub fn aspect(self) -> f64 {
        self.width / self.height
    }

    /// Return the viewport center.
    pub fn center(self) -> ScreenPoint {
        ScreenPoint {
            x: self.min_x + self.width * 0.5,
            y: self.min_y + self.height * 0.5,
        }
    }
}

/// Finite 2D screen point using the viewport's top-left, Y-down convention.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ScreenPoint {
    x: f64,
    y: f64,
}

impl ScreenPoint {
    /// Construct a finite screen point.
    pub fn new(x: f64, y: f64) -> Result<Self> {
        if !x.is_finite() || !y.is_finite() {
            return Err(Error::NonFinitePrimitive {
                value: "screen point",
            });
        }
        Ok(Self { x, y })
    }

    /// Return the screen x coordinate.
    pub const fn x(self) -> f64 {
        self.x
    }

    /// Return the screen y coordinate.
    pub const fn y(self) -> f64 {
        self.y
    }
}

/// Finite approximate world point produced by primitive-float camera math.
///
/// Unlike `hyperlattice::Point3`, this type carries no exact-geometry
/// claim. Converting it into exact scene geometry is an explicit application
/// decision.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ApproxPoint3 {
    coordinates: [f64; 3],
}

impl ApproxPoint3 {
    /// Construct a finite approximate point.
    pub fn new(coordinates: [f64; 3]) -> Result<Self> {
        if coordinates.iter().any(|coordinate| !coordinate.is_finite()) {
            return Err(Error::NonFinitePrimitive {
                value: "approximate world point",
            });
        }
        Ok(Self { coordinates })
    }

    /// Return the approximate coordinates.
    pub const fn to_array(self) -> [f64; 3] {
        self.coordinates
    }
}

/// `f64` render projection matrix.
#[derive(Clone, Debug, PartialEq)]
pub struct Projection64 {
    #[cfg(feature = "exact")]
    matrix: Matrix4<f64>,
    #[cfg(not(feature = "exact"))]
    matrix: [f64; 16],
}

impl Projection64 {
    /// Construct from a finite nalgebra matrix.
    #[cfg(feature = "exact")]
    pub fn new(matrix: Matrix4<f64>) -> Result<Self> {
        if matrix.iter().any(|value| !value.is_finite()) {
            return Err(Error::NonFinitePrimitive {
                value: "projection matrix",
            });
        }
        Ok(Self { matrix })
    }

    /// Construct from a finite column-major f32 matrix.
    ///
    /// This is an interoperability boundary for applications whose camera math
    /// uses another matrix package or nalgebra version. The input ordering
    /// matches OpenGL uniform matrices and `nalgebra::Matrix4::as_slice`.
    pub fn try_from_column_major_f32(values: [f32; 16]) -> Result<Self> {
        let mut widened = [0.0_f64; 16];
        for (index, value) in values.into_iter().enumerate() {
            if !value.is_finite() {
                return Err(Error::NonFinitePrimitive {
                    value: "projection matrix",
                });
            }
            widened[index] = f64::from(value);
        }
        #[cfg(feature = "exact")]
        {
            Self::new(Matrix4::from_column_slice(&widened))
        }
        #[cfg(not(feature = "exact"))]
        {
            Ok(Self { matrix: widened })
        }
    }

    /// Borrow the matrix.
    #[cfg(feature = "exact")]
    pub const fn matrix(&self) -> &Matrix4<f64> {
        &self.matrix
    }

    /// Return a finite f32 column-major slice for OpenGL uniforms.
    pub fn to_f32_array(&self) -> Result<[f32; 16]> {
        let mut out = [0.0_f32; 16];
        #[cfg(feature = "exact")]
        let values = self.matrix.as_slice();
        #[cfg(not(feature = "exact"))]
        let values = &self.matrix;
        for (index, value) in values.iter().copied().enumerate() {
            let narrowed = value as f32;
            if !narrowed.is_finite() {
                return Err(Error::F32Overflow {
                    value: "projection matrix",
                });
            }
            out[index] = narrowed;
        }
        Ok(out)
    }
}

/// Hyperreal-owned orbit camera parameters.
#[cfg(feature = "exact")]
#[derive(Clone, Debug, PartialEq)]
pub struct ExactCamera {
    /// Yaw angle in radians.
    pub yaw: Real,
    /// Pitch angle in radians.
    pub pitch: Real,
    /// Roll angle in radians.
    pub roll: Real,
    /// Camera distance from the target.
    pub zoom: Real,
    /// Orbit target.
    pub target: Point3,
    /// Vertical field of view in radians.
    pub fov_y: Real,
    /// Near clip plane.
    pub near: Real,
    /// Far clip plane.
    pub far: Real,
}

#[cfg(feature = "exact")]
impl Default for ExactCamera {
    fn default() -> Self {
        Self {
            yaw: Real::zero(),
            pitch: degrees(-55),
            roll: Real::zero(),
            zoom: Real::from(12),
            target: Point3::origin(),
            fov_y: degrees(60),
            near: Real::try_from(0.1_f64).expect("finite dyadic literal"),
            far: Real::from(10_000),
        }
    }
}

#[cfg(feature = "exact")]
impl ExactCamera {
    /// Apply an exact orbit delta in radians.
    pub fn orbit(&mut self, yaw_delta: Real, pitch_delta: Real) {
        self.yaw = self.yaw.clone() + yaw_delta;
        self.pitch = self.pitch.clone() + pitch_delta;
    }

    /// Apply a multiplicative zoom factor under an explicit predicate policy.
    pub fn zoom_by(&mut self, factor: Real, policy: PredicatePolicy) -> Result<()> {
        if decide_ordering(&factor, &Real::zero(), "camera zoom factor", policy)?
            != Ordering::Greater
        {
            return Err(Error::InvalidCameraParameter {
                value: "zoom factor",
            });
        }
        self.zoom = (self.zoom.clone() / factor)?;
        Ok(())
    }

    /// Reset to the default tilted top-down orientation.
    pub fn reset_top_down(&mut self) {
        self.yaw = Real::zero();
        self.pitch = degrees(-55);
        self.roll = Real::zero();
        self.target = Point3::origin();
    }

    /// Build a lossy f64 view-projection matrix under an explicit predicate policy.
    pub fn projection64(
        &self,
        viewport: Viewport,
        policy: PredicatePolicy,
    ) -> Result<Projection64> {
        viewport.validate()?;
        validate_camera_domain(self, policy)?;
        let yaw = real_to_f64(&self.yaw, "camera yaw")?;
        let pitch = real_to_f64(&self.pitch, "camera pitch")?;
        let roll = real_to_f64(&self.roll, "camera roll")?;
        let zoom = real_to_f64(&self.zoom, "camera zoom")?;
        let fov_y = real_to_f64(&self.fov_y, "camera fov_y")?;
        let near = real_to_f64(&self.near, "camera near")?;
        let far = real_to_f64(&self.far, "camera far")?;
        validate_lowered_camera_domain(viewport.aspect(), zoom, fov_y, near, far)?;
        let target = self
            .target
            .to_f64_array_lossy()
            .ok_or(Error::NonFiniteProjection {
                value: "camera target",
            })?;

        let rotation = UnitQuaternion::from_euler_angles(pitch, yaw, roll);
        let proj = Perspective3::new(viewport.aspect(), fov_y, near, far);
        let view = Translation3::new(0.0, 0.0, -zoom).to_homogeneous()
            * rotation.to_homogeneous()
            * Translation3::new(-target[0], -target[1], -target[2]).to_homogeneous();

        Projection64::new(proj.as_matrix() * view)
    }

    /// Project a hyperreal world point to screen coordinates.
    pub fn project_point(
        &self,
        viewport: Viewport,
        world: &Point3,
        policy: PredicatePolicy,
    ) -> Result<Option<ScreenPoint>> {
        project_point(&self.projection64(viewport, policy)?, viewport, world)
    }
}

/// Project a hyperreal world point through an f64 projection matrix.
#[cfg(feature = "exact")]
pub fn project_point(
    projection: &Projection64,
    viewport: Viewport,
    world: &Point3,
) -> Result<Option<ScreenPoint>> {
    viewport.validate()?;
    let [x, y, z] = world
        .to_f64_array_lossy()
        .ok_or(Error::NonFiniteProjection {
            value: "world point",
        })?;
    let clip = projection.matrix * Vector4::new(x, y, z, 1.0);
    if clip.iter().any(|value| !value.is_finite()) {
        return Err(Error::NonFiniteProjection {
            value: "projected point",
        });
    }
    if clip.w <= 0.0 {
        return Ok(None);
    }
    let ndc_x = clip.x / clip.w;
    let ndc_y = clip.y / clip.w;
    let center = viewport.center();
    let screen_x = center.x + ndc_x * viewport.width * 0.5;
    let screen_y = center.y - ndc_y * viewport.height * 0.5;
    ScreenPoint::new(screen_x, screen_y)
        .map(Some)
        .map_err(|_| Error::NonFiniteProjection {
            value: "screen point",
        })
}

/// Approximately unproject a screen point onto the Z=0 world plane.
///
/// Matrix inversion and ray construction happen in `f64`, so the returned
/// [`ApproxPoint3`] is presentation-space evidence, not exact geometry. Returns
/// `None` when the matrix is singular or ill-conditioned, its homogeneous
/// points are unstable, or the ray is parallel to the plane at numerical
/// precision.
#[cfg(feature = "exact")]
pub fn unproject_to_z0(
    projection: &Projection64,
    viewport: Viewport,
    screen: ScreenPoint,
) -> Result<Option<ApproxPoint3>> {
    viewport.validate()?;
    let Some(inverse) = projection.matrix.try_inverse() else {
        return Ok(None);
    };
    if !inverse.iter().all(|value| value.is_finite())
        || !inverse_is_well_conditioned(&projection.matrix, &inverse)
    {
        return Ok(None);
    }
    let nx = 2.0 * (screen.x - viewport.min_x) / viewport.width - 1.0;
    let ny = 1.0 - 2.0 * (screen.y - viewport.min_y) / viewport.height;
    if !nx.is_finite() || !ny.is_finite() {
        return Err(Error::NonFiniteProjection {
            value: "normalized screen point",
        });
    }
    let to_world = |z: f64| -> Result<Option<[f64; 3]>> {
        let clip = Vector4::new(nx, ny, z, 1.0);
        let w = inverse * clip;
        if w.iter().any(|value| !value.is_finite()) {
            return Err(Error::NonFiniteProjection {
                value: "unprojected point",
            });
        }
        let homogeneous_scale = w
            .iter()
            .fold(0.0_f64, |scale, value| scale.max(value.abs()));
        if homogeneous_scale == 0.0 || w.w.abs() <= numerical_tolerance(homogeneous_scale) {
            return Ok(None);
        }
        let point = [w.x / w.w, w.y / w.w, w.z / w.w];
        if point.iter().any(|value| !value.is_finite()) {
            return Err(Error::NonFiniteProjection {
                value: "unprojected point",
            });
        }
        Ok(Some(point))
    };
    let Some(near) = to_world(-1.0)? else {
        return Ok(None);
    };
    let Some(far) = to_world(1.0)? else {
        return Ok(None);
    };
    ray_intersection_with_z0(near, far)
}

#[cfg(feature = "exact")]
const NUMERICAL_TOLERANCE_FACTOR: f64 = 64.0;

#[cfg(feature = "exact")]
fn numerical_tolerance(scale: f64) -> f64 {
    NUMERICAL_TOLERANCE_FACTOR * f64::EPSILON * scale
}

#[cfg(feature = "exact")]
fn inverse_is_well_conditioned(matrix: &Matrix4<f64>, inverse: &Matrix4<f64>) -> bool {
    let infinity_norm = |value: &Matrix4<f64>| {
        (0..4)
            .map(|row| (0..4).map(|column| value[(row, column)].abs()).sum::<f64>())
            .fold(0.0_f64, f64::max)
    };
    let condition_estimate = infinity_norm(matrix) * infinity_norm(inverse);
    condition_estimate.is_finite()
        && condition_estimate * NUMERICAL_TOLERANCE_FACTOR * f64::EPSILON < 1.0
}

#[cfg(feature = "exact")]
fn ray_intersection_with_z0(near: [f64; 3], far: [f64; 3]) -> Result<Option<ApproxPoint3>> {
    let direction = [far[0] - near[0], far[1] - near[1], far[2] - near[2]];
    if direction.iter().any(|value| !value.is_finite()) {
        return Err(Error::NonFiniteProjection {
            value: "unprojection ray",
        });
    }
    let direction_scale = direction
        .iter()
        .fold(0.0_f64, |scale, value| scale.max(value.abs()));
    if direction_scale == 0.0 || direction[2].abs() <= numerical_tolerance(direction_scale) {
        return Ok(None);
    }
    let t = -near[2] / direction[2];
    ApproxPoint3::new([near[0] + direction[0] * t, near[1] + direction[1] * t, 0.0])
        .map(Some)
        .map_err(|_| Error::NonFiniteProjection {
            value: "unprojected point",
        })
}

#[cfg(feature = "exact")]
fn degrees(value: i32) -> Real {
    (Real::from(value) * pi() / Real::from(180)).expect("180 is a known nonzero divisor")
}

#[cfg(feature = "exact")]
fn real_to_f64(value: &Real, name: &'static str) -> Result<f64> {
    value
        .to_f64_lossy()
        .filter(|value| value.is_finite())
        .ok_or(Error::NonFiniteProjection { value: name })
}

#[cfg(feature = "exact")]
fn validate_camera_domain(camera: &ExactCamera, policy: PredicatePolicy) -> Result<()> {
    let zero = Real::zero();
    for (value, name) in [
        (&camera.zoom, "zoom"),
        (&camera.fov_y, "fov_y"),
        (&camera.near, "near"),
    ] {
        if decide_ordering(value, &zero, name, policy)? != Ordering::Greater {
            return Err(Error::InvalidCameraParameter { value: name });
        }
    }
    if decide_ordering(&camera.fov_y, &pi(), "fov_y", policy)? != Ordering::Less {
        return Err(Error::InvalidCameraParameter { value: "fov_y" });
    }
    if decide_ordering(&camera.far, &camera.near, "far", policy)? != Ordering::Greater {
        return Err(Error::InvalidCameraParameter { value: "far" });
    }
    Ok(())
}

#[cfg(feature = "exact")]
fn validate_lowered_camera_domain(
    aspect: f64,
    zoom: f64,
    fov_y: f64,
    near: f64,
    far: f64,
) -> Result<()> {
    if aspect <= f64::EPSILON {
        return Err(Error::InvalidViewportAspect);
    }
    if zoom <= 0.0 {
        return Err(Error::InvalidCameraParameter { value: "zoom" });
    }
    if fov_y <= f64::EPSILON || std::f64::consts::PI - fov_y <= f64::EPSILON {
        return Err(Error::InvalidCameraParameter { value: "fov_y" });
    }
    if near <= 0.0 {
        return Err(Error::InvalidCameraParameter { value: "near" });
    }
    if far <= near || !primitive_values_distinct(far, near) {
        return Err(Error::InvalidCameraParameter { value: "far" });
    }
    Ok(())
}

#[cfg(feature = "exact")]
fn primitive_values_distinct(left: f64, right: f64) -> bool {
    let difference = (left - right).abs();
    difference > f64::EPSILON && difference > f64::EPSILON * left.abs().max(right.abs())
}

#[cfg(feature = "exact")]
fn decide_ordering(
    left: &Real,
    right: &Real,
    predicate: &'static str,
    policy: PredicatePolicy,
) -> Result<Ordering> {
    match compare_reals(left, right, policy) {
        PredicateOutcome::Decided { value, .. } => Ok(value),
        PredicateOutcome::Unknown { needed, stage } => Err(Error::IndeterminatePredicate {
            predicate,
            needed,
            stage,
        }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[cfg(feature = "exact")]
    #[test]
    fn default_camera_projects_origin() {
        let camera = ExactCamera::default();
        let viewport = Viewport::new(0.0, 0.0, 800.0, 600.0).unwrap();
        let screen = camera
            .project_point(viewport, &Point3::origin(), PredicatePolicy::STRICT)
            .unwrap()
            .unwrap();
        assert!(screen.x().is_finite());
        assert!(screen.y().is_finite());
    }

    #[test]
    fn viewport_rejects_invalid_projection_domains() {
        assert!(matches!(
            Viewport::new(0.0, 0.0, 0.0, 600.0),
            Err(Error::NonPositiveViewportExtent { value: "width" })
        ));
        assert!(matches!(
            Viewport::new(0.0, 0.0, 800.0, -1.0),
            Err(Error::NonPositiveViewportExtent { value: "height" })
        ));
        assert!(matches!(
            Viewport::new(f64::MAX, 0.0, f64::MAX, 1.0),
            Err(Error::NonFinitePrimitive {
                value: "viewport maximum x"
            })
        ));
        assert!(matches!(
            Viewport::new(0.0, 0.0, f64::MAX, f64::MIN_POSITIVE),
            Err(Error::InvalidViewportAspect)
        ));
    }

    #[cfg(feature = "exact")]
    #[test]
    fn camera_rejects_invalid_projection_domains() {
        let viewport = Viewport::new(0.0, 0.0, 800.0, 600.0).unwrap();
        let camera = ExactCamera {
            near: Real::zero(),
            ..ExactCamera::default()
        };
        assert!(matches!(
            camera.projection64(viewport, PredicatePolicy::STRICT),
            Err(Error::InvalidCameraParameter { value: "near" })
        ));

        let camera = ExactCamera {
            fov_y: pi(),
            ..ExactCamera::default()
        };
        assert!(matches!(
            camera.projection64(viewport, PredicatePolicy::STRICT),
            Err(Error::InvalidCameraParameter { value: "fov_y" })
        ));
    }

    #[cfg(feature = "exact")]
    #[test]
    fn camera_domain_checks_use_requested_predicate_policy() {
        let viewport = Viewport::new(0.0, 0.0, 800.0, 600.0).unwrap();
        ExactCamera::default()
            .projection64(viewport, PredicatePolicy::STRICT)
            .expect("default camera has a strictly decidable domain");

        let mut camera = ExactCamera::default();
        assert!(matches!(
            camera.zoom_by(Real::zero(), PredicatePolicy::STRICT),
            Err(Error::InvalidCameraParameter {
                value: "zoom factor"
            })
        ));
        assert!(matches!(
            camera.zoom_by(Real::from(-1), PredicatePolicy::STRICT),
            Err(Error::InvalidCameraParameter {
                value: "zoom factor"
            })
        ));
    }

    #[cfg(feature = "exact")]
    #[test]
    fn projection_lowering_rejects_unstable_domains_without_panicking() {
        let tiny_aspect = Viewport::new(0.0, 0.0, f64::MIN_POSITIVE, 1.0).unwrap();
        assert!(matches!(
            ExactCamera::default().projection64(tiny_aspect, PredicatePolicy::STRICT),
            Err(Error::InvalidViewportAspect)
        ));

        let near = 1.0_f64;
        let far = f64::from_bits(near.to_bits() + 1);
        let camera = ExactCamera {
            near: Real::try_from(near).unwrap(),
            far: Real::try_from(far).unwrap(),
            ..ExactCamera::default()
        };
        assert!(matches!(
            camera.projection64(
                Viewport::new(0.0, 0.0, 800.0, 600.0).unwrap(),
                PredicatePolicy::STRICT,
            ),
            Err(Error::InvalidCameraParameter { value: "far" })
        ));

        let near = Real::one();
        let far = near.clone()
            + Real::new(hyperreal::Rational::fraction(1, u64::MAX).expect("nonzero denominator"));
        assert_eq!(near.to_f64_lossy(), far.to_f64_lossy());
        let camera = ExactCamera {
            near,
            far,
            ..ExactCamera::default()
        };
        assert!(matches!(
            camera.projection64(
                Viewport::new(0.0, 0.0, 800.0, 600.0).unwrap(),
                PredicatePolicy::STRICT,
            ),
            Err(Error::InvalidCameraParameter { value: "far" })
        ));
    }

    #[test]
    fn checked_screen_and_approximate_points_reject_non_finite_values() {
        assert!(matches!(
            ScreenPoint::new(f64::NAN, 0.0),
            Err(Error::NonFinitePrimitive {
                value: "screen point"
            })
        ));
        assert!(matches!(
            ApproxPoint3::new([0.0, 0.0, f64::INFINITY]),
            Err(Error::NonFinitePrimitive {
                value: "approximate world point"
            })
        ));
    }

    #[cfg(feature = "exact")]
    #[test]
    fn project_unproject_round_trip_retains_approximate_provenance() {
        let camera = ExactCamera::default();
        let viewport = Viewport::new(10.0, 20.0, 800.0, 600.0).unwrap();
        let projection = camera
            .projection64(viewport, PredicatePolicy::STRICT)
            .unwrap();
        let screen = camera
            .project_point(viewport, &Point3::origin(), PredicatePolicy::STRICT)
            .unwrap()
            .unwrap();
        let world = unproject_to_z0(&projection, viewport, screen)
            .unwrap()
            .unwrap()
            .to_array();

        assert!(world[0].abs() < 1.0e-10, "x={}", world[0]);
        assert!(world[1].abs() < 1.0e-10, "y={}", world[1]);
        assert_eq!(world[2], 0.0);
    }

    #[cfg(feature = "exact")]
    #[test]
    fn approximate_unprojection_rejects_nearly_parallel_rays() {
        assert_eq!(
            ray_intersection_with_z0([0.0, 0.0, 1.0], [1.0, 0.0, 1.0 + f64::EPSILON]).unwrap(),
            None
        );

        let mut ill_conditioned = Matrix4::identity();
        ill_conditioned[(2, 2)] = 1.0e-16;
        let inverse = ill_conditioned.try_inverse().unwrap();
        assert!(!inverse_is_well_conditioned(&ill_conditioned, &inverse));
    }

    #[cfg(feature = "exact")]
    #[test]
    fn projection_rejects_non_finite_matrix() {
        let mut matrix = Matrix4::identity();
        matrix[(1, 2)] = f64::INFINITY;
        assert!(matches!(
            Projection64::new(matrix),
            Err(Error::NonFinitePrimitive {
                value: "projection matrix"
            })
        ));
    }

    #[test]
    fn projection_accepts_external_column_major_f32_matrix() {
        let values = [
            1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0, 13.0, 14.0, 15.0, 16.0,
        ];
        let projection = Projection64::try_from_column_major_f32(values).unwrap();
        assert_eq!(projection.to_f32_array().unwrap(), values);
    }

    #[test]
    fn projection_rejects_non_finite_external_matrix() {
        let mut values = [0.0; 16];
        values[7] = f32::NAN;
        assert!(matches!(
            Projection64::try_from_column_major_f32(values),
            Err(Error::NonFinitePrimitive {
                value: "projection matrix"
            })
        ));
    }
}
