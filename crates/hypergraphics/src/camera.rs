//! Exact camera state and explicit f64 projection.

use hyperlattice::{Point3, Real, Vector3, pi};
use nalgebra::{Matrix4, Perspective3, Translation3, UnitQuaternion, Vector4};

use crate::error::{Error, Result};

/// Rectangular render viewport in screen coordinates.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Viewport {
    /// Minimum screen x coordinate.
    pub min_x: f64,
    /// Minimum screen y coordinate.
    pub min_y: f64,
    /// Viewport width in pixels.
    pub width: f64,
    /// Viewport height in pixels.
    pub height: f64,
}

impl Viewport {
    /// Construct a finite viewport.
    pub fn new(min_x: f64, min_y: f64, width: f64, height: f64) -> Result<Self> {
        for (value, name) in [
            (min_x, "min_x"),
            (min_y, "min_y"),
            (width, "width"),
            (height, "height"),
        ] {
            if !value.is_finite() {
                return Err(Error::NonFinitePrimitive { value: name });
            }
        }
        if width <= 0.0 {
            return Err(Error::NonPositiveViewportExtent { value: "width" });
        }
        if height <= 0.0 {
            return Err(Error::NonPositiveViewportExtent { value: "height" });
        }
        Ok(Self {
            min_x,
            min_y,
            width,
            height,
        })
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

/// 2D screen point.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ScreenPoint {
    /// Screen x coordinate.
    pub x: f64,
    /// Screen y coordinate.
    pub y: f64,
}

/// `f64` render projection matrix.
#[derive(Clone, Debug, PartialEq)]
pub struct Projection64 {
    matrix: Matrix4<f64>,
}

impl Projection64 {
    /// Construct from a nalgebra matrix.
    pub const fn new(matrix: Matrix4<f64>) -> Self {
        Self { matrix }
    }

    /// Borrow the matrix.
    pub const fn matrix(&self) -> &Matrix4<f64> {
        &self.matrix
    }

    /// Return a finite f32 column-major slice for OpenGL uniforms.
    pub fn to_f32_array(&self) -> Result<[f32; 16]> {
        let mut out = [0.0_f32; 16];
        for (index, value) in self.matrix.as_slice().iter().copied().enumerate() {
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

impl ExactCamera {
    /// Apply an exact orbit delta in radians.
    pub fn orbit(&mut self, yaw_delta: Real, pitch_delta: Real) {
        self.yaw = self.yaw.clone() + yaw_delta;
        self.pitch = self.pitch.clone() + pitch_delta;
    }

    /// Apply a multiplicative zoom factor.
    pub fn zoom_by(&mut self, factor: Real) -> Result<()> {
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

    /// Build a lossy f64 view-projection matrix for a viewport.
    pub fn projection64(&self, viewport: Viewport) -> Result<Projection64> {
        let yaw = real_to_f64(&self.yaw, "camera yaw")?;
        let pitch = real_to_f64(&self.pitch, "camera pitch")?;
        let roll = real_to_f64(&self.roll, "camera roll")?;
        let zoom = real_to_f64(&self.zoom, "camera zoom")?;
        let fov_y = real_to_f64(&self.fov_y, "camera fov_y")?;
        let near = real_to_f64(&self.near, "camera near")?;
        let far = real_to_f64(&self.far, "camera far")?;
        if zoom <= 0.0 {
            return Err(Error::InvalidCameraParameter { value: "zoom" });
        }
        if !(0.0 < fov_y && fov_y < std::f64::consts::PI) {
            return Err(Error::InvalidCameraParameter { value: "fov_y" });
        }
        if near <= 0.0 {
            return Err(Error::InvalidCameraParameter { value: "near" });
        }
        if far <= near {
            return Err(Error::InvalidCameraParameter { value: "far" });
        }
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

        Ok(Projection64::new(proj.as_matrix() * view))
    }

    /// Project a hyperreal world point to screen coordinates.
    pub fn project_point(&self, viewport: Viewport, world: &Point3) -> Result<Option<ScreenPoint>> {
        project_point(&self.projection64(viewport)?, viewport, world)
    }
}

/// Project a hyperreal world point through an f64 projection matrix.
pub fn project_point(
    projection: &Projection64,
    viewport: Viewport,
    world: &Point3,
) -> Result<Option<ScreenPoint>> {
    let [x, y, z] = world
        .to_f64_array_lossy()
        .ok_or(Error::NonFiniteProjection {
            value: "world point",
        })?;
    let clip = projection.matrix * Vector4::new(x, y, z, 1.0);
    if clip.w <= 0.0 {
        return Ok(None);
    }
    let ndc_x = clip.x / clip.w;
    let ndc_y = clip.y / clip.w;
    let center = viewport.center();
    Ok(Some(ScreenPoint {
        x: center.x + ndc_x * viewport.width * 0.5,
        y: center.y - ndc_y * viewport.height * 0.5,
    }))
}

/// Unproject a screen point onto the Z=0 world plane.
pub fn unproject_to_z0(
    projection: &Projection64,
    viewport: Viewport,
    screen: ScreenPoint,
) -> Result<Option<Point3>> {
    let Some(inverse) = projection.matrix.try_inverse() else {
        return Ok(None);
    };
    let nx = 2.0 * (screen.x - viewport.min_x) / viewport.width - 1.0;
    let ny = 1.0 - 2.0 * (screen.y - viewport.min_y) / viewport.height;
    let to_world = |z: f64| -> Option<Vector3> {
        let clip = Vector4::new(nx, ny, z, 1.0);
        let w = inverse * clip;
        if w.w.abs() < 1e-12 {
            return None;
        }
        let point = Point3::try_from_f64_array([w.x / w.w, w.y / w.w, w.z / w.w]).ok()?;
        Some(point.to_vector())
    };
    let Some(near) = to_world(-1.0) else {
        return Ok(None);
    };
    let Some(far) = to_world(1.0) else {
        return Ok(None);
    };
    let dz = far[2].clone() - near[2].clone();
    let t = (-near[2].clone() / dz)?;
    let world = near.clone() + (far - near) * t;
    Ok(Some(Point3::from(world)))
}

fn degrees(value: i32) -> Real {
    (Real::from(value) * pi() / Real::from(180)).expect("180 is a known nonzero divisor")
}

fn real_to_f64(value: &Real, name: &'static str) -> Result<f64> {
    value
        .to_f64_lossy()
        .filter(|value| value.is_finite())
        .ok_or(Error::NonFiniteProjection { value: name })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_camera_projects_origin() {
        let camera = ExactCamera::default();
        let viewport = Viewport::new(0.0, 0.0, 800.0, 600.0).unwrap();
        let screen = camera
            .project_point(viewport, &Point3::origin())
            .unwrap()
            .unwrap();
        assert!(screen.x.is_finite());
        assert!(screen.y.is_finite());
    }

    #[test]
    fn viewport_and_camera_reject_invalid_projection_domains() {
        assert!(matches!(
            Viewport::new(0.0, 0.0, 0.0, 600.0),
            Err(Error::NonPositiveViewportExtent { value: "width" })
        ));
        assert!(matches!(
            Viewport::new(0.0, 0.0, 800.0, -1.0),
            Err(Error::NonPositiveViewportExtent { value: "height" })
        ));

        let viewport = Viewport::new(0.0, 0.0, 800.0, 600.0).unwrap();
        let camera = ExactCamera {
            near: Real::zero(),
            ..ExactCamera::default()
        };
        assert!(matches!(
            camera.projection64(viewport),
            Err(Error::InvalidCameraParameter { value: "near" })
        ));
    }
}
