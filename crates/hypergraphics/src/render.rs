//! Primitive-float render attributes shared by exact and backend-only users.

use crate::error::{Error, Result};

/// Render primitive used by a mesh.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Primitive {
    /// Independent line segments.
    Lines,
    /// Independent triangles.
    Triangles,
}

/// Linear RGB color. Color is a render attribute, not geometric data.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Color3 {
    r: f32,
    g: f32,
    b: f32,
}

impl Color3 {
    /// Linear red.
    pub const RED: Self = Self {
        r: 1.0,
        g: 0.0,
        b: 0.0,
    };

    /// Linear green.
    pub const GREEN: Self = Self {
        r: 0.0,
        g: 1.0,
        b: 0.0,
    };

    /// Linear blue.
    pub const BLUE: Self = Self {
        r: 0.0,
        g: 0.0,
        b: 1.0,
    };

    /// Construct a finite linear RGB color.
    pub fn new(r: f32, g: f32, b: f32) -> Result<Self> {
        if !r.is_finite() {
            return Err(Error::NonFinitePrimitive { value: "red" });
        }
        if !g.is_finite() {
            return Err(Error::NonFinitePrimitive { value: "green" });
        }
        if !b.is_finite() {
            return Err(Error::NonFinitePrimitive { value: "blue" });
        }
        Ok(Self { r, g, b })
    }

    /// Return the red channel.
    pub const fn r(self) -> f32 {
        self.r
    }

    /// Return the green channel.
    pub const fn g(self) -> f32 {
        self.g
    }

    /// Return the blue channel.
    pub const fn b(self) -> f32 {
        self.b
    }

    /// Return `[r, g, b]`.
    pub const fn to_array(self) -> [f32; 3] {
        [self.r, self.g, self.b]
    }
}

/// One lossy render vertex after explicit projection/export to f64.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RenderVertex64 {
    position: [f64; 3],
    color: Color3,
}

impl RenderVertex64 {
    /// Construct a render vertex with a finite position and color.
    pub fn new(position: [f64; 3], color: Color3) -> Result<Self> {
        if position.iter().any(|value| !value.is_finite()) {
            return Err(Error::NonFinitePrimitive {
                value: "render vertex position",
            });
        }
        Ok(Self { position, color })
    }

    /// Borrow the finite position.
    pub const fn position(&self) -> &[f64; 3] {
        &self.position
    }

    /// Return the finite color.
    pub const fn color(&self) -> Color3 {
        self.color
    }

    /// Return the interleaved `xyz rgb` values as `f64`.
    pub fn to_xyz_rgb(self) -> [f64; 6] {
        [
            self.position[0],
            self.position[1],
            self.position[2],
            f64::from(self.color.r),
            f64::from(self.color.g),
            f64::from(self.color.b),
        ]
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn checked_render_values_reject_non_finite_components() {
        assert!(matches!(
            Color3::new(f32::NAN, 0.0, 0.0),
            Err(Error::NonFinitePrimitive { value: "red" })
        ));
        assert!(matches!(
            RenderVertex64::new([0.0, f64::INFINITY, 0.0], Color3::RED),
            Err(Error::NonFinitePrimitive {
                value: "render vertex position"
            })
        ));
    }

    #[test]
    fn checked_render_values_round_trip() {
        let color = Color3::new(0.25, 0.5, 0.75).unwrap();
        let vertex = RenderVertex64::new([1.0, 2.0, 3.0], color).unwrap();

        assert_eq!(vertex.position(), &[1.0, 2.0, 3.0]);
        assert_eq!(vertex.color(), color);
        assert_eq!(vertex.to_xyz_rgb(), [1.0, 2.0, 3.0, 0.25, 0.5, 0.75]);
    }
}
