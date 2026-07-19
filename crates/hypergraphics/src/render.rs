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
    /// Red channel.
    pub r: f32,
    /// Green channel.
    pub g: f32,
    /// Blue channel.
    pub b: f32,
}

impl Color3 {
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

    /// Return `[r, g, b]`.
    pub const fn to_array(self) -> [f32; 3] {
        [self.r, self.g, self.b]
    }
}

/// One lossy render vertex after explicit projection/export to f64.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct RenderVertex64 {
    /// World or projected position.
    pub position: [f64; 3],
    /// Linear RGB color.
    pub color: [f32; 3],
}

impl RenderVertex64 {
    /// Return the interleaved `xyz rgb` values as `f64`.
    pub fn to_xyz_rgb(self) -> [f64; 6] {
        [
            self.position[0],
            self.position[1],
            self.position[2],
            f64::from(self.color[0]),
            f64::from(self.color[1]),
            f64::from(self.color[2]),
        ]
    }
}
