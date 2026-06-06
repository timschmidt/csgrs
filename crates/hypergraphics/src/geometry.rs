//! Hyperreal-owned mesh geometry and explicit render exports.

use hyperlattice::Point3;
use hyperlimit::{Certainty, PredicateOutcome, Sign, orient3d};

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

/// One exact scene vertex.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactVertex {
    /// Hyperreal-owned position.
    pub position: Point3,
    /// Render color.
    pub color: Color3,
}

impl ExactVertex {
    /// Construct one exact vertex.
    pub const fn new(position: Point3, color: Color3) -> Self {
        Self { position, color }
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
    /// Return the CopperForge-compatible `xyz rgb` stride as f64.
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

/// Orientation of one triangle against a query point.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum TriangleOrientation {
    /// The query point is on the negative side of the oriented triangle plane.
    Negative(Certainty),
    /// The query point is exactly coplanar with the oriented triangle plane.
    Coplanar(Certainty),
    /// The query point is on the positive side of the oriented triangle plane.
    Positive(Certainty),
    /// The exact predicate could not decide under the default policy.
    Unknown,
}

impl TriangleOrientation {
    fn from_outcome(outcome: PredicateOutcome<Sign>) -> Self {
        match outcome {
            PredicateOutcome::Decided {
                value: Sign::Negative,
                certainty,
                ..
            } => Self::Negative(certainty),
            PredicateOutcome::Decided {
                value: Sign::Zero,
                certainty,
                ..
            } => Self::Coplanar(certainty),
            PredicateOutcome::Decided {
                value: Sign::Positive,
                certainty,
                ..
            } => Self::Positive(certainty),
            PredicateOutcome::Unknown { .. } => Self::Unknown,
        }
    }
}

/// Hyperreal-owned colored mesh.
#[derive(Clone, Debug, PartialEq)]
pub struct ExactMesh {
    primitive: Primitive,
    vertices: Vec<ExactVertex>,
}

impl ExactMesh {
    /// Construct a mesh from exact vertices.
    pub fn new(primitive: Primitive, vertices: Vec<ExactVertex>) -> Self {
        Self {
            primitive,
            vertices,
        }
    }

    /// Construct an empty mesh.
    pub fn empty(primitive: Primitive) -> Self {
        Self::new(primitive, Vec::new())
    }

    /// Return the render primitive.
    pub const fn primitive(&self) -> Primitive {
        self.primitive
    }

    /// Borrow exact vertices.
    pub fn vertices(&self) -> &[ExactVertex] {
        &self.vertices
    }

    /// Mutably borrow exact vertices.
    pub fn vertices_mut(&mut self) -> &mut Vec<ExactVertex> {
        &mut self.vertices
    }

    /// Append one exact vertex.
    pub fn push(&mut self, vertex: ExactVertex) {
        self.vertices.push(vertex);
    }

    /// Return the number of vertices.
    pub fn vertex_count(&self) -> usize {
        self.vertices.len()
    }

    /// Return the number of full triangles in the mesh.
    pub fn triangle_count(&self) -> usize {
        if self.primitive == Primitive::Triangles {
            self.vertices.len() / 3
        } else {
            0
        }
    }

    /// Lossily export exact vertices to finite f64 render vertices.
    pub fn to_render_vertices64(&self) -> Result<Vec<RenderVertex64>> {
        self.vertices
            .iter()
            .map(|vertex| {
                let position =
                    vertex
                        .position
                        .to_f64_array_lossy()
                        .ok_or(Error::NonFiniteProjection {
                            value: "vertex position",
                        })?;
                Ok(RenderVertex64 {
                    position,
                    color: vertex.color.to_array(),
                })
            })
            .collect()
    }

    /// Lossily export a flat `xyz rgb` f64 buffer.
    pub fn to_xyz_rgb_f64(&self) -> Result<Vec<f64>> {
        let render_vertices = self.to_render_vertices64()?;
        let mut out = Vec::with_capacity(render_vertices.len() * 6);
        for vertex in render_vertices {
            out.extend_from_slice(&vertex.to_xyz_rgb());
        }
        Ok(out)
    }

    /// Run an exact orientation predicate for one triangle against `point`.
    pub fn triangle_orientation_against(
        &self,
        triangle_index: usize,
        point: &Point3,
    ) -> Result<TriangleOrientation> {
        if self.primitive != Primitive::Triangles {
            return Err(Error::RequiresTriangles);
        }
        let triangle_count = self.triangle_count();
        if triangle_index >= triangle_count {
            return Err(Error::TriangleIndexOutOfBounds {
                index: triangle_index,
                triangle_count,
            });
        }
        let base = triangle_index * 3;
        let a = &self.vertices[base].position;
        let b = &self.vertices[base + 1].position;
        let c = &self.vertices[base + 2].position;
        Ok(TriangleOrientation::from_outcome(orient3d(a, b, c, point)))
    }
}

#[cfg(test)]
mod tests {
    use hyperlattice::Point3;
    use hyperreal::Real;

    use super::*;

    fn p(x: i32, y: i32, z: i32) -> Point3 {
        Point3::new(Real::from(x), Real::from(y), Real::from(z))
    }

    #[test]
    fn exports_vertices_to_f64_at_boundary() {
        let red = Color3::new(1.0, 0.0, 0.0).unwrap();
        let mesh = ExactMesh::new(Primitive::Lines, vec![ExactVertex::new(p(1, 2, 3), red)]);

        assert_eq!(
            mesh.to_xyz_rgb_f64().unwrap(),
            vec![1.0, 2.0, 3.0, 1.0, 0.0, 0.0]
        );
    }

    #[test]
    fn triangle_orientation_uses_hyperlimit() {
        let color = Color3::new(0.0, 1.0, 0.0).unwrap();
        let mesh = ExactMesh::new(
            Primitive::Triangles,
            vec![
                ExactVertex::new(p(0, 0, 0), color),
                ExactVertex::new(p(1, 0, 0), color),
                ExactVertex::new(p(0, 1, 0), color),
            ],
        );

        assert!(matches!(
            mesh.triangle_orientation_against(0, &p(0, 0, 1)).unwrap(),
            TriangleOrientation::Positive(_) | TriangleOrientation::Negative(_)
        ));
    }
}
