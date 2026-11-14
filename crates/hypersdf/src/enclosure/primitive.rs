//! Source polynomial lowering; parameter decisions stay with the primitive owner.
use super::{B, Builder, ExpressionError, SdfCoordinate, SdfProgramError, U};
use crate::{
    SdfPrimitive,
    evaluation::require_domain,
    primitive::{radius_squared_domain, torus_domain},
};
use hyperlimit::Point3;
use hyperreal::Real;

// Additional graph records plus one traversal result. Unsupported families emit none.
pub(super) fn entries(primitive: &SdfPrimitive) -> usize {
    match primitive {
        SdfPrimitive::Plane { .. } => 11,
        SdfPrimitive::Sphere { .. } => 14,
        SdfPrimitive::Torus { .. } => 21,
        _ => 0,
    }
}
impl Builder<'_> {
    pub(super) fn primitive(
        &mut self,
        primitive: &SdfPrimitive,
        xyz: [usize; 3],
    ) -> Result<usize, SdfProgramError> {
        match primitive {
            SdfPrimitive::Plane { plane } => Ok(self.linear(
                [&plane.normal.x, &plane.normal.y, &plane.normal.z],
                &plane.offset,
                xyz,
            )?),
            SdfPrimitive::Sphere {
                center,
                radius_squared,
            } => {
                self.budget.charge(1)?;
                require_domain(
                    radius_squared_domain(radius_squared),
                    "sphere radius squared",
                )?;
                let squares = self.centered_squares(center, xyz)?;
                let xy = self.binary(B::Add, squares[0], squares[1])?;
                let sum = self.binary(B::Add, xy, squares[2])?;
                let radius = self.constant(radius_squared)?;
                Ok(self.binary(B::Sub, sum, radius)?)
            }
            SdfPrimitive::Torus {
                axis,
                center,
                major_radius_squared,
                minor_radius_squared,
            } => {
                self.budget.charge(1)?;
                require_domain(
                    torus_domain(major_radius_squared, minor_radius_squared),
                    "torus squared radii",
                )?;
                let squares = self.centered_squares(center, xyz)?;
                let (a, b, axial) = match axis {
                    SdfCoordinate::X => (1, 2, 0),
                    SdfCoordinate::Y => (0, 2, 1),
                    SdfCoordinate::Z => (0, 1, 2),
                };
                let radial = self.binary(B::Add, squares[a], squares[b])?;
                let sum = self.binary(B::Add, radial, squares[axial])?;
                let major = self.constant(major_radius_squared)?;
                let sum = self.binary(B::Add, sum, major)?;
                let minor = self.constant(minor_radius_squared)?;
                let q = self.binary(B::Sub, sum, minor)?;
                let q_squared = self.unary(U::PowI(2), q)?;
                let four = self.constant(&Real::from(4))?;
                let scaled_major = self.binary(B::Mul, four, major)?;
                let radial_term = self.binary(B::Mul, scaled_major, radial)?;
                Ok(self.binary(B::Sub, q_squared, radial_term)?)
            }
            _ => Err(SdfProgramError::Unsupported("primitive")),
        }
    }
    fn centered_squares(
        &mut self,
        center: &Point3,
        xyz: [usize; 3],
    ) -> Result<[usize; 3], ExpressionError> {
        let mut squares = [0; 3];
        for (i, value) in [&center.x, &center.y, &center.z].into_iter().enumerate() {
            let c = self.constant(value)?;
            let delta = self.binary(B::Sub, c, xyz[i])?;
            // Keep the repeated factor correlated in native interval evaluation.
            squares[i] = self.unary(U::PowI(2), delta)?;
        }
        Ok(squares)
    }
}
