//! Retained expression tree for exact-aware implicit fields.
//!
//! This first expression core keeps the CSG operators that preserve zero-set
//! sign semantics: `min` for union, `max` for intersection, and negation for
//! complement. True metric-distance status is weakened unless a proof is
//! available, which follows the metric/status split in Yap-style exact
//! geometric computation rather than treating every formula as an SDF.

use crate::primitive::SdfPrimitive;
use crate::status::SdfMetricStatus;
use crate::transform::{SdfTransform, SdfTransformError};

/// Owned typed SDF/implicit expression.
#[derive(Clone, Debug, PartialEq)]
pub enum SdfExpr {
    /// Exact scalar constant field.
    Constant(hyperreal::Real),
    /// Coordinate extraction field.
    Coordinate(SdfCoordinate),
    /// Exact linear scalar field `coefficients dot point + offset`.
    ///
    /// The coefficient vector is retained as a `hyperlattice::Vector3` so
    /// vector structural facts remain available to downstream exact kernels
    /// before scalar expansion, matching Yap's object-package guidance.
    Linear {
        /// Exact vector coefficients in `[x, y, z]` order.
        coefficients: hyperlattice::Vector3,
        /// Exact scalar offset.
        offset: hyperreal::Real,
    },
    /// Analytic primitive with a retained exact classifier.
    Primitive(SdfPrimitive),
    /// CSG union, represented by `min(left, right)`.
    Union(Box<SdfExpr>, Box<SdfExpr>),
    /// CSG intersection, represented by `max(left, right)`.
    Intersection(Box<SdfExpr>, Box<SdfExpr>),
    /// Exact scalar addition.
    Add(Box<SdfExpr>, Box<SdfExpr>),
    /// Exact scalar subtraction.
    Sub(Box<SdfExpr>, Box<SdfExpr>),
    /// Exact scalar multiplication.
    Mul(Box<SdfExpr>, Box<SdfExpr>),
    /// Exact absolute value.
    Abs(Box<SdfExpr>),
    /// Exact principal square root.
    ///
    /// Negative child values are outside the scalar domain and therefore
    /// become unknown predicate evidence at query time, instead of being
    /// rounded or clamped into a preview value.
    Sqrt(Box<SdfExpr>),
    /// Exact sine of a scalar child field.
    ///
    /// Scalar point evaluation delegates to `hyperreal::Real::sin`, preserving
    /// exact pi-rational shortcuts and computable certificates. Cell intervals
    /// remain explicit unknown until a certified trig range reducer is attached.
    Sin(Box<SdfExpr>),
    /// Exact cosine of a scalar child field.
    ///
    /// Point classification may use exact/computable `Real` signs, while cell
    /// classification refuses to infer topology from primitive-float samples.
    Cos(Box<SdfExpr>),
    /// Exact tangent of a scalar child field.
    ///
    /// `hyperreal::Real::tan` rejects poles. Rejected point values become
    /// explicit unknowns instead of silently clamped scalar evidence.
    Tan(Box<SdfExpr>),
    /// CSG complement, represented by `-field`.
    Complement(Box<SdfExpr>),
    /// Exact scalar offset, represented by `field - amount`.
    Offset {
        /// Child expression.
        child: Box<SdfExpr>,
        /// Exact offset amount. Positive values expand negative-inside fields.
        amount: hyperreal::Real,
    },
    /// Exact transform applied lazily to a child field.
    Transform {
        /// Child expression in object space.
        child: Box<SdfExpr>,
        /// Transform from child object space to parent/world space.
        transform: SdfTransform,
    },
}

/// Coordinate component used by [`SdfExpr::Coordinate`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdfCoordinate {
    /// X coordinate.
    X,
    /// Y coordinate.
    Y,
    /// Z coordinate.
    Z,
}

impl SdfExpr {
    /// Construct an exact scalar constant expression.
    pub fn constant(value: hyperreal::Real) -> Self {
        Self::Constant(value)
    }

    /// Construct an x-coordinate expression.
    pub const fn x() -> Self {
        Self::Coordinate(SdfCoordinate::X)
    }

    /// Construct a y-coordinate expression.
    pub const fn y() -> Self {
        Self::Coordinate(SdfCoordinate::Y)
    }

    /// Construct a z-coordinate expression.
    pub const fn z() -> Self {
        Self::Coordinate(SdfCoordinate::Z)
    }

    /// Construct an exact linear scalar field `coefficients dot point + offset`.
    pub fn linear(coefficients: hyperlattice::Vector3, offset: hyperreal::Real) -> Self {
        Self::Linear {
            coefficients,
            offset,
        }
    }

    /// Construct a primitive expression.
    pub fn primitive(primitive: SdfPrimitive) -> Self {
        Self::Primitive(primitive)
    }

    /// Construct a plane/half-space expression.
    pub fn plane(plane: hyperlimit::Plane3) -> Self {
        Self::Primitive(SdfPrimitive::plane(plane))
    }

    /// Construct a sphere expression.
    pub fn sphere(center: hyperlimit::Point3, radius_squared: hyperreal::Real) -> Self {
        Self::Primitive(SdfPrimitive::sphere(center, radius_squared))
    }

    /// Construct a closed AABB expression.
    pub fn aabb(min: hyperlimit::Point3, max: hyperlimit::Point3) -> Self {
        Self::Primitive(SdfPrimitive::aabb(min, max))
    }

    /// Construct a rounded AABB expression from a core box and squared radius.
    ///
    /// The squared radius keeps exact classification square-root-free; the
    /// primitive remains sign-equivalent rather than a metric-distance field.
    pub fn rounded_aabb(
        min: hyperlimit::Point3,
        max: hyperlimit::Point3,
        radius_squared: hyperreal::Real,
    ) -> Self {
        Self::Primitive(SdfPrimitive::rounded_aabb(min, max, radius_squared))
    }

    /// Construct a finite axis-aligned cylinder.
    pub fn cylinder(
        axis: SdfCoordinate,
        center: hyperlimit::Point3,
        radius_squared: hyperreal::Real,
        half_height: hyperreal::Real,
    ) -> Self {
        Self::Primitive(SdfPrimitive::cylinder(
            axis,
            center,
            radius_squared,
            half_height,
        ))
    }

    /// Construct an axis-aligned capsule or segment tube.
    pub fn capsule(
        axis: SdfCoordinate,
        center: hyperlimit::Point3,
        radius_squared: hyperreal::Real,
        half_length: hyperreal::Real,
    ) -> Self {
        Self::Primitive(SdfPrimitive::capsule(
            axis,
            center,
            radius_squared,
            half_length,
        ))
    }

    /// Construct an axis-aligned torus from squared radii.
    pub fn torus(
        axis: SdfCoordinate,
        center: hyperlimit::Point3,
        major_radius_squared: hyperreal::Real,
        minor_radius_squared: hyperreal::Real,
    ) -> Self {
        Self::Primitive(SdfPrimitive::torus(
            axis,
            center,
            major_radius_squared,
            minor_radius_squared,
        ))
    }

    /// Construct a finite slab around an oriented plane.
    pub fn slab(plane: hyperlimit::Plane3, half_width: hyperreal::Real) -> Self {
        Self::Primitive(SdfPrimitive::slab(plane, half_width))
    }

    /// Construct a CSG union expression.
    pub fn union(self, other: Self) -> Self {
        Self::Union(Box::new(self), Box::new(other))
    }

    /// Construct a CSG intersection expression.
    pub fn intersection(self, other: Self) -> Self {
        Self::Intersection(Box::new(self), Box::new(other))
    }

    /// Construct an exact scalar addition expression.
    pub fn add_expr(self, other: Self) -> Self {
        Self::Add(Box::new(self), Box::new(other))
    }

    /// Construct an exact scalar subtraction expression.
    pub fn sub_expr(self, other: Self) -> Self {
        Self::Sub(Box::new(self), Box::new(other))
    }

    /// Construct an exact scalar multiplication expression.
    pub fn mul_expr(self, other: Self) -> Self {
        Self::Mul(Box::new(self), Box::new(other))
    }

    /// Construct an exact absolute-value expression.
    pub fn abs(self) -> Self {
        Self::Abs(Box::new(self))
    }

    /// Construct an exact principal-square-root expression.
    pub fn sqrt(self) -> Self {
        Self::Sqrt(Box::new(self))
    }

    /// Construct an exact sine expression.
    pub fn sin(self) -> Self {
        Self::Sin(Box::new(self))
    }

    /// Construct an exact cosine expression.
    pub fn cos(self) -> Self {
        Self::Cos(Box::new(self))
    }

    /// Construct an exact tangent expression.
    pub fn tan(self) -> Self {
        Self::Tan(Box::new(self))
    }

    /// Construct a CSG complement expression.
    pub fn complement(self) -> Self {
        Self::Complement(Box::new(self))
    }

    /// Construct an exact scalar offset expression.
    pub fn offset(self, amount: hyperreal::Real) -> Self {
        Self::Offset {
            child: Box::new(self),
            amount,
        }
    }

    /// Construct a translated expression.
    pub fn translate(self, offset: hyperlimit::Point3) -> Self {
        Self::Transform {
            child: Box::new(self),
            transform: SdfTransform::translation(offset),
        }
    }

    /// Construct an exact affine transform node.
    ///
    /// The supplied matrix maps child/object coordinates into parent/world
    /// coordinates. Construction rejects non-affine or singular matrices so
    /// later point and cell classification can replay the exact inverse
    /// without hidden lossy projection.
    pub fn affine_transform(
        self,
        matrix: hyperlattice::Matrix4,
    ) -> Result<Self, SdfTransformError> {
        Ok(Self::Transform {
            child: Box::new(self),
            transform: SdfTransform::affine(matrix)?,
        })
    }

    /// Return the strongest metric claim currently justified by expression
    /// structure alone.
    pub fn metric_status(&self) -> SdfMetricStatus {
        match self {
            Self::Constant(_) | Self::Coordinate(_) | Self::Linear { .. } => {
                SdfMetricStatus::SignEquivalent
            }
            Self::Primitive(primitive) => primitive.metric_status(),
            Self::Union(left, right) | Self::Intersection(left, right) => {
                left.metric_status().csg_pair(right.metric_status())
            }
            Self::Add(_, _)
            | Self::Sub(_, _)
            | Self::Mul(_, _)
            | Self::Abs(_)
            | Self::Sqrt(_)
            | Self::Sin(_)
            | Self::Cos(_)
            | Self::Tan(_) => SdfMetricStatus::SignEquivalent,
            Self::Complement(inner) => inner.metric_status().complemented(),
            Self::Offset { child, .. } => child
                .metric_status()
                .csg_pair(SdfMetricStatus::SignEquivalent),
            Self::Transform { child, transform } => transform.metric_status(child.metric_status()),
        }
    }
}
