//! Structural facts for retained SDF expressions.
//!
//! Facts are scheduling metadata, not topology certificates. They preserve
//! exact-set/common-scale opportunities for later kernels while classification
//! continues to return predicate reports. This follows Yap, "Towards Exact
//! Geometric Computation," *Computational Geometry* 7.1-2 (1997), especially
//! the separation between geometric object packages and arithmetic packages.

use hyperreal::{Real, RealExactSetFacts};

use crate::expr::SdfExpr;
use crate::primitive::SdfPrimitive;
use crate::status::{SdfDomainStatus, SdfGradientStatus, SdfLipschitzStatus, SdfMetricStatus};

/// Structural facts for an SDF expression.
#[derive(Clone, Debug, PartialEq)]
pub struct SdfFacts {
    /// Number of retained expression nodes.
    pub node_count: usize,
    /// Number of analytic primitive nodes.
    pub primitive_count: usize,
    /// Number of transform nodes.
    pub transform_count: usize,
    /// Exact-set facts for retained scalar parameters.
    pub parameter_exact: RealExactSetFacts,
    /// Metric claim implied by expression structure.
    pub metric_status: SdfMetricStatus,
    /// Certified domain validity for checked primitive parameters.
    pub domain_status: SdfDomainStatus,
    /// Gradient availability and provenance.
    pub gradient_status: SdfGradientStatus,
    /// Lipschitz evidence exposed by expression structure.
    pub lipschitz_status: SdfLipschitzStatus,
    /// Whether all retained parameters are exact rationals.
    pub all_parameters_exact: bool,
    /// Whether all retained parameters support a dyadic schedule.
    pub has_dyadic_schedule: bool,
    /// Whether all retained parameters share a denominator schedule.
    pub has_shared_denominator_schedule: bool,
}

impl SdfFacts {
    /// Build structural facts for a retained expression.
    pub fn from_expr(expr: &SdfExpr) -> Self {
        let mut params = Vec::new();
        let mut counts = FactCounts::default();
        collect_expr(expr, &mut params, &mut counts);
        let parameter_exact = Real::exact_set_facts(params.iter().copied());
        let domain_status = domain_status_expr(expr);
        let gradient_status = gradient_status_expr(expr);
        let lipschitz_status = lipschitz_status_expr(expr);
        Self {
            node_count: counts.node_count,
            primitive_count: counts.primitive_count,
            transform_count: counts.transform_count,
            all_parameters_exact: parameter_exact.all_exact_rational,
            has_dyadic_schedule: parameter_exact.has_dyadic_schedule(),
            has_shared_denominator_schedule: parameter_exact.has_shared_denominator_schedule(),
            parameter_exact,
            metric_status: expr.metric_status(),
            domain_status,
            gradient_status,
            lipschitz_status,
        }
    }
}

#[derive(Default)]
struct FactCounts {
    node_count: usize,
    primitive_count: usize,
    transform_count: usize,
}

fn collect_expr<'a>(expr: &'a SdfExpr, params: &mut Vec<&'a Real>, counts: &mut FactCounts) {
    counts.node_count += 1;
    match expr {
        SdfExpr::Constant(value) => params.push(value),
        SdfExpr::Coordinate(_) => {}
        SdfExpr::Linear {
            coefficients,
            offset,
        } => {
            params.extend([
                &coefficients.0[0],
                &coefficients.0[1],
                &coefficients.0[2],
                offset,
            ]);
        }
        SdfExpr::Primitive(primitive) => {
            counts.primitive_count += 1;
            collect_primitive(primitive, params);
        }
        SdfExpr::Union(left, right)
        | SdfExpr::Intersection(left, right)
        | SdfExpr::Add(left, right)
        | SdfExpr::Sub(left, right)
        | SdfExpr::Mul(left, right) => {
            collect_expr(left, params, counts);
            collect_expr(right, params, counts);
        }
        SdfExpr::Abs(inner) | SdfExpr::Sqrt(inner) => collect_expr(inner, params, counts),
        SdfExpr::Complement(inner) => collect_expr(inner, params, counts),
        SdfExpr::Offset { child, amount } => {
            params.push(amount);
            collect_expr(child, params, counts);
        }
        SdfExpr::Transform { child, transform } => {
            counts.transform_count += 1;
            params.extend(transform.parameters());
            collect_expr(child, params, counts);
        }
    }
}

fn collect_primitive<'a>(primitive: &'a SdfPrimitive, params: &mut Vec<&'a Real>) {
    match primitive {
        SdfPrimitive::Plane { plane } => {
            params.extend([
                &plane.normal.x,
                &plane.normal.y,
                &plane.normal.z,
                &plane.offset,
            ]);
        }
        SdfPrimitive::Sphere {
            center,
            radius_squared,
        } => {
            params.extend([&center.x, &center.y, &center.z, radius_squared]);
        }
        SdfPrimitive::Aabb { min, max } => {
            params.extend([&min.x, &min.y, &min.z, &max.x, &max.y, &max.z]);
        }
        SdfPrimitive::Cylinder {
            center,
            radius_squared,
            half_height,
            ..
        } => {
            params.extend([&center.x, &center.y, &center.z, radius_squared, half_height]);
        }
        SdfPrimitive::Capsule {
            center,
            radius_squared,
            half_length,
            ..
        } => {
            params.extend([&center.x, &center.y, &center.z, radius_squared, half_length]);
        }
        SdfPrimitive::Torus {
            center,
            major_radius_squared,
            minor_radius_squared,
            ..
        } => {
            params.extend([
                &center.x,
                &center.y,
                &center.z,
                major_radius_squared,
                minor_radius_squared,
            ]);
        }
        SdfPrimitive::Slab { plane, half_width } => {
            params.extend([
                &plane.normal.x,
                &plane.normal.y,
                &plane.normal.z,
                &plane.offset,
                half_width,
            ]);
        }
    }
}

fn domain_status_expr(expr: &SdfExpr) -> SdfDomainStatus {
    match expr {
        SdfExpr::Constant(_) | SdfExpr::Coordinate(_) | SdfExpr::Linear { .. } => {
            SdfDomainStatus::Valid
        }
        SdfExpr::Primitive(primitive) => domain_status_primitive(primitive),
        SdfExpr::Union(left, right)
        | SdfExpr::Intersection(left, right)
        | SdfExpr::Add(left, right)
        | SdfExpr::Sub(left, right)
        | SdfExpr::Mul(left, right) => domain_status_expr(left).combine(domain_status_expr(right)),
        SdfExpr::Abs(inner) | SdfExpr::Complement(inner) | SdfExpr::Offset { child: inner, .. } => {
            domain_status_expr(inner)
        }
        SdfExpr::Sqrt(inner) => domain_status_expr(inner).combine(SdfDomainStatus::Unknown),
        SdfExpr::Transform { child, .. } => domain_status_expr(child),
    }
}

fn domain_status_primitive(primitive: &SdfPrimitive) -> SdfDomainStatus {
    match primitive {
        SdfPrimitive::Sphere { radius_squared, .. } => {
            match crate::primitive::radius_squared_domain(radius_squared) {
                hyperlimit::PredicateOutcome::Decided { value: true, .. } => SdfDomainStatus::Valid,
                hyperlimit::PredicateOutcome::Decided { value: false, .. } => {
                    SdfDomainStatus::Invalid
                }
                hyperlimit::PredicateOutcome::Unknown { .. } => SdfDomainStatus::Unknown,
            }
        }
        SdfPrimitive::Cylinder {
            radius_squared,
            half_height,
            ..
        } => match crate::primitive::cylinder_domain(radius_squared, half_height) {
            hyperlimit::PredicateOutcome::Decided { value: true, .. } => SdfDomainStatus::Valid,
            hyperlimit::PredicateOutcome::Decided { value: false, .. } => SdfDomainStatus::Invalid,
            hyperlimit::PredicateOutcome::Unknown { .. } => SdfDomainStatus::Unknown,
        },
        SdfPrimitive::Capsule {
            radius_squared,
            half_length,
            ..
        } => match crate::primitive::capsule_domain(radius_squared, half_length) {
            hyperlimit::PredicateOutcome::Decided { value: true, .. } => SdfDomainStatus::Valid,
            hyperlimit::PredicateOutcome::Decided { value: false, .. } => SdfDomainStatus::Invalid,
            hyperlimit::PredicateOutcome::Unknown { .. } => SdfDomainStatus::Unknown,
        },
        SdfPrimitive::Torus {
            major_radius_squared,
            minor_radius_squared,
            ..
        } => match crate::primitive::torus_domain(major_radius_squared, minor_radius_squared) {
            hyperlimit::PredicateOutcome::Decided { value: true, .. } => SdfDomainStatus::Valid,
            hyperlimit::PredicateOutcome::Decided { value: false, .. } => SdfDomainStatus::Invalid,
            hyperlimit::PredicateOutcome::Unknown { .. } => SdfDomainStatus::Unknown,
        },
        SdfPrimitive::Slab { half_width, .. } => {
            match crate::primitive::half_width_domain(half_width) {
                hyperlimit::PredicateOutcome::Decided { value: true, .. } => SdfDomainStatus::Valid,
                hyperlimit::PredicateOutcome::Decided { value: false, .. } => {
                    SdfDomainStatus::Invalid
                }
                hyperlimit::PredicateOutcome::Unknown { .. } => SdfDomainStatus::Unknown,
            }
        }
        SdfPrimitive::Plane { .. } | SdfPrimitive::Aabb { .. } => SdfDomainStatus::Valid,
    }
}

fn gradient_status_expr(expr: &SdfExpr) -> SdfGradientStatus {
    match expr {
        SdfExpr::Constant(_) | SdfExpr::Coordinate(_) | SdfExpr::Linear { .. } => {
            SdfGradientStatus::ExactSymbolic
        }
        SdfExpr::Primitive(primitive) => gradient_status_primitive(primitive),
        SdfExpr::Union(left, right)
        | SdfExpr::Intersection(left, right)
        | SdfExpr::Add(left, right)
        | SdfExpr::Sub(left, right) => {
            gradient_status_expr(left).csg_pair(gradient_status_expr(right))
        }
        SdfExpr::Mul(_, _) | SdfExpr::Abs(_) | SdfExpr::Sqrt(_) => {
            SdfGradientStatus::PiecewiseExact
        }
        SdfExpr::Complement(inner) | SdfExpr::Offset { child: inner, .. } => {
            gradient_status_expr(inner)
        }
        SdfExpr::Transform { child, transform } => {
            transform.gradient_status(gradient_status_expr(child))
        }
    }
}

fn gradient_status_primitive(primitive: &SdfPrimitive) -> SdfGradientStatus {
    match primitive {
        SdfPrimitive::Plane { .. } | SdfPrimitive::Sphere { .. } => {
            SdfGradientStatus::ExactSymbolic
        }
        SdfPrimitive::Aabb { .. }
        | SdfPrimitive::Cylinder { .. }
        | SdfPrimitive::Capsule { .. }
        | SdfPrimitive::Torus { .. }
        | SdfPrimitive::Slab { .. } => SdfGradientStatus::PiecewiseExact,
    }
}

fn lipschitz_status_expr(expr: &SdfExpr) -> SdfLipschitzStatus {
    match expr {
        SdfExpr::Constant(_) | SdfExpr::Coordinate(_) => SdfLipschitzStatus::GlobalExact,
        SdfExpr::Linear { .. } => SdfLipschitzStatus::LocalOnly,
        SdfExpr::Primitive(primitive) => lipschitz_status_primitive(primitive),
        SdfExpr::Union(left, right)
        | SdfExpr::Intersection(left, right)
        | SdfExpr::Add(left, right)
        | SdfExpr::Sub(left, right) => {
            lipschitz_status_expr(left).csg_pair(lipschitz_status_expr(right))
        }
        SdfExpr::Mul(_, _) | SdfExpr::Abs(_) | SdfExpr::Sqrt(_) => SdfLipschitzStatus::LocalOnly,
        SdfExpr::Complement(inner) | SdfExpr::Offset { child: inner, .. } => {
            lipschitz_status_expr(inner)
        }
        SdfExpr::Transform { child, transform } => {
            transform.lipschitz_status(lipschitz_status_expr(child))
        }
    }
}

fn lipschitz_status_primitive(primitive: &SdfPrimitive) -> SdfLipschitzStatus {
    match primitive {
        SdfPrimitive::Aabb { .. }
        | SdfPrimitive::Cylinder { .. }
        | SdfPrimitive::Capsule { .. }
        | SdfPrimitive::Torus { .. }
        | SdfPrimitive::Slab { .. } => SdfLipschitzStatus::GlobalExact,
        SdfPrimitive::Sphere { .. } => SdfLipschitzStatus::LocalOnly,
        SdfPrimitive::Plane { .. } => SdfLipschitzStatus::Unknown,
    }
}
