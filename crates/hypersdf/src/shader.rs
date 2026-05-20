//! Preview shader export reports.
//!
//! Shader export is an adapter, not exact geometry. The generated GLSL-like
//! source lowers exact `Real` constants into primitive floats and preserves
//! only preview/evaluation intent. Sphere tracing and shader SDF practice are
//! useful display routes; see Hart, "Sphere Tracing: A Geometric Method for
//! the Antialiased Ray Tracing of Implicit Surfaces," *The Visual Computer*
//! 12.10 (1996). Topology still has to replay through exact Hyper predicates,
//! following Yap, "Towards Exact Geometric Computation," *Computational
//! Geometry* 7.1-2 (1997).

use hyperreal::Real;

use crate::expr::{SdfCoordinate, SdfExpr};
use crate::primitive::SdfPrimitive;
use crate::sampling::{SdfSampleTopologyStatus, SdfSamplingPrecision};
use crate::status::{SdfFreshness, SdfMetricStatus};
use crate::transform::SdfTransform;

/// Shader language requested by a preview export.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SdfShaderLanguage {
    /// GLSL-compatible scalar function over `vec3 p`.
    Glsl,
}

/// Report returned by shader export.
#[derive(Clone, Debug, PartialEq)]
pub struct SdfShaderExportReport {
    /// Target shader language.
    pub language: SdfShaderLanguage,
    /// Primitive scalar precision used when lowering constants.
    pub precision: SdfSamplingPrecision,
    /// Metric claim of the source expression before shader lowering.
    pub metric_status: SdfMetricStatus,
    /// Prepared-source freshness.
    pub freshness: SdfFreshness,
    /// Whether the generated source may be consumed as topology evidence.
    pub topology_status: SdfSampleTopologyStatus,
    /// Number of exact constants that could not be lowered to finite floats.
    pub non_finite_constant_count: usize,
    /// Unsupported node descriptions encountered during export.
    pub unsupported_nodes: Vec<String>,
    /// Generated shader source when every node and constant is supported.
    pub source: Option<String>,
}

impl SdfShaderExportReport {
    /// Returns whether complete preview shader source was emitted.
    pub fn is_complete(&self) -> bool {
        self.source.is_some()
            && self.non_finite_constant_count == 0
            && self.unsupported_nodes.is_empty()
    }
}

pub(crate) fn export_expr_glsl_preview(
    expr: &SdfExpr,
    function_name: &str,
    precision: SdfSamplingPrecision,
    metric_status: SdfMetricStatus,
    freshness: SdfFreshness,
) -> SdfShaderExportReport {
    let mut context = ShaderExportContext {
        precision,
        non_finite_constant_count: 0,
        unsupported_nodes: Vec::new(),
    };

    let valid_name = valid_glsl_identifier(function_name);
    if !valid_name {
        context.unsupported_nodes.push(format!(
            "invalid GLSL function identifier `{function_name}`"
        ));
    }

    let body = emit_expr(expr, "p", &mut context);
    let source = if valid_name
        && context.non_finite_constant_count == 0
        && context.unsupported_nodes.is_empty()
    {
        body.map(|body| format!("float {function_name}(vec3 p) {{\n    return {body};\n}}\n"))
    } else {
        None
    };

    SdfShaderExportReport {
        language: SdfShaderLanguage::Glsl,
        precision,
        metric_status,
        freshness,
        topology_status: SdfSampleTopologyStatus::PreviewOnly,
        non_finite_constant_count: context.non_finite_constant_count,
        unsupported_nodes: context.unsupported_nodes,
        source,
    }
}

struct ShaderExportContext {
    precision: SdfSamplingPrecision,
    non_finite_constant_count: usize,
    unsupported_nodes: Vec<String>,
}

fn emit_expr(
    expr: &SdfExpr,
    point_name: &str,
    context: &mut ShaderExportContext,
) -> Option<String> {
    match expr {
        SdfExpr::Constant(value) => emit_real(value, context),
        SdfExpr::Coordinate(axis) => Some(format!(
            "{point_name}.{}",
            match axis {
                SdfCoordinate::X => "x",
                SdfCoordinate::Y => "y",
                SdfCoordinate::Z => "z",
            }
        )),
        SdfExpr::Linear {
            coefficients,
            offset,
        } => Some(format!(
            "dot({}, {}) + {}",
            emit_lattice_vec3(coefficients, context)?,
            point_name,
            emit_real(offset, context)?
        )),
        SdfExpr::Primitive(primitive) => emit_primitive(primitive, point_name, context),
        SdfExpr::Union(left, right) => Some(format!(
            "min({}, {})",
            emit_expr(left, point_name, context)?,
            emit_expr(right, point_name, context)?
        )),
        SdfExpr::Intersection(left, right) => Some(format!(
            "max({}, {})",
            emit_expr(left, point_name, context)?,
            emit_expr(right, point_name, context)?
        )),
        SdfExpr::Add(left, right) => Some(format!(
            "({}) + ({})",
            emit_expr(left, point_name, context)?,
            emit_expr(right, point_name, context)?
        )),
        SdfExpr::Sub(left, right) => Some(format!(
            "({}) - ({})",
            emit_expr(left, point_name, context)?,
            emit_expr(right, point_name, context)?
        )),
        SdfExpr::Mul(left, right) => Some(format!(
            "({}) * ({})",
            emit_expr(left, point_name, context)?,
            emit_expr(right, point_name, context)?
        )),
        SdfExpr::Abs(inner) => Some(format!("abs({})", emit_expr(inner, point_name, context)?)),
        SdfExpr::Sqrt(inner) => Some(format!("sqrt({})", emit_expr(inner, point_name, context)?)),
        SdfExpr::Complement(inner) => {
            Some(format!("-({})", emit_expr(inner, point_name, context)?))
        }
        SdfExpr::Offset { child, amount } => Some(format!(
            "({}) - {}",
            emit_expr(child, point_name, context)?,
            emit_real(amount, context)?
        )),
        SdfExpr::Transform { child, transform } => match transform {
            SdfTransform::Translation { offset } => Some(format!(
                "({})",
                emit_expr(
                    child,
                    &format!("({point_name} - {})", emit_vec3(offset, context)?),
                    context
                )?
            )),
            SdfTransform::Affine { inverse, .. } => Some(format!(
                "({})",
                emit_expr(
                    child,
                    &format!("({})", emit_affine_point(inverse, point_name, context)?),
                    context
                )?
            )),
        },
    }
}

fn emit_primitive(
    primitive: &SdfPrimitive,
    point_name: &str,
    context: &mut ShaderExportContext,
) -> Option<String> {
    match primitive {
        SdfPrimitive::Plane { plane } => Some(format!(
            "dot({}, {}) + {}",
            emit_vec3(&plane.normal, context)?,
            point_name,
            emit_real(&plane.offset, context)?
        )),
        SdfPrimitive::Sphere {
            center,
            radius_squared,
        } => Some(format!(
            "dot({p} - {c}, {p} - {c}) - {r2}",
            p = point_name,
            c = emit_vec3(center, context)?,
            r2 = emit_real(radius_squared, context)?
        )),
        SdfPrimitive::Aabb { min, max } => Some(format!(
            "max(max(max({min_x} - {p}.x, {p}.x - {max_x}), max({min_y} - {p}.y, {p}.y - {max_y})), max({min_z} - {p}.z, {p}.z - {max_z}))",
            p = point_name,
            min_x = emit_real(&min.x, context)?,
            min_y = emit_real(&min.y, context)?,
            min_z = emit_real(&min.z, context)?,
            max_x = emit_real(&max.x, context)?,
            max_y = emit_real(&max.y, context)?,
            max_z = emit_real(&max.z, context)?,
        )),
        SdfPrimitive::Cylinder {
            axis,
            center,
            radius_squared,
            half_height,
        } => {
            let (a, b, ca, cb, axis_component, center_axis) = match axis {
                SdfCoordinate::X => ("y", "z", &center.y, &center.z, "x", &center.x),
                SdfCoordinate::Y => ("x", "z", &center.x, &center.z, "y", &center.y),
                SdfCoordinate::Z => ("x", "y", &center.x, &center.y, "z", &center.z),
            };
            Some(format!(
                "max(dot(vec2({p}.{a} - {ca}, {p}.{b} - {cb}), vec2({p}.{a} - {ca}, {p}.{b} - {cb})) - {r2}, abs({p}.{axis_component} - {center_axis}) - {half_height})",
                p = point_name,
                ca = emit_real(ca, context)?,
                cb = emit_real(cb, context)?,
                r2 = emit_real(radius_squared, context)?,
                center_axis = emit_real(center_axis, context)?,
                half_height = emit_real(half_height, context)?,
            ))
        }
        SdfPrimitive::Capsule {
            axis,
            center,
            radius_squared,
            half_length,
        } => {
            let (a, b, ca, cb, axis_component, center_axis) = match axis {
                SdfCoordinate::X => ("y", "z", &center.y, &center.z, "x", &center.x),
                SdfCoordinate::Y => ("x", "z", &center.x, &center.z, "y", &center.y),
                SdfCoordinate::Z => ("x", "y", &center.x, &center.y, "z", &center.z),
            };
            Some(format!(
                "dot(vec2({p}.{a} - {ca}, {p}.{b} - {cb}), vec2({p}.{a} - {ca}, {p}.{b} - {cb})) + pow(max(abs({p}.{axis_component} - {center_axis}) - {half_length}, 0.0), 2.0) - {r2}",
                p = point_name,
                ca = emit_real(ca, context)?,
                cb = emit_real(cb, context)?,
                center_axis = emit_real(center_axis, context)?,
                half_length = emit_real(half_length, context)?,
                r2 = emit_real(radius_squared, context)?,
            ))
        }
        SdfPrimitive::Torus {
            axis,
            center,
            major_radius_squared,
            minor_radius_squared,
        } => {
            let (a, b, ca, cb, axis_component, center_axis) = match axis {
                SdfCoordinate::X => ("y", "z", &center.y, &center.z, "x", &center.x),
                SdfCoordinate::Y => ("x", "z", &center.x, &center.z, "y", &center.y),
                SdfCoordinate::Z => ("x", "y", &center.x, &center.y, "z", &center.z),
            };
            Some(format!(
                "pow(dot(vec2({p}.{a} - {ca}, {p}.{b} - {cb}), vec2({p}.{a} - {ca}, {p}.{b} - {cb})) + pow({p}.{axis_component} - {center_axis}, 2.0) + {major_r2} - {minor_r2}, 2.0) - 4.0 * {major_r2} * dot(vec2({p}.{a} - {ca}, {p}.{b} - {cb}), vec2({p}.{a} - {ca}, {p}.{b} - {cb}))",
                p = point_name,
                ca = emit_real(ca, context)?,
                cb = emit_real(cb, context)?,
                center_axis = emit_real(center_axis, context)?,
                major_r2 = emit_real(major_radius_squared, context)?,
                minor_r2 = emit_real(minor_radius_squared, context)?,
            ))
        }
        SdfPrimitive::Slab { plane, half_width } => Some(format!(
            "abs(dot({}, {}) + {}) - {}",
            emit_vec3(&plane.normal, context)?,
            point_name,
            emit_real(&plane.offset, context)?,
            emit_real(half_width, context)?
        )),
    }
}

fn emit_vec3(point: &hyperlimit::Point3, context: &mut ShaderExportContext) -> Option<String> {
    Some(format!(
        "vec3({}, {}, {})",
        emit_real(&point.x, context)?,
        emit_real(&point.y, context)?,
        emit_real(&point.z, context)?
    ))
}

fn emit_lattice_vec3(
    vector: &hyperlattice::Vector3,
    context: &mut ShaderExportContext,
) -> Option<String> {
    Some(format!(
        "vec3({}, {}, {})",
        emit_real(&vector.0[0], context)?,
        emit_real(&vector.0[1], context)?,
        emit_real(&vector.0[2], context)?
    ))
}

fn emit_affine_point(
    matrix: &hyperlattice::Matrix4,
    point_name: &str,
    context: &mut ShaderExportContext,
) -> Option<String> {
    let row = |row: usize, context: &mut ShaderExportContext| {
        Some(format!(
            "dot(vec3({}, {}, {}), {}) + {}",
            emit_real(&matrix.0[row][0], context)?,
            emit_real(&matrix.0[row][1], context)?,
            emit_real(&matrix.0[row][2], context)?,
            point_name,
            emit_real(&matrix.0[row][3], context)?
        ))
    };
    Some(format!(
        "vec3({}, {}, {})",
        row(0, context)?,
        row(1, context)?,
        row(2, context)?
    ))
}

fn emit_real(value: &Real, context: &mut ShaderExportContext) -> Option<String> {
    let lowered = match context.precision {
        SdfSamplingPrecision::F32 => value.to_f32_lossy().map(f64::from),
        SdfSamplingPrecision::F64 => value.to_f64_lossy(),
    };
    let Some(value) = lowered else {
        context.non_finite_constant_count += 1;
        return None;
    };
    Some(format_float(value))
}

fn format_float(value: f64) -> String {
    let mut text = format!("{value:.9}");
    while text.contains('.') && text.ends_with('0') {
        text.pop();
    }
    if text.ends_with('.') {
        text.push('0');
    }
    if !text.contains('.') {
        text.push_str(".0");
    }
    text
}

fn valid_glsl_identifier(name: &str) -> bool {
    let mut chars = name.chars();
    let Some(first) = chars.next() else {
        return false;
    };
    (first == '_' || first.is_ascii_alphabetic())
        && chars.all(|ch| ch == '_' || ch.is_ascii_alphanumeric())
}
