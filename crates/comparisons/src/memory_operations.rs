//! Retained Hyper fixtures for every operation in the paired benchmark catalog.
//!
//! The allocator-instrumented executable uses these fixtures together with
//! [`crate::PreparedBenchmark`]. Keeping the IDs explicit makes memory coverage
//! mechanically checkable against the 160-case retained latency campaign.

use std::cmp::Ordering;
use std::hint::black_box;

use hypercurve::{CircularArc2, LineSeg2, Point2 as CurvePoint2, Segment2};
use hyperlattice::{Complex, HomogeneousPoint3, Matrix3, Matrix4, Vector2, Vector3, Vector4};
use hyperlimit::{
    Plane3, Point2, Point3, PredicatePolicy, classify_circle_line2, classify_circle_segment2,
    classify_plane_segment, classify_point_line, classify_point_plane, classify_point_segment,
    classify_point_segment3, classify_point_triangle3, classify_segment_intersection,
    classify_segment_triangle3_intersection, classify_segment3_intersection,
    classify_triangle_triangle3, construct_line_intersection_point, incircle2, orient2,
    orient2d_value, orient3, point_plane_value,
};
use hypermesh::{ConvexPolygon, MeshContext, Plane, convex_triangle, intersect_polygons};
use hyperpath::{ArcDirection, ExplicitCircularArc, LinePathSegment};
use hyperreal::{Rational, Real};
use hypersolve::{
    BivariatePolynomial, CurveIntersectionResultantConfig, CurveResultantParameter, Expr, Problem,
    SymbolId, isolate_univariate_polynomial_expr, resultant_bivariate_polynomial_system,
    resultant_univariate_polynomials, square_free_part, subresultant_chain_univariate_polynomials,
};
use hypertri::{PointD, TriangulationContext};

use crate::{ComplexBinary, ExprBinary, ExprUnary, RationalBinary, RationalUnary};

const POLICY: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;
const TRI_CONTEXT: TriangulationContext = TriangulationContext::new(POLICY);
const MESH_CONTEXT: MeshContext = MeshContext::new(POLICY);

#[cfg(test)]
use crate::COMPARABLE_OPERATION_IDS;

type Runner = Box<dyn FnMut()>;

/// One retained Hyper input fixture and its public operation.
pub struct HyperMemoryOperation {
    runner: Runner,
}

impl HyperMemoryOperation {
    /// Builds the retained fixture for a Criterion operation ID.
    pub fn new(id: &str) -> Result<Self, String> {
        let runner = rational_runner(id)
            .or_else(|| real_runner(id))
            .or_else(|| complex_runner(id))
            .or_else(|| vector_runner(id))
            .or_else(|| matrix_runner(id))
            .or_else(|| geometry2_runner(id))
            .or_else(|| geometry3_runner(id))
            .or_else(|| polynomial_runner(id))
            .or_else(|| bivariate_runner(id))
            .or_else(|| triangulation_runner(id))
            .or_else(|| curve_runner(id))
            .or_else(|| mesh_runner(id))
            .or_else(|| path_runner(id))
            .ok_or_else(|| format!("unknown comparable operation {id}"))?;
        Ok(Self { runner })
    }

    /// Runs the retained operation without reconstructing its inputs.
    pub fn run_iterations(&mut self, iterations: u64) {
        for _ in 0..iterations {
            (self.runner)();
        }
    }
}

fn rational(text: &str) -> Rational {
    text.parse().expect("fixed rational fixture")
}

fn real(text: &str) -> Real {
    Real::new(rational(text))
}

fn r(value: i64) -> Real {
    Real::from(value)
}

fn p2(x: i64, y: i64) -> Point2 {
    Point2::new(r(x), r(y))
}

fn p3(x: i64, y: i64, z: i64) -> Point3 {
    Point3::new(r(x), r(y), r(z))
}

fn curve_point(x: i64, y: i64) -> CurvePoint2 {
    CurvePoint2::new(r(x), r(y))
}

fn indexed_suffix(id: &str, prefix: &str, maximum: u8) -> Option<u8> {
    let suffix = id.strip_prefix(prefix)?;
    let value = suffix.parse::<u8>().ok()?;
    (value <= maximum).then_some(value)
}

fn rational_runner(id: &str) -> Option<Runner> {
    let binary = match id {
        "rational.add" => Some(RationalBinary::Add),
        "rational.subtract" => Some(RationalBinary::Subtract),
        "rational.multiply" => Some(RationalBinary::Multiply),
        "rational.divide" => Some(RationalBinary::Divide),
        _ => None,
    };
    if let Some(operation) = binary {
        let left = rational("123456789/1000003");
        let right = rational("-987654321/1000033");
        return Some(Box::new(move || match operation {
            RationalBinary::Add => drop(black_box(&left + &right)),
            RationalBinary::Subtract => drop(black_box(&left - &right)),
            RationalBinary::Multiply => drop(black_box(&left * &right)),
            RationalBinary::Divide => drop(black_box(&left / &right)),
        }));
    }

    let unary = match id {
        "rational.negate" => Some(RationalUnary::Negate),
        "rational.absolute" => Some(RationalUnary::Absolute),
        "rational.reciprocal" => Some(RationalUnary::Reciprocal),
        _ => None,
    };
    if let Some(operation) = unary {
        let value = rational("123456789/1000003");
        return Some(Box::new(move || match operation {
            RationalUnary::Negate => drop(black_box(-&value)),
            RationalUnary::Absolute => {
                let output = if value.is_negative() {
                    -&value
                } else {
                    value.clone()
                };
                drop(black_box(output));
            }
            RationalUnary::Reciprocal => drop(black_box(value.clone().inverse().unwrap())),
        }));
    }

    match id {
        "rational.compare" => {
            let left = rational("123456789/1000003");
            let right = rational("-987654321/1000033");
            Some(Box::new(move || {
                black_box(left.partial_cmp(black_box(&right)));
            }))
        }
        "rational.parts" => {
            let value = rational("123456789/1000003");
            Some(Box::new(move || {
                black_box((value.numerator().clone(), value.denominator().clone()));
            }))
        }
        _ => None,
    }
}

fn real_runner(id: &str) -> Option<Runner> {
    let unary = match id {
        "real.negate" => Some(ExprUnary::Negate),
        "real.absolute" => Some(ExprUnary::Absolute),
        "real.sqrt" => Some(ExprUnary::Sqrt),
        "real.cbrt" => Some(ExprUnary::Cbrt),
        "real.root_n" => Some(ExprUnary::RootN),
        "real.exp" => Some(ExprUnary::Exp),
        "real.ln" => Some(ExprUnary::Ln),
        "real.log2" => Some(ExprUnary::Log2),
        "real.log10" => Some(ExprUnary::Log10),
        "real.sin" => Some(ExprUnary::Sin),
        "real.cos" => Some(ExprUnary::Cos),
        "real.tan" => Some(ExprUnary::Tan),
        "real.asin" => Some(ExprUnary::Asin),
        "real.acos" => Some(ExprUnary::Acos),
        "real.atan" => Some(ExprUnary::Atan),
        "real.square" => Some(ExprUnary::Square),
        "real.exp2" => Some(ExprUnary::Exp2),
        "real.exp10" => Some(ExprUnary::Exp10),
        "real.cot" => Some(ExprUnary::Cot),
        _ => None,
    };
    if let Some(operation) = unary {
        let input = real("1/3");
        return Some(Box::new(move || match operation {
            ExprUnary::Negate => drop(black_box(-input.clone())),
            ExprUnary::Absolute => drop(black_box(input.clone().abs())),
            ExprUnary::Sqrt => drop(black_box(input.clone().sqrt().unwrap())),
            ExprUnary::Cbrt => drop(black_box(input.clone().cbrt().unwrap())),
            ExprUnary::RootN => drop(black_box(input.clone().root_n(5).unwrap())),
            ExprUnary::Exp => drop(black_box(input.clone().exp().unwrap())),
            ExprUnary::Ln => drop(black_box(input.clone().ln().unwrap())),
            ExprUnary::Log2 => drop(black_box(input.clone().log2().unwrap())),
            ExprUnary::Log10 => drop(black_box(input.clone().log10().unwrap())),
            ExprUnary::Sin => drop(black_box(input.clone().sin())),
            ExprUnary::Cos => drop(black_box(input.clone().cos())),
            ExprUnary::Tan => drop(black_box(input.clone().tan().unwrap())),
            ExprUnary::Asin => drop(black_box(input.clone().asin().unwrap())),
            ExprUnary::Acos => drop(black_box(input.clone().acos().unwrap())),
            ExprUnary::Atan => drop(black_box(input.clone().atan().unwrap())),
            ExprUnary::Square => drop(black_box(input.clone().powi_i64(2).unwrap())),
            ExprUnary::Exp2 => drop(black_box(r(2).pow(input.clone()).unwrap())),
            ExprUnary::Exp10 => drop(black_box(r(10).pow(input.clone()).unwrap())),
            ExprUnary::Cot => drop(black_box(
                (input.clone().cos() / input.clone().sin()).unwrap(),
            )),
        }));
    }

    let binary = match id {
        "real.add" => Some(ExprBinary::Add),
        "real.subtract" => Some(ExprBinary::Subtract),
        "real.multiply" => Some(ExprBinary::Multiply),
        "real.divide" => Some(ExprBinary::Divide),
        "real.powi" => Some(ExprBinary::IntegerPower),
        _ => None,
    };
    if let Some(operation) = binary {
        let left = real("7/3");
        let right = real("5/11");
        return Some(Box::new(move || match operation {
            ExprBinary::Add => drop(black_box(left.clone() + right.clone())),
            ExprBinary::Subtract => drop(black_box(left.clone() - right.clone())),
            ExprBinary::Multiply => drop(black_box(left.clone() * right.clone())),
            ExprBinary::Divide => drop(black_box((left.clone() / right.clone()).unwrap())),
            ExprBinary::IntegerPower => drop(black_box(left.clone().powi_i64(7).unwrap())),
        }));
    }

    match id {
        "real.pi" => Some(Box::new(|| drop(black_box(Real::pi())))),
        "real.e" => Some(Box::new(|| drop(black_box(Real::e())))),
        "real.sign" => {
            let input = real("1/3");
            Some(Box::new(move || {
                black_box(input.clone().sin().refine_sign_until(-128));
            }))
        }
        "real.floor" => {
            let input = real("7/3");
            Some(Box::new(move || {
                drop(black_box(input.floor_certified().unwrap()));
            }))
        }
        "real.ceil" => {
            let input = real("7/3");
            Some(Box::new(move || {
                drop(black_box(input.ceil_certified().unwrap()));
            }))
        }
        _ => None,
    }
}

fn complex_runner(id: &str) -> Option<Runner> {
    let binary = match id {
        "complex.add" => Some(ComplexBinary::Add),
        "complex.subtract" => Some(ComplexBinary::Subtract),
        "complex.multiply" => Some(ComplexBinary::Multiply),
        "complex.divide" => Some(ComplexBinary::Divide),
        _ => None,
    };
    if let Some(operation) = binary {
        let left = Complex::new(real("7/13"), real("-11/17"));
        let right = Complex::new(real("19/23"), real("29/31"));
        return Some(Box::new(move || match operation {
            ComplexBinary::Add => drop(black_box(&left + &right)),
            ComplexBinary::Subtract => drop(black_box(&left - &right)),
            ComplexBinary::Multiply => drop(black_box(&left * &right)),
            ComplexBinary::Divide => drop(black_box((&left / &right).unwrap())),
        }));
    }
    if id == "complex.norm_squared" {
        let value = Complex::new(real("7/13"), real("-11/17"));
        return Some(Box::new(move || {
            drop(black_box(value.norm_squared()));
        }));
    }
    None
}

fn vector_runner(id: &str) -> Option<Runner> {
    let operation = id.split_once('.')?.1.to_owned();
    match id.split_once('.')?.0 {
        "vector2" => {
            let left = Vector2::new([r(3), r(-4)]);
            let right = Vector2::new([r(7), r(11)]);
            Some(Box::new(move || match operation.as_str() {
                "add" => drop(black_box(&left + &right)),
                "subtract" => drop(black_box(&left - &right)),
                "dot" => drop(black_box(left.dot(&right))),
                "norm" => drop(black_box(left.norm())),
                "wedge" => drop(black_box(left.wedge(&right))),
                _ => unreachable!(),
            }))
        }
        "vector3" => {
            let left = Vector3::new([r(3), r(-4), r(5)]);
            let right = Vector3::new([r(7), r(11), r(-13)]);
            Some(Box::new(move || match operation.as_str() {
                "add" => drop(black_box(&left + &right)),
                "subtract" => drop(black_box(&left - &right)),
                "dot" => drop(black_box(left.dot(&right))),
                "cross" => drop(black_box(left.cross(&right))),
                "norm" => drop(black_box(left.norm())),
                _ => unreachable!(),
            }))
        }
        "vector4" => {
            let left = Vector4::new([r(3), r(-4), r(5), r(-6)]);
            let right = Vector4::new([r(7), r(11), r(-13), r(17)]);
            Some(Box::new(move || match operation.as_str() {
                "add" => drop(black_box(&left + &right)),
                "subtract" => drop(black_box(&left - &right)),
                "dot" => drop(black_box(left.dot(&right))),
                "norm" => drop(black_box(left.norm())),
                _ => unreachable!(),
            }))
        }
        _ => None,
    }
}

fn matrix3(values: [i64; 9]) -> Matrix3 {
    Matrix3::new([
        [r(values[0]), r(values[1]), r(values[2])],
        [r(values[3]), r(values[4]), r(values[5])],
        [r(values[6]), r(values[7]), r(values[8])],
    ])
}

fn matrix4(values: [i64; 16]) -> Matrix4 {
    Matrix4::new([
        [r(values[0]), r(values[1]), r(values[2]), r(values[3])],
        [r(values[4]), r(values[5]), r(values[6]), r(values[7])],
        [r(values[8]), r(values[9]), r(values[10]), r(values[11])],
        [r(values[12]), r(values[13]), r(values[14]), r(values[15])],
    ])
}

fn matrix_runner(id: &str) -> Option<Runner> {
    let (prefix, operation) = id.split_once('.')?;
    let operation = operation.to_owned();
    match prefix {
        "matrix3" => {
            let left = matrix3([2, -1, 3, 4, 0, 5, -2, 7, 1]);
            let right = matrix3([1, 2, 0, -3, 4, 1, 5, -2, 6]);
            Some(Box::new(move || match operation.as_str() {
                "add" => drop(black_box(&left + &right)),
                "subtract" => drop(black_box(&left - &right)),
                "multiply" => drop(black_box(&left * &right)),
                "transpose" => drop(black_box(left.transpose())),
                "determinant" => drop(black_box(left.determinant())),
                "inverse" => drop(black_box(left.clone().inverse())),
                _ => unreachable!(),
            }))
        }
        "matrix4" => {
            let left = matrix4([2, 1, 0, 3, -1, 4, 2, 0, 5, 0, 3, 1, 2, -2, 1, 6]);
            let right = matrix4([1, 0, 2, -1, 3, 1, 0, 4, -2, 5, 1, 0, 0, 2, 3, 1]);
            Some(Box::new(move || match operation.as_str() {
                "add" => drop(black_box(&left + &right)),
                "subtract" => drop(black_box(&left - &right)),
                "multiply" => drop(black_box(&left * &right)),
                "transpose" => drop(black_box(left.transpose())),
                "determinant" => drop(black_box(left.determinant())),
                "inverse" => drop(black_box(left.clone().inverse())),
                _ => unreachable!(),
            }))
        }
        _ => None,
    }
}

fn geometry2_runner(id: &str) -> Option<Runner> {
    match id {
        "geometry2.orientation" => {
            let a = p2(0, 0);
            let b = p2(13, 2);
            let query = p2(3, 17);
            return Some(Box::new(move || {
                black_box(orient2(&a, &b, &query, POLICY));
            }));
        }
        "geometry2.area" => {
            let a = p2(0, 0);
            let b = p2(13, 2);
            let query = p2(3, 17);
            return Some(Box::new(move || {
                drop(black_box(orient2d_value(&a, &b, &query)));
            }));
        }
        "geometry2.between" => {
            let a = p2(0, 0);
            let b = p2(13, 2);
            let query = p2(6, 1);
            return Some(Box::new(move || {
                let _ = black_box(classify_point_segment(&a, &b, &query, POLICY));
            }));
        }
        _ => {}
    }

    if let Some(operation) = indexed_suffix(id, "geometry2.line_relation_", 6) {
        let a = p2(0, 0);
        let b = p2(13, 2);
        let c = p2(3, 17);
        let d = p2(19, -7);
        return Some(Box::new(move || match operation {
            0 | 1 => drop(black_box(classify_point_line(&a, &b, &c, POLICY))),
            2..=4 => {
                let ab = &b - &a;
                let cd = &d - &c;
                drop(black_box((ab.wedge(&cd), orient2(&a, &b, &c, POLICY))));
            }
            5 => {
                black_box(a.x == b.x);
            }
            6 => {
                black_box(a.y == b.y);
            }
            _ => unreachable!(),
        }));
    }

    if id == "geometry2.line_intersection" {
        let a = p2(0, 0);
        let b = p2(13, 2);
        let c = p2(3, 17);
        let d = p2(19, -7);
        return Some(Box::new(move || {
            drop(black_box(construct_line_intersection_point(&a, &b, &c, &d)));
        }));
    }

    if let Some(operation) = indexed_suffix(id, "geometry2.segment_relation_", 3) {
        let a = p2(0, 0);
        let b = p2(13, 2);
        let c = p2(3, 17);
        let d = p2(19, -7);
        return Some(Box::new(move || match operation {
            0 => drop(black_box(classify_point_segment(&a, &b, &c, POLICY))),
            1 | 2 => drop(black_box(classify_segment_intersection(
                &a, &b, &c, &d, POLICY,
            ))),
            3 => drop(black_box((&b - &a).wedge(&(&d - &c)))),
            _ => unreachable!(),
        }));
    }

    match id {
        "geometry2.incircle" => {
            let a = p2(0, 0);
            let b = p2(13, 2);
            let c = p2(3, 17);
            let query = p2(4, 4);
            Some(Box::new(move || {
                black_box(incircle2(&a, &b, &c, &query, POLICY));
            }))
        }
        "geometry2.circle_line" => {
            let center = p2(0, 0);
            let radius_squared = r(25);
            let a = p2(-10, 7);
            let b = p2(10, 7);
            Some(Box::new(move || {
                let _ = black_box(classify_circle_line2(
                    &center,
                    &radius_squared,
                    &a,
                    &b,
                    POLICY,
                ));
            }))
        }
        "geometry2.circle_segment" => {
            let center = p2(0, 0);
            let radius_squared = r(25);
            let a = p2(-10, 7);
            let b = p2(10, 7);
            Some(Box::new(move || {
                let _ = black_box(classify_circle_segment2(
                    &center,
                    &radius_squared,
                    &a,
                    &b,
                    POLICY,
                ));
            }))
        }
        "geometry2.circle_point_distance" => {
            let center = p2(0, 0);
            let query = p2(13, 7);
            let radius = r(5);
            Some(Box::new(move || {
                drop(black_box((&query - &center).norm() - radius.clone()));
            }))
        }
        "geometry2.circle_circle_distance" => {
            let first = p2(0, 0);
            let second = p2(17, 4);
            let radii = r(12);
            Some(Box::new(move || {
                let separation = (&second - &first).norm() - radii.clone();
                let output = if separation.partial_cmp(&Real::zero()) == Some(Ordering::Greater) {
                    separation
                } else {
                    Real::zero()
                };
                drop(black_box(output));
            }))
        }
        "geometry2.point_distance" => {
            let a = p2(0, 0);
            let b = p2(3, 4);
            Some(Box::new(move || {
                drop(black_box((&a - &b).norm()));
            }))
        }
        "geometry2.line_point_distance" => {
            let a = p2(0, 0);
            let b = p2(13, 2);
            let query = p2(3, 17);
            Some(Box::new(move || {
                let direction = &b - &a;
                drop(black_box(
                    (direction.wedge(&(&query - &a)).abs() / direction.norm()).unwrap(),
                ));
            }))
        }
        "geometry2.segment_point_distance" => {
            let a = p2(0, 0);
            let b = p2(13, 2);
            let query = p2(3, 17);
            Some(Box::new(move || {
                let direction = &b - &a;
                let offset = &query - &a;
                let projection = direction.dot(&offset);
                let output = if projection.partial_cmp(&Real::zero()) != Some(Ordering::Greater) {
                    offset.norm()
                } else if projection.partial_cmp(&direction.norm_squared()) != Some(Ordering::Less)
                {
                    (&query - &b).norm()
                } else {
                    (direction.wedge(&offset).abs() / direction.norm()).unwrap()
                };
                drop(black_box(output));
            }))
        }
        _ => None,
    }
}

fn geometry3_runner(id: &str) -> Option<Runner> {
    match id {
        "geometry3.orientation" => {
            let a = p3(0, 0, 0);
            let b = p3(13, 2, 1);
            let c = p3(3, 17, -2);
            let d = p3(4, 5, 19);
            return Some(Box::new(move || {
                black_box(orient3(&a, &b, &c, &d, POLICY));
            }));
        }
        "geometry3.volume" => {
            let a = p3(0, 0, 0);
            let b = p3(13, 2, 1);
            let c = p3(3, 17, -2);
            let d = p3(4, 5, 19);
            return Some(Box::new(move || {
                let rows = [
                    [&a.x - &d.x, &a.y - &d.y, &a.z - &d.z],
                    [&b.x - &d.x, &b.y - &d.y, &b.z - &d.z],
                    [&c.x - &d.x, &c.y - &d.y, &c.z - &d.z],
                ];
                drop(black_box(Matrix3::new(rows).determinant()));
            }));
        }
        _ => {}
    }

    if let Some(operation) = indexed_suffix(id, "geometry3.line_relation_", 4) {
        let a = p3(0, 0, 0);
        let b = p3(13, 2, 1);
        let c = p3(3, 17, -2);
        let d = p3(4, 5, 19);
        return Some(Box::new(move || match operation {
            0 => drop(black_box(classify_point_segment3(&a, &b, &c, POLICY))),
            1 | 4 => drop(black_box((&b - &a).cross(&(&d - &c)).norm_squared())),
            2 | 3 => {
                black_box(orient3(&a, &b, &c, &d, POLICY));
            }
            _ => unreachable!(),
        }));
    }

    if let Some(operation) = indexed_suffix(id, "geometry3.segment_relation_", 3) {
        let a = p3(0, 0, 0);
        let b = p3(13, 2, 1);
        let c = p3(3, 17, -2);
        let d = p3(4, 5, 19);
        return Some(Box::new(move || match operation {
            0 => drop(black_box(classify_point_segment3(&a, &b, &c, POLICY))),
            1 | 2 => drop(black_box(classify_segment3_intersection(
                &a, &b, &c, &d, POLICY,
            ))),
            3 => {
                black_box(orient3(&a, &b, &c, &d, POLICY));
            }
            _ => unreachable!(),
        }));
    }

    if let Some(operation) = indexed_suffix(id, "geometry3.plane_relation_", 7) {
        let c = p3(3, 17, -2);
        let d = p3(4, 5, 19);
        let query = p3(0, 0, 7);
        let plane = Plane3 {
            normal: p3(0, 0, 1),
            offset: r(0),
        };
        return Some(Box::new(move || match operation {
            0 | 1 => drop(black_box(classify_point_plane(&c, &plane, POLICY))),
            2..=5 => drop(black_box(classify_plane_segment(&plane, &c, &d, POLICY))),
            6 | 7 => drop(black_box(point_plane_value(&plane, &query))),
            _ => unreachable!(),
        }));
    }

    if let Some(operation) = indexed_suffix(id, "geometry3.triangle_relation_", 6) {
        let a = p3(0, 0, 0);
        let b = p3(13, 0, 0);
        let c = p3(0, 17, 0);
        let query = p3(3, 4, -2);
        let segment_end = p3(3, 4, 19);
        let other = p3(19, 19, 3);
        return Some(Box::new(move || match operation {
            0..=3 => drop(black_box(classify_point_triangle3(
                &a, &b, &c, &query, POLICY,
            ))),
            4 | 5 => drop(black_box(classify_segment_triangle3_intersection(
                &query,
                &segment_end,
                &a,
                &b,
                &c,
                POLICY,
            ))),
            6 => drop(black_box(classify_triangle_triangle3(
                &a,
                &b,
                &c,
                &query,
                &segment_end,
                &other,
                POLICY,
            ))),
            _ => unreachable!(),
        }));
    }

    match id {
        "geometry3.point_distance" => {
            let a = p3(0, 0, 0);
            let b = p3(3, 4, 12);
            Some(Box::new(move || drop(black_box((&a - &b).norm()))))
        }
        "geometry3.line_point_distance" => {
            let a = p3(0, 0, 0);
            let b = p3(13, 2, 1);
            let query = p3(3, 17, -2);
            Some(Box::new(move || {
                let direction = &b - &a;
                drop(black_box(
                    (direction.cross(&(&query - &a)).norm() / direction.norm()).unwrap(),
                ));
            }))
        }
        "geometry3.segment_point_distance" => {
            let a = p3(0, 0, 0);
            let b = p3(13, 2, 1);
            let query = p3(3, 17, -2);
            Some(Box::new(move || {
                let direction = &b - &a;
                let offset = &query - &a;
                let projection = direction.dot(&offset);
                let output = if projection.partial_cmp(&Real::zero()) != Some(Ordering::Greater) {
                    offset.norm()
                } else if projection.partial_cmp(&direction.norm_squared()) != Some(Ordering::Less)
                {
                    (&query - &b).norm()
                } else {
                    (direction.cross(&offset).norm() / direction.norm()).unwrap()
                };
                drop(black_box(output));
            }))
        }
        "geometry3.plane_point_distance" => {
            let plane = Plane3 {
                normal: p3(0, 0, 1),
                offset: r(0),
            };
            let query = p3(3, 4, 12);
            Some(Box::new(move || {
                drop(black_box(
                    (point_plane_value(&plane, &query).abs() / plane.normal.to_vector().norm())
                        .unwrap(),
                ));
            }))
        }
        _ => None,
    }
}

fn polynomial_expr(coefficients: &[i64], symbol: &Expr) -> Expr {
    coefficients
        .iter()
        .rev()
        .fold(Expr::zero(), |value, coefficient| {
            value * symbol.clone() + Expr::int(*coefficient)
        })
}

fn polynomial_runner(id: &str) -> Option<Runner> {
    let left = [-6, 11, -6, 1].map(r);
    let right = [2, -3, 1].map(r);

    if id == "polynomial.evaluate" {
        return Some(Box::new(move || {
            drop(black_box(Real::eval_poly(&left, &r(7))));
        }));
    }

    if let Some(operation) = indexed_suffix(id, "polynomial.binary_", 4) {
        return Some(Box::new(move || {
            let output = match operation {
                0 => Real::eval_poly(&left, &r(7)) + Real::eval_poly(&right, &r(7)),
                1 => Real::eval_poly(&left, &r(7)) - Real::eval_poly(&right, &r(7)),
                2 => Real::eval_poly(&left, &r(7)) * Real::eval_poly(&right, &r(7)),
                3 => Real::eval_poly(
                    &subresultant_chain_univariate_polynomials(&left, &right, -128)
                        .unwrap()
                        .steps[0]
                        .remainder,
                    &r(7),
                ),
                4 => Real::eval_poly(&left, &Real::eval_poly(&right, &r(7))),
                _ => unreachable!(),
            };
            drop(black_box(output));
        }));
    }

    match id {
        "polynomial.derivative" => {
            let derivative = [r(-12), r(6)];
            Some(Box::new(move || {
                drop(black_box(Real::eval_poly(&derivative, &r(7))));
            }))
        }
        "polynomial.resultant" => Some(Box::new(move || {
            drop(black_box(
                resultant_univariate_polynomials(&left, &right, -128).unwrap(),
            ));
        })),
        "polynomial.discriminant" => {
            let derivative = [r(11), r(-12), r(3)];
            Some(Box::new(move || {
                drop(black_box(
                    resultant_univariate_polynomials(&left, &derivative, -128).unwrap(),
                ));
            }))
        }
        "polynomial.gcd" => Some(Box::new(move || {
            drop(black_box(
                subresultant_chain_univariate_polynomials(&left, &right, -128).unwrap(),
            ));
        })),
        "polynomial.square_free" => Some(Box::new(move || {
            drop(black_box(square_free_part(left.to_vec(), POLICY)));
        })),
        "polynomial.root_count"
        | "polynomial.root_count_interval"
        | "polynomial.root_isolation" => {
            let x = Expr::symbol(SymbolId(0), "x");
            let mut problem = Problem::default();
            problem.add_variable("x", Real::zero());
            let expression = polynomial_expr(&[-6, 11, -6, 1], &x);
            Some(Box::new(move || {
                drop(black_box(isolate_univariate_polynomial_expr(
                    0,
                    &expression,
                    &problem,
                    POLICY,
                )));
            }))
        }
        _ => None,
    }
}

fn bivariate_runner(id: &str) -> Option<Runner> {
    let left = BivariatePolynomial::new(vec![vec![r(0), r(1)], vec![r(-1), r(0)]]);
    if id == "bivariate.evaluate" {
        return Some(Box::new(move || {
            let coefficients = left
                .coefficients
                .iter()
                .map(|row| Real::eval_poly(row, &r(7)))
                .collect::<Vec<_>>();
            drop(black_box(Real::eval_poly(&coefficients, &r(3))));
        }));
    }
    if id == "bivariate.resultant" {
        let right = BivariatePolynomial::new(vec![vec![r(-1), r(1)], vec![r(1), r(0)]]);
        return Some(Box::new(move || {
            drop(black_box(resultant_bivariate_polynomial_system(
                &left,
                &right,
                CurveResultantParameter::First,
                CurveIntersectionResultantConfig::default(),
            )));
        }));
    }
    None
}

fn triangulation_runner(id: &str) -> Option<Runner> {
    if id != "triangulation.delaunay_complex" {
        return None;
    }
    let points = [[0, 0], [6, 0], [7, 4], [3, 7], [-2, 4], [2, 3]]
        .into_iter()
        .map(|point| PointD::new(vec![r(point[0]), r(point[1])]))
        .collect::<Vec<_>>();
    Some(Box::new(move || {
        drop(black_box(
            hypertri::nd::delaunay_complex(&TRI_CONTEXT, &points).unwrap(),
        ));
    }))
}

fn curve_arc(center_x: i64) -> CircularArc2 {
    CircularArc2::try_from_center(
        curve_point(center_x + 5, 0),
        curve_point(center_x - 5, 0),
        curve_point(center_x, 0),
        false,
    )
    .expect("fixed circular-arc fixture")
}

fn curve_runner(id: &str) -> Option<Runner> {
    match id {
        "curve.line_length" => {
            let line = LineSeg2::try_new(curve_point(0, 0), curve_point(13, 2)).unwrap();
            Some(Box::new(move || {
                drop(black_box(line.length_squared().sqrt().unwrap()));
            }))
        }
        "curve.line_side" => {
            let line = LineSeg2::try_new(curve_point(0, 0), curve_point(13, 2)).unwrap();
            let query = curve_point(4, 4);
            Some(Box::new(move || {
                let _ = black_box(
                    line.classify_point(&query)
                        .expect("fixed curve fixture must certify"),
                );
            }))
        }
        "curve.line_contains_point" => {
            let line = LineSeg2::try_new(curve_point(0, 0), curve_point(13, 2)).unwrap();
            let query = curve_point(4, 4);
            Some(Box::new(move || {
                let _ = black_box(
                    line.contains_point(&query)
                        .expect("fixed curve fixture must certify"),
                );
            }))
        }
        "curve.line_intersection_topology" | "curve.line_intersection_witness" => {
            let first = LineSeg2::try_new(curve_point(0, 0), curve_point(13, 2)).unwrap();
            let second = LineSeg2::try_new(curve_point(3, 17), curve_point(19, -7)).unwrap();
            Some(Box::new(move || {
                drop(black_box(
                    first
                        .intersect_line(&second)
                        .expect("fixed curve fixture must certify"),
                ));
            }))
        }
        "curve.segment_dispatch" => {
            let first =
                Segment2::Line(LineSeg2::try_new(curve_point(0, 0), curve_point(13, 2)).unwrap());
            let second =
                Segment2::Line(LineSeg2::try_new(curve_point(3, 17), curve_point(19, -7)).unwrap());
            Some(Box::new(move || {
                drop(black_box(
                    first
                        .intersect_segment(&second)
                        .expect("fixed curve fixture must certify"),
                ));
            }))
        }
        "curve.supporting_line_circle" => {
            let circle = curve_arc(0);
            let line = LineSeg2::try_new(curve_point(-10, 7), curve_point(10, 7)).unwrap();
            Some(Box::new(move || {
                drop(black_box(
                    line.supporting_line_circle_relation(&circle)
                        .expect("fixed curve fixture must certify"),
                ));
            }))
        }
        "curve.circle_point_distance" => {
            let circle = curve_arc(0);
            let query = curve_point(13, 7);
            Some(Box::new(move || {
                drop(black_box(
                    query.distance_squared(circle.center()).sqrt().unwrap() - r(5),
                ));
            }))
        }
        "curve.circle_circle_relation" => {
            let first = curve_arc(0);
            let second = curve_arc(12);
            Some(Box::new(move || {
                drop(black_box(
                    first
                        .circle_relation(&second)
                        .expect("fixed curve fixture must certify"),
                ));
            }))
        }
        _ => None,
    }
}

fn homogeneous(point: &Point3) -> HomogeneousPoint3 {
    HomogeneousPoint3::new(
        point.x.clone(),
        point.y.clone(),
        point.z.clone(),
        Real::one(),
    )
}

fn mesh_triangle(points: [[i64; 3]; 3], index: isize) -> ConvexPolygon {
    convex_triangle(
        &MESH_CONTEXT,
        &p3(points[0][0], points[0][1], points[0][2]),
        &p3(points[1][0], points[1][1], points[1][2]),
        &p3(points[2][0], points[2][1], points[2][2]),
        0,
        index,
    )
    .unwrap()
    .into_value()
}

fn mesh_runner(id: &str) -> Option<Runner> {
    match id {
        "mesh.plane_point_classification" => {
            let query = p3(4, 4, 0);
            let plane = Plane::from_points(&p3(0, 0, 0), &p3(13, 2, 0), &p3(3, 17, 0));
            Some(Box::new(move || {
                drop(black_box(hypermesh::classify_point(
                    &MESH_CONTEXT,
                    &query,
                    &plane,
                )));
            }))
        }
        "mesh.triangle_contains_point" => {
            let triangle = mesh_triangle([[0, 0, 0], [13, 2, 0], [3, 17, 0]], 0);
            let query = homogeneous(&p3(4, 4, 0));
            Some(Box::new(move || {
                drop(black_box(triangle.contains_point(&MESH_CONTEXT, &query)));
            }))
        }
        "mesh.triangle_contains_point_strictly" => {
            let triangle = mesh_triangle([[0, 0, 0], [13, 2, 0], [3, 17, 0]], 0);
            let query = homogeneous(&p3(4, 4, 0));
            Some(Box::new(move || {
                drop(black_box(
                    triangle.contains_point_strictly(&MESH_CONTEXT, &query),
                ));
            }))
        }
        "mesh.triangle_boundary_point" => {
            let triangle = mesh_triangle([[0, 0, 0], [13, 2, 0], [3, 17, 0]], 0);
            let endpoint = homogeneous(&p3(13, 2, 0));
            Some(Box::new(move || {
                let closed = triangle
                    .contains_point(&MESH_CONTEXT, &endpoint)
                    .unwrap()
                    .into_value();
                let strict = triangle
                    .contains_point_strictly(&MESH_CONTEXT, &endpoint)
                    .unwrap()
                    .into_value();
                black_box(closed && !strict);
            }))
        }
        "mesh.triangle_triangle_intersection" => {
            let first = mesh_triangle([[0, 0, 0], [13, 2, 0], [3, 17, 0]], 0);
            let second = mesh_triangle([[4, 4, -7], [4, 4, 7], [9, 4, 0]], 1);
            Some(Box::new(move || {
                drop(black_box(intersect_polygons(
                    &MESH_CONTEXT,
                    &first,
                    &second,
                    1,
                )));
            }))
        }
        _ => None,
    }
}

fn path_line(x0: i64, y0: i64, x1: i64, y1: i64) -> LinePathSegment {
    LinePathSegment::new(p2(x0, y0), p2(x1, y1), POLICY).unwrap()
}

fn path_circle(center_x: i64, radius: i64) -> ExplicitCircularArc {
    ExplicitCircularArc::new(
        p2(center_x, 0),
        r(radius),
        p2(center_x + radius, 0),
        p2(center_x + radius, 0),
        ArcDirection::Ccw,
        POLICY,
    )
    .unwrap()
}

fn path_runner(id: &str) -> Option<Runner> {
    match id {
        "path.line_length" => {
            let line = path_line(-2, 3, 6, -3);
            Some(Box::new(move || {
                drop(black_box(line.euclidean_length().unwrap()));
            }))
        }
        "path.line_axis_classification" => {
            let line = path_line(0, 4, 12, 4);
            Some(Box::new(move || {
                drop(black_box(line.axis_length(POLICY)));
            }))
        }
        "path.line_endpoint_equality" => {
            let first = path_line(-2, 3, 6, -3);
            let second = path_line(6, -3, -2, 3);
            Some(Box::new(move || {
                let _ = black_box(first.exact_endpoint_equal(&second, POLICY));
            }))
        }
        "path.line_parameter_order" => {
            let line = path_line(0, 0, 10, 0);
            let first = p2(2, 0);
            let second = p2(8, 0);
            Some(Box::new(move || {
                let _ = black_box(line.compare_points_along(&first, &second, POLICY));
            }))
        }
        "path.circle_point_membership" => {
            let circle = path_circle(0, 5);
            let query = p2(3, 4);
            Some(Box::new(move || {
                let _ = black_box(circle.classify_point(&query, POLICY));
            }))
        }
        "path.circle_segment_intersection" => {
            let circle = path_circle(0, 5);
            let crossing = path_line(-10, 0, 10, 0);
            Some(Box::new(move || {
                drop(black_box(circle.intersect_segment(&crossing, POLICY)));
            }))
        }
        "path.circle_circle_relation" => {
            let first = path_circle(0, 5);
            let second = path_circle(12, 5);
            Some(Box::new(move || {
                drop(black_box(first.classify_circle_relation(&second, POLICY)));
            }))
        }
        _ => None,
    }
}

#[cfg(test)]
mod tests {
    use std::collections::BTreeSet;

    use super::*;

    #[test]
    fn catalog_has_160_unique_executable_hyper_operations() {
        assert_eq!(COMPARABLE_OPERATION_IDS.len(), 160);
        assert_eq!(
            COMPARABLE_OPERATION_IDS
                .iter()
                .copied()
                .collect::<BTreeSet<_>>()
                .len(),
            COMPARABLE_OPERATION_IDS.len()
        );
        for id in COMPARABLE_OPERATION_IDS {
            let mut operation = HyperMemoryOperation::new(id)
                .unwrap_or_else(|error| panic!("could not construct {id}: {error}"));
            operation.run_iterations(1);
        }
    }
}
