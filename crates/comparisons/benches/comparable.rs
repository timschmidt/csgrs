use std::cmp::Ordering;
use std::hint::black_box;
use std::time::Instant;

use criterion::{Criterion, criterion_group, criterion_main};
use exactcore_hyper_comparison::{
    COMPARABLE_OPERATION_IDS, ComplexBinary, ExprBinary, ExprUnary, LinearBinary,
    PreparedBenchmark, RationalBinary, RationalUnary, area2, between2, bivariate_eval,
    bivariate_resultant_eval, circle2_distance, circle2_relation, complex_binary,
    complex_norm_squared, delaunay_dt4, expr_binary, expr_constant, expr_floor_ceil,
    expr_sign_unary, expr_unary, incircle2, line2_intersection, line2_point_distance,
    line2_relation, line3_point_distance, line3_relation, matrix_adjugate, matrix_binary,
    matrix_determinant, matrix_transpose, orientation2, orientation3, plane3_point_distance,
    plane3_relation, point2_distance, point3_distance, polynomial_binary_eval,
    polynomial_derivative_eval, polynomial_discriminant, polynomial_eval, polynomial_gcd_degree,
    polynomial_isolate_roots, polynomial_resultant, polynomial_root_count,
    polynomial_root_count_interval, polynomial_square_free_degree, rational_binary,
    rational_compare, rational_parts, rational_unary, segment2_point_distance, segment2_relation,
    segment3_point_distance, segment3_relation, triangle3_relation, vector_binary, vector_cross3,
    vector_dot, vector_norm, volume3,
};
use hyperlattice::{Complex, Matrix3, Matrix4, Point3, Vector2, Vector3, Vector4};
use hyperlimit::{
    Plane3, PredicatePolicy, classify_circle_line2, classify_circle_segment2,
    classify_plane_segment, classify_point_line, classify_point_plane, classify_point_segment,
    classify_point_segment3, classify_point_triangle3, classify_segment_intersection,
    classify_segment_triangle3_intersection, classify_segment3_intersection,
    classify_triangle_triangle3, construct_line_intersection_point, incircle2 as hyper_incircle2,
    orient2, orient2d_value, orient3, point_plane_value,
};
use hyperreal::{Rational, Real};
use hypersolve::{
    BivariatePolynomial, CurveIntersectionResultantConfig, CurveResultantParameter, Expr, Problem,
    SymbolId, isolate_univariate_polynomial_expr, resultant_bivariate_polynomial_system,
    resultant_univariate_polynomials, square_free_part, subresultant_chain_univariate_polynomials,
};
use hypertri::{PointD, TriangulationContext};

const POLICY: PredicatePolicy = PredicatePolicy::APPROXIMATE_512;
const TRI_CONTEXT: TriangulationContext = TriangulationContext::new(POLICY);

fn pair<Exact, Hyper, ExactOutput, HyperOutput>(
    criterion: &mut Criterion,
    id: &str,
    mut exact: Exact,
    mut hyper: Hyper,
) where
    Exact: FnMut() -> ExactOutput,
    Hyper: FnMut() -> HyperOutput,
{
    assert!(
        COMPARABLE_OPERATION_IDS.contains(&id),
        "{id} is missing from the memory-operation catalog"
    );
    let exact_id = format!("{id}/exactCorelib");
    let retained_id = format!("{id}/exactCorelib-retained");
    let hyper_id = format!("{id}/hyper");
    criterion.bench_function(&exact_id, |bencher| bencher.iter(|| black_box(exact())));
    let mut retained = PreparedBenchmark::new(id).expect("construct retained exactCore fixture");
    criterion.bench_function(&retained_id, |bencher| {
        bencher.iter_custom(|iterations| {
            let started = Instant::now();
            retained
                .run_iterations(iterations)
                .expect("run retained exactCore fixture");
            started.elapsed()
        })
    });
    criterion.bench_function(&hyper_id, |bencher| bencher.iter(|| black_box(hyper())));
}

fn rational(text: &str) -> Rational {
    text.parse().unwrap()
}

fn real(text: &str) -> Real {
    Real::new(rational(text))
}

fn r(value: i64) -> Real {
    Real::from(value)
}

fn p2(x: i64, y: i64) -> hyperlimit::Point2 {
    hyperlimit::Point2::new(r(x), r(y))
}

fn p3(x: i64, y: i64, z: i64) -> Point3 {
    Point3::new(r(x), r(y), r(z))
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

fn scalar_benchmarks(criterion: &mut Criterion) {
    let left_text = "123456789/1000003";
    let right_text = "-987654321/1000033";
    let left = rational(left_text);
    let right = rational(right_text);
    for (id, operation) in [
        ("rational.add", RationalBinary::Add),
        ("rational.subtract", RationalBinary::Subtract),
        ("rational.multiply", RationalBinary::Multiply),
        ("rational.divide", RationalBinary::Divide),
    ] {
        pair(
            criterion,
            id,
            || rational_binary(operation, black_box(left_text), black_box(right_text)).unwrap(),
            || match operation {
                RationalBinary::Add => black_box(&left + &right),
                RationalBinary::Subtract => black_box(&left - &right),
                RationalBinary::Multiply => black_box(&left * &right),
                RationalBinary::Divide => black_box(&left / &right),
            },
        );
    }
    for (id, operation) in [
        ("rational.negate", RationalUnary::Negate),
        ("rational.absolute", RationalUnary::Absolute),
        ("rational.reciprocal", RationalUnary::Reciprocal),
    ] {
        pair(
            criterion,
            id,
            || rational_unary(operation, black_box(left_text)).unwrap(),
            || match operation {
                RationalUnary::Negate => -&left,
                RationalUnary::Absolute => {
                    if left.is_negative() {
                        -&left
                    } else {
                        left.clone()
                    }
                }
                RationalUnary::Reciprocal => left.clone().inverse().unwrap(),
            },
        );
    }
    pair(
        criterion,
        "rational.compare",
        || rational_compare(black_box(left_text), black_box(right_text)).unwrap(),
        || left.partial_cmp(black_box(&right)),
    );
    pair(
        criterion,
        "rational.parts",
        || rational_parts(black_box(left_text)).unwrap(),
        || (left.numerator().clone(), left.denominator().clone()),
    );

    let unary_input = "1/3";
    let input = real(unary_input);
    for (id, operation, root_degree) in [
        ("real.negate", ExprUnary::Negate, 0),
        ("real.absolute", ExprUnary::Absolute, 0),
        ("real.sqrt", ExprUnary::Sqrt, 0),
        ("real.cbrt", ExprUnary::Cbrt, 0),
        ("real.root_n", ExprUnary::RootN, 5),
        ("real.exp", ExprUnary::Exp, 0),
        ("real.ln", ExprUnary::Ln, 0),
        ("real.log2", ExprUnary::Log2, 0),
        ("real.log10", ExprUnary::Log10, 0),
        ("real.sin", ExprUnary::Sin, 0),
        ("real.cos", ExprUnary::Cos, 0),
        ("real.tan", ExprUnary::Tan, 0),
        ("real.asin", ExprUnary::Asin, 0),
        ("real.acos", ExprUnary::Acos, 0),
        ("real.atan", ExprUnary::Atan, 0),
        ("real.square", ExprUnary::Square, 0),
        ("real.exp2", ExprUnary::Exp2, 0),
        ("real.exp10", ExprUnary::Exp10, 0),
        ("real.cot", ExprUnary::Cot, 0),
    ] {
        pair(
            criterion,
            id,
            || expr_unary(operation, black_box(unary_input), root_degree),
            || match operation {
                ExprUnary::Negate => -input.clone(),
                ExprUnary::Absolute => input.clone().abs(),
                ExprUnary::Sqrt => input.clone().sqrt().unwrap(),
                ExprUnary::Cbrt => input.clone().cbrt().unwrap(),
                ExprUnary::RootN => input.clone().root_n(root_degree).unwrap(),
                ExprUnary::Exp => input.clone().exp().unwrap(),
                ExprUnary::Ln => input.clone().ln().unwrap(),
                ExprUnary::Log2 => input.clone().log2().unwrap(),
                ExprUnary::Log10 => input.clone().log10().unwrap(),
                ExprUnary::Sin => input.clone().sin(),
                ExprUnary::Cos => input.clone().cos(),
                ExprUnary::Tan => input.clone().tan().unwrap(),
                ExprUnary::Asin => input.clone().asin().unwrap(),
                ExprUnary::Acos => input.clone().acos().unwrap(),
                ExprUnary::Atan => input.clone().atan().unwrap(),
                ExprUnary::Square => input.clone().powi_i64(2).unwrap(),
                ExprUnary::Exp2 => r(2).pow(input.clone()).unwrap(),
                ExprUnary::Exp10 => r(10).pow(input.clone()).unwrap(),
                ExprUnary::Cot => (input.clone().cos() / input.clone().sin()).unwrap(),
            },
        );
    }

    let binary_left = real("7/3");
    let binary_right = real("5/11");
    for (id, operation, parameter) in [
        ("real.add", ExprBinary::Add, 0),
        ("real.subtract", ExprBinary::Subtract, 0),
        ("real.multiply", ExprBinary::Multiply, 0),
        ("real.divide", ExprBinary::Divide, 0),
        ("real.powi", ExprBinary::IntegerPower, 7),
    ] {
        pair(
            criterion,
            id,
            || expr_binary(operation, "7/3", "5/11", parameter),
            || match operation {
                ExprBinary::Add => binary_left.clone() + binary_right.clone(),
                ExprBinary::Subtract => binary_left.clone() - binary_right.clone(),
                ExprBinary::Multiply => binary_left.clone() * binary_right.clone(),
                ExprBinary::Divide => (binary_left.clone() / binary_right.clone()).unwrap(),
                ExprBinary::IntegerPower => binary_left.clone().powi_i64(parameter).unwrap(),
            },
        );
    }
    pair(criterion, "real.pi", || expr_constant(0), Real::pi);
    pair(criterion, "real.e", || expr_constant(1), Real::e);
    pair(
        criterion,
        "real.sign",
        || expr_sign_unary(ExprUnary::Sin, unary_input, 0),
        || input.clone().sin().refine_sign_until(-128),
    );
    pair(
        criterion,
        "real.floor",
        || expr_floor_ceil(false, "7/3").unwrap(),
        || binary_left.floor_certified().unwrap(),
    );
    pair(
        criterion,
        "real.ceil",
        || expr_floor_ceil(true, "7/3").unwrap(),
        || binary_left.ceil_certified().unwrap(),
    );
}

fn linear_algebra_benchmarks(criterion: &mut Criterion) {
    let complex_left = Complex::new(real("7/13"), real("-11/17"));
    let complex_right = Complex::new(real("19/23"), real("29/31"));
    for (id, operation) in [
        ("complex.add", ComplexBinary::Add),
        ("complex.subtract", ComplexBinary::Subtract),
        ("complex.multiply", ComplexBinary::Multiply),
        ("complex.divide", ComplexBinary::Divide),
    ] {
        pair(
            criterion,
            id,
            || complex_binary(operation, ("7/13", "-11/17"), ("19/23", "29/31")).unwrap(),
            || match operation {
                ComplexBinary::Add => &complex_left + &complex_right,
                ComplexBinary::Subtract => &complex_left - &complex_right,
                ComplexBinary::Multiply => &complex_left * &complex_right,
                ComplexBinary::Divide => (&complex_left / &complex_right).unwrap(),
            },
        );
    }
    pair(
        criterion,
        "complex.norm_squared",
        || complex_norm_squared("7/13", "-11/17").unwrap(),
        || complex_left.norm_squared(),
    );

    let left2 = [3, -4];
    let right2 = [7, 11];
    let a2 = Vector2::new(left2.map(r));
    let b2 = Vector2::new(right2.map(r));
    pair(
        criterion,
        "vector2.add",
        || vector_binary(LinearBinary::Add, &left2, &right2),
        || &a2 + &b2,
    );
    pair(
        criterion,
        "vector2.subtract",
        || vector_binary(LinearBinary::Subtract, &left2, &right2),
        || &a2 - &b2,
    );
    pair(
        criterion,
        "vector2.dot",
        || vector_dot(&left2, &right2),
        || a2.dot(&b2),
    );
    pair(
        criterion,
        "vector2.norm",
        || vector_norm(&left2),
        || a2.norm(),
    );
    pair(
        criterion,
        "vector2.wedge",
        || area2([0, 0, left2[0], left2[1], right2[0], right2[1]]),
        || a2.wedge(&b2),
    );

    let left3 = [3, -4, 5];
    let right3 = [7, 11, -13];
    let a3 = Vector3::new(left3.map(r));
    let b3 = Vector3::new(right3.map(r));
    pair(
        criterion,
        "vector3.add",
        || vector_binary(LinearBinary::Add, &left3, &right3),
        || &a3 + &b3,
    );
    pair(
        criterion,
        "vector3.subtract",
        || vector_binary(LinearBinary::Subtract, &left3, &right3),
        || &a3 - &b3,
    );
    pair(
        criterion,
        "vector3.dot",
        || vector_dot(&left3, &right3),
        || a3.dot(&b3),
    );
    pair(
        criterion,
        "vector3.cross",
        || vector_cross3(left3, right3),
        || a3.cross(&b3),
    );
    pair(
        criterion,
        "vector3.norm",
        || vector_norm(&left3),
        || a3.norm(),
    );

    let left4 = [3, -4, 5, -6];
    let right4 = [7, 11, -13, 17];
    let a4 = Vector4::new(left4.map(r));
    let b4 = Vector4::new(right4.map(r));
    pair(
        criterion,
        "vector4.add",
        || vector_binary(LinearBinary::Add, &left4, &right4),
        || &a4 + &b4,
    );
    pair(
        criterion,
        "vector4.subtract",
        || vector_binary(LinearBinary::Subtract, &left4, &right4),
        || &a4 - &b4,
    );
    pair(
        criterion,
        "vector4.dot",
        || vector_dot(&left4, &right4),
        || a4.dot(&b4),
    );
    pair(
        criterion,
        "vector4.norm",
        || vector_norm(&left4),
        || a4.norm(),
    );

    let left3m = [2, -1, 3, 4, 0, 5, -2, 7, 1];
    let right3m = [1, 2, 0, -3, 4, 1, 5, -2, 6];
    let a3m = matrix3(left3m);
    let b3m = matrix3(right3m);
    for (id, operation) in [
        ("matrix3.add", LinearBinary::Add),
        ("matrix3.subtract", LinearBinary::Subtract),
        ("matrix3.multiply", LinearBinary::Multiply),
    ] {
        pair(
            criterion,
            id,
            || matrix_binary(operation, &left3m, &right3m),
            || match operation {
                LinearBinary::Add => &a3m + &b3m,
                LinearBinary::Subtract => &a3m - &b3m,
                LinearBinary::Multiply => &a3m * &b3m,
            },
        );
    }
    pair(
        criterion,
        "matrix3.transpose",
        || matrix_transpose(&left3m),
        || a3m.transpose(),
    );
    pair(
        criterion,
        "matrix3.determinant",
        || matrix_determinant(&left3m),
        || a3m.determinant(),
    );
    pair(
        criterion,
        "matrix3.inverse",
        || matrix_adjugate(&left3m),
        || a3m.clone().inverse(),
    );

    let left4m = [2, 1, 0, 3, -1, 4, 2, 0, 5, 0, 3, 1, 2, -2, 1, 6];
    let right4m = [1, 0, 2, -1, 3, 1, 0, 4, -2, 5, 1, 0, 0, 2, 3, 1];
    let a4m = matrix4(left4m);
    let b4m = matrix4(right4m);
    for (id, operation) in [
        ("matrix4.add", LinearBinary::Add),
        ("matrix4.subtract", LinearBinary::Subtract),
        ("matrix4.multiply", LinearBinary::Multiply),
    ] {
        pair(
            criterion,
            id,
            || matrix_binary(operation, &left4m, &right4m),
            || match operation {
                LinearBinary::Add => &a4m + &b4m,
                LinearBinary::Subtract => &a4m - &b4m,
                LinearBinary::Multiply => &a4m * &b4m,
            },
        );
    }
    pair(
        criterion,
        "matrix4.transpose",
        || matrix_transpose(&left4m),
        || a4m.transpose(),
    );
    pair(
        criterion,
        "matrix4.determinant",
        || matrix_determinant(&left4m),
        || a4m.determinant(),
    );
    pair(
        criterion,
        "matrix4.inverse",
        || matrix_adjugate(&left4m),
        || a4m.clone().inverse(),
    );
}

fn geometry_2d_benchmarks(criterion: &mut Criterion) {
    let triangle = [0, 0, 13, 2, 3, 17];
    let a = p2(0, 0);
    let b = p2(13, 2);
    let query = p2(3, 17);
    pair(
        criterion,
        "geometry2.orientation",
        || orientation2(triangle),
        || orient2(&a, &b, &query, POLICY),
    );
    pair(
        criterion,
        "geometry2.area",
        || area2(triangle),
        || orient2d_value(&a, &b, &query),
    );
    pair(
        criterion,
        "geometry2.between",
        || between2([0, 0, 13, 2, 6, 1]),
        || classify_point_segment(&a, &b, &p2(6, 1), POLICY),
    );

    let lines = [0, 0, 13, 2, 3, 17, 19, -7];
    let c = p2(3, 17);
    let d = p2(19, -7);
    for operation in 0..=6 {
        let id = format!("geometry2.line_relation_{operation}");
        pair(
            criterion,
            &id,
            || line2_relation(operation, &lines),
            || match operation {
                0 | 1 => {
                    black_box(classify_point_line(&a, &b, &c, POLICY));
                }
                2..=4 => {
                    let ab = &b - &a;
                    let cd = &d - &c;
                    black_box((ab.wedge(&cd), orient2(&a, &b, &c, POLICY)));
                }
                5 => {
                    black_box(a.x == b.x);
                }
                6 => {
                    black_box(a.y == b.y);
                }
                _ => unreachable!(),
            },
        );
    }
    pair(
        criterion,
        "geometry2.line_intersection",
        || line2_intersection(lines),
        || construct_line_intersection_point(&a, &b, &c, &d),
    );

    let segments = [0, 0, 13, 2, 3, 17, 19, -7];
    for operation in 0..=3 {
        let id = format!("geometry2.segment_relation_{operation}");
        pair(
            criterion,
            &id,
            || segment2_relation(operation, &segments),
            || match operation {
                0 => {
                    black_box(classify_point_segment(&a, &b, &c, POLICY));
                }
                1 | 2 => {
                    black_box(classify_segment_intersection(&a, &b, &c, &d, POLICY));
                }
                3 => {
                    black_box((&b - &a).wedge(&(&d - &c)));
                }
                _ => unreachable!(),
            },
        );
    }
    pair(
        criterion,
        "geometry2.incircle",
        || incircle2([0, 0, 13, 2, 3, 17, 4, 4]),
        || hyper_incircle2(&a, &b, &query, &p2(4, 4), POLICY),
    );
    pair(
        criterion,
        "geometry2.circle_line",
        || circle2_relation(0, 5, [0, 0, -10, 7, 10, 7]),
        || classify_circle_line2(&p2(0, 0), &r(25), &p2(-10, 7), &p2(10, 7), POLICY),
    );
    pair(
        criterion,
        "geometry2.circle_segment",
        || circle2_relation(1, 5, [0, 0, -10, 7, 10, 7]),
        || classify_circle_segment2(&p2(0, 0), &r(25), &p2(-10, 7), &p2(10, 7), POLICY),
    );
    pair(
        criterion,
        "geometry2.circle_point_distance",
        || circle2_distance(0, 5, 0, [0, 0, 13, 7]),
        || (&p2(13, 7) - &p2(0, 0)).norm() - r(5),
    );
    pair(
        criterion,
        "geometry2.circle_circle_distance",
        || circle2_distance(1, 5, 7, [0, 0, 17, 4]),
        || {
            let separation = (&p2(17, 4) - &p2(0, 0)).norm() - r(12);
            if separation.partial_cmp(&Real::zero()) == Some(Ordering::Greater) {
                separation
            } else {
                Real::zero()
            }
        },
    );
    pair(
        criterion,
        "geometry2.point_distance",
        || point2_distance([0, 0, 3, 4]),
        || (&p2(0, 0) - &p2(3, 4)).norm(),
    );
    pair(
        criterion,
        "geometry2.line_point_distance",
        || line2_point_distance([0, 0, 13, 2, 3, 17]),
        || {
            let direction = &b - &a;
            (direction.wedge(&(&query - &a)).abs() / direction.norm()).unwrap()
        },
    );
    pair(
        criterion,
        "geometry2.segment_point_distance",
        || segment2_point_distance([0, 0, 13, 2, 3, 17]),
        || {
            let direction = &b - &a;
            let offset = &query - &a;
            let projection = direction.dot(&offset);
            if projection.partial_cmp(&Real::zero()) != Some(Ordering::Greater) {
                offset.norm()
            } else if projection.partial_cmp(&direction.norm_squared()) != Some(Ordering::Less) {
                (&query - &b).norm()
            } else {
                (direction.wedge(&offset).abs() / direction.norm()).unwrap()
            }
        },
    );
}

fn geometry_3d_benchmarks(criterion: &mut Criterion) {
    let tetrahedron = [0, 0, 0, 13, 2, 1, 3, 17, -2, 4, 5, 19];
    let a = p3(0, 0, 0);
    let b = p3(13, 2, 1);
    let c = p3(3, 17, -2);
    let d = p3(4, 5, 19);
    pair(
        criterion,
        "geometry3.orientation",
        || orientation3(tetrahedron),
        || orient3(&a, &b, &c, &d, POLICY),
    );
    pair(
        criterion,
        "geometry3.volume",
        || volume3(tetrahedron),
        || {
            let rows = [
                [&a.x - &d.x, &a.y - &d.y, &a.z - &d.z],
                [&b.x - &d.x, &b.y - &d.y, &b.z - &d.z],
                [&c.x - &d.x, &c.y - &d.y, &c.z - &d.z],
            ];
            Matrix3::new(rows).determinant()
        },
    );

    let lines = [0, 0, 0, 13, 2, 1, 3, 17, -2, 4, 5, 19];
    for operation in 0..=4 {
        let id = format!("geometry3.line_relation_{operation}");
        pair(
            criterion,
            &id,
            || line3_relation(operation, &lines),
            || match operation {
                0 => {
                    black_box(classify_point_segment3(&a, &b, &c, POLICY));
                }
                1 | 4 => {
                    black_box((&b - &a).cross(&(&d - &c)).norm_squared());
                }
                2 | 3 => {
                    black_box(orient3(&a, &b, &c, &d, POLICY));
                }
                _ => unreachable!(),
            },
        );
    }
    for operation in 0..=3 {
        let id = format!("geometry3.segment_relation_{operation}");
        pair(
            criterion,
            &id,
            || segment3_relation(operation, &lines),
            || match operation {
                0 => {
                    black_box(classify_point_segment3(&a, &b, &c, POLICY));
                }
                1 | 2 => {
                    black_box(classify_segment3_intersection(&a, &b, &c, &d, POLICY));
                }
                3 => {
                    black_box(orient3(&a, &b, &c, &d, POLICY));
                }
                _ => unreachable!(),
            },
        );
    }

    let plane = Plane3 {
        normal: p3(0, 0, 1),
        offset: r(0),
    };
    let plane_coordinates = [0, 0, 0, 13, 0, 0, 0, 17, 0, 4, 5, -2, 4, 5, 19, 0, 0, 7];
    for operation in 0..=7 {
        let id = format!("geometry3.plane_relation_{operation}");
        pair(
            criterion,
            &id,
            || plane3_relation(operation, &plane_coordinates),
            || match operation {
                0 | 1 => {
                    black_box(classify_point_plane(&c, &plane, POLICY));
                }
                2..=5 => {
                    black_box(classify_plane_segment(&plane, &c, &d, POLICY));
                }
                6 | 7 => {
                    black_box(point_plane_value(&plane, &p3(0, 0, 7)));
                }
                _ => unreachable!(),
            },
        );
    }

    pair(
        criterion,
        "geometry3.point_distance",
        || point3_distance([0, 0, 0, 3, 4, 12]),
        || (&p3(0, 0, 0) - &p3(3, 4, 12)).norm(),
    );
    pair(
        criterion,
        "geometry3.line_point_distance",
        || line3_point_distance([0, 0, 0, 13, 2, 1, 3, 17, -2]),
        || {
            let direction = &b - &a;
            (direction.cross(&(&c - &a)).norm() / direction.norm()).unwrap()
        },
    );
    pair(
        criterion,
        "geometry3.segment_point_distance",
        || segment3_point_distance([0, 0, 0, 13, 2, 1, 3, 17, -2]),
        || {
            let direction = &b - &a;
            let offset = &c - &a;
            let projection = direction.dot(&offset);
            if projection.partial_cmp(&Real::zero()) != Some(Ordering::Greater) {
                offset.norm()
            } else if projection.partial_cmp(&direction.norm_squared()) != Some(Ordering::Less) {
                (&c - &b).norm()
            } else {
                (direction.cross(&offset).norm() / direction.norm()).unwrap()
            }
        },
    );
    pair(
        criterion,
        "geometry3.plane_point_distance",
        || plane3_point_distance([0, 0, 0, 13, 0, 0, 0, 17, 0, 3, 4, 12]),
        || {
            (point_plane_value(&plane, &p3(3, 4, 12)).abs() / plane.normal.to_vector().norm())
                .unwrap()
        },
    );

    let triangle_coordinates = [0, 0, 0, 13, 0, 0, 0, 17, 0, 3, 4, -2, 3, 4, 19, 19, 19, 3];
    let ta = p3(0, 0, 0);
    let tb = p3(13, 0, 0);
    let tc = p3(0, 17, 0);
    let tq = p3(3, 4, -2);
    let te = p3(3, 4, 19);
    let tf = p3(19, 19, 3);
    for operation in 0..=6 {
        let id = format!("geometry3.triangle_relation_{operation}");
        pair(
            criterion,
            &id,
            || triangle3_relation(operation, &triangle_coordinates),
            || match operation {
                0..=3 => {
                    black_box(classify_point_triangle3(&ta, &tb, &tc, &tq, POLICY));
                }
                4 | 5 => {
                    black_box(classify_segment_triangle3_intersection(
                        &tq, &te, &ta, &tb, &tc, POLICY,
                    ));
                }
                6 => {
                    black_box(classify_triangle_triangle3(
                        &ta, &tb, &tc, &tq, &te, &tf, POLICY,
                    ));
                }
                _ => unreachable!(),
            },
        );
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

fn polynomial_benchmarks(criterion: &mut Criterion) {
    let left = [-6, 11, -6, 1];
    let right = [2, -3, 1];
    let left_real = left.map(r);
    let right_real = right.map(r);
    pair(
        criterion,
        "polynomial.evaluate",
        || polynomial_eval(&left, 7).unwrap(),
        || Real::eval_poly(&left_real, &r(7)),
    );
    for operation in 0..=4 {
        let id = format!("polynomial.binary_{operation}");
        pair(
            criterion,
            &id,
            || polynomial_binary_eval(operation, &left, &right, 7).unwrap(),
            || match operation {
                0 => Real::eval_poly(&left_real, &r(7)) + Real::eval_poly(&right_real, &r(7)),
                1 => Real::eval_poly(&left_real, &r(7)) - Real::eval_poly(&right_real, &r(7)),
                2 => Real::eval_poly(&left_real, &r(7)) * Real::eval_poly(&right_real, &r(7)),
                3 => Real::eval_poly(
                    &subresultant_chain_univariate_polynomials(&left_real, &right_real, -128)
                        .unwrap()
                        .steps[0]
                        .remainder,
                    &r(7),
                ),
                4 => Real::eval_poly(&left_real, &Real::eval_poly(&right_real, &r(7))),
                _ => unreachable!(),
            },
        );
    }
    pair(
        criterion,
        "polynomial.derivative",
        || polynomial_derivative_eval(&left, 2, 7).unwrap(),
        || {
            let derivative = [r(-12), r(6)];
            Real::eval_poly(&derivative, &r(7))
        },
    );
    pair(
        criterion,
        "polynomial.resultant",
        || polynomial_resultant(&left, &right).unwrap(),
        || resultant_univariate_polynomials(&left_real, &right_real, -128).unwrap(),
    );
    pair(
        criterion,
        "polynomial.discriminant",
        || polynomial_discriminant(&left).unwrap(),
        || {
            let derivative = [r(11), r(-12), r(3)];
            resultant_univariate_polynomials(&left_real, &derivative, -128).unwrap()
        },
    );
    pair(
        criterion,
        "polynomial.gcd",
        || polynomial_gcd_degree(&left, &right),
        || subresultant_chain_univariate_polynomials(&left_real, &right_real, -128).unwrap(),
    );
    pair(
        criterion,
        "polynomial.square_free",
        || polynomial_square_free_degree(&left),
        || square_free_part(left_real.to_vec(), POLICY),
    );

    let x = Expr::symbol(SymbolId(0), "x");
    let mut problem = Problem::default();
    problem.add_variable("x", Real::zero());
    let expression = polynomial_expr(&left, &x);
    pair(
        criterion,
        "polynomial.root_count",
        || polynomial_root_count(&left),
        || isolate_univariate_polynomial_expr(0, &expression, &problem, POLICY),
    );
    pair(
        criterion,
        "polynomial.root_count_interval",
        || polynomial_root_count_interval(&left, 0, 4),
        || isolate_univariate_polynomial_expr(0, &expression, &problem, POLICY),
    );
    pair(
        criterion,
        "polynomial.root_isolation",
        || polynomial_isolate_roots(&left).unwrap(),
        || isolate_univariate_polynomial_expr(0, &expression, &problem, POLICY),
    );

    let left_flat = [0, 1, -1, 0];
    let right_flat = [-1, 1, 1, 0];
    let left_bivariate = BivariatePolynomial::new(vec![vec![r(0), r(1)], vec![r(-1), r(0)]]);
    let right_bivariate = BivariatePolynomial::new(vec![vec![r(-1), r(1)], vec![r(1), r(0)]]);
    pair(
        criterion,
        "bivariate.evaluate",
        || bivariate_eval(&left_flat, 2, 2, 3, 7).unwrap(),
        || {
            let first_coefficients = left_bivariate
                .coefficients
                .iter()
                .map(|row| Real::eval_poly(row, &r(7)))
                .collect::<Vec<_>>();
            Real::eval_poly(&first_coefficients, &r(3))
        },
    );
    pair(
        criterion,
        "bivariate.resultant",
        || bivariate_resultant_eval(true, &left_flat, &right_flat, 2, 2, 7).unwrap(),
        || {
            resultant_bivariate_polynomial_system(
                &left_bivariate,
                &right_bivariate,
                CurveResultantParameter::First,
                CurveIntersectionResultantConfig::default(),
            )
        },
    );
}

fn triangulation_benchmarks(criterion: &mut Criterion) {
    let points = [[0, 0], [6, 0], [7, 4], [3, 7], [-2, 4], [2, 3]];
    let hyper_points = points
        .iter()
        .map(|point| PointD::new(vec![r(point[0]), r(point[1])]))
        .collect::<Vec<_>>();
    pair(
        criterion,
        "triangulation.delaunay_complex",
        || delaunay_dt4(&points).unwrap(),
        || hypertri::nd::delaunay_complex(&TRI_CONTEXT, &hyper_points).unwrap(),
    );
}

criterion_group!(
    benches,
    scalar_benchmarks,
    linear_algebra_benchmarks,
    geometry_2d_benchmarks,
    geometry_3d_benchmarks,
    polynomial_benchmarks,
    triangulation_benchmarks,
);
criterion_main!(benches);
