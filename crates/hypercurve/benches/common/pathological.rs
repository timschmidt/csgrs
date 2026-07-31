//! Shared generators for the native and cross-suite pathological benchmarks.
//!
//! This is intentionally a benchmark-local module.  The fixture is sharded into
//! independent region pairs so its resident size can reach a gigabyte without
//! making preparation itself a single quadratic all-to-all carrier comparison.

#![allow(dead_code)]

use std::env;

use hypercurve::{
    BulgeVertex2, CircularArc2, Contour2, CubicBezier2, Curve2, CurveContext, CurveFamily2,
    CurvePath2, CurveRegion2, LineArcRegion2, LineSeg2, Point2, QuadraticBezier2, Rational,
    RationalBezier2, RationalQuadraticBezier2, Real, Similarity2,
};
use num::bigint::{BigInt, BigUint};

const MIB: usize = 1024 * 1024;
// A release-build calibration over 100 cells retains approximately 1.5 MiB per
// cell after native Bezier promotion and exact-polyline projection of both the
// source and transformed region.
// Keep the tier mapping explicit and evidence `/proc` RSS alongside it so changes
// in upstream carrier layouts are visible in benchmark output.
const NATIVE_ESTIMATED_BYTES_PER_CELL: usize = 3 * MIB / 2;
const CURVE_SAMPLES: usize = 6;

/// Named in-memory size class used by every pathological benchmark suite.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum MemoryTier {
    Mib100,
    Mib500,
    Gib1,
}

impl MemoryTier {
    pub const ALL: [Self; 3] = [Self::Mib100, Self::Mib500, Self::Gib1];

    pub const fn name(self) -> &'static str {
        match self {
            Self::Mib100 => "100mb",
            Self::Mib500 => "500mb",
            Self::Gib1 => "1gb",
        }
    }

    pub const fn target_bytes(self) -> usize {
        match self {
            Self::Mib100 => 100 * MIB,
            Self::Mib500 => 500 * MIB,
            Self::Gib1 => 1024 * MIB,
        }
    }

    fn parse(value: &str) -> Option<Self> {
        match value.trim().to_ascii_lowercase().as_str() {
            "100mb" | "100m" | "small" => Some(Self::Mib100),
            "500mb" | "500m" | "medium" => Some(Self::Mib500),
            "1gb" | "1g" | "large" => Some(Self::Gib1),
            _ => None,
        }
    }
}

pub fn selected_tiers(variable: &str, default: &[MemoryTier]) -> Vec<MemoryTier> {
    let Ok(selection) = env::var(variable) else {
        return default.to_vec();
    };
    if selection.trim().eq_ignore_ascii_case("all") {
        return MemoryTier::ALL.to_vec();
    }
    let tiers = selection
        .split(',')
        .map(|value| {
            MemoryTier::parse(value).unwrap_or_else(|| {
                panic!("invalid {variable} entry {value:?}; expected 100mb, 500mb, 1gb, or all")
            })
        })
        .collect::<Vec<_>>();
    assert!(
        !tiers.is_empty(),
        "{variable} must select at least one tier"
    );
    tiers
}

fn requested_cell_count(tier: MemoryTier, bytes_per_cell: usize) -> usize {
    let tier_count = tier.target_bytes().div_ceil(bytes_per_cell);
    env::var("HYPERCURVE_PATHOLOGICAL_CELL_LIMIT")
        .ok()
        .map(|value| {
            let limit = value.parse::<usize>().unwrap_or_else(|error| {
                panic!("invalid HYPERCURVE_PATHOLOGICAL_CELL_LIMIT={value:?}: {error}")
            });
            assert!(
                limit > 0,
                "HYPERCURVE_PATHOLOGICAL_CELL_LIMIT must be nonzero"
            );
            limit
        })
        .map_or(tier_count, |limit| tier_count.min(limit))
}

#[derive(Clone, Debug)]
pub struct RepresentationSample {
    pub name: &'static str,
    pub value: Real,
}

/// Exercises rational storage, primitive dyadics, every recognized symbolic
/// class, and the opaque computable fallback exposed by `Real`.
pub fn real_representation_samples() -> Vec<RepresentationSample> {
    let rational = |numerator: i64, denominator: u64| {
        Real::new(Rational::fraction(numerator, denominator).expect("nonzero denominator"))
    };
    let sqrt_two = Real::from(2).sqrt().expect("positive square root");
    let exp_two = Real::from(2).exp().expect("finite exponential");
    let ln_two = Real::from(2).ln().expect("positive logarithm");
    let ln_three = Real::from(3).ln().expect("positive logarithm");
    let pi = Real::pi();
    let huge_integer = BigInt::from(1_u8) << 256;
    let huge_fraction = Rational::from_bigint_fraction(
        (BigInt::from(1_u8) << 257) + BigInt::from(19_u8),
        (BigUint::from(1_u8) << 193) + BigUint::from(7_u8),
    )
    .expect("nonzero huge denominator");
    vec![
        RepresentationSample {
            name: "small_integer_rational",
            value: Real::from(7),
        },
        RepresentationSample {
            name: "fraction_rational",
            value: rational(17, 31),
        },
        RepresentationSample {
            name: "multi_limb_integer_rational",
            value: Real::new(Rational::from_bigint(huge_integer)),
        },
        RepresentationSample {
            name: "multi_limb_fraction_rational",
            value: Real::new(huge_fraction),
        },
        RepresentationSample {
            name: "f32_dyadic_rational",
            value: Real::try_from(0.1_f32).expect("finite f32"),
        },
        RepresentationSample {
            name: "f64_dyadic_rational",
            value: Real::try_from(0.1_f64).expect("finite f64"),
        },
        RepresentationSample {
            name: "pi",
            value: pi.clone(),
        },
        RepresentationSample {
            name: "pi_power",
            value: &pi * &pi,
        },
        RepresentationSample {
            name: "pi_inverse",
            value: pi.clone().inverse().expect("pi is nonzero"),
        },
        RepresentationSample {
            name: "exp_rational",
            value: exp_two.clone(),
        },
        RepresentationSample {
            name: "pi_exp",
            value: &pi * &exp_two,
        },
        RepresentationSample {
            name: "pi_inverse_exp",
            value: (&exp_two / &pi).expect("pi is nonzero"),
        },
        RepresentationSample {
            name: "constant_product",
            value: &(&pi * &pi) * &exp_two,
        },
        RepresentationSample {
            name: "constant_offset",
            value: &pi - Real::from(3),
        },
        RepresentationSample {
            name: "square_root",
            value: sqrt_two.clone(),
        },
        RepresentationSample {
            name: "pi_square_root",
            value: &pi * &sqrt_two,
        },
        RepresentationSample {
            name: "constant_product_square_root",
            value: &(&(&pi * &pi) * &exp_two) * &sqrt_two,
        },
        RepresentationSample {
            name: "natural_logarithm",
            value: ln_two.clone(),
        },
        RepresentationSample {
            name: "logarithm_affine",
            value: Real::one() + &ln_two,
        },
        RepresentationSample {
            name: "logarithm_product",
            value: &ln_two * &ln_three,
        },
        RepresentationSample {
            name: "log10",
            value: Real::from(2).log10().expect("positive logarithm"),
        },
        RepresentationSample {
            name: "log2",
            value: Real::from(3).log2().expect("positive logarithm"),
        },
        RepresentationSample {
            name: "sin_pi_rational",
            value: rational(1, 5).sin_pi(),
        },
        RepresentationSample {
            name: "tan_pi_rational",
            value: rational(1, 5).tan_pi().expect("finite tangent"),
        },
        RepresentationSample {
            name: "opaque_computable",
            value: Real::from(1).sin(),
        },
    ]
}

#[derive(Debug)]
pub struct NativeCell {
    pub source_path: CurvePath2,
    pub rotated_path: CurvePath2,
    pub source: CurveRegion2,
    pub rotated: CurveRegion2,
    pub source_projection: LineArcRegion2,
    pub rotated_projection: LineArcRegion2,
    pub representations: Vec<RepresentationSample>,
}

#[derive(Debug)]
pub struct NativeDataset {
    pub tier: MemoryTier,
    pub cells: Vec<NativeCell>,
    pub estimated_resident_bytes: usize,
}

impl NativeDataset {
    pub fn build(tier: MemoryTier) -> Self {
        let cell_count = requested_cell_count(tier, NATIVE_ESTIMATED_BYTES_PER_CELL);
        let cells = (0..cell_count).map(build_native_cell).collect();
        Self {
            tier,
            cells,
            estimated_resident_bytes: cell_count * NATIVE_ESTIMATED_BYTES_PER_CELL,
        }
    }
}

#[derive(Clone, Debug)]
pub struct PolylinePair {
    pub source: Vec<[f64; 2]>,
    pub rotated: Vec<[f64; 2]>,
}

#[derive(Debug)]
pub struct CrossSuiteDataset {
    pub tier: MemoryTier,
    pub cells: Vec<PolylinePair>,
    pub allocated_coordinate_bytes: usize,
}

impl CrossSuiteDataset {
    pub fn build(tier: MemoryTier) -> Self {
        // Preserve the same geometric cell count as the native memory tier.
        // Floating-point polygon carriers are intentionally much smaller than
        // exact retained curves, so calling their storage 100 MiB/500 MiB/1 GiB
        // would not describe an equivalent cross-suite workload.
        let cell_count = requested_cell_count(tier, NATIVE_ESTIMATED_BYTES_PER_CELL);
        let cells = (0..cell_count)
            .map(|index| {
                let cell = build_native_cell(index);
                PolylinePair {
                    source: flatten_path(&cell.source_path),
                    rotated: flatten_path(&cell.rotated_path),
                }
            })
            .collect::<Vec<_>>();
        let allocated_coordinate_bytes = cells
            .iter()
            .map(|cell| {
                (cell.source.capacity() + cell.rotated.capacity()) * std::mem::size_of::<[f64; 2]>()
            })
            .sum();
        Self {
            tier,
            cells,
            allocated_coordinate_bytes,
        }
    }
}

pub fn build_native_cell(index: usize) -> NativeCell {
    let representations = real_representation_samples();
    let origin_x = i64::try_from(index % 1024).expect("cell column fits i64") * 96;
    let origin_y = i64::try_from(index / 1024).expect("cell row fits i64") * 48;
    let source_path = all_family_path(origin_x, origin_y, &representations);
    let transform = cell_rotation(origin_x, origin_y);
    let rotated_path = source_path
        .transform_similarity(&transform)
        .expect("pathological rotation remains exact");
    let source = CurveRegion2::try_from_boundary_paths(
        std::slice::from_ref(&source_path),
        &CurveContext::STRICT,
    )
    .expect("pathological source region is valid");
    let rotated = CurveRegion2::try_from_boundary_paths(
        std::slice::from_ref(&rotated_path),
        &CurveContext::STRICT,
    )
    .expect("pathological rotated region is valid");
    let source_projection = line_region(&flatten_path(&source_path));
    let rotated_projection = line_region(&flatten_path(&rotated_path));
    NativeCell {
        source_path,
        rotated_path,
        source,
        rotated,
        source_projection,
        rotated_projection,
        representations,
    }
}

pub fn rotated_region(path: &CurvePath2, index: usize) -> CurveRegion2 {
    let origin_x = i64::try_from(index % 1024).expect("cell column fits i64") * 96;
    let origin_y = i64::try_from(index / 1024).expect("cell row fits i64") * 48;
    let rotated = path
        .transform_similarity(&cell_rotation(origin_x, origin_y))
        .expect("pathological rotation remains exact");
    CurveRegion2::try_from_boundary_paths(&[rotated], &CurveContext::STRICT)
        .expect("pathological rotated region is valid")
}

fn all_family_path(
    origin_x: i64,
    origin_y: i64,
    representations: &[RepresentationSample],
) -> CurvePath2 {
    let mut curves = Vec::with_capacity(18);
    for pass in 0..2 {
        let top = pass == 1;
        for family_index in 0..8 {
            let logical_index = if top { 7 - family_index } else { family_index };
            let start_x = origin_x + i64::try_from(logical_index * 8).unwrap();
            let end_x = if top { start_x } else { start_x + 8 };
            let start_x = if top { start_x + 8 } else { start_x };
            let y = origin_y + if top { 16 } else { 0 };
            let outward = if top { 3 } else { -3 };
            let family = family_for_index(family_index);
            let representation = &representations
                [(index_mix(origin_x, origin_y) + family_index) % representations.len()]
            .value;
            curves.push(family_curve(
                family,
                point(start_x, y),
                point(end_x, y),
                outward,
                representation,
                top,
            ));
        }
        if !top {
            curves.push(line(
                point(origin_x + 64, origin_y),
                point(origin_x + 64, origin_y + 16),
            ));
        }
    }
    curves.push(line(
        point(origin_x, origin_y + 16),
        point(origin_x, origin_y),
    ));
    CurvePath2::try_new(curves).expect("all-family benchmark contour is connected")
}

fn family_for_index(index: usize) -> CurveFamily2 {
    [
        CurveFamily2::Line,
        CurveFamily2::CircularArc,
        CurveFamily2::QuadraticBezier,
        CurveFamily2::CubicBezier,
        CurveFamily2::RationalQuadraticBezier,
        CurveFamily2::RationalBezier,
        CurveFamily2::PolynomialBSpline,
        CurveFamily2::Nurbs,
    ][index]
}

fn family_curve(
    family: CurveFamily2,
    start: Point2,
    end: Point2,
    outward: i64,
    representation: &Real,
    _top: bool,
) -> Curve2 {
    let control1 = affine_control(&start, &end, 1, 3, outward);
    let control2 = affine_control(&start, &end, 2, 3, outward);
    let middle = affine_control(&start, &end, 1, 2, outward);
    let linear_control1 = affine_control(&start, &end, 1, 3, 0);
    let linear_control2 = affine_control(&start, &end, 2, 3, 0);
    match family {
        CurveFamily2::Line => line(start, end),
        CurveFamily2::CircularArc => {
            let center = Point2::new(midpoint(start.x(), end.x()), midpoint(start.y(), end.y()));
            Curve2::from(
                CircularArc2::try_from_center(start, end, center, false)
                    .expect("diameter arc is valid"),
            )
        }
        CurveFamily2::QuadraticBezier => Curve2::from(QuadraticBezier2::new(start, middle, end)),
        CurveFamily2::CubicBezier => {
            Curve2::from(CubicBezier2::new(start, control1, control2, end))
        }
        CurveFamily2::RationalQuadraticBezier => Curve2::from(
            RationalQuadraticBezier2::try_new(
                start,
                middle,
                end,
                Real::one(),
                representation.clone(),
                Real::one(),
            )
            .expect("positive rational quadratic weights are valid"),
        ),
        CurveFamily2::RationalBezier => Curve2::from(
            RationalBezier2::try_new(
                vec![start, linear_control1, linear_control2, end],
                vec![Real::one(); 4],
            )
            .expect("positive rational Bezier weights are valid"),
        ),
        CurveFamily2::PolynomialBSpline => Curve2::try_polynomial_bspline(
            3,
            vec![start, control1, control2, end],
            clamped_cubic_knots(),
        )
        .expect("clamped polynomial spline is valid"),
        CurveFamily2::Nurbs => Curve2::try_nurbs(
            3,
            vec![start, linear_control1, linear_control2, end],
            vec![Real::one(); 4],
            clamped_cubic_knots(),
        )
        .expect("clamped NURBS is valid"),
    }
}

fn index_mix(origin_x: i64, origin_y: i64) -> usize {
    usize::try_from((origin_x / 96 + origin_y / 48).rem_euclid(23)).unwrap()
}

fn affine_control(
    start: &Point2,
    end: &Point2,
    numerator: i64,
    denominator: i64,
    y: i64,
) -> Point2 {
    let t = fraction(numerator, denominator);
    Point2::new(
        start.x() + &((end.x() - start.x()) * &t),
        start.y() + &((end.y() - start.y()) * t) + Real::from(y),
    )
}

fn midpoint(left: &Real, right: &Real) -> Real {
    ((left + right) / Real::from(2)).expect("two is nonzero")
}

fn clamped_cubic_knots() -> Vec<Real> {
    vec![
        Real::zero(),
        Real::zero(),
        Real::zero(),
        Real::zero(),
        Real::one(),
        Real::one(),
        Real::one(),
        Real::one(),
    ]
}

fn cell_rotation(origin_x: i64, origin_y: i64) -> Similarity2 {
    let a = fraction(3, 5);
    let b = -fraction(4, 5);
    let d = fraction(4, 5);
    let e = fraction(3, 5);
    let center_x = Real::from(origin_x + 32);
    let center_y = Real::from(origin_y + 8);
    let xoff = &center_x - &a * &center_x - &b * &center_y + fraction(1, 3);
    let yoff = &center_y - &d * &center_x - &e * &center_y + fraction(1, 7);
    Similarity2::try_from_real_affine(a, b, d, e, xoff, yoff)
        .expect("three-four-five rotation is an exact similarity")
}

fn line(start: Point2, end: Point2) -> Curve2 {
    Curve2::from(LineSeg2::try_new(start, end).expect("benchmark line is nonzero"))
}

fn point(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

fn fraction(numerator: i64, denominator: i64) -> Real {
    (Real::from(numerator) / Real::from(denominator)).expect("nonzero denominator")
}

fn flatten_path(path: &CurvePath2) -> Vec<[f64; 2]> {
    let mut points = Vec::with_capacity(path.curves().len() * CURVE_SAMPLES);
    for (curve_index, curve) in path.curves().iter().enumerate() {
        let domain = curve.parameter_domain();
        let span = domain.end() - domain.start();
        for sample in 0..CURVE_SAMPLES {
            if curve_index > 0 && sample == 0 {
                continue;
            }
            let t = fraction(sample as i64, (CURVE_SAMPLES - 1) as i64);
            let parameter = domain.start() + &(&span * t);
            let point = curve
                .point_at(&parameter)
                .expect("benchmark curve evaluates at a rational parameter");
            points.push([
                point.x().to_f64_lossy().expect("finite x coordinate"),
                point.y().to_f64_lossy().expect("finite y coordinate"),
            ]);
        }
    }
    if points.len() > 1 && points.first() == points.last() {
        points.pop();
    }
    points
}

fn line_region(points: &[[f64; 2]]) -> LineArcRegion2 {
    let vertices = points
        .iter()
        .map(|point| {
            BulgeVertex2::new(
                Point2::new(
                    Real::try_from(point[0]).expect("finite projected x"),
                    Real::try_from(point[1]).expect("finite projected y"),
                ),
                Real::zero(),
            )
        })
        .collect::<Vec<_>>();
    let contour = Contour2::from_bulge_vertices(&vertices)
        .expect("flattened pathological contour is a valid closed polyline");
    LineArcRegion2::from_material_contours(vec![contour])
}
