//! Fixed Criterion workloads for scalar and small exact-linear-algebra regressions.
//!
//! These 27 named cases isolate symbolic cancellation, sign refinement, vector
//! reductions, shared-denominator recognition, matrix inversion/products and
//! homogeneous transforms. They are timing sentinels, not correctness tests:
//! returned facts, signs and coordinates are not compared with expected values,
//! and no dispatch counts are asserted. Unwrapped errors fail by panic; timings
//! alone do not establish successful certification, a particular internal path,
//! or preservation of the full supported input domain.
//!
//! Most rows retain fixtures outside `iter`, so repeated calls can reuse scalar
//! evidence. Clones inside timed closures remain part of the measurement. The
//! four `iter_batched` rows construct fresh inputs outside the measured routine
//! using Criterion's SmallInput batching. In the locked Criterion implementation,
//! ordinary `iter` includes returned-value destruction; `iter_batched` drops
//! returned outputs after its measured interval, while consumed input destruction
//! can occur inside the routine. Thus these are different workload boundaries.
//! Fresh construction does not promise process-wide cache eviction, cold hardware
//! caches, or an allocation-free computation. Fixed input dimensions and the sign
//! precision argument do not impose total work, memory or wall-clock limits.
//!
//! The generated `benches` function configures Criterion defaults from argv and
//! calls the sentinel suite followed by the shared report callback. The generated
//! `main` calls that group and then Criterion's final summary. There are no local
//! sample-count or measurement-time overrides. The callback can rewrite the
//! crate's benchmarks.md from stored results, including older or unrelated rows;
//! setting HYPERLATTICE_SKIP_BENCHMARK_REPORTS suppresses that write but does not
//! suppress the benchmarks or Criterion's own output files.

use criterion::{Criterion, criterion_group, criterion_main};
use hyperlattice::{Complex, Matrix3, Matrix4, Real, SharedScaleVec, Vector3, Vector4};
use std::hint::black_box;

/// Shared stored-result report writer, invoked after the sentinel registrations.
///
/// Its environment/argv gate controls Markdown writes, and I/O errors are logged
/// without failing the benchmark. It does not validate the timed computations.
#[path = "support/benchmark_report.rs"]
mod benchmark_report;

/// Embed a signed 32-bit fixture integer as an exact native scalar.
///
/// Uses integer conversion directly; no floating-point ingress or rounding occurs.
fn r(value: i32) -> Real {
    value.into()
}

/// Construct an exact reduced rational fixture from a signed numerator and divisor.
///
/// Panics if the denominator is zero. Every caller in this suite supplies a
/// positive denominator; this is a fixture convenience, not a fallible public API.
fn frac(numerator: i64, denominator: u64) -> Real {
    hyperlattice::Rational::fraction(numerator, denominator)
        .unwrap()
        .into()
}

/// Reconstruct the same three nonzero dyadic coordinates for norm workloads.
///
/// The coordinates are 123456789012345 / 2^50, -234567890123457 / 2^49 and
/// 345678901234569 / 2^48. Their odd numerators leave different reduced
/// denominators, while a power-of-two common scale remains available for exact
/// arithmetic. The squared norm is strictly positive, so magnitude, reciprocal
/// magnitude and normalization are mathematically in-domain.
///
/// Each call constructs a new vector rather than cloning a retained one. The
/// function does not flush caches or itself establish a measured cold-state
/// guarantee; both retained and freshly batched cases use this constructor.
fn cold_self_dot_vector() -> Vector3 {
    Vector3::new([
        frac(123_456_789_012_345, 1_u64 << 50),
        frac(-234_567_890_123_457, 1_u64 << 49),
        frac(345_678_901_234_569, 1_u64 << 48),
    ])
}

/// Register the 27 fixed workloads with the caller's Criterion configuration.
///
/// Row names below omit the `sentinel/` prefix. Except for the four rows explicitly
/// marked fresh, fixture construction precedes the timed iteration. Each row
/// returns its operation result to Criterion, without an independent value oracle.
///
/// | Row suffix | Operation and input meaning |
/// | --- | --- |
/// | scalar/cancellation_zero_status | Inspect the retained expression ((pi*e)/e)-pi, mathematically zero; construction and division precede timing. |
/// | scalar/sqrt2_minus_convergent_sign | Refine the retained sqrt(2)-99/70 sign to precision -128; it is negative since (99/70)^2=2+1/4900. The returned Option is not unwrapped. |
/// | complex/powi_zero_known_nonzero | Clone 3+4i and unwrap its zeroth power, mathematically one; includes the known-nonzero base check. |
/// | vector/dot_sparse_symbolic | Dot (pi,0,sqrt(2)) with (2/3,e,0), mathematically 2*pi/3. |
/// | vector/self_dot_retained_dyadic | Squared norm of the retained dyadic vector. |
/// | vector/magnitude_retained_dyadic | Unwrap that vector's Euclidean norm. |
/// | vector/inverse_magnitude_retained_dyadic | Unwrap its norm and then the scalar reciprocal, rather than a dedicated inverse-norm API. |
/// | vector/normalize_retained_dyadic | Unwrap normalization of the retained vector. |
/// | vector/normalize_cold_dyadic | Fresh dyadic vector per batched setup; unwrap normalization in the routine. |
/// | vector/self_dot_cold_dyadic | Fresh dyadic vector per batched setup; compute its squared norm. |
/// | vector/dot_equal_distinct_cold_dyadic | Independently construct two equal dyadic vectors per setup and dot them; no shared allocation identity is required. |
/// | vector/shared_scale_view_common_denominator | Borrow a shared-scale view of (1,-2,3,4)/7; the Option is returned unchecked. |
/// | vector/shared_scale_owned_common_denominator | Fresh component array per setup; consume it into an owned shared scale, expecting equal reduced denominators of seven. |
/// | matrix3/inverse_fractional | Clone and invert a dense nonsingular rational 3x3 matrix; unwrap the checked inverse. |
/// | matrix4/exact_facts | Inspect structural facts of a retained dense dyadic 4x4 matrix; no determinant or inverse is requested. |
/// | matrix3/sparse_mask_product | Borrowed product of upper- and lower-bidiagonal rational 3x3 inputs; no mask schedule is asserted. |
/// | matrix3/dense_transform_batch_public | Transform four rational 3-vectors with the same dense matrix used by inverse_fractional, allocating an output batch. |
/// | matrix4/division_fractional | Clone identity and a dense nonsingular rational divisor, then compute checked right division I/B=B^-1 and unwrap. |
/// | matrix4/determinant_fractional | Compute the determinant of the same divisor fixture through a borrowed receiver. |
/// | matrix4/sparse_mask_product | Borrowed product of upper- and lower-bidiagonal rational 4x4 inputs. |
/// | matrix4/translated_diagonal_direction_transform_public | Apply diag(2,3,4) plus translation (100,200,300) to (5,7,11,0) through the direction API; translation contributes zero. |
/// | matrix4/translated_diagonal_point_transform_public | Apply that translated scaling to (5,7,11,1) through the point API. |
/// | matrix4/translated_diagonal_direction_batch_public | Four w=0 vectors through the general homogeneous batch API, not the direction-only batch API. |
/// | matrix4/translated_diagonal_point_batch_public | Four w=1 vectors through the general homogeneous batch API, not the point-only batch API. |
/// | matrix4/diagonal_direction_batch | Four w=0 vectors through the direction-only batch API with scaling (2,3,4) and zero translation. |
/// | matrix4/diagonal_point_batch | Four w=1 vectors through the point-only batch API with that untranslated scaling. |
/// | matrix4/diagonal_unknown_batch | General homogeneous batch with exact weights 1/2,1/3,1/5,1/7. "Unknown" means neither the zero-weight nor unit-weight specialization, not an undecided scalar value. |
///
/// Homogeneous transform outputs keep their fourth coordinate; the suite does not
/// request Cartesian perspective division. The batch rows each produce four
/// vectors per iteration, whereas the single-vector rows produce one. Existing
/// fixture construction and operation unwraps panic on errors; non-unwrapped facts,
/// signs and view Options do not make unsuccessful or unresolved results fail.
/// No Hyperlimit policy or certainty receipt is selected by this harness.
fn bench_regression_sentinels(c: &mut Criterion) {
    c.bench_function("sentinel/scalar/cancellation_zero_status", |b| {
        let value: Real = ((Real::pi() * Real::e()) / Real::e()).unwrap() - Real::pi();
        b.iter(|| value.zero_status())
    });

    c.bench_function("sentinel/scalar/sqrt2_minus_convergent_sign", |b| {
        let value = Real::sqrt(r(2)).unwrap() - frac(99, 70);
        b.iter(|| value.refine_sign_until(-128))
    });

    c.bench_function("sentinel/complex/powi_zero_known_nonzero", |b| {
        let value = Complex::new(r(3), r(4));
        b.iter(|| black_box(value.clone()).powi(0).unwrap())
    });

    c.bench_function("sentinel/vector/dot_sparse_symbolic", |b| {
        let left = Vector3::new([Real::pi(), r(0), Real::sqrt(r(2)).unwrap()]);
        let right = Vector3::new([frac(2, 3), Real::e(), r(0)]);
        b.iter(|| left.dot(&right))
    });

    c.bench_function("sentinel/vector/self_dot_retained_dyadic", |b| {
        let vector = cold_self_dot_vector();
        b.iter(|| black_box(&vector).norm_squared())
    });

    c.bench_function("sentinel/vector/magnitude_retained_dyadic", |b| {
        let vector = cold_self_dot_vector();
        b.iter(|| black_box(&vector).magnitude().unwrap())
    });

    c.bench_function("sentinel/vector/inverse_magnitude_retained_dyadic", |b| {
        let vector = cold_self_dot_vector();
        b.iter(|| black_box(&vector).magnitude().unwrap().inverse().unwrap())
    });

    c.bench_function("sentinel/vector/normalize_retained_dyadic", |b| {
        let vector = cold_self_dot_vector();
        b.iter(|| black_box(&vector).normalize().unwrap())
    });

    c.bench_function("sentinel/vector/normalize_cold_dyadic", |b| {
        b.iter_batched(
            cold_self_dot_vector,
            |vector| black_box(vector).normalize().unwrap(),
            criterion::BatchSize::SmallInput,
        )
    });

    c.bench_function("sentinel/vector/self_dot_cold_dyadic", |b| {
        b.iter_batched(
            cold_self_dot_vector,
            |vector| black_box(vector).norm_squared(),
            criterion::BatchSize::SmallInput,
        )
    });

    c.bench_function("sentinel/vector/dot_equal_distinct_cold_dyadic", |b| {
        b.iter_batched(
            || (cold_self_dot_vector(), cold_self_dot_vector()),
            |(left, right)| black_box(left).dot(black_box(&right)),
            criterion::BatchSize::SmallInput,
        )
    });

    c.bench_function(
        "sentinel/vector/shared_scale_view_common_denominator",
        |b| {
            let vector = Vector4::new([frac(1, 7), frac(-2, 7), frac(3, 7), frac(4, 7)]);
            b.iter(|| vector.shared_scale_view())
        },
    );

    c.bench_function(
        "sentinel/vector/shared_scale_owned_common_denominator",
        |b| {
            b.iter_batched(
                || [frac(1, 7), frac(-2, 7), frac(3, 7), frac(4, 7)],
                |components| {
                    black_box(SharedScaleVec::from_components(components))
                        .expect("sevenths share a reduced denominator")
                },
                criterion::BatchSize::SmallInput,
            )
        },
    );

    c.bench_function("sentinel/matrix3/inverse_fractional", |b| {
        let matrix = Matrix3::new([
            [frac(9, 8), frac(3, 16), frac(-5, 8)],
            [frac(7, 4), frac(-11, 8), frac(13, 16)],
            [frac(5, 8), frac(17, 16), frac(19, 8)],
        ]);
        b.iter(|| matrix.clone().inverse_checked().unwrap())
    });

    c.bench_function("sentinel/matrix4/exact_facts", |b| {
        let matrix = Matrix4::new([
            [frac(9, 8), frac(3, 16), frac(-5, 8), frac(7, 4)],
            [frac(-11, 8), frac(13, 16), frac(5, 8), frac(17, 16)],
            [frac(19, 8), frac(-23, 16), frac(29, 8), frac(31, 16)],
            [frac(37, 8), frac(41, 16), frac(-43, 8), frac(47, 16)],
        ]);
        b.iter(|| black_box(&matrix).exact_facts())
    });

    c.bench_function("sentinel/matrix3/sparse_mask_product", |b| {
        let left = Matrix3::new([
            [frac(2, 3), frac(3, 5), r(0)],
            [r(0), frac(5, 7), frac(7, 11)],
            [r(0), r(0), frac(11, 13)],
        ]);
        let right = Matrix3::new([
            [frac(13, 17), r(0), r(0)],
            [frac(17, 19), frac(19, 23), r(0)],
            [r(0), frac(23, 29), frac(29, 31)],
        ]);
        b.iter(|| black_box(black_box(&left) * black_box(&right)))
    });

    c.bench_function("sentinel/matrix3/dense_transform_batch_public", |b| {
        let transform = Matrix3::new([
            [frac(9, 8), frac(3, 16), frac(-5, 8)],
            [frac(7, 4), frac(-11, 8), frac(13, 16)],
            [frac(5, 8), frac(17, 16), frac(19, 8)],
        ]);
        let vectors = vec![
            Vector3::new([frac(2, 3), frac(5, 7), frac(11, 13)]),
            Vector3::new([frac(17, 19), frac(23, 29), frac(31, 37)]),
            Vector3::new([frac(41, 43), frac(47, 53), frac(59, 61)]),
            Vector3::new([frac(67, 71), frac(73, 79), frac(83, 89)]),
        ];
        b.iter(|| transform.transform_vec3_batch(&vectors))
    });

    c.bench_function("sentinel/matrix4/division_fractional", |b| {
        let numerator = Matrix4::identity();
        let divisor = Matrix4::new([
            [frac(11, 10), frac(2, 10), frac(3, 10), frac(4, 10)],
            [frac(5, 10), frac(17, 10), frac(7, 10), frac(-8, 10)],
            [frac(9, 10), frac(-10, 10), frac(23, 10), frac(12, 10)],
            [frac(-13, 10), frac(14, 10), frac(-15, 10), frac(19, 10)],
        ]);
        b.iter(|| {
            numerator
                .clone()
                .div_matrix_checked(divisor.clone())
                .unwrap()
        })
    });

    c.bench_function("sentinel/matrix4/determinant_fractional", |b| {
        let matrix = Matrix4::new([
            [frac(11, 10), frac(2, 10), frac(3, 10), frac(4, 10)],
            [frac(5, 10), frac(17, 10), frac(7, 10), frac(-8, 10)],
            [frac(9, 10), frac(-10, 10), frac(23, 10), frac(12, 10)],
            [frac(-13, 10), frac(14, 10), frac(-15, 10), frac(19, 10)],
        ]);
        b.iter(|| black_box(black_box(&matrix).determinant()))
    });

    c.bench_function("sentinel/matrix4/sparse_mask_product", |b| {
        let left = Matrix4::new([
            [frac(2, 3), frac(3, 5), r(0), r(0)],
            [r(0), frac(5, 7), frac(7, 11), r(0)],
            [r(0), r(0), frac(11, 13), frac(13, 17)],
            [r(0), r(0), r(0), frac(17, 19)],
        ]);
        let right = Matrix4::new([
            [frac(19, 23), r(0), r(0), r(0)],
            [frac(23, 29), frac(29, 31), r(0), r(0)],
            [r(0), frac(31, 37), frac(37, 41), r(0)],
            [r(0), r(0), frac(41, 43), frac(43, 47)],
        ]);
        b.iter(|| black_box(black_box(&left) * black_box(&right)))
    });

    c.bench_function(
        "sentinel/matrix4/translated_diagonal_direction_transform_public",
        |b| {
            let transform = Matrix4::new([
                [r(2), r(0), r(0), r(100)],
                [r(0), r(3), r(0), r(200)],
                [r(0), r(0), r(4), r(300)],
                [r(0), r(0), r(0), r(1)],
            ]);
            let direction = Vector4::new([r(5), r(7), r(11), r(0)]);
            b.iter(|| transform.transform_vec4_direction(&direction))
        },
    );

    c.bench_function(
        "sentinel/matrix4/translated_diagonal_point_transform_public",
        |b| {
            let transform = Matrix4::new([
                [r(2), r(0), r(0), r(100)],
                [r(0), r(3), r(0), r(200)],
                [r(0), r(0), r(4), r(300)],
                [r(0), r(0), r(0), r(1)],
            ]);
            let point = Vector4::new([r(5), r(7), r(11), r(1)]);
            b.iter(|| transform.transform_vec4_point(&point))
        },
    );

    c.bench_function(
        "sentinel/matrix4/translated_diagonal_direction_batch_public",
        |b| {
            let transform = Matrix4::new([
                [r(2), r(0), r(0), r(100)],
                [r(0), r(3), r(0), r(200)],
                [r(0), r(0), r(4), r(300)],
                [r(0), r(0), r(0), r(1)],
            ]);
            let vectors = vec![
                Vector4::new([r(5), r(7), r(11), r(0)]),
                Vector4::new([r(13), r(17), r(19), r(0)]),
                Vector4::new([r(23), r(29), r(31), r(0)]),
                Vector4::new([r(37), r(41), r(43), r(0)]),
            ];
            b.iter(|| transform.transform_vec4_batch(&vectors))
        },
    );

    c.bench_function(
        "sentinel/matrix4/translated_diagonal_point_batch_public",
        |b| {
            let transform = Matrix4::new([
                [r(2), r(0), r(0), r(100)],
                [r(0), r(3), r(0), r(200)],
                [r(0), r(0), r(4), r(300)],
                [r(0), r(0), r(0), r(1)],
            ]);
            let vectors = vec![
                Vector4::new([r(5), r(7), r(11), r(1)]),
                Vector4::new([r(13), r(17), r(19), r(1)]),
                Vector4::new([r(23), r(29), r(31), r(1)]),
                Vector4::new([r(37), r(41), r(43), r(1)]),
            ];
            b.iter(|| transform.transform_vec4_batch(&vectors))
        },
    );

    c.bench_function("sentinel/matrix4/diagonal_direction_batch", |b| {
        let transform = Matrix4::new([
            [r(2), r(0), r(0), r(0)],
            [r(0), r(3), r(0), r(0)],
            [r(0), r(0), r(4), r(0)],
            [r(0), r(0), r(0), r(1)],
        ]);
        let vectors = vec![
            Vector4::new([r(5), r(7), r(11), r(0)]),
            Vector4::new([r(13), r(17), r(19), r(0)]),
            Vector4::new([r(23), r(29), r(31), r(0)]),
            Vector4::new([r(37), r(41), r(43), r(0)]),
        ];
        b.iter(|| transform.transform_vec4_direction_batch(&vectors))
    });

    c.bench_function("sentinel/matrix4/diagonal_point_batch", |b| {
        let transform = Matrix4::new([
            [r(2), r(0), r(0), r(0)],
            [r(0), r(3), r(0), r(0)],
            [r(0), r(0), r(4), r(0)],
            [r(0), r(0), r(0), r(1)],
        ]);
        let vectors = vec![
            Vector4::new([r(5), r(7), r(11), r(1)]),
            Vector4::new([r(13), r(17), r(19), r(1)]),
            Vector4::new([r(23), r(29), r(31), r(1)]),
            Vector4::new([r(37), r(41), r(43), r(1)]),
        ];
        b.iter(|| transform.transform_vec4_point_batch(&vectors))
    });

    c.bench_function("sentinel/matrix4/diagonal_unknown_batch", |b| {
        let transform = Matrix4::new([
            [r(2), r(0), r(0), r(0)],
            [r(0), r(3), r(0), r(0)],
            [r(0), r(0), r(4), r(0)],
            [r(0), r(0), r(0), r(1)],
        ]);
        let vectors = vec![
            Vector4::new([r(5), r(7), r(11), frac(1, 2)]),
            Vector4::new([r(13), r(17), r(19), frac(1, 3)]),
            Vector4::new([r(23), r(29), r(31), frac(1, 5)]),
            Vector4::new([r(37), r(41), r(43), frac(1, 7)]),
        ];
        b.iter(|| transform.transform_vec4_batch(&vectors))
    });
}

criterion_group!(
    benches,
    bench_regression_sentinels,
    benchmark_report::finish_benchmark_report
);
criterion_main!(benches);
