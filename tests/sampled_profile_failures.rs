//! Sampled constructors must return failures before geometry reaches consumers.
#![cfg(feature = "curve")]
#[path = "support/uncertainty.rs"]
mod uncertainty;

use csgrs::{GeometryContext, Real, curve, errors::ValidationError, solid};
use hypercurve::CurveRegion2;

/// Unifies historical infallible and current fallible constructor results for assertions.
///
/// An infallible empty carrier remains successful, so it cannot satisfy a
/// regression assertion requiring an explicit construction failure.
trait Attempt {
    /// Preserves the original success or failure without interpreting empty geometry.
    fn attempt(self) -> Result<CurveRegion2, ValidationError>;
}
impl Attempt for CurveRegion2 {
    /// Wraps even empty infallible geometry as success.
    fn attempt(self) -> Result<CurveRegion2, ValidationError> {
        Ok(self)
    }
}
impl Attempt for Result<CurveRegion2, ValidationError> {
    /// Passes through the fallible result unchanged.
    fn attempt(self) -> Result<CurveRegion2, ValidationError> {
        self
    }
}
/// Promotes a finite binary64 fixture value exactly; decimal spelling is not rational decimal input.
fn r(x: f64) -> Real {
    Real::try_from(x).unwrap()
}
/// Requires an explicit invalid-input error, treating infallible empty carriers as success.
fn invalid(result: impl Attempt) {
    let error = result.attempt().expect_err("invalid input became geometry");
    assert_eq!(error.kind(), csgrs::errors::ValidationErrorKind::InvalidInput);
}
/// Checks the legacy trigonometric zero before returning an independent strict-undecided zero.
fn unresolved_zero() -> Real {
    let sine = Real::e().sin();
    let cosine = Real::e().cos();
    uncertainty::terminal_zero(&sine * &sine + &cosine * &cosine - Real::one())
}

#[test]
/// Checks a negative shaft length reports invalid input.
fn arrow_rejects_nonpositive_dimensions() {
    invalid(curve::arrow(-1, 1, 1, 2));
}
#[test]
/// Checks a negative bottom width reports invalid input.
fn trapezoid_rejects_nonpositive_dimensions() {
    invalid(curve::trapezoid(1, -1, 1, 0));
}
#[test]
/// Checks a two-tip star reports invalid input.
fn star_rejects_invalid_resolution() {
    invalid(curve::star(2, 2, 1));
}
#[test]
/// Checks a negative teardrop width reports invalid input.
fn teardrop_rejects_invalid_dimensions() {
    invalid(curve::teardrop(-1, 2, 4));
}
#[test]
/// Checks a negative egg length reports invalid input.
fn egg_rejects_invalid_dimensions() {
    invalid(curve::egg(2, -1, 4));
}
#[test]
/// Checks a two-sample squircle reports invalid input.
fn squircle_rejects_invalid_resolution() {
    invalid(curve::squircle(2, 2, 2));
}
#[test]
/// Checks a 361-degree sweep reports invalid input.
fn sector_rejects_excessive_sweep() {
    invalid(curve::pie_slice(1, 0, 361, 4));
}
#[test]
/// Checks a seven-sample heart reports invalid input.
fn heart_rejects_invalid_resolution() {
    invalid(curve::heart(2, 2, 7));
}
#[test]
/// Checks a zero superformula axis parameter reports invalid input.
fn supershape_rejects_nonpositive_axes() {
    invalid(curve::supershape(0, 1, 4, 2, 2, 2, 8));
}
#[test]
/// Checks zero `n1` retains a scalar-domain error instead of successful empty geometry.
fn scalar_domain_failure_does_not_become_empty_geometry() {
    let error = curve::supershape(1, 1, 4, 0, 2, 2, 8)
        .attempt()
        .expect_err("scalar domain failure became geometry");
    assert!(matches!(error, ValidationError::Scalar(_)));
}
#[test]
/// Checks an unresolved arrow dimension retains its typed predicate failure.
fn undecided_dimensions_do_not_become_empty_geometry() {
    let error = curve::arrow(unresolved_zero(), 1, 1, 2)
        .attempt()
        .expect_err("undecided input became geometry");
    assert!(matches!(error, ValidationError::PredicateUndecided { .. }));
}
#[test]
/// Checks doubled star counts and incremented teardrop counts report capacity overflow.
fn checked_count_overflows_do_not_become_empty_geometry() {
    for result in [
        curve::star(usize::MAX, 2, 1).attempt(),
        curve::teardrop(1, 2, usize::MAX).attempt(),
    ] {
        assert!(matches!(result, Err(ValidationError::CapacityOverflow)));
    }
}
#[test]
/// Checks a one-chord half-turn sector is certified empty and composes with union and extrusion.
fn zero_area_sampled_sector_is_a_completed_empty_set() {
    // The exact sampled vertices (0,0), (2,0), (-2,0) are collinear.
    // Regularization of that valid zero-area set is canonical empty geometry.
    let empty = curve::pie_slice_with_context(
        2.into(),
        0.into(),
        180.into(),
        1,
        GeometryContext::STRICT,
    )
    .unwrap();
    assert_eq!(empty.certainty, csgrs::GeometryCertainty::Certified);
    assert!(empty.value.is_empty());
    let square = curve::square(2).unwrap();
    let union = empty
        .value
        .boolean_region(&square, hypercurve::BooleanOp::Union)
        .unwrap();
    assert_eq!(union.filled_area().unwrap(), Some(4.into()));
    assert!(
        curve::try_extrude(&empty.value, 3.into(), GeometryContext::STRICT)
            .unwrap()
            .value
            .triangles
            .is_empty()
    );
}
#[test]
/// Checks invalid planar teardrop dimensions propagate through the solid constructor.
fn extruded_teardrop_does_not_certify_a_failed_profile() {
    assert!(
        solid::teardrop_cylinder((-1).into(), 2.into(), 3.into(), 4, GeometryContext::STRICT)
            .is_err()
    );
}
#[test]
/// Checks omitting equal-width shoulder duplicates retains exact arrow area 5/2.
fn equal_arrow_widths_preserve_the_shaft_and_head() {
    let region = curve::arrow(2, 1, 1, 1).attempt().unwrap();
    assert_eq!(
        region.filled_area().unwrap(),
        Some((Real::from(5) / Real::from(2)).unwrap())
    );
}

/// Requires one nonempty hole-free linear boundary profile for an original example.
///
/// With `svg-io` and `CSGRS_SAMPLED_PROFILE_ARTIFACTS` set, writes exact segment-start
/// expression rows to a named JSON file. Those rows supplement semantic witnesses;
/// serialization equality alone does not prove the intended set.
fn complete_fixture(label: &str, result: impl Attempt) {
    let region = result.attempt().unwrap();
    assert!(!region.is_empty(), "{label} did not complete");
    let profiles = region.boundary_profiles().unwrap();
    assert_eq!(profiles.len(), 1, "{label}");
    let (material, holes) = profiles[0].linear_contours().unwrap().unwrap();
    assert!(holes.is_empty(), "{label}");
    assert!(!material.segments().is_empty(), "{label}");
    // Exact expression rows supplement independent semantic tests; byte equality
    // alone is not an oracle for the intended set. No display demotion is used.
    #[cfg(feature = "svg-io")]
    if let Ok(directory) = std::env::var("CSGRS_SAMPLED_PROFILE_ARTIFACTS") {
        let rows = material
            .segments()
            .iter()
            .map(|edge| {
                format!(
                    "[{},{}]",
                    edge.start().x().to_json(),
                    edge.start().y().to_json()
                )
            })
            .collect::<Vec<_>>();
        let directory = std::path::Path::new(&directory);
        std::fs::create_dir_all(directory).unwrap();
        std::fs::write(
            directory.join(format!("{label}.json")),
            format!("[{}]\n", rows.join(",\n")),
        )
        .unwrap();
    }
}
/// Generates a named original-example regression using the shared profile assertions.
macro_rules! fixture {
    ($name:ident, $value:expr) => {
        /// Checks this named original-example input with [`complete_fixture`].
        #[test]
        fn $name() {
            complete_fixture(stringify!($name), $value);
        }
    };
}
fixture!(
    original_readme_arrow,
    curve::arrow(r(2.2), r(0.35), r(0.8), r(1.0))
);
fixture!(
    original_readme_trapezoid,
    curve::trapezoid(r(1.2), r(2.2), r(1.4), r(0.45))
);
fixture!(original_readme_star, curve::star(5, r(1.1), r(0.45)));
fixture!(original_readme_teardrop, curve::teardrop(r(1.5), r(2.3), 96));
fixture!(original_readme_egg, curve::egg(r(1.6), r(2.2), 96));
fixture!(original_readme_squircle, curve::squircle(r(2.0), r(1.6), 128));
fixture!(
    original_readme_sector,
    curve::pie_slice(r(1.1), r(-35.0), r(115.0), 64)
);
fixture!(original_readme_heart, curve::heart(r(2.0), r(1.8), 160));
fixture!(
    original_readme_supershape,
    curve::supershape(r(1.0), r(1.0), r(5.0), r(2.0), r(7.0), r(7.0), 160)
);

/// Returns an exact integer ratio; fixture denominators must be nonzero.
fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}

#[test]
/// Checks nine independent sampled-area expectations and certified receipts under both policies.
///
/// Each result also passes through empty union, self-difference and empty extrusion.
/// Area agreement supplements these checks without proving universal set equality.
fn independent_sampled_areas_and_context_receipts_agree() {
    use csgrs::GeometryCertainty;
    let sqrt2 = Real::from(2).sqrt().unwrap();
    // Shoelace area of the eight unscaled heart samples is 328*sqrt(2).
    // Their independent spans are 32 and 18 + 15*sqrt(2)/2.
    let heart_area = (Real::from(4 * 328) * &sqrt2
        / (Real::from(32) * (Real::from(18) + q(15, 2) * &sqrt2)))
        .unwrap();
    for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
        for (name, result, expected) in [
            (
                "arrow",
                curve::arrow_with_context(2.into(), 1.into(), 1.into(), 2.into(), context),
                Real::from(3),
            ),
            (
                "trapezoid",
                curve::trapezoid_with_context(1.into(), 3.into(), 2.into(), 5.into(), context),
                Real::from(4),
            ),
            (
                "star",
                curve::star_with_context(4, 2.into(), 1.into(), context),
                Real::from(4) * &sqrt2,
            ),
            (
                "teardrop",
                curve::teardrop_with_context(2.into(), 3.into(), 2, context),
                Real::from(3),
            ),
            (
                "egg",
                curve::egg_with_context(2.into(), 4.into(), 4, context),
                Real::from(4),
            ),
            (
                "squircle",
                curve::squircle_with_context(4.into(), 2.into(), 4, context),
                Real::from(4),
            ),
            (
                "sector",
                curve::pie_slice_with_context(2.into(), 0.into(), 90.into(), 1, context),
                Real::from(2),
            ),
            (
                "heart",
                curve::heart_with_context(2.into(), 2.into(), 8, context),
                heart_area.clone(),
            ),
            (
                "supershape",
                curve::supershape_with_context(
                    1.into(),
                    1.into(),
                    4.into(),
                    2.into(),
                    2.into(),
                    2.into(),
                    4,
                    context,
                ),
                Real::from(2),
            ),
        ] {
            let result = result.unwrap_or_else(|error| panic!("{name}: {error}"));
            assert_eq!(result.certainty, GeometryCertainty::Certified, "{name}");
            let actual = result.value.filled_area().unwrap().unwrap();
            assert_eq!(
                actual.certified_eq_until(&expected, -1024).as_bool(),
                Some(true),
                "{name}"
            );
            let restored = result
                .value
                .boolean_region(&curve::empty(), hypercurve::BooleanOp::Union)
                .unwrap();
            let removed = restored
                .boolean_region(&result.value, hypercurve::BooleanOp::Difference)
                .unwrap();
            assert!(removed.is_empty(), "{name}");
            assert!(
                curve::try_extrude(&removed, 3.into(), context)
                    .unwrap()
                    .value
                    .triangles
                    .is_empty()
            );
        }
    }
}

#[test]
/// Checks zero dimensions, equal star radii, teardrop collapse and zero sweep remain invalid inputs.
fn positive_shape_ranges_and_zero_dimensions_are_explicit_errors() {
    for result in [
        curve::arrow(0, 1, 1, 2),
        curve::trapezoid(0, 1, 1, 0),
        curve::star(3, 1, 1),
        curve::teardrop(2, 1, 2),
        curve::egg(1, 0, 3),
        curve::squircle(0, 1, 3),
        curve::pie_slice(1, 0, 0, 1),
        curve::heart(1, 0, 8),
    ] {
        assert_eq!(
            result.unwrap_err().kind(),
            csgrs::errors::ValidationErrorKind::InvalidInput
        );
    }
}

#[test]
/// Checks maximal sample counts fail across seven constructors, then a normal egg still succeeds.
fn every_input_controlled_sampling_buffer_checks_capacity() {
    for result in [
        curve::star(usize::MAX, 2, 1),
        curve::teardrop(1, 2, usize::MAX),
        curve::egg(1, 2, usize::MAX),
        curve::squircle(1, 2, usize::MAX),
        curve::pie_slice(1, 0, 90, usize::MAX),
        curve::heart(1, 2, usize::MAX),
        curve::supershape(1, 1, 4, 2, 2, 2, usize::MAX),
    ] {
        assert!(matches!(result, Err(ValidationError::CapacityOverflow)));
    }
    assert!(!curve::egg(1, 2, 4).unwrap().is_empty());
}

#[test]
/// Checks approximate shoulder coalescing stays qualified when it reuses a strict ring cache entry.
///
/// Strict requests fail before and after the approximate request; area remains 5/2.
fn approximate_shoulder_equality_stays_qualified_across_cache_hits() {
    let head_width = Real::one() + unresolved_zero();
    assert!(
        curve::arrow_with_context(
            2.into(),
            1.into(),
            1.into(),
            head_width.clone(),
            GeometryContext::STRICT
        )
        .is_err()
    );
    // Populate the identical five-point strict ring before using qualified
    // shoulder equality. A ring-cache proof must not erase that dependency.
    curve::arrow(2, 1, 1, 1).unwrap();
    let result = curve::arrow_with_context(
        2.into(),
        1.into(),
        1.into(),
        head_width.clone(),
        GeometryContext::APPROXIMATE_512,
    )
    .unwrap();
    assert_eq!(
        result.certainty,
        csgrs::GeometryCertainty::Approximate512Consumed
    );
    assert_eq!(result.value.filled_area().unwrap(), Some(q(5, 2)));
    assert!(
        curve::arrow_with_context(
            2.into(),
            1.into(),
            1.into(),
            head_width,
            GeometryContext::STRICT
        )
        .is_err()
    );
}

#[test]
/// Checks a perforated arrow and sampled teardrop extrude to certified solids of volume nine.
///
/// Geometric manifold admission and exact determinant sums check both outputs.
/// With `obj-io`, the dyadic arrow fixture also retains positions and volume through
/// OBJ interchange; this does not establish exact replay for general OBJ geometry.
fn profile_hole_extrusion_and_dyadic_interchange_keep_exact_volume() {
    let arrow = curve::arrow(2, 1, 1, 1).unwrap();
    let cutter =
        curve::translated(&curve::square(q(1, 2)).unwrap(), q(1, 2), q(-1, 4)).unwrap();
    let cut = arrow
        .boolean_region(&cutter, hypercurve::BooleanOp::Difference)
        .unwrap();
    assert_eq!(cut.filled_area().unwrap(), Some(q(9, 4)));
    let extruded = curve::try_extrude(&cut, 4.into(), GeometryContext::STRICT).unwrap();
    assert_eq!(extruded.certainty, csgrs::GeometryCertainty::Certified);
    let check = |mesh: &csgrs::TriangleMesh| {
        assert!(
            mesh.is_closed_manifold_geometry(&GeometryContext::STRICT.mesh_context())
                .unwrap()
                .value
        );
        let six_volume = mesh.triangles.iter().fold(Real::zero(), |sum, triangle| {
            let [a, b, c] = triangle.indices().map(|i| mesh.positions[i].to_vector());
            sum + a.dot(&b.cross(&c))
        });
        assert_eq!(six_volume, Real::from(54));
    };
    check(&extruded.value);
    #[cfg(feature = "obj-io")]
    {
        let text = csgrs::io::obj::to_obj(&extruded.value, "sampled-arrow-hole").unwrap();
        let restored =
            csgrs::io::obj::from_obj_with_context(text.as_bytes(), GeometryContext::STRICT)
                .unwrap();
        assert_eq!(restored.certainty, csgrs::GeometryCertainty::Certified);
        assert_eq!(restored.value.positions, extruded.value.positions);
        check(&restored.value);
    }
    let solid =
        solid::teardrop_cylinder(2.into(), 3.into(), 3.into(), 2, GeometryContext::STRICT)
            .unwrap();
    assert_eq!(solid.certainty, csgrs::GeometryCertainty::Certified);
    check(&solid.value); // sampled area 3, height 3, volume 9
}

#[test]
/// Checks signed quarter-turn areas and the minimum three-vertex full-turn sampling area.
fn clockwise_and_full_turn_sectors_retain_their_sampling_contract() {
    for end in [-90, 90] {
        assert_eq!(
            curve::pie_slice(2, 0, end, 1).unwrap().filled_area().unwrap(),
            Some(2.into())
        );
    }
    let expected = Real::from(3) * Real::from(3).sqrt().unwrap();
    for end in [-360, 360] {
        let full = curve::pie_slice(2, 0, end, 1).unwrap();
        assert_eq!(
            full.filled_area()
                .unwrap()
                .unwrap()
                .certified_eq_until(&expected, -1024)
                .as_bool(),
            Some(true)
        );
    }
}
