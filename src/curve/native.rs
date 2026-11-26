//! CSG grammar over native Hypercurve carriers.
//!
//! Filled construction returns [`CurveRegion2`]. Open geometry is returned as
//! [`CurveString2`] or [`CurvePath2`] by the APIs that create it; csgrs does not
//! combine those distinct native meanings in a public profile facade.

use crate::context::GeometryDecisions;
use crate::errors::ValidationError;
use crate::session::IntoReal;
use crate::solid;
use crate::{GeometryContext, GeometryOutcome};
use hypercurve::{
    BooleanOp, Contour2, CubicBezier2, Curve2, CurveGeometry2, CurvePath2,
    CurvePathRegionTrim2, CurveRegion2, CurveString2, FiniteProjectionOptions,
    FiniteRegionProfile2, LineSeg2, Point2, PolynomialSplineCurve2, QuadraticBezier2,
    RationalBezier2, RegionPointLocation, Segment2,
};
use hyperlattice::{Aabb, Matrix4, Point3, Real, Vector3};
use hypermesh::TriangleMesh;
use std::cell::RefCell;
use std::cmp::Ordering;

thread_local! {
    /// Up to 64 successful strict ring constructions, oldest insertion first.
    ///
    /// Keys own the exact coordinate rows and are compared with `Real` equality,
    /// without a policy-directed geometric equivalence query. Hits do not refresh
    /// insertion order. This bounds entry count, not retained bytes or proof work.
    static REGION_RING_CACHE: RefCell<Vec<(Vec<[Real; 2]>, CurveRegion2)>> =
        const { RefCell::new(Vec::new()) };
}

/// Requires a positive dimension under the selected policy and records its certainty.
///
/// Zero and negative decisions become `NonPositiveParameter`; an unresolved
/// decision propagates its error instead of being classified as nonpositive.
fn positive_with(
    value: &Real,
    name: &'static str,
    decisions: &GeometryDecisions,
) -> Result<(), ValidationError> {
    match decisions.sign(value, name)? {
        hyperreal::RealSign::Positive => Ok(()),
        _ => Err(ValidationError::NonPositiveParameter { name }),
    }
}

/// Rejects a negative dimension and returns its policy-selected sign for collapse checks.
///
/// The shared decisions object retains any approximation consumed by the sign
/// query. Unresolved signs propagate as errors.
fn nonnegative_with(
    value: &Real,
    name: &'static str,
    decisions: &GeometryDecisions,
) -> Result<hyperreal::RealSign, ValidationError> {
    match decisions.sign(value, name)? {
        hyperreal::RealSign::Negative => Err(ValidationError::NegativeParameter { name }),
        sign => Ok(sign),
    }
}

/// Orders curve dimensions under the selected policy, retaining certainty or failure.
fn compare_with(
    left: &Real,
    right: &Real,
    decisions: &GeometryDecisions,
) -> Result<Ordering, ValidationError> {
    decisions.decide(
        hyperlimit::compare_reals(left, right, decisions.predicate_policy()),
        "curve dimensions",
    )
}

/// Admits coordinate rows as one native material contour, reusing strict cached results.
///
/// Cache misses run contour construction and region admission through the shared
/// decision scope. Only successful strict requests insert entries; approximate
/// requests may reuse those entries but cannot populate the cache. The native
/// constructors own ring validity and filled-region semantics.
fn ring_with(
    points: &[[Real; 2]],
    decisions: &GeometryDecisions,
) -> Result<CurveRegion2, ValidationError> {
    if let Some(region) = REGION_RING_CACHE.with_borrow(|entries| {
        entries
            .iter()
            .rev()
            .find(|(key, _)| key == points)
            .map(|(_, region)| region.clone())
    }) {
        return Ok(region);
    }
    let region = decisions.exact_curve(|| -> Result<_, ValidationError> {
        let contour = Contour2::from_real_ring(points)?;
        Ok(CurveRegion2::try_from_native_material_contours(vec![
            contour,
        ])?)
    })?;
    // Only successful strict proofs enter this thread-local, 64-entry FIFO.
    // Owned exact coordinate keys and cloned regions preserve input identity.
    // Approximate requests may reuse a strict proof, but never populate it.
    if decisions.predicate_policy() == hyperlimit::PredicatePolicy::STRICT {
        REGION_RING_CACHE.with_borrow_mut(|entries| {
            if entries.len() == 64 {
                entries.remove(0);
            }
            entries.push((points.to_vec(), region.clone()));
        });
    }
    Ok(region)
}

/// Builds the positive-axis rectangle after checking width and then height.
///
/// Both dimensions are checked before a policy-selected zero yields canonical
/// empty geometry. The caller retains the shared decision receipt.
fn rectangle_with(
    width: Real,
    height: Real,
    decisions: &GeometryDecisions,
) -> Result<CurveRegion2, ValidationError> {
    let width_sign = nonnegative_with(&width, "width", decisions)?;
    let height_sign = nonnegative_with(&height, "height", decisions)?;
    if width_sign == hyperreal::RealSign::Zero || height_sign == hyperreal::RealSign::Zero {
        return Ok(empty());
    }
    ring_with(
        &[
            [Real::zero(), Real::zero()],
            [width.clone(), Real::zero()],
            [width, height.clone()],
            [Real::zero(), height],
        ],
        decisions,
    )
}

/// Connects equally spaced angular samples of an origin-centered ellipse as a ring.
///
/// Callers validate radii and sample count. Samples start at `(radius_x, 0)`
/// and increase angle through one turn without duplicating the endpoint. Exact
/// sine/cosine expressions define polygon vertices, not analytic ellipse arcs;
/// no ellipse-to-polygon fidelity bound is established here.
fn ellipse_points_with(
    radius_x: Real,
    radius_y: Real,
    segments: usize,
    decisions: &GeometryDecisions,
) -> Result<CurveRegion2, ValidationError> {
    let denominator = Real::from(segments as u64);
    let mut points = solid::reserved(segments)?;
    for index in 0..segments {
        let fraction = (Real::from(index as u64) / &denominator)?;
        let angle = Real::tau() * fraction;
        points.push([
            radius_x.clone() * angle.clone().cos(),
            radius_y.clone() * angle.sin(),
        ]);
    }
    ring_with(&points, decisions)
}

/// Checks sample count and capacity before radius sign, then builds a sampled circle.
///
/// At least three samples are required even when a zero radius returns empty.
fn circle_with(
    radius: Real,
    segments: usize,
    decisions: &GeometryDecisions,
) -> Result<CurveRegion2, ValidationError> {
    solid::sample_minimum(segments, 3, "segments")?;
    solid::capacity::<[Real; 2]>(Some(segments))?;
    if nonnegative_with(&radius, "radius", decisions)? == hyperreal::RealSign::Zero {
        return Ok(empty());
    }
    ellipse_points_with(radius.clone(), radius, segments, decisions)
}

/// Applies native exact affine translation inside the shared curve decision scope.
fn translate_with(
    region: &CurveRegion2,
    x: Real,
    y: Real,
    decisions: &GeometryDecisions,
) -> Result<CurveRegion2, ValidationError> {
    Ok(decisions.exact_curve(|| {
        region.transform_affine(
            &Real::one(),
            &Real::zero(),
            &Real::zero(),
            &Real::one(),
            &x,
            &y,
        )
    })?)
}

/// Pairs each boundary loop of one profile with whether its side walls must
/// traverse it backwards, so that the profile's material lies to the left.
///
/// Hypertri emits caps with a normalized positive outer winding and negative
/// hole winding. Exact curve-region profiles preserve authored loop
/// orientation, so each loop is aligned with its cap role from the signed area
/// of its segment-start polygon, evaluated under the selected policy. This is
/// the polygonal side-wall contract, not integration along arbitrary curved
/// edges. Zero signed area is an error. Material precedes holes in the result.
fn oriented_profile_rings<'a>(
    material: &'a Contour2,
    holes: &'a [Contour2],
    decisions: &GeometryDecisions,
) -> Result<Vec<(&'a Contour2, bool)>, ValidationError> {
    std::iter::once((material, false))
        .chain(holes.iter().map(|ring| (ring, true)))
        .map(|(ring, is_hole)| {
            let points = ring
                .segments()
                .iter()
                .map(|segment| {
                    hyperlimit::Point2::new(
                        segment.start().x().clone(),
                        segment.start().y().clone(),
                    )
                })
                .collect::<Vec<_>>();
            let reverse = match decisions.decide(
                hyperlimit::ring_area_sign(&points, decisions.predicate_policy()),
                "profile side-ring signed area",
            )? {
                hyperlimit::Sign::Positive => is_hole,
                hyperlimit::Sign::Negative => !is_hole,
                hyperlimit::Sign::Zero => {
                    return Err(ValidationError::Geometry(
                        "profile side ring has zero signed area".into(),
                    ));
                },
            };
            Ok((ring, reverse))
        })
        .collect()
}

/// The start and end points of each loop edge, in the side-wall direction
/// chosen by [`oriented_profile_rings`].
///
/// Reversal swaps each pair's endpoints but preserves segment iteration order;
/// the output is for independent wall construction, not a reversed connected wire.
fn oriented_ring_edges(ring: &Contour2, reverse: bool) -> impl Iterator<Item = [&Point2; 2]> {
    ring.segments().iter().map(move |segment| {
        if reverse {
            [segment.end(), segment.start()]
        } else {
            [segment.start(), segment.end()]
        }
    })
}

/// Converts counts through checked `u64` values and divides without floating-point rounding.
///
/// Conversion overflow is a capacity error; a zero denominator propagates the
/// scalar division error.
fn exact_ratio(numerator: usize, denominator: usize) -> Result<Real, ValidationError> {
    let numerator = u64::try_from(numerator).map_err(|_| ValidationError::CapacityOverflow)?;
    let denominator =
        u64::try_from(denominator).map_err(|_| ValidationError::CapacityOverflow)?;
    Ok((Real::from(numerator) / Real::from(denominator))?)
}

/// Returns `(radius * cos(angle), radius * sin(angle))` as exact scalar expressions.
///
/// Angle is in radians; this helper does not constrain the sign of the radius.
fn exact_polar(radius: &Real, angle: Real) -> [Real; 2] {
    [
        radius.clone() * angle.clone().cos(),
        radius.clone() * angle.sin(),
    ]
}

/// Concatenates equally rotated copies of a point pattern in copy-then-point order.
///
/// Rotations are exact expressions at fractions of a full turn. Checked output
/// capacity precedes allocation; zero copies returns no points without division.
/// This helper neither welds repeated endpoints nor validates a filled ring.
fn repeat_exact_rotational_pattern(
    local_points: &[[Real; 2]],
    copies: usize,
) -> Result<Vec<[Real; 2]>, ValidationError> {
    let capacity = solid::capacity::<[Real; 2]>(copies.checked_mul(local_points.len()))?;
    let mut points = solid::reserved(capacity)?;
    for copy in 0..copies {
        let angle = Real::tau() * exact_ratio(copy, copies)?;
        let sine = angle.clone().sin();
        let cosine = angle.cos();
        points.extend(local_points.iter().map(|[x, y]| {
            [
                x.clone() * cosine.clone() - y.clone() * sine.clone(),
                x.clone() * sine.clone() + y.clone() * cosine.clone(),
            ]
        }));
    }
    Ok(points)
}

/// Returns the canonical empty filled region, with no boundary contours.
pub fn empty() -> CurveRegion2 {
    CurveRegion2::empty()
}

/// Axis-aligned rectangle with its minimum corner at the origin.
///
/// Dimensions must be nonnegative. A certified zero dimension produces the
/// canonical empty set after both arguments are validated. Negative or
/// undecided dimensions fail; they never become successful empty geometry.
pub fn rectangle(
    width: impl IntoReal,
    length: impl IntoReal,
) -> Result<CurveRegion2, ValidationError> {
    let width = width.into_real()?;
    let length = length.into_real()?;
    rectangle_with_context(width, length, GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Axis-aligned square with its minimum corner at the origin.
///
/// Uses the strict [`rectangle`] contract for both equal dimensions.
pub fn square(width: impl IntoReal) -> Result<CurveRegion2, ValidationError> {
    let width = width.into_real()?;
    rectangle(width.clone(), width)
}

/// Regular sampled circle.
///
/// The radius is nonnegative and at least three segments are required, even
/// for zero radius. Certification concerns the sampled polygon; no analytic
/// circle fidelity bound is implied.
/// The center is the origin and the first vertex is `(radius, 0)`, followed
/// counterclockwise. Dimensions and topology are decided under the strict policy.
pub fn circle(
    radius: impl IntoReal,
    segments: usize,
) -> Result<CurveRegion2, ValidationError> {
    let radius = radius.into_real()?;
    circle_with_context(radius, segments, GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Constructs a rectangle with the selected policy and retains predicate certainty.
///
/// The minimum corner is the origin. Width and height must be nonnegative;
/// both are validated before a selected zero returns empty geometry. An
/// approximate zero decision remains qualified in the outcome.
pub fn rectangle_with_context(
    width: Real,
    height: Real,
    context: GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let region = rectangle_with(width, height, &decisions)?;
    Ok(decisions.finish(region))
}

/// Constructs a square with the selected policy and retains predicate certainty.
///
/// Delegates equal dimensions to [`rectangle_with_context`], including collapse
/// and validation behavior.
pub fn square_with_context(
    width: Real,
    context: GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    rectangle_with_context(width.clone(), width, context)
}

/// Constructs a sampled circle with the selected policy and retains predicate certainty.
///
/// The polygon is centered at the origin with its first vertex at `(radius, 0)`.
/// At least three segments and representable sample capacity are required before
/// validating the nonnegative radius; a selected zero then returns empty.
/// Certification applies to the polygon and its decisions, not analytic-circle fidelity.
pub fn circle_with_context(
    radius: Real,
    segments: usize,
    context: GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let region = circle_with(radius, segments, &decisions)?;
    Ok(decisions.finish(region))
}

/// Constructs a sampled ellipse with the selected policy and retains predicate certainty.
///
/// Width and height are full axis lengths, centered at the origin. At least
/// three samples and representable capacity are required, then both dimensions
/// are checked as nonnegative before a selected zero returns empty. Positive
/// dimensions give counterclockwise angular samples starting at `(width / 2, 0)`.
/// The result has straight edges; certification supplies no analytic-ellipse
/// approximation bound.
pub fn ellipse_with_context(
    width: Real,
    height: Real,
    segments: usize,
    context: GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    solid::sample_minimum(segments, 3, "segments")?;
    solid::capacity::<[Real; 2]>(Some(segments))?;
    let width_sign = nonnegative_with(&width, "width", &decisions)?;
    let height_sign = nonnegative_with(&height, "height", &decisions)?;
    if width_sign == hyperreal::RealSign::Zero || height_sign == hyperreal::RealSign::Zero {
        return Ok(decisions.finish(empty()));
    }
    let two = Real::from(2_u8);
    let region = ellipse_points_with(
        (width / &two).expect("two is nonzero"),
        (height / two).expect("two is nonzero"),
        segments,
        &decisions,
    )?;
    Ok(decisions.finish(region))
}

/// Constructs a regular polygon with the selected policy and retains predicate certainty.
///
/// Radius is the circumradius. Delegates to [`circle_with_context`] with `sides`
/// samples, sharing its origin, orientation, validation and collapse contract.
pub fn regular_ngon_with_context(
    sides: usize,
    radius: Real,
    context: GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    circle_with_context(radius, sides, context)
}

/// Right triangle with its right-angle corner at the origin.
///
/// The other vertices are `(width, 0)` and `(0, height)`. Both dimensions must
/// be nonnegative under the strict policy; a zero returns empty only after both
/// arguments are validated.
pub fn right_triangle(
    width: impl IntoReal,
    height: impl IntoReal,
) -> Result<CurveRegion2, ValidationError> {
    let width = width.into_real()?;
    let height = height.into_real()?;
    right_triangle_with_context(width, height, GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Constructs a right triangle, retaining the certainty of dimension and topology decisions.
///
/// Vertices are `(0, 0)`, `(width, 0)` and `(0, height)`. Both dimensions are
/// checked as nonnegative before a selected zero produces empty geometry.
pub fn right_triangle_with_context(
    width: Real,
    height: Real,
    context: GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let width_sign = nonnegative_with(&width, "width", &decisions)?;
    let height_sign = nonnegative_with(&height, "height", &decisions)?;
    if width_sign == hyperreal::RealSign::Zero || height_sign == hyperreal::RealSign::Zero {
        return Ok(decisions.finish(empty()));
    }
    let region = ring_with(
        &[
            [Real::zero(), Real::zero()],
            [width, Real::zero()],
            [Real::zero(), height],
        ],
        &decisions,
    )?;
    Ok(decisions.finish(region))
}

/// Closed polygon from exact coordinate rows.
///
/// Requires a valid ring with at least three vertices. Use [`empty`] for
/// canonical absence; an empty or malformed ring is an error.
/// A repeated closing point is accepted and removed before contour construction;
/// at least three vertices must remain. Edges are straight, coordinates stay
/// exact, and admission uses the strict policy with native nonzero fill semantics.
pub fn polygon(points: &[[Real; 2]]) -> Result<CurveRegion2, ValidationError> {
    polygon_with_context(points, GeometryContext::STRICT).map(GeometryOutcome::into_value)
}

/// Publishes a filled ring with the selected policy and aggregate certainty.
///
/// Native contour and region admission validate the coordinate rows; invalid
/// or empty rings remain errors. A successful strict cache entry may be reused.
pub fn polygon_with_context(
    points: &[[Real; 2]],
    context: GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let region = ring_with(points, &decisions)?;
    Ok(decisions.finish(region))
}

/// Closed polygon from native Hypercurve points.
///
/// Clones exact coordinates into checked temporary storage and delegates to
/// strict [`polygon`] admission; this does not sample or round the points.
pub fn polygon_points(points: &[Point2]) -> Result<CurveRegion2, ValidationError> {
    let mut rows = solid::reserved(points.len())?;
    rows.extend(
        points
            .iter()
            .map(|point| [point.x().clone(), point.y().clone()]),
    );
    polygon(&rows)
}

/// Origin-centered ellipse represented by a polygon with equally spaced angular samples.
///
/// Width and height are full nonnegative axis lengths. Uses strict
/// [`ellipse_with_context`] validation, including at least three samples even
/// for a collapsed ellipse. The first vertex is `(width / 2, 0)`; no bound on
/// approximation to analytic ellipse arcs is supplied.
pub fn ellipse(
    width: impl IntoReal,
    height: impl IntoReal,
    segments: usize,
) -> Result<CurveRegion2, ValidationError> {
    let width = width.into_real()?;
    let height = height.into_real()?;
    ellipse_with_context(width, height, segments, GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Origin-centered regular polygon specified by side count and circumradius.
///
/// Uses strict [`regular_ngon_with_context`] validation: at least three sides,
/// nonnegative radius and an empty result for zero radius. Positive-radius
/// vertices run counterclockwise from `(radius, 0)`.
pub fn regular_ngon(
    sides: usize,
    radius: impl IntoReal,
) -> Result<CurveRegion2, ValidationError> {
    let radius = radius.into_real()?;
    regular_ngon_with_context(sides, radius, GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Filled polygonal arrow pointing along positive X, symmetric about the X axis.
///
/// The shaft runs from `x = 0` to `shaft_length`; the tip is at
/// `(shaft_length + head_length, 0)`. Widths are full widths. All four dimensions
/// must be positive under the strict policy. Equal shaft and head widths share
/// their shoulder endpoints; only those proven duplicates are omitted. The head
/// may be narrower than the shaft.
pub fn arrow(
    shaft_length: impl IntoReal,
    shaft_width: impl IntoReal,
    head_length: impl IntoReal,
    head_width: impl IntoReal,
) -> Result<CurveRegion2, ValidationError> {
    let shaft_length = shaft_length.into_real()?;
    let shaft_width = shaft_width.into_real()?;
    let head_length = head_length.into_real()?;
    let head_width = head_width.into_real()?;
    arrow_with_context(
        shaft_length,
        shaft_width,
        head_length,
        head_width,
        GeometryContext::STRICT,
    )
    .map(GeometryOutcome::into_value)
}

/// Constructs the polygonal arrow of [`arrow`] with explicit policy and certainty.
///
/// Checks shaft length, shaft width, head length and head width in that order,
/// then compares the widths. A selected equality omits the two head-shoulder
/// vertices; any approximate equality remains qualified even on a strict ring
/// cache hit. Nonpositive dimensions and unresolved decisions remain errors.
pub fn arrow_with_context(
    shaft_length: Real,
    shaft_width: Real,
    head_length: Real,
    head_width: Real,
    context: GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    for (value, name) in [
        (&shaft_length, "shaft_length"),
        (&shaft_width, "shaft_width"),
        (&head_length, "head_length"),
        (&head_width, "head_width"),
    ] {
        positive_with(value, name, &decisions)?;
    }
    let shoulders = compare_with(&shaft_width, &head_width, &decisions)? != Ordering::Equal;
    let two = Real::from(2_u8);
    let half_shaft = (shaft_width / &two)?;
    let half_head = (head_width / &two)?;
    let tip = shaft_length.clone() + head_length;
    let mut points = solid::reserved(7)?;
    points.push([Real::zero(), -half_shaft.clone()]);
    points.push([shaft_length.clone(), -half_shaft.clone()]);
    if shoulders {
        points.push([shaft_length.clone(), -half_head.clone()]);
    }
    points.push([tip, Real::zero()]);
    if shoulders {
        points.push([shaft_length.clone(), half_head]);
    }
    points.push([shaft_length, half_shaft.clone()]);
    points.push([Real::zero(), half_shaft]);
    let region = ring_with(&points, &decisions)?;
    Ok(decisions.finish(region))
}

/// Filled trapezoid with bottom edge from `(0, 0)` to `(bottom_width, 0)`.
///
/// The top edge runs from `(top_offset, height)` to
/// `(top_offset + top_width, height)`. Widths and height must be strictly positive;
/// the horizontal top offset is unrestricted. Uses strict polygon admission.
pub fn trapezoid(
    top_width: impl IntoReal,
    bottom_width: impl IntoReal,
    height: impl IntoReal,
    top_offset: impl IntoReal,
) -> Result<CurveRegion2, ValidationError> {
    let top_width = top_width.into_real()?;
    let bottom_width = bottom_width.into_real()?;
    let height = height.into_real()?;
    let top_offset = top_offset.into_real()?;
    trapezoid_with_context(
        top_width,
        bottom_width,
        height,
        top_offset,
        GeometryContext::STRICT,
    )
    .map(GeometryOutcome::into_value)
}

/// Constructs the four-vertex trapezoid of [`trapezoid`] with explicit policy and certainty.
///
/// Checks top width, bottom width and height as positive before ring admission.
/// The top offset is a displacement, not a dimension subject to a sign constraint.
pub fn trapezoid_with_context(
    top_width: Real,
    bottom_width: Real,
    height: Real,
    top_offset: Real,
    context: GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    positive_with(&top_width, "top_width", &decisions)?;
    positive_with(&bottom_width, "bottom_width", &decisions)?;
    positive_with(&height, "height", &decisions)?;
    let region = ring_with(
        &[
            [Real::zero(), Real::zero()],
            [bottom_width, Real::zero()],
            [top_offset.clone() + top_width, height.clone()],
            [top_offset, height],
        ],
        &decisions,
    )?;
    Ok(decisions.finish(region))
}

/// Origin-centered polygon alternating outer and inner radii at equal angular steps.
///
/// Requires at least three points and `outer_radius > inner_radius > 0` under
/// the strict policy. `num_points` counts outer tips; the ring has twice that
/// many vertices, starts at `(outer_radius, 0)` and proceeds counterclockwise.
/// Straight edges connect alternating samples; this is not a crossing star path.
pub fn star(
    num_points: usize,
    outer_radius: impl IntoReal,
    inner_radius: impl IntoReal,
) -> Result<CurveRegion2, ValidationError> {
    let outer_radius = outer_radius.into_real()?;
    let inner_radius = inner_radius.into_real()?;
    star_with_context(
        num_points,
        outer_radius,
        inner_radius,
        GeometryContext::STRICT,
    )
    .map(GeometryOutcome::into_value)
}

/// Constructs the alternating-radius polygon of [`star`] with explicit policy and certainty.
///
/// Checks the minimum tip count and doubled vertex capacity before requiring a
/// positive inner radius and a strictly larger outer radius. A failed ordering
/// returns `InvalidArguments`; unresolved decisions propagate their typed errors.
pub fn star_with_context(
    num_points: usize,
    outer_radius: Real,
    inner_radius: Real,
    context: GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    solid::sample_minimum(num_points, 3, "num_points")?;
    let count = solid::capacity::<[Real; 2]>(num_points.checked_mul(2))?;
    positive_with(&inner_radius, "inner_radius", &decisions)?;
    if compare_with(&outer_radius, &inner_radius, &decisions)? != Ordering::Greater {
        return Err(ValidationError::InvalidArguments);
    }
    let denominator = Real::from(count as u64);
    let mut points = solid::reserved(count)?;
    for index in 0..count {
        let angle = Real::tau() * (Real::from(index as u64) / &denominator)?;
        let radius = if index % 2 == 0 {
            &outer_radius
        } else {
            &inner_radius
        };
        points.push([
            radius.clone() * angle.clone().cos(),
            radius.clone() * angle.sin(),
        ]);
    }
    let region = ring_with(&points, &decisions)?;
    Ok(decisions.finish(region))
}

/// Polygon joining the origin tip to samples of an upper semicircle.
///
/// Requires positive dimensions, `length > width / 2`, and at least two
/// semicircle chords under the strict policy. The semicircle has radius
/// `width / 2` and center `(0, length - width / 2)`. Its endpoints and
/// `segments - 1` interior samples are joined by chords, then connected to the
/// tip by straight sides. The top reaches `length` only when the top angular
/// sample is included (an even segment count); no analytic fidelity bound is given.
pub fn teardrop(
    width: impl IntoReal,
    length: impl IntoReal,
    segments: usize,
) -> Result<CurveRegion2, ValidationError> {
    let width = width.into_real()?;
    let length = length.into_real()?;
    teardrop_with_context(width, length, segments, GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Constructs the sampled teardrop of [`teardrop`] with explicit policy and certainty.
///
/// Checks at least two semicircle intervals and capacity for `segments + 2`
/// vertices before positive width/length and `length > width / 2`. The tip is
/// at the origin; the sampled semicircle lies above it. Certainty concerns the
/// polygon and consumed decisions, not its error relative to a curved teardrop.
pub fn teardrop_with_context(
    width: Real,
    length: Real,
    segments: usize,
    context: GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    solid::sample_minimum(segments, 2, "segments")?;
    let capacity = solid::capacity::<[Real; 2]>(segments.checked_add(2))?;
    positive_with(&width, "width", &decisions)?;
    positive_with(&length, "length", &decisions)?;
    let radius = (width / Real::from(2_u8))?;
    if compare_with(&length, &radius, &decisions)? != Ordering::Greater {
        return Err(ValidationError::InvalidArguments);
    }
    let center_y = length - radius.clone();
    let mut points = solid::reserved(capacity)?;
    points.push([Real::zero(), Real::zero()]);
    for index in 0..=segments {
        let fraction = (Real::from(index as u64) / Real::from(segments as u64))?;
        let angle = Real::pi() * fraction;
        points.push([
            -radius.clone() * angle.clone().cos(),
            center_y.clone() + radius.clone() * angle.sin(),
        ]);
    }
    let region = ring_with(&points, &decisions)?;
    Ok(decisions.finish(region))
}

/// Sampled egg polygon with its bounding-box center at the origin.
///
/// Requires positive dimensions and at least three samples under the strict
/// policy. Samples of `(-sin(t), cos(t) + cos(t)^2 / 5)` over one turn are scaled
/// and centered so their extrema are exactly `+/- width / 2` and `+/- length / 2`.
/// The represented boundary has straight chords; no analytic-curve error bound
/// is established.
pub fn egg(
    width: impl IntoReal,
    length: impl IntoReal,
    segments: usize,
) -> Result<CurveRegion2, ValidationError> {
    let width = width.into_real()?;
    let length = length.into_real()?;
    egg_with_context(width, length, segments, GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Constructs the normalized sampled egg of [`egg`] with explicit policy and certainty.
///
/// Checks sample count/capacity before positive width and length. Known angular
/// extremum indices supply the sample bounds; tied extrema receive the same exact
/// half-dimension coordinate. The first sample is the top at `(0, length / 2)`;
/// normalization depends on the sampling count rather than continuous curve extrema.
pub fn egg_with_context(
    width: Real,
    length: Real,
    segments: usize,
    context: GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    solid::sample_minimum(segments, 3, "segments")?;
    solid::capacity::<[Real; 2]>(Some(segments))?;
    positive_with(&width, "width", &decisions)?;
    positive_with(&length, "length", &decisions)?;
    let denominator = Real::from(segments as u64);
    let mut points = solid::reserved(segments)?;
    for index in 0..segments {
        let angle = Real::tau() * (Real::from(index as u64) / &denominator)?;
        let sine = angle.clone().sin();
        let cosine = angle.cos();
        let correction = (cosine.clone() * cosine.clone() / Real::from(5_u8))?;
        points.push([-sine, cosine + correction]);
    }
    let quarter_turns = segments / 4;
    let remainder = segments % 4;
    let min_x_index = quarter_turns + usize::from(remainder >= 2);
    let max_x_index = 3 * quarter_turns + ((3 * remainder + 2) / 4);
    let min_y_index = segments / 2;
    let min_x = points[min_x_index][0].clone();
    let max_x = points[max_x_index][0].clone();
    let min_y = points[min_y_index][1].clone();
    let max_y = points[0][1].clone();
    let x_scale = (width.clone() / (max_x.clone() - min_x.clone()))?;
    let y_scale = (length.clone() / (max_y.clone() - min_y.clone()))?;
    let x_midpoint = ((min_x + max_x) / Real::from(2_u8))?;
    let y_midpoint = ((min_y + max_y) / Real::from(2_u8))?;
    for [x, y] in &mut points {
        *x = (x.clone() - x_midpoint.clone()) * x_scale.clone();
        *y = (y.clone() - y_midpoint.clone()) * y_scale.clone();
    }
    let half_width = (width / Real::from(2_u8))?;
    let half_length = (length / Real::from(2_u8))?;
    points[min_x_index][0] = -half_width.clone();
    points[max_x_index][0] = half_width;
    if remainder == 2 {
        points[min_x_index - 1][0] = points[min_x_index][0].clone();
        points[max_x_index - 1][0] = points[max_x_index][0].clone();
    }
    points[min_y_index][1] = -half_length.clone();
    if segments % 2 == 1 {
        points[min_y_index + 1][1] = -half_length.clone();
    }
    points[0][1] = half_length;
    let region = ring_with(&points, &decisions)?;
    Ok(decisions.finish(region))
}

/// Rounded rectangle represented by exact angular samples and straight chords.
///
/// The bounding rectangle is `[0, width]` by `[0, height]`. The radius is clamped
/// to half the smaller dimension. For positive radius and `corner_segments = n > 0`,
/// each quarter-circle has `n` uniform angular intervals, and every nonzero
/// straight side joins its tangent endpoints. Only coincident endpoints are
/// omitted. Zero radius or zero corner segments selects the sharp rectangle.
/// Dimensions and radius must be nonnegative under the strict policy; a zero
/// dimension returns empty only after all three are checked. Certification does
/// not supply a chord-error bound for the rounded outline.
pub fn rounded_rectangle(
    width: impl IntoReal,
    height: impl IntoReal,
    corner_radius: impl IntoReal,
    corner_segments: usize,
) -> Result<CurveRegion2, ValidationError> {
    let width = width.into_real()?;
    let height = height.into_real()?;
    let corner_radius = corner_radius.into_real()?;
    rounded_rectangle_with_context(
        width,
        height,
        corner_radius,
        corner_segments,
        GeometryContext::STRICT,
    )
    .map(GeometryOutcome::into_value)
}

/// Constructs the sampled rounded rectangle with explicit policy and aggregate certainty.
///
/// Uses the placement and sampling of [`rounded_rectangle`]. Capacity for
/// `4 * (corner_segments + 1)` vertices is checked first, even for a sharp or
/// empty result. Width, height and radius are then checked as nonnegative before
/// collapse. Clamp comparisons select the radius and shared tangent endpoints;
/// any approximate comparison remains qualified. Zero intervals selects the
/// sharp rectangle after validation and clamping.
pub fn rounded_rectangle_with_context(
    width: Real,
    height: Real,
    corner_radius: Real,
    corner_segments: usize,
    context: GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let point_capacity = solid::capacity::<[Real; 2]>(
        corner_segments
            .checked_add(1)
            .and_then(|count| count.checked_mul(4)),
    )?;
    let width_sign = nonnegative_with(&width, "width", &decisions)?;
    let height_sign = nonnegative_with(&height, "height", &decisions)?;
    nonnegative_with(&corner_radius, "corner_radius", &decisions)?;
    if width_sign == hyperreal::RealSign::Zero || height_sign == hyperreal::RealSign::Zero {
        return Ok(decisions.finish(empty()));
    }
    let half_width = (&width / Real::from(2_u8)).expect("two is nonzero");
    let half_height = (&height / Real::from(2_u8)).expect("two is nonzero");
    let (radius, shared_x) = match compare_with(&corner_radius, &half_width, &decisions)? {
        Ordering::Greater => (half_width, true),
        Ordering::Equal => (corner_radius, true),
        Ordering::Less => (corner_radius, false),
    };
    let (radius, shared_x, shared_y) = match compare_with(&radius, &half_height, &decisions)? {
        // half_height < radius <= half_width also proves a nonzero horizontal side.
        Ordering::Greater => (half_height, false, true),
        Ordering::Equal => (radius, shared_x, true),
        Ordering::Less => (radius, shared_x, false),
    };
    if corner_segments == 0
        || decisions.sign(&radius, "corner radius")? == hyperreal::RealSign::Zero
    {
        return Ok(decisions.finish(rectangle_with(width, height, &decisions)?));
    }
    let half_pi = (Real::pi() / Real::from(2_u8)).expect("two is nonzero");
    let centers = [
        (radius.clone(), radius.clone(), Real::pi()),
        (
            width.clone() - radius.clone(),
            radius.clone(),
            Real::pi() + half_pi.clone(),
        ),
        (
            width.clone() - radius.clone(),
            height.clone() - radius.clone(),
            Real::zero(),
        ),
        (radius.clone(), height - radius.clone(), half_pi.clone()),
    ];
    let mut points = solid::reserved(point_capacity)?;
    let corner_count = centers.len();
    for (corner_index, (cx, cy, start)) in centers.into_iter().enumerate() {
        let shared_start = match corner_index {
            1 | 3 => shared_x,
            2 => shared_y,
            _ => false,
        };
        for index in 0..=corner_segments {
            // The clamp comparisons prove which tangent endpoints coincide.
            // Other adjacent corners are joined by a nonzero straight side,
            // whose two endpoints must both remain in the sampled boundary.
            if (shared_start && index == 0)
                || (shared_y && corner_index + 1 == corner_count && index == corner_segments)
            {
                continue;
            }
            let fraction = (Real::from(index as u64) / Real::from(corner_segments as u64))?;
            let angle = start.clone() + half_pi.clone() * fraction;
            points.push([
                cx.clone() + radius.clone() * angle.clone().cos(),
                cy.clone() + radius.clone() * angle.sin(),
            ]);
        }
    }
    let region = ring_with(&points, &decisions)?;
    Ok(decisions.finish(region))
}

/// Origin-centered polygon sampled from a fourth-power superellipse.
///
/// Requires positive dimensions and at least three samples under the strict
/// policy. With `rx = width / 2`, `ry = height / 2`, samples are
/// `(rx * sign(cos(t)) * sqrt(abs(cos(t))),
/// ry * sign(sin(t)) * sqrt(abs(sin(t))))` at uniform angles from zero.
/// They lie on `(x / rx)^4 + (y / ry)^4 = 1`; straight chords define the result.
/// Width and height describe the analytic axes, so sampled bounds may be smaller
/// when extrema are absent. No analytic fidelity bound is supplied.
pub fn squircle(
    width: impl IntoReal,
    height: impl IntoReal,
    segments: usize,
) -> Result<CurveRegion2, ValidationError> {
    let width = width.into_real()?;
    let height = height.into_real()?;
    squircle_with_context(width, height, segments, GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Constructs the sampled squircle of [`squircle`] with explicit policy and certainty.
///
/// Checks sample count/capacity and positive dimensions, then decides each
/// sine/cosine sign before attaching it to the square root of the absolute value.
/// Those sign decisions and native ring admission contribute to the outcome;
/// scalar failures remain errors.
pub fn squircle_with_context(
    width: Real,
    height: Real,
    segments: usize,
    context: GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    solid::sample_minimum(segments, 3, "segments")?;
    solid::capacity::<[Real; 2]>(Some(segments))?;
    positive_with(&width, "width", &decisions)?;
    positive_with(&height, "height", &decisions)?;
    let rx = (width / Real::from(2_u8))?;
    let ry = (height / Real::from(2_u8))?;
    let signed_root = |value: Real| -> Result<Real, ValidationError> {
        let negative =
            decisions.sign(&value, "squircle sample sign")? == hyperreal::RealSign::Negative;
        let root = value.abs().sqrt()?;
        Ok(if negative { -root } else { root })
    };
    let denominator = Real::from(segments as u64);
    let mut points = solid::reserved(segments)?;
    for index in 0..segments {
        let angle = Real::tau() * (Real::from(index as u64) / &denominator)?;
        points.push([
            rx.clone() * signed_root(angle.clone().cos())?,
            ry.clone() * signed_root(angle.sin())?,
        ]);
    }
    let region = ring_with(&points, &decisions)?;
    Ok(decisions.finish(region))
}

/// Unions an origin-centered sampled circle with a rectangular handle extending along positive Y.
///
/// The handle occupies `[-handle_width / 2, handle_width / 2]` by
/// `[0, handle_height]`. All three dimensions must be positive and at least three
/// circle segments are required. The handle need not be narrower than the circle.
/// Native regularized union determines the boundary; the outcome retains predicate
/// certainty. The circular part is polygonal, with no analytic-circle error bound.
pub fn keyhole(
    circle_radius: Real,
    handle_width: Real,
    handle_height: Real,
    segments: usize,
    context: GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    solid::sample_minimum(segments, 3, "segments")?;
    solid::capacity::<[Real; 2]>(Some(segments))?;
    positive_with(&circle_radius, "circle_radius", &decisions)?;
    positive_with(&handle_width, "handle_width", &decisions)?;
    positive_with(&handle_height, "handle_height", &decisions)?;
    let handle_x = -(handle_width.clone() / Real::from(2_u8)).expect("two is nonzero");
    let handle = translate_with(
        &rectangle_with(handle_width, handle_height, &decisions)?,
        handle_x,
        Real::zero(),
        &decisions,
    )?;
    let operand = circle_with(circle_radius, segments, &decisions)?;
    let region =
        decisions.exact_curve(|| operand.boolean_region(&handle, BooleanOp::Union))?;
    Ok(decisions.finish(region))
}

/// Intersects sampled disks arranged at the vertices of an odd regular polygon.
///
/// Requires odd `sides >= 3`, positive `diameter` and `circle_segments >= 6`.
/// The centers have circumradius `diameter / (2 * cos(pi / (2 * sides)))`,
/// starting on positive X. Each operand is a regular `circle_segments`-gon of
/// radius `diameter`, with the same angular orientation, translated to a center.
///
/// `diameter` is the intended Reuleaux width and the operand disk radius, not its
/// diameter. Native intersections act on sampled polygons; the result is not an
/// analytic constant-width curve and has no approximation error bound. Sample
/// capacity and doubled-side arithmetic are checked, but these checks do not
/// bound total Boolean work. The outcome retains aggregate predicate certainty.
pub fn reuleaux(
    sides: usize,
    diameter: Real,
    circle_segments: usize,
    context: GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    solid::sample_minimum(sides, 3, "sides")?;
    solid::sample_minimum(circle_segments, 6, "circle_segments")?;
    solid::capacity::<[Real; 2]>(Some(circle_segments))?;
    if sides.is_multiple_of(2) {
        return Err(ValidationError::InvalidArguments);
    }
    let double_sides = sides
        .checked_mul(2)
        .ok_or(ValidationError::CapacityOverflow)?;
    positive_with(&diameter, "diameter", &decisions)?;
    let half_angle = (Real::pi() / Real::from(double_sides as u64))?;
    let circumradius = (diameter.clone() / (Real::from(2_u8) * half_angle.cos()))?;
    let denominator = Real::from(sides as u64);
    let mut result: Option<CurveRegion2> = None;
    for index in 0..sides {
        let angle = Real::tau()
            * (Real::from(index as u64) / &denominator)
                .expect("the validated side count is nonzero");
        let disk = translate_with(
            &circle_with(diameter.clone(), circle_segments, &decisions)?,
            circumradius.clone() * angle.clone().cos(),
            circumradius.clone() * angle.sin(),
            &decisions,
        )?;
        result = Some(match result {
            None => disk,
            Some(current) => decisions
                .exact_curve(|| current.boolean_region(&disk, BooleanOp::Intersection))?,
        });
    }
    Ok(decisions.finish(result.unwrap_or_else(empty)))
}

/// Subtracts concentric sampled circles to form a polygonal annulus.
///
/// Requires positive `inner_diameter` and radial `thickness`, plus at least three
/// segments. The inner circumradius is `inner_diameter / 2`; the outer is that
/// radius plus `thickness`. Both polygons start on positive X with identical
/// angular samples. Corresponding edge support lines are separated by
/// `thickness * cos(pi / segments)`, rather than by `thickness` itself.
/// Native difference supplies the hole and
/// aggregate certainty; no analytic-annulus approximation bound is supplied.
pub fn ring(
    inner_diameter: Real,
    thickness: Real,
    segments: usize,
    context: GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    solid::sample_minimum(segments, 3, "segments")?;
    solid::capacity::<[Real; 2]>(Some(segments))?;
    positive_with(&inner_diameter, "inner_diameter", &decisions)?;
    positive_with(&thickness, "thickness", &decisions)?;
    let inner_radius = (inner_diameter / Real::from(2_u8))?;
    let inner = circle_with(inner_radius.clone(), segments, &decisions)?;
    let operand = circle_with(inner_radius + thickness, segments, &decisions)?;
    let region =
        decisions.exact_curve(|| operand.boolean_region(&inner, BooleanOp::Difference))?;
    Ok(decisions.finish(region))
}

/// Origin-centered circular sector represented by two radial edges and sampled chords.
///
/// Requires positive radius, at least one chord, and a nonzero signed sweep
/// `end_angle_deg - start_angle_deg` of at most 360 degrees in magnitude under
/// the strict policy. Partial sectors include both angular endpoints; negative
/// sweeps run clockwise. Native failures remain typed, while a successfully
/// regularized zero-area sample represents the empty set.
///
/// A full turn uses `max(segments, 3)` regular-polygon vertices starting on positive
/// X, regardless of starting angle or sweep direction. No analytic-sector fidelity
/// bound is implied.
pub fn pie_slice(
    radius: impl IntoReal,
    start_angle_deg: impl IntoReal,
    end_angle_deg: impl IntoReal,
    segments: usize,
) -> Result<CurveRegion2, ValidationError> {
    let radius = radius.into_real()?;
    let start_angle_deg = start_angle_deg.into_real()?;
    let end_angle_deg = end_angle_deg.into_real()?;
    pie_slice_with_context(
        radius,
        start_angle_deg,
        end_angle_deg,
        segments,
        GeometryContext::STRICT,
    )
    .map(GeometryOutcome::into_value)
}

/// Constructs the sampled sector of [`pie_slice`] with explicit policy and certainty.
///
/// Checks at least one interval and capacity for `segments + 2` vertices before
/// radius and sweep validation. A policy-selected full turn takes the regular
/// polygon path; all other admitted sweeps include the origin and `segments + 1`
/// angular samples. Approximate decisions remain qualified, including selection
/// of the full-turn path.
pub fn pie_slice_with_context(
    radius: Real,
    start_angle_deg: Real,
    end_angle_deg: Real,
    segments: usize,
    context: GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    solid::sample_minimum(segments, 1, "segments")?;
    let capacity = solid::capacity::<[Real; 2]>(segments.checked_add(2))?;
    positive_with(&radius, "radius", &decisions)?;
    let sweep = end_angle_deg - start_angle_deg.clone();
    let absolute_sweep = sweep.clone().abs();
    positive_with(&absolute_sweep, "sweep", &decisions)?;
    match compare_with(&absolute_sweep, &Real::from(360_u16), &decisions)? {
        Ordering::Greater => return Err(ValidationError::InvalidArguments),
        Ordering::Equal => {
            let region =
                ellipse_points_with(radius.clone(), radius, segments.max(3), &decisions)?;
            return Ok(decisions.finish(region));
        },
        Ordering::Less => {},
    }
    let start = (start_angle_deg * Real::pi() / Real::from(180_u16))?;
    let sweep = (sweep * Real::pi() / Real::from(180_u16))?;
    let mut points = solid::reserved(capacity)?;
    points.push([Real::zero(), Real::zero()]);
    for index in 0..=segments {
        let fraction = (Real::from(index as u64) / Real::from(segments as u64))?;
        let angle = start.clone() + sweep.clone() * fraction;
        points.push([
            radius.clone() * angle.clone().cos(),
            radius.clone() * angle.sin(),
        ]);
    }
    let region = ring_with(&points, &decisions)?;
    Ok(decisions.finish(region))
}

/// Sampled heart polygon normalized into `[0, width]` by `[0, height]`.
///
/// Requires positive dimensions and at least eight samples under the strict
/// policy. Uniform angular samples use `x = 16 * sin(t)^3` and
/// `y = 13*cos(t) - 5*cos(2*t) - 2*cos(3*t) - cos(4*t)`, then rescale their
/// sample extrema to the requested dimensions. These are polygon bounds, not
/// normalization by continuous extrema; no analytic-curve fidelity bound is given.
pub fn heart(
    width: impl IntoReal,
    height: impl IntoReal,
    segments: usize,
) -> Result<CurveRegion2, ValidationError> {
    let width = width.into_real()?;
    let height = height.into_real()?;
    heart_with_context(width, height, segments, GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Constructs the normalized sampled heart of [`heart`] with explicit policy and certainty.
///
/// Checks sample capacity and index arithmetic before positive dimensions. The
/// horizontal extrema use angular symmetry; comparisons between adjacent samples
/// locate the upper vertical extremum. Those comparisons contribute to the
/// certainty receipt. Samples are joined by straight edges after rescaling.
pub fn heart_with_context(
    width: Real,
    height: Real,
    segments: usize,
    context: GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    solid::sample_minimum(segments, 8, "segments")?;
    solid::capacity::<[Real; 2]>(Some(segments))?;
    let quarter_rounding_numerator = segments
        .checked_add(2)
        .ok_or(ValidationError::CapacityOverflow)?;
    positive_with(&width, "width", &decisions)?;
    positive_with(&height, "height", &decisions)?;
    let denominator = Real::from(segments as u64);
    let mut points = solid::reserved(segments)?;
    for index in 0..segments {
        let angle = Real::tau() * (Real::from(index as u64) / &denominator)?;
        let sine = angle.clone().sin();
        let cosine = angle.cos();
        let cosine_squared = cosine.clone() * cosine.clone();
        points.push([
            Real::from(16_u8) * sine.clone() * sine.clone() * sine,
            Real::from(4_u8) + Real::from(19_u8) * cosine.clone()
                - Real::from(2_u8) * cosine_squared.clone()
                - Real::from(8_u8) * cosine_squared.clone() * cosine
                - Real::from(8_u8) * cosine_squared.clone() * cosine_squared,
        ]);
    }
    let max_x_index = quarter_rounding_numerator / 4;
    let min_x_index = (segments - max_x_index) % segments;
    let min_x = points[min_x_index][0].clone();
    let max_x = points[max_x_index][0].clone();
    let mut lower = 0;
    let mut upper = segments / 2;
    while lower < upper {
        let middle = lower + (upper - lower) / 2;
        match compare_with(&points[middle][1], &points[middle + 1][1], &decisions)? {
            Ordering::Less => lower = middle + 1,
            Ordering::Equal | Ordering::Greater => upper = middle,
        }
    }
    let min_y = points[segments / 2][1].clone();
    let max_y = points[lower][1].clone();
    let span_x = max_x - min_x.clone();
    let span_y = max_y - min_y.clone();
    for [x, y] in &mut points {
        *x = width.clone() * ((x.clone() - min_x.clone()) / span_x.clone())?;
        *y = height.clone() * ((y.clone() - min_y.clone()) / span_y.clone())?;
    }
    let region = ring_with(&points, &decisions)?;
    Ok(decisions.finish(region))
}

/// Subtracts a horizontally shifted sampled circle from an origin-centered larger one.
///
/// Requires `outer_radius > inner_radius > 0` and at least six segments. The
/// inner center is `(offset, 0)`; offset has no sign or overlap restriction.
/// Thus zero offset yields a polygonal annulus, and a sufficiently distant cutter
/// leaves the outer polygon. Both circles use the same angular sampling orientation.
/// Native difference supplies region semantics and aggregate certainty, with no
/// analytic-crescent fidelity bound.
pub fn crescent(
    outer_radius: Real,
    inner_radius: Real,
    offset: Real,
    segments: usize,
    context: GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    solid::sample_minimum(segments, 6, "segments")?;
    solid::capacity::<[Real; 2]>(Some(segments))?;
    positive_with(&inner_radius, "inner_radius", &decisions)?;
    if compare_with(&outer_radius, &inner_radius, &decisions)? != Ordering::Greater {
        return Err(ValidationError::InvalidArguments);
    }
    let outer = circle_with(outer_radius, segments, &decisions)?;
    let inner = translate_with(
        &circle_with(inner_radius, segments, &decisions)?,
        offset,
        Real::zero(),
        &decisions,
    )?;
    let region =
        decisions.exact_curve(|| outer.boolean_region(&inner, BooleanOp::Difference))?;
    Ok(decisions.finish(region))
}

/// Polygon sampled from the polar superformula over one turn.
///
/// Requires positive `a` and `b`, nonzero `n1`, at least three samples, and
/// defined scalar powers at every sample. At uniform `theta` from zero, the radius
/// is `(|cos(m*theta/4)/a|^n2 + |sin(m*theta/4)/b|^n3)^(-1/n1)`.
/// No separate sign or integrality constraint is imposed on `m`, `n2` or `n3`.
///
/// Uses strict native polygon admission. The final sample connects directly to
/// the first; a periodic or smooth analytic seam is not required or certified.
/// Native scalar and topology failures remain typed, and no analytic fidelity
/// bound is implied.
#[allow(clippy::too_many_arguments)]
pub fn supershape(
    a: impl IntoReal,
    b: impl IntoReal,
    m: impl IntoReal,
    n1: impl IntoReal,
    n2: impl IntoReal,
    n3: impl IntoReal,
    segments: usize,
) -> Result<CurveRegion2, ValidationError> {
    let a = a.into_real()?;
    let b = b.into_real()?;
    let m = m.into_real()?;
    let n1 = n1.into_real()?;
    let n2 = n2.into_real()?;
    let n3 = n3.into_real()?;
    supershape_with_context(a, b, m, n1, n2, n3, segments, GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Constructs the polar polygon of [`supershape`] with explicit policy and certainty.
///
/// Checks sample count/capacity and positive axes before computing `-1 / n1`.
/// A zero `n1` therefore propagates a scalar division error. Scalar powers must
/// be supported and defined at every sample; their failures are not empty results.
/// The outcome records policy decisions consumed by validation and ring admission.
#[allow(clippy::too_many_arguments)]
pub fn supershape_with_context(
    a: Real,
    b: Real,
    m: Real,
    n1: Real,
    n2: Real,
    n3: Real,
    segments: usize,
    context: GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    solid::sample_minimum(segments, 3, "segments")?;
    solid::capacity::<[Real; 2]>(Some(segments))?;
    positive_with(&a, "a", &decisions)?;
    positive_with(&b, "b", &decisions)?;
    let exponent = (Real::from(-1_i8) / n1)?;
    let denominator = Real::from(segments as u64);
    let mut points = solid::reserved(segments)?;
    for index in 0..segments {
        let theta = Real::tau() * (Real::from(index as u64) / &denominator)?;
        let angle = (m.clone() * theta.clone() / Real::from(4_u8))?;
        let first = (angle.clone().cos().abs() / a.clone())?.pow(n2.clone())?;
        let second = (angle.sin().abs() / b.clone())?.pow(n3.clone())?;
        let radius = (first + second).pow(exponent.clone())?;
        points.push([radius.clone() * theta.clone().cos(), radius * theta.sin()]);
    }
    let region = ring_with(&points, &decisions)?;
    Ok(decisions.finish(region))
}

/// Cuts a positive-X rectangular notch from an origin-centered sampled circle.
///
/// Radius, key width and key depth must be positive; both key dimensions must be
/// strictly less than `2 * radius`, and at least three circle segments are required.
/// The cutter occupies `[radius - key_depth, radius]` by
/// `[-key_width / 2, key_width / 2]`. Native difference acts on the sampled polygon
/// and retains aggregate certainty; no analytic-circle fidelity bound is supplied.
pub fn circle_with_keyway(
    radius: Real,
    segments: usize,
    key_width: Real,
    key_depth: Real,
    context: GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    solid::sample_minimum(segments, 3, "segments")?;
    solid::capacity::<[Real; 2]>(Some(segments))?;
    positive_with(&radius, "radius", &decisions)?;
    positive_with(&key_width, "key_width", &decisions)?;
    positive_with(&key_depth, "key_depth", &decisions)?;
    let diameter = Real::from(2_u8) * radius.clone();
    if compare_with(&key_width, &diameter, &decisions)? != Ordering::Less
        || compare_with(&key_depth, &diameter, &decisions)? != Ordering::Less
    {
        return Err(ValidationError::InvalidArguments);
    }
    let key_y = -(key_width.clone() / Real::from(2_u8)).expect("two is nonzero");
    let cutter = translate_with(
        &rectangle_with(key_depth.clone(), key_width, &decisions)?,
        radius.clone() - key_depth,
        key_y,
        &decisions,
    )?;
    let operand = circle_with(radius, segments, &decisions)?;
    let region =
        decisions.exact_curve(|| operand.boolean_region(&cutter, BooleanOp::Difference))?;
    Ok(decisions.finish(region))
}

/// Clips an origin-centered sampled circle to the half-plane `y >= -flat_distance`.
///
/// Requires positive radius, at least three segments and
/// `0 <= flat_distance < radius`. The rectangular clip extends to the circle's
/// other bounds, so the retained material is above the lower horizontal cut.
/// For zero distance this retains the nonnegative-Y portion. The source boundary
/// already consists of chords; native intersection determines whether the cut
/// adds a new edge. The outcome retains aggregate predicate certainty.
pub fn circle_with_flat(
    radius: Real,
    segments: usize,
    flat_distance: Real,
    context: GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    solid::sample_minimum(segments, 3, "segments")?;
    solid::capacity::<[Real; 2]>(Some(segments))?;
    positive_with(&radius, "radius", &decisions)?;
    nonnegative_with(&flat_distance, "flat_distance", &decisions)?;
    if compare_with(&flat_distance, &radius, &decisions)? != Ordering::Less {
        return Err(ValidationError::InvalidArguments);
    }
    let diameter = Real::from(2_u8) * radius.clone();
    let clip = translate_with(
        &rectangle_with(diameter, radius.clone() + flat_distance.clone(), &decisions)?,
        -radius.clone(),
        -flat_distance,
        &decisions,
    )?;
    let operand = circle_with(radius, segments, &decisions)?;
    let region =
        decisions.exact_curve(|| operand.boolean_region(&clip, BooleanOp::Intersection))?;
    Ok(decisions.finish(region))
}

/// Clips an origin-centered sampled circle to `-flat_distance <= y <= flat_distance`.
///
/// Requires positive radius, at least three segments and
/// `0 <= flat_distance < radius`. Zero distance makes the rectangular clip empty
/// and yields the regularized empty region after input validation. For positive
/// distance, native intersection determines the retained strip of the sampled
/// polygon; its boundary need not meet both cut lines at very coarse resolution.
/// The outcome retains aggregate predicate certainty.
pub fn circle_with_two_flats(
    radius: Real,
    segments: usize,
    flat_distance: Real,
    context: GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    solid::sample_minimum(segments, 3, "segments")?;
    solid::capacity::<[Real; 2]>(Some(segments))?;
    positive_with(&radius, "radius", &decisions)?;
    nonnegative_with(&flat_distance, "flat_distance", &decisions)?;
    if compare_with(&flat_distance, &radius, &decisions)? != Ordering::Less {
        return Err(ValidationError::InvalidArguments);
    }
    let clip = translate_with(
        &rectangle_with(
            Real::from(2_u8) * radius.clone(),
            Real::from(2_u8) * flat_distance.clone(),
            &decisions,
        )?,
        -radius.clone(),
        -flat_distance,
        &decisions,
    )?;
    let operand = circle_with(radius, segments, &decisions)?;
    let region =
        decisions.exact_curve(|| operand.boolean_region(&clip, BooleanOp::Intersection))?;
    Ok(decisions.finish(region))
}

/// Origin-centered sampled involute gear with a tooth bisector on positive X.
///
/// For module `m` and tooth count `z`, the pitch, base, tip and root radii are
/// `R = m*z/2`, `R*cos(alpha)`, `R + m` and `R - 5*m/4 - clearance`, where
/// `alpha` is the pressure angle in radians. Clearance and backlash are lengths;
/// backlash reduces pitch-circle tooth thickness from `pi*m/2`.
///
/// Requires positive module, at least four teeth, at least two flank intervals,
/// pressure angle in `(0, 90)` degrees and nonnegative clearance/backlash.
/// Derived base/root radii and half-tooth angle must be positive, and paired
/// flanks must retain positive angular separation at their start and tip.
/// Uses strict validation and native ring admission.
///
/// `segments_per_flank` divides each involute flank uniformly in radius and each
/// tip/root circular span uniformly in angle. Roots below the base circle connect
/// radially to the flank. All samples become straight polygon edges; no bound on
/// error relative to the continuous involute outline is supplied.
pub fn involute_gear(
    module: impl IntoReal,
    teeth: usize,
    pressure_angle_degrees: impl IntoReal,
    clearance: impl IntoReal,
    backlash: impl IntoReal,
    segments_per_flank: usize,
) -> Result<CurveRegion2, ValidationError> {
    let module = module.into_real()?;
    let pressure_angle_degrees = pressure_angle_degrees.into_real()?;
    let clearance = clearance.into_real()?;
    let backlash = backlash.into_real()?;
    involute_gear_with_context(
        module,
        teeth,
        pressure_angle_degrees,
        clearance,
        backlash,
        segments_per_flank,
        GeometryContext::STRICT,
    )
    .map(GeometryOutcome::into_value)
}

/// Constructs [`involute_gear`] with explicit predicate policy and local certainty.
///
/// Validates tooth/flank counts, module, clearance, backlash and pressure angle
/// before the private sampler checks capacities and derived geometry. Decisions
/// selecting the root transition and separating the flanks contribute to the
/// receipt along with native admission. Scalar domain errors, invalid derived
/// dimensions, undecided predicates and capacity failures remain errors.
pub fn involute_gear_with_context(
    module: Real,
    teeth: usize,
    pressure_angle_degrees: Real,
    clearance: Real,
    backlash: Real,
    segments_per_flank: usize,
    context: GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    solid::sample_minimum(teeth, 4, "teeth")?;
    solid::sample_minimum(segments_per_flank, 2, "segments_per_flank")?;
    positive_with(&module, "module", &decisions)?;
    nonnegative_with(&clearance, "clearance", &decisions)?;
    nonnegative_with(&backlash, "backlash", &decisions)?;
    positive_with(&pressure_angle_degrees, "pressure_angle_degrees", &decisions)?;
    if compare_with(&pressure_angle_degrees, &Real::from(90_u8), &decisions)? != Ordering::Less
    {
        return Err(ValidationError::InvalidArguments);
    }
    let region = sampled_involute_gear(
        &module,
        teeth,
        &pressure_angle_degrees,
        &clearance,
        &backlash,
        segments_per_flank,
        &decisions,
    )?;
    Ok(decisions.finish(region))
}

/// Origin-centered polygon sampling epicycloidal tooth faces and hypocycloidal roots.
///
/// For module `m` and tooth count `z`, pitch radius is `R = m*z/2`, tip radius
/// is `R + m`, and root radius is `R - 5*m/4 - clearance`. Generating radius
/// `g` and clearance have the same length units as module. A tooth bisector lies
/// on positive X; angular tooth thickness at the pitch circle is half the angular pitch.
///
/// Requires positive module/generating radius, at least three teeth, at least
/// two flank intervals, nonnegative clearance and positive root radius. The
/// sampler requires `2*g < R` and `m <= 2*g`, selects first-lobe endpoints, and
/// rejects tip or root endpoints that reach the relevant tooth bisector. A root
/// below the hypocycloid's minimum radius uses radial connecting segments.
///
/// Each epicycloidal and hypocycloidal part has `segments_per_flank` intervals
/// uniform in rolling parameter. Tip/root circle samples use that many angular
/// intervals. Strict native admission assembles the straight-edged polygon;
/// there is no analytic-fidelity error bound.
pub fn cycloidal_gear(
    module: impl IntoReal,
    teeth: usize,
    generating_radius: impl IntoReal,
    clearance: impl IntoReal,
    segments_per_flank: usize,
) -> Result<CurveRegion2, ValidationError> {
    let module = module.into_real()?;
    let generating_radius = generating_radius.into_real()?;
    let clearance = clearance.into_real()?;
    cycloidal_gear_with_context(
        module,
        teeth,
        generating_radius,
        clearance,
        segments_per_flank,
        GeometryContext::STRICT,
    )
    .map(GeometryOutcome::into_value)
}

/// Constructs [`cycloidal_gear`] with explicit predicate policy and local certainty.
///
/// Checks tooth/flank counts, generating radius, module and clearance before the
/// private sampler checks capacity and derived radii. The selected root cutoff,
/// endpoint separation and radial transition decisions remain in the receipt;
/// failed scalar constructions and unresolved predicates propagate as errors.
pub fn cycloidal_gear_with_context(
    module: Real,
    teeth: usize,
    generating_radius: Real,
    clearance: Real,
    segments_per_flank: usize,
    context: GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    solid::sample_minimum(teeth, 3, "teeth")?;
    solid::sample_minimum(segments_per_flank, 2, "segments_per_flank")?;
    positive_with(&generating_radius, "generating_radius", &decisions)?;
    positive_with(&module, "module", &decisions)?;
    nonnegative_with(&clearance, "clearance", &decisions)?;
    let region = sampled_cycloidal_gear(
        &module,
        teeth,
        &generating_radius,
        &clearance,
        segments_per_flank,
        &decisions,
    )?;
    Ok(decisions.finish(region))
}

/// Straight-sided rack with its pitch line on `y = 0` and first tooth centered at `x = 0`.
///
/// For module `m`, pitch is `pi*m`, tip height is `m`, root height is
/// `-(5*m/4 + clearance)`, and a backing strip extends one module below the root.
/// Teeth advance along positive X. Backlash is a length subtracted from the
/// pitch-line tooth thickness `pi*m/2`; flank slope is set by the pressure angle.
/// The end walls lie at the first and last outer root-flank endpoints.
///
/// Requires at least one tooth, positive module, pressure angle in `(0, 90)`
/// degrees, nonnegative clearance/backlash, and positive derived pitch thickness,
/// tip width and root gap. Uses strict decisions and native polygon admission;
/// the rack flanks themselves are straight segments, with no curved-flank sampling.
pub fn involute_rack(
    module: impl IntoReal,
    teeth: usize,
    pressure_angle_degrees: impl IntoReal,
    clearance: impl IntoReal,
    backlash: impl IntoReal,
) -> Result<CurveRegion2, ValidationError> {
    let module = module.into_real()?;
    let pressure_angle_degrees = pressure_angle_degrees.into_real()?;
    let clearance = clearance.into_real()?;
    let backlash = backlash.into_real()?;
    involute_rack_with_context(
        module,
        teeth,
        pressure_angle_degrees,
        clearance,
        backlash,
        GeometryContext::STRICT,
    )
    .map(GeometryOutcome::into_value)
}

/// Constructs [`involute_rack`] with explicit predicate policy and local certainty.
///
/// After primary parameter validation, checks capacity for `6*teeth + 2` vertices.
/// Let `h = 5*module/4 + clearance`, `w = pi*module/2 - backlash` and
/// `u = tan(alpha)` for pressure angle `alpha` in radians.
/// Requires `w > 0`, `w - 2*module*u > 0` and
/// `pi*module - w - 2*h*u > 0` before constructing any tooth ring. The final
/// receipt includes these decisions and native admission.
pub fn involute_rack_with_context(
    module: Real,
    teeth: usize,
    pressure_angle_degrees: Real,
    clearance: Real,
    backlash: Real,
    context: GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    solid::sample_minimum(teeth, 1, "teeth")?;
    positive_with(&module, "module", &decisions)?;
    positive_with(&pressure_angle_degrees, "pressure_angle_degrees", &decisions)?;
    nonnegative_with(&clearance, "clearance", &decisions)?;
    nonnegative_with(&backlash, "backlash", &decisions)?;
    if compare_with(&pressure_angle_degrees, &Real::from(90_u8), &decisions)? != Ordering::Less
    {
        return Err(ValidationError::InvalidArguments);
    }
    let point_capacity = solid::capacity::<[Real; 2]>(
        teeth.checked_mul(6).and_then(|count| count.checked_add(2)),
    )?;
    let pressure_angle = (pressure_angle_degrees * Real::pi() / Real::from(180_u16))
        .expect("degrees denominator is nonzero");
    let pitch = Real::pi() * module.clone();
    let dedendum = module.clone()
        * (Real::from(5_u8) / Real::from(4_u8)).expect("four is nonzero")
        + clearance;
    let root = -dedendum.clone();
    let half_pitch = (pitch.clone() / Real::from(2_u8))?;
    let tooth_thickness = half_pitch - backlash;
    let half_thickness = (tooth_thickness.clone() / Real::from(2_u8)).expect("two is nonzero");
    let tangent = (pressure_angle.clone().sin() / pressure_angle.cos())?;
    let root_slant = dedendum * tangent.clone();
    let tip_slant = module.clone() * tangent;
    let tip_width = tooth_thickness.clone() - Real::from(2_u8) * tip_slant.clone();
    let root_space =
        pitch.clone() - tooth_thickness.clone() - Real::from(2_u8) * root_slant.clone();
    positive_with(&tooth_thickness, "tooth_thickness", &decisions)?;
    positive_with(&tip_width, "tip_width", &decisions)?;
    positive_with(&root_space, "root_space", &decisions)?;
    let first_x = -half_thickness.clone() - root_slant.clone();
    let mut points = solid::reserved(point_capacity)?;
    points.push([first_x, root.clone()]);
    for tooth in 0..teeth {
        let center = Real::from(tooth as u64) * pitch.clone();
        let left_pitch = center.clone() - half_thickness.clone();
        let right_pitch = center + half_thickness.clone();
        points.extend([
            [left_pitch.clone(), Real::zero()],
            [left_pitch + tip_slant.clone(), module.clone()],
            [right_pitch.clone() - tip_slant.clone(), module.clone()],
            [right_pitch.clone(), Real::zero()],
            [right_pitch + root_slant.clone(), root.clone()],
        ]);
        if tooth + 1 < teeth {
            let next_left =
                Real::from((tooth + 1) as u64) * pitch.clone() - half_thickness.clone();
            points.push([next_left - root_slant.clone(), root.clone()]);
        }
    }
    let left = points.first().expect("validated rack has a left root")[0].clone();
    let right = points.last().expect("validated rack has a right root")[0].clone();
    let backing = root - module;
    points.push([right, backing.clone()]);
    points.push([left, backing]);
    Ok(decisions.finish(ring_with(&points, &decisions)?))
}

/// Sampled cycloidal rack with a horizontal backing strip below its tooth roots.
///
/// Pitch is `pi*module`, the first tooth is centered at `x = 0`, and the pitch
/// line is `y = 0`. Teeth extend along positive X. Addendum height is `module`;
/// dedendum depth is `5*module/4 + clearance`. The backing extends one module
/// below the root, from `x = -pitch/2` to `(teeth - 1/2)*pitch`.
///
/// Each side joins an addendum cycloid above the pitch line to a dedendum cycloid
/// below it. Each rolling radius is twice its corresponding height; uniform
/// rolling-parameter samples span one sixth turn. Horizontal lands join tips
/// and neighboring roots, and all intervening samples are connected by chords.
///
/// Requires positive module, at least one tooth, at least four intervals per
/// cycloidal part and nonnegative clearance. Each flank's horizontal run must be
/// less than a quarter pitch, leaving positive tip and root lands. Uses strict
/// admission; no analytic-fidelity error bound is supplied.
pub fn cycloidal_rack(
    module: impl IntoReal,
    teeth: usize,
    clearance: impl IntoReal,
    segments_per_flank: usize,
) -> Result<CurveRegion2, ValidationError> {
    let module = module.into_real()?;
    let clearance = clearance.into_real()?;
    cycloidal_rack_with_context(
        module,
        teeth,
        clearance,
        segments_per_flank,
        GeometryContext::STRICT,
    )
    .map(GeometryOutcome::into_value)
}

/// Constructs [`cycloidal_rack`] with explicit predicate policy and local certainty.
///
/// Checks capacities for the reusable flank, complete tooth, repeated top and
/// backed polygon. The normalized flank is
/// `(2*(theta - sin(theta)), 2*(1 - cos(theta)))` for `0 <= theta <= pi/3`;
/// addendum and dedendum scale it by their respective heights. Policy comparisons
/// require both terminal runs to fit within a quarter pitch. Repetition skips
/// shared root-land endpoints, and native ring admission finishes the receipt.
pub fn cycloidal_rack_with_context(
    module: Real,
    teeth: usize,
    clearance: Real,
    segments_per_flank: usize,
    context: GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    solid::sample_minimum(teeth, 1, "teeth")?;
    solid::sample_minimum(segments_per_flank, 4, "segments_per_flank")?;
    positive_with(&module, "module", &decisions)?;
    nonnegative_with(&clearance, "clearance", &decisions)?;
    let sample_capacity = solid::capacity::<[Real; 2]>(segments_per_flank.checked_add(1))?;
    let tooth_capacity = solid::capacity::<[Real; 2]>(
        segments_per_flank
            .checked_mul(4)
            .and_then(|count| count.checked_add(4)),
    )?;
    let top_capacity = solid::capacity::<[Real; 2]>(
        teeth
            .checked_mul(tooth_capacity - 1)
            .and_then(|count| count.checked_add(1)),
    )?;
    let point_capacity = solid::capacity::<[Real; 2]>(top_capacity.checked_add(2))?;
    let two = Real::from(2_u8);
    let three = Real::from(3_u8);
    let four = Real::from(4_u8);
    let pitch = Real::pi() * module.clone();
    let half_pitch = (pitch.clone() / two.clone()).expect("two is nonzero");
    let quarter_pitch = (pitch.clone() / four.clone()).expect("four is nonzero");
    let dedendum =
        module.clone() * (Real::from(5_u8) / four).expect("four is nonzero") + clearance;
    let root = -dedendum.clone();
    let flank_sweep = (Real::pi() / three).expect("three is nonzero");
    let run_factor = two.clone() * (flank_sweep.clone() - flank_sweep.clone().sin());
    let face_run = module.clone() * run_factor.clone();
    let root_run = dedendum.clone() * run_factor;
    if compare_with(&face_run, &quarter_pitch, &decisions)? != Ordering::Less
        || compare_with(&root_run, &quarter_pitch, &decisions)? != Ordering::Less
    {
        return Err(ValidationError::InvalidArguments);
    }

    let mut unit_flank = solid::reserved(sample_capacity)?;
    for sample in 0..=segments_per_flank {
        let fraction = exact_ratio(sample, segments_per_flank)?;
        let theta = flank_sweep.clone() * fraction;
        unit_flank.push([
            two.clone() * (theta.clone() - theta.clone().sin()),
            two.clone() * (Real::one() - theta.cos()),
        ]);
    }

    let mut tooth_points = solid::reserved(tooth_capacity)?;
    tooth_points.push([-half_pitch.clone(), root.clone()]);
    tooth_points.extend(unit_flank.iter().rev().map(|[run, rise]| {
        [
            -quarter_pitch.clone() - dedendum.clone() * run.clone(),
            -(dedendum.clone() * rise.clone()),
        ]
    }));
    tooth_points.extend(unit_flank.iter().skip(1).map(|[run, rise]| {
        [
            -quarter_pitch.clone() + module.clone() * run.clone(),
            module.clone() * rise.clone(),
        ]
    }));
    tooth_points.push([quarter_pitch.clone() - face_run, module.clone()]);
    tooth_points.extend(
        unit_flank[..segments_per_flank]
            .iter()
            .rev()
            .map(|[run, rise]| {
                [
                    quarter_pitch.clone() - module.clone() * run.clone(),
                    module.clone() * rise.clone(),
                ]
            }),
    );
    tooth_points.extend(unit_flank.iter().skip(1).map(|[run, rise]| {
        [
            quarter_pitch.clone() + dedendum.clone() * run.clone(),
            -(dedendum.clone() * rise.clone()),
        ]
    }));
    tooth_points.push([half_pitch.clone(), root.clone()]);

    let mut top = solid::reserved(top_capacity)?;
    for tooth in 0..teeth {
        let offset = Real::from(tooth as u64) * pitch.clone();
        for (point_index, point) in tooth_points.iter().enumerate() {
            if tooth > 0 && point_index == 0 {
                continue;
            }
            top.push([offset.clone() + point[0].clone(), point[1].clone()]);
        }
    }

    let mut points = solid::reserved(point_capacity)?;
    let backing = root.clone() - module;
    let left = top.first().expect("validated rack has a left root")[0].clone();
    let right = top.last().expect("validated rack has a right root")[0].clone();
    points.push([left, backing.clone()]);
    points.push([right, backing]);
    points.extend(top.into_iter().rev());
    Ok(decisions.finish(ring_with(&points, &decisions)?))
}

/// Polygon sampling a NACA four-digit airfoil along a positive-X chord from the origin.
///
/// Inputs are digit values: `max_camber / 100` is maximum camber as a chord
/// fraction, `camber_position / 10` locates it, and `thickness / 100` is the
/// thickness fraction. Inputs need not be integers. Camber and position must be
/// in `[0, 10)`, thickness in `(0, 100)`, chord positive, and `samples >= 10`.
/// Positive camber requires positive camber position; zero camber selects the
/// symmetric mean line. Validation uses the strict policy.
///
/// Stations are uniform in normalized chord coordinate, `x = i / samples`,
/// including both ends. Thickness offsets follow the normal to the piecewise
/// parabolic mean line. The thickness coefficients are the exact represented
/// binary64 literals `0.2969`, `0.1260`, `0.3516`, `0.2843`, `0.1015`, not their
/// exact decimal fractions.
///
/// The ring keeps all upper stations and reversed lower interior stations,
/// for `2*samples` authored vertices. It omits the lower trailing-edge endpoint:
/// a straight edge joins the upper trailing edge to the last lower interior
/// station. The leading-edge endpoint is shared. No analytic-fidelity or exact
/// chordwise-bounds guarantee follows from this sampled construction.
pub fn airfoil_naca4(
    max_camber: impl IntoReal,
    camber_position: impl IntoReal,
    thickness: impl IntoReal,
    chord: impl IntoReal,
    samples: usize,
) -> Result<CurveRegion2, ValidationError> {
    let max_camber = max_camber.into_real()?;
    let camber_position = camber_position.into_real()?;
    let thickness = thickness.into_real()?;
    let chord = chord.into_real()?;
    airfoil_naca4_with_context(
        max_camber,
        camber_position,
        thickness,
        chord,
        samples,
        GeometryContext::STRICT,
    )
    .map(GeometryOutcome::into_value)
}

/// Constructs [`airfoil_naca4`] with explicit predicate policy and local certainty.
///
/// Parameter validation selects cambered versus symmetric mean lines before
/// checking storage capacities. For normalized fractions `m`, `p`, `t`, the
/// mean line is `m*(2*p*x - x^2)/p^2` when `x < p`, otherwise
/// `m*(1 - 2*p + 2*p*x - x^2)/(1-p)^2`; its derivative gives the normal angle
/// `atan(dy/dx)`. A selected zero camber uses zero height and slope throughout.
///
/// The half-thickness is `5*t*(a0*sqrt(x) - a1*x - a2*x^2 + a3*x^3 - a4*x^4)`
/// with the binary64 coefficients listed in [`airfoil_naca4`]. Upper/lower points
/// are `(x -/+ yt*sin(theta), yc +/- yt*cos(theta))`, scaled by chord. Camber
/// selection and per-station comparisons retain their qualification, including
/// when the resulting ring reuses a strict cache entry. Scalar failures remain typed.
pub fn airfoil_naca4_with_context(
    max_camber: Real,
    camber_position: Real,
    thickness: Real,
    chord: Real,
    samples: usize,
    context: GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    solid::sample_minimum(samples, 10, "samples")?;
    let cambered = nonnegative_with(&max_camber, "max_camber", &decisions)?
        == hyperreal::RealSign::Positive;
    nonnegative_with(&camber_position, "camber_position", &decisions)?;
    positive_with(&thickness, "thickness", &decisions)?;
    positive_with(&chord, "chord", &decisions)?;
    if compare_with(&max_camber, &Real::from(10_u8), &decisions)? != Ordering::Less
        || compare_with(&camber_position, &Real::from(10_u8), &decisions)? != Ordering::Less
        || compare_with(&thickness, &Real::from(100_u8), &decisions)? != Ordering::Less
    {
        return Err(ValidationError::InvalidArguments);
    }
    let m = (max_camber / Real::from(100_u8))?;
    let p = (camber_position / Real::from(10_u8))?;
    let t = (thickness / Real::from(100_u8))?;
    if cambered {
        positive_with(&p, "camber_position", &decisions)?;
    }
    let coefficient = |value: f64| Real::try_from(value).expect("finite NACA coefficient");
    let sample_capacity = solid::capacity::<[Real; 2]>(samples.checked_add(1))?;
    let point_capacity = solid::capacity::<[Real; 2]>(samples.checked_mul(2))?;
    let mut upper = solid::reserved(point_capacity)?;
    let mut lower = solid::reserved(sample_capacity)?;
    for index in 0..=samples {
        let x = exact_ratio(index, samples)?;
        let x2 = x.clone() * x.clone();
        let x3 = x2.clone() * x.clone();
        let x4 = x3.clone() * x.clone();
        let root_x = x.clone().sqrt()?;
        let yt = Real::from(5_u8)
            * t.clone()
            * (coefficient(0.2969) * root_x
                - coefficient(0.1260) * x.clone()
                - coefficient(0.3516) * x2.clone()
                + coefficient(0.2843) * x3
                - coefficient(0.1015) * x4);
        let (yc, dy) = if cambered {
            let x_ordering = compare_with(&x, &p, &decisions)?;
            if x_ordering == Ordering::Less {
                let p2 = p.clone() * p.clone();
                let y_factor = (m.clone() / p2.clone())?;
                let slope_factor = (Real::from(2_u8) * m.clone() / p2)?;
                (
                    y_factor * (Real::from(2_u8) * p.clone() * x.clone() - x2.clone()),
                    slope_factor * (p.clone() - x.clone()),
                )
            } else {
                let q = Real::one() - p.clone();
                let denominator = q.clone() * q;
                let y_factor = (m.clone() / denominator.clone())?;
                let slope_factor = (Real::from(2_u8) * m.clone() / denominator)?;
                (
                    y_factor
                        * (Real::one() - Real::from(2_u8) * p.clone()
                            + Real::from(2_u8) * p.clone() * x.clone()
                            - x2.clone()),
                    slope_factor * (p.clone() - x.clone()),
                )
            }
        } else {
            (Real::zero(), Real::zero())
        };
        let theta = dy.atan()?;
        let sin_theta = theta.clone().sin();
        let cos_theta = theta.cos();
        upper.push([
            chord.clone() * (x.clone() - yt.clone() * sin_theta.clone()),
            chord.clone() * (yc.clone() + yt.clone() * cos_theta.clone()),
        ]);
        lower.push([
            chord.clone() * (x + yt.clone() * sin_theta),
            chord.clone() * (yc - yt * cos_theta),
        ]);
    }
    upper.extend(lower.into_iter().rev().skip(1).take(samples - 1));
    Ok(decisions.finish(ring_with(&upper, &decisions)?))
}

/// Builds one sampled involute tooth, rotates it exactly, and admits the resulting ring.
///
/// The caller validates primary counts and dimensions. This helper checks
/// per-tooth and total point capacity, derived positive radii and half-thickness,
/// then compares root radius with base radius to select an optional radial foot.
/// For base radius `b`, the polar involute deflection at radius `r >= b` is
/// `u - atan(u)` with `u = sqrt((r/b)^2 - 1)`. Adding the pitch deflection to
/// half the pitch-circle tooth angle fixes both flank rotations.
///
/// Flanks use uniform radius samples from `max(root, base)` to tip. Tip and root
/// lands use uniform angle samples; the next rotated tooth supplies the root
/// land's final endpoint. Optional radial segments bridge a root below the base
/// circle. The shared decision scope retains all comparisons and ring admission;
/// this sampling does not establish a continuous-outline error bound.
fn sampled_involute_gear(
    module: &Real,
    teeth: usize,
    pressure_angle_degrees: &Real,
    clearance: &Real,
    backlash: &Real,
    segments_per_flank: usize,
    decisions: &GeometryDecisions,
) -> Result<CurveRegion2, ValidationError> {
    let tooth_capacity = solid::capacity::<[Real; 2]>(
        segments_per_flank
            .checked_mul(4)
            .and_then(|count| count.checked_add(3)),
    )?;
    solid::capacity::<[Real; 2]>(teeth.checked_mul(tooth_capacity))?;
    let two = Real::from(2_u8);
    let four = Real::from(4_u8);
    let tooth_count = Real::from(teeth as u64);
    let pressure_angle = (pressure_angle_degrees.clone() * Real::pi() / Real::from(180_u16))?;
    let pitch_radius = (module.clone() * tooth_count.clone() / two.clone())?;
    let base_radius = pitch_radius.clone() * pressure_angle.cos();
    let outer_radius = pitch_radius.clone() + module.clone();
    let root_radius = pitch_radius.clone()
        - (module.clone() * Real::from(5_u8) / four.clone())?
        - clearance.clone();
    let angular_pitch = (Real::tau() / tooth_count)?;
    let backlash_angle = (backlash.clone() / pitch_radius.clone())?;
    let half_tooth_angle = (((angular_pitch.clone() / two.clone())? - backlash_angle) / two)?;
    positive_with(&base_radius, "base_radius", decisions)?;
    positive_with(&root_radius, "root_radius", decisions)?;
    positive_with(&half_tooth_angle, "half_tooth_angle", decisions)?;
    let involute_angle = |radius: &Real| -> Result<Real, ValidationError> {
        let ratio = (radius.clone() / base_radius.clone())?;
        let parameter = (ratio.clone() * ratio - Real::one()).sqrt()?;
        Ok(parameter.clone() - parameter.atan()?)
    };
    let (flank_start_radius, has_root_transition) =
        match compare_with(&root_radius, &base_radius, decisions)? {
            Ordering::Less => (base_radius.clone(), true),
            Ordering::Equal | Ordering::Greater => (root_radius.clone(), false),
        };
    let pitch_involute = involute_angle(&pitch_radius)?;
    let start_involute = involute_angle(&flank_start_radius)?;
    let outer_involute = involute_angle(&outer_radius)?;
    let offset = half_tooth_angle + pitch_involute;
    let right_start = start_involute.clone() - offset.clone();
    let left_start = offset.clone() - start_involute.clone();
    let right_tip = outer_involute.clone() - offset.clone();
    let left_tip = offset.clone() - outer_involute.clone();
    if compare_with(&right_start, &left_start, decisions)? != Ordering::Less
        || compare_with(&right_tip, &left_tip, decisions)? != Ordering::Less
    {
        return Err(ValidationError::InvalidArguments);
    }

    let mut flank_samples = solid::reserved(segments_per_flank)?;
    for sample in 1..segments_per_flank {
        let fraction = exact_ratio(sample, segments_per_flank)?;
        let radius = flank_start_radius.clone()
            + fraction * (outer_radius.clone() - flank_start_radius.clone());
        flank_samples.push((radius.clone(), involute_angle(&radius)?));
    }
    flank_samples.push((outer_radius.clone(), outer_involute));

    let mut tooth = solid::reserved(tooth_capacity)?;
    tooth.push(exact_polar(&root_radius, right_start.clone()));
    if has_root_transition {
        tooth.push(exact_polar(&flank_start_radius, right_start.clone()));
    }
    for (radius, angle) in &flank_samples {
        tooth.push(exact_polar(radius, angle.clone() - offset.clone()));
    }
    for sample in 1..=segments_per_flank {
        let fraction = exact_ratio(sample, segments_per_flank)?;
        tooth.push(exact_polar(
            &outer_radius,
            right_tip.clone() + fraction * (left_tip.clone() - right_tip.clone()),
        ));
    }
    for (radius, angle) in flank_samples[..flank_samples.len() - 1].iter().rev() {
        tooth.push(exact_polar(radius, offset.clone() - angle.clone()));
    }
    tooth.push(exact_polar(
        &flank_start_radius,
        offset.clone() - start_involute,
    ));
    if has_root_transition {
        tooth.push(exact_polar(&root_radius, left_start.clone()));
    }
    let next_right = angular_pitch + right_start;
    for sample in 1..segments_per_flank {
        let fraction = exact_ratio(sample, segments_per_flank)?;
        tooth.push(exact_polar(
            &root_radius,
            left_start.clone() + fraction * (next_right.clone() - left_start.clone()),
        ));
    }
    let points = repeat_exact_rotational_pattern(&tooth, teeth)?;
    ring_with(&points, decisions)
}

/// Builds and rotates a tooth from first-lobe epicycloid and hypocycloid samples.
///
/// The caller validates primary counts and dimensions. This helper checks point
/// capacities, positive root radius, `2*g < R`, and tip reach `R + module <= R + 2*g`,
/// where `R` is pitch radius and `g` generating radius. Principal `acos` phases
/// select the tip radius and `max(root, R - 2*g)` on the first lobes.
///
/// For parameter `u`, the epicycloid is
/// `((R+g)*cos(u)-g*cos((R+g)*u/g), (R+g)*sin(u)-g*sin((R+g)*u/g))`;
/// the hypocycloid is
/// `((R-g)*cos(u)+g*cos((R-g)*u/g), (R-g)*sin(u)-g*sin((R-g)*u/g))`.
/// The latter is sampled at negative parameters for the right root flank. Endpoint
/// cross-product comparisons against the quarter-pitch ray require positive tip
/// and neighboring-root gaps before rotation by minus a quarter pitch.
///
/// Mirroring assembles the left flank. Uniform angular tip/root lands and optional
/// radial feet complete each tooth's boundary sequence. Exact rotational repetition
/// and native ring admission then form the region. All policy-selected comparisons
/// share the caller's receipt; scalar-domain failures and capacity errors propagate.
fn sampled_cycloidal_gear(
    module: &Real,
    teeth: usize,
    generating_radius: &Real,
    clearance: &Real,
    segments_per_flank: usize,
    decisions: &GeometryDecisions,
) -> Result<CurveRegion2, ValidationError> {
    let tooth_capacity = solid::capacity::<[Real; 2]>(
        segments_per_flank
            .checked_mul(6)
            .and_then(|count| count.checked_add(2)),
    )?;
    solid::capacity::<[Real; 2]>(teeth.checked_mul(tooth_capacity))?;
    let two = Real::from(2_u8);
    let pitch_radius = (module.clone() * Real::from(teeth as u64) / two.clone())?;
    let outer_radius = pitch_radius.clone() + module.clone();
    let root_radius = pitch_radius.clone()
        - (module.clone() * Real::from(5_u8) / Real::from(4_u8))?
        - clearance.clone();
    let twice_generator = two.clone() * generating_radius.clone();
    positive_with(&root_radius, "root_radius", decisions)?;
    if compare_with(&twice_generator, &pitch_radius, decisions)? != Ordering::Less {
        return Err(ValidationError::InvalidArguments);
    }
    let epicycle_radius = pitch_radius.clone() + generating_radius.clone();
    let hypocycle_radius = pitch_radius.clone() - generating_radius.clone();
    if compare_with(
        &outer_radius,
        &(pitch_radius.clone() + twice_generator.clone()),
        decisions,
    )? == Ordering::Greater
    {
        return Err(ValidationError::InvalidArguments);
    }
    let generated_root_radius = pitch_radius.clone() - twice_generator;
    let flank_root_radius =
        match compare_with(&root_radius, &generated_root_radius, decisions)? {
            Ordering::Less => generated_root_radius,
            Ordering::Equal | Ordering::Greater => root_radius.clone(),
        };
    let square = |value: &Real| value.clone() * value.clone();
    let tip_phase = ((square(&epicycle_radius) + square(generating_radius)
        - square(&outer_radius))
        / (two.clone() * generating_radius.clone() * epicycle_radius.clone()))?
    .acos()?;
    let tip_parameter =
        (tip_phase.clone() * generating_radius.clone() / pitch_radius.clone())?;
    let root_phase = ((square(&flank_root_radius)
        - square(&hypocycle_radius)
        - square(generating_radius))
        / (two.clone() * generating_radius.clone() * hypocycle_radius.clone()))?
    .acos()?;
    let root_parameter =
        (root_phase.clone() * generating_radius.clone() / pitch_radius.clone())?;
    let epicycle_ratio = (epicycle_radius.clone() / generating_radius.clone())?;
    let hypocycle_ratio = (hypocycle_radius.clone() / generating_radius.clone())?;
    let angular_pitch = (Real::tau() / Real::from(teeth as u64))?;
    let pitch_half_thickness = (angular_pitch.clone() / Real::from(4_u8))?;
    let epicycloid = |parameter: Real| {
        let carrier = parameter.clone();
        let rolling = epicycle_ratio.clone() * parameter;
        [
            epicycle_radius.clone() * carrier.clone().cos()
                - generating_radius.clone() * rolling.clone().cos(),
            epicycle_radius.clone() * carrier.sin()
                - generating_radius.clone() * rolling.sin(),
        ]
    };
    let hypocycloid = |parameter: Real| {
        let carrier = parameter.clone();
        let rolling = hypocycle_ratio.clone() * parameter;
        [
            hypocycle_radius.clone() * carrier.clone().cos()
                + generating_radius.clone() * rolling.clone().cos(),
            hypocycle_radius.clone() * carrier.sin()
                - generating_radius.clone() * rolling.sin(),
        ]
    };
    let sample_capacity = solid::capacity::<[Real; 2]>(segments_per_flank.checked_add(1))?;
    let mut epi = solid::reserved(sample_capacity)?;
    let mut hypo = solid::reserved(sample_capacity)?;
    for sample in 0..=segments_per_flank {
        let fraction = exact_ratio(sample, segments_per_flank)?;
        epi.push(epicycloid(tip_parameter.clone() * fraction.clone()));
        hypo.push(hypocycloid(-(root_parameter.clone() * fraction)));
    }

    let limit_sine = pitch_half_thickness.clone().sin();
    let limit_cosine = pitch_half_thickness.clone().cos();
    let precedes_tooth_bisector = |[x, y]: &[Real; 2]| -> Result<bool, ValidationError> {
        Ok(compare_with(
            &(y.clone() * limit_cosine.clone() - x.clone() * limit_sine.clone()),
            &Real::zero(),
            decisions,
        )? == Ordering::Less)
    };
    let negative_root_endpoint = hypo.last().expect("validated flank has an endpoint");
    let positive_root_endpoint = [
        negative_root_endpoint[0].clone(),
        -negative_root_endpoint[1].clone(),
    ];
    if !precedes_tooth_bisector(epi.last().expect("validated flank has an endpoint"))?
        || !precedes_tooth_bisector(&positive_root_endpoint)?
    {
        return Err(ValidationError::InvalidArguments);
    }

    let flank_rotation = -pitch_half_thickness;
    let flank_sine = flank_rotation.clone().sin();
    let flank_cosine = flank_rotation.cos();
    let rotate = |[x, y]: &[Real; 2]| {
        [
            x.clone() * flank_cosine.clone() - y.clone() * flank_sine.clone(),
            x.clone() * flank_sine.clone() + y.clone() * flank_cosine.clone(),
        ]
    };
    let mut right_epi = solid::reserved(sample_capacity)?;
    right_epi.extend(epi.iter().map(rotate));
    let mut right_hypo = solid::reserved(sample_capacity)?;
    right_hypo.extend(hypo.iter().map(rotate));
    let right_tip_point = right_epi.last().expect("validated flank has an endpoint");
    let right_root_point = right_hypo.last().expect("validated flank has an endpoint");
    let right_tip = right_tip_point[1].clone().atan2(right_tip_point[0].clone());
    let right_root = right_root_point[1].clone().atan2(right_root_point[0].clone());
    let left_tip = -right_tip.clone();
    let left_root = -right_root.clone();
    let next_right_root = angular_pitch.clone() + right_root.clone();
    let mut tooth = solid::reserved(tooth_capacity)?;
    let has_root_transition =
        compare_with(&flank_root_radius, &root_radius, decisions)? == Ordering::Greater;
    if has_root_transition {
        tooth.push(exact_polar(&root_radius, right_root.clone()));
    }
    tooth.extend(right_hypo.iter().rev().cloned());
    tooth.extend(right_epi.iter().skip(1).cloned());
    for sample in 1..=segments_per_flank {
        let fraction = exact_ratio(sample, segments_per_flank)?;
        tooth.push(exact_polar(
            &outer_radius,
            right_tip.clone() + fraction * (left_tip.clone() - right_tip.clone()),
        ));
    }
    tooth.extend(
        right_epi[..segments_per_flank]
            .iter()
            .rev()
            .map(|[x, y]| [x.clone(), -y.clone()]),
    );
    tooth.extend(
        right_hypo
            .iter()
            .skip(1)
            .map(|[x, y]| [x.clone(), -y.clone()]),
    );
    if has_root_transition {
        tooth.push(exact_polar(&root_radius, left_root.clone()));
    }
    for sample in 1..segments_per_flank {
        let fraction = exact_ratio(sample, segments_per_flank)?;
        tooth.push(exact_polar(
            &root_radius,
            left_root.clone() + fraction * (next_right_root.clone() - left_root.clone()),
        ));
    }
    let points = repeat_exact_rotational_pattern(&tooth, teeth)?;
    ring_with(&points, decisions)
}

/// Constructs a path containing one exact positive-degree polynomial Bezier curve.
///
/// At least two controls and a positive `display_segments` value are required.
/// Two, three and four controls use native line, quadratic and cubic carriers;
/// more controls use a rational Bezier carrier with all weights equal to one.
/// The parameter domain is `[0, 1]`, traversed from the first to last control.
/// The strict policy governs native construction; a two-control zero-length line
/// fails with its native cause.
///
/// The display count is otherwise unused: it does not sample, subdivide or change
/// the control net. A path may have coincident endpoints without acquiring fill
/// semantics; use [`bezier_region`] to admit a closed boundary as a region.
pub fn bezier_path(
    control: &[[Real; 2]],
    display_segments: usize,
) -> Result<CurvePath2, ValidationError> {
    bezier_path_with_context(control, display_segments, GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Constructs the single-curve path of [`bezier_path`] with explicit policy and local certainty.
///
/// Controls are cloned exactly into the chosen native carrier, then wrapped in
/// a nonempty path. Only the minimum display count is checked, so even
/// `usize::MAX` does not request a sampling buffer. Native line, curve and path
/// construction failures preserve their typed causes. This constructor retains
/// the source definition; it does not establish every subsequent operation's support.
pub fn bezier_path_with_context(
    control: &[[Real; 2]],
    display_segments: usize,
    context: GeometryContext,
) -> Result<GeometryOutcome<CurvePath2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    if control.len() < 2 || display_segments < 1 {
        return Err(ValidationError::InvalidArguments);
    }
    let points = control
        .iter()
        .map(|point| Point2::new(point[0].clone(), point[1].clone()))
        .collect::<Vec<_>>();
    let path = decisions.exact_curve(|| -> Result<_, ValidationError> {
        let curve = match points.as_slice() {
            [start, end] => Curve2::from(LineSeg2::try_new(start.clone(), end.clone())?),
            [start, control, end] => Curve2::from(QuadraticBezier2::new(
                start.clone(),
                control.clone(),
                end.clone(),
            )),
            [start, control1, control2, end] => Curve2::from(CubicBezier2::new(
                start.clone(),
                control1.clone(),
                control2.clone(),
                end.clone(),
            )),
            _ => Curve2::from(RationalBezier2::try_new(
                points,
                vec![Real::one(); control.len()],
            )?),
        };
        Ok(CurvePath2::try_new(vec![curve])?)
    })?;
    Ok(decisions.finish(path))
}

/// Admits a closed exact Bezier boundary as an even-odd filled region.
///
/// Uses the control and display-count rules of [`bezier_path`] under the strict
/// policy, then requires the curve's first and last endpoints to coincide.
/// No closing line is added. Native boundary-path admission determines the
/// regularized filled set; a successfully admitted retraced boundary can be empty.
/// Unresolved closure or region admission remains an error, not empty success.
pub fn bezier_region(
    control: &[[Real; 2]],
    display_segments: usize,
) -> Result<CurveRegion2, ValidationError> {
    bezier_region_with_context(control, display_segments, GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Admits [`bezier_region`] with explicit policy and aggregate local certainty.
///
/// Consumes the path-construction receipt, checks endpoint coincidence, and uses
/// native `EvenOdd` boundary-path admission without sampling the curve. A decided
/// nonclosure is `InvalidArguments`; native uncertainty or unsupported work keeps
/// its typed diagnostic. The outcome combines construction, closure and region
/// admission decisions.
pub fn bezier_region_with_context(
    control: &[[Real; 2]],
    display_segments: usize,
    context: GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let path = decisions.consume_geometry(bezier_path_with_context(
        control,
        display_segments,
        context,
    )?);
    if !decisions.exact_curve(|| path.start().coincides_with(&path.end()))? {
        return Err(ValidationError::InvalidArguments);
    }
    let region = decisions.exact_curve(|| {
        CurveRegion2::try_from_boundary_paths(
            std::slice::from_ref(&path),
            hypercurve::FillRule::EvenOdd,
        )
    })?;
    Ok(decisions.finish(region))
}

/// Constructs one retained polynomial B-spline with clamped uniform knots.
///
/// Requires positive degree `d`, at least `d + 1` controls and a positive
/// `display_segments_per_span`. For `C` controls there are `C - d` unit knot spans.
/// The knots start with `d + 1` zeros, have simple interior integers, and end with
/// `d + 1` copies of `C - d`. Evaluation uses the knot domain `[0, C - d]`;
/// traversal runs from the first to last control.
///
/// The path retains a native polynomial B-spline rather than a sampled or eagerly
/// expanded Bezier chain. The display count is otherwise unused. Uses strict
/// native admission; degree-zero and other invalid spline layouts retain typed errors.
pub fn bspline_path(
    control: &[[Real; 2]],
    degree: usize,
    display_segments_per_span: usize,
) -> Result<CurvePath2, ValidationError> {
    bspline_path_with_context(
        control,
        degree,
        display_segments_per_span,
        GeometryContext::STRICT,
    )
    .map(GeometryOutcome::into_value)
}

/// Constructs [`bspline_path`] with explicit policy and aggregate local certainty.
///
/// Checks control-count and knot-index arithmetic before generating integer knots
/// and cloning exact controls. Overflow in these index calculations is reported as
/// `InvalidArguments`; native spline validation rejects degree zero. A positive
/// display count neither allocates samples nor controls knot spacing. Spline and
/// path admission share the decision scope and preserve native diagnostic causes.
pub fn bspline_path_with_context(
    control: &[[Real; 2]],
    degree: usize,
    display_segments_per_span: usize,
    context: GeometryContext,
) -> Result<GeometryOutcome<CurvePath2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let minimum_control_count = degree
        .checked_add(1)
        .ok_or(ValidationError::InvalidArguments)?;
    if control.len() < minimum_control_count || display_segments_per_span < 1 {
        return Err(ValidationError::InvalidArguments);
    }
    let n = control.len() - 1;
    let m = n
        .checked_add(degree)
        .and_then(|count| count.checked_add(1))
        .ok_or(ValidationError::InvalidArguments)?;
    let span_count = n - degree + 1;
    let knots = (0..=m)
        .map(|index| {
            if index <= degree {
                Real::zero()
            } else if index >= m - degree {
                Real::from(span_count as u64)
            } else {
                Real::from((index - degree) as u64)
            }
        })
        .collect();
    let points = control
        .iter()
        .map(|point| Point2::new(point[0].clone(), point[1].clone()))
        .collect();
    let spline =
        decisions.exact_curve(|| PolynomialSplineCurve2::try_new(degree, points, knots))?;
    let path = decisions.exact_curve(|| CurvePath2::try_new(vec![Curve2::from(spline)]))?;
    Ok(decisions.finish(path))
}

/// Returns the exact native line-string view of [`hilbert_paths`] under the strict policy.
///
/// Shares its order, padding, bounds and closed-set clipping semantics. This
/// geometry-only convenience omits trim contact/parameter records. Use
/// [`hilbert_paths`] when those records or selected algebraic endpoints are
/// needed. A fragment without represented `Real` endpoints returns a typed
/// unsupported error; it is never replaced by a display chord or omitted.
pub fn hilbert_strings(
    boundary: &CurveRegion2,
    order: usize,
    padding: impl IntoReal,
) -> Result<Vec<CurveString2>, ValidationError> {
    let padding = padding.into_real()?;
    hilbert_strings_with_context(boundary, order, padding, GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Constructs [`hilbert_strings`] with explicit policy and aggregate local certainty.
///
/// Consumes the retained-path receipt, copies native lines where available, and
/// rebuilds a line from exact represented endpoints for range-wrapped fragments.
/// Selected endpoints without scalar coordinates produce `UnsupportedPrimitive`.
/// Fragment and run order are preserved; each output string is checked for native
/// connectivity. Failure returns no partial collection. The receipt includes
/// trimming and conversion but does not restore discarded trim/contact records.
pub fn hilbert_strings_with_context(
    boundary: &CurveRegion2,
    order: usize,
    padding: Real,
    context: GeometryContext,
) -> Result<GeometryOutcome<Vec<CurveString2>>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let paths = decisions
        .consume_geometry(hilbert_paths_with_context(boundary, order, padding, context)?);
    let mut strings = solid::reserved(paths.len())?;
    for path in paths {
        let mut segments = solid::reserved(path.fragments().len())?;
        for fragment in path.fragments() {
            let curve = fragment.trim_fragment().curve();
            let line = if let Some(CurveGeometry2::Line(line)) = curve.geometry() {
                line.clone()
            } else {
                // Every source curve constructed by hilbert_paths is a line.
                // A native range wrapper may retain represented endpoints;
                // copy those exact values without reconstructing selected roots.
                let start = curve.start();
                let end = curve.end();
                let (Some(start), Some(end)) = (start.coordinates(), end.coordinates()) else {
                    return Err(ValidationError::UnsupportedPrimitive {
                        reason: "Hilbert trim requires retained exact endpoints; use hilbert_paths",
                    });
                };
                decisions.exact_curve(|| LineSeg2::try_new(start.clone(), end.clone()))?
            };
            segments.push(Segment2::Line(line));
        }
        strings.push(decisions.exact_curve(|| CurveString2::try_new(segments))?);
    }
    Ok(decisions.finish(strings))
}

/// Clips a finite Hilbert cell-center polyline to a region's closed filled set.
///
/// Native trim records retain maximal connected runs in traversal order,
/// source segment indices and parameters, and boundary contact evidence.
/// Positive-length boundary overlaps are included; isolated contacts add no
/// piece. Holes and disconnected components use the native exact trim kernel.
/// Undecided or unsupported operations return typed errors without partial output.
/// The convenience function uses the strict policy.
///
/// `order` must be in `1..=10`, and `padding` must be nonnegative. Both are
/// validated before empty-region handling. Padding insets the rectangle returned
/// by native bounds, not every boundary of the region; retained algebraic trims
/// may give an enclosing rectangle larger than the tight bounds. Valid empty
/// regions return an empty collection. Nonempty padded bounds must have positive
/// width and height.
///
/// For `N = 2^order`, the grid has `N*N` cell centers and `N*N - 1` source lines.
/// Each center is `(xmin + padding + width*(i+1/2)/N,
/// ymin + padding + height*(j+1/2)/N)` in the usable rectangle. The traversal
/// starts at its lower-left cell; at order one it visits lower-left, upper-left,
/// upper-right, then lower-right. The order cap limits generated points to
/// 1,048,576; it is not a total-work, coefficient-growth, heap-byte or
/// cancellation budget for native trimming.
pub fn hilbert_paths(
    boundary: &CurveRegion2,
    order: usize,
    padding: impl IntoReal,
) -> Result<Vec<CurvePathRegionTrim2>, ValidationError> {
    let padding = padding.into_real()?;
    hilbert_paths_with_context(boundary, order, padding, GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Constructs [`hilbert_paths`] with policy-explicit bounds, grid construction and trimming.
///
/// Rejects invalid order before testing padding. An empty bound returns no paths
/// after padding validation; otherwise width is checked before height and each
/// must be selected positive. The grid is constructed once as an ordered exact
/// line path, then passed to native inside-region trimming.
///
/// The receipt covers consumed bounds, construction and trim decisions; it
/// does not certify the intent of an earlier approximate region construction.
pub fn hilbert_paths_with_context(
    boundary: &CurveRegion2,
    order: usize,
    padding: Real,
    context: GeometryContext,
) -> Result<GeometryOutcome<Vec<CurvePathRegionTrim2>>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    if order == 0
        || order > 10
        || !matches!(
            decisions.sign(&padding, "Hilbert padding")?,
            hyperreal::RealSign::Positive | hyperreal::RealSign::Zero
        )
    {
        return Err(ValidationError::InvalidArguments);
    }
    let Some(bounds) =
        decisions.consume_geometry(bounding_box_with_context(boundary, context)?)
    else {
        return Ok(decisions.finish(Vec::new()));
    };
    let width =
        bounds.maxs.x.clone() - bounds.mins.x.clone() - Real::from(2_u8) * padding.clone();
    let height =
        bounds.maxs.y.clone() - bounds.mins.y.clone() - Real::from(2_u8) * padding.clone();
    if !matches!(
        decisions.sign(&width, "Hilbert width")?,
        hyperreal::RealSign::Positive
    ) || !matches!(
        decisions.sign(&height, "Hilbert height")?,
        hyperreal::RealSign::Positive
    ) {
        return Err(ValidationError::InvalidArguments);
    }
    /// Reflects and swaps a Hilbert subquadrant according to its two orientation bits.
    ///
    /// The caller supplies `rx` and `ry` in `{0, 1}` and coordinates smaller
    /// than `size`, so reflection about `size - 1` stays in the integer grid.
    const fn rotate_quadrant(size: u64, x: &mut u64, y: &mut u64, rx: u64, ry: u64) {
        if ry == 0 {
            if rx == 1 {
                *x = size - 1 - *x;
                *y = size - 1 - *y;
            }
            std::mem::swap(x, y);
        }
    }
    /// Decodes a Hilbert distance into integer cell coordinates, least-significant quadrant first.
    ///
    /// The caller supplies a power-of-two `size` and `distance < size * size`.
    /// Successive base-four digits choose reflections and quadrant offsets;
    /// the public order cap bounds all arithmetic used here.
    const fn point(size: u64, mut distance: u64) -> (u64, u64) {
        let (mut x, mut y) = (0, 0);
        let mut scale = 1;
        while scale < size {
            let rx = (distance / 2) & 1;
            let ry = (distance ^ rx) & 1;
            rotate_quadrant(scale, &mut x, &mut y, rx, ry);
            x += scale * rx;
            y += scale * ry;
            distance /= 4;
            scale *= 2;
        }
        (x, y)
    }
    let size = 1_u64 << order;
    let denominator = Real::from(size);
    let half = (Real::one() / Real::from(2_u8)).expect("two is nonzero");
    let origin_x = bounds.mins.x.clone() + padding.clone();
    let origin_y = bounds.mins.y.clone() + padding;
    let count =
        usize::try_from(size * size - 1).map_err(|_| ValidationError::CapacityOverflow)?;
    let mut curves = solid::reserved(count)?;
    let mut previous = None;
    for distance in 0..size * size {
        let (x, y) = point(size, distance);
        // Cell centers keep the grid symmetrically inset from its usable bounds.
        let u = ((Real::from(x) + half.clone()) / &denominator)?;
        let v = ((Real::from(y) + half.clone()) / &denominator)?;
        let current = Point2::new(
            origin_x.clone() + width.clone() * u,
            origin_y.clone() + height.clone() * v,
        );
        if let Some(start) = previous {
            curves.push(Curve2::from(
                decisions.exact_curve(|| LineSeg2::try_new(start, current.clone()))?,
            ));
        }
        previous = Some(current);
    }
    let path = decisions.exact_curve(|| CurvePath2::try_new(curves))?;
    let paths = decisions.exact_curve(|| path.trim_inside_region(boundary))?;
    Ok(decisions.finish(paths))
}

/// Triangulates a filled region into a triangle surface in the `z = 0` plane.
///
/// Native material/hole classification supplies each component to Hypertri.
/// Exact line members and represented native curve endpoints keep their coordinates,
/// including in mixed profiles; nonlinear interiors and endpoints lacking native
/// coordinates use the sampled modeling boundary described by
/// [`SAMPLED_PROFILE_CHORD_ERROR`]. An empty region produces an empty mesh.
///
/// The result retains local predicate certainty and accumulated mesh construction
/// qualifications. Validation checks valid, unique, nondegenerate triangle geometry;
/// it does not require a closed solid or certify fidelity to the original curves.
/// Debug builds can panic if emission cannot certify a triangle's nondegeneracy
/// under the strict policy before final validation.
pub fn try_triangulate(
    input: &CurveRegion2,
    context: GeometryContext,
) -> Result<GeometryOutcome<TriangleMesh>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let mesh = try_triangulate_with(input, &decisions)?;
    Ok(decisions.finish_mesh(mesh))
}

/// Triangulates each modeling profile while accumulating native certainty and sampling history.
///
/// Converts each exact planar triangle to `z = 0`, reuses structurally equal
/// positions through the shared emitter, and applies surface validation once to
/// the completed mesh. Material/hole grouping remains the curve kernel's decision;
/// Hypertri owns cap triangulation. Empty profile lists leave both buffers empty.
fn try_triangulate_with(
    input: &CurveRegion2,
    decisions: &GeometryDecisions,
) -> Result<TriangleMesh, ValidationError> {
    let mut positions = Vec::new();
    let mut triangles = Vec::new();
    for (material, holes) in try_modeling_profiles_with(input, decisions)? {
        let faces = decisions.consume_triangulation(hypercurve::triangulate_linear_contours(
            &decisions.triangulation_context(),
            &material,
            &holes,
        )?);
        for face in faces {
            let points = face.map(|p| Point3::new(p.x().clone(), p.y().clone(), Real::zero()));
            if !emit_certified_triangle(&mut positions, &mut triangles, points, false) {
                return Err(ValidationError::InvalidArguments);
            }
        }
    }
    validate_surface_mesh(TriangleMesh::new(positions, triangles), decisions)
}

/// Chord-error request used to sample nonlinear interiors for modeling.
///
/// [`try_triangulate`], [`try_extrude`], [`try_extrude_vector`], [`try_sweep`],
/// [`extrude_twisted`] and [`revolve`] consume an exact line profile's own
/// coordinates, even beside curved components or holes. Curved boundaries use
/// [`hypercurve::CurveRegionProfile2::sample_to_linear_contours`], retaining
/// native endpoints and line spans while sampling nonlinear interiors. The
/// mesh result records sampling and coordinate projection in its construction
/// evidence. Endpoints without native scalar coordinates may also cross the
/// finite boundary. This value is a request in model-coordinate units, not a
/// certified topology or source-fidelity error bound.
/// [`try_finite_profiles`] remains the explicit all-coordinate `f64` export.
pub const SAMPLED_PROFILE_CHORD_ERROR: f64 = 1.0e-3;

/// Chooses exact or sampled line contours independently for each classified material profile.
///
/// First requests native linear contours, including their owned holes. A successful
/// view is returned unchanged; a typed failure propagates. Only absence of that
/// view selects sampling, recording both sampling and coordinate-projection
/// qualification before calling the native mixed-boundary adapter.
///
/// The sampled adapter still preserves exact line members and represented native
/// endpoints, even when other components or holes are curved. Other coordinates
/// come from finite projection and are promoted back to exact binary values.
/// Material/hole ownership is retained, but sampling is a source-fidelity boundary.
/// The caller attaches accumulated qualifications when publishing a mesh.
fn try_modeling_profiles_with(
    input: &CurveRegion2,
    decisions: &GeometryDecisions,
) -> Result<Vec<(Contour2, Vec<Contour2>)>, ValidationError> {
    let options = FiniteProjectionOptions::try_new(SAMPLED_PROFILE_CHORD_ERROR)
        .expect("positive finite projection tolerance");
    decisions
        .exact_curve(|| input.boundary_profiles())?
        .iter()
        .map(|profile| {
            if let Some(exact) = decisions.exact_curve(|| profile.linear_contours())? {
                return Ok(exact);
            }
            decisions.record_construction(
                hypermesh::ConstructionEvidence::UNTRACKED
                    .with_sampling()
                    .with_coordinate_projection(),
            );
            Ok(decisions.exact_curve(|| profile.sample_to_linear_contours(&options))?)
        })
        .collect()
}

/// Exports finite polygon-with-holes profiles for rendering and interchange.
///
/// Every coordinate, including exact line vertices, crosses the `f64` boundary;
/// curves are sampled with the request [`SAMPLED_PROFILE_CHORD_ERROR`]. Native
/// regularized topology supplies material/hole ownership instead of recovering it
/// from finite containment. Empty geometry returns no profiles; unsupported or
/// undecided native work and nonfinite projection failures remain errors.
///
/// The outcome carries local predicate certainty. The finite profiles carry their
/// rings and requested chord error, not a mesh construction-history receipt or a
/// certificate of exact source fidelity.
pub fn try_finite_profiles(
    input: &CurveRegion2,
    context: GeometryContext,
) -> Result<GeometryOutcome<Vec<FiniteRegionProfile2>>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let profiles = try_finite_profiles_with(input, &decisions)?;
    Ok(decisions.finish(profiles))
}

/// Projects every boundary coordinate using native exact profile ownership and shared decisions.
///
/// Records sampling/projection in the construction accumulator and calls the
/// finite adapter at [`SAMPLED_PROFILE_CHORD_ERROR`]. The public finite-profile
/// wrapper publishes a generic outcome, so it does not attach that accumulator
/// as mesh history. The returned primitive coordinates remain an export product.
fn try_finite_profiles_with(
    input: &CurveRegion2,
    decisions: &GeometryDecisions,
) -> Result<Vec<FiniteRegionProfile2>, ValidationError> {
    decisions.record_construction(
        hypermesh::ConstructionEvidence::UNTRACKED
            .with_sampling()
            .with_coordinate_projection(),
    );
    let options = FiniteProjectionOptions::try_new(SAMPLED_PROFILE_CHORD_ERROR)
        .expect("positive finite projection tolerance");
    let profiles =
        decisions.exact_curve(|| input.project_to_finite_profiles_exact(&options))?;
    Ok(profiles)
}

/// Extrudes a filled XY region from `z = 0` to the signed `height`.
///
/// Height must have a nonzero selected sign, even for an empty region; negative
/// height is supported with outward cap and wall orientation reversed as needed.
/// A valid nonzero extrusion of an empty region returns an empty mesh. Delegates
/// to [`try_extrude_vector`] with zero X and Y displacement.
///
/// Exact line members and represented native curve endpoints keep their coordinates
/// in mixed profiles. Other boundary portions use the sampled modeling contract
/// of [`SAMPLED_PROFILE_CHORD_ERROR`]. The mesh retains construction qualifications
/// and local certainty; these do not certify source-curve fidelity.
pub fn try_extrude(
    region: &CurveRegion2,
    height: Real,
    context: GeometryContext,
) -> Result<GeometryOutcome<TriangleMesh>, ValidationError> {
    try_extrude_vector(
        region,
        Vector3::from_xyz(Real::zero(), Real::zero(), height),
        context,
    )
}

/// Extrudes a filled XY region between its original plane and a translated copy.
///
/// `direction` is the complete displacement, without normalization. Its Z component
/// must have a nonzero selected sign before profile admission, including for empty
/// input; X and Y may produce an oblique extrusion. Each cap uses the native
/// material/hole triangulation and each side quadrilateral becomes two triangles.
///
/// The output is checked for unique nondegenerate faces and paired geometric edges,
/// with canonical empty output allowed for an empty source. These checks do not
/// establish vertex-link manifoldness or absence of geometric self-intersections.
/// Fidelity and construction qualifications follow [`try_extrude`].
/// Debug builds can panic if emission cannot certify a triangle's nondegeneracy
/// under the strict policy before final validation.
pub fn try_extrude_vector(
    region: &CurveRegion2,
    direction: Vector3,
    context: GeometryContext,
) -> Result<GeometryOutcome<TriangleMesh>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let mesh = try_extrude_vector_with(region, direction, &decisions)?;
    Ok(decisions.finish_mesh(mesh))
}

/// Builds oriented caps and side walls after deciding the extrusion's Z direction.
///
/// A selected zero Z component is `ZeroParameter`; unresolved classification
/// propagates. Negative Z reverses cap and wall orientation. Each ring's material
/// side is selected from its role and signed polygon area; holes face oppositely
/// to the outer material wall. After structural vertex reuse, closed-mesh validation
/// allows empty output only when the source region itself is empty.
fn try_extrude_vector_with(
    region: &CurveRegion2,
    direction: Vector3,
    decisions: &GeometryDecisions,
) -> Result<TriangleMesh, ValidationError> {
    let direction_sign = decisions.decide(
        hyperlimit::classify_real_sign(&direction.0[2], decisions.predicate_policy()),
        "extrusion direction Z sign",
    )?;
    if direction_sign == hyperlimit::Sign::Zero {
        return Err(ValidationError::ZeroParameter {
            name: "extrusion direction Z component",
        });
    }
    let flip = direction_sign == hyperlimit::Sign::Negative;
    let mut positions = Vec::new();
    let mut triangles = Vec::new();
    let point = |p: &Point2| Point3::new(p.x().clone(), p.y().clone(), Real::zero());
    let place = |p: &Point2, section: usize| {
        Ok(if section == 0 {
            point(p)
        } else {
            point(p) + &direction
        })
    };
    for (material, holes) in try_modeling_profiles_with(region, decisions)? {
        emit_section_caps(
            &mut positions,
            &mut triangles,
            (&material, &holes),
            &[(0, !flip), (1, flip)],
            decisions,
            &place,
        )?;
        for (ring, reverse) in oriented_profile_rings(&material, &holes, decisions)? {
            for [start, end] in oriented_ring_edges(ring, reverse) {
                let bottom_a = point(start);
                let bottom_b = point(end);
                let top_a = bottom_a.clone() + &direction;
                let top_b = bottom_b.clone() + &direction;
                if flip {
                    let emitted = emit_certified_triangle(
                        &mut positions,
                        &mut triangles,
                        [bottom_b.clone(), bottom_a, top_a.clone()],
                        false,
                    ) && emit_certified_triangle(
                        &mut positions,
                        &mut triangles,
                        [bottom_b, top_a, top_b],
                        false,
                    );
                    if !emitted {
                        return Err(ValidationError::InvalidArguments);
                    }
                } else {
                    let emitted = emit_certified_triangle(
                        &mut positions,
                        &mut triangles,
                        [bottom_a.clone(), bottom_b, top_b.clone()],
                        false,
                    ) && emit_certified_triangle(
                        &mut positions,
                        &mut triangles,
                        [bottom_a, top_b, top_a],
                        false,
                    );
                    if !emitted {
                        return Err(ValidationError::InvalidArguments);
                    }
                }
            }
        }
    }
    validate_closed_mesh(
        TriangleMesh::new(positions, triangles),
        region.is_empty(),
        decisions,
    )
}

/// Checks unique nondegenerate face geometry and attaches accumulated construction evidence.
///
/// The native query checks indices, coordinate equality under the selected policy,
/// triangle degeneracy and duplicate unoriented triangle keys. A decided false
/// becomes a `Geometry` error; native undecided or other failures retain their
/// mesh cause. An empty triangle list can pass. This is not a closure, intersection
/// or vertex-link test.
fn validate_surface_mesh(
    mesh: TriangleMesh,
    decisions: &GeometryDecisions,
) -> Result<TriangleMesh, ValidationError> {
    let valid = decisions.consume_mesh(
        mesh.has_unique_nondegenerate_triangles(decisions.mesh_context())
            .map_err(ValidationError::from)?,
    );
    if valid {
        Ok(decisions.qualify_mesh(mesh))
    } else {
        Err(ValidationError::Geometry(
            "triangulation emitted duplicate, invalid, or degenerate triangles".into(),
        ))
    }
}

/// Checks geometric edge pairing and unique nondegenerate faces before qualifying a mesh.
///
/// When `allow_empty` is true, an empty triangle list bypasses validation; this
/// shortcut does not inspect positions. Otherwise the native geometric closure
/// query requires nonempty faces and one use of each edge in each direction after
/// coordinate canonicalization. A decided false becomes a `Geometry` error;
/// native failures retain their mesh causes. Neither branch establishes an
/// embedded surface or checks vertex links.
fn validate_closed_mesh(
    mesh: TriangleMesh,
    allow_empty: bool,
    decisions: &GeometryDecisions,
) -> Result<TriangleMesh, ValidationError> {
    if allow_empty && mesh.triangles.is_empty() {
        return Ok(decisions.qualify_mesh(mesh));
    }
    let closed = decisions.consume_mesh(
        mesh.is_closed_manifold_geometry(decisions.mesh_context())
            .map_err(ValidationError::from)?,
    );
    if closed {
        Ok(decisions.qualify_mesh(mesh))
    } else {
        Err(ValidationError::Geometry(
            "operation did not produce a unique nondegenerate closed manifold".into(),
        ))
    }
}

/// Appends a triangle using existing structurally equal position rows where possible.
///
/// For each point, searches existing positions from the end using `Point3`
/// equality and appends only when no equal row exists. This does not perform a
/// policy-directed coordinate-equivalence search or tolerance welding. `reverse`
/// reverses the three emitted indices.
///
/// Nondegeneracy is the caller's construction obligation or is checked by later
/// whole-mesh validation. With debug assertions enabled, an unproved strict
/// nondegeneracy query panics. Otherwise repeated resolved indices suppress the
/// face. Every normal return is `true`, including that skipped-face case; the
/// Boolean result does not prove that a triangle was emitted.
fn emit_certified_triangle(
    positions: &mut Vec<Point3>,
    triangles: &mut Vec<hypermesh::Triangle>,
    points: [Point3; 3],
    reverse: bool,
) -> bool {
    debug_assert!(crate::hyper_math::htriangle_area2_is_nonzero(
        &points[0], &points[1], &points[2]
    ));
    let mut indices = [0; 3];
    for (slot, point) in indices.iter_mut().zip(points) {
        *slot = positions
            .iter()
            .rposition(|candidate| candidate == &point)
            .unwrap_or_else(|| {
                let index = positions.len();
                positions.push(point);
                index
            });
    }
    if indices[0] == indices[1] || indices[1] == indices[2] || indices[2] == indices[0] {
        return true;
    }
    triangles.push(if reverse {
        hypermesh::Triangle::new(indices[2], indices[1], indices[0])
    } else {
        hypermesh::Triangle::new(indices[0], indices[1], indices[2])
    });
    true
}

/// Fallible placement of an exact profile point at a caller-selected section index.
///
/// Sweep, twist and extrusion callers own the frame, scale, rotation and index
/// interpretation. Cap/wall emitters consume the resulting exact coordinates;
/// the callback itself does not certify nondegeneracy or final mesh topology.
type SectionPlacement<'a> = dyn Fn(&Point2, usize) -> Result<Point3, ValidationError> + 'a;

/// Triangulates one material profile with holes and places each face into the requested caps.
///
/// The cap list contains `(section, reverse)` pairs. Processing is face-major,
/// then cap-list order; each placement runs in vertex order and may fail. Native
/// triangulation certainty is consumed before emitting faces. This appends to the
/// caller's buffers and can leave partial local state on error; the caller owns
/// final validation and publication.
fn emit_section_caps(
    positions: &mut Vec<Point3>,
    triangles: &mut Vec<hypermesh::Triangle>,
    (material, holes): (&Contour2, &[Contour2]),
    caps: &[(usize, bool)],
    decisions: &GeometryDecisions,
    place: &SectionPlacement<'_>,
) -> Result<(), ValidationError> {
    let cap_faces = decisions.consume_triangulation(hypercurve::triangulate_linear_contours(
        &decisions.triangulation_context(),
        material,
        holes,
    )?);
    for [a, b, c] in &cap_faces {
        for &(section, reverse) in caps {
            let points = [place(a, section)?, place(b, section)?, place(c, section)?];
            if !emit_certified_triangle(positions, triangles, points, reverse) {
                return Err(ValidationError::InvalidArguments);
            }
        }
    }
    Ok(())
}

/// Selects whether the final side-wall band stops, wraps or collapses to a caller-supplied apex.
#[derive(Clone, Copy, PartialEq)]
enum SectionWallEnd {
    /// The last section is not joined to the first.
    Open,
    /// The last section is also joined to the first.
    Closed,
    /// The caller collapses the last section to one point; emit one triangle per final edge band.
    Apex,
}

/// Connects each oriented profile edge across consecutive placed sections with triangles.
///
/// The quadrilateral of edge `(start, end)` between sections `i` and `i + 1`
/// splits along the diagonal from `start` at `i` to `end` at `i + 1`. `Closed`
/// wraps the last section to the first. `Apex` omits the second triangle of the
/// last band; the caller must make that final section a common point. All four
/// corner placements still run, including the unused fourth apex corner.
///
/// Processing is ring-major, edge-major, then section order. `reverse` flips every
/// triangle. Zero sections emits no walls; open/apex forms with one section emit
/// none. Placement errors may leave partial local buffers; this helper does not
/// validate closure, certify an apex or publish the completed mesh.
fn emit_section_walls(
    positions: &mut Vec<Point3>,
    triangles: &mut Vec<hypermesh::Triangle>,
    rings: &[(&Contour2, bool)],
    sections: usize,
    end: SectionWallEnd,
    reverse: bool,
    place: &SectionPlacement<'_>,
) -> Result<(), ValidationError> {
    let quads = if end == SectionWallEnd::Closed {
        sections
    } else {
        sections.saturating_sub(1)
    };
    for &(ring, reverse_ring) in rings {
        for [start, finish] in oriented_ring_edges(ring, reverse_ring) {
            for section in 0..quads {
                let next = (section + 1) % sections;
                let [a, b, c, d] = [
                    place(start, section)?,
                    place(finish, section)?,
                    place(finish, next)?,
                    place(start, next)?,
                ];
                let tapered = end == SectionWallEnd::Apex && section + 1 == quads;
                if !emit_certified_triangle(
                    positions,
                    triangles,
                    [a.clone(), b, c.clone()],
                    reverse,
                ) || (!tapered
                    && !emit_certified_triangle(positions, triangles, [a, c, d], reverse))
                {
                    return Err(ValidationError::InvalidArguments);
                }
            }
        }
    }
    Ok(())
}

/// Segmented swept boundary of a native filled region about the Y axis.
///
/// The X coordinate of the profile is its radius. Each profile point `(x, y)`
/// is placed at `(x cos t, y, x sin t)` for `segments + 1` equally spaced
/// angles `t` from zero to `angle_degrees`, or `segments` angles for a full
/// turn, which has no caps. An exact line profile keeps its own coordinates,
/// so the start cap is the authored profile; other profiles are sampled at
/// [`SAMPLED_PROFILE_CHORD_ERROR`]. The result is the polyhedron through these
/// slices, not the analytic solid of revolution.
///
/// A vertex on the axis is one position shared by every slice. A full turn
/// preserves each signed radius's authored meridian. Material on both sides
/// of the axis is split there and its swept polyhedra are united exactly,
/// so overlapping pieces have winding one. With an even segment count a half
/// turn permutes the angular lattice; the equivalent planar radial fold is
/// used instead. With an odd count the negative-radius lattice has a distinct
/// phase, which is retained even when unrelated positive material is added.
///
/// For a partial turn each profile loop must lie on one side of the axis,
/// which it may touch. Loops on opposite sides are revolved separately
/// through the opposite angular ranges; beyond a half turn those ranges
/// overlap, and where the loops' mirror images also overlap the result is a
/// closed mesh with winding number two there, not an embedded solid. A loop
/// crossing the axis in a partial turn or a zero angle returns
/// [`ValidationError::InvalidArguments`]. Exact integer multiples of a full
/// turn cover the same sampled set once, with `segments` slices and the
/// authored turn direction. Oversized noninteger sweeps (such as 450 degrees)
/// remain invalid: neither saturation nor modulo reduction is implicit. If
/// integrality cannot be proved, the result is a typed undecided predicate.
/// Use [`revolve_from`] to choose the authored starting angle explicitly, or
/// [`revolve_solid`] for regularized partial material including axis crossings.
///
/// Requires at least two segments for a partial turn and three for full
/// coverage. Angle and angular-table capacity checks precede empty-profile
/// handling; an empty region with valid parameters returns canonical empty
/// geometry. For oversized angles, integrality is recognized through
/// [`Real::exact_rational`]; failure to expose a rational turn count is
/// undecided, not proof of a noninteger value. Partial sample-count overflow
/// returns `InvalidArguments`; an unaddressable angular table returns
/// [`ValidationError::CapacityOverflow`]. This preflight does not bound total
/// work or allocations during construction.
///
/// # Panics
///
/// In debug builds, partial-turn cap emission asserts strict nondegeneracy of
/// each placed triangle. A degenerate or strictly undecided cap triangle can
/// panic at that internal assertion; other validation failures are returned.
pub fn revolve(
    region: &CurveRegion2,
    angle_degrees: Real,
    segments: usize,
    context: GeometryContext,
) -> Result<GeometryOutcome<TriangleMesh>, ValidationError> {
    revolve_with_output(
        region,
        angle_degrees,
        segments,
        context,
        RevolutionOutput::SweptBoundary,
    )
}

/// Regularized material of a segmented revolution about the Y axis.
///
/// Uses the same authored angular lattice, profile conversion, angle domain and
/// direction as [`revolve`]. Signed material is split at the axis without folding
/// partial sweeps; its one-sided swept polyhedra are united by the exact mesh
/// arrangement. Axis-crossing profiles are admitted, and overlapping pieces
/// contribute material once. The result is the regularized union of the sampled
/// polyhedra, not an analytic surface of revolution. Nonlinear profile sampling
/// retains the same construction qualifications as [`revolve`].
///
/// Regularization can retain point/edge contacts with a nonmanifold boundary;
/// the output satisfies the native closed PWN contract rather than requiring a
/// manifold boundary. Legacy [`revolve`] retains its swept-boundary semantics.
/// One-sided inputs use the same positions and triangles as `revolve`, with
/// PWN validation; they do not incur a separate union. That validation checks
/// nondegenerate faces and balanced geometric edge multiplicities, not a
/// global proof that winding is everywhere zero or one. Mixed-side material
/// is regularized by the arrangement (or the equivalent even full-turn fold).
/// This operation adds no total-work, retained-byte or cancellation bound.
///
/// The segment minimum, validation-before-empty behavior and debug cap
/// assertion described by [`revolve`] also apply here.
pub fn revolve_solid(
    region: &CurveRegion2,
    angle_degrees: Real,
    segments: usize,
    context: GeometryContext,
) -> Result<GeometryOutcome<TriangleMesh>, ValidationError> {
    revolve_with_output(
        region,
        angle_degrees,
        segments,
        context,
        RevolutionOutput::RegularizedSolid,
    )
}

/// Selects signed-radius composition and the final native validation contract.
///
/// Full turns regularize mixed signs in both modes; partial turns differ in
/// whether axis crossings and overlapping opposite-side pieces are resolved.
#[derive(Clone, Copy, PartialEq, Eq)]
enum RevolutionOutput {
    /// Preserve partial swept shells and require unique faces with paired edges.
    SweptBoundary,
    /// Union signed material and admit balanced nonmanifold edge multiplicities.
    RegularizedSolid,
}

/// Validates the angle, constructs its lattice, and dispatches signed profiles.
///
/// Checks the base segment minimum before angle sign/magnitude, then the
/// full-turn minimum and angular-table capacity before inspecting the region.
/// Oversized rational integer turn counts normalize to one signed turn;
/// unrecognized rationality is a typed undecided result, not resource exhaustion.
///
/// Mixed signs use a planar fold only for even full-turn lattices. Otherwise,
/// full turns and regularized partial turns split at the axis and union two
/// meshes through the native arrangement. Legacy partial turns go directly to
/// the one-sided emitter, which rejects crossing loops. The shared decisions
/// collect profile conversion, predicate and nested operation qualifications
/// before publishing the mesh. Capacity preflight covers only addressability
/// of the angular table, not a construction-wide resource budget.
fn revolve_with_output(
    region: &CurveRegion2,
    angle_degrees: Real,
    segments: usize,
    context: GeometryContext,
    output: RevolutionOutput,
) -> Result<GeometryOutcome<TriangleMesh>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    if segments < 2 {
        return Err(ValidationError::FieldLessThan {
            name: "segments",
            min: 2,
        });
    }
    let sign = decisions.decide(
        hyperlimit::classify_real_sign(&angle_degrees, decisions.predicate_policy()),
        "revolution angle sign",
    )?;
    let magnitude_order = decisions.decide(
        hyperlimit::compare_reals(
            &angle_degrees.clone().abs(),
            &Real::from(360_u16),
            decisions.predicate_policy(),
        ),
        "revolution angle magnitude",
    )?;
    let full = match (magnitude_order, sign) {
        (Ordering::Less, hyperlimit::Sign::Positive | hyperlimit::Sign::Negative) => false,
        (Ordering::Equal, hyperlimit::Sign::Positive | hyperlimit::Sign::Negative) => true,
        (Ordering::Greater, hyperlimit::Sign::Positive | hyperlimit::Sign::Negative) => {
            let turns = (angle_degrees.clone() / Real::from(360_u16))?;
            match turns.exact_rational() {
                Some(turns) if turns.is_integer() => true,
                Some(_) => return Err(ValidationError::InvalidArguments),
                None => {
                    return Err(ValidationError::PredicateUndecided {
                        predicate: "revolution full-turn count",
                        policy: decisions.predicate_policy(),
                    });
                },
            }
        },
        _ => return Err(ValidationError::InvalidArguments),
    };
    if full && segments < 3 {
        return Err(ValidationError::FieldLessThan {
            name: "segments",
            min: 3,
        });
    }
    let sample_count = if full {
        segments
    } else {
        segments
            .checked_add(1)
            .ok_or(ValidationError::InvalidArguments)?
    };
    // The required angular table must fit the platform's addressable Vec size.
    // This is a capacity preflight, not a total-work or retained-heap budget.
    solid::capacity::<(Real, Real)>(Some(sample_count))?;
    let angle_degrees = if magnitude_order == Ordering::Greater {
        Real::from(if sign == hyperlimit::Sign::Positive {
            360
        } else {
            -360
        })
    } else {
        angle_degrees
    };
    let samples = (0..sample_count)
        .map(|index| {
            let fraction = (Real::from(index as u64) / Real::from(segments as u64)).ok()?;
            let radians =
                (angle_degrees.clone() * Real::pi() * fraction / Real::from(180_u16)).ok()?;
            Some((radians.clone().sin(), radians.cos()))
        })
        .collect::<Option<Vec<_>>>()
        .ok_or(ValidationError::InvalidArguments)?;
    let mut profiles = try_modeling_profiles_with(region, &decisions)?;
    let mut radii = profile_radial_signs(&profiles, &decisions)?;
    let has_radial_sign = |radii: &[Vec<Vec<hyperreal::RealSign>>], sign| {
        radii.iter().flatten().flatten().any(|radial| *radial == sign)
    };
    if (full || output == RevolutionOutput::RegularizedSolid)
        && has_radial_sign(&radii, hyperreal::RealSign::Negative)
        && has_radial_sign(&radii, hyperreal::RealSign::Positive)
    {
        if !(full && segments.is_multiple_of(2)) {
            let halves = split_at_revolution_axis(profiles, &decisions)?;
            let mut meshes = Vec::with_capacity(2);
            for half in &halves {
                let profiles = try_modeling_profiles_with(half, &decisions)?;
                let radii = profile_radial_signs(&profiles, &decisions)?;
                meshes.push(revolve_profiles(
                    &profiles, &radii, &samples, full, sign, output, &decisions,
                )?);
            }
            let united = decisions.consume_geometry(solid::boolean_with_context(
                &meshes[0],
                &meshes[1],
                hypermesh::BooleanOp::Union,
                context,
            )?);
            let mesh =
                validate_revolution_mesh(united, region.is_empty(), output, &decisions)?;
            return Ok(decisions.finish_mesh(mesh));
        }
        profiles = fold_across_revolution_axis(profiles, &decisions)?;
        radii = profile_radial_signs(&profiles, &decisions)?;
    }
    let mesh = revolve_profiles(&profiles, &radii, &samples, full, sign, output, &decisions)?;
    Ok(decisions.finish_mesh(mesh))
}

/// Segmented revolution with an explicit authored meridian in degrees.
///
/// The placement is `(x cos(start + t), y, x sin(start + t))`; full-turn
/// coverage is canonicalized by [`revolve`] without discarding `start_degrees`.
/// Rotating the completed mesh transports its exact geometry and construction
/// qualifications, and combines both operations' certainty receipts. This API
/// returns a segmented mesh, not a parameterized multi-turn surface.
/// It inherits swept-boundary semantics, parameter validation and debug cap
/// assertions from [`revolve`], including possible partial-sweep overlap.
/// A structurally zero start skips the transform; other starts use the native
/// transform path after revolution succeeds.
pub fn revolve_from(
    region: &CurveRegion2,
    start_degrees: Real,
    angle_degrees: Real,
    segments: usize,
    context: GeometryContext,
) -> Result<GeometryOutcome<TriangleMesh>, ValidationError> {
    let swept = revolve(region, angle_degrees, segments, context)?;
    if start_degrees == Real::zero() {
        return Ok(swept);
    }
    let radians = (-start_degrees * Real::pi() / Real::from(180_u16))?;
    swept.and_then(|mesh| {
        solid::transform_with_context(&mesh, &Matrix4::rotation_y(radians), context)
    })
}

/// Emits already classified, one-sided linear profiles on the authored lattice.
///
/// The caller supplies matching profile/ring/vertex sign arrays, `(sin, cos)`
/// samples in traversal order, a nonzero turn sign, and the validated minimum
/// sample count. These lengths are not revalidated: outer zips can truncate and
/// vertex/sample indexing can panic on mismatches. Material precedes its holes,
/// which must lie on the same radial side. A ring containing both radial signs
/// is rejected even for a full turn; splitting/folding belongs to the caller.
///
/// Each input axis vertex owns one position shared across its angular row.
/// Walls use the fixed first-to-third quad diagonal and omit the triangle
/// collapsed by an axis endpoint. Loop orientation, radial side and sweep sign
/// determine outward winding. Partial turns add triangulated start/end caps;
/// full turns wrap the lattice without caps. Final validation and accumulated
/// construction qualifications follow [`validate_revolution_mesh`].
fn revolve_profiles(
    profiles: &[(Contour2, Vec<Contour2>)],
    radii: &[Vec<Vec<hyperreal::RealSign>>],
    samples: &[(Real, Real)],
    full: bool,
    sign: hyperlimit::Sign,
    output: RevolutionOutput,
    decisions: &GeometryDecisions,
) -> Result<TriangleMesh, ValidationError> {
    let segments = if full {
        samples.len()
    } else {
        samples.len() - 1
    };
    let map = |point: &Point2, sample: &(Real, Real)| {
        Point3::new(
            point.x().clone() * sample.1.clone(),
            point.y().clone(),
            point.x().clone() * sample.0.clone(),
        )
    };
    let mut positions = Vec::new();
    let mut triangles = Vec::new();
    for ((material, holes), ring_radii) in profiles.iter().zip(radii) {
        // The material loop comes first; its holes lie on the same side.
        let mut material_radial_positive = true;
        for (index, ((ring, reverse_ring), radial)) in
            oriented_profile_rings(material, holes, decisions)?
                .into_iter()
                .zip(ring_radii)
                .enumerate()
        {
            let vertices = ring
                .segments()
                .iter()
                .map(Segment2::start)
                .collect::<Vec<_>>();
            let on_axis = |index: usize| radial[index] == hyperreal::RealSign::Zero;
            let negative = radial.contains(&hyperreal::RealSign::Negative);
            let positive = radial.contains(&hyperreal::RealSign::Positive);
            if negative && positive {
                return Err(ValidationError::InvalidArguments);
            }
            if index == 0 {
                material_radial_positive = positive;
            }
            // With material on the left of each edge, a positive turn of a
            // positive-radius edge sweeps its side wall outward.
            let reverse = (sign == hyperlimit::Sign::Positive) != positive;
            let rows = vertices
                .iter()
                .enumerate()
                .map(|(index, vertex)| {
                    if on_axis(index) {
                        let index = positions.len();
                        positions.push(Point3::new(
                            Real::zero(),
                            vertex.y().clone(),
                            Real::zero(),
                        ));
                        vec![index; samples.len()]
                    } else {
                        samples
                            .iter()
                            .map(|sample| {
                                positions.push(map(vertex, sample));
                                positions.len() - 1
                            })
                            .collect()
                    }
                })
                .collect::<Vec<_>>();
            for index in 0..vertices.len() {
                let mut edge = [index, (index + 1) % vertices.len()];
                if reverse_ring {
                    edge.reverse();
                }
                let [start, end] = edge;
                for slice in 0..segments {
                    let next = if full {
                        (slice + 1) % samples.len()
                    } else {
                        slice + 1
                    };
                    let a = rows[start][slice];
                    let b = rows[end][slice];
                    let c = rows[end][next];
                    let d = rows[start][next];
                    // Rotation maps an axis endpoint to the same point in both
                    // adjacent slices. OpenSCAD-style rotate_extrude omits that
                    // collapsed fan triangle while retaining its nondegenerate
                    // neighbor.
                    for (axis_endpoint, indices) in
                        [(on_axis(end), [a, b, c]), (on_axis(start), [a, c, d])]
                    {
                        if axis_endpoint
                            || indices[0] == indices[1]
                            || indices[1] == indices[2]
                            || indices[2] == indices[0]
                        {
                            continue;
                        }
                        triangles.push(if reverse {
                            hypermesh::Triangle::new(indices[2], indices[1], indices[0])
                        } else {
                            hypermesh::Triangle::new(indices[0], indices[1], indices[2])
                        });
                    }
                }
            }
        }
        if !full {
            // Caps are counterclockwise in the profile plane. The start cap
            // faces away from the swept material, whose side follows the turn
            // direction and the profile's radial side.
            let reverse_start =
                (sign == hyperlimit::Sign::Positive) == material_radial_positive;
            emit_section_caps(
                &mut positions,
                &mut triangles,
                (material, holes),
                &[(0, reverse_start), (samples.len() - 1, !reverse_start)],
                decisions,
                &|point, sample| Ok(map(point, &samples[sample])),
            )?;
        }
    }
    validate_revolution_mesh(
        TriangleMesh::new(positions, triangles),
        profiles.is_empty(),
        output,
        decisions,
    )
}

/// Applies the selected output contract and attaches accumulated qualifications.
///
/// Swept boundaries delegate to [`validate_closed_mesh`], including its
/// triangle-only allowed-empty shortcut. Regularized output accepts empty
/// geometry only when allowed and both buffers are empty, rejects other
/// triangle-free meshes, and otherwise consumes native closed-PWN validation.
/// That native check admits balanced edge multiplicities without demanding
/// manifoldness or proving nonzero enclosed winding; it does not regularize
/// geometry. Native errors retain their sources through the modeling boundary.
fn validate_revolution_mesh(
    mesh: TriangleMesh,
    allow_empty: bool,
    output: RevolutionOutput,
    decisions: &GeometryDecisions,
) -> Result<TriangleMesh, ValidationError> {
    if output == RevolutionOutput::SweptBoundary {
        return validate_closed_mesh(mesh, allow_empty, decisions);
    }
    if allow_empty && mesh.positions.is_empty() && mesh.triangles.is_empty() {
        return Ok(decisions.qualify_mesh(mesh));
    }
    if mesh.triangles.is_empty() {
        return Err(ValidationError::Geometry(
            "nonempty revolution produced no material".into(),
        ));
    }
    decisions.consume_mesh(mesh.validate_closed_pwn(decisions.mesh_context())?);
    Ok(decisions.qualify_mesh(mesh))
}

/// Radial sign of every ring vertex, by profile and by ring in the order of
/// [`oriented_profile_rings`]: material first, then holes.
///
/// Reads each segment's start X coordinate without reversing or changing the
/// profiles. The consumer relies on linear segments, for which endpoint signs
/// suffice to identify a crossing; this is not an interior-extrema test for
/// general curves. All decisions use the supplied context and retain uncertainty.
fn profile_radial_signs(
    profiles: &[(Contour2, Vec<Contour2>)],
    decisions: &GeometryDecisions,
) -> Result<Vec<Vec<Vec<hyperreal::RealSign>>>, ValidationError> {
    profiles
        .iter()
        .map(|(material, holes)| {
            std::iter::once(material)
                .chain(holes)
                .map(|ring| {
                    ring.segments()
                        .iter()
                        .map(|segment| {
                            decisions
                                .sign(segment.start().x(), "revolution profile radius sign")
                        })
                        .collect()
                })
                .collect()
        })
        .collect()
}

/// The material a full turn sweeps: the part of the line `profiles` at
/// nonnegative radius united with the mirror image of their part at
/// nonpositive radius.
///
/// This preserves the sampled solid only for an even angular lattice, where
/// changing radial sign permutes its vertices by half a turn.
/// The caller establishes that precondition; this helper has no lattice input.
/// Native region intersection, reflection and union own the fill/hole semantics,
/// and the resulting region is converted back to modeling profiles with the
/// shared decision and construction history.
fn fold_across_revolution_axis(
    profiles: Vec<(Contour2, Vec<Contour2>)>,
    decisions: &GeometryDecisions,
) -> Result<Vec<(Contour2, Vec<Contour2>)>, ValidationError> {
    let [positive, negative] = split_at_revolution_axis(profiles, decisions)?;
    let folded = decisions.exact_curve(|| -> Result<CurveRegion2, ValidationError> {
        let mirrored = negative.transform_affine(
            &-Real::one(),
            &Real::zero(),
            &Real::zero(),
            &Real::one(),
            &Real::zero(),
            &Real::zero(),
        )?;
        Ok(positive.boolean_region(&mirrored, BooleanOp::Union)?)
    })?;
    try_modeling_profiles_with(&folded, decisions)
}

/// Cuts sampled or exact linear profiles at the axis without changing their
/// signed radial coordinates. Every new point is a rational expression in the
/// input line coordinates, rather than a newly sampled point on a curve.
///
/// Returns the nonnegative-X half first, then the nonpositive-X half, using
/// native regularized region intersections. The caller supplies a profile set
/// spanning both radial signs, so the finite clipping rectangles bracket its
/// bounds. One unit of padding makes their other edges exterior; it is not an
/// offset of the input. A region without bounds returns two empty regions.
/// Axis-only remnants follow the native filled-region regularization contract.
fn split_at_revolution_axis(
    profiles: Vec<(Contour2, Vec<Contour2>)>,
    decisions: &GeometryDecisions,
) -> Result<[CurveRegion2; 2], ValidationError> {
    let (materials, holes): (Vec<_>, Vec<_>) = profiles.into_iter().unzip();
    let region = decisions.exact_curve(|| {
        CurveRegion2::try_from_native_contours(
            materials,
            holes.into_iter().flatten().collect(),
        )
    })?;
    let Some(bounds) = decisions.exact_curve(|| region.bounds())? else {
        return Ok([empty(), empty()]);
    };
    let low = bounds.min_y().clone() - Real::one();
    let high = bounds.max_y().clone() + Real::one();
    let half = |from: Real, to: Real| -> Result<CurveRegion2, ValidationError> {
        let ring = Contour2::from_real_ring(&[
            [from.clone(), low.clone()],
            [to.clone(), low.clone()],
            [to, high.clone()],
            [from, high.clone()],
        ])?;
        Ok(CurveRegion2::try_from_native_material_contours(vec![ring])?)
    };
    decisions.exact_curve(|| -> Result<[CurveRegion2; 2], ValidationError> {
        let positive = half(Real::zero(), bounds.max_x().clone() + Real::one())?;
        let negative = half(bounds.min_x().clone() - Real::one(), Real::zero())?;
        let positive = region.boolean_region(&positive, BooleanOp::Intersection)?;
        let negative = region.boolean_region(&negative, BooleanOp::Intersection)?;
        Ok([positive, negative])
    })
}

/// Names the sweep construction step whose scalar arithmetic failed.
///
/// A scalar that could not be proved nonzero is an undecided predicate, not
/// invalid input; exhaustion and other scalar problems keep their own cause.
const fn sweep_scalar_error(
    problem: hyperreal::Problem,
    predicate: &'static str,
    decisions: &GeometryDecisions,
) -> ValidationError {
    match problem {
        hyperreal::Problem::UnknownZero | hyperreal::Problem::UnknownCertificate => {
            ValidationError::PredicateUndecided {
                predicate,
                policy: decisions.predicate_policy(),
            }
        },
        problem => ValidationError::Scalar(problem),
    }
}

/// Normalizes a sweep direction already proved nonzero by policy decisions.
///
/// The structural normalization is tried first, retaining its representation.
/// Missing structural evidence is not a zero: the value is then normalized
/// with the scalar layer's certified magnitude.
fn sweep_unit_direction(
    direction: &Vector3,
    predicate: &'static str,
    decisions: &GeometryDecisions,
) -> Result<Vector3, ValidationError> {
    match direction.normalize_checked() {
        Ok(unit) => Ok(unit),
        Err(hyperreal::Problem::UnknownZero) => direction.normalize(),
        Err(problem) => Err(problem),
    }
    .map_err(|problem| sweep_scalar_error(problem, predicate, decisions))
}

/// Decides whether two path chords point in exactly opposite directions,
/// where the bisector tangent of a sweep vertex is undefined.
fn sweep_chords_reverse(
    incoming: &Vector3,
    outgoing: &Vector3,
    decisions: &GeometryDecisions,
) -> Result<bool, ValidationError> {
    // One certified nonzero cross-product component proves the chords are
    // not parallel, even when another component is undecided.
    let mut undecided = None;
    for component in &incoming.cross(outgoing).0 {
        match decisions.sign(component, "sweep path reversal") {
            Ok(hyperreal::RealSign::Zero) => {},
            Ok(_) => return Ok(false),
            Err(error) => undecided = Some(error),
        }
    }
    if let Some(error) = undecided {
        return Err(error);
    }
    Ok(
        decisions.sign(&incoming.dot(outgoing), "sweep path reversal")?
            == hyperreal::RealSign::Negative,
    )
}

/// Orients the first sweep slice from `+z` to its unit normal `tangent`,
/// whose unnormalized direction is `direction`.
///
/// Normals along the `z` axis are decided under the policy and receive the
/// identity or the half turn about `y`; all others receive the minimal
/// rotation.
fn sweep_initial_frame(
    direction: &Vector3,
    tangent: &Vector3,
    decisions: &GeometryDecisions,
) -> Result<Matrix4, ValidationError> {
    let mut undecided = None;
    for component in &direction.0[..2] {
        match decisions.sign(component, "sweep initial tangent orientation") {
            Ok(hyperreal::RealSign::Zero) => {},
            Ok(_) => {
                return Matrix4::rotation_between_unit_vectors(&Vector3::z(), tangent)
                    .map_err(|problem| {
                        sweep_scalar_error(problem, "sweep frame transport", decisions)
                    });
            },
            Err(error) => undecided = Some(error),
        }
    }
    if let Some(error) = undecided {
        return Err(error);
    }
    Ok(
        match decisions.sign(&direction.0[2], "sweep initial tangent orientation")? {
            hyperreal::RealSign::Negative => Matrix4::rotation_y(Real::pi()),
            _ => Matrix4::identity(),
        },
    )
}

/// Sweeps a native filled region along a 3D point path.
///
/// Consecutive points proved equal are coalesced, and a path whose last point
/// equals its first is closed. Each remaining point carries one copy of the
/// profile, placed rigidly in the plane through that point whose normal is the
/// unit chord at an open endpoint, or otherwise the normalized bisector of the
/// two adjacent unit chords. The first slice is rotated minimally from `+z` (a
/// half turn about `y` for `-z`), and every later frame is transported by the
/// exact minimal rotation between consecutive normals. An exact line profile
/// keeps its own coordinates; other profiles are sampled at
/// [`SAMPLED_PROFILE_CHORD_ERROR`]. Slice coordinates are exact images of
/// those profile coordinates. They are rational when the profile and every
/// unit chord and bisector are rational; for a general path they are nested
/// square roots, and later strict predicates on them can be undecidable.
///
/// Fewer than two distinct points, a closed path with fewer than three, a
/// revisited point, or a chord that exactly reverses its predecessor returns
/// [`ValidationError::InvalidArguments`]. A predicate the selected policy
/// cannot decide returns [`ValidationError::PredicateUndecided`] naming the
/// sweep step, and other scalar failures keep their
/// [`hyperreal::Problem`] cause.
pub fn try_sweep(
    region: &CurveRegion2,
    path: &[Point3],
    context: GeometryContext,
) -> Result<GeometryOutcome<TriangleMesh>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let mut deduplicated = Vec::with_capacity(path.len());
    for point in path {
        let duplicate = match deduplicated.last() {
            Some(previous) => decisions.decide(
                hyperlimit::point3_equal(previous, point, decisions.predicate_policy()),
                "sweep adjacent-point equality",
            )?,
            None => false,
        };
        if !duplicate {
            deduplicated.push(point.clone());
        }
    }
    let path = deduplicated;
    if path.len() < 2 {
        return Err(ValidationError::InvalidArguments);
    }
    let closed = match path.first().zip(path.last()) {
        Some((first, last)) => decisions.decide(
            hyperlimit::point3_equal(first, last, decisions.predicate_policy()),
            "sweep endpoint equality",
        )?,
        None => return Err(ValidationError::InvalidArguments),
    };
    let slice_count = if closed { path.len() - 1 } else { path.len() };
    if slice_count < if closed { 3 } else { 2 } {
        return Err(ValidationError::InvalidArguments);
    }
    for index in 0..slice_count {
        for previous in &path[..index] {
            if decisions.decide(
                hyperlimit::point3_equal(previous, &path[index], decisions.predicate_policy()),
                "sweep repeated-point equality",
            )? {
                return Err(ValidationError::InvalidArguments);
            }
        }
    }
    let segment_count = if closed { slice_count } else { slice_count - 1 };
    // Each chord joins two points proved distinct above, so it is nonzero.
    let chords = (0..segment_count)
        .map(|index| &path[(index + 1) % slice_count] - &path[index])
        .collect::<Vec<_>>();
    let segment_directions = chords
        .iter()
        .map(|chord| sweep_unit_direction(chord, "sweep segment direction", &decisions))
        .collect::<Result<Vec<_>, _>>()?;
    // Slice normals: the unit chord at an open endpoint, otherwise the
    // normalized bisector of the adjacent unit chords. The bisector vanishes
    // exactly when the path reverses, which is decided on the chords.
    let mut tangents = Vec::with_capacity(slice_count);
    for index in 0..slice_count {
        if !closed && (index == 0 || index + 1 == slice_count) {
            let segment = if index == 0 { 0 } else { segment_count - 1 };
            tangents.push((chords[segment].clone(), segment_directions[segment].clone()));
            continue;
        }
        let incoming = (index + segment_count - 1) % segment_count;
        let outgoing = index % segment_count;
        if sweep_chords_reverse(&chords[incoming], &chords[outgoing], &decisions)? {
            return Err(ValidationError::InvalidArguments);
        }
        let bisector = &segment_directions[incoming] + &segment_directions[outgoing];
        let tangent = sweep_unit_direction(&bisector, "sweep tangent", &decisions)?;
        tangents.push((bisector, tangent));
    }
    let mut orientation = sweep_initial_frame(&tangents[0].0, &tangents[0].1, &decisions)?;
    let mut frames = Vec::with_capacity(slice_count);
    frames.push((orientation.clone(), path[0].clone()));
    for index in 1..slice_count {
        // Both tangents have a positive component along their shared chord,
        // so they are never antipodal and the minimal rotation exists.
        let transport =
            Matrix4::rotation_between_unit_vectors(&tangents[index - 1].1, &tangents[index].1)
                .map_err(|problem| {
                    sweep_scalar_error(problem, "sweep frame transport", &decisions)
                })?;
        orientation = transport * orientation;
        frames.push((orientation.clone(), path[index].clone()));
    }
    let map = |point: &Point2, frame: &(Matrix4, Point3)| -> Result<Point3, ValidationError> {
        frame
            .0
            .transform_point3(&Point3::new(
                point.x().clone(),
                point.y().clone(),
                Real::zero(),
            ))
            .map(|point| point + frame.1.to_vector())
            .map_err(|problem| {
                sweep_scalar_error(problem, "sweep slice placement", &decisions)
            })
    };
    let place = |point: &Point2, section: usize| map(point, &frames[section]);
    let mut positions = Vec::new();
    let mut triangles = Vec::new();
    for (material, holes) in try_modeling_profiles_with(region, &decisions)? {
        emit_section_walls(
            &mut positions,
            &mut triangles,
            &oriented_profile_rings(&material, &holes, &decisions)?,
            slice_count,
            if closed {
                SectionWallEnd::Closed
            } else {
                SectionWallEnd::Open
            },
            false,
            &place,
        )?;
        if !closed {
            emit_section_caps(
                &mut positions,
                &mut triangles,
                (&material, &holes),
                &[(0, true), (slice_count - 1, false)],
                &decisions,
                &place,
            )?;
        }
    }
    // Frame transport can collapse otherwise valid source geometry, so sweep
    // performs one policy-consistent whole-mesh certification before return.
    let mesh = validate_closed_mesh(
        TriangleMesh::new(positions, triangles),
        region.is_empty(),
        &decisions,
    )?;
    Ok(decisions.finish_mesh(mesh))
}

/// Extrudes while varying twist and terminal XY scale.
///
/// The section at fraction `f = i / slices`, for `i` in `0..=slices`, scales
/// the profile by `1 + (end_scale - 1) f` on each axis, then rotates it by
/// `twist_degrees f` about `+z` at height `height f`. An exact line profile
/// keeps its own coordinates, so zero twist with unit scale bounds the
/// ordinary [`try_extrude`] prism; other profiles are sampled at
/// [`SAMPLED_PROFILE_CHORD_ERROR`].
///
/// The height must be nonzero; a zero height returns
/// [`ValidationError::ZeroParameter`], like a zero extrusion height in
/// [`try_extrude`]. Both end scales must be positive, or both zero, which
/// tapers the profile to the apex `(0, 0, height)`: the last band of side
/// triangles meets there and there is no top cap. One zero or a negative scale
/// would collapse or reflect the top section and returns
/// [`ValidationError::NonPositiveParameter`].
///
/// Each side quadrilateral between consecutive sections is split along a
/// fixed diagonal, and the result is validated as a closed manifold, not as an
/// embedded solid. Side walls of a thin profile can cross: the strip
/// `[1, 1 + 2^-20] x [0, 2]` with a 90 degree twist in three slices and end
/// scale `(1/2, 2/3)` gives a closed mesh with negative signed volume.
pub fn extrude_twisted(
    input: &CurveRegion2,
    height: Real,
    twist_degrees: Real,
    end_scale: [Real; 2],
    slices: usize,
    context: GeometryContext,
) -> Result<GeometryOutcome<TriangleMesh>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    if slices < 1 {
        return Err(ValidationError::FieldLessThan {
            name: "slices",
            min: 1,
        });
    }
    let height_sign = decisions.decide(
        hyperlimit::classify_real_sign(&height, decisions.predicate_policy()),
        "twisted-extrusion height sign",
    )?;
    if height_sign == hyperlimit::Sign::Zero {
        return Err(ValidationError::ZeroParameter {
            name: "twisted-extrusion height",
        });
    }
    let mut scale_signs = [hyperlimit::Sign::Zero; 2];
    for (sign, scale) in scale_signs.iter_mut().zip(&end_scale) {
        *sign = decisions.decide(
            hyperlimit::classify_real_sign(scale, decisions.predicate_policy()),
            "twisted-extrusion scale sign",
        )?;
    }
    // Both end scales zero taper every section to the apex on the axis.
    let apex = match scale_signs {
        [hyperlimit::Sign::Positive, hyperlimit::Sign::Positive] => false,
        [hyperlimit::Sign::Zero, hyperlimit::Sign::Zero] => true,
        _ => {
            return Err(ValidationError::NonPositiveParameter {
                name: "twisted-extrusion end scale",
            });
        },
    };
    let parameters = (0..=slices)
        .map(|index| {
            let fraction = (Real::from(index as u64) / Real::from(slices as u64)).ok()?;
            let radians = (twist_degrees.clone() * fraction.clone() * Real::pi()
                / Real::from(180_u16))
            .ok()?;
            Some((
                radians.clone().sin(),
                radians.cos(),
                Real::one() + (end_scale[0].clone() - Real::one()) * fraction.clone(),
                Real::one() + (end_scale[1].clone() - Real::one()) * fraction.clone(),
                height.clone() * fraction,
            ))
        })
        .collect::<Option<Vec<_>>>()
        .ok_or(ValidationError::InvalidArguments)?;
    let map = |point: &Point2, parameter: &(Real, Real, Real, Real, Real)| {
        let x = point.x().clone() * parameter.2.clone();
        let y = point.y().clone() * parameter.3.clone();
        Point3::new(
            x.clone() * parameter.1.clone() - y.clone() * parameter.0.clone(),
            x * parameter.0.clone() + y * parameter.1.clone(),
            parameter.4.clone(),
        )
    };
    let reverse_sides = height_sign == hyperlimit::Sign::Negative;
    let place = |point: &Point2, section: usize| Ok(map(point, &parameters[section]));
    // At the apex the last band is triangles and there is no top cap.
    let (end, caps): (_, &[_]) = if apex {
        (SectionWallEnd::Apex, &[(0, !reverse_sides)])
    } else {
        (
            SectionWallEnd::Open,
            &[(0, !reverse_sides), (slices, reverse_sides)],
        )
    };
    let mut positions = Vec::new();
    let mut triangles = Vec::new();
    for (material, holes) in try_modeling_profiles_with(input, &decisions)? {
        emit_section_walls(
            &mut positions,
            &mut triangles,
            &oriented_profile_rings(&material, &holes, &decisions)?,
            parameters.len(),
            end,
            reverse_sides,
            &place,
        )?;
        emit_section_caps(
            &mut positions,
            &mut triangles,
            (&material, &holes),
            caps,
            &decisions,
            &place,
        )?;
    }
    let mesh = validate_closed_mesh(
        TriangleMesh::new(positions, triangles),
        input.is_empty(),
        &decisions,
    )?;
    Ok(decisions.finish_mesh(mesh))
}

/// Filled contours sampled from Hypersdf's normalized inverse-square field.
///
/// Positive radii/iso, nonnegative padding and dimensions of at least two are
/// required, including for empty inputs. Errors remain typed. The exact chord
/// proposal uses native marching squares and nesting-aware region admission;
/// its numerical certainty does not certify fidelity to the continuous field.
#[cfg(feature = "metaballs")]
pub fn metaballs(
    balls: &[(Point2, Real)],
    resolution: (usize, usize),
    iso_value: impl IntoReal,
    padding: impl IntoReal,
) -> Result<CurveRegion2, ValidationError> {
    let iso_value = iso_value.into_real()?;
    let padding = padding.into_real()?;
    metaballs_with_context(balls, resolution, iso_value, padding, GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Planar metaball sampling with the selected extraction policy and certainty.
/// Field construction and conservative bounds remain strict.
#[cfg(feature = "metaballs")]
pub fn metaballs_with_context(
    balls: &[(Point2, Real)],
    resolution: (usize, usize),
    iso_value: Real,
    padding: Real,
    context: GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    super::metaballs::sample(balls, resolution, iso_value, padding, context)
}

/// Filled contours traced from a grayscale image.
#[cfg(feature = "image-io")]
pub fn from_image(
    image: &image::GrayImage,
    threshold: u8,
) -> Result<CurveRegion2, super::image::RasterTraceError> {
    super::image::try_from_image(image, threshold)
        .map(super::image::RasterTraceReport::into_region)
}

/// Filled native line and Bézier glyph outlines at a strictly positive point size.
///
/// Uses exactly 127/360 millimeters per point and retains the requested `Real`
/// scale. Coordinates emitted by ttf-parser are promoted exactly from f32;
/// this does not certify fidelity to coordinates before parser rounding.
/// Layout uses horizontal advances and pair adjustments, with newline, carriage
/// return and tab controls; it is not a shaping or bidirectional-text engine.
/// Invalid fonts, missing glyphs and failed outlines return typed errors.
#[cfg(feature = "truetype-text")]
pub fn truetype_text(
    text: &str,
    font_data: &[u8],
    scale: impl IntoReal,
) -> Result<CurveRegion2, ValidationError> {
    let scale = scale.into_real()?;
    truetype_text_with_context(text, font_data, scale, GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Policy-explicit text construction with an operation-local certainty receipt.
#[cfg(feature = "truetype-text")]
pub fn truetype_text_with_context(
    text: &str,
    font_data: &[u8],
    scale: Real,
    context: GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    positive_with(&scale, "font scale", &decisions)?;
    let region =
        decisions.exact_curve(|| super::truetype::text_region(text, font_data, scale))?;
    Ok(decisions.finish(region))
}

/// Hershey stroke geometry as native open curve strings in Cartesian coordinates.
///
/// Hershey's source records use screen-style coordinates with positive Y
/// pointing down. The returned geometry reflects that axis so positive Y
/// points up, consistently with the rest of the planar curve API.
/// Size must be provably nonzero; negative sizes are supported. Missing glyphs,
/// malformed records and failed strokes return typed errors without partial text.
#[cfg(feature = "hershey-text")]
pub fn hershey_strings(
    text: &str,
    font: &hypercurve::hershey::Font<'_>,
    size: impl IntoReal,
) -> Result<Vec<CurveString2>, ValidationError> {
    let size = size.into_real()?;
    hershey_strings_with_context(text, font, size, GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Policy-explicit Hershey text with an operation-local certainty receipt.
#[cfg(feature = "hershey-text")]
pub fn hershey_strings_with_context(
    text: &str,
    font: &hypercurve::hershey::Font<'_>,
    size: Real,
    context: GeometryContext,
) -> Result<GeometryOutcome<Vec<CurveString2>>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let strings = decisions.exact_curve(|| hypercurve::hershey::strings(text, font, size))?;
    let source_to_cartesian = decisions.exact_curve(|| {
        hypercurve::Similarity2::try_from_real_affine(
            Real::one(),
            Real::zero(),
            Real::zero(),
            -Real::one(),
            Real::zero(),
            Real::zero(),
        )
    })?;
    let mut cartesian = solid::reserved(strings.len())?;
    for string in strings {
        cartesian.push(
            decisions.exact_curve(|| string.transform_similarity(&source_to_cartesian))?,
        );
    }
    Ok(decisions.finish(cartesian))
}

/// Applies a homogeneous matrix whose restriction to the XY plane is affine.
///
/// The image must remain in the XY plane and have a constant nonzero
/// homogeneous weight. That weight is divided out, including negative weights.
/// Perspective and out-of-plane maps return typed unsupported errors. A zero
/// constant weight with a nonzero coordinate image returns an unbounded error;
/// an all-zero homogeneous image is invalid. Hypercurve validates the planar
/// map's nonsingularity and topology, including for empty inputs.
pub fn transformed(
    input: &CurveRegion2,
    matrix: &Matrix4,
) -> Result<CurveRegion2, ValidationError> {
    transformed_with_context(input, matrix, GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Applies the same planar matrix contract with aggregate predicate certainty.
pub fn transformed_with_context(
    input: &CurveRegion2,
    matrix: &Matrix4,
    context: GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    for value in [&matrix.0[3][0], &matrix.0[3][1]] {
        if decisions.sign(value, "planar homogeneous weight is constant")?
            != hyperreal::RealSign::Zero
        {
            return Err(ValidationError::UnsupportedTransform {
                reason: "perspective planar transforms require a projective curve operation",
            });
        }
    }
    for value in [&matrix.0[2][0], &matrix.0[2][1], &matrix.0[2][3]] {
        if decisions.sign(value, "transform preserves the XY plane")?
            != hyperreal::RealSign::Zero
        {
            return Err(ValidationError::UnsupportedTransform {
                reason: "the transform leaves the XY plane",
            });
        }
    }
    let weight = &matrix.0[3][3];
    if decisions.sign(weight, "planar homogeneous weight")? == hyperreal::RealSign::Zero {
        let mut nonzero_image = false;
        for value in [
            &matrix.0[0][0],
            &matrix.0[0][1],
            &matrix.0[0][3],
            &matrix.0[1][0],
            &matrix.0[1][1],
            &matrix.0[1][3],
        ] {
            nonzero_image |= decisions.sign(value, "homogeneous coordinate image")?
                != hyperreal::RealSign::Zero;
        }
        if !nonzero_image {
            // [0,0,0,0] is not a homogeneous point, including at infinity.
            return Err(ValidationError::InvalidArguments);
        }
        return Err(ValidationError::UnboundedTransform {
            reason: "the XY plane has zero homogeneous weight",
        });
    }
    // z=0 removes the third column. Normalize the remaining homogeneous
    // coordinates; do not duplicate Hypercurve's affine determinant proof.
    let [m00, m01, m10, m11, tx, ty] = [
        (&matrix.0[0][0] / weight)?,
        (&matrix.0[0][1] / weight)?,
        (&matrix.0[1][0] / weight)?,
        (&matrix.0[1][1] / weight)?,
        (&matrix.0[0][3] / weight)?,
        (&matrix.0[1][3] / weight)?,
    ];
    let region =
        decisions.exact_curve(|| input.transform_affine(&m00, &m01, &m10, &m11, &tx, &ty))?;
    Ok(decisions.finish(region))
}

/// Applies an exact planar translation to a filled native region.
pub fn translated(
    input: &CurveRegion2,
    x: impl IntoReal,
    y: impl IntoReal,
) -> Result<CurveRegion2, ValidationError> {
    let x = x.into_real()?;
    let y = y.into_real()?;
    translated_with_context(input, x, y, GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Applies the curve transform with the selected policy and aggregate certainty.
pub fn translated_with_context(
    input: &CurveRegion2,
    x: Real,
    y: Real,
    context: GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let region = decisions.exact_curve(|| {
        input.transform_affine(
            &Real::one(),
            &Real::zero(),
            &Real::zero(),
            &Real::one(),
            &x,
            &y,
        )
    })?;
    Ok(decisions.finish(region))
}

/// Rotates a filled native region counterclockwise about the origin in degrees.
pub fn rotated(
    input: &CurveRegion2,
    z_degrees: impl IntoReal,
) -> Result<CurveRegion2, ValidationError> {
    let z_degrees = z_degrees.into_real()?;
    rotated_with_context(input, z_degrees, GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Applies the curve transform with the selected policy and aggregate certainty.
pub fn rotated_with_context(
    input: &CurveRegion2,
    z_degrees: Real,
    context: GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let radians = (z_degrees * Real::pi() / Real::from(180_u16)).expect("180 is nonzero");
    let cosine = radians.clone().cos();
    let sine = radians.sin();
    let region = decisions.exact_curve(|| {
        input.transform_affine(
            &cosine,
            &(-sine.clone()),
            &sine,
            &cosine,
            &Real::zero(),
            &Real::zero(),
        )
    })?;
    Ok(decisions.finish(region))
}

/// Scales a filled native region independently along the planar axes.
///
/// A zero factor is a singular transform, not a zero-size feature: it returns
/// a typed invalid-input error, for empty input too, rather than an empty
/// region.
pub fn scaled(
    input: &CurveRegion2,
    x: impl IntoReal,
    y: impl IntoReal,
) -> Result<CurveRegion2, ValidationError> {
    let x = x.into_real()?;
    let y = y.into_real()?;
    scaled_with_context(input, x, y, GeometryContext::STRICT).map(GeometryOutcome::into_value)
}

/// Applies the curve transform with the selected policy and aggregate certainty.
pub fn scaled_with_context(
    input: &CurveRegion2,
    x: Real,
    y: Real,
    context: GeometryContext,
) -> Result<GeometryOutcome<CurveRegion2>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let region = decisions.exact_curve(|| {
        input.transform_affine(
            &x,
            &Real::zero(),
            &Real::zero(),
            &y,
            &Real::zero(),
            &Real::zero(),
        )
    })?;
    Ok(decisions.finish(region))
}

/// Classifies an exact XY point. Only a proved boundary returns `Ok(None)`;
/// undecided classifications and native failures return typed errors.
pub fn contains_xy(
    input: &CurveRegion2,
    x: impl IntoReal,
    y: impl IntoReal,
) -> Result<Option<bool>, ValidationError> {
    let x = x.into_real()?;
    let y = y.into_real()?;
    contains_xy_with_context(input, x, y, GeometryContext::STRICT)
        .map(GeometryOutcome::into_value)
}

/// Classifies a point with the selected policy. Boundaries return `None`;
/// undecided classifications retain their native error and context.
/// Empty regions retain any inherited topology qualification.
pub fn contains_xy_with_context(
    input: &CurveRegion2,
    x: Real,
    y: Real,
    context: GeometryContext,
) -> Result<GeometryOutcome<Option<bool>>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let classification =
        decisions.exact_curve(|| input.classify_point(&Point2::new(x, y).into()))?;
    let inside = match classification {
        RegionPointLocation::Inside => Some(true),
        RegionPointLocation::Outside => Some(false),
        RegionPointLocation::Boundary => None,
    };
    Ok(decisions.finish(inside))
}

/// Returns certified planar bounds with exact coordinates, embedded in the XY
/// plane.
///
/// Line and arc boundaries and native curve extrema give the region's exact
/// bounds. A boundary fragment cut at an algebraic parameter contributes the
/// certified envelope of its source curve over a rational parameter interval,
/// which encloses the region but can be looser; see `CurveRegion2::bounds`.
pub fn bounding_box(input: &CurveRegion2) -> Result<Option<Aabb>, ValidationError> {
    bounding_box_with_context(input, GeometryContext::STRICT).map(GeometryOutcome::into_value)
}

/// Computes curve bounds with the selected policy and aggregate certainty.
pub fn bounding_box_with_context(
    input: &CurveRegion2,
    context: GeometryContext,
) -> Result<GeometryOutcome<Option<Aabb>>, ValidationError> {
    let decisions = GeometryDecisions::new(context);
    let bounds = decisions.exact_curve(|| input.bounds())?.map(|bounds| {
        Aabb::new(
            Point3::new(bounds.min_x().clone(), bounds.min_y().clone(), Real::zero()),
            Point3::new(bounds.max_x().clone(), bounds.max_y().clone(), Real::zero()),
        )
    });
    Ok(decisions.finish(bounds))
}

/// Returns the same native solid translation helper used by the 3D grammar.
pub fn translated_solid(
    mesh: &TriangleMesh,
    x: impl IntoReal,
    y: impl IntoReal,
    z: impl IntoReal,
) -> Result<TriangleMesh, ValidationError> {
    let x = x.into_real()?;
    let y = y.into_real()?;
    let z = z.into_real()?;
    solid::SolidExt::translated(mesh, x, y, z)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ring_cache_keeps_only_strict_success_and_owned_identity() {
        REGION_RING_CACHE.with_borrow_mut(Vec::clear);
        for _ in 0..2 {
            assert!(polygon(&[]).is_err());
            assert!(square(terminally_unresolved_zero()).is_err());
            assert_eq!(REGION_RING_CACHE.with_borrow(Vec::len), 0);
        }
        let unit = square(Real::one()).unwrap();
        assert_eq!(REGION_RING_CACHE.with_borrow(Vec::len), 1);
        let warm = square_with_context(Real::one(), GeometryContext::APPROXIMATE_512).unwrap();
        assert_eq!(warm.certainty, crate::GeometryCertainty::Certified);
        let scaled = scaled(&warm.value, Real::from(2), Real::one()).unwrap();
        assert_eq!(scaled.filled_area().unwrap(), Some(Real::from(2)));
        assert_eq!(unit.filled_area().unwrap(), Some(Real::one()));
        for width in 2..=66 {
            let region = rectangle(Real::from(width), Real::one()).unwrap();
            assert_eq!(region.filled_area().unwrap(), Some(Real::from(width)));
        }
        assert_eq!(REGION_RING_CACHE.with_borrow(Vec::len), 64);
        assert_eq!(
            square(Real::one()).unwrap().filled_area().unwrap(),
            Some(Real::one())
        );
        REGION_RING_CACHE.with_borrow_mut(Vec::clear);
        let outcome =
            square_with_context(Real::one(), GeometryContext::APPROXIMATE_512).unwrap();
        assert_eq!(outcome.certainty, crate::GeometryCertainty::Certified);
        assert_eq!(REGION_RING_CACHE.with_borrow(Vec::len), 0);
        let child = std::thread::spawn(|| {
            assert_eq!(REGION_RING_CACHE.with_borrow(Vec::len), 0);
            assert_eq!(
                square(Real::one()).unwrap().filled_area().unwrap(),
                Some(Real::one())
            );
        });
        child.join().unwrap();
        assert_eq!(REGION_RING_CACHE.with_borrow(Vec::len), 0);
    }

    fn certified_unit_circle_zero() -> Real {
        let sine = Real::e().sin();
        let cosine = Real::e().cos();
        &sine * &sine + &cosine * &cosine - Real::one()
    }

    fn terminally_unresolved_zero() -> Real {
        // Angle addition remains outside the bounded exact normal form. The
        // unit-circle identity now has a proof and is tested separately below.
        let zero = (Real::e() + Real::one()).sin()
            - Real::e().sin() * Real::one().cos()
            - Real::e().cos() * Real::one().sin();
        assert!(matches!(
            hyperlimit::compare_reals(
                &zero,
                &Real::zero(),
                hyperlimit::PredicatePolicy::STRICT
            ),
            hyperlimit::PredicateOutcome::Unknown { .. }
        ));
        zero
    }

    #[test]
    fn unit_circle_zero_is_certified_in_empty_profiles_and_sweep_deduplication() {
        let zero = certified_unit_circle_zero();
        assert_eq!(
            zero.exact_rational_normal_form(),
            Some(hyperreal::Rational::zero())
        );
        let profile = square(Real::from(2_u8)).unwrap();
        let path = [
            Point3::origin(),
            Point3::new(zero.clone(), Real::zero(), Real::zero()),
            Point3::new(Real::zero(), Real::zero(), Real::one()),
        ];
        for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
            let empty = square_with_context(zero.clone(), context).unwrap();
            assert!(empty.value.is_empty());
            assert_eq!(empty.certainty, crate::GeometryCertainty::Certified);
            let swept = try_sweep(&profile, &path, context).unwrap();
            assert_eq!(swept.certainty, crate::GeometryCertainty::Certified);
            assert_exact_solid_boundary(&swept.value, "certified duplicate sweep point");
            let six_volume =
                swept
                    .value
                    .triangles
                    .iter()
                    .fold(Real::zero(), |sum, triangle| {
                        let a = swept.value.positions[triangle.v0].to_vector();
                        let b = swept.value.positions[triangle.v1].to_vector();
                        let c = swept.value.positions[triangle.v2].to_vector();
                        sum + a.dot(&b.cross(&c))
                    });
            assert_eq!(six_volume, Real::from(24));
        }
    }

    fn signed_volume(mesh: &TriangleMesh) -> f64 {
        mesh.triangles
            .iter()
            .filter_map(|triangle| {
                let a = mesh.positions[triangle.v0].to_vector();
                let b = mesh.positions[triangle.v1].to_vector();
                let c = mesh.positions[triangle.v2].to_vector();
                a.dot(&b.cross(&c)).to_f64_lossy()
            })
            .sum::<f64>()
            / 6.0
    }

    fn assert_exact_solid_boundary(mesh: &TriangleMesh, label: &str) {
        assert!(
            mesh.has_unique_nondegenerate_triangles(&crate::MESH_CONTEXT)
                .expect("boundary predicate")
                .into_value(),
            "{label} contains duplicate or degenerate exact triangles"
        );
        assert!(
            mesh.is_closed_manifold_geometry(&crate::MESH_CONTEXT)
                .expect("boundary predicate")
                .into_value(),
            "{label} is not an exact geometric two-manifold"
        );
    }

    #[test]
    fn finite_profiles_reuse_retained_loop_topology_without_reconsumption() {
        let start_x = Real::e().sin();
        let terminal_start_x = start_x.clone() + terminally_unresolved_zero();
        let p0 = Point2::new(start_x.clone(), Real::zero());
        let p1 = Point2::new(start_x.clone() + Real::one(), Real::zero());
        let p2 = Point2::new(start_x.clone() + Real::one(), Real::one());
        let p3 = Point2::new(start_x, Real::one());
        let terminal_p0 = Point2::new(terminal_start_x, Real::zero());
        let path = CurvePath2::try_new(vec![
            Curve2::from(LineSeg2::try_new(p0, p1.clone()).unwrap()),
            Curve2::from(LineSeg2::try_new(p1, p2.clone()).unwrap()),
            Curve2::from(LineSeg2::try_new(p2, p3.clone()).unwrap()),
            Curve2::from(LineSeg2::try_new(p3, terminal_p0).unwrap()),
        ])
        .unwrap();
        let constructed = hypercurve::provisional(|| {
            CurveRegion2::try_from_boundary_paths(&[path], hypercurve::FillRule::EvenOdd)
        });
        assert_eq!(
            constructed.certainty(),
            hypercurve::CurveCertainty::Approximate512Consumed
        );
        let region = constructed.into_unverified().unwrap();
        // Reuse the retained loop topology while preserving the consumed
        // closure receipt, including after an authorized warm-cache read.
        for context in [
            GeometryContext::STRICT,
            GeometryContext::APPROXIMATE_512,
            GeometryContext::STRICT,
            GeometryContext::APPROXIMATE_512,
            GeometryContext::STRICT,
        ] {
            let profiles = try_finite_profiles(&region, context);
            if context == GeometryContext::STRICT {
                let error = profiles.unwrap_err();
                assert_eq!(error.kind(), crate::errors::ValidationErrorKind::Undecided);
                assert!(matches!(error,
                    ValidationError::ExactCurve(hypercurve::ExactCurveError::Blocked(reason))
                        if reason.reason() == hypercurve::UncertaintyReason::Predicate));
            } else {
                let profiles = profiles.unwrap();
                assert_eq!(
                    profiles.certainty,
                    crate::GeometryCertainty::Approximate512Consumed
                );
                assert_eq!(profiles.value.len(), 1);
                assert!(profiles.value[0].holes().is_empty());
                assert!(profiles.value[0].material().is_closed());
            }
        }
    }

    #[test]
    fn filled_constructor_and_extrusion_remain_native() {
        let region: CurveRegion2 = circle(Real::from(2_u8), 32).expect("valid planar fixture");
        assert!(!region.is_empty());
        let mesh: TriangleMesh =
            try_extrude(&region, Real::from(3_u8), GeometryContext::STRICT)
                .expect("native extrusion")
                .into_value();
        assert!(!mesh.triangles.is_empty());
        assert!(mesh.is_closed_manifold());
        hypermesh::polygon_soup(&crate::MESH_CONTEXT, &[mesh.as_ref()])
            .expect("native extrusion must be reusable Hypermesh input");
    }

    #[test]
    fn profile_solid_constructors_share_closed_indexed_topology() {
        let square = square(Real::from(2_u8)).expect("valid planar fixture");
        let linear = try_extrude(&square, Real::from(2_u8), GeometryContext::STRICT)
            .expect("linear extrusion")
            .into_value();
        assert!(linear.is_closed_manifold(), "linear extrusion");
        assert_exact_solid_boundary(&linear, "linear extrusion");
        assert!(signed_volume(&linear) > 0.0);
        let linear_negative = try_extrude(&square, Real::from(-2_i8), GeometryContext::STRICT)
            .expect("negative linear extrusion")
            .into_value();
        assert!(
            linear_negative.is_closed_manifold(),
            "negative linear extrusion"
        );
        assert_exact_solid_boundary(&linear_negative, "negative linear extrusion");
        assert!(signed_volume(&linear_negative) > 0.0);
        let twisted = extrude_twisted(
            &square,
            Real::from(2_u8),
            Real::from(45_u8),
            [Real::one(), Real::one()],
            4,
            GeometryContext::STRICT,
        )
        .expect("valid twisted extrusion")
        .into_value();
        assert!(twisted.is_closed_manifold(), "twisted extrusion");
        assert_exact_solid_boundary(&twisted, "twisted extrusion");
        assert!(signed_volume(&twisted) > 0.0);
        let path = [
            Point3::origin(),
            Point3::new(Real::zero(), Real::zero(), Real::one()),
            Point3::new(Real::zero(), Real::zero(), Real::one()),
            Point3::new(Real::zero(), Real::zero(), Real::from(2_u8)),
        ];
        let swept = try_sweep(&square, &path, GeometryContext::STRICT)
            .expect("valid sweep")
            .into_value();
        assert!(swept.is_closed_manifold(), "sweep");
        assert_exact_solid_boundary(&swept, "sweep");
        assert!(signed_volume(&swept) > 0.0);

        let radial = polygon(&[
            [Real::one(), -Real::one()],
            [Real::from(2_u8), -Real::one()],
            [Real::from(2_u8), Real::one()],
            [Real::one(), Real::one()],
        ])
        .expect("valid radial profile fixture");
        let revolved = revolve(&radial, Real::from(360_u16), 16, GeometryContext::STRICT)
            .expect("valid revolution")
            .into_value();
        assert!(revolved.is_closed_manifold(), "revolution");
        assert_exact_solid_boundary(&revolved, "revolution");
        assert!(signed_volume(&revolved) > 0.0);
        for angle in [Real::from(180_u16), Real::from(-180_i16)] {
            let partial = revolve(&radial, angle, 8, GeometryContext::STRICT)
                .expect("valid partial revolution")
                .into_value();
            assert!(partial.is_closed_manifold(), "partial revolution");
            assert_exact_solid_boundary(&partial, "partial revolution");
            assert!(signed_volume(&partial) > 0.0);
        }
    }

    #[test]
    fn sweep_rejects_revisited_open_path_vertices() {
        let square = square(Real::from(2_u8)).expect("valid planar fixture");
        let a = Point3::new(Real::from(-1_i8), Real::one(), Real::from(-1_i8));
        let b = Point3::new(Real::one(), Real::one(), Real::from(-1_i8));
        let path = [
            a.clone(),
            b.clone(),
            Point3::new(Real::one(), Real::from(-1_i8), Real::one()),
            Point3::new(Real::from(-1_i8), Real::one(), Real::one()),
            a,
            b,
        ];

        assert!(try_sweep(&square, &path, GeometryContext::STRICT).is_err());
    }

    #[test]
    fn degenerate_profile_solid_requests_do_not_claim_solids() {
        let square = square(Real::from(2_u8)).expect("valid planar fixture");
        assert!(try_extrude(&square, Real::zero(), GeometryContext::STRICT).is_err());
        assert!(
            try_extrude_vector(
                &square,
                Vector3::from_xyz(Real::one(), Real::zero(), Real::zero()),
                GeometryContext::STRICT,
            )
            .is_err()
        );
        assert!(try_sweep(&square, &[Point3::origin()], GeometryContext::STRICT).is_err());
        assert!(revolve(&square, Real::from(360_u16), 2, GeometryContext::STRICT).is_err());
        assert!(
            revolve(
                &square,
                Real::from(180_u16),
                usize::MAX,
                GeometryContext::STRICT,
            )
            .is_err()
        );
        assert!(
            extrude_twisted(
                &square,
                Real::one(),
                Real::zero(),
                [Real::zero(), Real::one()],
                2,
                GeometryContext::STRICT,
            )
            .is_err()
        );
    }

    #[test]
    fn approximate_sweep_reports_consumed_terminal_point_equality() {
        let square = square(Real::from(2_u8)).expect("valid planar fixture");
        let symbolic_zero = terminally_unresolved_zero();
        let path = [
            Point3::origin(),
            Point3::new(symbolic_zero, Real::zero(), Real::zero()),
            Point3::new(Real::zero(), Real::zero(), Real::one()),
        ];

        assert!(try_sweep(&square, &path, GeometryContext::STRICT).is_err());
        let outcome = try_sweep(&square, &path, GeometryContext::APPROXIMATE_512)
            .expect("approximate-512 should deduplicate the symbolic zero point");
        assert_eq!(
            outcome.certainty,
            crate::GeometryCertainty::Approximate512Consumed
        );
        assert!(outcome.value.is_closed_manifold());
    }

    #[test]
    fn sweep_scalar_failures_keep_their_outcome_category() {
        for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
            let decisions = GeometryDecisions::new(context);
            for problem in [
                hyperreal::Problem::UnknownZero,
                hyperreal::Problem::UnknownCertificate,
            ] {
                let error = sweep_scalar_error(problem, "sweep frame transport", &decisions);
                assert_eq!(
                    error,
                    ValidationError::PredicateUndecided {
                        predicate: "sweep frame transport",
                        policy: context.predicate_policy(),
                    }
                );
                assert_eq!(error.kind(), crate::errors::ValidationErrorKind::Undecided);
            }
            let exhausted =
                sweep_scalar_error(hyperreal::Problem::Exhausted, "sweep tangent", &decisions);
            assert_eq!(
                exhausted,
                ValidationError::Scalar(hyperreal::Problem::Exhausted)
            );
            assert_eq!(
                exhausted.kind(),
                crate::errors::ValidationErrorKind::ResourceExhausted
            );
            // A definite zero after the policy proved the value nonzero is a
            // scalar-layer contradiction, not caller input.
            assert_eq!(
                sweep_scalar_error(
                    hyperreal::Problem::DivideByZero,
                    "sweep tangent",
                    &decisions
                )
                .kind(),
                crate::errors::ValidationErrorKind::Geometry
            );
        }
    }

    #[test]
    fn native_region_boolean_does_not_construct_a_public_facade() {
        let left = square(Real::from(2_u8)).expect("valid planar fixture");
        let right =
            rectangle(Real::from(1_u8), Real::from(3_u8)).expect("valid planar fixture");
        let union: CurveRegion2 = left
            .boolean_region(&right, BooleanOp::Union)
            .expect("region union");
        assert!(!union.is_empty());
    }

    #[test]
    fn planar_constructors_reject_invalid_dimensions_and_ranges() {
        let negative = -Real::one();
        let invalid_compound =
            |result: Result<GeometryOutcome<CurveRegion2>, ValidationError>| result.is_err();
        assert!(circle(negative.clone(), 16).is_err());
        assert!(right_triangle(negative.clone(), Real::one()).is_err());
        assert!(ellipse(Real::one(), negative.clone(), 16).is_err());
        assert!(regular_ngon(5, negative.clone()).is_err());
        assert!(trapezoid(Real::one(), negative.clone(), Real::one(), Real::zero()).is_err());
        assert!(star(5, Real::one(), Real::from(2_u8)).is_err());
        assert!(invalid_compound(keyhole(
            Real::one(),
            Real::zero(),
            Real::one(),
            16,
            GeometryContext::STRICT,
        )));
        assert!(invalid_compound(ring(
            Real::one(),
            negative.clone(),
            16,
            GeometryContext::STRICT,
        )));
        assert!(invalid_compound(crescent(
            Real::one(),
            Real::from(2_u8),
            Real::zero(),
            16,
            GeometryContext::STRICT,
        )));
        assert!(invalid_compound(circle_with_keyway(
            Real::one(),
            16,
            Real::from(2_u8),
            Real::one(),
            GeometryContext::STRICT,
        )));
        assert!(invalid_compound(circle_with_flat(
            Real::one(),
            16,
            Real::one(),
            GeometryContext::STRICT,
        )));
        assert!(invalid_compound(circle_with_two_flats(
            Real::one(),
            16,
            Real::one(),
            GeometryContext::STRICT,
        )));
        assert!(
            airfoil_naca4(
                Real::from(10_u8),
                Real::from(4_u8),
                Real::from(12_u8),
                Real::one(),
                20,
            )
            .is_err()
        );
        assert!(
            airfoil_naca4(
                Real::from(2_u8),
                Real::from(10_u8),
                Real::from(12_u8),
                Real::one(),
                20,
            )
            .is_err()
        );
        assert!(
            airfoil_naca4(
                Real::zero(),
                Real::zero(),
                Real::from(100_u8),
                Real::one(),
                20,
            )
            .is_err()
        );
        assert!(star(usize::MAX, Real::from(2_u8), Real::one()).is_err());
        assert!(invalid_compound(reuleaux(
            usize::MAX,
            Real::one(),
            16,
            GeometryContext::STRICT,
        )));
        assert!(teardrop(Real::one(), Real::from(2_u8), usize::MAX).is_err());
        assert!(
            involute_rack(
                Real::one(),
                usize::MAX,
                Real::from(20_u8),
                Real::zero(),
                Real::zero(),
            )
            .is_err()
        );
        assert!(cycloidal_rack(Real::one(), 1, Real::zero(), usize::MAX).is_err());
        assert!(
            cycloidal_gear(
                Real::one(),
                12,
                (Real::from(3_u8) / Real::from(4_u8)).unwrap(),
                Real::zero(),
                usize::MAX,
            )
            .is_err()
        );
        assert!(
            airfoil_naca4(
                Real::zero(),
                Real::zero(),
                Real::from(12_u8),
                Real::one(),
                usize::MAX,
            )
            .is_err()
        );
        assert!(bspline_path(&[], usize::MAX, 1).is_err());
    }

    #[test]
    fn normalized_egg_and_heart_match_requested_sampled_bounds() {
        let egg_bounds = bounding_box(
            &egg(Real::from(3_u8), Real::from(5_u8), 24)
                .expect("valid sampled profile fixture"),
        )
        .unwrap()
        .unwrap();
        assert_eq!(
            egg_bounds.maxs.x.clone() - egg_bounds.mins.x,
            Real::from(3_u8)
        );
        assert_eq!(
            egg_bounds.maxs.y.clone() - egg_bounds.mins.y,
            Real::from(5_u8)
        );

        let heart_bounds = bounding_box(
            &heart(Real::from(4_u8), Real::from(6_u8), 32)
                .expect("valid sampled profile fixture"),
        )
        .unwrap()
        .unwrap();
        assert_eq!(
            heart_bounds.maxs.x.clone() - heart_bounds.mins.x,
            Real::from(4_u8)
        );
        assert_eq!(
            heart_bounds.maxs.y.clone() - heart_bounds.mins.y,
            Real::from(6_u8)
        );
    }

    #[test]
    fn analytic_shape_parameters_change_their_geometry() {
        let involute_20 = involute_gear(
            Real::one(),
            16,
            Real::from(20_u8),
            Real::zero(),
            Real::zero(),
            4,
        )
        .expect("valid sampled mechanical fixture");
        let involute_25 = involute_gear(
            Real::one(),
            16,
            Real::from(25_u8),
            Real::zero(),
            Real::zero(),
            4,
        )
        .expect("valid sampled mechanical fixture");
        assert!(!involute_20.is_empty());
        assert!(!involute_25.is_empty());
        assert_ne!(involute_20, involute_25);

        let cycloidal_a = cycloidal_gear(Real::one(), 16, Real::one(), Real::zero(), 4)
            .expect("valid sampled mechanical fixture");
        let cycloidal_b = cycloidal_gear(
            Real::one(),
            16,
            (Real::from(5_u8) / Real::from(4_u8)).unwrap(),
            Real::zero(),
            4,
        )
        .expect("valid sampled mechanical fixture");
        assert!(!cycloidal_a.is_empty());
        assert!(!cycloidal_b.is_empty());
        assert_ne!(cycloidal_a, cycloidal_b);

        let rack_20 =
            involute_rack(Real::one(), 4, Real::from(20_u8), Real::zero(), Real::zero())
                .expect("valid sampled mechanical fixture");
        let rack_25 =
            involute_rack(Real::one(), 4, Real::from(25_u8), Real::zero(), Real::zero())
                .expect("valid sampled mechanical fixture");
        assert!(!rack_20.is_empty());
        assert!(!rack_25.is_empty());
        assert_ne!(rack_20, rack_25);
        assert_ne!(
            cycloidal_rack(Real::one(), 4, Real::zero(), 4)
                .expect("valid sampled mechanical fixture"),
            cycloidal_rack(Real::one(), 4, Real::zero(), 8)
                .expect("valid sampled mechanical fixture")
        );

        let symmetric =
            airfoil_naca4(Real::zero(), Real::zero(), Real::from(12_u8), Real::one(), 20)
                .expect("valid sampled mechanical fixture");
        let cambered = airfoil_naca4(
            Real::from(2_u8),
            Real::from(4_u8),
            Real::from(12_u8),
            Real::one(),
            20,
        )
        .expect("valid sampled mechanical fixture");
        assert!(!symmetric.is_empty());
        assert!(!cambered.is_empty());
        assert_ne!(symmetric, cambered);
    }

    #[test]
    fn cycloidal_rack_alternates_face_and_root_flanks() {
        let teeth = 4;
        let tip = Real::one();
        let pitch_line = Real::zero();
        let root = -(Real::from(5_u8) / Real::from(4_u8)).expect("four is nonzero");
        let rack = cycloidal_rack(Real::one(), teeth, Real::zero(), 8)
            .expect("valid sampled mechanical fixture");
        let paths = rack.boundary_paths().expect("cycloidal rack exact boundary");
        let paths = paths;
        assert_eq!(paths.len(), 1, "rack must have one closed boundary");

        let curves = paths[0].curves();
        let vertices_at = |level: &Real| {
            let witness =
                hypercurve::CurvePoint2::from(Point2::new(Real::zero(), level.clone()));
            curves
                .iter()
                .filter(|curve| {
                    match curve
                        .start()
                        .compare_coordinate(&witness, hypercurve::Axis2::Y)
                    {
                        Ok(order) => order == Ordering::Equal,
                        Err(hypercurve::ExactCurveError::Blocked(_)) => false,
                        Err(error) => panic!("finite rack coordinate order: {error:?}"),
                    }
                })
                .count()
        };
        // The neighboring root/face samples are scalar multiples of the
        // same (run, rise), so native admission may remove their collinear
        // pitch vertex. Count geometric crossings, independent of that split.
        let pitch = hypercurve::CurvePoint2::from(Point2::new(Real::zero(), pitch_line));
        let crossings = curves
            .iter()
            .filter(|curve| {
                let start = curve
                    .start()
                    .compare_coordinate(&pitch, hypercurve::Axis2::Y)
                    .expect("rack start height");
                let end = curve
                    .end()
                    .compare_coordinate(&pitch, hypercurve::Axis2::Y)
                    .expect("rack end height");
                (start == Ordering::Less) != (end == Ordering::Less)
            })
            .count();
        assert_eq!(crossings, teeth * 2, "both sides must cross the pitch line");
        let epsilon = (Real::one() / Real::from(1_000_000_000_000_u64)).unwrap();
        for tooth in 0..teeth {
            for side in [-1, 1] {
                let x = Real::pi()
                    * (Real::from(tooth as u64) + (Real::from(side) / Real::from(4)).unwrap());
                for (y, expected) in [
                    (Real::zero(), None),
                    (-&epsilon, Some(true)),
                    (epsilon.clone(), Some(false)),
                ] {
                    let location =
                        contains_xy_with_context(&rack, x.clone(), y, GeometryContext::STRICT)
                            .expect("exact rack pitch contact and displaced membership");
                    assert_eq!(location.certainty, crate::GeometryCertainty::Certified);
                    assert_eq!(location.value, expected);
                }
            }
        }
        assert_eq!(
            vertices_at(&tip),
            teeth * 2,
            "both addendum cycloids must reach every tooth tip"
        );
        assert!(
            vertices_at(&root) >= teeth * 2,
            "both dedendum cycloids must reach every tooth root"
        );
    }

    #[test]
    fn gear_and_rack_profiles_form_one_hole_free_material_component() {
        let cycloidal = cycloidal_gear(
            Real::one(),
            16,
            (Real::from(5_u8) / Real::from(4_u8)).unwrap(),
            Real::zero(),
            8,
        )
        .expect("valid sampled mechanical fixture");
        let involute =
            involute_rack(Real::one(), 6, Real::from(20_u8), Real::zero(), Real::zero())
                .expect("valid sampled mechanical fixture");
        let cycloidal_rack_profile = cycloidal_rack(Real::one(), 6, Real::zero(), 12)
            .expect("valid sampled mechanical fixture");
        let readme_cycloidal = cycloidal_gear(
            Real::try_from(0.22).unwrap(),
            14,
            Real::try_from(0.275).unwrap(),
            Real::try_from(0.02).unwrap(),
            8,
        )
        .expect("valid sampled mechanical fixture");
        assert!(
            cycloidal_gear(
                Real::one(),
                16,
                (Real::from(3_u8) / Real::from(4_u8)).unwrap(),
                Real::zero(),
                8,
            )
            .is_err(),
            "overlapping neighboring cycloidal root flanks must be rejected"
        );
        let projection =
            FiniteProjectionOptions::try_new(1.0e-3).expect("positive projection tolerance");

        for (name, region) in [
            ("cycloidal gear", cycloidal),
            ("README cycloidal gear", readme_cycloidal),
            ("involute rack", involute),
            ("cycloidal rack", cycloidal_rack_profile),
        ] {
            let profiles = region
                .project_to_finite_profiles_exact(&projection)
                .unwrap_or_else(|error| panic!("{name} profile projection failed: {error}"));
            assert_eq!(
                profiles.len(),
                1,
                "{name} must be one connected material component"
            );
            assert!(
                profiles[0].holes().is_empty(),
                "{name} must not acquire holes from a self-intersecting boundary"
            );
            let paths = region
                .boundary_paths()
                .unwrap_or_else(|error| panic!("{name} exact boundary failed: {error}"));
            let paths = paths;
            assert_eq!(
                paths.len(),
                1,
                "{name} must expose one simple closed edge path"
            );
        }

        for (name, rack) in [
            (
                "involute rack",
                involute_rack(Real::one(), 6, Real::from(20_u8), Real::zero(), Real::zero())
                    .expect("valid sampled mechanical fixture"),
            ),
            (
                "cycloidal rack",
                cycloidal_rack(Real::one(), 6, Real::zero(), 12)
                    .expect("valid sampled mechanical fixture"),
            ),
        ] {
            let bounds = bounding_box(&rack)
                .unwrap_or_else(|error| panic!("{name} bounds failed: {error}"))
                .expect("nonempty rack bounds");
            assert!(
                bounds.mins.y < -Real::one(),
                "{name} must include backing below its tooth-root line"
            );
        }
    }

    #[test]
    fn higher_order_curve_constructors_and_flat_triangulation_are_usable() {
        let zero = Real::zero();
        let one = Real::one();
        let two = Real::from(2_u8);
        let controls = [
            [zero.clone(), zero.clone()],
            [one.clone(), two.clone()],
            [two.clone(), one.clone()],
            [Real::from(3_u8), zero.clone()],
        ];
        assert!(bezier_path(&controls, 16).is_ok());
        assert!(bspline_path(&controls, 2, 8).is_ok());
        assert!(bezier_path(&controls, 0).is_err());
        assert!(bspline_path(&controls, 2, 0).is_err());
        assert!(matches!(
            bezier_region_with_context(&controls, 16, GeometryContext::STRICT),
            Err(ValidationError::InvalidArguments)
        ));

        let closed_controls = [
            [zero.clone(), zero.clone()],
            [two.clone(), zero.clone()],
            [two.clone(), two.clone()],
            [zero.clone(), zero],
        ];
        assert!(!bezier_region(&closed_controls, 16).unwrap().is_empty());

        let mesh = try_triangulate(
            &square(two).expect("valid planar fixture"),
            GeometryContext::STRICT,
        )
        .expect("square triangulation")
        .into_value();
        assert_eq!(mesh.positions.len(), 4);
        assert_eq!(mesh.triangles.len(), 2);
        assert!(
            mesh.triangles
                .iter()
                .flat_map(|triangle| [triangle.v0, triangle.v1, triangle.v2])
                .all(|index| index < mesh.positions.len())
        );
    }

    #[test]
    fn nominal_planar_shape_catalog_constructs_material() {
        let one = Real::one();
        let two = Real::from(2_u8);
        let three_quarters = (Real::from(3_u8) / Real::from(4_u8)).unwrap();
        let compound = |result: Result<GeometryOutcome<CurveRegion2>, ValidationError>| {
            result.expect("nominal compound region").into_value()
        };
        for context in [GeometryContext::STRICT, GeometryContext::APPROXIMATE_512] {
            let crescent = crescent(two.clone(), one.clone(), one.clone(), 16, context)
                .expect("the symbolic crescent now has a strict scalar proof");
            assert_eq!(crescent.certainty, crate::GeometryCertainty::Certified);
            assert!(!crescent.value.is_empty());
            let profiles = try_finite_profiles(&crescent.value, GeometryContext::STRICT)
                .expect("the certified crescent retains projectable boundary topology");
            assert_eq!(profiles.certainty, crate::GeometryCertainty::Certified);
            assert!(!profiles.value.is_empty());
            // (-1,0) is material, (1,0) lies in the removed unit polygon,
            // and (3,0) lies outside the outer radius-two polygon.
            for (x, expected) in [(-1, true), (1, false), (3, false)] {
                let location = contains_xy_with_context(
                    &crescent.value,
                    Real::from(x),
                    Real::zero(),
                    GeometryContext::STRICT,
                )
                .expect("certified crescent membership");
                assert_eq!(location.certainty, crate::GeometryCertainty::Certified);
                assert_eq!(location.value, Some(expected));
            }
        }
        let shapes = [
            rectangle(two.clone(), one.clone()).expect("valid planar fixture"),
            square(two.clone()).expect("valid planar fixture"),
            circle(one.clone(), 16).expect("valid planar fixture"),
            right_triangle(two.clone(), one.clone()).expect("valid planar fixture"),
            polygon(&[
                [Real::zero(), Real::zero()],
                [one.clone(), Real::zero()],
                [Real::zero(), one.clone()],
            ])
            .expect("valid planar fixture"),
            ellipse(two.clone(), one.clone(), 16).expect("valid planar fixture"),
            regular_ngon(5, one.clone()).expect("valid planar fixture"),
            arrow(two.clone(), one.clone(), one.clone(), two.clone())
                .expect("valid sampled profile fixture"),
            trapezoid(one.clone(), two.clone(), one.clone(), one.clone())
                .expect("valid sampled profile fixture"),
            star(5, two.clone(), one.clone()).expect("valid sampled profile fixture"),
            teardrop(two.clone(), Real::from(3_u8), 12)
                .expect("valid sampled profile fixture"),
            egg(two.clone(), Real::from(3_u8), 16).expect("valid sampled profile fixture"),
            rounded_rectangle(two.clone(), one.clone(), three_quarters.clone(), 4)
                .expect("valid planar fixture"),
            squircle(two.clone(), one.clone(), 16).expect("valid sampled profile fixture"),
            compound(keyhole(
                one.clone(),
                one.clone(),
                two.clone(),
                16,
                GeometryContext::APPROXIMATE_512,
            )),
            compound(reuleaux(3, two.clone(), 24, GeometryContext::APPROXIMATE_512)),
            compound(ring(
                one.clone(),
                three_quarters.clone(),
                16,
                GeometryContext::APPROXIMATE_512,
            )),
            pie_slice(one.clone(), Real::zero(), Real::from(90_u8), 8)
                .expect("valid sampled profile fixture"),
            heart(two.clone(), two.clone(), 16).expect("valid sampled profile fixture"),
            supershape(
                one.clone(),
                one.clone(),
                Real::from(5_u8),
                two.clone(),
                one.clone(),
                one.clone(),
                24,
            )
            .expect("valid sampled profile fixture"),
            compound(circle_with_keyway(
                two.clone(),
                24,
                one.clone(),
                three_quarters.clone(),
                GeometryContext::APPROXIMATE_512,
            )),
            compound(circle_with_flat(
                two.clone(),
                24,
                one.clone(),
                GeometryContext::APPROXIMATE_512,
            )),
            compound(circle_with_two_flats(
                two.clone(),
                24,
                one.clone(),
                GeometryContext::APPROXIMATE_512,
            )),
            involute_gear(
                one.clone(),
                16,
                Real::from(20_u8),
                Real::zero(),
                Real::zero(),
                4,
            )
            .expect("valid sampled mechanical fixture"),
            cycloidal_gear(one.clone(), 16, one.clone(), Real::zero(), 4)
                .expect("valid sampled mechanical fixture"),
            involute_rack(one.clone(), 4, Real::from(20_u8), Real::zero(), Real::zero())
                .expect("valid sampled mechanical fixture"),
            cycloidal_rack(one.clone(), 4, Real::zero(), 8)
                .expect("valid sampled mechanical fixture"),
            airfoil_naca4(Real::from(2_u8), Real::from(4_u8), Real::from(12_u8), one, 20)
                .expect("valid sampled mechanical fixture"),
        ];
        for (index, shape) in shapes.into_iter().enumerate() {
            assert!(!shape.is_empty(), "planar catalog entry {index}");
            assert!(
                !try_finite_profiles(&shape, GeometryContext::APPROXIMATE_512)
                    .unwrap_or_else(|error| {
                        panic!("finite catalog profile {index} failed: {error}")
                    })
                    .into_value()
                    .is_empty(),
                "finite profile for catalog entry {index}"
            );
        }
    }

    #[test]
    fn hilbert_samples_are_centered_symmetrically_in_the_usable_grid() {
        let strings = hilbert_strings(
            &square(Real::from(8_u8)).expect("valid planar fixture"),
            3,
            Real::one(),
        )
        .expect("the square has an exact Hilbert line string");
        let points = strings
            .iter()
            .flat_map(|string| string.segments())
            .flat_map(|segment| [segment.start(), segment.end()])
            .collect::<Vec<_>>();
        assert!(!points.is_empty());
        let order = |left: &&Real, right: &&Real| {
            hyperlimit::compare_reals(left, right, crate::PREDICATE_POLICY)
                .value()
                .expect(
                    "catalog coordinates must be ordered by the centralized predicate policy",
                )
        };
        let min_x = points.iter().map(|point| point.x()).min_by(order).unwrap();
        let max_x = points.iter().map(|point| point.x()).max_by(order).unwrap();
        let min_y = points.iter().map(|point| point.y()).min_by(order).unwrap();
        let max_y = points.iter().map(|point| point.y()).max_by(order).unwrap();
        let expected_min = (Real::from(11_u8) / Real::from(8_u8)).unwrap();
        let expected_max = (Real::from(53_u8) / Real::from(8_u8)).unwrap();
        assert_eq!(min_x, &expected_min);
        assert_eq!(min_y, &expected_min);
        assert_eq!(max_x, &expected_max);
        assert_eq!(max_y, &expected_max);
    }

    #[cfg(feature = "hershey-text")]
    #[test]
    fn hershey_catalog_is_native_hypercurve_code() {
        let strings =
            hershey_strings("CSG", &crate::curve::hershey::fonts::FUTURAL, Real::one())
                .unwrap();

        assert!(!strings.is_empty());
        assert_eq!(crate::curve::hershey::fonts::ALL.len(), 32);
        assert!(strings.iter().all(|string| !string.segments().is_empty()));
    }

    #[cfg(feature = "hershey-text")]
    #[test]
    fn hershey_strings_convert_source_coordinates_to_y_up() {
        let font = crate::curve::hershey::Font::new(&["MWRMNV RRMVV"], 'A');
        let strings = hershey_strings("A", &font, Real::one()).unwrap();

        assert_eq!(strings.len(), 2);
        let left = &strings[0].segments()[0];
        let right = &strings[1].segments()[0];
        assert_eq!(left.start(), &Point2::new(Real::zero(), Real::from(5_u8)));
        assert_eq!(left.end(), &Point2::new(Real::from(-4_i8), Real::from(-4_i8)));
        assert_eq!(right.start(), &Point2::new(Real::zero(), Real::from(5_u8)));
        assert_eq!(right.end(), &Point2::new(Real::from(4_u8), Real::from(-4_i8)));
    }
}

/// Source-diagnostic witnesses for the modeling surface and closure validators.
#[cfg(test)]
mod native_boundary_source_probe {
    use super::*;
    use crate::errors::ValidationErrorKind;
    use hypermesh::Triangle;
    use std::error::Error;

    /// Compares both native validation errors with modeled errors and requires retained sources.
    fn check(mesh: TriangleMesh, kind: ValidationErrorKind) {
        let context = GeometryContext::STRICT;
        let native = [
            mesh.has_unique_nondegenerate_triangles(&context.mesh_context())
                .unwrap_err(),
            mesh.is_closed_manifold_geometry(&context.mesh_context())
                .unwrap_err(),
        ];
        let decisions = GeometryDecisions::new(context);
        let modeled = [
            validate_surface_mesh(mesh.clone(), &decisions).unwrap_err(),
            validate_closed_mesh(mesh, false, &decisions).unwrap_err(),
        ];
        let mut retained = 0;
        for (native, modeled) in native.into_iter().zip(modeled) {
            let expected = ValidationError::from(native);
            assert_eq!(expected.kind(), kind);
            assert_eq!(modeled.to_string(), expected.to_string());
            println!(
                "native={expected:?}; modeled={modeled:?}; source={}",
                modeled.source().is_some()
            );
            retained += usize::from(modeled == expected && modeled.source().is_some());
        }
        assert_eq!(
            retained, 2,
            "both validation helpers must preserve native causes"
        );
    }

    #[test]
    /// Checks native false results for invalid indices become explicit modeling geometry errors.
    fn invalid_indices_remain_a_definite_failed_output_validation() {
        let mesh = TriangleMesh::new(Vec::new(), vec![Triangle::new(0, 1, 2)]);
        let context = GeometryContext::STRICT;
        assert!(
            !mesh
                .has_unique_nondegenerate_triangles(&context.mesh_context())
                .unwrap()
                .value
        );
        assert!(
            !mesh
                .is_closed_manifold_geometry(&context.mesh_context())
                .unwrap()
                .value
        );
        let decisions = GeometryDecisions::new(context);
        for error in [
            validate_surface_mesh(mesh.clone(), &decisions).unwrap_err(),
            validate_closed_mesh(mesh, false, &decisions).unwrap_err(),
        ] {
            assert!(matches!(error, ValidationError::Geometry(_)));
        }
    }

    #[test]
    /// Checks a proved-flat tetrahedron and requires undecided native causes for a double-angle zero.
    fn undecided_geometry_keeps_the_native_mesh_cause() {
        let sine = Real::e().sin();
        let cosine = Real::e().cos();
        let tetrahedron = |height| {
            TriangleMesh::new(
                vec![
                    Point3::origin(),
                    Point3::new(1.into(), 0.into(), 0.into()),
                    Point3::new(0.into(), 1.into(), 0.into()),
                    Point3::new(0.into(), 0.into(), height),
                ],
                vec![
                    Triangle::new(0, 2, 1),
                    Triangle::new(0, 1, 3),
                    Triangle::new(0, 3, 2),
                    Triangle::new(1, 2, 3),
                ],
            )
        };
        // The scalar layer proves sin(a)^2 + cos(a)^2 = 1, so this flat
        // tetrahedron is certified degenerate.
        let pythagorean = &sine * &sine + &cosine * &cosine - Real::one();
        let flat = tetrahedron(pythagorean)
            .has_unique_nondegenerate_triangles(&GeometryContext::STRICT.mesh_context())
            .unwrap();
        assert!(!flat.value);
        // No retained relation proves the double-angle zero.
        let zero = (Real::from(2) * Real::e()).sin() - Real::from(2) * &sine * &cosine;
        check(tetrahedron(zero), ValidationErrorKind::Undecided);
    }
}
