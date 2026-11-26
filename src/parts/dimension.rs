//! Dimension annotations drawn on a blueprint view.
//!
//! A dimension is a drawing entity: two extension lines, one dimension line,
//! and an optional Hershey label. The measured length stays a `Real`. The
//! glyphs are a picture of that length and are not read back as geometry.
//! With `hershey-text`, the label uses a finite lossy binary64 display value;
//! without it, annotations contain only projected line segments. Drawing entities
//! do not establish visibility or certify the measured object's geometry.

use crate::errors::ValidationError;
use hyperlattice::{Point3, Real, Vector3};

#[cfg(feature = "hershey-text")]
use super::blueprint::ProjectedPoint2;
use super::blueprint::{
    BlueprintEdgeStyle, BlueprintOcclusionStatus, BlueprintProjection, BlueprintView, edge,
    project_point,
};
use super::scalar::real_eq;
#[cfg(feature = "hershey-text")]
use super::scalar::{real_gt, real_lt};

/// Measured segment plus the offset of its dimension line.
/// Constructors derive the Euclidean length; public fields permit inconsistent
/// authored values, and [`annotate`] does not revalidate that relation.
#[derive(Clone, Debug, PartialEq)]
pub struct Dimension {
    /// First measured point.
    pub start: Point3,
    /// Second measured point.
    pub end: Point3,
    /// Offset from the measured segment to the dimension line; need not be perpendicular.
    pub offset: Vector3,
    /// Exact distance from `start` to `end` when produced by the constructors.
    /// Annotation labels use this stored value rather than recomputing the distance.
    pub length: Real,
}

/// Dimension between two points.
///
/// Checks each displacement component with the crate's strict predicate policy.
/// Returns `None` for a zero displacement, any undecided component sign, or a
/// native square-root failure; those reasons are not distinguished. A single
/// nonzero component uses its absolute value, otherwise the length is `sqrt(d·d)`.
/// Stores the offset unchanged without checking its direction or magnitude.
pub fn dimension(start: Point3, end: Point3, offset: Vector3) -> Option<Dimension> {
    let length = measured_length(&(&end - &start))?;
    Some(Dimension {
        start,
        end,
        offset,
        length,
    })
}

/// Dimension parallel to X, of positive `length`, starting at `start`.
/// Returns `None` unless strict sign admission and [`dimension`] both succeed.
pub fn dimension_x(start: Point3, length: Real, offset: Vector3) -> Option<Dimension> {
    axis_dimension(start, length, offset, 0)
}

/// Dimension parallel to Y, of positive `length`, starting at `start`.
/// Returns `None` unless strict sign admission and [`dimension`] both succeed.
pub fn dimension_y(start: Point3, length: Real, offset: Vector3) -> Option<Dimension> {
    axis_dimension(start, length, offset, 1)
}

/// Dimension parallel to Z, of positive `length`, starting at `start`.
/// Returns `None` unless strict sign admission and [`dimension`] both succeed.
pub fn dimension_z(start: Point3, length: Real, offset: Vector3) -> Option<Dimension> {
    axis_dimension(start, length, offset, 2)
}

/// Appends projected extension/dimension segments and optional Hershey label strokes.
/// Proven zero-length projected segments are omitted. With `hershey-text`, the
/// label uses the stored length's finite lossy display value and is omitted for
/// a zero projected span or unavailable glyph center. Without that feature, no
/// label is attempted. Segments/strokes are drawing entities, not occlusion proofs.
///
/// Returned label-construction, scalar/predicate and checked buffer errors occur
/// before any edge is appended. This is not a bound on every allocation or scalar
/// operation. Does not validate the dimension's stored length against its endpoints.
pub fn annotate(
    view: &mut BlueprintView,
    dimension: &Dimension,
    handle: &str,
) -> Result<(), ValidationError> {
    let start = dimension.start.clone();
    let end = dimension.end.clone();
    let offset_start = start.clone() + &dimension.offset;
    let offset_end = end.clone() + &dimension.offset;
    let labels = label_edges(
        view.projection,
        &offset_start,
        &offset_end,
        &dimension.length,
        handle,
    )?;
    let additional = labels
        .len()
        .checked_add(3)
        .ok_or(ValidationError::CapacityOverflow)?;
    crate::solid::reserve_additional(&mut view.edges, additional)?;
    push_segment(view, handle, &start, &offset_start);
    push_segment(view, handle, &end, &offset_end);
    push_segment(view, handle, &offset_start, &offset_end);
    view.edges.extend(labels);
    Ok(())
}

/// Admits a strictly positive length, adds it to coordinate `axis`, and measures
/// the resulting segment. Internal callers must supply an axis in `0..3`;
/// an out-of-range axis panics after successful sign admission.
fn axis_dimension(
    start: Point3,
    length: Real,
    offset: Vector3,
    axis: usize,
) -> Option<Dimension> {
    if !is_positive(&length) {
        return None;
    }
    let mut coordinates = [start.x.clone(), start.y.clone(), start.z.clone()];
    coordinates[axis] = coordinates[axis].clone() + length;
    let end = Point3::new(
        coordinates[0].clone(),
        coordinates[1].clone(),
        coordinates[2].clone(),
    );
    dimension(start, end, offset)
}

/// Measures a nonzero displacement after strict component-sign decisions in XYZ order.
/// Uses absolute value for one nonzero component, otherwise the native norm;
/// zero, undecided signs and square-root errors all return `None`.
fn measured_length(delta: &Vector3) -> Option<Real> {
    let mut nonzero = Vec::new();
    for component in [&delta.0[0], &delta.0[1], &delta.0[2]] {
        match hyperlimit::classify_real_sign(component, crate::PREDICATE_POLICY).value() {
            Some(hyperlimit::Sign::Zero) => {},
            Some(_) => nonzero.push(component.clone()),
            None => return None,
        }
    }
    match nonzero.as_slice() {
        [] => None,
        [only] => Some(only.abs()),
        _ => delta.dot(delta).sqrt().ok(),
    }
}

/// Tests for a proved positive strict sign, mapping zero, negative and undecided to false.
fn is_positive(value: &Real) -> bool {
    hyperlimit::classify_real_sign(value, crate::PREDICATE_POLICY).value()
        == Some(hyperlimit::Sign::Positive)
}

/// Projects and appends a dimension drawing edge unless both coordinates prove equal.
/// An undecided equality does not suppress the edge. The caller reserves capacity;
/// this helper neither classifies occlusion nor checks dimension consistency.
fn push_segment(view: &mut BlueprintView, handle: &str, start: &Point3, end: &Point3) {
    let (projected_start, _) = project_point(start, view.projection);
    let (projected_end, _) = project_point(end, view.projection);
    if real_eq(&projected_start.x, &projected_end.x) == Some(true)
        && real_eq(&projected_start.y, &projected_end.y) == Some(true)
    {
        return;
    }
    view.edges.push(edge(
        handle,
        &projected_start,
        &projected_end,
        BlueprintEdgeStyle::Dimension,
        BlueprintOcclusionStatus::DrawingEntity,
        "dimension",
    ));
}

#[cfg(feature = "hershey-text")]
/// Builds unit-scale FUTURAL display strokes in the projected segment's frame.
/// Formats the length before checking projected span, centers glyph bounds at the
/// segment midpoint and offsets them by two units along the left perpendicular.
/// A zero span or missing glyph center yields no strokes; scalar, strict span-sign,
/// text-construction and checked vector reservation failures propagate as errors.
fn label_edges(
    projection: BlueprintProjection,
    start: &Point3,
    end: &Point3,
    length: &Real,
    handle: &str,
) -> Result<Vec<super::blueprint::BlueprintEdge>, ValidationError> {
    let text = length_label(length)?;
    let (start, _) = project_point(start, projection);
    let (end, _) = project_point(end, projection);
    let dx = &end.x - &start.x;
    let dy = &end.y - &start.y;
    let span = (&dx * &dx + &dy * &dy).sqrt()?;
    let decisions = crate::context::GeometryDecisions::new(crate::GeometryContext::STRICT);
    if decisions.sign(&span, "dimension label projected span")? == hyperreal::RealSign::Zero {
        // A dimension viewed along its axis has no direction for a label.
        return Ok(Vec::new());
    }
    let dir_x = (&dx / &span)?;
    let dir_y = (&dy / &span)?;
    let perp_x = Real::zero() - &dir_y;
    let perp_y = dir_x.clone();
    let two = Real::from(2);
    let mid_x = ((&start.x + &end.x) / &two)?;
    let mid_y = ((&start.y + &end.y) / &two)?;
    let strings = crate::curve::hershey_strings(
        &text,
        &crate::curve::hershey::fonts::FUTURAL,
        Real::one(),
    )?;
    let mut samples = Vec::new();
    for string in &strings {
        for segment in string.segments() {
            crate::solid::reserve_additional(&mut samples, 2)?;
            samples.push((segment.start().x().clone(), segment.start().y().clone()));
            samples.push((segment.end().x().clone(), segment.end().y().clone()));
        }
    }
    let Some((center_x, center_y)) = glyph_center(&samples) else {
        return Ok(Vec::new());
    };
    let gap = Real::from(2);
    let mut edges = Vec::new();
    for string in &strings {
        for segment in string.segments() {
            crate::solid::reserve_additional(&mut edges, 1)?;
            let local_start = place_glyph(
                &(segment.start().x().clone() - &center_x),
                &(segment.start().y().clone() - &center_y + &gap),
                &mid_x,
                &mid_y,
                &dir_x,
                &dir_y,
                &perp_x,
                &perp_y,
            );
            let local_end = place_glyph(
                &(segment.end().x().clone() - &center_x),
                &(segment.end().y().clone() - &center_y + &gap),
                &mid_x,
                &mid_y,
                &dir_x,
                &dir_y,
                &perp_x,
                &perp_y,
            );
            edges.push(edge(
                handle,
                &local_start,
                &local_end,
                BlueprintEdgeStyle::Dimension,
                BlueprintOcclusionStatus::DrawingEntity,
                "dimension label",
            ));
        }
    }
    Ok(edges)
}

#[cfg(not(feature = "hershey-text"))]
/// Supplies no label strokes when text support is disabled, without inspecting inputs.
const fn label_edges(
    _projection: BlueprintProjection,
    _start: &Point3,
    _end: &Point3,
    _length: &Real,
    _handle: &str,
) -> Result<Vec<super::blueprint::BlueprintEdge>, ValidationError> {
    Ok(Vec::new())
}

#[cfg(feature = "hershey-text")]
/// Returns the midpoint of coordinate extrema selected by strict comparisons.
/// Empty samples or failed division return `None`. Undecided comparisons leave
/// the previous extrema unchanged, so this is not a certified bounding-box query
/// for arbitrary scalar samples. The caller uses it only to place display glyphs.
fn glyph_center(samples: &[(Real, Real)]) -> Option<(Real, Real)> {
    let (mut min_x, mut min_y) = samples.first().cloned()?;
    let (mut max_x, mut max_y) = (min_x.clone(), min_y.clone());
    for (x, y) in samples.iter().skip(1) {
        if real_lt(x, &min_x) == Some(true) {
            min_x = x.clone();
        }
        if real_gt(x, &max_x) == Some(true) {
            max_x = x.clone();
        }
        if real_lt(y, &min_y) == Some(true) {
            min_y = y.clone();
        }
        if real_gt(y, &max_y) == Some(true) {
            max_y = y.clone();
        }
    }
    let two = Real::from(2);
    Some((
        ((&min_x + &max_x) / &two).ok()?,
        ((&min_y + &max_y) / &two).ok()?,
    ))
}

#[cfg(feature = "hershey-text")]
/// Maps glyph-local coordinates into a projected midpoint/direction/perpendicular frame.
/// Assumes the supplied axes have the normalization intended by the caller.
fn place_glyph(
    local_x: &Real,
    local_y: &Real,
    mid_x: &Real,
    mid_y: &Real,
    dir_x: &Real,
    dir_y: &Real,
    perp_x: &Real,
    perp_y: &Real,
) -> ProjectedPoint2 {
    ProjectedPoint2 {
        x: mid_x.clone() + dir_x.clone() * local_x.clone() + perp_x.clone() * local_y.clone(),
        y: mid_y.clone() + dir_y.clone() * local_x.clone() + perp_y.clone() * local_y.clone(),
    }
}

#[cfg(feature = "hershey-text")]
/// Formats a finite lossy binary64 length for display, without changing its exact source.
/// Integral values of magnitude below `1e15` use integer formatting; other finite
/// values use the float display format. Unavailable/nonfinite projections return
/// an unsupported-primitive error. No rounding-error bound or positivity check is added.
fn length_label(length: &Real) -> Result<String, ValidationError> {
    let unavailable = || ValidationError::UnsupportedPrimitive {
        reason: "dimension length has no finite binary64 display label",
    };
    let value = length
        .to_f64_lossy()
        .filter(|value| value.is_finite())
        .ok_or_else(unavailable)?;
    if value.is_finite() && value.fract() == 0.0 && value.abs() < 1.0e15 {
        Ok(format!("{}", value as i64))
    } else {
        Ok(format!("{value}"))
    }
}

#[cfg(test)]
/// Witnesses for native lengths, projected annotations, optional glyphs and error atomicity.
mod tests {
    use super::*;
    use crate::parts::blueprint::{BlueprintEdgeStyle, BlueprintProjection, BlueprintView};
    use crate::parts::scalar::real_eq;

    /// Creates an empty assembled front view for annotation assertions.
    fn view() -> BlueprintView {
        BlueprintView {
            projection: BlueprintProjection::Front,
            exploded: false,
            edges: Vec::new(),
        }
    }

    /// Constructs an exact integer displacement along Y.
    fn offset_y(y: i64) -> Vector3 {
        Vector3::new([Real::zero(), Real::from(y), Real::zero()])
    }

    #[test]
    /// Checks a 12-unit length, dimension styles and an offset horizontal edge;
    /// does not compare every glyph/extension endpoint or full serialization.
    fn dimension_x_keeps_the_exact_length() {
        let annotation =
            dimension_x(Point3::origin(), Real::from(12), offset_y(-4)).expect("length");
        assert_eq!(real_eq(&annotation.length, &Real::from(12)), Some(true));
        let mut drawing = view();
        annotate(&mut drawing, &annotation, "block").unwrap();
        assert!(drawing.edges.len() >= 3);
        assert!(
            drawing
                .edges
                .iter()
                .all(|edge| edge.style == BlueprintEdgeStyle::Dimension)
        );
        assert!(drawing.edges.iter().any(|edge| {
            real_eq(&edge.start.y, &Real::from(-4)) == Some(true)
                && real_eq(&edge.end.y, &Real::from(-4)) == Some(true)
        }));
    }

    #[test]
    /// Rejects zero X length, negative Y length and coincident measured endpoints.
    fn zero_and_negative_lengths_are_rejected() {
        assert!(dimension_x(Point3::origin(), Real::zero(), offset_y(1)).is_none());
        assert!(dimension_y(Point3::origin(), Real::from(-2), offset_y(1)).is_none());
        assert!(dimension(Point3::origin(), Point3::origin(), offset_y(1)).is_none());
    }

    #[cfg(feature = "hershey-text")]
    #[test]
    /// Checks that a right-view Z dimension adds edges carrying the label note.
    fn label_adds_hershey_strokes() {
        let annotation = dimension_z(
            Point3::origin(),
            Real::from(8),
            Vector3::new([Real::from(3), Real::zero(), Real::zero()]),
        )
        .expect("length");
        let mut drawing = view();
        drawing.projection = BlueprintProjection::Right;
        annotate(&mut drawing, &annotation, "height").unwrap();
        assert!(drawing.edges.len() > 3);
        assert!(drawing.edges.iter().any(|edge| {
            edge.evidence
                .notes
                .iter()
                .any(|note| note == "dimension label")
        }));
    }

    #[cfg(feature = "hershey-text")]
    #[test]
    /// Checks that an exact `2^2048` length's unavailable display label returns an
    /// unsupported error and preserves the previously annotated edge list exactly.
    fn failed_label_leaves_existing_drawing_unchanged() {
        let mut drawing = view();
        let annotation = dimension_x(Point3::origin(), Real::from(12), offset_y(-4)).unwrap();
        annotate(&mut drawing, &annotation, "existing").unwrap();
        let before = drawing.edges.clone();
        let annotation = dimension_x(
            Point3::origin(),
            Real::from(2).powi_i64(2048).unwrap(),
            offset_y(-4),
        )
        .unwrap();
        let error = annotate(&mut drawing, &annotation, "failed").unwrap_err();
        assert_eq!(error.kind(), crate::errors::ValidationErrorKind::Unsupported);
        assert_eq!(drawing.edges, before);
    }
}
