//! Dimension annotations drawn on a blueprint view.
//!
//! A dimension is a drawing entity: two extension lines, one dimension line,
//! and an optional Hershey label. The measured length stays a `Real`. The
//! glyphs are a picture of that length and are not read back as geometry.

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
#[derive(Clone, Debug, PartialEq)]
pub struct Dimension {
    /// First measured point.
    pub start: Point3,
    /// Second measured point.
    pub end: Point3,
    /// Offset from the measured segment to the dimension line.
    pub offset: Vector3,
    /// Exact distance from `start` to `end`.
    pub length: Real,
}

/// Dimension between two points.
///
/// A zero length, or a length whose sign cannot be decided, returns `None`.
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
pub fn dimension_x(start: Point3, length: Real, offset: Vector3) -> Option<Dimension> {
    axis_dimension(start, length, offset, 0)
}

/// Dimension parallel to Y, of positive `length`, starting at `start`.
pub fn dimension_y(start: Point3, length: Real, offset: Vector3) -> Option<Dimension> {
    axis_dimension(start, length, offset, 1)
}

/// Dimension parallel to Z, of positive `length`, starting at `start`.
pub fn dimension_z(start: Point3, length: Real, offset: Vector3) -> Option<Dimension> {
    axis_dimension(start, length, offset, 2)
}

/// Appends extension lines, the dimension line, and a label to `view`.
pub fn annotate(view: &mut BlueprintView, dimension: &Dimension, handle: &str) {
    let start = dimension.start.clone();
    let end = dimension.end.clone();
    let offset_start = start.clone() + &dimension.offset;
    let offset_end = end.clone() + &dimension.offset;
    push_segment(view, handle, &start, &offset_start);
    push_segment(view, handle, &end, &offset_end);
    push_segment(view, handle, &offset_start, &offset_end);
    view.edges.extend(label_edges(
        view.projection,
        &offset_start,
        &offset_end,
        &dimension.length,
        handle,
    ));
}

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

fn is_positive(value: &Real) -> bool {
    hyperlimit::classify_real_sign(value, crate::PREDICATE_POLICY).value()
        == Some(hyperlimit::Sign::Positive)
}

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
fn label_edges(
    projection: BlueprintProjection,
    start: &Point3,
    end: &Point3,
    length: &Real,
    handle: &str,
) -> Vec<super::blueprint::BlueprintEdge> {
    let text = length_label(length);
    if text.is_empty() {
        return Vec::new();
    }
    let (start, _) = project_point(start, projection);
    let (end, _) = project_point(end, projection);
    let dx = &end.x - &start.x;
    let dy = &end.y - &start.y;
    let span = match (&dx * &dx + &dy * &dy).sqrt() {
        Ok(span) => span,
        Err(_) => return Vec::new(),
    };
    if !is_positive(&span) {
        return Vec::new();
    }
    let Ok(dir_x) = &dx / &span else {
        return Vec::new();
    };
    let Ok(dir_y) = &dy / &span else {
        return Vec::new();
    };
    let perp_x = Real::zero() - &dir_y;
    let perp_y = dir_x.clone();
    let two = Real::from(2);
    let Ok(mid_x) = (&start.x + &end.x) / &two else {
        return Vec::new();
    };
    let Ok(mid_y) = (&start.y + &end.y) / &two else {
        return Vec::new();
    };
    let strings = crate::curve::hershey_strings(
        &text,
        &crate::curve::hershey::fonts::FUTURAL,
        Real::one(),
    );
    let mut samples = Vec::new();
    for string in &strings {
        for segment in string.segments() {
            samples.push((segment.start().x().clone(), segment.start().y().clone()));
            samples.push((segment.end().x().clone(), segment.end().y().clone()));
        }
    }
    let Some((center_x, center_y)) = glyph_center(&samples) else {
        return Vec::new();
    };
    let gap = Real::from(2);
    let mut edges = Vec::new();
    for string in &strings {
        for segment in string.segments() {
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
    edges
}

#[cfg(not(feature = "hershey-text"))]
fn label_edges(
    _projection: BlueprintProjection,
    _start: &Point3,
    _end: &Point3,
    _length: &Real,
    _handle: &str,
) -> Vec<super::blueprint::BlueprintEdge> {
    Vec::new()
}

#[cfg(feature = "hershey-text")]
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
fn length_label(length: &Real) -> String {
    let Some(value) = length.to_f64_lossy() else {
        return String::new();
    };
    if value.is_finite() && value.fract() == 0.0 && value.abs() < 1.0e15 {
        format!("{}", value as i64)
    } else if value.is_finite() {
        format!("{value}")
    } else {
        String::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parts::blueprint::{BlueprintEdgeStyle, BlueprintProjection, BlueprintView};
    use crate::parts::scalar::real_eq;

    fn view() -> BlueprintView {
        BlueprintView {
            projection: BlueprintProjection::Front,
            exploded: false,
            edges: Vec::new(),
        }
    }

    fn offset_y(y: i64) -> Vector3 {
        Vector3::new([Real::zero(), Real::from(y), Real::zero()])
    }

    #[test]
    fn dimension_x_keeps_the_exact_length() {
        let annotation =
            dimension_x(Point3::origin(), Real::from(12), offset_y(-4)).expect("length");
        assert_eq!(real_eq(&annotation.length, &Real::from(12)), Some(true));
        let mut drawing = view();
        annotate(&mut drawing, &annotation, "block");
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
    fn zero_and_negative_lengths_are_rejected() {
        assert!(dimension_x(Point3::origin(), Real::zero(), offset_y(1)).is_none());
        assert!(dimension_y(Point3::origin(), Real::from(-2), offset_y(1)).is_none());
        assert!(dimension(Point3::origin(), Point3::origin(), offset_y(1)).is_none());
    }

    #[cfg(feature = "hershey-text")]
    #[test]
    fn label_adds_hershey_strokes() {
        let annotation = dimension_z(
            Point3::origin(),
            Real::from(8),
            Vector3::new([Real::from(3), Real::zero(), Real::zero()]),
        )
        .expect("length");
        let mut drawing = view();
        drawing.projection = BlueprintProjection::Right;
        annotate(&mut drawing, &annotation, "height");
        assert!(drawing.edges.len() > 3);
        assert!(drawing.edges.iter().any(|edge| {
            edge.evidence
                .notes
                .iter()
                .any(|note| note == "dimension label")
        }));
    }
}
