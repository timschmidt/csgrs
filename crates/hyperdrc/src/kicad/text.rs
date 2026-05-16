//! Conservative KiCad text geometry helpers.
//!
//! KiCad text can be stroke-font or TrueType-backed depending on version and
//! project settings. Until glyph rendering is modeled, readiness checks use a
//! deterministic bounding box so copper/text presence is not silently dropped.

use geo::Polygon;

use crate::geometry::{rect_polygon, transform_polygon};
use crate::sexp::Sexp;

use super::xy_from_child;

pub(super) fn text_bbox_polygon(
    primitive: &Sexp,
    origin: [f64; 2],
    origin_angle_degrees: f64,
) -> Option<Polygon<f64>> {
    let text = text_value(primitive)?;
    if text.is_empty() {
        return None;
    }

    let text_at = xy_from_child(primitive, "at").unwrap_or([0.0, 0.0]);
    let text_angle = primitive
        .named_child("at")
        .and_then(|at| at.f64_at(3))
        .unwrap_or(0.0);
    let effects = primitive.named_child("effects");
    let font = effects.and_then(|effects| effects.named_child("font"));
    let text_height = font
        .and_then(|font| font.named_child("size"))
        .and_then(|size| size.f64_at(1))
        .unwrap_or(1.0)
        .abs();
    let text_width = font
        .and_then(|font| font.named_child("size"))
        .and_then(|size| size.f64_at(2))
        .unwrap_or(text_height)
        .abs();
    let thickness = font
        .and_then(|font| font.named_child("thickness"))
        .and_then(|thickness| thickness.f64_at(1))
        .unwrap_or(text_height * 0.15)
        .abs();

    let line_count = text.lines().count().max(1);
    let max_chars = text
        .lines()
        .map(|line| line.chars().count())
        .max()
        .unwrap_or_else(|| text.chars().count())
        .max(1);
    let bbox_width = text_width * max_chars as f64 * 0.65 + thickness;
    let bbox_height = text_height * line_count as f64 + thickness;
    if !(bbox_width.is_finite() && bbox_height.is_finite())
        || bbox_width <= 0.0
        || bbox_height <= 0.0
    {
        return None;
    }

    let local_center = justified_text_center(primitive, text_at, [bbox_width, bbox_height]);
    let local = rect_polygon(local_center, [bbox_width, bbox_height], text_angle);
    // This intentionally models a readability/coverage envelope rather than
    // glyph strokes. The conservative bounding-box treatment follows the
    // typographic measurement framing in Paterson and Tinker, "Studies of
    // Typographical Factors Influencing Speed of Reading. II. Size of Type",
    // Journal of Applied Psychology 13.2 (1929),
    // <https://doi.org/10.1037/h0074167>.
    let transformed = transform_polygon(&local, origin, origin_angle_degrees);
    log::trace!(
        "parsed KiCad text primitive: location=({:.3},{:.3}) chars={} bbox=({:.3},{:.3})",
        origin[0],
        origin[1],
        max_chars,
        bbox_width,
        bbox_height
    );
    Some(transformed)
}

fn text_value(primitive: &Sexp) -> Option<&str> {
    if primitive.list_name() == Some("fp_text") {
        primitive.atom_at(2).or_else(|| primitive.atom_at(1))
    } else {
        primitive.atom_at(1)
    }
}

fn justified_text_center(primitive: &Sexp, at: [f64; 2], size: [f64; 2]) -> [f64; 2] {
    let justify = primitive
        .named_child("effects")
        .and_then(|effects| effects.named_child("justify"));
    let horizontal = justify.and_then(|justify| {
        if justify
            .children()
            .iter()
            .any(|item| item.as_atom() == Some("left"))
        {
            Some(1.0)
        } else if justify
            .children()
            .iter()
            .any(|item| item.as_atom() == Some("right"))
        {
            Some(-1.0)
        } else {
            None
        }
    });
    let vertical = justify.and_then(|justify| {
        if justify
            .children()
            .iter()
            .any(|item| item.as_atom() == Some("top"))
        {
            Some(1.0)
        } else if justify
            .children()
            .iter()
            .any(|item| item.as_atom() == Some("bottom"))
        {
            Some(-1.0)
        } else {
            None
        }
    });

    [
        at[0] + horizontal.unwrap_or(0.0) * size[0] / 2.0,
        at[1] + vertical.unwrap_or(0.0) * size[1] / 2.0,
    ]
}
