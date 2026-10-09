//! Pad source tokens and board-frame placement, before report projection.

use super::{Sexp, chamfered_corners, custom_pad_polygons, exact_origin, rotate_translate_scalar};
use crate::geometry::{
    SourceScalar, SourceUnit, circle_region, polygons_to_profile, rectangle_region,
    rounded_chamfered_rect_region, trapezoid_region,
};
use crate::{LayerMetadata, PcbGeometryUncertainty, PcbRegion, Scalar};
use hypercurve::CurveRegion2;

type SourceResult<T> = Result<T, PcbGeometryUncertainty>;

#[derive(Clone)]
pub(super) struct Placement {
    pub center: [Scalar; 2],
    pub angle: Scalar,
}

impl Placement {
    pub fn parse(parent: &Sexp, field: &str) -> SourceResult<Self> {
        let Some(at) = parent.named_child("at") else {
            return Ok(Self {
                center: exact_origin(),
                angle: Scalar::zero(),
            });
        };
        if !(3..=4).contains(&at.children().len()) {
            return Err(failure(format!(
                "{field} requires two coordinates and an optional angle"
            )));
        }
        Ok(Self {
            center: [number(at, 1, field)?, number(at, 2, field)?],
            angle: if at.children().len() == 4 {
                number(at, 3, field)?
            } else {
                Scalar::zero()
            },
        })
    }

    pub fn in_footprint(&self, footprint: &Self) -> Self {
        // KiCad stores local pad coordinates but an absolute board-frame angle.
        Self {
            center: rotate_translate_scalar(&self.center, &footprint.center, &footprint.angle),
            angle: self.angle.clone(),
        }
    }
}

pub(super) fn regions(
    pad: &Sexp,
    shape: &str,
    placement: SourceResult<Placement>,
) -> Vec<PcbRegion> {
    let metadata = || {
        Some(LayerMetadata {
            name: "KiCad pad".into(),
        })
    };
    let native = crate::operations::evaluate("kicad-pad-source", None, || {
        let placement = placement?;
        // Native geometry uses Cartesian positive angles; KiCad uses positive
        // rotation toward negative Y in its board coordinate system.
        let angle = -&placement.angle;
        if shape == "custom" {
            let polygons = custom_pad_polygons(pad, [0.0, 0.0], 0.0);
            if !polygons.is_empty() {
                return polygons
                    .into_iter()
                    .map(|polygon| {
                        let source =
                            polygons_to_profile(vec![polygon], metadata()).try_into_native()?;
                        place_custom(source, &placement.center, &angle)
                    })
                    .collect();
            }
        }
        let center = &placement.center;
        let size = pair(pad, "size", None)?.map(|value| value.abs());
        match shape {
            "circle" => {
                let diameter = maximum(&size[0], &size[1])?;
                Ok(vec![circle_region(
                    center,
                    &crate::scalar::half(&diameter),
                )?])
            }
            "oval" => oval(center, &size, &angle),
            "trapezoid" => {
                let delta = pair(pad, "rect_delta", Some(exact_origin()))?;
                Ok(vec![trapezoid_region(center, &size, &delta, &angle)?])
            }
            "roundrect" => {
                let radius_ratio = ratio(
                    pad,
                    "roundrect_rratio",
                    crate::scalar::half(&crate::scalar::half(&Scalar::one())),
                )?;
                let chamfer_ratio = ratio(pad, "chamfer_ratio", Scalar::zero())?;
                let width = minimum(&size[0], &size[1])?;
                let selected = if chamfer_ratio == Scalar::zero() {
                    [false; 4]
                } else {
                    chamfered_corners(pad)
                };
                Ok(vec![rounded_chamfered_rect_region(
                    center,
                    &size,
                    &(&width * radius_ratio),
                    &(&width * chamfer_ratio),
                    selected,
                    &angle,
                )?])
            }
            "rect" | "custom" => Ok(vec![rectangle_region(center, &size, &angle)?]),
            _ => Err(failure(format!("unsupported KiCad pad shape {shape}"))),
        }
    });
    match native {
        Ok(regions) => regions
            .into_iter()
            .map(|native| PcbRegion::new(native, metadata()))
            .collect(),
        Err(error) => vec![
            PcbRegion::new(CurveRegion2::empty(), metadata())
                .with_exact_construction_error(error.to_string()),
        ],
    }
}

fn number(node: &Sexp, index: usize, field: &str) -> SourceResult<Scalar> {
    node.atom_at(index)
        .and_then(|token| SourceScalar::parse(SourceUnit::Unknown, token))
        .map(|source| source.value)
        .ok_or_else(|| {
            failure(format!(
                "KiCad {field} has a missing or invalid numeric token at position {index}"
            ))
        })
}

fn pair(parent: &Sexp, field: &str, default: Option<[Scalar; 2]>) -> SourceResult<[Scalar; 2]> {
    let Some(node) = parent.named_child(field) else {
        return default.ok_or_else(|| failure(format!("KiCad pad {field} is missing")));
    };
    if node.children().len() != 3 {
        return Err(failure(format!(
            "KiCad pad {field} requires two numeric tokens"
        )));
    }
    Ok([
        number(node, 1, &format!("pad {field}"))?,
        number(node, 2, &format!("pad {field}"))?,
    ])
}

fn ratio(parent: &Sexp, field: &str, default: Scalar) -> SourceResult<Scalar> {
    let Some(node) = parent.named_child(field) else {
        return Ok(default);
    };
    if node.children().len() != 2 {
        return Err(failure(format!(
            "KiCad pad {field} requires one numeric token"
        )));
    }
    let value = number(node, 1, field)?;
    maximum(
        &Scalar::zero(),
        &minimum(&value, &crate::scalar::half(&Scalar::one()))?,
    )
}

fn minimum(left: &Scalar, right: &Scalar) -> SourceResult<Scalar> {
    Ok(if crate::scalar::try_compare(left, right)?.is_le() {
        left.clone()
    } else {
        right.clone()
    })
}

fn maximum(left: &Scalar, right: &Scalar) -> SourceResult<Scalar> {
    Ok(if crate::scalar::try_compare(left, right)?.is_ge() {
        left.clone()
    } else {
        right.clone()
    })
}

fn oval(
    center: &[Scalar; 2],
    size: &[Scalar; 2],
    angle: &Scalar,
) -> SourceResult<Vec<CurveRegion2>> {
    if size.contains(&Scalar::zero()) {
        return Ok(vec![CurveRegion2::empty()]);
    }
    let horizontal = crate::scalar::try_compare(&size[0], &size[1])?.is_ge();
    let (length, width) = if horizontal {
        (&size[0], &size[1])
    } else {
        (&size[1], &size[0])
    };
    let radius = crate::scalar::half(width);
    let straight = length - width;
    if straight == Scalar::zero() {
        return Ok(vec![circle_region(center, &radius)?]);
    }
    let half_straight = crate::scalar::half(&straight);
    let endpoint = if horizontal {
        [half_straight, Scalar::zero()]
    } else {
        [Scalar::zero(), half_straight]
    };
    // The helper takes KiCad degrees; `angle` is the Cartesian native angle.
    let a = rotate_translate_scalar(&endpoint.clone().map(|value| -value), center, &(-angle));
    let b = rotate_translate_scalar(&endpoint, center, &(-angle));
    let body = if horizontal {
        [straight, width.clone()]
    } else {
        [width.clone(), straight]
    };
    Ok(vec![
        rectangle_region(center, &body, angle)?,
        circle_region(&a, &radius)?,
        circle_region(&b, &radius)?,
    ])
}

fn place_custom(
    region: CurveRegion2,
    center: &[Scalar; 2],
    angle: &Scalar,
) -> SourceResult<CurveRegion2> {
    let radians = (angle * Scalar::pi() / Scalar::from(180_u16))
        .expect("180 is a nonzero exact degree denominator");
    let cosine = radians.clone().cos();
    let sine = radians.sin();
    region
        .transform_affine(
            &cosine,
            &(-sine.clone()),
            &sine,
            &cosine,
            &center[0],
            &center[1],
        )
        .map_err(failure)
}

fn failure(detail: impl std::fmt::Display) -> PcbGeometryUncertainty {
    crate::operations::failure("kicad-pad-source", None, detail)
}
