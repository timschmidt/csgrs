//! Exact source fields and round-capped linear copper tracks.

use super::{CopperFeature, CopperKind, Sexp, exact_origin, midpoint_scalar, net_name};
use crate::geometry::{SourceScalar, SourceUnit, linear_stroke_region};
use crate::{LayerMetadata, PcbGeometryUncertainty, PcbRegion, Scalar};
use hypercurve::{CurveRegion2, OffsetCap};
use std::collections::HashMap;

pub(super) fn parse_segments(
    root: &Sexp,
    nets: &HashMap<i32, String>,
    copper: &mut Vec<CopperFeature>,
) {
    for segment in root.named_children("segment") {
        let source = crate::operations::evaluate("kicad-track-source", None, || {
            let start = point(segment, "start")?;
            let end = point(segment, "end")?;
            let width = field(segment, "width", 1)?;
            let width = number(width, 1, "width")?;
            Ok((start, end, width))
        });
        let location = source
            .as_ref()
            .map(|(start, end, _)| midpoint_scalar(start, end))
            .unwrap_or_else(|_| exact_origin());
        let native = source.and_then(|(start, end, width)| {
            linear_stroke_region(&start, &end, &width, OffsetCap::Round)
        });
        let metadata = Some(LayerMetadata {
            name: "KiCad segment".to_owned(),
        });
        let region = match native {
            Ok(native) => PcbRegion::new(native, metadata),
            Err(error) => PcbRegion::new(CurveRegion2::empty(), metadata)
                .with_exact_construction_error(error.to_string()),
        };
        if matches!(region.try_is_empty(), Ok(true)) {
            continue;
        }
        copper.push(CopperFeature {
            layer: segment
                .named_child("layer")
                .and_then(|layer| layer.atom_at(1))
                .unwrap_or("unknown")
                .to_owned(),
            net: net_name(segment, nets),
            kind: CopperKind::Segment,
            location,
            region,
        });
    }
}

fn field<'a>(
    parent: &'a Sexp,
    name: &str,
    count: usize,
) -> Result<&'a Sexp, PcbGeometryUncertainty> {
    let field = parent.named_child(name).ok_or_else(|| failure(name))?;
    if field.children().len() != count + 1 {
        return Err(failure(name));
    }
    Ok(field)
}

fn point(parent: &Sexp, name: &str) -> Result<[Scalar; 2], PcbGeometryUncertainty> {
    let field = field(parent, name, 2)?;
    Ok([number(field, 1, name)?, number(field, 2, name)?])
}

fn number(field: &Sexp, index: usize, name: &str) -> Result<Scalar, PcbGeometryUncertainty> {
    field
        .atom_at(index)
        .and_then(|token| SourceScalar::parse(SourceUnit::KiCadMillimeter, token))
        .map(|source| source.value)
        .ok_or_else(|| failure(name))
}

fn failure(field: &str) -> PcbGeometryUncertainty {
    crate::operations::failure(
        "kicad-track-source",
        None,
        format!("missing or invalid {field} field"),
    )
}
