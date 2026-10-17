//! Checked exact source fields and native round-capped straight tracks.

use std::cmp::Ordering;
use std::collections::HashMap;
use std::fmt;

use hypercurve::{
    CurveFamily2, CurveOperation2, CurvePath2, CurveRegion2, ExactCurveError, LineSeg2, OffsetCap,
    OffsetCornerStyle2, Point2,
};
use hyperlimit::{PredicatePolicy, compare_reals};
use hypersolve::work_budget::{self, BudgetExceeded};

use super::{CopperFeature, CopperKind, midpoint_scalar, net_name};
use crate::geometry::{SourceScalar, SourceUnit};
use crate::sexp::Sexp;
use crate::{LayerMetadata, PcbGeometryUncertainty, PcbRegion, Scalar};

/// A straight track prevented the loader from publishing a complete board.
///
/// [`super::load_kicad_pcb`] retains this type inside its `anyhow::Error`, so
/// callers can use `downcast_ref::<TrackLoadError>()` through path context.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TrackLoadError {
    /// Zero-based index among the board's `segment` records.
    pub segment_index: usize,
    /// Source, representation, predicate, or resource failure.
    pub failure: TrackLoadFailure,
}

/// Why a straight track could not be loaded.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum TrackLoadFailure {
    /// A required source field is absent.
    MissingField {
        /// KiCad field name.
        field: &'static str,
    },
    /// A required source field occurs more than once.
    DuplicateField {
        /// KiCad field name.
        field: &'static str,
    },
    /// A source field has the wrong number of values.
    InvalidArity {
        /// KiCad field name.
        field: &'static str,
        /// Required number of values, excluding the field name.
        expected: usize,
        /// Observed number of values, excluding the field name.
        actual: usize,
    },
    /// A source value is not a supported exact numeric atom.
    InvalidNumber {
        /// KiCad field name.
        field: &'static str,
        /// Zero-based position among the field's values.
        value_index: usize,
    },
    /// The exact source width is negative.
    NegativeWidth,
    /// The exact width's sign could not be established under STRICT.
    UndecidedWidth,
    /// A shared work budget denied an operation.
    BudgetExceeded(BudgetExceeded),
    /// Valid source values did not produce the required nonempty track geometry.
    UnsupportedGeometry {
        /// Failed adapter requirement; this is not a source syntax error.
        detail: &'static str,
    },
    /// The native curve constructor retained an invalid-input or exact-blocker diagnosis.
    Construction(ExactCurveError),
    /// Native construction or validation did not produce usable geometry.
    Geometry(PcbGeometryUncertainty),
}

impl fmt::Display for TrackLoadError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "KiCad segment {}: ", self.segment_index + 1)?;
        match &self.failure {
            TrackLoadFailure::MissingField { field } => write!(f, "missing {field}"),
            TrackLoadFailure::DuplicateField { field } => write!(f, "duplicate {field}"),
            TrackLoadFailure::InvalidArity {
                field,
                expected,
                actual,
            } => {
                write!(f, "{field} requires {expected} values, found {actual}")
            }
            TrackLoadFailure::InvalidNumber { field, value_index } => {
                write!(f, "invalid numeric value {} in {field}", value_index + 1)
            }
            TrackLoadFailure::NegativeWidth => write!(f, "width must not be negative"),
            TrackLoadFailure::UndecidedWidth => write!(f, "width sign is undecided under STRICT"),
            TrackLoadFailure::BudgetExceeded(error) => write!(f, "{error}"),
            TrackLoadFailure::UnsupportedGeometry { detail } => {
                write!(f, "unsupported track geometry: {detail}")
            }
            TrackLoadFailure::Construction(error) => write!(f, "{error}"),
            TrackLoadFailure::Geometry(error) => write!(f, "{error}"),
        }
    }
}

impl std::error::Error for TrackLoadError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match &self.failure {
            TrackLoadFailure::BudgetExceeded(error) => Some(error),
            TrackLoadFailure::Construction(error) => Some(error),
            TrackLoadFailure::Geometry(error) => Some(error),
            _ => None,
        }
    }
}

fn check_budget() -> Result<(), TrackLoadFailure> {
    match work_budget::exhaustion() {
        Some(error) => Err(TrackLoadFailure::BudgetExceeded(error)),
        None => Ok(()),
    }
}

fn field<'a>(
    segment: &'a Sexp,
    name: &'static str,
    expected: usize,
) -> Result<&'a Sexp, TrackLoadFailure> {
    let mut fields = segment.named_children(name);
    let value = fields
        .next()
        .ok_or(TrackLoadFailure::MissingField { field: name })?;
    if fields.next().is_some() {
        return Err(TrackLoadFailure::DuplicateField { field: name });
    }
    let actual = value.children().len() - 1;
    if actual != expected {
        return Err(TrackLoadFailure::InvalidArity {
            field: name,
            expected,
            actual,
        });
    }
    Ok(value)
}

fn number(
    field: &Sexp,
    name: &'static str,
    index: usize,
) -> Result<SourceScalar, TrackLoadFailure> {
    field
        .atom_at(index + 1)
        .and_then(|token| SourceScalar::parse(SourceUnit::KiCadMillimeter, token))
        .ok_or(TrackLoadFailure::InvalidNumber {
            field: name,
            value_index: index,
        })
}

fn parse_segment(
    segment: &Sexp,
    nets: &HashMap<i32, String>,
) -> Result<Option<CopperFeature>, TrackLoadFailure> {
    check_budget()?;
    let start = field(segment, "start", 2)?;
    let start = [number(start, "start", 0)?, number(start, "start", 1)?];
    let end = field(segment, "end", 2)?;
    let end = [number(end, "end", 0)?, number(end, "end", 1)?];
    let width_field = field(segment, "width", 1)?;
    let width = number(width_field, "width", 0)?;
    let sign = compare_reals(&width.value, &Scalar::zero(), PredicatePolicy::STRICT).value();
    check_budget()?;
    match sign {
        Some(Ordering::Less) => return Err(TrackLoadFailure::NegativeWidth),
        // All required exact fields have been checked. No finite projection is
        // needed to establish the regularized empty set of a zero-width track.
        Some(Ordering::Equal) => return Ok(None),
        Some(Ordering::Greater) => {}
        None => return Err(TrackLoadFailure::UndecidedWidth),
    }
    let metadata = Some(LayerMetadata {
        name: "KiCad segment".to_string(),
    });
    // SourceScalar parses rationals. Equality here certifies source coincidence;
    // endpoints that merely round to the same binary64 pair are not a disk.
    let region = if start[0].value == end[0].value && start[1].value == end[1].value {
        let center = [start[0].value.clone(), start[1].value.clone()];
        let native = crate::geometry::circle_region(&center, &crate::scalar::half(&width.value));
        check_budget()?;
        PcbRegion::new(native.map_err(TrackLoadFailure::Geometry)?, metadata)
    } else {
        // The format defines a round-capped stroke of the exact source line.
        // STRICT suspends any ambient approximate or preview policy; retain
        // its receipt and structured construction error before publishing.
        let native = hypercurve::evaluate_under(PredicatePolicy::STRICT, || {
            let line = LineSeg2::try_new(
                Point2::new(start[0].value.clone(), start[1].value.clone()),
                Point2::new(end[0].value.clone(), end[1].value.clone()),
            )
            .map_err(|cause| ExactCurveError::Invalid {
                operation: CurveOperation2::Construction,
                family: Some(CurveFamily2::Line),
                cause,
            })?;
            let path = CurvePath2::try_new(vec![line.into()])?;
            CurveRegion2::stroke_path(
                &path,
                crate::scalar::half(&width.value),
                &OffsetCornerStyle2::Round,
                OffsetCap::Round,
            )
        });
        check_budget()?;
        let native = native.certified().ok_or_else(|| {
            TrackLoadFailure::Geometry(crate::operations::failure(
                "linear-stroke-construction",
                None,
                "missing STRICT receipt",
            ))
        })?;
        PcbRegion::new(native.map_err(TrackLoadFailure::Construction)?, metadata)
    };
    let empty = region.try_is_empty();
    check_budget()?;
    if empty.map_err(TrackLoadFailure::Geometry)? {
        return Err(TrackLoadFailure::UnsupportedGeometry {
            detail: "positive-width track produced empty geometry",
        });
    }
    let location = midpoint_scalar(
        &start.map(|value| value.value),
        &end.map(|value| value.value),
    );
    check_budget()?;
    Ok(Some(CopperFeature {
        layer: segment
            .named_child("layer")
            .and_then(|layer| layer.atom_at(1))
            .unwrap_or("unknown")
            .to_string(),
        net: net_name(segment, nets),
        kind: CopperKind::Segment,
        region,
        location,
    }))
}

pub(super) fn parse_segments(
    root: &Sexp,
    nets: &HashMap<i32, String>,
    copper: &mut Vec<CopperFeature>,
) -> Result<(), TrackLoadError> {
    for (segment_index, segment) in root.named_children("segment").enumerate() {
        if let Some(feature) = parse_segment(segment, nets).map_err(|failure| TrackLoadError {
            segment_index,
            failure,
        })? {
            copper.push(feature);
        }
    }
    Ok(())
}
