//! Swept path geometry.
//!
//! A swept line segment is the geometry used by both simple cutter-width
//! planning and PCB trace checking. The first implementation preserves exact
//! centerline facts and width facts; specialized axis-aligned clearance lives
//! in the PCB module while general curve offsets remain future `hypercurve`
//! work.

use hyperlimit::PredicatePolicy;
use hyperreal::{Real, RealExactSetFacts};

use crate::provenance::PathProvenance;
use crate::segment::LinePathSegment;

/// Cached facts for a swept line segment.
#[derive(Clone, Debug, PartialEq)]
pub struct SweptLineSegmentFacts {
    /// Exact-set facts across centerline endpoints and width.
    pub exact: RealExactSetFacts,
    /// Whether the width is structurally nonnegative.
    pub width_nonnegative: Option<bool>,
}

/// Centerline segment plus exact sweep width.
#[derive(Clone, Debug, PartialEq)]
pub struct SweptLineSegment {
    centerline: LinePathSegment,
    width: Real,
    facts: SweptLineSegmentFacts,
}

impl SweptLineSegment {
    /// Construct swept geometry when width is certified nonnegative.
    pub fn new(centerline: LinePathSegment, width: Real) -> Result<Self, &'static str> {
        let width_nonnegative = match width.structural_facts().sign {
            Some(hyperreal::RealSign::Negative) => Some(false),
            Some(hyperreal::RealSign::Zero | hyperreal::RealSign::Positive) => Some(true),
            None => None,
        };
        if width_nonnegative == Some(false) {
            return Err("swept path width must be nonnegative");
        }
        let coordinates = [
            &centerline.start().x,
            &centerline.start().y,
            &centerline.end().x,
            &centerline.end().y,
            &width,
        ];
        let facts = SweptLineSegmentFacts {
            exact: Real::exact_set_facts(coordinates),
            width_nonnegative,
        };
        Ok(Self {
            centerline,
            width,
            facts,
        })
    }

    /// Return centerline geometry.
    pub const fn centerline(&self) -> &LinePathSegment {
        &self.centerline
    }

    /// Return exact sweep width.
    pub const fn width(&self) -> &Real {
        &self.width
    }

    /// Return cached facts.
    pub const fn facts(&self) -> &SweptLineSegmentFacts {
        &self.facts
    }

    /// Return an exact centerline axis length if the segment is axis-aligned.
    pub fn axis_centerline_length(&self, policy: PredicatePolicy) -> Option<Real> {
        self.centerline.axis_length(policy)
    }

    /// Return provenance inherited from the centerline segment.
    pub fn provenance(&self) -> PathProvenance {
        self.centerline.provenance()
    }
}
