//! Typed PCB panel manufacturing intent.

use crate::predicate::RealPredicateExt as _;
use std::cmp::Ordering;
use std::collections::BTreeSet;

use hyperlattice::Point2;

use crate::{
    BoardId, BoardOutline, CoordinateFrame2, DesignForTestIntent, Real, RigidTransform2, TestAccess,
};

/// One child board instance retained inside a panel.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct PanelBoardInstance {
    pub id: String,
    pub board: BoardId,
    pub outline: BoardOutline,
    pub transform: RigidTransform2,
}

/// Panel rail or reserved web region.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct PanelRail {
    pub id: String,
    #[cfg_attr(feature = "interchange", serde(with = "crate::interchange::point"))]
    pub origin: Point2,
    pub width: Real,
    pub height: Real,
}

/// Tooling-hole manufacturing intent.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct PanelToolingHole {
    pub id: String,
    #[cfg_attr(feature = "interchange", serde(with = "crate::interchange::point"))]
    pub center: Point2,
    pub diameter: Real,
    pub plated: bool,
}

/// Global or child-local optical fiducial.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct PanelFiducial {
    pub id: String,
    pub child: Option<String>,
    #[cfg_attr(feature = "interchange", serde(with = "crate::interchange::point"))]
    pub center: Point2,
    pub copper_diameter: Real,
    pub clearance_diameter: Real,
}

/// Typed board-separation feature.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub enum PanelSeparationFeature {
    BreakawayTab {
        id: String,
        child: String,
        width: Real,
    },
    MouseBites {
        id: String,
        child: String,
        hole_diameter: Real,
        pitch: Real,
        count: usize,
    },
    VScore {
        id: String,
        start: [Real; 2],
        end: [Real; 2],
        remaining_thickness: Real,
    },
    RoutedPath {
        id: String,
        width: Real,
        points: Vec<[Real; 2]>,
    },
}

impl PanelSeparationFeature {
    fn id(&self) -> &str {
        match self {
            Self::BreakawayTab { id, .. }
            | Self::MouseBites { id, .. }
            | Self::VScore { id, .. }
            | Self::RoutedPath { id, .. } => id,
        }
    }
}

/// Process-monitor or impedance coupon.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct PanelCoupon {
    pub id: String,
    pub purpose: String,
    pub outline: BoardOutline,
    pub transform: RigidTransform2,
}

/// Marking region reserved on the panel or one child.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct PanelMarkingRegion {
    pub id: String,
    pub child: Option<String>,
    pub outline: BoardOutline,
}

/// Typed special edge-process requirement.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub enum PanelEdgeRequirement {
    EdgePlating {
        child: String,
        edge: String,
    },
    Bevel {
        child: String,
        edge: String,
        angle_degrees: Real,
        depth: Real,
    },
}

/// Complete retained panel definition.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct PanelDefinition {
    pub id: String,
    pub frame: CoordinateFrame2,
    pub outline: BoardOutline,
    pub thickness: Real,
    pub minimum_web: Real,
    pub minimum_rail: Real,
    pub children: Vec<PanelBoardInstance>,
    pub rails: Vec<PanelRail>,
    pub keepouts: Vec<BoardOutline>,
    pub tooling_holes: Vec<PanelToolingHole>,
    pub fiducials: Vec<PanelFiducial>,
    pub separation: Vec<PanelSeparationFeature>,
    pub coupons: Vec<PanelCoupon>,
    pub markings: Vec<PanelMarkingRegion>,
    pub edge_requirements: Vec<PanelEdgeRequirement>,
}

/// One child-qualified test-access claim transformed into panel coordinates.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct PanelTestAccess {
    pub child: String,
    pub access: TestAccess,
}

/// Structural panel-validation issue.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum PanelIssue {
    EmptyId,
    DuplicateId(String),
    InvalidFrame,
    NonPositiveDimension(String),
    CurvedOutlineRequiresCertifiedGeometry(String),
    IndeterminateGeometry(String),
    ChildOutsidePanel(String),
    ChildOverlap { left: String, right: String },
    ChildWebTooSmall { left: String, right: String },
    UnknownChild(String),
    BoardNotPresent(String),
}

#[derive(Clone, Debug)]
struct Bounds {
    min_x: Real,
    min_y: Real,
    max_x: Real,
    max_y: Real,
}

impl PanelDefinition {
    /// Returns child instances of one source board in stable authored order.
    pub fn children_for_board<'a>(
        &'a self,
        board: &'a BoardId,
    ) -> impl Iterator<Item = &'a PanelBoardInstance> {
        self.children
            .iter()
            .filter(move |child| &child.board == board)
    }

    /// Applies one child transform while retaining exact scalar coordinates.
    pub fn child_point_to_panel(
        &self,
        child: &str,
        point: [Real; 2],
    ) -> Result<[Real; 2], PanelIssue> {
        self.children
            .iter()
            .find(|candidate| candidate.id == child)
            .map(|candidate| candidate.transform.apply(point))
            .ok_or_else(|| PanelIssue::UnknownChild(child.into()))
    }

    /// Expands board-space DFT access into child-qualified panel-space claims.
    pub fn panelize_test_access(
        &self,
        board: &BoardId,
        intent: &DesignForTestIntent,
    ) -> Result<Vec<PanelTestAccess>, PanelIssue> {
        let children = self.children_for_board(board).collect::<Vec<_>>();
        if children.is_empty() {
            return Err(PanelIssue::BoardNotPresent(board.as_str().into()));
        }
        let mut output = Vec::new();
        for child in children {
            for access in &intent.access {
                let mut transformed = access.clone();
                transformed.id =
                    format!("panel:{}/child:{}/access:{}", self.id, child.id, access.id);
                transformed.position = access.position.as_ref().map(|position| {
                    let [x, y] = child
                        .transform
                        .apply([position.x.clone(), position.y.clone()]);
                    Point2::new(x, y)
                });
                transformed.frame = self.frame.clone();
                output.push(PanelTestAccess {
                    child: child.id.clone(),
                    access: transformed,
                });
            }
        }
        Ok(output)
    }

    /// Validates exact linear-outline containment, overlap, web, and identities.
    ///
    /// Curved outlines are retained losslessly but require the certified
    /// geometry path instead of being silently approximated for panel gating.
    pub fn validate(&self) -> Vec<PanelIssue> {
        let mut issues = Vec::new();
        if self.id.trim().is_empty() {
            issues.push(PanelIssue::EmptyId);
        }
        if self.frame.validate().is_err() {
            issues.push(PanelIssue::InvalidFrame);
        }
        for (name, value) in [
            ("thickness", &self.thickness),
            ("minimum_web", &self.minimum_web),
            ("minimum_rail", &self.minimum_rail),
        ] {
            match value.predicate_gt(&Real::zero()) {
                Some(true) => {}
                Some(false) => issues.push(PanelIssue::NonPositiveDimension(name.into())),
                None => issues.push(PanelIssue::IndeterminateGeometry(name.into())),
            }
        }
        let mut ids = BTreeSet::new();
        let named = self
            .children
            .iter()
            .map(|item| item.id.as_str())
            .chain(self.rails.iter().map(|item| item.id.as_str()))
            .chain(self.tooling_holes.iter().map(|item| item.id.as_str()))
            .chain(self.fiducials.iter().map(|item| item.id.as_str()))
            .chain(self.separation.iter().map(PanelSeparationFeature::id))
            .chain(self.coupons.iter().map(|item| item.id.as_str()))
            .chain(self.markings.iter().map(|item| item.id.as_str()));
        for id in named {
            if id.trim().is_empty() {
                issues.push(PanelIssue::EmptyId);
            } else if !ids.insert(id) {
                issues.push(PanelIssue::DuplicateId(id.into()));
            }
        }
        let child_ids = self
            .children
            .iter()
            .map(|child| child.id.as_str())
            .collect::<BTreeSet<_>>();
        for child in self
            .fiducials
            .iter()
            .filter_map(|item| item.child.as_deref())
            .chain(
                self.markings
                    .iter()
                    .filter_map(|item| item.child.as_deref()),
            )
        {
            if !child_ids.contains(child) {
                issues.push(PanelIssue::UnknownChild(child.into()));
            }
        }
        for feature in &self.separation {
            let child = match feature {
                PanelSeparationFeature::BreakawayTab { child, .. }
                | PanelSeparationFeature::MouseBites { child, .. } => Some(child),
                _ => None,
            };
            if let Some(child) = child
                && !child_ids.contains(child.as_str())
            {
                issues.push(PanelIssue::UnknownChild(child.clone()));
            }
        }
        let panel_bounds = match linear_bounds(&self.outline, None) {
            Ok(Some(bounds)) => bounds,
            Ok(None) => {
                issues.push(PanelIssue::CurvedOutlineRequiresCertifiedGeometry(
                    self.id.clone(),
                ));
                return issues;
            }
            Err(()) => {
                issues.push(PanelIssue::IndeterminateGeometry(self.id.clone()));
                return issues;
            }
        };
        let mut children = Vec::new();
        for child in &self.children {
            let bounds = match linear_bounds(&child.outline, Some(&child.transform)) {
                Ok(Some(bounds)) => bounds,
                Ok(None) => {
                    issues.push(PanelIssue::CurvedOutlineRequiresCertifiedGeometry(
                        child.id.clone(),
                    ));
                    continue;
                }
                Err(()) => {
                    issues.push(PanelIssue::IndeterminateGeometry(child.id.clone()));
                    continue;
                }
            };
            match contains(&panel_bounds, &bounds) {
                Some(true) => {}
                Some(false) => issues.push(PanelIssue::ChildOutsidePanel(child.id.clone())),
                None => issues.push(PanelIssue::IndeterminateGeometry(child.id.clone())),
            }
            children.push((child.id.as_str(), bounds));
        }
        for index in 0..children.len() {
            for right in children.iter().skip(index + 1) {
                let left = &children[index];
                match overlaps(&left.1, &right.1) {
                    Some(true) => issues.push(PanelIssue::ChildOverlap {
                        left: left.0.into(),
                        right: right.0.into(),
                    }),
                    Some(false) => match orthogonal_gap(&left.1, &right.1) {
                        Ok(Some(gap)) => match gap.predicate_lt(&self.minimum_web) {
                            Some(true) => issues.push(PanelIssue::ChildWebTooSmall {
                                left: left.0.into(),
                                right: right.0.into(),
                            }),
                            Some(false) => {}
                            None => issues.push(PanelIssue::IndeterminateGeometry(format!(
                                "{}:{}",
                                left.0, right.0
                            ))),
                        },
                        Ok(None) => {}
                        Err(()) => {
                            issues.push(PanelIssue::IndeterminateGeometry(format!(
                                "{}:{}",
                                left.0, right.0
                            )));
                        }
                    },
                    None => issues.push(PanelIssue::IndeterminateGeometry(format!(
                        "{}:{}",
                        left.0, right.0
                    ))),
                }
            }
        }
        for rail in &self.rails {
            match (
                rail.width.predicate_ge(&self.minimum_rail),
                rail.height.predicate_ge(&self.minimum_rail),
            ) {
                (Some(true), Some(true)) => {}
                (Some(_), Some(_)) => issues.push(PanelIssue::NonPositiveDimension(format!(
                    "rail:{} below minimum rail",
                    rail.id
                ))),
                _ => issues.push(PanelIssue::IndeterminateGeometry(format!(
                    "rail:{}",
                    rail.id
                ))),
            }
        }
        issues
    }

    /// Generates a lightweight human review SVG for linear retained outlines.
    pub fn to_svg(&self) -> Result<String, PanelIssue> {
        let panel =
            self.outline.exterior.linear_vertices().ok_or_else(|| {
                PanelIssue::CurvedOutlineRequiresCertifiedGeometry(self.id.clone())
            })?;
        let mut svg =
            String::from(r#"<svg xmlns="http://www.w3.org/2000/svg" fill="none" stroke="black">"#);
        svg.push_str(&svg_polygon(&panel, None));
        for child in &self.children {
            let points = child.outline.exterior.linear_vertices().ok_or_else(|| {
                PanelIssue::CurvedOutlineRequiresCertifiedGeometry(child.id.clone())
            })?;
            svg.push_str(&format!(
                r#"<g data-child="{}">{}</g>"#,
                child.id,
                svg_polygon(&points, Some(&child.transform))
            ));
        }
        svg.push_str("</svg>");
        Ok(svg)
    }
}

fn linear_bounds(
    outline: &BoardOutline,
    transform: Option<&RigidTransform2>,
) -> Result<Option<Bounds>, ()> {
    let Some(points) = outline.exterior.linear_vertices() else {
        return Ok(None);
    };
    let mut points = points.into_iter().map(|point| {
        let coordinates = [point.x, point.y];
        transform.map_or(coordinates.clone(), |transform| {
            transform.apply(coordinates)
        })
    });
    let Some(first) = points.next() else {
        return Ok(None);
    };
    let mut bounds = Bounds {
        min_x: first[0].clone(),
        min_y: first[1].clone(),
        max_x: first[0].clone(),
        max_y: first[1].clone(),
    };
    for [x, y] in points {
        match x.predicate_cmp(&bounds.min_x) {
            Some(Ordering::Less) => bounds.min_x = x.clone(),
            Some(_) => {}
            None => return Err(()),
        }
        match x.predicate_cmp(&bounds.max_x) {
            Some(Ordering::Greater) => bounds.max_x = x,
            Some(_) => {}
            None => return Err(()),
        }
        match y.predicate_cmp(&bounds.min_y) {
            Some(Ordering::Less) => bounds.min_y = y.clone(),
            Some(_) => {}
            None => return Err(()),
        }
        match y.predicate_cmp(&bounds.max_y) {
            Some(Ordering::Greater) => bounds.max_y = y,
            Some(_) => {}
            None => return Err(()),
        }
    }
    Ok(Some(bounds))
}

fn contains(outer: &Bounds, inner: &Bounds) -> Option<bool> {
    Some(
        outer.min_x.predicate_le(&inner.min_x)?
            && outer.min_y.predicate_le(&inner.min_y)?
            && outer.max_x.predicate_ge(&inner.max_x)?
            && outer.max_y.predicate_ge(&inner.max_y)?,
    )
}

fn overlaps(left: &Bounds, right: &Bounds) -> Option<bool> {
    Some(
        left.min_x.predicate_lt(&right.max_x)?
            && left.max_x.predicate_gt(&right.min_x)?
            && left.min_y.predicate_lt(&right.max_y)?
            && left.max_y.predicate_gt(&right.min_y)?,
    )
}

fn orthogonal_gap(left: &Bounds, right: &Bounds) -> Result<Option<Real>, ()> {
    let y_overlap = left.min_y.predicate_lt(&right.max_y).ok_or(())?
        && left.max_y.predicate_gt(&right.min_y).ok_or(())?;
    if y_overlap {
        return if left.max_x.predicate_le(&right.min_x).ok_or(())? {
            Ok(Some(right.min_x.clone() - left.max_x.clone()))
        } else if right.max_x.predicate_le(&left.min_x).ok_or(())? {
            Ok(Some(left.min_x.clone() - right.max_x.clone()))
        } else {
            Ok(None)
        };
    }
    let x_overlap = left.min_x.predicate_lt(&right.max_x).ok_or(())?
        && left.max_x.predicate_gt(&right.min_x).ok_or(())?;
    if x_overlap {
        return if left.max_y.predicate_le(&right.min_y).ok_or(())? {
            Ok(Some(right.min_y.clone() - left.max_y.clone()))
        } else if right.max_y.predicate_le(&left.min_y).ok_or(())? {
            Ok(Some(left.min_y.clone() - right.max_y.clone()))
        } else {
            Ok(None)
        };
    }
    Ok(None)
}

fn svg_polygon(points: &[Point2], transform: Option<&RigidTransform2>) -> String {
    let points = points
        .iter()
        .map(|point| {
            let coordinates = [point.x.clone(), point.y.clone()];
            let [x, y] = transform.map_or(coordinates.clone(), |value| value.apply(coordinates));
            format!("{x},{y}")
        })
        .collect::<Vec<_>>()
        .join(" ");
    format!(r#"<polygon points="{points}"/>"#)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn child(id: &str, x: i64) -> PanelBoardInstance {
        PanelBoardInstance {
            id: id.into(),
            board: BoardId::new(id).unwrap(),
            outline: BoardOutline::rectangle(Real::from(10), Real::from(10))
                .expect("integer rectangle coordinates are strictly orderable"),
            transform: RigidTransform2::new([Real::from(x), Real::from(5)], 0, false).unwrap(),
        }
    }

    #[test]
    fn validates_containment_overlap_and_minimum_web_exactly() {
        let mut panel = PanelDefinition {
            id: "p1".into(),
            frame: CoordinateFrame2::panel_default(),
            outline: BoardOutline::rectangle(Real::from(40), Real::from(20))
                .expect("integer rectangle coordinates are strictly orderable"),
            thickness: Real::from(2),
            minimum_web: Real::from(2),
            minimum_rail: Real::from(3),
            children: vec![child("a", 5), child("b", 16)],
            rails: Vec::new(),
            keepouts: Vec::new(),
            tooling_holes: Vec::new(),
            fiducials: Vec::new(),
            separation: Vec::new(),
            coupons: Vec::new(),
            markings: Vec::new(),
            edge_requirements: Vec::new(),
        };
        assert!(panel.validate().iter().any(|issue| matches!(
            issue,
            PanelIssue::ChildWebTooSmall { left, right } if left == "a" && right == "b"
        )));
        panel.children[1].transform.translation[0] = Real::from(14);
        assert!(
            panel
                .validate()
                .iter()
                .any(|issue| matches!(issue, PanelIssue::ChildOverlap { .. }))
        );
        assert!(panel.to_svg().unwrap().contains("data-child=\"a\""));
    }

    #[test]
    fn panelizes_access_with_child_identity_and_exact_mirrored_rotation() {
        let board = BoardId::new("board").unwrap();
        let panel = PanelDefinition {
            id: "p1".into(),
            frame: CoordinateFrame2::panel_default(),
            outline: BoardOutline::rectangle(Real::from(40), Real::from(40))
                .expect("integer rectangle coordinates are strictly orderable"),
            thickness: Real::from(2),
            minimum_web: Real::one(),
            minimum_rail: Real::from(3),
            children: vec![PanelBoardInstance {
                id: "unit-a".into(),
                board: board.clone(),
                outline: BoardOutline::rectangle(Real::from(10), Real::from(10))
                    .expect("integer rectangle coordinates are strictly orderable"),
                transform: RigidTransform2::new([Real::from(20), Real::from(20)], 1, true).unwrap(),
            }],
            rails: Vec::new(),
            keepouts: Vec::new(),
            tooling_holes: Vec::new(),
            fiducials: Vec::new(),
            separation: Vec::new(),
            coupons: Vec::new(),
            markings: Vec::new(),
            edge_requirements: Vec::new(),
        };
        let intent = DesignForTestIntent {
            access: vec![TestAccess {
                id: "TP1".into(),
                target: crate::TestTarget::Net {
                    circuit: crate::CircuitId::new("main").unwrap(),
                    net: crate::NetId::new("VCC").unwrap(),
                },
                method: crate::TestCoverageMethod::PhysicalAccess,
                position: Some(Point2::new(Real::from(2), Real::from(3))),
                frame: CoordinateFrame2::board_default(),
                side: crate::BoardSide::Front,
                reference: None,
                pad_or_pin: None,
                probe_diameter: Some(Real::one()),
            }],
            ..DesignForTestIntent::default()
        };
        let transformed = panel.panelize_test_access(&board, &intent).unwrap();
        assert_eq!(transformed[0].child, "unit-a");
        assert_eq!(transformed[0].access.id, "panel:p1/child:unit-a/access:TP1");
        let expected = panel
            .children
            .first()
            .unwrap()
            .transform
            .apply([Real::from(2), Real::from(3)]);
        let position = transformed[0].access.position.as_ref().unwrap();
        assert_eq!([position.x.clone(), position.y.clone()], expected);
        assert_eq!(transformed[0].access.frame, panel.frame);
    }
}
