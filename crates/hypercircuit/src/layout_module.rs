//! Reusable hierarchical PCB layout modules over flattened circuit scopes.
//!
//! Modules retain circuit-local placements and copper intent. Composition uses
//! [`CircuitLibrary::flatten_with_scopes`] to remap electrical identities and
//! applies exact transforms before producing an ordinary validated
//! [`PcbLayout`]. Geometry remains owned by hyperpath/csgrs after this semantic
//! composition boundary.

use std::collections::{BTreeMap, BTreeSet};
use std::fmt::{Display, Formatter};

use hyperlattice::Point2;
use hyperpath::{ArcDirection, CubicBezier, ExplicitCircularArc, LinePathSegment};
use hyperreal::Real;

use crate::{
    BoardSide, Circuit, CircuitId, CircuitInstanceId, CircuitLibrary, DifferentialPairId,
    EscapePolicyId, FlattenedCircuitScope, HierarchyError, KeepoutId, LandPattern, LandPatternId,
    LayoutModuleId, LayoutValidationIssue, LengthTuningPatternId, LengthTuningSide, NetClassId,
    NetId, PcbDesignRules, PcbKeepout, PcbLayout, PcbPlacement, PcbRoute, PcbRouteSegment, PcbVia,
    PhaseTuningGroupId, PlacementConstraint, PlacementGroupId, PlacementResolutionIssue,
    RouteConstraintRegionId, RouteDirection, RouteId, RouteRuleRegionId, SubcircuitInstanceId,
    ViaId, ViaStyleId, ZoneId,
};

/// Exact rigid/mirrored transform from module-local to parent-board space.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct LayoutTransform {
    /// Parent-space origin of the local coordinate system.
    #[cfg_attr(feature = "interchange", serde(with = "crate::interchange::point"))]
    pub position: Point2,
    /// Counter-clockwise parent-space rotation in exact degrees.
    pub rotation_degrees: Real,
    /// Front preserves local X; back mirrors local X before rotation.
    pub side: BoardSide,
}

impl Default for LayoutTransform {
    fn default() -> Self {
        Self {
            position: Point2::new(Real::zero(), Real::zero()),
            rotation_degrees: Real::zero(),
            side: BoardSide::Front,
        }
    }
}

impl LayoutTransform {
    /// Applies this exact transform to one local point.
    pub fn transform_point(&self, point: &Point2) -> Point2 {
        let x = match self.side {
            BoardSide::Front => point.x.clone(),
            BoardSide::Back => -point.x.clone(),
        };
        let radians = self.rotation_degrees.clone().to_radians();
        let sin = radians.clone().sin();
        let cos = radians.cos();
        Point2::new(
            x.clone() * cos.clone() - point.y.clone() * sin.clone() + self.position.x.clone(),
            x * sin + point.y.clone() * cos + self.position.y.clone(),
        )
    }

    /// Composes a child-to-local transform after this local-to-parent transform.
    ///
    /// The result maps child coordinates directly into this transform's parent
    /// space, including exact back-side mirroring and rotation reversal.
    pub fn compose(&self, child: &Self) -> Self {
        let (rotation_degrees, side) =
            self.transform_orientation(&child.rotation_degrees, child.side);
        Self {
            position: self.transform_point(&child.position),
            rotation_degrees,
            side,
        }
    }

    fn transform_orientation(&self, rotation_degrees: &Real, side: BoardSide) -> (Real, BoardSide) {
        match self.side {
            BoardSide::Front => (
                self.rotation_degrees.clone() + rotation_degrees.clone(),
                side,
            ),
            BoardSide::Back => (
                self.rotation_degrees.clone() - rotation_degrees.clone(),
                flip_side(side),
            ),
        }
    }
}

/// Named group of local placements moved as one exact subassembly.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct PlacementGroup {
    /// Stable group identity within its layout module.
    pub id: PlacementGroupId,
    /// Local circuit instances transformed by the group.
    pub instances: Vec<CircuitInstanceId>,
    /// Group-local to module-local transform.
    pub transform: LayoutTransform,
}

/// Reusable PCB fragment bound to one reusable circuit definition.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct LayoutModule {
    /// Stable reusable layout identity.
    pub id: LayoutModuleId,
    /// Circuit definition whose local identities this module addresses.
    pub circuit: CircuitId,
    /// Module-local land patterns.
    pub land_patterns: Vec<LandPattern>,
    /// Module-local component placements.
    pub placements: Vec<PcbPlacement>,
    /// Module-local placement equations and predicates.
    #[cfg_attr(feature = "interchange", serde(default))]
    pub placement_constraints: Vec<PlacementConstraint>,
    /// Optional placement subassembly transforms.
    pub placement_groups: Vec<PlacementGroup>,
    /// Module-local routed copper.
    pub routes: Vec<PcbRoute>,
    /// Module-local vias.
    pub vias: Vec<PcbVia>,
    /// Module-local copper zones.
    pub zones: Vec<crate::CopperZone>,
    /// Module-local keepouts.
    pub keepouts: Vec<PcbKeepout>,
    /// Module-local rules remapped to flattened net identities.
    pub rules: PcbDesignRules,
}

/// Placement of one reusable layout module at a circuit hierarchy path.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct LayoutModuleInstance {
    /// Subcircuit-instance path from the root circuit.
    pub hierarchy_path: Vec<SubcircuitInstanceId>,
    /// Reusable layout module to compose at that path.
    pub module: LayoutModuleId,
    /// Module-local to board-space transform.
    pub transform: LayoutTransform,
}

/// Root board plus reusable module definitions and their hierarchy bindings.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct LayoutAssembly {
    /// Board-owned outline, stackup, root placements/features, and global rules.
    pub board: PcbLayout,
    /// Reusable layout definitions.
    pub modules: Vec<LayoutModule>,
    /// Module instances bound to elaborated circuit scopes.
    pub instances: Vec<LayoutModuleInstance>,
}

/// Local module defect found before composition.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum LayoutModuleValidationIssue {
    /// Module circuit definition is absent from the circuit library.
    UnknownCircuit(CircuitId),
    /// Local layout intent is structurally invalid against its circuit definition.
    InvalidLocalLayout(Vec<LayoutValidationIssue>),
    /// Two placement groups share one stable id.
    DuplicatePlacementGroup(PlacementGroupId),
    /// A group references no local placement for an instance.
    UnknownGroupedPlacement {
        group: PlacementGroupId,
        instance: CircuitInstanceId,
    },
    /// One local placement belongs to more than one transformed group.
    PlacementInMultipleGroups(CircuitInstanceId),
    /// Structurally valid local placement predicates are not satisfied.
    UnsatisfiedPlacementConstraints(Vec<PlacementResolutionIssue>),
}

/// Failure to elaborate reusable PCB intent into one flat board.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum LayoutCompositionError {
    /// Circuit hierarchy could not be flattened.
    Hierarchy(HierarchyError),
    /// Two module definitions share one identity.
    DuplicateModule(LayoutModuleId),
    /// One module failed local validation.
    InvalidModule {
        module: LayoutModuleId,
        issues: Vec<LayoutModuleValidationIssue>,
    },
    /// A module instance references an absent definition.
    UnknownModule(LayoutModuleId),
    /// No elaborated circuit scope exists at the authored hierarchy path.
    UnknownHierarchyPath(Vec<SubcircuitInstanceId>),
    /// The selected module belongs to another circuit definition.
    ModuleCircuitMismatch {
        module: LayoutModuleId,
        expected: CircuitId,
        actual: CircuitId,
    },
    /// More than one layout module instance targets the same circuit scope.
    DuplicateHierarchyPath(Vec<SubcircuitInstanceId>),
    /// A module-local route arc could not be reconstructed after transformation.
    RouteTransform(RouteId),
    /// Manhattan routing intent became non-axis-aligned after module transformation.
    RoutingConstraintTransform(String),
    /// The fully composed flat PCB failed ordinary layout validation.
    InvalidComposedLayout(Vec<LayoutValidationIssue>),
}

impl Display for LayoutCompositionError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Hierarchy(error) => write!(formatter, "circuit hierarchy failed: {error}"),
            Self::DuplicateModule(module) => {
                write!(formatter, "duplicate layout module {}", module.as_str())
            }
            Self::InvalidModule { module, issues } => write!(
                formatter,
                "layout module {} has {} validation issue(s)",
                module.as_str(),
                issues.len()
            ),
            Self::UnknownModule(module) => {
                write!(formatter, "unknown layout module {}", module.as_str())
            }
            Self::UnknownHierarchyPath(path) => {
                write!(
                    formatter,
                    "unknown layout hierarchy path {}",
                    path_text(path)
                )
            }
            Self::ModuleCircuitMismatch {
                module,
                expected,
                actual,
            } => write!(
                formatter,
                "layout module {} belongs to {}, not {}",
                module.as_str(),
                expected.as_str(),
                actual.as_str()
            ),
            Self::DuplicateHierarchyPath(path) => {
                write!(
                    formatter,
                    "duplicate layout hierarchy path {}",
                    path_text(path)
                )
            }
            Self::RouteTransform(route) => {
                write!(formatter, "failed to transform route {}", route.as_str())
            }
            Self::RoutingConstraintTransform(constraint) => write!(
                formatter,
                "failed to preserve Manhattan routing constraint {constraint} after transformation"
            ),
            Self::InvalidComposedLayout(issues) => write!(
                formatter,
                "composed layout has {} validation issue(s)",
                issues.len()
            ),
        }
    }
}

impl std::error::Error for LayoutCompositionError {}

/// Evidence for one composed reusable module instance.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ComposedLayoutModule {
    /// Reusable module definition.
    pub module: LayoutModuleId,
    /// Bound circuit hierarchy path.
    pub hierarchy_path: Vec<SubcircuitInstanceId>,
    /// Number of namespaced placements emitted.
    pub placements: usize,
    /// Number of retained module-local placement constraints resolved.
    pub placement_constraints: usize,
    /// Number of namespaced routes emitted.
    pub routes: usize,
    /// Number of namespaced vias emitted.
    pub vias: usize,
    /// Number of namespaced zones emitted.
    pub zones: usize,
    /// Number of namespaced keepouts emitted.
    pub keepouts: usize,
    /// Stable path-qualified placement-group identities retained as evidence.
    pub placement_groups: Vec<PlacementGroupId>,
}

/// Flat circuit/layout pair and deterministic module-composition evidence.
#[derive(Clone, Debug, PartialEq)]
pub struct LayoutCompositionReport {
    /// Flattened circuit carrying the identities referenced by `layout`.
    pub circuit: Circuit,
    /// Ordinary flat PCB layout ready for routing, DRC, materialization, and export.
    pub layout: PcbLayout,
    /// One evidence record per authored module instance.
    pub modules: Vec<ComposedLayoutModule>,
}

impl LayoutAssembly {
    /// Composes reusable module instances into one validated flat circuit/PCB pair.
    pub fn compose(
        &self,
        circuits: &CircuitLibrary,
    ) -> Result<LayoutCompositionReport, LayoutCompositionError> {
        let flattened = circuits
            .flatten_with_scopes()
            .map_err(LayoutCompositionError::Hierarchy)?;
        let definitions = circuits
            .circuits
            .iter()
            .map(|circuit| (circuit.id.clone(), circuit))
            .collect::<BTreeMap<_, _>>();
        let mut modules = BTreeMap::new();
        for module in &self.modules {
            if modules.insert(module.id.clone(), module).is_some() {
                return Err(LayoutCompositionError::DuplicateModule(module.id.clone()));
            }
            let issues = validate_module(module, &self.board, &definitions);
            if !issues.is_empty() {
                return Err(LayoutCompositionError::InvalidModule {
                    module: module.id.clone(),
                    issues,
                });
            }
        }

        let scopes = flattened
            .scopes
            .iter()
            .map(|scope| (scope.path.clone(), scope))
            .collect::<BTreeMap<_, _>>();
        let mut claimed_paths = BTreeSet::new();
        let mut layout = self.board.clone();
        let mut evidence = Vec::new();
        for instance in &self.instances {
            if !claimed_paths.insert(instance.hierarchy_path.clone()) {
                return Err(LayoutCompositionError::DuplicateHierarchyPath(
                    instance.hierarchy_path.clone(),
                ));
            }
            let module = modules
                .get(&instance.module)
                .copied()
                .ok_or_else(|| LayoutCompositionError::UnknownModule(instance.module.clone()))?;
            let scope = scopes
                .get(&instance.hierarchy_path)
                .copied()
                .ok_or_else(|| {
                    LayoutCompositionError::UnknownHierarchyPath(instance.hierarchy_path.clone())
                })?;
            if module.circuit != scope.circuit {
                return Err(LayoutCompositionError::ModuleCircuitMismatch {
                    module: module.id.clone(),
                    expected: module.circuit.clone(),
                    actual: scope.circuit.clone(),
                });
            }
            let circuit = definitions
                .get(&module.circuit)
                .copied()
                .expect("validated module circuit must exist");
            let placements = resolved_module_placements(module, &self.board, circuit);
            compose_module(&mut layout, module, &placements, instance, scope)?;
            evidence.push(ComposedLayoutModule {
                module: module.id.clone(),
                hierarchy_path: instance.hierarchy_path.clone(),
                placements: module.placements.len(),
                placement_constraints: module.placement_constraints.len(),
                routes: module.routes.len(),
                vias: module.vias.len(),
                zones: module.zones.len(),
                keepouts: module.keepouts.len(),
                placement_groups: module
                    .placement_groups
                    .iter()
                    .map(|group| {
                        namespaced_id::<PlacementGroupId>(
                            &path_text(&instance.hierarchy_path),
                            group.id.as_str(),
                        )
                    })
                    .collect(),
            });
        }
        let validation = layout.validate(&flattened.circuit);
        if !validation.is_valid() {
            return Err(LayoutCompositionError::InvalidComposedLayout(
                validation.issues,
            ));
        }
        Ok(LayoutCompositionReport {
            circuit: flattened.circuit,
            layout,
            modules: evidence,
        })
    }
}

fn validate_module(
    module: &LayoutModule,
    board: &PcbLayout,
    definitions: &BTreeMap<CircuitId, &Circuit>,
) -> Vec<LayoutModuleValidationIssue> {
    let Some(circuit) = definitions.get(&module.circuit).copied() else {
        return vec![LayoutModuleValidationIssue::UnknownCircuit(
            module.circuit.clone(),
        )];
    };
    let mut issues = Vec::new();
    let placements = module
        .placements
        .iter()
        .map(|placement| placement.instance.clone())
        .collect::<BTreeSet<_>>();
    let mut groups = BTreeSet::new();
    let mut grouped = BTreeSet::new();
    for group in &module.placement_groups {
        if !groups.insert(group.id.clone()) {
            issues.push(LayoutModuleValidationIssue::DuplicatePlacementGroup(
                group.id.clone(),
            ));
        }
        for instance in &group.instances {
            if !placements.contains(instance) {
                issues.push(LayoutModuleValidationIssue::UnknownGroupedPlacement {
                    group: group.id.clone(),
                    instance: instance.clone(),
                });
            }
            if !grouped.insert(instance.clone()) {
                issues.push(LayoutModuleValidationIssue::PlacementInMultipleGroups(
                    instance.clone(),
                ));
            }
        }
    }
    let placements = if issues.is_empty() {
        grouped_module_placements(module)
    } else {
        module.placements.clone()
    };
    let candidate = module_layout(module, board, placements);
    let local = candidate.validate(circuit);
    if !local.is_valid() {
        issues.push(LayoutModuleValidationIssue::InvalidLocalLayout(
            local.issues,
        ));
    } else {
        let resolution = candidate.resolve_placement_constraints(circuit);
        if !resolution.issues.is_empty() {
            issues.push(
                LayoutModuleValidationIssue::UnsatisfiedPlacementConstraints(resolution.issues),
            );
        }
    }
    issues
}

fn module_layout(
    module: &LayoutModule,
    board: &PcbLayout,
    placements: Vec<PcbPlacement>,
) -> PcbLayout {
    PcbLayout {
        id: board.id.clone(),
        outline: board.outline.clone(),
        stackup: board.stackup.clone(),
        land_patterns: module.land_patterns.clone(),
        placements,
        placement_constraints: module.placement_constraints.clone(),
        routes: module.routes.clone(),
        vias: module.vias.clone(),
        zones: module.zones.clone(),
        keepouts: module.keepouts.clone(),
        rules: module.rules.clone(),
    }
}

fn grouped_module_placements(module: &LayoutModule) -> Vec<PcbPlacement> {
    let group_by_instance = module
        .placement_groups
        .iter()
        .flat_map(|group| {
            group
                .instances
                .iter()
                .map(move |instance| (instance.clone(), &group.transform))
        })
        .collect::<BTreeMap<_, _>>();
    module
        .placements
        .iter()
        .map(|placement| {
            let mut placement = placement.clone();
            if let Some(group) = group_by_instance.get(&placement.instance) {
                placement.position = group.transform_point(&placement.position);
                (placement.rotation_degrees, placement.side) =
                    group.transform_orientation(&placement.rotation_degrees, placement.side);
            }
            placement
        })
        .collect()
}

fn resolved_module_placements(
    module: &LayoutModule,
    board: &PcbLayout,
    circuit: &Circuit,
) -> Vec<PcbPlacement> {
    let candidate = module_layout(module, board, grouped_module_placements(module));
    let resolution = candidate.resolve_placement_constraints(circuit);
    debug_assert!(resolution.issues.is_empty());
    resolution.placements
}

fn compose_module(
    output: &mut PcbLayout,
    module: &LayoutModule,
    local_placements: &[PcbPlacement],
    instance: &LayoutModuleInstance,
    scope: &FlattenedCircuitScope,
) -> Result<(), LayoutCompositionError> {
    let prefix = path_text(&instance.hierarchy_path);
    let pattern_map = module
        .land_patterns
        .iter()
        .map(|pattern| {
            let id = namespaced_id::<LandPatternId>(&prefix, pattern.id.as_str());
            let mut pattern = pattern.clone();
            let source = pattern.id.clone();
            pattern.id = id.clone();
            output.land_patterns.push(pattern);
            (source, id)
        })
        .collect::<BTreeMap<_, _>>();
    for placement in local_placements {
        let mut placement = placement.clone();
        placement.position = instance.transform.transform_point(&placement.position);
        (placement.rotation_degrees, placement.side) = instance
            .transform
            .transform_orientation(&placement.rotation_degrees, placement.side);
        placement.instance = scope
            .instances
            .get(&placement.instance)
            .expect("validated module placement must map to flattened instance")
            .clone();
        placement.land_pattern = pattern_map
            .get(&placement.land_pattern)
            .expect("validated module pattern must exist")
            .clone();
        output.placements.push(placement);
    }
    for route in &module.routes {
        let mut route = route.clone();
        route.id = namespaced_id::<RouteId>(&prefix, route.id.as_str());
        route.net = remap_net(scope, &route.net);
        route.segments = route
            .segments
            .iter()
            .map(|segment| transform_segment(segment, &instance.transform, &route.id))
            .collect::<Result<_, _>>()?;
        output.routes.push(route);
    }
    for via in &module.vias {
        let mut via = via.clone();
        via.id = namespaced_id::<ViaId>(&prefix, via.id.as_str());
        via.net = remap_net(scope, &via.net);
        via.center = instance.transform.transform_point(&via.center);
        output.vias.push(via);
    }
    for zone in &module.zones {
        let mut zone = zone.clone();
        zone.id = namespaced_id::<ZoneId>(&prefix, zone.id.as_str());
        zone.net = remap_net(scope, &zone.net);
        zone.boundary = zone
            .boundary
            .iter()
            .map(|point| instance.transform.transform_point(point))
            .collect();
        output.zones.push(zone);
    }
    for keepout in &module.keepouts {
        let mut keepout = keepout.clone();
        keepout.id = namespaced_id::<KeepoutId>(&prefix, keepout.id.as_str());
        keepout.boundary = keepout
            .boundary
            .iter()
            .map(|point| instance.transform.transform_point(point))
            .collect();
        output.keepouts.push(keepout);
    }
    for style in &module.rules.via_styles {
        let mut style = style.clone();
        style.id = namespaced_id::<ViaStyleId>(&prefix, style.id.as_str());
        output.rules.via_styles.push(style);
    }
    for class in &module.rules.net_classes {
        let mut class = class.clone();
        class.id = namespaced_id::<NetClassId>(&prefix, class.id.as_str());
        class.parent = class
            .parent
            .as_ref()
            .map(|parent| namespaced_id::<NetClassId>(&prefix, parent.as_str()));
        class.nets = class.nets.iter().map(|net| remap_net(scope, net)).collect();
        class.preferred_via_style = class
            .preferred_via_style
            .as_ref()
            .map(|style| namespaced_id::<ViaStyleId>(&prefix, style.as_str()));
        output.rules.net_classes.push(class);
    }
    for pair in &module.rules.differential_pairs {
        let mut pair = pair.clone();
        pair.id = namespaced_id::<DifferentialPairId>(&prefix, pair.id.as_str());
        pair.positive = remap_net(scope, &pair.positive);
        pair.negative = remap_net(scope, &pair.negative);
        output.rules.differential_pairs.push(pair);
    }
    for region in &module.rules.route_constraint_regions {
        let mut region = region.clone();
        region.id = namespaced_id::<RouteConstraintRegionId>(&prefix, region.id.as_str());
        region.boundary = region
            .boundary
            .iter()
            .map(|point| instance.transform.transform_point(point))
            .collect();
        region.nets = region
            .nets
            .iter()
            .map(|net| remap_net(scope, net))
            .collect();
        region.allowed_directions = transform_directions(
            &region.allowed_directions,
            &instance.transform,
            region.id.as_str(),
        )?;
        output.rules.route_constraint_regions.push(region);
    }
    for region in &module.rules.route_rule_regions {
        let mut region = region.clone();
        region.id = namespaced_id::<RouteRuleRegionId>(&prefix, region.id.as_str());
        region.boundary = region
            .boundary
            .iter()
            .map(|point| instance.transform.transform_point(point))
            .collect();
        region.nets = region
            .nets
            .iter()
            .map(|net| remap_net(scope, net))
            .collect();
        output.rules.route_rule_regions.push(region);
    }
    for policy in &module.rules.escape_policies {
        let mut policy = policy.clone();
        policy.id = namespaced_id::<EscapePolicyId>(&prefix, policy.id.as_str());
        policy.instances = policy
            .instances
            .iter()
            .map(|local| {
                scope
                    .instances
                    .get(local)
                    .expect("validated escape-policy instance must map to flattened scope")
                    .clone()
            })
            .collect();
        policy.nets = policy
            .nets
            .iter()
            .map(|net| remap_net(scope, net))
            .collect();
        policy.allowed_directions = transform_directions(
            &policy.allowed_directions,
            &instance.transform,
            policy.id.as_str(),
        )?;
        output.rules.escape_policies.push(policy);
    }
    for pattern in &module.rules.length_tuning_patterns {
        let mut pattern = pattern.clone();
        pattern.id = namespaced_id::<LengthTuningPatternId>(&prefix, pattern.id.as_str());
        pattern.net = remap_net(scope, &pattern.net);
        pattern.route = pattern
            .route
            .as_ref()
            .map(|route| namespaced_id::<RouteId>(&prefix, route.as_str()));
        pattern.region = pattern
            .region
            .iter()
            .map(|point| instance.transform.transform_point(point))
            .collect();
        if instance.transform.side == BoardSide::Back {
            pattern.side = match pattern.side {
                LengthTuningSide::Left => LengthTuningSide::Right,
                LengthTuningSide::Right => LengthTuningSide::Left,
            };
        }
        output.rules.length_tuning_patterns.push(pattern);
    }
    for group in &module.rules.phase_tuning_groups {
        let mut group = group.clone();
        group.id = namespaced_id::<PhaseTuningGroupId>(&prefix, group.id.as_str());
        group.patterns = group
            .patterns
            .iter()
            .map(|pattern| namespaced_id::<LengthTuningPatternId>(&prefix, pattern.as_str()))
            .collect();
        group.differential_pair = group
            .differential_pair
            .as_ref()
            .map(|pair| namespaced_id::<DifferentialPairId>(&prefix, pair.as_str()));
        output.rules.phase_tuning_groups.push(group);
    }
    Ok(())
}

fn transform_directions(
    directions: &[RouteDirection],
    transform: &LayoutTransform,
    constraint: &str,
) -> Result<Vec<RouteDirection>, LayoutCompositionError> {
    let origin = transform.transform_point(&Point2::new(Real::zero(), Real::zero()));
    let mut transformed = BTreeSet::new();
    for direction in directions {
        if *direction == RouteDirection::Arbitrary {
            transformed.insert(RouteDirection::Arbitrary);
            continue;
        }
        let axis = match direction {
            RouteDirection::Horizontal => Point2::new(Real::one(), Real::zero()),
            RouteDirection::Vertical => Point2::new(Real::zero(), Real::one()),
            RouteDirection::DiagonalRising => Point2::new(Real::one(), Real::one()),
            RouteDirection::DiagonalFalling => Point2::new(Real::one(), -Real::one()),
            RouteDirection::Arbitrary => unreachable!("arbitrary family handled above"),
        };
        let axis = transform.transform_point(&axis);
        let dx = axis.x - origin.x.clone();
        let dy = axis.y - origin.y.clone();
        let direction = if dy == Real::zero() && dx != Real::zero() {
            RouteDirection::Horizontal
        } else if dx == Real::zero() && dy != Real::zero() {
            RouteDirection::Vertical
        } else if dx.clone() * dx.clone() == dy.clone() * dy.clone() && dx != Real::zero() {
            if (dx > Real::zero()) == (dy > Real::zero()) {
                RouteDirection::DiagonalRising
            } else {
                RouteDirection::DiagonalFalling
            }
        } else {
            return Err(LayoutCompositionError::RoutingConstraintTransform(
                constraint.to_owned(),
            ));
        };
        transformed.insert(direction);
    }
    Ok(transformed.into_iter().collect())
}

fn transform_segment(
    segment: &PcbRouteSegment,
    transform: &LayoutTransform,
    route: &RouteId,
) -> Result<PcbRouteSegment, LayoutCompositionError> {
    Ok(match segment {
        PcbRouteSegment::Line(line) => PcbRouteSegment::Line(LinePathSegment::new(
            transform.transform_point(line.start()),
            transform.transform_point(line.end()),
        )),
        PcbRouteSegment::CircularArc(arc) => {
            let direction = match (transform.side, arc.direction()) {
                (BoardSide::Front, direction) => direction,
                (BoardSide::Back, ArcDirection::Ccw) => ArcDirection::Cw,
                (BoardSide::Back, ArcDirection::Cw) => ArcDirection::Ccw,
            };
            PcbRouteSegment::CircularArc(
                ExplicitCircularArc::new(
                    transform.transform_point(arc.center()),
                    arc.radius().clone(),
                    transform.transform_point(arc.start()),
                    transform.transform_point(arc.end()),
                    direction,
                )
                .map_err(|_| LayoutCompositionError::RouteTransform(route.clone()))?,
            )
        }
        PcbRouteSegment::CubicBezier(bezier) => PcbRouteSegment::CubicBezier(CubicBezier::new(
            transform.transform_point(bezier.start()),
            transform.transform_point(bezier.control0()),
            transform.transform_point(bezier.control1()),
            transform.transform_point(bezier.end()),
        )),
    })
}

fn remap_net(scope: &FlattenedCircuitScope, net: &NetId) -> NetId {
    scope
        .nets
        .get(net)
        .expect("validated module net must map to flattened scope")
        .clone()
}

fn path_text(path: &[SubcircuitInstanceId]) -> String {
    path.iter()
        .map(SubcircuitInstanceId::as_str)
        .collect::<Vec<_>>()
        .join("/")
}

fn namespaced_id<T>(path: &str, local: &str) -> T
where
    T: StableId,
{
    T::from_text(format!("{path}/{local}"))
}

trait StableId {
    fn from_text(text: String) -> Self;
}

macro_rules! stable_id {
    ($($identity:ty),+ $(,)?) => {
        $(
            impl StableId for $identity {
                fn from_text(text: String) -> Self {
                    <$identity>::new(text)
                        .expect("qualified nonempty stable ids remain nonempty")
                }
            }
        )+
    };
}

stable_id!(
    LandPatternId,
    RouteId,
    ViaId,
    ZoneId,
    KeepoutId,
    NetClassId,
    ViaStyleId,
    DifferentialPairId,
    PlacementGroupId,
    RouteConstraintRegionId,
    RouteRuleRegionId,
    EscapePolicyId,
    LengthTuningPatternId,
    PhaseTuningGroupId,
);

fn flip_side(side: BoardSide) -> BoardSide {
    match side {
        BoardSide::Front => BoardSide::Back,
        BoardSide::Back => BoardSide::Front,
    }
}
