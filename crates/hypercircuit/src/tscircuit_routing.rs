//! Audited `SimpleRouteJson` interoperability for ts-circuit-compatible routers.
//!
//! HyperCircuit remains authoritative for logical nets, physical layers, pad
//! ownership, rules, and process intent. This adapter projects a conservative
//! rectangular obstacle problem into the finite JSON protocol used by
//! `@tscircuit/capacity-autorouter`, then reconstructs returned wire/via paths
//! through HyperPath before they become semantic layout objects.

use std::{
    cmp::Ordering,
    collections::{BTreeMap, BTreeSet},
    fmt::{Display, Formatter},
    str::FromStr,
};

use hyperlattice::Point2;
use hyperpath::{
    LinePathSegment, PcbTrace, PcbViaStack, SpecctraRoute, SweptLineSegment, TraceLayer,
    ViaDrillIntent,
};
use hyperreal::Real;
use serde_json::{Map, Number, Value, json};

use crate::{
    BoardContour, BoardSide, CircuitInstanceId, KeepoutId, KeepoutScope, NetId, PadId, PadShape,
    PcbLayout, PcbRouteSegment, Plating, RouteId, RoutingProblem, RoutingProblemReport,
    RoutingSolution, ZoneId,
};

/// Finite JSON projection policy for a ts-circuit routing problem.
#[derive(Clone, Debug, PartialEq)]
pub struct TscircuitRoutingExportOptions {
    /// Maximum decimal places written for every exact scalar.
    pub decimal_places: usize,
    /// Explicit global width when retained net classes do not provide one.
    pub fallback_min_trace_width: Option<Real>,
}

impl Default for TscircuitRoutingExportOptions {
    fn default() -> Self {
        Self {
            decimal_places: 6,
            fallback_min_trace_width: None,
        }
    }
}

/// One exact-to-finite scalar projection at the routing protocol boundary.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TscircuitRoutingProjection {
    /// Stable semantic source field.
    pub field: String,
    /// Exact retained value or exact source expression.
    pub exact: String,
    /// Decimal JSON token sent to the external router.
    pub emitted: String,
}

/// Retained routing facts conservatively lowered to `SimpleRouteJson`.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum TscircuitRoutingOmission {
    /// A non-rectangular board is represented only by its exact exterior bounds.
    BoardBoundaryReducedToBounds,
    /// A non-rectangular cutout became its conservative bounding rectangle.
    CutoutBoxed { index: usize },
    /// A non-rectangular keepout became its conservative bounding rectangle.
    KeepoutBoxed(KeepoutId),
    /// A via-only or component-only keepout was widened to all routing layers.
    KeepoutScopeWidened(KeepoutId),
    /// A non-rectangular pad became a conservative rectangular obstacle.
    PadShapeBoxed {
        instance: CircuitInstanceId,
        pad: PadId,
    },
    /// A non-rectangular copper zone became a conservative rectangular obstacle.
    CopperZoneBoxed(ZoneId),
    /// A curved retained route became a conservative bounding obstacle.
    CurvedRouteBoxed(RouteId),
    /// Distinct retained per-net widths became the protocol's one global minimum.
    PerNetWidthsCollapsed { distinct_widths: usize },
    /// A plated terminal spanning multiple layers used one protocol endpoint layer.
    MultilayerTerminalCollapsed {
        instance: CircuitInstanceId,
        pad: PadId,
        layers: usize,
    },
}

/// A ts-circuit routing problem plus complete projection/loss evidence.
#[derive(Clone, Debug, PartialEq)]
pub struct TscircuitSimpleRouteJsonReport {
    /// Pretty-printed protocol document.
    pub json: String,
    /// Every exact scalar projected to a finite JSON number.
    pub projections: Vec<TscircuitRoutingProjection>,
    /// Conservative semantic reductions required by the protocol.
    pub omissions: Vec<TscircuitRoutingOmission>,
}

/// Process policy absent from returned `SimpleRouteJson` via records.
#[derive(Clone, Debug, PartialEq)]
pub struct TscircuitRoutingImportOptions {
    /// Copper land diameter assigned to every returned via.
    pub via_land_diameter: Real,
    /// Finished drill diameter assigned to every returned via.
    pub via_drill_diameter: Real,
    /// Plating intent assigned to every returned via.
    pub via_plating: Plating,
}

/// One finite JSON number reconstructed as an exact decimal.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TscircuitRoutingNumericImport {
    /// JSON field path.
    pub field: String,
    /// Source JSON numeric token.
    pub source: String,
    /// Exact decimal retained by HyperCircuit.
    pub exact: String,
}

/// Returned ts-circuit candidate mapped through HyperPath to semantic routes.
#[derive(Clone, Debug, PartialEq)]
pub struct TscircuitRoutingImportReport {
    /// Circuit-owned route and via objects.
    pub solution: RoutingSolution,
    /// Every imported finite coordinate, width, and via position.
    pub numeric_imports: Vec<TscircuitRoutingNumericImport>,
}

/// Failure at the finite `SimpleRouteJson` adapter boundary.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum TscircuitRoutingError {
    /// Projection precision or fallback width policy is invalid.
    InvalidOptions,
    /// No retained or explicitly supplied minimum trace width exists.
    MissingMinimumTraceWidth,
    /// An exact value has no finite decimal projection.
    NonFinite(String),
    /// Retained geometry cannot produce a conservative protocol obstacle.
    InvalidGeometry(String),
    /// JSON syntax or shape is invalid.
    InvalidJson(String),
    /// A required protocol field is absent or has the wrong type.
    InvalidField(String),
    /// A returned trace id is not a logical net name from the exported problem.
    UnknownNet(String),
    /// A returned layer name is not a retained physical conductor layer.
    UnknownLayer(String),
    /// A returned route sequence is disconnected or structurally invalid.
    InvalidRoute(String),
    /// HyperPath rejected returned wire or via geometry.
    Hyperpath(String),
    /// The HyperPath candidate could not be restored to semantic identities.
    Semantic(String),
}

impl Display for TscircuitRoutingError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidOptions => formatter.write_str("invalid ts-circuit routing options"),
            Self::MissingMinimumTraceWidth => {
                formatter.write_str("ts-circuit routing requires an explicit minimum width")
            }
            Self::NonFinite(field) => write!(formatter, "non-finite routing field: {field}"),
            Self::InvalidGeometry(source) => {
                write!(formatter, "invalid routing geometry: {source}")
            }
            Self::InvalidJson(error) => write!(formatter, "invalid SimpleRouteJson: {error}"),
            Self::InvalidField(field) => {
                write!(formatter, "invalid SimpleRouteJson field: {field}")
            }
            Self::UnknownNet(net) => write!(formatter, "unknown returned routing net: {net}"),
            Self::UnknownLayer(layer) => write!(formatter, "unknown returned layer: {layer}"),
            Self::InvalidRoute(route) => write!(formatter, "invalid returned route: {route}"),
            Self::Hyperpath(error) => write!(formatter, "HyperPath rejected route: {error}"),
            Self::Semantic(error) => {
                write!(formatter, "semantic route restoration failed: {error}")
            }
        }
    }
}

impl std::error::Error for TscircuitRoutingError {}

impl RoutingProblemReport {
    /// Projects this retained problem and its source layout to `SimpleRouteJson`.
    pub fn export_tscircuit_simple_route_json(
        &self,
        layout: &PcbLayout,
        options: TscircuitRoutingExportOptions,
    ) -> Result<TscircuitSimpleRouteJsonReport, TscircuitRoutingError> {
        if options.decimal_places > 15
            || options
                .fallback_min_trace_width
                .as_ref()
                .is_some_and(|width| !positive(width))
        {
            return Err(TscircuitRoutingError::InvalidOptions);
        }
        let mut emitter = JsonEmitter::new(options.decimal_places);
        let mut omissions = Vec::new();
        let all_layers = self
            .problem
            .layer_aliases
            .iter()
            .map(|layer| Value::String(layer.name.clone()))
            .collect::<Vec<_>>();
        if all_layers.is_empty() {
            return Err(TscircuitRoutingError::InvalidGeometry(
                "board has no conductor layers".into(),
            ));
        }

        let boundary = layout
            .outline
            .boundary_geometry()
            .map_err(|error| TscircuitRoutingError::InvalidGeometry(error.to_string()))?;
        let (bounds_min, bounds_max) = boundary.exterior_bounds();
        if !is_axis_aligned_rectangle(&layout.outline.exterior) {
            omissions.push(TscircuitRoutingOmission::BoardBoundaryReducedToBounds);
        }
        let bounds = json!({
            "minX": emitter.real("bounds.minX", &bounds_min.x)?,
            "maxX": emitter.real("bounds.maxX", &bounds_max.x)?,
            "minY": emitter.real("bounds.minY", &bounds_min.y)?,
            "maxY": emitter.real("bounds.maxY", &bounds_max.y)?,
        });

        let min_trace_width =
            global_minimum_width(&self.problem, options.fallback_min_trace_width.as_ref())?;
        let mut distinct_widths = Vec::<Real>::new();
        for width in self
            .problem
            .rules
            .iter()
            .map(|rule| &rule.width)
            .chain(options.fallback_min_trace_width.iter())
        {
            if !distinct_widths.iter().any(|known| known == width) {
                distinct_widths.push(width.clone());
            }
        }
        if distinct_widths.len() > 1 {
            omissions.push(TscircuitRoutingOmission::PerNetWidthsCollapsed {
                distinct_widths: distinct_widths.len(),
            });
        }

        let connections = connection_json(&self.problem, &mut emitter, &mut omissions)?;
        let mut obstacles = Vec::new();
        append_pad_obstacles(
            &self.problem,
            layout,
            &mut emitter,
            &mut omissions,
            &mut obstacles,
        )?;
        append_cutout_obstacles(
            layout,
            &all_layers,
            &mut emitter,
            &mut omissions,
            &mut obstacles,
        )?;
        append_keepout_obstacles(
            &self.problem,
            layout,
            &all_layers,
            &mut emitter,
            &mut omissions,
            &mut obstacles,
        )?;
        append_zone_obstacles(
            &self.problem,
            layout,
            &mut emitter,
            &mut omissions,
            &mut obstacles,
        )?;
        append_existing_copper_obstacles(
            &self.problem,
            layout,
            &mut emitter,
            &mut omissions,
            &mut obstacles,
        )?;

        let document = json!({
            "layerCount": self.problem.layer_aliases.len(),
            "minTraceWidth": emitter.real("minTraceWidth", &min_trace_width)?,
            "obstacles": obstacles,
            "connections": connections,
            "bounds": bounds,
        });
        let json = serde_json::to_string_pretty(&document)
            .map_err(|error| TscircuitRoutingError::InvalidJson(error.to_string()))?;
        Ok(TscircuitSimpleRouteJsonReport {
            json,
            projections: emitter.projections,
            omissions,
        })
    }
}

impl TscircuitRoutingImportReport {
    /// Restores a ts-circuit-compatible router result through HyperPath.
    pub fn from_str(
        problem: &RoutingProblem,
        source: &str,
        options: TscircuitRoutingImportOptions,
    ) -> Result<Self, TscircuitRoutingError> {
        if !positive(&options.via_land_diameter)
            || !positive(&options.via_drill_diameter)
            || options.via_drill_diameter > options.via_land_diameter
        {
            return Err(TscircuitRoutingError::InvalidOptions);
        }
        let root: Value = serde_json::from_str(source)
            .map_err(|error| TscircuitRoutingError::InvalidJson(error.to_string()))?;
        let traces = root
            .get("traces")
            .and_then(Value::as_array)
            .ok_or_else(|| TscircuitRoutingError::InvalidField("traces".into()))?;
        let mut numeric_imports = Vec::new();
        let mut route_geometry = Vec::new();
        let mut vias = Vec::new();
        for (trace_index, trace) in traces.iter().enumerate() {
            if trace.get("type").and_then(Value::as_str) != Some("pcb_trace") {
                return Err(TscircuitRoutingError::InvalidField(format!(
                    "traces[{trace_index}].type"
                )));
            }
            let trace_id = string_field(trace, "pcb_trace_id", &format!("traces[{trace_index}]"))?;
            let logical = problem
                .aliases
                .iter()
                .find_map(|(logical, _)| (logical.as_str() == trace_id).then_some(logical))
                .ok_or_else(|| TscircuitRoutingError::UnknownNet(trace_id.into()))?;
            let routing_net = problem
                .aliases
                .get(logical)
                .ok_or_else(|| TscircuitRoutingError::UnknownNet(trace_id.into()))?;
            let route = trace
                .get("route")
                .and_then(Value::as_array)
                .ok_or_else(|| {
                    TscircuitRoutingError::InvalidField(format!("traces[{trace_index}].route"))
                })?;
            let mut previous: Option<WirePoint> = None;
            let mut pending_via: Option<(Point2, TraceLayer)> = None;
            for (point_index, point) in route.iter().enumerate() {
                let field = format!("traces[{trace_index}].route[{point_index}]");
                match point.get("route_type").and_then(Value::as_str) {
                    Some("wire") => {
                        let wire = import_wire_point(problem, point, &field, &mut numeric_imports)?;
                        if let Some((center, expected_layer)) = pending_via.take()
                            && (wire.point != center || wire.layer != expected_layer)
                        {
                            return Err(TscircuitRoutingError::InvalidRoute(format!(
                                "{field} does not continue the preceding via"
                            )));
                        }
                        if let Some(prior) = previous.take() {
                            if prior.layer != wire.layer {
                                return Err(TscircuitRoutingError::InvalidRoute(format!(
                                    "{field} changes layer without a via"
                                )));
                            }
                            if prior.point != wire.point {
                                let centerline =
                                    LinePathSegment::new(prior.point, wire.point.clone());
                                let swept = SweptLineSegment::new(centerline, wire.width.clone())
                                    .map_err(|error| {
                                    TscircuitRoutingError::Hyperpath(error.into())
                                })?;
                                route_geometry.push(PcbTrace::new(routing_net, wire.layer, swept));
                            }
                        }
                        previous = Some(wire);
                    }
                    Some("via") => {
                        let center = import_point(point, &field, &mut numeric_imports)?;
                        let from_name = string_field(point, "from_layer", &field)?;
                        let to_name = string_field(point, "to_layer", &field)?;
                        let from_layer = layer_by_name(problem, from_name)?;
                        let to_layer = layer_by_name(problem, to_name)?;
                        if let Some(prior) = previous.take()
                            && (prior.point != center || prior.layer != from_layer)
                        {
                            return Err(TscircuitRoutingError::InvalidRoute(format!(
                                "{field} is disconnected from its preceding wire"
                            )));
                        }
                        let (start_layer, end_layer) = if from_layer <= to_layer {
                            (from_layer, to_layer)
                        } else {
                            (to_layer, from_layer)
                        };
                        vias.push(
                            PcbViaStack::with_drill_intent(
                                routing_net,
                                start_layer,
                                end_layer,
                                center.clone(),
                                options.via_land_diameter.clone(),
                                options.via_drill_diameter.clone(),
                                drill_intent(options.via_plating),
                            )
                            .map_err(|error| TscircuitRoutingError::Hyperpath(error.into()))?,
                        );
                        pending_via = Some((center, to_layer));
                    }
                    _ => {
                        return Err(TscircuitRoutingError::InvalidField(format!(
                            "{field}.route_type"
                        )));
                    }
                }
            }
        }
        let candidate = SpecctraRoute::with_vias(route_geometry, vias);
        let solution = RoutingSolution::from_hyperpath(problem, &candidate)
            .map_err(|error| TscircuitRoutingError::Semantic(format!("{error:?}")))?;
        Ok(Self {
            solution,
            numeric_imports,
        })
    }
}

#[derive(Clone)]
struct WirePoint {
    point: Point2,
    layer: TraceLayer,
    width: Real,
}

struct JsonEmitter {
    decimal_places: usize,
    projections: Vec<TscircuitRoutingProjection>,
}

impl JsonEmitter {
    fn new(decimal_places: usize) -> Self {
        Self {
            decimal_places,
            projections: Vec::new(),
        }
    }

    fn real(&mut self, field: &str, value: &Real) -> Result<Value, TscircuitRoutingError> {
        let finite = value
            .to_f64_lossy()
            .filter(|value| value.is_finite())
            .ok_or_else(|| TscircuitRoutingError::NonFinite(field.into()))?;
        self.finite(field, value.to_string(), finite)
    }

    fn derived(
        &mut self,
        field: &str,
        exact: String,
        finite: f64,
    ) -> Result<Value, TscircuitRoutingError> {
        self.finite(field, exact, finite)
    }

    fn finite(
        &mut self,
        field: &str,
        exact: String,
        finite: f64,
    ) -> Result<Value, TscircuitRoutingError> {
        if !finite.is_finite() {
            return Err(TscircuitRoutingError::NonFinite(field.into()));
        }
        let emitted = format!("{:.*}", self.decimal_places, finite);
        let number = Number::from_str(&emitted)
            .map_err(|_| TscircuitRoutingError::NonFinite(field.into()))?;
        self.projections.push(TscircuitRoutingProjection {
            field: field.into(),
            exact,
            emitted,
        });
        Ok(Value::Number(number))
    }
}

fn global_minimum_width(
    problem: &RoutingProblem,
    fallback: Option<&Real>,
) -> Result<Real, TscircuitRoutingError> {
    let mut widths = problem
        .rules
        .iter()
        .map(|rule| rule.width.clone())
        .chain(fallback.cloned());
    let first = widths.next();
    let Some(mut minimum) = first else {
        return Err(TscircuitRoutingError::MissingMinimumTraceWidth);
    };
    for width in widths {
        if width < minimum {
            minimum = width;
        }
    }
    if !positive(&minimum) {
        return Err(TscircuitRoutingError::InvalidOptions);
    }
    Ok(minimum)
}

fn connection_json(
    problem: &RoutingProblem,
    emitter: &mut JsonEmitter,
    omissions: &mut Vec<TscircuitRoutingOmission>,
) -> Result<Vec<Value>, TscircuitRoutingError> {
    let mut grouped = BTreeMap::<NetId, Vec<_>>::new();
    for terminal in &problem.terminals {
        grouped
            .entry(terminal.net.clone())
            .or_default()
            .push(terminal);
    }
    let mut connections = Vec::new();
    for (net, terminals) in grouped {
        let mut physical = BTreeSet::new();
        let unique = terminals
            .into_iter()
            .filter(|terminal| physical.insert((terminal.instance.clone(), terminal.pad.clone())))
            .collect::<Vec<_>>();
        if unique.len() < 2 {
            continue;
        }
        let mut points = Vec::new();
        for terminal in unique {
            let Some(layer) = terminal.layers.first().copied() else {
                continue;
            };
            if terminal.layers.len() > 1 {
                omissions.push(TscircuitRoutingOmission::MultilayerTerminalCollapsed {
                    instance: terminal.instance.clone(),
                    pad: terminal.pad.clone(),
                    layers: terminal.layers.len(),
                });
            }
            let layer = problem
                .layer_aliases
                .iter()
                .find(|alias| alias.layer == layer)
                .ok_or_else(|| {
                    TscircuitRoutingError::InvalidGeometry(format!(
                        "terminal {}:{} has an unnamed layer",
                        terminal.instance.as_str(),
                        terminal.pad.as_str()
                    ))
                })?;
            let prefix = format!(
                "connections.{}.{}.{}",
                net.as_str(),
                terminal.instance.as_str(),
                terminal.pad.as_str()
            );
            points.push(json!({
                "x": emitter.real(&format!("{prefix}.x"), &terminal.center.x)?,
                "y": emitter.real(&format!("{prefix}.y"), &terminal.center.y)?,
                "layer": layer.name,
            }));
        }
        connections.push(json!({
            "name": net.as_str(),
            "pointsToConnect": points,
        }));
    }
    Ok(connections)
}

fn append_pad_obstacles(
    problem: &RoutingProblem,
    layout: &PcbLayout,
    emitter: &mut JsonEmitter,
    omissions: &mut Vec<TscircuitRoutingOmission>,
    output: &mut Vec<Value>,
) -> Result<(), TscircuitRoutingError> {
    let last_layer = problem
        .layer_aliases
        .iter()
        .map(|alias| alias.layer.0)
        .max()
        .unwrap_or(0);
    for placement in &layout.placements {
        let pattern = layout
            .land_patterns
            .iter()
            .find(|pattern| pattern.id == placement.land_pattern)
            .expect("validated placement land pattern exists");
        for pad in &pattern.pads {
            let (shape_center, width, height, boxed) = pad_box(&pad.shape)?;
            if boxed {
                omissions.push(TscircuitRoutingOmission::PadShapeBoxed {
                    instance: placement.instance.clone(),
                    pad: pad.id.clone(),
                });
            }
            let rotated_center = rotate_point(&shape_center, &pad.rotation_degrees);
            let local_center = Point2::new(
                rotated_center.x + pad.center.x.clone(),
                rotated_center.y + pad.center.y.clone(),
            );
            let center = placement.transform_point(&local_center);
            let rotation = match placement.side {
                BoardSide::Front => {
                    placement.rotation_degrees.clone() + pad.rotation_degrees.clone()
                }
                BoardSide::Back => {
                    placement.rotation_degrees.clone() - pad.rotation_degrees.clone()
                }
            };
            let layers = pad
                .copper_layers
                .iter()
                .map(|layer| match placement.side {
                    BoardSide::Front => *layer,
                    BoardSide::Back => TraceLayer(last_layer.saturating_sub(layer.0)),
                })
                .map(|layer| layer_name(problem, layer).map(|name| Value::String(name.into())))
                .collect::<Result<Vec<_>, _>>()?;
            let connected = problem
                .terminals
                .iter()
                .filter(|terminal| {
                    terminal.instance == placement.instance && terminal.pad == pad.id
                })
                .map(|terminal| terminal.net.as_str().to_owned())
                .collect::<BTreeSet<_>>()
                .into_iter()
                .map(Value::String)
                .collect();
            let prefix = format!(
                "obstacle.pad.{}.{}",
                placement.instance.as_str(),
                pad.id.as_str()
            );
            output.push(rect_obstacle(
                emitter,
                &prefix,
                &center,
                &width,
                &height,
                Some(&rotation),
                layers,
                connected,
                false,
            )?);
        }
    }
    Ok(())
}

fn append_cutout_obstacles(
    layout: &PcbLayout,
    all_layers: &[Value],
    emitter: &mut JsonEmitter,
    omissions: &mut Vec<TscircuitRoutingOmission>,
    output: &mut Vec<Value>,
) -> Result<(), TscircuitRoutingError> {
    for (index, cutout) in layout.outline.cutouts.iter().enumerate() {
        let vertices = contour_points(cutout);
        let (center, width, height) = bounds_box(&vertices, &format!("cutout[{index}]"))?;
        if !is_axis_aligned_rectangle(cutout) {
            omissions.push(TscircuitRoutingOmission::CutoutBoxed { index });
        }
        output.push(rect_obstacle(
            emitter,
            &format!("obstacle.cutout[{index}]"),
            &center,
            &width,
            &height,
            None,
            all_layers.to_vec(),
            Vec::new(),
            false,
        )?);
    }
    Ok(())
}

fn append_keepout_obstacles(
    problem: &RoutingProblem,
    layout: &PcbLayout,
    all_layers: &[Value],
    emitter: &mut JsonEmitter,
    omissions: &mut Vec<TscircuitRoutingOmission>,
    output: &mut Vec<Value>,
) -> Result<(), TscircuitRoutingError> {
    for keepout in &layout.keepouts {
        let (center, width, height) = bounds_box(&keepout.boundary, keepout.id.as_str())?;
        if !is_rectangle_points(&keepout.boundary) {
            omissions.push(TscircuitRoutingOmission::KeepoutBoxed(keepout.id.clone()));
        }
        let layers = match &keepout.scope {
            KeepoutScope::All => all_layers.to_vec(),
            KeepoutScope::Copper(layers) => layers
                .iter()
                .map(|layer| layer_name(problem, *layer).map(|name| Value::String(name.into())))
                .collect::<Result<Vec<_>, _>>()?,
            KeepoutScope::Vias | KeepoutScope::Components => {
                omissions.push(TscircuitRoutingOmission::KeepoutScopeWidened(
                    keepout.id.clone(),
                ));
                all_layers.to_vec()
            }
        };
        output.push(rect_obstacle(
            emitter,
            &format!("obstacle.keepout.{}", keepout.id.as_str()),
            &center,
            &width,
            &height,
            None,
            layers,
            Vec::new(),
            false,
        )?);
    }
    Ok(())
}

fn append_zone_obstacles(
    problem: &RoutingProblem,
    layout: &PcbLayout,
    emitter: &mut JsonEmitter,
    omissions: &mut Vec<TscircuitRoutingOmission>,
    output: &mut Vec<Value>,
) -> Result<(), TscircuitRoutingError> {
    for zone in &layout.zones {
        let (center, width, height) = bounds_box(&zone.boundary, zone.id.as_str())?;
        if !is_rectangle_points(&zone.boundary) {
            omissions.push(TscircuitRoutingOmission::CopperZoneBoxed(zone.id.clone()));
        }
        output.push(rect_obstacle(
            emitter,
            &format!("obstacle.zone.{}", zone.id.as_str()),
            &center,
            &width,
            &height,
            None,
            vec![Value::String(layer_name(problem, zone.layer)?.into())],
            vec![Value::String(zone.net.as_str().into())],
            true,
        )?);
    }
    Ok(())
}

fn append_existing_copper_obstacles(
    problem: &RoutingProblem,
    layout: &PcbLayout,
    emitter: &mut JsonEmitter,
    omissions: &mut Vec<TscircuitRoutingOmission>,
    output: &mut Vec<Value>,
) -> Result<(), TscircuitRoutingError> {
    for route in &layout.routes {
        for (index, segment) in route.segments.iter().enumerate() {
            let prefix = format!("obstacle.route.{}[{index}]", route.id.as_str());
            let layer = vec![Value::String(layer_name(problem, route.layer)?.into())];
            let connected = vec![Value::String(route.net.as_str().into())];
            match segment {
                PcbRouteSegment::Line(line) => {
                    let center = midpoint(line.start(), line.end())?;
                    let length = line.length_squared().sqrt().map_err(|error| {
                        TscircuitRoutingError::InvalidGeometry(error.to_string())
                    })? + route.width.clone();
                    let dx = line.end().x.clone() - line.start().x.clone();
                    let dy = line.end().y.clone() - line.start().y.clone();
                    let dx_finite = finite(&dx, &format!("{prefix}.rotation.dx"))?;
                    let dy_finite = finite(&dy, &format!("{prefix}.rotation.dy"))?;
                    let angle = dy_finite.atan2(dx_finite).to_degrees();
                    let rotation = emitter.derived(
                        &format!("{prefix}.ccwRotationDegrees"),
                        format!("atan2({dy},{dx})"),
                        angle,
                    )?;
                    output.push(rect_obstacle_with_rotation_value(
                        emitter,
                        &prefix,
                        &center,
                        &length,
                        &route.width,
                        Some(rotation),
                        layer,
                        connected,
                        false,
                    )?);
                }
                PcbRouteSegment::CircularArc(arc) => {
                    omissions.push(TscircuitRoutingOmission::CurvedRouteBoxed(route.id.clone()));
                    let diameter = arc.radius().clone() * Real::from(2) + route.width.clone();
                    output.push(rect_obstacle(
                        emitter,
                        &prefix,
                        arc.center(),
                        &diameter,
                        &diameter,
                        None,
                        layer,
                        connected,
                        false,
                    )?);
                }
                PcbRouteSegment::CubicBezier(bezier) => {
                    omissions.push(TscircuitRoutingOmission::CurvedRouteBoxed(route.id.clone()));
                    let points = [
                        bezier.start().clone(),
                        bezier.control0().clone(),
                        bezier.control1().clone(),
                        bezier.end().clone(),
                    ];
                    let (center, width, height) = bounds_box(&points, &prefix)?;
                    output.push(rect_obstacle(
                        emitter,
                        &prefix,
                        &center,
                        &(width + route.width.clone()),
                        &(height + route.width.clone()),
                        None,
                        layer,
                        connected,
                        false,
                    )?);
                }
            }
        }
    }
    for via in &problem.existing_vias {
        let layers = problem
            .layer_aliases
            .iter()
            .filter(|alias| alias.layer >= via.start_layer && alias.layer <= via.end_layer)
            .map(|alias| Value::String(alias.name.clone()))
            .collect();
        output.push(rect_obstacle(
            emitter,
            &format!("obstacle.via.{}", via.id.as_str()),
            &via.center,
            &via.land_diameter,
            &via.land_diameter,
            None,
            layers,
            vec![Value::String(via.net.as_str().into())],
            false,
        )?);
    }
    Ok(())
}

#[allow(clippy::too_many_arguments)]
fn rect_obstacle(
    emitter: &mut JsonEmitter,
    field: &str,
    center: &Point2,
    width: &Real,
    height: &Real,
    rotation: Option<&Real>,
    layers: Vec<Value>,
    connected_to: Vec<Value>,
    copper_pour: bool,
) -> Result<Value, TscircuitRoutingError> {
    let rotation = rotation
        .map(|rotation| emitter.real(&format!("{field}.ccwRotationDegrees"), rotation))
        .transpose()?;
    rect_obstacle_with_rotation_value(
        emitter,
        field,
        center,
        width,
        height,
        rotation,
        layers,
        connected_to,
        copper_pour,
    )
}

#[allow(clippy::too_many_arguments)]
fn rect_obstacle_with_rotation_value(
    emitter: &mut JsonEmitter,
    field: &str,
    center: &Point2,
    width: &Real,
    height: &Real,
    rotation: Option<Value>,
    layers: Vec<Value>,
    connected_to: Vec<Value>,
    copper_pour: bool,
) -> Result<Value, TscircuitRoutingError> {
    if !positive(width) || !positive(height) || layers.is_empty() {
        return Err(TscircuitRoutingError::InvalidGeometry(field.into()));
    }
    let mut object = Map::new();
    object.insert("type".into(), Value::String("rect".into()));
    object.insert("layers".into(), Value::Array(layers));
    object.insert(
        "center".into(),
        json!({
            "x": emitter.real(&format!("{field}.center.x"), &center.x)?,
            "y": emitter.real(&format!("{field}.center.y"), &center.y)?,
        }),
    );
    object.insert(
        "width".into(),
        emitter.real(&format!("{field}.width"), width)?,
    );
    object.insert(
        "height".into(),
        emitter.real(&format!("{field}.height"), height)?,
    );
    if let Some(rotation) = rotation {
        object.insert("ccwRotationDegrees".into(), rotation);
    }
    object.insert("connectedTo".into(), Value::Array(connected_to));
    if copper_pour {
        object.insert("isCopperPour".into(), Value::Bool(true));
    }
    Ok(Value::Object(object))
}

fn pad_box(shape: &PadShape) -> Result<(Point2, Real, Real, bool), TscircuitRoutingError> {
    match shape {
        PadShape::Circle { diameter } => Ok((
            Point2::new(Real::zero(), Real::zero()),
            diameter.clone(),
            diameter.clone(),
            true,
        )),
        PadShape::Rectangle { width, height } => Ok((
            Point2::new(Real::zero(), Real::zero()),
            width.clone(),
            height.clone(),
            false,
        )),
        PadShape::RoundedRectangle { width, height, .. } | PadShape::Obround { width, height } => {
            Ok((
                Point2::new(Real::zero(), Real::zero()),
                width.clone(),
                height.clone(),
                true,
            ))
        }
        PadShape::Polygon { vertices } => {
            let (center, width, height) = bounds_box(vertices, "pad polygon")?;
            Ok((center, width, height, true))
        }
    }
}

fn contour_points(contour: &BoardContour) -> Vec<Point2> {
    contour
        .segments()
        .iter()
        .flat_map(|segment| match segment {
            crate::BoardContourSegment::Line(line) => {
                vec![line.start().clone(), line.end().clone()]
            }
            crate::BoardContourSegment::CircularArc(arc) => {
                let radius = arc.radius().clone();
                vec![
                    Point2::new(
                        arc.center().x.clone() - radius.clone(),
                        arc.center().y.clone() - radius.clone(),
                    ),
                    Point2::new(
                        arc.center().x.clone() + radius.clone(),
                        arc.center().y.clone() + radius,
                    ),
                ]
            }
            crate::BoardContourSegment::CubicBezier(bezier) => vec![
                bezier.start().clone(),
                bezier.control0().clone(),
                bezier.control1().clone(),
                bezier.end().clone(),
            ],
        })
        .collect()
}

fn bounds_box(
    points: &[Point2],
    source: &str,
) -> Result<(Point2, Real, Real), TscircuitRoutingError> {
    let Some(first) = points.first() else {
        return Err(TscircuitRoutingError::InvalidGeometry(source.into()));
    };
    let mut min_x = first.x.clone();
    let mut max_x = first.x.clone();
    let mut min_y = first.y.clone();
    let mut max_y = first.y.clone();
    for point in &points[1..] {
        if point.x < min_x {
            min_x = point.x.clone();
        }
        if point.x > max_x {
            max_x = point.x.clone();
        }
        if point.y < min_y {
            min_y = point.y.clone();
        }
        if point.y > max_y {
            max_y = point.y.clone();
        }
    }
    let center = Point2::new(
        ((min_x.clone() + max_x.clone()) / Real::from(2))
            .map_err(|error| TscircuitRoutingError::InvalidGeometry(error.to_string()))?,
        ((min_y.clone() + max_y.clone()) / Real::from(2))
            .map_err(|error| TscircuitRoutingError::InvalidGeometry(error.to_string()))?,
    );
    Ok((center, max_x - min_x, max_y - min_y))
}

fn midpoint(left: &Point2, right: &Point2) -> Result<Point2, TscircuitRoutingError> {
    Ok(Point2::new(
        ((left.x.clone() + right.x.clone()) / Real::from(2))
            .map_err(|error| TscircuitRoutingError::InvalidGeometry(error.to_string()))?,
        ((left.y.clone() + right.y.clone()) / Real::from(2))
            .map_err(|error| TscircuitRoutingError::InvalidGeometry(error.to_string()))?,
    ))
}

fn rotate_point(point: &Point2, degrees: &Real) -> Point2 {
    let radians = degrees.clone().to_radians();
    let sin = radians.clone().sin();
    let cos = radians.cos();
    Point2::new(
        point.x.clone() * cos.clone() - point.y.clone() * sin.clone(),
        point.x.clone() * sin + point.y.clone() * cos,
    )
}

fn is_axis_aligned_rectangle(contour: &BoardContour) -> bool {
    contour
        .linear_vertices()
        .is_some_and(|vertices| is_rectangle_points(&vertices))
}

fn is_rectangle_points(points: &[Point2]) -> bool {
    if points.len() != 4 {
        return false;
    }
    let mut xs = Vec::<Real>::new();
    let mut ys = Vec::<Real>::new();
    let mut corners = Vec::<Point2>::new();
    for point in points {
        if !xs.iter().any(|x| x == &point.x) {
            xs.push(point.x.clone());
        }
        if !ys.iter().any(|y| y == &point.y) {
            ys.push(point.y.clone());
        }
        if !corners.iter().any(|corner| corner == point) {
            corners.push(point.clone());
        }
    }
    xs.len() == 2 && ys.len() == 2 && corners.len() == 4
}

fn layer_name(problem: &RoutingProblem, layer: TraceLayer) -> Result<&str, TscircuitRoutingError> {
    problem
        .layer_aliases
        .iter()
        .find(|alias| alias.layer == layer)
        .map(|alias| alias.name.as_str())
        .ok_or_else(|| TscircuitRoutingError::UnknownLayer(format!("{}", layer.0)))
}

fn layer_by_name(
    problem: &RoutingProblem,
    name: &str,
) -> Result<TraceLayer, TscircuitRoutingError> {
    problem
        .layer_aliases
        .iter()
        .find(|alias| alias.name == name)
        .map(|alias| alias.layer)
        .ok_or_else(|| TscircuitRoutingError::UnknownLayer(name.into()))
}

fn import_wire_point(
    problem: &RoutingProblem,
    value: &Value,
    field: &str,
    numeric_imports: &mut Vec<TscircuitRoutingNumericImport>,
) -> Result<WirePoint, TscircuitRoutingError> {
    let point = import_point(value, field, numeric_imports)?;
    let layer = layer_by_name(problem, string_field(value, "layer", field)?)?;
    let width = real_field(value, "width", field, numeric_imports)?;
    if !positive(&width) {
        return Err(TscircuitRoutingError::InvalidRoute(format!(
            "{field}.width is not positive"
        )));
    }
    Ok(WirePoint {
        point,
        layer,
        width,
    })
}

fn import_point(
    value: &Value,
    field: &str,
    numeric_imports: &mut Vec<TscircuitRoutingNumericImport>,
) -> Result<Point2, TscircuitRoutingError> {
    Ok(Point2::new(
        real_field(value, "x", field, numeric_imports)?,
        real_field(value, "y", field, numeric_imports)?,
    ))
}

fn real_field(
    value: &Value,
    name: &str,
    field: &str,
    numeric_imports: &mut Vec<TscircuitRoutingNumericImport>,
) -> Result<Real, TscircuitRoutingError> {
    let path = format!("{field}.{name}");
    let source = value
        .get(name)
        .and_then(Value::as_number)
        .ok_or_else(|| TscircuitRoutingError::InvalidField(path.clone()))?
        .to_string();
    let exact =
        Real::from_str(&source).map_err(|_| TscircuitRoutingError::InvalidField(path.clone()))?;
    numeric_imports.push(TscircuitRoutingNumericImport {
        field: path,
        source,
        exact: exact.to_string(),
    });
    Ok(exact)
}

fn string_field<'a>(
    value: &'a Value,
    name: &str,
    field: &str,
) -> Result<&'a str, TscircuitRoutingError> {
    value
        .get(name)
        .and_then(Value::as_str)
        .ok_or_else(|| TscircuitRoutingError::InvalidField(format!("{field}.{name}")))
}

fn positive(value: &Real) -> bool {
    value.partial_cmp(&Real::zero()) == Some(Ordering::Greater)
}

fn finite(value: &Real, field: &str) -> Result<f64, TscircuitRoutingError> {
    value
        .to_f64_lossy()
        .filter(|value| value.is_finite())
        .ok_or_else(|| TscircuitRoutingError::NonFinite(field.into()))
}

fn drill_intent(plating: Plating) -> ViaDrillIntent {
    match plating {
        Plating::Plated => ViaDrillIntent::Plated,
        Plating::NonPlated => ViaDrillIntent::NonPlated,
        Plating::Unspecified => ViaDrillIntent::Unspecified,
    }
}
