//! Audited native 2D PCB review rendering.

use std::fmt::{Display, Formatter, Write};

use hyperlattice::Point2;
use hyperpath::{ArcDirection, ExplicitArcSweepClass, TraceLayer};
use hyperreal::Real;

use crate::{
    BoardSide, Circuit, CopperZoneConnection, CopperZoneFill, DrillShape, PadShape, PcbLayout,
};
use crate::{NegotiatedRouteConflictGeometry, NegotiatedRouteReport, NegotiatedRouteStatus};

/// Native PCB SVG rendering policy.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PcbSvgOptions {
    /// Digits emitted after each decimal point.
    pub decimal_places: usize,
    /// Display-space margin around the board bounds.
    pub margin: usize,
    /// Optional single copper layer; `None` overlays every layer.
    pub layer: Option<TraceLayer>,
}

impl Default for PcbSvgOptions {
    fn default() -> Self {
        Self {
            decimal_places: 6,
            margin: 20,
            layer: None,
        }
    }
}

/// Audit record for one exact PCB scalar projected into SVG.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PcbSvgProjection {
    /// Semantic source field.
    pub field: String,
    /// Exact-aware source spelling.
    pub source: String,
    /// Decimal token written to SVG.
    pub emitted: String,
}

/// Native PCB SVG artifact and its numeric projection audit.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct PcbSvgReport {
    /// Standalone SVG document.
    pub svg: String,
    /// Every exact-to-finite coordinate or dimension projection.
    pub projections: Vec<PcbSvgProjection>,
}

/// Native negotiated-routing iteration SVG policy.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct NegotiatedRouteSvgOptions {
    /// Zero-based retained router pass.
    pub iteration: usize,
    /// Digits emitted after each decimal point.
    pub decimal_places: usize,
    /// Display-space margin around the board bounds.
    pub margin: usize,
    /// Optional conductor layer; `None` overlays every layer.
    pub layer: Option<TraceLayer>,
}

impl Default for NegotiatedRouteSvgOptions {
    fn default() -> Self {
        Self {
            iteration: 0,
            decimal_places: 6,
            margin: 20,
            layer: None,
        }
    }
}

/// Standalone negotiated-routing iteration view and projection audit.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct NegotiatedRouteSvgReport {
    /// Standalone SVG document.
    pub svg: String,
    /// Rendered zero-based router pass.
    pub iteration: usize,
    /// Terminal status of the complete routing run.
    pub status: NegotiatedRouteStatus,
    /// Provisional planar path edges and layer transitions rendered.
    pub rendered_edges: usize,
    /// Over-subscribed resources rendered.
    pub rendered_conflicts: usize,
    /// Typed failures retained by this pass.
    pub failures: usize,
    /// Every exact-to-finite coordinate projection.
    pub projections: Vec<PcbSvgProjection>,
}

/// Self-contained negotiated-routing replay viewer policy.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct NegotiatedRouteHtmlOptions {
    /// Digits emitted after each decimal point.
    pub decimal_places: usize,
    /// Display-space margin around the board bounds.
    pub margin: usize,
    /// Optional conductor layer; `None` overlays every layer.
    pub layer: Option<TraceLayer>,
    /// Human-readable document title.
    pub title: String,
}

impl Default for NegotiatedRouteHtmlOptions {
    fn default() -> Self {
        Self {
            decimal_places: 6,
            margin: 20,
            layer: None,
            title: "HyperCircuit negotiated-route replay".into(),
        }
    }
}

/// Standalone interactive routing replay and aggregate projection audit.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct NegotiatedRouteHtmlReport {
    /// Self-contained HTML document.
    pub html: String,
    /// Terminal status of the complete routing run.
    pub status: NegotiatedRouteStatus,
    /// Number of retained passes embedded in the viewer.
    pub rendered_iterations: usize,
    /// Provisional path edges and layer transitions across every pass.
    pub rendered_edges: usize,
    /// Over-subscribed resources across every pass.
    pub rendered_conflicts: usize,
    /// Typed failures across every pass.
    pub failures: usize,
    /// Every exact-to-finite coordinate projection across every pass.
    pub projections: Vec<PcbSvgProjection>,
}

/// Failure to render one retained negotiated-routing pass.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum NegotiatedRouteSvgError {
    /// Shared PCB projection failed.
    Pcb(PcbSvgError),
    /// The report does not retain any routing passes.
    EmptyReport,
    /// The report does not retain the requested pass.
    UnknownIteration(usize),
}

impl Display for NegotiatedRouteSvgError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Pcb(error) => Display::fmt(error, formatter),
            Self::EmptyReport => formatter.write_str("negotiated route report has no iterations"),
            Self::UnknownIteration(iteration) => {
                write!(
                    formatter,
                    "negotiated route iteration {iteration} is absent"
                )
            }
        }
    }
}

impl std::error::Error for NegotiatedRouteSvgError {}

impl From<PcbSvgError> for NegotiatedRouteSvgError {
    fn from(error: PcbSvgError) -> Self {
        Self::Pcb(error)
    }
}

/// Failure to produce a faithful finite PCB review artifact.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum PcbSvgError {
    /// Circuit or PCB structural validation failed.
    InvalidLayout,
    /// An exact scalar had no finite display projection.
    NonFinite(String),
    /// No exterior contour was available for review bounds.
    Empty,
}

impl Display for PcbSvgError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::InvalidLayout => formatter.write_str("cannot render an invalid PCB layout"),
            Self::NonFinite(field) => write!(formatter, "non-finite PCB field: {field}"),
            Self::Empty => formatter.write_str("PCB layout has no renderable exterior"),
        }
    }
}

impl std::error::Error for PcbSvgError {}

impl NegotiatedRouteReport {
    /// Renders every retained routing pass into a self-contained interactive viewer.
    pub fn replay_html(
        &self,
        layout: &PcbLayout,
        options: NegotiatedRouteHtmlOptions,
    ) -> Result<NegotiatedRouteHtmlReport, NegotiatedRouteSvgError> {
        if self.iteration_states.is_empty() {
            return Err(NegotiatedRouteSvgError::EmptyReport);
        }
        let mut views = Vec::with_capacity(self.iteration_states.len());
        for state in &self.iteration_states {
            views.push(self.iteration_svg(
                layout,
                NegotiatedRouteSvgOptions {
                    iteration: state.iteration,
                    decimal_places: options.decimal_places,
                    margin: options.margin,
                    layer: options.layer,
                },
            )?);
        }
        let rendered_edges = views.iter().map(|view| view.rendered_edges).sum();
        let rendered_conflicts = views.iter().map(|view| view.rendered_conflicts).sum();
        let failures = views.iter().map(|view| view.failures).sum();
        let mut projections = Vec::new();
        for view in &mut views {
            projections.append(&mut view.projections);
        }
        let layer = options
            .layer
            .map_or_else(|| "all".into(), |layer| layer.0.to_string());
        let mut html = format!(
            "<!doctype html>\n<html lang=\"en\"><head><meta charset=\"utf-8\"><meta name=\"viewport\" content=\"width=device-width,initial-scale=1\"><title>{}</title>\n",
            xml(&options.title)
        );
        html.push_str(
            r#"<style>
:root{color-scheme:dark;font-family:ui-monospace,SFMono-Regular,Consolas,monospace;background:#0f1117;color:#e7e9ee}
*{box-sizing:border-box}body{margin:0;min-height:100vh;display:grid;grid-template-rows:auto 1fr;background:#0f1117}
header{display:flex;flex-wrap:wrap;align-items:center;gap:.8rem;padding:.8rem 1rem;border-bottom:1px solid #343946;background:#171b24}
h1{font:600 1rem system-ui,sans-serif;margin:0 auto 0 0}.controls{display:flex;align-items:center;gap:.5rem}
button,input{accent-color:#4cc9f0}button{border:1px solid #4b5263;border-radius:.35rem;background:#262c38;color:inherit;padding:.3rem .65rem}
button:disabled{opacity:.4}.metrics{min-width:20rem;color:#b9c0ce}.stage{display:grid;place-items:center;overflow:auto;padding:1rem}
.route-frame{width:min(100%,80rem)}.route-frame[hidden]{display:none}.route-frame svg{display:block;width:100%;height:auto;max-height:calc(100vh - 7rem)}
</style></head>
"#,
        );
        writeln!(
            html,
            "<body data-status=\"{:?}\" data-layer=\"{}\"><header><h1>{}</h1><div class=\"controls\"><button id=\"previous\" type=\"button\" aria-label=\"Previous pass\">Previous</button><input id=\"pass\" type=\"range\" min=\"0\" max=\"{}\" value=\"0\" step=\"1\" aria-label=\"Routing pass\"><button id=\"next\" type=\"button\" aria-label=\"Next pass\">Next</button></div><output id=\"metrics\" class=\"metrics\"></output></header><main class=\"stage\">",
            self.status,
            layer,
            xml(&options.title),
            views.len() - 1,
        )
        .expect("writing to String cannot fail");
        for (index, view) in views.iter().enumerate() {
            writeln!(
                html,
                "<section class=\"route-frame\" data-position=\"{index}\" data-iteration=\"{}\" data-edges=\"{}\" data-conflicts=\"{}\" data-failures=\"{}\"{}>{}</section>",
                view.iteration,
                view.rendered_edges,
                view.rendered_conflicts,
                view.failures,
                if index == 0 { "" } else { " hidden" },
                view.svg,
            )
            .expect("writing to String cannot fail");
        }
        html.push_str(
            r##"</main><script>
const frames=[...document.querySelectorAll(".route-frame")];
const pass=document.querySelector("#pass");
const previous=document.querySelector("#previous");
const next=document.querySelector("#next");
const metrics=document.querySelector("#metrics");
function show(position){
  const bounded=Math.max(0,Math.min(frames.length-1,Number(position)));
  frames.forEach((frame,index)=>frame.hidden=index!==bounded);
  pass.value=String(bounded);
  previous.disabled=bounded===0;
  next.disabled=bounded===frames.length-1;
  const frame=frames[bounded];
  metrics.textContent=`pass ${frame.dataset.iteration} · ${frame.dataset.edges} edges · ${frame.dataset.conflicts} conflicts · ${frame.dataset.failures} failures`;
}
pass.addEventListener("input",event=>show(event.target.value));
previous.addEventListener("click",()=>show(Number(pass.value)-1));
next.addEventListener("click",()=>show(Number(pass.value)+1));
document.addEventListener("keydown",event=>{
  if(event.key==="ArrowLeft")show(Number(pass.value)-1);
  if(event.key==="ArrowRight")show(Number(pass.value)+1);
});
show(0);
</script></body></html>
"##,
        );
        Ok(NegotiatedRouteHtmlReport {
            html,
            status: self.status,
            rendered_iterations: views.len(),
            rendered_edges,
            rendered_conflicts,
            failures,
            projections,
        })
    }

    /// Renders one retained provisional routing pass without reconstructing private grid state.
    pub fn iteration_svg(
        &self,
        layout: &PcbLayout,
        options: NegotiatedRouteSvgOptions,
    ) -> Result<NegotiatedRouteSvgReport, NegotiatedRouteSvgError> {
        if options.decimal_places == 0 {
            return Err(PcbSvgError::InvalidLayout.into());
        }
        let state = self
            .iteration_states
            .iter()
            .find(|state| state.iteration == options.iteration)
            .ok_or(NegotiatedRouteSvgError::UnknownIteration(options.iteration))?;
        if layout.outline.exterior.segments().is_empty() {
            return Err(PcbSvgError::Empty.into());
        }
        let mut projector = Projector::new(options.decimal_places);
        let exterior = projector.contour("outline.exterior", &layout.outline.exterior)?;
        let cutouts = layout
            .outline
            .cutouts
            .iter()
            .enumerate()
            .map(|(index, contour)| projector.contour(&format!("outline.cutout[{index}]"), contour))
            .collect::<Result<Vec<_>, _>>()?;
        let mut bounds = Bounds::default();
        include_contour_bounds(&mut bounds, &exterior);
        let margin = options.margin as f64;
        let min_x = bounds.min_x - margin;
        let min_y = bounds.min_y - margin;
        let width = bounds.max_x - bounds.min_x + 2.0 * margin;
        let height = bounds.max_y - bounds.min_y + 2.0 * margin;
        let mut svg = format!(
            "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"{min_x} {min_y} {width} {height}\" data-iteration=\"{}\" data-status=\"{:?}\">\n",
            state.iteration, self.status
        );
        writeln!(
            svg,
            "<path d=\"{}\" fill=\"#171b24\" stroke=\"#8a93a5\" stroke-width=\"0.35\"/>",
            svg_contour_path(&exterior)
        )
        .expect("writing to String cannot fail");
        for cutout in &cutouts {
            writeln!(
                svg,
                "<path d=\"{}\" fill=\"#ffffff\" stroke=\"#8a93a5\" stroke-width=\"0.35\"/>",
                svg_contour_path(cutout)
            )
            .expect("writing to String cannot fail");
        }
        let mut rendered_edges = 0_usize;
        for (net_index, net) in state.nets.iter().enumerate() {
            for (path_index, path) in net.paths.iter().enumerate() {
                for (edge_index, edge) in path.windows(2).enumerate() {
                    let first = &edge[0];
                    let second = &edge[1];
                    if first.layer == second.layer {
                        if options.layer.is_some_and(|layer| layer != first.layer) {
                            continue;
                        }
                        let start = projector.point(
                            &format!(
                                "iteration[{}].net[{net_index}].path[{path_index}].edge[{edge_index}].start",
                                state.iteration
                            ),
                            &first.center,
                        )?;
                        let end = projector.point(
                            &format!(
                                "iteration[{}].net[{net_index}].path[{path_index}].edge[{edge_index}].end",
                                state.iteration
                            ),
                            &second.center,
                        )?;
                        writeln!(
                            svg,
                            "<line data-net=\"{}\" data-layer=\"{}\" data-path=\"{}\" data-edge=\"{}\" x1=\"{}\" y1=\"{}\" x2=\"{}\" y2=\"{}\" stroke=\"{}\" stroke-width=\"0.65\" stroke-linecap=\"round\"/>",
                            xml(net.net.as_str()),
                            first.layer.0,
                            path_index,
                            edge_index,
                            start.0,
                            start.1,
                            end.0,
                            end.1,
                            route_layer_color(first.layer),
                        )
                        .expect("writing to String cannot fail");
                    } else {
                        if options
                            .layer
                            .is_some_and(|layer| layer != first.layer && layer != second.layer)
                        {
                            continue;
                        }
                        let center = projector.point(
                            &format!(
                                "iteration[{}].net[{net_index}].path[{path_index}].via[{edge_index}]",
                                state.iteration
                            ),
                            &first.center,
                        )?;
                        writeln!(
                            svg,
                            "<circle data-net=\"{}\" data-via=\"true\" data-from-layer=\"{}\" data-to-layer=\"{}\" cx=\"{}\" cy=\"{}\" r=\"0.8\" fill=\"#f4d35e\" stroke=\"#111\" stroke-width=\"0.2\"/>",
                            xml(net.net.as_str()),
                            first.layer.0,
                            second.layer.0,
                            center.0,
                            center.1,
                        )
                        .expect("writing to String cannot fail");
                    }
                    rendered_edges = rendered_edges.saturating_add(1);
                }
            }
        }
        let mut rendered_conflicts = 0_usize;
        for (index, conflict) in state.conflicts.iter().enumerate() {
            let nets = conflict
                .nets
                .iter()
                .map(|net| net.as_str())
                .collect::<Vec<_>>()
                .join(",");
            match &conflict.geometry {
                NegotiatedRouteConflictGeometry::Node { center, layer } => {
                    if options.layer.is_some_and(|selected| selected != *layer) {
                        continue;
                    }
                    let center = projector.point(
                        &format!("iteration[{}].conflict[{index}]", state.iteration),
                        center,
                    )?;
                    writeln!(
                        svg,
                        "<circle data-conflict=\"node\" data-nets=\"{}\" data-layer=\"{}\" cx=\"{}\" cy=\"{}\" r=\"1.25\" fill=\"#ff3b30\"/>",
                        xml(&nets), layer.0, center.0, center.1
                    )
                    .expect("writing to String cannot fail");
                }
                NegotiatedRouteConflictGeometry::Segment { start, end, layer } => {
                    if options.layer.is_some_and(|selected| selected != *layer) {
                        continue;
                    }
                    let start = projector.point(
                        &format!("iteration[{}].conflict[{index}].start", state.iteration),
                        start,
                    )?;
                    let end = projector.point(
                        &format!("iteration[{}].conflict[{index}].end", state.iteration),
                        end,
                    )?;
                    writeln!(
                        svg,
                        "<line data-conflict=\"segment\" data-nets=\"{}\" data-layer=\"{}\" x1=\"{}\" y1=\"{}\" x2=\"{}\" y2=\"{}\" stroke=\"#ff3b30\" stroke-width=\"1.8\"/>",
                        xml(&nets), layer.0, start.0, start.1, end.0, end.1
                    )
                    .expect("writing to String cannot fail");
                }
                NegotiatedRouteConflictGeometry::DiagonalCell {
                    lower_left,
                    upper_right,
                    layer,
                } => {
                    if options.layer.is_some_and(|selected| selected != *layer) {
                        continue;
                    }
                    let lower_left = projector.point(
                        &format!(
                            "iteration[{}].conflict[{index}].lower_left",
                            state.iteration
                        ),
                        lower_left,
                    )?;
                    let upper_right = projector.point(
                        &format!(
                            "iteration[{}].conflict[{index}].upper_right",
                            state.iteration
                        ),
                        upper_right,
                    )?;
                    writeln!(
                        svg,
                        "<rect data-conflict=\"diagonal-cell\" data-nets=\"{}\" data-layer=\"{}\" x=\"{}\" y=\"{}\" width=\"{}\" height=\"{}\" fill=\"#ff3b3044\" stroke=\"#ff3b30\" stroke-width=\"0.5\"/>",
                        xml(&nets),
                        layer.0,
                        lower_left.0.min(upper_right.0),
                        lower_left.1.min(upper_right.1),
                        (upper_right.0 - lower_left.0).abs(),
                        (upper_right.1 - lower_left.1).abs(),
                    )
                    .expect("writing to String cannot fail");
                }
                NegotiatedRouteConflictGeometry::Via {
                    center,
                    start_layer,
                    end_layer,
                } => {
                    if options
                        .layer
                        .is_some_and(|layer| layer < *start_layer || layer > *end_layer)
                    {
                        continue;
                    }
                    let center = projector.point(
                        &format!("iteration[{}].conflict[{index}]", state.iteration),
                        center,
                    )?;
                    writeln!(
                        svg,
                        "<circle data-conflict=\"via\" data-nets=\"{}\" data-from-layer=\"{}\" data-to-layer=\"{}\" cx=\"{}\" cy=\"{}\" r=\"1.5\" fill=\"none\" stroke=\"#ff3b30\" stroke-width=\"0.8\"/>",
                        xml(&nets), start_layer.0, end_layer.0, center.0, center.1
                    )
                    .expect("writing to String cannot fail");
                }
            }
            rendered_conflicts = rendered_conflicts.saturating_add(1);
        }
        for (index, failure) in state.failures.iter().enumerate() {
            writeln!(
                svg,
                "<metadata data-failure-index=\"{index}\" data-failure=\"{}\"/>",
                xml(&format!("{failure:?}"))
            )
            .expect("writing to String cannot fail");
        }
        svg.push_str("</svg>\n");
        Ok(NegotiatedRouteSvgReport {
            svg,
            iteration: state.iteration,
            status: self.status,
            rendered_edges,
            rendered_conflicts,
            failures: state.failures.len(),
            projections: projector.projections,
        })
    }
}

fn route_layer_color(layer: TraceLayer) -> &'static str {
    const COLORS: [&str; 8] = [
        "#4cc9f0", "#f72585", "#80ed99", "#ff9f1c", "#b5179e", "#ffd166", "#00b4d8", "#ef476f",
    ];
    COLORS[layer.0 as usize % COLORS.len()]
}

impl PcbLayout {
    /// Renders board contours, copper zones, routes, vias, placed pads, and origins.
    ///
    /// This is a review view, not manufacturing output. Every exact scalar
    /// crossing the SVG boundary is retained in [`PcbSvgReport::projections`].
    pub fn to_svg(
        &self,
        circuit: &Circuit,
        options: PcbSvgOptions,
    ) -> Result<PcbSvgReport, PcbSvgError> {
        if options.decimal_places == 0
            || !circuit.validate().is_valid()
            || !self.validate(circuit).is_valid()
        {
            return Err(PcbSvgError::InvalidLayout);
        }
        if self.outline.exterior.segments().is_empty() {
            return Err(PcbSvgError::Empty);
        }
        let mut projector = Projector::new(options.decimal_places);
        let exterior = projector.contour("outline.exterior", &self.outline.exterior)?;
        let mut bounds = Bounds::default();
        include_contour_bounds(&mut bounds, &exterior);
        let cutouts = self
            .outline
            .cutouts
            .iter()
            .enumerate()
            .map(|(index, contour)| projector.contour(&format!("outline.cutout[{index}]"), contour))
            .collect::<Result<Vec<_>, _>>()?;
        let zones = self
            .zones
            .iter()
            .filter(|zone| options.layer.is_none_or(|layer| layer == zone.layer))
            .map(|zone| {
                projector
                    .points(
                        &format!("zone.{}.boundary", zone.id.as_str()),
                        &zone.boundary,
                    )
                    .map(|points| (zone, points))
            })
            .collect::<Result<Vec<_>, _>>()?;
        let mut routes = Vec::new();
        for route in self
            .routes
            .iter()
            .filter(|route| options.layer.is_none_or(|layer| layer == route.layer))
        {
            let width =
                projector.value(&format!("route.{}.width", route.id.as_str()), &route.width)?;
            let segments = route
                .segments
                .iter()
                .enumerate()
                .map(|(index, segment)| match segment {
                    crate::PcbRouteSegment::Line(segment) => Ok(RenderedRouteSegment::Line {
                        start: projector.point(
                            &format!("route.{}.segment[{index}].start", route.id.as_str()),
                            segment.start(),
                        )?,
                        end: projector.point(
                            &format!("route.{}.segment[{index}].end", route.id.as_str()),
                            segment.end(),
                        )?,
                    }),
                    crate::PcbRouteSegment::CircularArc(arc) => Ok(RenderedRouteSegment::Arc {
                        center: projector.point(
                            &format!("route.{}.segment[{index}].center", route.id.as_str()),
                            arc.center(),
                        )?,
                        radius: projector.value(
                            &format!("route.{}.segment[{index}].radius", route.id.as_str()),
                            arc.radius(),
                        )?,
                        start: projector.point(
                            &format!("route.{}.segment[{index}].start", route.id.as_str()),
                            arc.start(),
                        )?,
                        end: projector.point(
                            &format!("route.{}.segment[{index}].end", route.id.as_str()),
                            arc.end(),
                        )?,
                        sweep: arc.direction() != ArcDirection::Ccw,
                        sweep_class: arc.facts().sweep_class,
                    }),
                    crate::PcbRouteSegment::CubicBezier(bezier) => {
                        Ok(RenderedRouteSegment::CubicBezier {
                            start: projector.point(
                                &format!("route.{}.segment[{index}].start", route.id.as_str()),
                                bezier.start(),
                            )?,
                            control0: projector.point(
                                &format!("route.{}.segment[{index}].control0", route.id.as_str()),
                                bezier.control0(),
                            )?,
                            control1: projector.point(
                                &format!("route.{}.segment[{index}].control1", route.id.as_str()),
                                bezier.control1(),
                            )?,
                            end: projector.point(
                                &format!("route.{}.segment[{index}].end", route.id.as_str()),
                                bezier.end(),
                            )?,
                        })
                    }
                })
                .collect::<Result<Vec<_>, PcbSvgError>>()?;
            routes.push((route, width, segments));
        }
        let stitching = self.realize_stitching_vias();
        let mut vias = Vec::new();
        for via in self.vias.iter().chain(&stitching.vias).filter(|via| {
            options
                .layer
                .is_none_or(|layer| layer >= via.start_layer && layer <= via.end_layer)
        }) {
            let center =
                projector.point(&format!("via.{}.center", via.id.as_str()), &via.center)?;
            let diameter = projector.value(
                &format!("via.{}.land_diameter", via.id.as_str()),
                &via.land_diameter,
            )?;
            let drill = projector.value(
                &format!("via.{}.drill_diameter", via.id.as_str()),
                &via.drill_diameter,
            )?;
            vias.push((via, center, diameter, drill));
        }
        let mut placements = Vec::new();
        let mut placed_pads = Vec::new();
        for placement in &self.placements {
            let origin = projector.point(
                &format!("placement.{}.position", placement.instance.as_str()),
                &placement.position,
            )?;
            let rotation = projector.value(
                &format!("placement.{}.rotation", placement.instance.as_str()),
                &placement.rotation_degrees,
            )?;
            let pattern = self
                .land_patterns
                .iter()
                .find(|pattern| pattern.id == placement.land_pattern)
                .expect("validated placement land pattern must exist");
            for pad in pattern.pads.iter().filter(|pad| {
                options
                    .layer
                    .is_none_or(|layer| pad.copper_layers.contains(&layer))
            }) {
                let local = projector.point(
                    &format!(
                        "placement.{}.pad.{}.center",
                        placement.instance.as_str(),
                        pad.id.as_str()
                    ),
                    &pad.center,
                )?;
                let center = transform(local, origin, rotation, placement.side);
                let field = format!(
                    "placement.{}.pad.{}",
                    placement.instance.as_str(),
                    pad.id.as_str()
                );
                let pad_rotation =
                    projector.value(&format!("{field}.rotation"), &pad.rotation_degrees)?;
                let rendered_rotation = match placement.side {
                    BoardSide::Front => rotation + pad_rotation,
                    BoardSide::Back => rotation - pad_rotation,
                };
                let shape = match &pad.shape {
                    PadShape::Circle { diameter } => RenderedPadShape::Circle {
                        diameter: projector.value(&format!("{field}.diameter"), diameter)?,
                    },
                    PadShape::Rectangle { width, height } => RenderedPadShape::Rectangle {
                        width: projector.value(&format!("{field}.width"), width)?,
                        height: projector.value(&format!("{field}.height"), height)?,
                        radius: 0.0,
                    },
                    PadShape::RoundedRectangle {
                        width,
                        height,
                        corner_radius,
                    } => RenderedPadShape::Rectangle {
                        width: projector.value(&format!("{field}.width"), width)?,
                        height: projector.value(&format!("{field}.height"), height)?,
                        radius: projector
                            .value(&format!("{field}.corner_radius"), corner_radius)?,
                    },
                    PadShape::Obround { width, height } => {
                        let width = projector.value(&format!("{field}.width"), width)?;
                        let height = projector.value(&format!("{field}.height"), height)?;
                        RenderedPadShape::Rectangle {
                            width,
                            height,
                            radius: width.min(height) / 2.0,
                        }
                    }
                    PadShape::Polygon { vertices } => {
                        let points = vertices
                            .iter()
                            .enumerate()
                            .map(|(index, vertex)| {
                                projector
                                    .point(&format!("{field}.vertex[{index}]"), vertex)
                                    .map(|vertex| {
                                        let vertex = rotate(vertex, pad_rotation);
                                        transform(
                                            (local.0 + vertex.0, local.1 + vertex.1),
                                            origin,
                                            rotation,
                                            placement.side,
                                        )
                                    })
                            })
                            .collect::<Result<Vec<_>, _>>()?;
                        RenderedPadShape::Polygon { points }
                    }
                };
                let drill = match &pad.drill {
                    Some(DrillShape::Round { diameter }) => Some(RenderedDrill::Round {
                        diameter: projector.value(&format!("{field}.drill"), diameter)?,
                    }),
                    Some(DrillShape::Slot { start, end, width }) => {
                        let slot_start = projector.point(&format!("{field}.slot_start"), start)?;
                        let slot_end = projector.point(&format!("{field}.slot_end"), end)?;
                        let slot_start = rotate(slot_start, pad_rotation);
                        let slot_end = rotate(slot_end, pad_rotation);
                        Some(RenderedDrill::Slot {
                            start: transform(
                                (local.0 + slot_start.0, local.1 + slot_start.1),
                                origin,
                                rotation,
                                placement.side,
                            ),
                            end: transform(
                                (local.0 + slot_end.0, local.1 + slot_end.1),
                                origin,
                                rotation,
                                placement.side,
                            ),
                            width: projector.value(&format!("{field}.slot_width"), width)?,
                        })
                    }
                    None => None,
                };
                placed_pads.push(RenderedPad {
                    instance: placement.instance.as_str().into(),
                    pattern: pattern.id.as_str().into(),
                    pad: pad.id.as_str().into(),
                    center,
                    rotation: rendered_rotation,
                    shape,
                    drill,
                });
            }
            placements.push((placement, origin));
        }

        let margin = options.margin as f64;
        let min_x = bounds.min_x - margin;
        let min_y = bounds.min_y - margin;
        let width = bounds.max_x - bounds.min_x + 2.0 * margin;
        let height = bounds.max_y - bounds.min_y + 2.0 * margin;
        let mut svg = format!(
            "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"{min_x} {min_y} {width} {height}\">\n"
        );
        svg.push_str("<defs>\n");
        for (index, (zone, _)) in zones.iter().enumerate() {
            if let CopperZoneFill::Hatched {
                line_width,
                gap,
                angle_degrees,
            } = &zone.fill
            {
                let line_width = projector.value(
                    &format!("zone.{}.hatch.line_width", zone.id.as_str()),
                    line_width,
                )?;
                let gap = projector.value(&format!("zone.{}.hatch.gap", zone.id.as_str()), gap)?;
                let angle = projector.value(
                    &format!("zone.{}.hatch.angle", zone.id.as_str()),
                    angle_degrees,
                )?;
                let pitch = line_width + gap;
                writeln!(
                    svg,
                    "<pattern id=\"zone-hatch-{index}\" patternUnits=\"userSpaceOnUse\" width=\"{pitch}\" height=\"{pitch}\" patternTransform=\"rotate({angle})\"><line x1=\"0\" y1=\"0\" x2=\"{pitch}\" y2=\"0\" stroke=\"#b87333\" stroke-width=\"{line_width}\"/></pattern>"
                )
                .expect("writing to String cannot fail");
            }
        }
        svg.push_str("</defs>\n");
        svg.push_str("<g stroke-linecap=\"round\" stroke-linejoin=\"round\">\n");
        writeln!(
            svg,
            "<path d=\"{}\" fill=\"#222831\" stroke=\"#111\" stroke-width=\"0.8\"/>",
            svg_contour_path(&exterior)
        )
        .expect("writing to String cannot fail");
        for contour in &cutouts {
            writeln!(
                svg,
                "<path d=\"{}\" fill=\"#fff\" stroke=\"#111\" stroke-width=\"0.8\"/>",
                svg_contour_path(contour)
            )
            .expect("writing to String cannot fail");
        }
        for (index, (zone, points)) in zones.iter().enumerate() {
            let fill = match &zone.fill {
                CopperZoneFill::Solid => "#b8733355".to_owned(),
                CopperZoneFill::Hatched { .. } => format!("url(#zone-hatch-{index})"),
            };
            let connection = match &zone.connection {
                CopperZoneConnection::Solid => "solid",
                CopperZoneConnection::Isolated => "isolated",
                CopperZoneConnection::ThermalRelief { .. } => "thermal-relief",
            };
            let clearance = projector.value(
                &format!("zone.{}.clearance", zone.id.as_str()),
                &zone.clearance,
            )?;
            let island_mode = match (
                zone.islands.remove_unconnected,
                zone.islands.minimum_area.is_some(),
            ) {
                (false, false) => "retain",
                (true, false) => "remove-all-unconnected",
                (true, true) => "remove-unconnected-below-area",
                (false, true) => unreachable!("validated zone island policy"),
            };
            let minimum_area = zone
                .islands
                .minimum_area
                .as_ref()
                .map(|area| {
                    projector.value(
                        &format!("zone.{}.islands.minimum_area", zone.id.as_str()),
                        area,
                    )
                })
                .transpose()?
                .unwrap_or_default();
            writeln!(
                svg,
                "<polygon data-source=\"{}\" data-net=\"{}\" data-layer=\"{}\" data-priority=\"{}\" data-clearance=\"{}\" data-connection=\"{}\" data-island-mode=\"{}\" data-island-area-min=\"{}\" points=\"{}\" fill=\"{}\" stroke=\"#b87333\" stroke-width=\"0.4\"/>",
                xml(zone.id.as_str()),
                xml(zone.net.as_str()),
                zone.layer.0,
                zone.priority,
                clearance,
                connection,
                island_mode,
                minimum_area,
                svg_points(points),
                fill,
            )
            .expect("writing to String cannot fail");
        }
        for (route, width, segments) in &routes {
            for segment in segments {
                match segment {
                    RenderedRouteSegment::Line { start, end } => writeln!(
                        svg,
                        "<line data-source=\"{}\" data-net=\"{}\" data-layer=\"{}\" x1=\"{}\" y1=\"{}\" x2=\"{}\" y2=\"{}\" stroke=\"#f4a261\" stroke-width=\"{width}\"/>",
                        xml(route.id.as_str()),
                        xml(route.net.as_str()),
                        route.layer.0,
                        start.0,
                        start.1,
                        end.0,
                        end.1
                    ),
                    RenderedRouteSegment::Arc {
                        center,
                        radius,
                        sweep_class: ExplicitArcSweepClass::FullCircle,
                        ..
                    } => writeln!(
                        svg,
                        "<circle data-source=\"{}\" data-net=\"{}\" data-layer=\"{}\" cx=\"{}\" cy=\"{}\" r=\"{radius}\" fill=\"none\" stroke=\"#f4a261\" stroke-width=\"{width}\"/>",
                        xml(route.id.as_str()),
                        xml(route.net.as_str()),
                        route.layer.0,
                        center.0,
                        center.1
                    ),
                    RenderedRouteSegment::Arc {
                        radius,
                        start,
                        end,
                        sweep,
                        sweep_class,
                        ..
                    } => writeln!(
                        svg,
                        "<path data-source=\"{}\" data-net=\"{}\" data-layer=\"{}\" d=\"M {} {} A {radius} {radius} 0 {} {} {} {}\" fill=\"none\" stroke=\"#f4a261\" stroke-width=\"{width}\"/>",
                        xml(route.id.as_str()),
                        xml(route.net.as_str()),
                        route.layer.0,
                        start.0,
                        start.1,
                        usize::from(*sweep_class == ExplicitArcSweepClass::GreaterThanHalfTurn),
                        usize::from(*sweep),
                        end.0,
                        end.1
                    ),
                    RenderedRouteSegment::CubicBezier {
                        start,
                        control0,
                        control1,
                        end,
                    } => writeln!(
                        svg,
                        "<path data-source=\"{}\" data-net=\"{}\" data-layer=\"{}\" d=\"M {} {} C {} {}, {} {}, {} {}\" fill=\"none\" stroke=\"#f4a261\" stroke-width=\"{width}\"/>",
                        xml(route.id.as_str()),
                        xml(route.net.as_str()),
                        route.layer.0,
                        start.0,
                        start.1,
                        control0.0,
                        control0.1,
                        control1.0,
                        control1.1,
                        end.0,
                        end.1
                    ),
                }
                .expect("writing to String cannot fail");
            }
        }
        for (via, (x, y), diameter, drill) in &vias {
            writeln!(
                svg,
                "<circle data-source=\"{}\" data-net=\"{}\" cx=\"{x}\" cy=\"{y}\" r=\"{}\" fill=\"#f4a261\" stroke=\"#111\" stroke-width=\"0.3\"/>",
                xml(via.id.as_str()),
                xml(via.net.as_str()),
                diameter / 2.0
            )
            .expect("writing to String cannot fail");
            writeln!(
                svg,
                "<circle cx=\"{x}\" cy=\"{y}\" r=\"{}\" fill=\"#fff\"/>",
                drill / 2.0
            )
            .expect("writing to String cannot fail");
        }
        for pad in &placed_pads {
            let attributes = format!(
                "data-instance=\"{}\" data-land-pattern=\"{}\" data-pad=\"{}\"",
                xml(&pad.instance),
                xml(&pad.pattern),
                xml(&pad.pad)
            );
            match &pad.shape {
                RenderedPadShape::Circle { diameter } => writeln!(
                    svg,
                    "<circle {attributes} cx=\"{}\" cy=\"{}\" r=\"{}\" fill=\"#f4a261\" stroke=\"#111\" stroke-width=\"0.25\"/>",
                    pad.center.0,
                    pad.center.1,
                    diameter / 2.0
                ),
                RenderedPadShape::Rectangle {
                    width,
                    height,
                    radius,
                } => writeln!(
                    svg,
                    "<rect {attributes} x=\"{}\" y=\"{}\" width=\"{width}\" height=\"{height}\" rx=\"{radius}\" transform=\"rotate({} {} {})\" fill=\"#f4a261\" stroke=\"#111\" stroke-width=\"0.25\"/>",
                    pad.center.0 - width / 2.0,
                    pad.center.1 - height / 2.0,
                    pad.rotation,
                    pad.center.0,
                    pad.center.1
                ),
                RenderedPadShape::Polygon { points } => writeln!(
                    svg,
                    "<polygon {attributes} points=\"{}\" fill=\"#f4a261\" stroke=\"#111\" stroke-width=\"0.25\"/>",
                    svg_points(points)
                ),
            }
            .expect("writing to String cannot fail");
            if let Some(drill) = &pad.drill {
                match drill {
                    RenderedDrill::Round { diameter } => writeln!(
                        svg,
                        "<circle cx=\"{}\" cy=\"{}\" r=\"{}\" fill=\"#fff\"/>",
                        pad.center.0,
                        pad.center.1,
                        diameter / 2.0
                    ),
                    RenderedDrill::Slot { start, end, width } => writeln!(
                        svg,
                        "<line x1=\"{}\" y1=\"{}\" x2=\"{}\" y2=\"{}\" stroke=\"#fff\" stroke-width=\"{width}\"/>",
                        start.0, start.1, end.0, end.1
                    ),
                }
                .expect("writing to String cannot fail");
            }
        }
        for (placement, (x, y)) in &placements {
            writeln!(
                svg,
                "<g data-instance=\"{}\" stroke=\"#63c5da\" stroke-width=\"0.5\"><line x1=\"{}\" y1=\"{y}\" x2=\"{}\" y2=\"{y}\"/><line x1=\"{x}\" y1=\"{}\" x2=\"{x}\" y2=\"{}\"/></g>",
                xml(placement.instance.as_str()),
                x - 2.0,
                x + 2.0,
                y - 2.0,
                y + 2.0
            )
            .expect("writing to String cannot fail");
        }
        svg.push_str("</g>\n</svg>\n");
        Ok(PcbSvgReport {
            svg,
            projections: projector.projections,
        })
    }
}

enum RenderedRouteSegment {
    Line {
        start: (f64, f64),
        end: (f64, f64),
    },
    Arc {
        center: (f64, f64),
        radius: f64,
        start: (f64, f64),
        end: (f64, f64),
        sweep: bool,
        sweep_class: ExplicitArcSweepClass,
    },
    CubicBezier {
        start: (f64, f64),
        control0: (f64, f64),
        control1: (f64, f64),
        end: (f64, f64),
    },
}

enum RenderedPadShape {
    Circle {
        diameter: f64,
    },
    Rectangle {
        width: f64,
        height: f64,
        radius: f64,
    },
    Polygon {
        points: Vec<(f64, f64)>,
    },
}

struct RenderedPad {
    instance: String,
    pattern: String,
    pad: String,
    center: (f64, f64),
    rotation: f64,
    shape: RenderedPadShape,
    drill: Option<RenderedDrill>,
}

enum RenderedDrill {
    Round {
        diameter: f64,
    },
    Slot {
        start: (f64, f64),
        end: (f64, f64),
        width: f64,
    },
}

fn transform(
    local: (f64, f64),
    origin: (f64, f64),
    rotation_degrees: f64,
    side: BoardSide,
) -> (f64, f64) {
    let local_x = match side {
        BoardSide::Front => local.0,
        BoardSide::Back => -local.0,
    };
    let angle = rotation_degrees.to_radians();
    let cosine = angle.cos();
    let sine = angle.sin();
    (
        origin.0 + local_x * cosine - local.1 * sine,
        origin.1 + local_x * sine + local.1 * cosine,
    )
}

fn rotate(point: (f64, f64), rotation_degrees: f64) -> (f64, f64) {
    let angle = rotation_degrees.to_radians();
    let cosine = angle.cos();
    let sine = angle.sin();
    (
        point.0 * cosine - point.1 * sine,
        point.0 * sine + point.1 * cosine,
    )
}

struct Projector {
    decimal_places: usize,
    projections: Vec<PcbSvgProjection>,
}

impl Projector {
    fn new(decimal_places: usize) -> Self {
        Self {
            decimal_places,
            projections: Vec::new(),
        }
    }

    fn points(&mut self, field: &str, points: &[Point2]) -> Result<Vec<(f64, f64)>, PcbSvgError> {
        points
            .iter()
            .enumerate()
            .map(|(index, point)| self.point(&format!("{field}[{index}]"), point))
            .collect()
    }

    fn contour(
        &mut self,
        field: &str,
        contour: &crate::BoardContour,
    ) -> Result<Vec<RenderedRouteSegment>, PcbSvgError> {
        contour
            .segments()
            .iter()
            .enumerate()
            .map(|(index, segment)| match segment {
                crate::BoardContourSegment::Line(line) => Ok(RenderedRouteSegment::Line {
                    start: self.point(&format!("{field}[{index}].start"), line.start())?,
                    end: self.point(&format!("{field}[{index}].end"), line.end())?,
                }),
                crate::BoardContourSegment::CircularArc(arc) => Ok(RenderedRouteSegment::Arc {
                    center: self.point(&format!("{field}[{index}].center"), arc.center())?,
                    radius: self.value(&format!("{field}[{index}].radius"), arc.radius())?,
                    start: self.point(&format!("{field}[{index}].start"), arc.start())?,
                    end: self.point(&format!("{field}[{index}].end"), arc.end())?,
                    sweep: arc.direction() != ArcDirection::Ccw,
                    sweep_class: arc.facts().sweep_class,
                }),
                crate::BoardContourSegment::CubicBezier(bezier) => {
                    Ok(RenderedRouteSegment::CubicBezier {
                        start: self.point(&format!("{field}[{index}].start"), bezier.start())?,
                        control0: self
                            .point(&format!("{field}[{index}].control0"), bezier.control0())?,
                        control1: self
                            .point(&format!("{field}[{index}].control1"), bezier.control1())?,
                        end: self.point(&format!("{field}[{index}].end"), bezier.end())?,
                    })
                }
            })
            .collect()
    }

    fn point(&mut self, field: &str, point: &Point2) -> Result<(f64, f64), PcbSvgError> {
        Ok((
            self.value(&format!("{field}.x"), &point.x)?,
            self.value(&format!("{field}.y"), &point.y)?,
        ))
    }

    fn value(&mut self, field: &str, value: &Real) -> Result<f64, PcbSvgError> {
        let Some(finite) = value.to_f64_lossy().filter(|value| value.is_finite()) else {
            return Err(PcbSvgError::NonFinite(field.into()));
        };
        self.projections.push(PcbSvgProjection {
            field: field.into(),
            source: value.to_string(),
            emitted: format!("{:.*}", self.decimal_places, finite),
        });
        Ok(finite)
    }
}

fn include_contour_bounds(bounds: &mut Bounds, contour: &[RenderedRouteSegment]) {
    for segment in contour {
        match segment {
            RenderedRouteSegment::Line { start, end } => {
                bounds.include(start.0, start.1);
                bounds.include(end.0, end.1);
            }
            RenderedRouteSegment::Arc { center, radius, .. } => {
                bounds.include(center.0 - radius, center.1 - radius);
                bounds.include(center.0 + radius, center.1 + radius);
            }
            RenderedRouteSegment::CubicBezier {
                start,
                control0,
                control1,
                end,
            } => {
                for point in [start, control0, control1, end] {
                    bounds.include(point.0, point.1);
                }
            }
        }
    }
}

fn svg_contour_path(contour: &[RenderedRouteSegment]) -> String {
    let Some(first) = contour.first() else {
        return String::new();
    };
    let start = match first {
        RenderedRouteSegment::Line { start, .. }
        | RenderedRouteSegment::Arc { start, .. }
        | RenderedRouteSegment::CubicBezier { start, .. } => start,
    };
    let mut path = format!("M {} {}", start.0, start.1);
    for segment in contour {
        match segment {
            RenderedRouteSegment::Line { end, .. } => {
                path.push_str(&format!(" L {} {}", end.0, end.1));
            }
            RenderedRouteSegment::Arc {
                radius,
                end,
                sweep,
                sweep_class,
                ..
            } => {
                path.push_str(&format!(
                    " A {radius} {radius} 0 {} {} {} {}",
                    usize::from(*sweep_class == ExplicitArcSweepClass::GreaterThanHalfTurn),
                    usize::from(*sweep),
                    end.0,
                    end.1
                ));
            }
            RenderedRouteSegment::CubicBezier {
                control0,
                control1,
                end,
                ..
            } => {
                path.push_str(&format!(
                    " C {} {}, {} {}, {} {}",
                    control0.0, control0.1, control1.0, control1.1, end.0, end.1
                ));
            }
        }
    }
    path.push_str(" Z");
    path
}

#[derive(Default)]
struct Bounds {
    min_x: f64,
    min_y: f64,
    max_x: f64,
    max_y: f64,
    valid: bool,
}

impl Bounds {
    fn include(&mut self, x: f64, y: f64) {
        if self.valid {
            self.min_x = self.min_x.min(x);
            self.min_y = self.min_y.min(y);
            self.max_x = self.max_x.max(x);
            self.max_y = self.max_y.max(y);
        } else {
            self.min_x = x;
            self.min_y = y;
            self.max_x = x;
            self.max_y = y;
            self.valid = true;
        }
    }
}

fn svg_points(points: &[(f64, f64)]) -> String {
    points
        .iter()
        .map(|(x, y)| format!("{x},{y}"))
        .collect::<Vec<_>>()
        .join(" ")
}

fn xml(value: &str) -> String {
    value
        .replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
        .replace('\'', "&apos;")
}
