//! Nonlinear and event-device report surfaces.
//!
//! Nonlinear circuit devices need numeric Newton proposals in practical
//! simulators, but their model domains, monotonicity/slope facts, parameters,
//! and event decisions should be explicit before a solver is trusted. This
//! module therefore records exact model/evaluation metadata for diodes, MOSFET
//! placeholders, piecewise-linear devices, switches, and protection devices;
//! it does not perform Newton iteration. The README collects the supporting
//! circuit-simulation and exact-computation references.

use hyperreal::Real;

use crate::{CircuitParameter, ComponentId};

/// Nonlinear device family.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum NonlinearDeviceKind {
    /// Diode model.
    Diode,
    /// MOSFET placeholder model.
    MosfetPlaceholder,
    /// Piecewise-linear source or device.
    PiecewiseLinear,
    /// Ideal or controlled switch.
    Switch,
    /// Protection device such as TVS, fuse, or clamp placeholder.
    Protection,
}

/// Event policy for switches and protection devices.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum EventPolicy {
    /// Event state is fixed by caller policy.
    Fixed,
    /// Event candidate must be replayed through exact residuals.
    ExactReplayRequired,
    /// Event comes from a lossy adapter proposal.
    LossyAdapterProposal,
}

/// Switch state exposed to the solver policy.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SwitchState {
    /// Switch is open.
    Open,
    /// Switch is closed.
    Closed,
    /// Switch state was proposed by an adapter and still needs replay.
    Proposed,
}

/// Exact piecewise-linear segment.
#[derive(Clone, Debug, PartialEq)]
pub struct PiecewiseLinearSegment {
    /// Lower domain bound.
    pub lower: Real,
    /// Upper domain bound.
    pub upper: Real,
    /// Exact slope.
    pub slope: Real,
    /// Exact intercept.
    pub intercept: Real,
}

/// Nonlinear device report before Newton or event lowering.
#[derive(Clone, Debug, PartialEq)]
pub struct NonlinearDeviceReport {
    /// Component id.
    pub component: ComponentId,
    /// Device family.
    pub kind: NonlinearDeviceKind,
    /// Exact parameter provenance.
    pub parameters: Vec<CircuitParameter>,
    /// Human-readable domain assumptions.
    pub domains: Vec<String>,
    /// Human-readable monotonicity or slope facts.
    pub slope_facts: Vec<String>,
    /// Optional event policy.
    pub event_policy: Option<EventPolicy>,
    /// Optional switch state.
    pub switch_state: Option<SwitchState>,
    /// Piecewise-linear segments when available.
    pub segments: Vec<PiecewiseLinearSegment>,
}

impl NonlinearDeviceReport {
    /// Creates a diode model-domain report with exact parameter provenance.
    pub fn diode(component: ComponentId, parameters: Vec<CircuitParameter>) -> Self {
        Self {
            component,
            kind: NonlinearDeviceKind::Diode,
            parameters,
            domains: vec![
                "diode exponential model; Newton proposal requires residual replay".into(),
            ],
            slope_facts: vec![
                "forward branch monotone under positive saturation-current assumptions".into(),
            ],
            event_policy: None,
            switch_state: None,
            segments: Vec::new(),
        }
    }

    /// Creates a MOSFET placeholder report with explicit unsupported-domain status.
    pub fn mosfet_placeholder(component: ComponentId, parameters: Vec<CircuitParameter>) -> Self {
        Self {
            component,
            kind: NonlinearDeviceKind::MosfetPlaceholder,
            parameters,
            domains: vec!["MOSFET equations not yet lowered to exact residual blocks".into()],
            slope_facts: vec!["model requires adapter proposal or future exact device law".into()],
            event_policy: None,
            switch_state: None,
            segments: Vec::new(),
        }
    }

    /// Creates a piecewise-linear report from exact segments.
    pub fn piecewise_linear(component: ComponentId, segments: Vec<PiecewiseLinearSegment>) -> Self {
        let slope_facts = segments
            .iter()
            .enumerate()
            .map(|(index, _)| format!("segment {index} has exact slope and intercept"))
            .collect();
        Self {
            component,
            kind: NonlinearDeviceKind::PiecewiseLinear,
            parameters: Vec::new(),
            domains: vec!["piecewise-linear domains are exact interval endpoints".into()],
            slope_facts,
            event_policy: None,
            switch_state: None,
            segments,
        }
    }

    /// Creates a switch/event report with visible policy.
    pub fn switch(component: ComponentId, event_policy: EventPolicy, state: SwitchState) -> Self {
        Self {
            component,
            kind: NonlinearDeviceKind::Switch,
            parameters: Vec::new(),
            domains: vec!["switch event state is policy-visible".into()],
            slope_facts: vec!["open/closed topology change is not hidden in tolerance".into()],
            event_policy: Some(event_policy),
            switch_state: Some(state),
            segments: Vec::new(),
        }
    }
}
