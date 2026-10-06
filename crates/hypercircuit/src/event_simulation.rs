//! Exact discrete-event carriers and deterministic simulation agenda.
//!
//! This module retains scheduling intent and lifecycle evidence independently
//! from the analog solver. Events use exact [`Real`] timestamps and explicit
//! `(time, phase, sequence)` ordering; no tolerance or container iteration
//! order participates in execution.

use crate::predicate::RealPredicateExt as _;
use std::cmp::Ordering;
use std::collections::{BTreeMap, VecDeque};
use std::fmt::{Display, Formatter};

use crate::{
    BundleEndpointId, BundleMemberId, Circuit, CircuitEventId, CircuitId, CircuitLibrary,
    ComponentId, NetId, PortId, RandomDrawId, Real, SignalBundleLibrary, SourceWaveform,
    SubcircuitInstanceId, SwitchState,
};

/// Failure to obtain an exactly ordered simulation breakpoint.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ExactBreakpointError {
    /// Stable provider identity.
    pub provider: String,
    /// Provider-specific failure detail.
    pub detail: String,
}

impl Display for ExactBreakpointError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        write!(
            formatter,
            "breakpoint provider {} failed: {}",
            self.provider, self.detail
        )
    }
}

impl std::error::Error for ExactBreakpointError {}

/// Source of exact discontinuity or event times for transient integration.
pub trait ExactBreakpointProvider {
    /// Stable provider identity used in diagnostics and run evidence.
    fn breakpoint_provider_id(&self) -> String;

    /// Returns the first breakpoint strictly after `time`.
    fn next_breakpoint_after(&self, time: &Real) -> Result<Option<Real>, ExactBreakpointError>;
}

impl ExactBreakpointProvider for SourceWaveform {
    fn breakpoint_provider_id(&self) -> String {
        "source-waveform".into()
    }

    fn next_breakpoint_after(&self, time: &Real) -> Result<Option<Real>, ExactBreakpointError> {
        SourceWaveform::next_breakpoint_after(self, time).map_err(|error| ExactBreakpointError {
            provider: self.breakpoint_provider_id(),
            detail: error.to_string(),
        })
    }
}

/// Four-state digital value retained at mixed-signal boundaries.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LogicValue {
    /// Logic low.
    Low,
    /// Logic high.
    High,
    /// Unknown or conflicting value.
    Unknown,
    /// High-impedance value.
    HighImpedance,
}

/// Deterministic processing phase for simultaneous events.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Copy, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum CircuitEventPhase {
    /// External stimuli and trace inputs are applied first.
    Stimulus,
    /// Topology-affecting switch and protection changes.
    Topology,
    /// Behavioral work after the analog endpoint has been solved.
    PostSolve,
    /// Threshold and state observations derived from the solved endpoint.
    Detection,
    /// Non-semantic probes and presentation notifications.
    Observation,
}

/// Stable typed address participating in a circuit event.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
pub enum CircuitEventTarget {
    /// Whole reusable circuit definition.
    Circuit(CircuitId),
    /// One component in the active circuit scope.
    Component(ComponentId),
    /// One electrical net.
    Net(NetId),
    /// One circuit boundary port.
    Port {
        /// Owning circuit definition.
        circuit: CircuitId,
        /// Boundary port.
        port: PortId,
    },
    /// One member of a directional signal-bundle endpoint.
    BundleMember {
        /// Owning circuit definition.
        circuit: CircuitId,
        /// Bundle endpoint.
        endpoint: BundleEndpointId,
        /// Named bundle member.
        member: BundleMemberId,
    },
    /// Hierarchical child scope.
    Hierarchy(Vec<SubcircuitInstanceId>),
    /// Explicit external adapter endpoint.
    External(String),
}

/// Serializable value available to typed behavioral events.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub enum CircuitEventValue {
    /// Exact numeric value.
    Real(Real),
    /// Signed integer value.
    Integer(i64),
    /// Boolean value.
    Boolean(bool),
    /// Four-state logic value.
    Logic(LogicValue),
    /// Retained text value.
    Text(String),
}

/// Closed, serializable circuit-event payload family.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub enum CircuitEventKind {
    /// Digital value transition.
    DigitalTransition { value: LogicValue },
    /// Exact independent-source override.
    SourceOverride { value: Real },
    /// Ideal/controlled switch state change.
    SwitchTransition { state: SwitchState },
    /// Protection device activation or release.
    ProtectionState { active: bool },
    /// Timer expiration identified within its behavioral component.
    Timer { key: String },
    /// Retained external trace sample.
    TraceSample { channel: String, value: Real },
    /// Domain extension with a stable kind and typed fields.
    Behavioral {
        /// Stable behavior-defined kind.
        kind: String,
        /// Deterministically ordered typed fields.
        fields: BTreeMap<String, CircuitEventValue>,
    },
}

/// Auditable cause of a scheduled event.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub enum CircuitEventCause {
    /// Authored root stimulus or testbench action.
    Authored { provenance: String },
    /// Event derived while handling an earlier event.
    Event(CircuitEventId),
    /// Event derived from an exact analog threshold observation.
    AnalogThreshold {
        /// Observed net.
        net: NetId,
        /// Exact threshold.
        threshold: Real,
        /// Whether the triggering transition was rising.
        rising: bool,
    },
    /// Event derived from a reproducible stochastic draw.
    RandomDraw(RandomDrawId),
    /// Replacement for an obsolete pending event.
    RescheduledFrom(CircuitEventId),
}

/// Validated request to schedule one circuit event.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct CircuitEventRequest {
    /// Exact absolute simulation time.
    pub time: Real,
    /// Explicit simultaneous-event phase.
    pub phase: CircuitEventPhase,
    /// Optional originating circuit identity.
    pub source: Option<CircuitEventTarget>,
    /// Required destination identity.
    pub target: CircuitEventTarget,
    /// Typed retained payload.
    pub kind: CircuitEventKind,
    /// Retained causal provenance.
    pub cause: CircuitEventCause,
}

/// Circuit- and hierarchy-aware validation problem for an event request.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum CircuitEventValidationIssue {
    /// A circuit-qualified address names another circuit.
    WrongCircuit(CircuitId),
    /// A component address is absent.
    UnknownComponent(ComponentId),
    /// A net address is absent.
    UnknownNet(NetId),
    /// A boundary-port address is absent.
    UnknownPort(PortId),
    /// Bundle validation was requested without the corresponding contract library.
    MissingBundleLibrary,
    /// A bundle endpoint is absent from the addressed circuit.
    UnknownBundleEndpoint(BundleEndpointId),
    /// A bundle endpoint does not expose the addressed member.
    UnknownBundleMember {
        /// Bundle endpoint.
        endpoint: BundleEndpointId,
        /// Requested member.
        member: BundleMemberId,
    },
    /// A hierarchy instance path cannot be followed from the library root.
    UnknownHierarchyPath(Vec<SubcircuitInstanceId>),
    /// A source override targets a component without retained source stimulus.
    UnknownSourceStimulus(ComponentId),
}

impl CircuitEventRequest {
    /// Validates local circuit and optional signal-bundle identities.
    pub fn validate_against(
        &self,
        circuit: &Circuit,
        bundles: Option<&SignalBundleLibrary>,
    ) -> Vec<CircuitEventValidationIssue> {
        let mut issues = Vec::new();
        if let Some(source) = &self.source {
            validate_target(source, circuit, bundles, &mut issues);
        }
        validate_target(&self.target, circuit, bundles, &mut issues);
        if matches!(self.kind, CircuitEventKind::SourceOverride { .. })
            && let CircuitEventTarget::Component(component) = &self.target
            && !circuit
                .source_stimuli
                .iter()
                .any(|stimulus| stimulus.component == *component)
        {
            issues.push(CircuitEventValidationIssue::UnknownSourceStimulus(
                component.clone(),
            ));
        }
        issues
    }

    /// Validates a hierarchy path against reusable circuit definitions.
    pub fn validate_hierarchy(&self, library: &CircuitLibrary) -> Vec<CircuitEventValidationIssue> {
        let mut issues = Vec::new();
        for target in self.source.iter().chain(std::iter::once(&self.target)) {
            let CircuitEventTarget::Hierarchy(path) = target else {
                continue;
            };
            let mut circuit_id = &library.root;
            let mut valid = true;
            for instance_id in path {
                let Some(circuit) = library
                    .circuits
                    .iter()
                    .find(|circuit| &circuit.id == circuit_id)
                else {
                    valid = false;
                    break;
                };
                let Some(instance) = circuit
                    .subcircuits
                    .iter()
                    .find(|instance| &instance.id == instance_id)
                else {
                    valid = false;
                    break;
                };
                circuit_id = &instance.circuit;
            }
            if !valid
                || !library
                    .circuits
                    .iter()
                    .any(|circuit| &circuit.id == circuit_id)
            {
                issues.push(CircuitEventValidationIssue::UnknownHierarchyPath(
                    path.clone(),
                ));
            }
        }
        issues
    }
}

fn validate_target(
    target: &CircuitEventTarget,
    circuit: &Circuit,
    bundles: Option<&SignalBundleLibrary>,
    issues: &mut Vec<CircuitEventValidationIssue>,
) {
    match target {
        CircuitEventTarget::Circuit(id) if id != &circuit.id => {
            issues.push(CircuitEventValidationIssue::WrongCircuit(id.clone()));
        }
        CircuitEventTarget::Component(id)
            if !circuit
                .instances
                .iter()
                .any(|instance| &instance.component == id) =>
        {
            issues.push(CircuitEventValidationIssue::UnknownComponent(id.clone()));
        }
        CircuitEventTarget::Net(id) if !circuit.nets.iter().any(|net| &net.id == id) => {
            issues.push(CircuitEventValidationIssue::UnknownNet(id.clone()));
        }
        CircuitEventTarget::Port {
            circuit: owner,
            port,
        } => {
            if owner != &circuit.id {
                issues.push(CircuitEventValidationIssue::WrongCircuit(owner.clone()));
            } else if !circuit.ports.iter().any(|candidate| &candidate.id == port) {
                issues.push(CircuitEventValidationIssue::UnknownPort(port.clone()));
            }
        }
        CircuitEventTarget::BundleMember {
            circuit: owner,
            endpoint,
            member,
        } => {
            if owner != &circuit.id {
                issues.push(CircuitEventValidationIssue::WrongCircuit(owner.clone()));
            } else if let Some(bundles) = bundles {
                let Some(endpoint_contract) = bundles
                    .endpoints
                    .iter()
                    .find(|candidate| &candidate.circuit == owner && &candidate.id == endpoint)
                else {
                    issues.push(CircuitEventValidationIssue::UnknownBundleEndpoint(
                        endpoint.clone(),
                    ));
                    return;
                };
                if !endpoint_contract
                    .ports
                    .iter()
                    .any(|binding| &binding.member == member)
                {
                    issues.push(CircuitEventValidationIssue::UnknownBundleMember {
                        endpoint: endpoint.clone(),
                        member: member.clone(),
                    });
                }
            } else {
                issues.push(CircuitEventValidationIssue::MissingBundleLibrary);
            }
        }
        _ => {}
    }
}

/// One scheduled event with deterministic identity and sequence.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct CircuitEvent {
    /// Stable event identity.
    pub id: CircuitEventId,
    /// Exact absolute simulation time.
    pub time: Real,
    /// Explicit simultaneous-event phase.
    pub phase: CircuitEventPhase,
    /// Monotonic tie-break sequence.
    pub sequence: u64,
    /// Optional originating circuit identity.
    pub source: Option<CircuitEventTarget>,
    /// Required destination identity.
    pub target: CircuitEventTarget,
    /// Typed retained payload.
    pub kind: CircuitEventKind,
    /// Retained causal provenance.
    pub cause: CircuitEventCause,
}

impl CircuitEvent {
    fn from_request(id: CircuitEventId, request: CircuitEventRequest) -> Self {
        Self {
            id,
            time: request.time,
            phase: request.phase,
            sequence: id.get(),
            source: request.source,
            target: request.target,
            kind: request.kind,
            cause: request.cause,
        }
    }
}

/// Event lifecycle transition retained by the agenda.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum CircuitEventLifecycleAction {
    /// Event entered the pending agenda.
    Scheduled,
    /// Pending event was explicitly canceled.
    Canceled,
    /// Event was removed and replaced atomically.
    Rescheduled,
    /// Event was selected for delivery.
    Delivered,
}

/// Replayable event lifecycle evidence.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct CircuitEventLifecycleRecord {
    /// Event affected by this transition.
    pub event: CircuitEvent,
    /// Lifecycle action.
    pub action: CircuitEventLifecycleAction,
    /// Replacement event for a reschedule action.
    pub related: Option<CircuitEventId>,
}

/// Aggregate deterministic agenda counters.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct CircuitEventCounters {
    /// Events successfully scheduled.
    pub scheduled: u64,
    /// Events explicitly canceled without replacement.
    pub canceled: u64,
    /// Atomic event replacements.
    pub rescheduled: u64,
    /// Events selected for delivery.
    pub delivered: u64,
    /// Events accepted through the preordered trace path.
    pub preordered: u64,
    /// Largest pending agenda size.
    pub maximum_pending: usize,
}

/// Lifecycle replay inconsistency.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum CircuitEventReplayIssue {
    /// An event identity was scheduled more than once.
    DuplicateSchedule(CircuitEventId),
    /// A cancellation, reschedule, or delivery referenced no pending event.
    UnknownTransition(CircuitEventId),
    /// Lifecycle evidence for an existing identity changed event content.
    EventContentMismatch(CircuitEventId),
    /// Reschedule evidence did not link the old and replacement identities.
    InvalidRescheduleRelation(CircuitEventId),
    /// Replayed event times could not be ordered exactly.
    IndeterminateOrder,
    /// Replayed pending events differ from retained agenda state.
    PendingMismatch,
    /// Replayed lifecycle counters differ from retained counters.
    CounterMismatch,
}

/// Exact replay result for retained event lifecycle evidence.
#[derive(Clone, Debug, PartialEq)]
pub struct CircuitEventReplayReport {
    /// Every lifecycle inconsistency in evidence order.
    pub issues: Vec<CircuitEventReplayIssue>,
    /// Pending events reconstructed from lifecycle evidence.
    pub pending: Vec<CircuitEvent>,
    /// Counters reconstructed from lifecycle evidence.
    pub counters: CircuitEventCounters,
}

impl CircuitEventReplayReport {
    /// True when lifecycle evidence exactly reconstructs agenda state.
    pub fn is_valid(&self) -> bool {
        self.issues.is_empty()
    }
}

/// Deterministic fingerprint of retained agenda state and evidence.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CircuitSimulationFingerprint {
    /// Stable fingerprint algorithm identifier.
    pub algorithm: String,
    /// Lowercase hexadecimal fingerprint.
    pub value: String,
}

/// Exact-time scheduling failure.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum CircuitEventAgendaError {
    /// Event time cannot be ordered exactly.
    IndeterminateTime,
    /// Event was scheduled before the agenda clock.
    EventInPast,
    /// Advancing the agenda clock would skip a pending event.
    ClockWouldSkipEvent,
    /// A same-time event was scheduled into a phase that has already completed.
    PhaseInPast,
    /// Selected event identity is not pending.
    UnknownEvent(CircuitEventId),
    /// Preordered requests were not in nondecreasing `(time, phase)` order.
    TraceOutOfOrder { index: usize },
    /// Event identity space was exhausted.
    IdentityExhausted,
    /// A retained agenda counter was exhausted.
    CounterExhausted,
    /// Retained agenda state violated an internal ordering invariant.
    InconsistentState,
    /// A required retained name or provenance string was blank.
    EmptyRetainedText { field: String },
}

impl Display for CircuitEventAgendaError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::IndeterminateTime => formatter.write_str("circuit event time is indeterminate"),
            Self::EventInPast => formatter.write_str("cannot schedule a circuit event in the past"),
            Self::ClockWouldSkipEvent => {
                formatter.write_str("cannot advance agenda clock past a pending event")
            }
            Self::PhaseInPast => {
                formatter.write_str("cannot schedule into an already completed same-time phase")
            }
            Self::UnknownEvent(id) => write!(formatter, "unknown pending event {}", id.get()),
            Self::TraceOutOfOrder { index } => {
                write!(
                    formatter,
                    "preordered event trace is out of order at {index}"
                )
            }
            Self::IdentityExhausted => {
                formatter.write_str("circuit event identity space exhausted")
            }
            Self::CounterExhausted => formatter.write_str("circuit event counter exhausted"),
            Self::InconsistentState => formatter.write_str("circuit event agenda is inconsistent"),
            Self::EmptyRetainedText { field } => {
                write!(formatter, "circuit event field {field} must not be blank")
            }
        }
    }
}

impl std::error::Error for CircuitEventAgendaError {}

/// Exact deterministic pending-event agenda with retained lifecycle evidence.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct CircuitEventAgenda {
    now: Real,
    #[cfg_attr(feature = "interchange", serde(default))]
    last_delivered_phase: Option<CircuitEventPhase>,
    next_id: u64,
    pending: VecDeque<CircuitEvent>,
    lifecycle: Vec<CircuitEventLifecycleRecord>,
    counters: CircuitEventCounters,
}

impl CircuitEventAgenda {
    /// Creates an empty agenda at an exact simulation time.
    pub fn new(now: Real) -> Self {
        Self {
            now,
            last_delivered_phase: None,
            next_id: 0,
            pending: VecDeque::new(),
            lifecycle: Vec::new(),
            counters: CircuitEventCounters::default(),
        }
    }

    /// Returns the exact agenda clock.
    pub fn time(&self) -> &Real {
        &self.now
    }

    /// Returns pending events in deterministic execution order.
    pub fn pending(&self) -> &VecDeque<CircuitEvent> {
        &self.pending
    }

    /// Returns retained lifecycle evidence in action order.
    pub fn lifecycle(&self) -> &[CircuitEventLifecycleRecord] {
        &self.lifecycle
    }

    /// Returns aggregate agenda counters.
    pub fn counters(&self) -> &CircuitEventCounters {
        &self.counters
    }

    /// Returns the next pending event without advancing the agenda.
    pub fn peek(&self) -> Option<&CircuitEvent> {
        self.pending.front()
    }

    /// Advances the exact agenda clock without delivering an event.
    ///
    /// Sessions use this after an analog-only endpoint. Advancing cannot move
    /// backward or skip a pending event.
    pub fn advance_time(&mut self, time: Real) -> Result<(), CircuitEventAgendaError> {
        match time.predicate_cmp(&self.now) {
            Some(Ordering::Less) => return Err(CircuitEventAgendaError::EventInPast),
            Some(Ordering::Equal) => return Ok(()),
            Some(Ordering::Greater) => {}
            None => return Err(CircuitEventAgendaError::IndeterminateTime),
        }
        if let Some(next) = self.pending.front() {
            match next.time.predicate_cmp(&time) {
                Some(Ordering::Less) => {
                    return Err(CircuitEventAgendaError::ClockWouldSkipEvent);
                }
                Some(Ordering::Equal | Ordering::Greater) => {}
                None => return Err(CircuitEventAgendaError::IndeterminateTime),
            }
        }
        self.now = time;
        self.last_delivered_phase = None;
        Ok(())
    }

    /// Schedules one event atomically.
    pub fn schedule(
        &mut self,
        request: CircuitEventRequest,
    ) -> Result<CircuitEventId, CircuitEventAgendaError> {
        validate_request(&request)?;
        self.validate_future_time(&request.time)?;
        self.validate_future_phase(&request.time, request.phase)?;
        let id = CircuitEventId::new(self.next_id);
        let next_id = self
            .next_id
            .checked_add(1)
            .ok_or(CircuitEventAgendaError::IdentityExhausted)?;
        let scheduled = self
            .counters
            .scheduled
            .checked_add(1)
            .ok_or(CircuitEventAgendaError::CounterExhausted)?;
        let event = CircuitEvent::from_request(id, request);
        let position = self.insertion_position(&event)?;
        self.pending.insert(position, event.clone());
        self.next_id = next_id;
        self.lifecycle.push(CircuitEventLifecycleRecord {
            event,
            action: CircuitEventLifecycleAction::Scheduled,
            related: None,
        });
        self.counters.scheduled = scheduled;
        self.counters.maximum_pending = self.counters.maximum_pending.max(self.pending.len());
        Ok(id)
    }

    /// Atomically appends a trace already sorted by exact time and phase.
    ///
    /// The agenda still validates ordering and merges the trace with existing
    /// pending events. No request is retained when any request is invalid.
    pub fn schedule_preordered<I>(
        &mut self,
        requests: I,
    ) -> Result<Vec<CircuitEventId>, CircuitEventAgendaError>
    where
        I: IntoIterator<Item = CircuitEventRequest>,
    {
        let requests = requests.into_iter().collect::<Vec<_>>();
        for (index, request) in requests.iter().enumerate() {
            validate_request(request)?;
            self.validate_future_time(&request.time)?;
            self.validate_future_phase(&request.time, request.phase)?;
            if let Some(previous) = index.checked_sub(1).and_then(|i| requests.get(i))
                && compare_request_order(previous, request)? == Ordering::Greater
            {
                return Err(CircuitEventAgendaError::TraceOutOfOrder { index });
            }
        }
        let count = u64::try_from(requests.len())
            .map_err(|_| CircuitEventAgendaError::IdentityExhausted)?;
        let next_id = self
            .next_id
            .checked_add(count)
            .ok_or(CircuitEventAgendaError::IdentityExhausted)?;
        let scheduled = self
            .counters
            .scheduled
            .checked_add(count)
            .ok_or(CircuitEventAgendaError::CounterExhausted)?;
        let preordered = self
            .counters
            .preordered
            .checked_add(count)
            .ok_or(CircuitEventAgendaError::CounterExhausted)?;
        let mut ids = Vec::with_capacity(requests.len());
        let mut added = VecDeque::with_capacity(requests.len());
        for (offset, request) in requests.into_iter().enumerate() {
            let offset =
                u64::try_from(offset).map_err(|_| CircuitEventAgendaError::IdentityExhausted)?;
            let id = CircuitEventId::new(
                self.next_id
                    .checked_add(offset)
                    .ok_or(CircuitEventAgendaError::IdentityExhausted)?,
            );
            ids.push(id);
            added.push_back(CircuitEvent::from_request(id, request));
        }
        let lifecycle = added
            .iter()
            .cloned()
            .map(|event| CircuitEventLifecycleRecord {
                event,
                action: CircuitEventLifecycleAction::Scheduled,
                related: None,
            })
            .collect::<Vec<_>>();
        let mut existing = self.pending.clone();
        let mut merged = VecDeque::with_capacity(existing.len() + added.len());
        while let (Some(left), Some(right)) = (existing.front(), added.front()) {
            if compare_event_order(left, right)? != Ordering::Greater {
                merged.push_back(
                    existing
                        .pop_front()
                        .ok_or(CircuitEventAgendaError::InconsistentState)?,
                );
            } else {
                merged.push_back(
                    added
                        .pop_front()
                        .ok_or(CircuitEventAgendaError::InconsistentState)?,
                );
            }
        }
        merged.extend(existing);
        merged.extend(added);
        self.next_id = next_id;
        self.pending = merged;
        self.lifecycle.extend(lifecycle);
        self.counters.scheduled = scheduled;
        self.counters.preordered = preordered;
        self.counters.maximum_pending = self.counters.maximum_pending.max(self.pending.len());
        Ok(ids)
    }

    /// Cancels one pending event and retains the cancellation evidence.
    pub fn cancel(&mut self, id: CircuitEventId) -> Result<CircuitEvent, CircuitEventAgendaError> {
        let index = self
            .pending
            .iter()
            .position(|event| event.id == id)
            .ok_or(CircuitEventAgendaError::UnknownEvent(id))?;
        let canceled = self
            .counters
            .canceled
            .checked_add(1)
            .ok_or(CircuitEventAgendaError::CounterExhausted)?;
        let event = self
            .pending
            .remove(index)
            .ok_or(CircuitEventAgendaError::InconsistentState)?;
        self.lifecycle.push(CircuitEventLifecycleRecord {
            event: event.clone(),
            action: CircuitEventLifecycleAction::Canceled,
            related: None,
        });
        self.counters.canceled = canceled;
        Ok(event)
    }

    /// Atomically replaces a pending event with a new request.
    pub fn reschedule(
        &mut self,
        id: CircuitEventId,
        mut replacement: CircuitEventRequest,
    ) -> Result<CircuitEventId, CircuitEventAgendaError> {
        if !self.pending.iter().any(|event| event.id == id) {
            return Err(CircuitEventAgendaError::UnknownEvent(id));
        }
        replacement.cause = CircuitEventCause::RescheduledFrom(id);
        let mut proposed = self.clone();
        let old = proposed.cancel(id)?;
        proposed.counters.canceled -= 1;
        let replacement_id = proposed.schedule(replacement)?;
        proposed.lifecycle.pop();
        proposed.lifecycle.pop();
        let replacement_event = proposed
            .pending
            .iter()
            .find(|event| event.id == replacement_id)
            .ok_or(CircuitEventAgendaError::UnknownEvent(replacement_id))?
            .clone();
        proposed.lifecycle.push(CircuitEventLifecycleRecord {
            event: old,
            action: CircuitEventLifecycleAction::Rescheduled,
            related: Some(replacement_id),
        });
        proposed.lifecycle.push(CircuitEventLifecycleRecord {
            event: replacement_event,
            action: CircuitEventLifecycleAction::Scheduled,
            related: Some(id),
        });
        proposed.counters.rescheduled = proposed
            .counters
            .rescheduled
            .checked_add(1)
            .ok_or(CircuitEventAgendaError::CounterExhausted)?;
        *self = proposed;
        Ok(replacement_id)
    }

    /// Delivers the next event and advances the exact agenda clock.
    pub fn deliver_next(&mut self) -> Option<CircuitEvent> {
        if self.pending.is_empty() {
            return None;
        }
        let event = self.pending.pop_front()?;
        if self.now.predicate_cmp(&event.time) != Some(Ordering::Equal) {
            self.last_delivered_phase = None;
        }
        self.now = event.time.clone();
        self.last_delivered_phase = Some(event.phase);
        self.lifecycle.push(CircuitEventLifecycleRecord {
            event: event.clone(),
            action: CircuitEventLifecycleAction::Delivered,
            related: None,
        });
        self.counters.delivered = self.counters.delivered.saturating_add(1);
        Some(event)
    }

    /// Produces a deterministic fingerprint of the clock, pending events,
    /// lifecycle evidence, and counters.
    pub fn fingerprint(&self) -> CircuitSimulationFingerprint {
        let canonical = format!(
            "time={};phase={:?};next={};pending={:?};lifecycle={:?};counters={:?}",
            self.now,
            self.last_delivered_phase,
            self.next_id,
            self.pending,
            self.lifecycle,
            self.counters
        );
        CircuitSimulationFingerprint {
            algorithm: "fnv1a64-pair-v1".into(),
            value: stable_fingerprint(canonical.as_bytes()),
        }
    }

    /// Replays retained lifecycle evidence and checks pending state and counters.
    pub fn verify_lifecycle(&self) -> CircuitEventReplayReport {
        let mut issues = Vec::new();
        let mut pending = BTreeMap::<CircuitEventId, CircuitEvent>::new();
        let mut expected_replacements = BTreeMap::<CircuitEventId, CircuitEventId>::new();
        let mut counters = CircuitEventCounters {
            preordered: self.counters.preordered,
            ..CircuitEventCounters::default()
        };
        for record in &self.lifecycle {
            match record.action {
                CircuitEventLifecycleAction::Scheduled => {
                    match (
                        expected_replacements.remove(&record.event.id),
                        record.related,
                    ) {
                        (None, None) => {}
                        (Some(expected), Some(actual)) if expected == actual => {}
                        _ => issues.push(CircuitEventReplayIssue::InvalidRescheduleRelation(
                            record.event.id,
                        )),
                    }
                    if pending
                        .insert(record.event.id, record.event.clone())
                        .is_some()
                    {
                        issues.push(CircuitEventReplayIssue::DuplicateSchedule(record.event.id));
                    }
                    counters.scheduled += 1;
                    counters.maximum_pending = counters.maximum_pending.max(pending.len());
                }
                CircuitEventLifecycleAction::Canceled
                | CircuitEventLifecycleAction::Rescheduled
                | CircuitEventLifecycleAction::Delivered => {
                    match pending.remove(&record.event.id) {
                        None => {
                            issues
                                .push(CircuitEventReplayIssue::UnknownTransition(record.event.id));
                        }
                        Some(event) if event != record.event => {
                            issues.push(CircuitEventReplayIssue::EventContentMismatch(
                                record.event.id,
                            ));
                        }
                        Some(_) => {}
                    }
                    match record.action {
                        CircuitEventLifecycleAction::Canceled => counters.canceled += 1,
                        CircuitEventLifecycleAction::Rescheduled => {
                            counters.rescheduled += 1;
                            if let Some(replacement) = record.related {
                                expected_replacements.insert(replacement, record.event.id);
                            } else {
                                issues.push(CircuitEventReplayIssue::InvalidRescheduleRelation(
                                    record.event.id,
                                ));
                            }
                        }
                        CircuitEventLifecycleAction::Delivered => counters.delivered += 1,
                        CircuitEventLifecycleAction::Scheduled => {}
                    }
                }
            }
        }
        issues.extend(
            expected_replacements
                .into_keys()
                .map(CircuitEventReplayIssue::InvalidRescheduleRelation),
        );
        let mut replayed = pending.into_values().collect::<Vec<_>>();
        let mut indeterminate = false;
        replayed.sort_by(|left, right| match compare_event_order(left, right) {
            Ok(ordering) => ordering,
            Err(_) => {
                indeterminate = true;
                Ordering::Equal
            }
        });
        if indeterminate {
            issues.push(CircuitEventReplayIssue::IndeterminateOrder);
        }
        if !replayed.iter().eq(self.pending.iter()) {
            issues.push(CircuitEventReplayIssue::PendingMismatch);
        }
        if counters != self.counters {
            issues.push(CircuitEventReplayIssue::CounterMismatch);
        }
        CircuitEventReplayReport {
            issues,
            pending: replayed,
            counters,
        }
    }

    fn validate_future_time(&self, time: &Real) -> Result<(), CircuitEventAgendaError> {
        match time.predicate_cmp(&self.now) {
            Some(Ordering::Less) => Err(CircuitEventAgendaError::EventInPast),
            Some(Ordering::Equal) => Ok(()),
            Some(Ordering::Greater) => Ok(()),
            None => Err(CircuitEventAgendaError::IndeterminateTime),
        }
    }

    fn validate_future_phase(
        &self,
        time: &Real,
        phase: CircuitEventPhase,
    ) -> Result<(), CircuitEventAgendaError> {
        if time.predicate_cmp(&self.now) == Some(Ordering::Equal)
            && self
                .last_delivered_phase
                .is_some_and(|completed| phase < completed)
        {
            return Err(CircuitEventAgendaError::PhaseInPast);
        }
        Ok(())
    }

    fn insertion_position(&self, event: &CircuitEvent) -> Result<usize, CircuitEventAgendaError> {
        for (index, existing) in self.pending.iter().enumerate() {
            if compare_event_order(event, existing)? == Ordering::Less {
                return Ok(index);
            }
        }
        Ok(self.pending.len())
    }
}

impl ExactBreakpointProvider for CircuitEventAgenda {
    fn breakpoint_provider_id(&self) -> String {
        "circuit-event-agenda".into()
    }

    fn next_breakpoint_after(&self, time: &Real) -> Result<Option<Real>, ExactBreakpointError> {
        for event in &self.pending {
            match time.predicate_cmp(&event.time) {
                Some(Ordering::Less) => return Ok(Some(event.time.clone())),
                Some(Ordering::Equal | Ordering::Greater) => {}
                None => {
                    return Err(ExactBreakpointError {
                        provider: self.breakpoint_provider_id(),
                        detail: "event time cannot be ordered exactly".into(),
                    });
                }
            }
        }
        Ok(None)
    }
}

/// Returns the earliest exact breakpoint supplied by any provider.
pub fn earliest_exact_breakpoint_after(
    time: &Real,
    providers: &[&dyn ExactBreakpointProvider],
) -> Result<Option<Real>, ExactBreakpointError> {
    let mut earliest: Option<Real> = None;
    for provider in providers {
        let Some(candidate) = provider.next_breakpoint_after(time)? else {
            continue;
        };
        match candidate.predicate_cmp(time) {
            Some(Ordering::Greater) => {}
            Some(Ordering::Less | Ordering::Equal) => {
                return Err(ExactBreakpointError {
                    provider: provider.breakpoint_provider_id(),
                    detail: "provider returned a breakpoint that is not strictly in the future"
                        .into(),
                });
            }
            None => {
                return Err(ExactBreakpointError {
                    provider: provider.breakpoint_provider_id(),
                    detail: "breakpoint time cannot be ordered exactly".into(),
                });
            }
        }
        match earliest.as_ref() {
            None => earliest = Some(candidate),
            Some(current) => match candidate.predicate_cmp(current) {
                Some(Ordering::Less) => earliest = Some(candidate),
                Some(Ordering::Equal | Ordering::Greater) => {}
                None => {
                    return Err(ExactBreakpointError {
                        provider: provider.breakpoint_provider_id(),
                        detail: "breakpoint candidates cannot be ordered exactly".into(),
                    });
                }
            },
        }
    }
    Ok(earliest)
}

fn compare_request_order(
    left: &CircuitEventRequest,
    right: &CircuitEventRequest,
) -> Result<Ordering, CircuitEventAgendaError> {
    match left.time.predicate_cmp(&right.time) {
        Some(Ordering::Equal) => Ok(left.phase.cmp(&right.phase)),
        Some(ordering) => Ok(ordering),
        None => Err(CircuitEventAgendaError::IndeterminateTime),
    }
}

fn validate_request(request: &CircuitEventRequest) -> Result<(), CircuitEventAgendaError> {
    fn nonblank(value: &str, field: &'static str) -> Result<(), CircuitEventAgendaError> {
        if value.trim().is_empty() {
            Err(CircuitEventAgendaError::EmptyRetainedText {
                field: field.into(),
            })
        } else {
            Ok(())
        }
    }

    if let CircuitEventTarget::External(endpoint) = &request.target {
        nonblank(endpoint, "target.external")?;
    }
    if let Some(CircuitEventTarget::External(endpoint)) = &request.source {
        nonblank(endpoint, "source.external")?;
    }
    match &request.kind {
        CircuitEventKind::Timer { key } => nonblank(key, "kind.timer.key")?,
        CircuitEventKind::TraceSample { channel, .. } => {
            nonblank(channel, "kind.trace_sample.channel")?
        }
        CircuitEventKind::Behavioral { kind, fields } => {
            nonblank(kind, "kind.behavioral.kind")?;
            for field in fields.keys() {
                nonblank(field, "kind.behavioral.field")?;
            }
        }
        _ => {}
    }
    if let CircuitEventCause::Authored { provenance } = &request.cause {
        nonblank(provenance, "cause.authored.provenance")?;
    }
    Ok(())
}

fn compare_event_order(
    left: &CircuitEvent,
    right: &CircuitEvent,
) -> Result<Ordering, CircuitEventAgendaError> {
    match left.time.predicate_cmp(&right.time) {
        Some(Ordering::Equal) => Ok(left
            .phase
            .cmp(&right.phase)
            .then_with(|| left.sequence.cmp(&right.sequence))),
        Some(ordering) => Ok(ordering),
        None => Err(CircuitEventAgendaError::IndeterminateTime),
    }
}

pub(crate) fn stable_fingerprint(bytes: &[u8]) -> String {
    fn fnv(bytes: &[u8], mut state: u64) -> u64 {
        for byte in bytes {
            state ^= u64::from(*byte);
            state = state.wrapping_mul(0x1000_0000_01b3);
        }
        state
    }
    let first = fnv(bytes, 0xcbf2_9ce4_8422_2325);
    let second = fnv(bytes, 0x8422_2325_cbf2_9ce4);
    format!("{first:016x}{second:016x}")
}

/// One retained deterministic stochastic sample.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct StochasticSample {
    /// Stable draw identity.
    pub id: RandomDrawId,
    /// Caller-authored purpose or parameter identity.
    pub label: String,
    /// Generator algorithm identifier.
    pub algorithm: String,
    /// Initial stream seed.
    pub seed: u64,
    /// Raw generated bits.
    pub raw: u64,
    /// Exact unit-interval projection.
    pub value: Real,
}

/// Failure to draw from a reproducible stochastic stream.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum StochasticStreamError {
    /// The stable draw-identity space was exhausted.
    IdentityExhausted,
    /// The exact unit-interval projection could not be represented.
    Projection,
    /// Retained stream state and draw evidence did not replay exactly.
    EvidenceMismatch,
}

impl Display for StochasticStreamError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::IdentityExhausted => formatter.write_str("stochastic draw identity exhausted"),
            Self::Projection => formatter.write_str("stochastic exact projection failed"),
            Self::EvidenceMismatch => {
                formatter.write_str("stochastic retained evidence failed exact replay")
            }
        }
    }
}

impl std::error::Error for StochasticStreamError {}

/// Explicit reproducible pseudo-random stream with retained draw evidence.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct StochasticStream {
    seed: u64,
    state: u64,
    draw_index: u64,
    samples: Vec<StochasticSample>,
}

impl StochasticStream {
    /// Stable generator algorithm identifier.
    pub const ALGORITHM: &'static str = "xorshift64star-unit32-v1";

    /// Creates a deterministic stream. A zero seed is mapped to the documented
    /// nonzero xorshift initial state while retaining the authored zero seed.
    pub fn new(seed: u64) -> Self {
        Self {
            seed,
            state: if seed == 0 {
                0x9e37_79b9_7f4a_7c15
            } else {
                seed
            },
            draw_index: 0,
            samples: Vec::new(),
        }
    }

    /// Returns retained samples in draw order.
    pub fn samples(&self) -> &[StochasticSample] {
        &self.samples
    }

    /// Replays every retained draw from the authored seed.
    pub fn verify(&self) -> Result<(), StochasticStreamError> {
        let mut replay = Self::new(self.seed);
        for retained in &self.samples {
            let candidate = replay.draw_unit(retained.label.clone())?;
            if &candidate != retained {
                return Err(StochasticStreamError::EvidenceMismatch);
            }
        }
        if replay.state != self.state || replay.draw_index != self.draw_index {
            return Err(StochasticStreamError::EvidenceMismatch);
        }
        Ok(())
    }

    /// Draws one exact value in `[0, 1)` and retains its provenance.
    pub fn draw_unit(
        &mut self,
        label: impl Into<String>,
    ) -> Result<StochasticSample, StochasticStreamError> {
        let next_index = self
            .draw_index
            .checked_add(1)
            .ok_or(StochasticStreamError::IdentityExhausted)?;
        let mut state = self.state;
        state ^= state >> 12;
        state ^= state << 25;
        state ^= state >> 27;
        let raw = state.wrapping_mul(0x2545_f491_4f6c_dd1d);
        let unit_bits = raw >> 32;
        let value = (Real::from(unit_bits) / Real::from(1_u64 << 32))
            .map_err(|_| StochasticStreamError::Projection)?;
        let sample = StochasticSample {
            id: RandomDrawId::new(self.draw_index),
            label: label.into(),
            algorithm: Self::ALGORITHM.into(),
            seed: self.seed,
            raw,
            value,
        };
        self.state = state;
        self.draw_index = next_index;
        self.samples.push(sample.clone());
        Ok(sample)
    }
}

/// Named exact breakpoint sequence for traces, topology devices, protection
/// devices, and future coupled-domain adapters.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct ExactBreakpointSchedule {
    /// Stable provider identity retained in diagnostics.
    pub provider: String,
    /// Strictly increasing exact breakpoint times.
    pub times: Vec<Real>,
}

impl ExactBreakpointSchedule {
    /// Validates and constructs a reusable exact breakpoint schedule.
    pub fn new(
        provider: impl Into<String>,
        times: Vec<Real>,
    ) -> Result<Self, ExactBreakpointError> {
        let provider = provider.into();
        if provider.trim().is_empty() {
            return Err(ExactBreakpointError {
                provider,
                detail: "provider identity must not be blank".into(),
            });
        }
        for (index, pair) in times.windows(2).enumerate() {
            if pair[0].predicate_cmp(&pair[1]) != Some(Ordering::Less) {
                return Err(ExactBreakpointError {
                    provider,
                    detail: format!(
                        "breakpoint times must be strictly increasing at index {}",
                        index + 1
                    ),
                });
            }
        }
        Ok(Self { provider, times })
    }
}

impl ExactBreakpointProvider for ExactBreakpointSchedule {
    fn breakpoint_provider_id(&self) -> String {
        self.provider.clone()
    }

    fn next_breakpoint_after(&self, time: &Real) -> Result<Option<Real>, ExactBreakpointError> {
        for candidate in &self.times {
            match candidate.predicate_cmp(time) {
                Some(Ordering::Greater) => return Ok(Some(candidate.clone())),
                Some(Ordering::Less | Ordering::Equal) => {}
                None => {
                    return Err(ExactBreakpointError {
                        provider: self.provider.clone(),
                        detail: "breakpoint time cannot be ordered exactly".into(),
                    });
                }
            }
        }
        Ok(None)
    }
}
