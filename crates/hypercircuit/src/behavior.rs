//! Runtime behavioral adapters over retained typed circuit events.
//!
//! Handlers and queues are conveniences, never circuit truth. Every action
//! affecting simulation passes through [`CircuitEventAgenda`] and receives
//! exact ordering, identity, causality, and lifecycle evidence.

use std::collections::{BTreeMap, VecDeque};
use std::fmt::{Display, Formatter};

use crate::{
    CircuitEvent, CircuitEventAgenda, CircuitEventAgendaError, CircuitEventCause, CircuitEventId,
    CircuitEventKind, CircuitEventPhase, CircuitEventRequest, CircuitEventTarget, Real,
};

/// Failure in an external behavioral adapter.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum BehaviorError {
    /// Exact event scheduling failed.
    Agenda(CircuitEventAgendaError),
    /// No callback is registered for an event target.
    MissingHandler(CircuitEventTarget),
    /// Handler rejected an event with typed diagnostic text.
    Handler(String),
}

impl Display for BehaviorError {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Agenda(error) => Display::fmt(error, formatter),
            Self::MissingHandler(target) => write!(formatter, "no behavior handler for {target:?}"),
            Self::Handler(detail) => write!(formatter, "behavior handler failed: {detail}"),
        }
    }
}

impl std::error::Error for BehaviorError {}

impl From<CircuitEventAgendaError> for BehaviorError {
    fn from(value: CircuitEventAgendaError) -> Self {
        Self::Agenda(value)
    }
}

/// Restricted component-scoped access to exact event scheduling.
pub struct BehaviorContext<'a> {
    owner: CircuitEventTarget,
    cause: CircuitEventId,
    now: Real,
    agenda: &'a mut CircuitEventAgenda,
}

impl<'a> BehaviorContext<'a> {
    /// Creates a context while dispatching one delivered event.
    pub fn new(
        owner: CircuitEventTarget,
        cause: CircuitEventId,
        now: Real,
        agenda: &'a mut CircuitEventAgenda,
    ) -> Self {
        Self {
            owner,
            cause,
            now,
            agenda,
        }
    }

    /// Returns the exact callback time.
    pub fn time(&self) -> &Real {
        &self.now
    }

    /// Returns the stable component-owned source identity.
    pub fn owner(&self) -> &CircuitEventTarget {
        &self.owner
    }

    /// Emits a causally linked event at an exact absolute time.
    pub fn emit_at(
        &mut self,
        time: Real,
        phase: CircuitEventPhase,
        target: CircuitEventTarget,
        kind: CircuitEventKind,
    ) -> Result<CircuitEventId, BehaviorError> {
        Ok(self.agenda.schedule(CircuitEventRequest {
            time,
            phase,
            source: Some(self.owner.clone()),
            target,
            kind,
            cause: CircuitEventCause::Event(self.cause),
        })?)
    }

    /// Emits a causally linked event after an exact delay.
    pub fn emit_after(
        &mut self,
        delay: Real,
        phase: CircuitEventPhase,
        target: CircuitEventTarget,
        kind: CircuitEventKind,
    ) -> Result<CircuitEventId, BehaviorError> {
        self.emit_at(self.now.clone() + delay, phase, target, kind)
    }

    /// Schedules a component-owned timer.
    pub fn sleep_until(
        &mut self,
        time: Real,
        key: impl Into<String>,
    ) -> Result<CircuitEventId, BehaviorError> {
        self.emit_at(
            time,
            CircuitEventPhase::PostSolve,
            self.owner.clone(),
            CircuitEventKind::Timer { key: key.into() },
        )
    }

    /// Cancels a pending event through the retained agenda.
    pub fn cancel(&mut self, event: CircuitEventId) -> Result<CircuitEvent, BehaviorError> {
        Ok(self.agenda.cancel(event)?)
    }

    /// Atomically replaces a pending event while retaining causal lifecycle evidence.
    pub fn reschedule(
        &mut self,
        event: CircuitEventId,
        time: Real,
        phase: CircuitEventPhase,
        target: CircuitEventTarget,
        kind: CircuitEventKind,
    ) -> Result<CircuitEventId, BehaviorError> {
        Ok(self.agenda.reschedule(
            event,
            CircuitEventRequest {
                time,
                phase,
                source: Some(self.owner.clone()),
                target,
                kind,
                cause: CircuitEventCause::Event(self.cause),
            },
        )?)
    }
}

/// Callback adapter for simple component behavior.
pub trait CircuitEventHandler {
    /// Handles one delivered typed event and may schedule retained follow-ups.
    fn on_event(
        &mut self,
        event: &CircuitEvent,
        context: &mut BehaviorContext<'_>,
    ) -> Result<(), BehaviorError>;
}

/// Runtime-only callback registry keyed by stable event target.
#[derive(Default)]
pub struct BehaviorRuntime {
    handlers: BTreeMap<CircuitEventTarget, Box<dyn CircuitEventHandler>>,
}

impl BehaviorRuntime {
    /// Creates an empty runtime.
    pub fn new() -> Self {
        Self::default()
    }

    /// Registers or replaces one target's external callback.
    pub fn register(
        &mut self,
        target: CircuitEventTarget,
        handler: impl CircuitEventHandler + 'static,
    ) {
        self.handlers.insert(target, Box::new(handler));
    }

    /// True when a callback is registered for `target`.
    pub fn has_handler(&self, target: &CircuitEventTarget) -> bool {
        self.handlers.contains_key(target)
    }

    /// Dispatches one delivered event with a restricted scoped context.
    pub fn dispatch(
        &mut self,
        event: &CircuitEvent,
        agenda: &mut CircuitEventAgenda,
    ) -> Result<(), BehaviorError> {
        let handler = self
            .handlers
            .get_mut(&event.target)
            .ok_or_else(|| BehaviorError::MissingHandler(event.target.clone()))?;
        let mut context =
            BehaviorContext::new(event.target.clone(), event.id, event.time.clone(), agenda);
        handler.on_event(event, &mut context)
    }

    /// Dispatches when registered and otherwise leaves the retained event observed.
    pub fn dispatch_if_registered(
        &mut self,
        event: &CircuitEvent,
        agenda: &mut CircuitEventAgenda,
    ) -> Result<bool, BehaviorError> {
        if !self.has_handler(&event.target) {
            return Ok(false);
        }
        self.dispatch(event, agenda)?;
        Ok(true)
    }
}

/// Runtime-only FIFO used by cooperating behavioral activities.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct BehaviorQueue<T> {
    values: VecDeque<T>,
}

impl<T> BehaviorQueue<T> {
    /// Creates an empty queue.
    pub fn new() -> Self {
        Self {
            values: VecDeque::new(),
        }
    }

    /// Adds one value to the queue tail.
    pub fn put(&mut self, value: T) {
        self.values.push_back(value);
    }

    /// Removes one value from the queue head.
    pub fn take(&mut self) -> Option<T> {
        self.values.pop_front()
    }

    /// Returns the number of queued values.
    pub fn len(&self) -> usize {
        self.values.len()
    }

    /// True when no values are pending.
    pub fn is_empty(&self) -> bool {
        self.values.is_empty()
    }
}

/// Runtime mailbox supporting deterministic selective receive.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SelectiveEventMailbox {
    events: VecDeque<CircuitEvent>,
}

impl SelectiveEventMailbox {
    /// Adds one delivered event to the mailbox.
    pub fn push(&mut self, event: CircuitEvent) {
        self.events.push_back(event);
    }

    /// Receives the first event satisfying a deterministic predicate.
    pub fn receive(&mut self, predicate: impl Fn(&CircuitEvent) -> bool) -> Option<CircuitEvent> {
        let index = self.events.iter().position(predicate)?;
        self.events.remove(index)
    }

    /// Receives the first event with matching target and payload predicate.
    pub fn receive_target(
        &mut self,
        target: &CircuitEventTarget,
        predicate: impl Fn(&CircuitEventKind) -> bool,
    ) -> Option<CircuitEvent> {
        self.receive(|event| &event.target == target && predicate(&event.kind))
    }
}

/// Result of cooperatively resuming an optional asynchronous behavior.
#[cfg(feature = "behavior-async")]
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AsyncBehaviorStatus {
    /// Resume only when another matching circuit event is delivered.
    Waiting,
    /// The activity has completed and will be removed.
    Complete,
}

/// Runtime-only, cooperatively resumed asynchronous behavior.
///
/// Implementations keep ordinary Rust state between calls. Timers and circuit
/// effects must be emitted through the supplied context, so the retained event
/// agenda remains the sole simulation clock and source of circuit truth.
#[cfg(feature = "behavior-async")]
pub trait AsyncCircuitBehavior {
    /// Selects which target-addressed events may resume this behavior.
    fn accepts(&self, _event: &CircuitEvent) -> bool {
        true
    }

    /// Resumes the behavior for one delivered event.
    fn resume(
        &mut self,
        event: &CircuitEvent,
        context: &mut BehaviorContext<'_>,
    ) -> Result<AsyncBehaviorStatus, BehaviorError>;
}

/// Optional cooperative asynchronous adapter keyed by stable event target.
#[cfg(feature = "behavior-async")]
#[derive(Default)]
pub struct AsyncBehaviorRuntime {
    tasks: BTreeMap<CircuitEventTarget, Box<dyn AsyncCircuitBehavior>>,
    mailboxes: BTreeMap<CircuitEventTarget, SelectiveEventMailbox>,
}

#[cfg(feature = "behavior-async")]
impl AsyncBehaviorRuntime {
    /// Creates an empty runtime.
    pub fn new() -> Self {
        Self::default()
    }

    /// Registers or replaces one target-scoped asynchronous activity.
    pub fn register(
        &mut self,
        target: CircuitEventTarget,
        task: impl AsyncCircuitBehavior + 'static,
    ) {
        self.tasks.insert(target, Box::new(task));
    }

    /// Queues a delivered event, selectively receives the first event for its
    /// target, and resumes that target's task once.
    pub fn dispatch(
        &mut self,
        event: &CircuitEvent,
        agenda: &mut CircuitEventAgenda,
    ) -> Result<bool, BehaviorError> {
        let Some(task) = self.tasks.get_mut(&event.target) else {
            return Ok(false);
        };
        let mailbox = self.mailboxes.entry(event.target.clone()).or_default();
        mailbox.push(event.clone());
        let Some(selected) = mailbox.receive(|candidate| task.accepts(candidate)) else {
            return Ok(true);
        };
        let mut context = BehaviorContext::new(
            event.target.clone(),
            selected.id,
            selected.time.clone(),
            agenda,
        );
        if task.resume(&selected, &mut context)? == AsyncBehaviorStatus::Complete {
            self.tasks.remove(&event.target);
            self.mailboxes.remove(&event.target);
        }
        Ok(true)
    }
}
