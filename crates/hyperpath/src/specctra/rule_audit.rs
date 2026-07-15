//! Exact retained route-rule auditing for Specctra route objects.
//!
//! Specctra DSN/SES rule declarations are source constraints, not edits to
//! route geometry. This module keeps that boundary explicit: it selects the
//! applicable retained rule scope for each trace/arc, computes the strongest
//! exact width and clearance values within that scope, certifies route width,
//! and replays straight-trace pairwise clearance from the retained rule
//! evidence. Exact source objects, predicates, and predicate reports remain
//! separate. The design also mirrors
//! Lee/Hightower-style autorouting, where graph/path proposals and rule
//! acceptance are distinct phases.

use std::cmp::Ordering;

use hyperlimit::{PredicatePolicy, compare_reals_with_policy};
use hyperreal::{Real, RealSign};

use crate::pcb::{ClearanceStatus, NetId, TraceClearanceReport, TraceLayer, check_trace_clearance};
use crate::specctra::{
    SpecctraArcWireRecord, SpecctraImportError, SpecctraRouteRuleRecord, SpecctraTraceRecord,
    import_specctra_trace_record,
};

/// Kind of route item audited against retained Specctra route rules.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SpecctraRouteRuleItemKind {
    /// A straight wire/trace record.
    Trace,
    /// A retained circular-arc route record.
    Arc,
}

/// Discrete scope class used when selecting a retained route rule.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SpecctraRouteRuleScopeClass {
    /// Rule applies to every route item.
    Global,
    /// Rule applies to one layer.
    Layer,
    /// Rule applies to one net.
    Net,
    /// Rule applies to one net on one layer.
    NetLayer,
}

impl SpecctraRouteRuleScopeClass {
    fn specificity(self) -> u8 {
        match self {
            Self::Global => 0,
            Self::Layer => 1,
            Self::Net => 2,
            Self::NetLayer => 3,
        }
    }
}

/// Exact width certification status for one route item.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SpecctraRouteRuleWidthStatus {
    /// No retained route rule matched the item.
    NoApplicableRule,
    /// The route width was certified to satisfy the selected minimum width.
    Certified,
    /// The route width was certified below the selected minimum width.
    Violation,
    /// Exact comparison could not decide the route-width predicate.
    Unknown,
}

/// Exact pairwise clearance audit status for straight trace records.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SpecctraRouteRuleTraceClearanceStatus {
    /// At least one trace had no applicable retained route rule.
    NoApplicableRule,
    /// The pair was not applicable, for example same-net traces.
    NotApplicable,
    /// The traces overlap or touch before spacing can be considered.
    NoShortViolation,
    /// Exact trace clearance satisfied the retained effective clearance.
    CertifiedClear,
    /// Exact trace clearance violated the retained effective clearance.
    ClearanceViolation,
    /// Exact clearance comparison could not decide.
    Unknown,
}

impl From<ClearanceStatus> for SpecctraRouteRuleTraceClearanceStatus {
    fn from(status: ClearanceStatus) -> Self {
        match status {
            ClearanceStatus::NotApplicable => Self::NotApplicable,
            ClearanceStatus::CertifiedClear => Self::CertifiedClear,
            ClearanceStatus::ClearanceViolation => Self::ClearanceViolation,
            ClearanceStatus::NoShortViolation => Self::NoShortViolation,
            ClearanceStatus::Unknown => Self::Unknown,
        }
    }
}

/// Audit result for one route item against retained route rules.
#[derive(Clone, Debug, PartialEq)]
pub struct SpecctraRouteRuleItemAudit {
    /// Route item kind.
    pub item_kind: SpecctraRouteRuleItemKind,
    /// Zero-based index within the corresponding input slice.
    pub item_index: usize,
    /// Route item net.
    pub net: NetId,
    /// Route item layer.
    pub layer: TraceLayer,
    /// Exact route item width.
    pub width: Real,
    /// Selected highest-specificity rule scope, if any rule applied.
    pub selected_scope: Option<SpecctraRouteRuleScopeClass>,
    /// Indices of all retained rules in the selected highest-specificity scope.
    pub selected_rule_indices: Vec<usize>,
    /// Strongest exact clearance among selected rules.
    pub effective_clearance: Option<Real>,
    /// Strongest exact minimum width among selected rules.
    pub effective_width: Option<Real>,
    /// Exact width certification status.
    pub width_status: SpecctraRouteRuleWidthStatus,
}

/// Retained audit report for route rules applied to exact route objects.
#[derive(Clone, Debug, PartialEq)]
pub struct SpecctraRouteRuleAudit {
    /// Per-item audit records in trace order followed by arc order.
    pub items: Vec<SpecctraRouteRuleItemAudit>,
}

/// Pairwise exact clearance audit for two straight Specctra trace records.
#[derive(Clone, Debug, PartialEq)]
pub struct SpecctraRouteRuleTraceClearancePairAudit {
    /// First trace index.
    pub first_index: usize,
    /// Second trace index.
    pub second_index: usize,
    /// Exact retained clearance selected from the two effective route rules.
    pub required_clearance: Option<Real>,
    /// Exact PCB clearance report when both traces were ruled.
    pub clearance_report: Option<TraceClearanceReport>,
    /// Retained pairwise clearance status.
    pub status: SpecctraRouteRuleTraceClearanceStatus,
}

/// Exact retained rule audit for straight Specctra trace clearance.
#[derive(Clone, Debug, PartialEq)]
pub struct SpecctraRouteRuleTraceClearanceAudit {
    /// Per-trace retained width/rule-selection audits.
    pub item_audits: Vec<SpecctraRouteRuleItemAudit>,
    /// Pairwise trace clearance audits in lexicographic index order.
    pub pairs: Vec<SpecctraRouteRuleTraceClearancePairAudit>,
}

impl SpecctraRouteRuleTraceClearanceAudit {
    /// Return true when every pair with applicable rules is certified clear or not applicable.
    pub fn all_clearances_certified(&self) -> bool {
        self.pairs.iter().all(|pair| {
            matches!(
                pair.status,
                SpecctraRouteRuleTraceClearanceStatus::CertifiedClear
                    | SpecctraRouteRuleTraceClearanceStatus::NotApplicable
            )
        })
    }

    /// Return the first pair with a reported clearance violation or no-short violation.
    pub fn first_clearance_violation(&self) -> Option<&SpecctraRouteRuleTraceClearancePairAudit> {
        self.pairs.iter().find(|pair| {
            matches!(
                pair.status,
                SpecctraRouteRuleTraceClearanceStatus::ClearanceViolation
                    | SpecctraRouteRuleTraceClearanceStatus::NoShortViolation
            )
        })
    }

    /// Return true when at least one pair could not select a retained clearance rule.
    pub fn has_unruled_pair(&self) -> bool {
        self.pairs
            .iter()
            .any(|pair| pair.status == SpecctraRouteRuleTraceClearanceStatus::NoApplicableRule)
    }
}

impl SpecctraRouteRuleAudit {
    /// Return true when every audited route item has a certified width.
    pub fn all_widths_certified(&self) -> bool {
        self.items
            .iter()
            .all(|item| item.width_status == SpecctraRouteRuleWidthStatus::Certified)
    }

    /// Return the first route item whose width was rejected.
    pub fn first_width_violation(&self) -> Option<&SpecctraRouteRuleItemAudit> {
        self.items
            .iter()
            .find(|item| item.width_status == SpecctraRouteRuleWidthStatus::Violation)
    }

    /// Return true when at least one item has no applicable retained rule.
    pub fn has_unruled_item(&self) -> bool {
        self.items
            .iter()
            .any(|item| item.width_status == SpecctraRouteRuleWidthStatus::NoApplicableRule)
    }
}

/// Errors while auditing retained route rules.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SpecctraRouteRuleAuditError {
    /// A route item width was structurally negative.
    NegativeRouteWidth,
    /// A rule width or clearance was structurally negative.
    NegativeRuleValue,
    /// Exact comparison could not choose the strongest selected rule value.
    UnknownRuleOrdering,
    /// A trace record could not be lowered into exact PCB trace geometry.
    InvalidTrace(SpecctraImportError),
}

/// Audit exact route widths against retained Specctra route-rule records.
///
/// Rule scope selection is deterministic and source-retaining. A rule matches
/// an item when every specified scope coordinate agrees with the item. The
/// highest available specificity wins: net+layer rules override net-only,
/// net-only rules override layer-only rules, and layer-only rules override
/// global rules. If several selected rules share the same scope class, the
/// effective width and clearance are the exact maxima of those selected
/// values. This function does not perform clearance DRC; it retains the
/// effective clearance for downstream pairwise route predicates.
pub fn audit_specctra_route_rule_widths(
    traces: &[SpecctraTraceRecord],
    arcs: &[SpecctraArcWireRecord],
    rules: &[SpecctraRouteRuleRecord],
    policy: PredicatePolicy,
) -> Result<SpecctraRouteRuleAudit, SpecctraRouteRuleAuditError> {
    validate_rules(rules)?;
    let mut items = Vec::with_capacity(traces.len() + arcs.len());
    for (index, trace) in traces.iter().enumerate() {
        items.push(audit_item(
            SpecctraRouteRuleItemKind::Trace,
            index,
            trace.net,
            trace.layer,
            trace.width.clone(),
            rules,
            policy,
        )?);
    }
    for (index, arc) in arcs.iter().enumerate() {
        items.push(audit_item(
            SpecctraRouteRuleItemKind::Arc,
            index,
            arc.net,
            arc.layer,
            arc.width.clone(),
            rules,
            policy,
        )?);
    }
    Ok(SpecctraRouteRuleAudit { items })
}

/// Audit pairwise straight-trace clearance using retained Specctra route rules.
///
/// The effective clearance for a pair is the exact maximum of the two selected
/// item clearances. This mirrors common DRC behavior while preserving the
/// exact-computation boundary advocated by Yap, "Towards Exact Geometric
/// Computation," *Computational Geometry* 7.1-2 (1997): rule selection is
/// retained as source evidence, trace lowering is exact, and the PCB swept-line
/// predicate certifies the pair. Arcs are intentionally not accepted here;
/// curved route clearance needs its own exact swept-arc predicate instead of
/// being flattened into chords.
pub fn audit_specctra_trace_rule_clearances(
    traces: &[SpecctraTraceRecord],
    rules: &[SpecctraRouteRuleRecord],
    policy: PredicatePolicy,
) -> Result<SpecctraRouteRuleTraceClearanceAudit, SpecctraRouteRuleAuditError> {
    let item_report = audit_specctra_route_rule_widths(traces, &[], rules, policy)?;
    let lowered_traces = traces
        .iter()
        .map(import_specctra_trace_record)
        .collect::<Result<Vec<_>, _>>()
        .map_err(SpecctraRouteRuleAuditError::InvalidTrace)?;

    let mut pairs = Vec::new();
    for first_index in 0..traces.len() {
        for second_index in (first_index + 1)..traces.len() {
            let first = &item_report.items[first_index];
            let second = &item_report.items[second_index];
            let Some(first_clearance) = first.effective_clearance.clone() else {
                pairs.push(unruled_pair(first_index, second_index));
                continue;
            };
            let Some(second_clearance) = second.effective_clearance.clone() else {
                pairs.push(unruled_pair(first_index, second_index));
                continue;
            };
            let required_clearance =
                max_real_option(Some(first_clearance), second_clearance, policy)?;
            let clearance_report = check_trace_clearance(
                &lowered_traces[first_index],
                &lowered_traces[second_index],
                &required_clearance,
                policy,
            );
            let status = clearance_report.status.clone().into();
            pairs.push(SpecctraRouteRuleTraceClearancePairAudit {
                first_index,
                second_index,
                required_clearance: Some(required_clearance),
                clearance_report: Some(clearance_report),
                status,
            });
        }
    }

    Ok(SpecctraRouteRuleTraceClearanceAudit {
        item_audits: item_report.items,
        pairs,
    })
}

fn validate_rules(rules: &[SpecctraRouteRuleRecord]) -> Result<(), SpecctraRouteRuleAuditError> {
    for rule in rules {
        if rule.width.structural_facts().sign == Some(RealSign::Negative)
            || rule.clearance.structural_facts().sign == Some(RealSign::Negative)
        {
            return Err(SpecctraRouteRuleAuditError::NegativeRuleValue);
        }
    }
    Ok(())
}

fn audit_item(
    item_kind: SpecctraRouteRuleItemKind,
    item_index: usize,
    net: NetId,
    layer: TraceLayer,
    width: Real,
    rules: &[SpecctraRouteRuleRecord],
    policy: PredicatePolicy,
) -> Result<SpecctraRouteRuleItemAudit, SpecctraRouteRuleAuditError> {
    if width.structural_facts().sign == Some(RealSign::Negative) {
        return Err(SpecctraRouteRuleAuditError::NegativeRouteWidth);
    }
    let Some(selected_scope) = rules
        .iter()
        .filter_map(|rule| matching_scope(rule, net, layer))
        .max_by_key(|scope| scope.specificity())
    else {
        return Ok(SpecctraRouteRuleItemAudit {
            item_kind,
            item_index,
            net,
            layer,
            width,
            selected_scope: None,
            selected_rule_indices: Vec::new(),
            effective_clearance: None,
            effective_width: None,
            width_status: SpecctraRouteRuleWidthStatus::NoApplicableRule,
        });
    };

    let mut selected_rule_indices = Vec::new();
    let mut effective_clearance = None;
    let mut effective_width = None;
    for (rule_index, rule) in rules.iter().enumerate() {
        if matching_scope(rule, net, layer) == Some(selected_scope) {
            selected_rule_indices.push(rule_index);
            effective_clearance = Some(max_real_option(
                effective_clearance,
                rule.clearance.clone(),
                policy,
            )?);
            effective_width = Some(max_real_option(
                effective_width,
                rule.width.clone(),
                policy,
            )?);
        }
    }
    let effective_width = effective_width.expect("selected scope implies at least one rule");
    let width_status = match compare_reals_with_policy(&width, &effective_width, policy).value() {
        Some(Ordering::Less) => SpecctraRouteRuleWidthStatus::Violation,
        Some(Ordering::Equal | Ordering::Greater) => SpecctraRouteRuleWidthStatus::Certified,
        None => SpecctraRouteRuleWidthStatus::Unknown,
    };

    Ok(SpecctraRouteRuleItemAudit {
        item_kind,
        item_index,
        net,
        layer,
        width,
        selected_scope: Some(selected_scope),
        selected_rule_indices,
        effective_clearance,
        effective_width: Some(effective_width),
        width_status,
    })
}

fn matching_scope(
    rule: &SpecctraRouteRuleRecord,
    net: NetId,
    layer: TraceLayer,
) -> Option<SpecctraRouteRuleScopeClass> {
    if rule.net.is_some_and(|rule_net| rule_net != net)
        || rule.layer.is_some_and(|rule_layer| rule_layer != layer)
    {
        return None;
    }
    match (rule.net, rule.layer) {
        (Some(_), Some(_)) => Some(SpecctraRouteRuleScopeClass::NetLayer),
        (Some(_), None) => Some(SpecctraRouteRuleScopeClass::Net),
        (None, Some(_)) => Some(SpecctraRouteRuleScopeClass::Layer),
        (None, None) => Some(SpecctraRouteRuleScopeClass::Global),
    }
}

fn max_real_option(
    current: Option<Real>,
    candidate: Real,
    policy: PredicatePolicy,
) -> Result<Real, SpecctraRouteRuleAuditError> {
    let Some(current) = current else {
        return Ok(candidate);
    };
    match compare_reals_with_policy(&current, &candidate, policy)
        .value()
        .ok_or(SpecctraRouteRuleAuditError::UnknownRuleOrdering)?
    {
        Ordering::Less => Ok(candidate),
        Ordering::Equal | Ordering::Greater => Ok(current),
    }
}

fn unruled_pair(
    first_index: usize,
    second_index: usize,
) -> SpecctraRouteRuleTraceClearancePairAudit {
    SpecctraRouteRuleTraceClearancePairAudit {
        first_index,
        second_index,
        required_clearance: None,
        clearance_report: None,
        status: SpecctraRouteRuleTraceClearanceStatus::NoApplicableRule,
    }
}
