//! Part family, variant, import, and query graph.

use std::collections::BTreeMap;

use crate::{
    Capability, CompatibilityRelation, ConnectionDecision, ElectricalPolarity, Interface,
    ManufacturingRoute, PartAspect, PartAssertion, PartConstraint, PartId, PartQuery, PartsError,
    PartsResult, ProcessCapability, QueryCandidate, QueryEvidence, QueryMatchStatus, QueryResult,
    QueryUnknown, Relationship, RevisionId, SourceRef, Terminal, TerminalId, ToolCapability,
    VariantId,
};

/// Evidence attached to query results and compatibility edges.
#[derive(Clone, Debug, PartialEq)]
pub struct PartQueryEvidence {
    /// Sources consulted.
    pub sources: Vec<SourceRef>,
    /// Short machine-readable facts used in the decision.
    pub facts: Vec<String>,
}

/// Query result with explicit evidence.
#[derive(Clone, Debug, PartialEq)]
pub struct PartQueryResult<T> {
    /// Query decision or payload.
    pub value: T,
    /// Evidence used to produce the value.
    pub evidence: PartQueryEvidence,
}

/// External corpus/import family.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ImportTargetKind {
    /// KiCad symbols, footprints, boards, or database-library rows.
    KiCad,
    /// LibrePCB symbol/component/package/device elements.
    LibrePcb,
    /// Altium database-library style rows.
    Altium,
    /// NopSCADlib vitamin/assembly/module data.
    NopScadLib,
    /// Replimat parts, techniques, tools, or progress pages.
    Replimat,
    /// Distributor or component-search snapshot.
    Distributor,
    /// BSDL package/pin/boundary-scan data.
    Bsdl,
    /// LEF/DEF/OpenDB/GDS-derived IC metadata.
    IcLayout,
    /// STEP/WRL/SCAD/STL model artifact.
    ModelArtifact,
    /// Source-specific target.
    Custom(String),
}

/// Kind of import issue retained in an audit report.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ImportIssueKind {
    /// Parsed and accepted field.
    Parsed,
    /// Rejected field.
    Rejected,
    /// Lossy conversion.
    LossyConversion,
    /// Stale source or cache.
    StaleSource,
    /// Unresolved reference.
    UnresolvedReference,
    /// License/provenance note.
    LicenseNote,
    /// Human review requirement.
    ReviewRequired,
}

/// Field-level importer issue.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ImportIssue {
    /// Source field or object path.
    pub field: String,
    /// Issue kind.
    pub kind: ImportIssueKind,
    /// Human-readable detail.
    pub detail: String,
}

/// Import audit summary for external part libraries.
#[derive(Clone, Debug, PartialEq)]
pub struct ImportReport {
    /// Import source.
    pub source: SourceRef,
    /// Import target family.
    pub target: ImportTargetKind,
    /// Number of part families imported.
    pub imported_family_count: usize,
    /// Number of variants imported.
    pub imported_variant_count: usize,
    /// Number of fields deliberately retained as explicit unknowns.
    pub unknown_field_count: usize,
    /// Parsed source fields.
    pub parsed_assertions: Vec<ImportIssue>,
    /// Rejected source fields.
    pub rejected_fields: Vec<ImportIssue>,
    /// Lossy conversions kept explicit.
    pub lossy_conversions: Vec<ImportIssue>,
    /// Stale or freshness-sensitive sources.
    pub stale_sources: Vec<ImportIssue>,
    /// Unresolved references.
    pub unresolved_references: Vec<ImportIssue>,
    /// License/provenance notes.
    pub license_notes: Vec<ImportIssue>,
    /// Human review requirements.
    pub review_requirements: Vec<ImportIssue>,
    /// Human-readable warnings.
    pub warnings: Vec<String>,
}

/// A part family such as a resistor, connector series, package family, or BOM item.
#[derive(Clone, Debug, PartialEq)]
pub struct PartFamily {
    id: PartId,
    name: String,
    variants: BTreeMap<VariantId, PartVariant>,
}

/// A concrete or semi-concrete part variant.
#[derive(Clone, Debug, PartialEq)]
pub struct PartVariant {
    id: VariantId,
    revision: Option<RevisionId>,
    assertions: Vec<PartAssertion>,
    aspects: Vec<PartAspect>,
    terminals: BTreeMap<TerminalId, Terminal>,
    interfaces: Vec<Interface>,
    subparts: Vec<PartId>,
    processes: Vec<ProcessCapability>,
}

/// Queryable source-attributed part graph.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct PartGraph {
    families: BTreeMap<PartId, PartFamily>,
    compatibility: Vec<CompatibilityRelation>,
    relationships: Vec<Relationship>,
    capabilities: Vec<Capability>,
    routes: Vec<ManufacturingRoute>,
    tools: Vec<ToolCapability>,
    imports: Vec<ImportReport>,
}

impl PartQueryEvidence {
    /// Creates empty evidence.
    pub fn empty() -> Self {
        Self {
            sources: Vec::new(),
            facts: Vec::new(),
        }
    }

    /// Creates evidence from one source and one fact.
    pub fn from_fact(source: SourceRef, fact: impl Into<String>) -> Self {
        Self {
            sources: vec![source],
            facts: vec![fact.into()],
        }
    }
}

impl PartFamily {
    /// Creates an empty part family.
    pub fn new(id: PartId, name: impl Into<String>) -> Self {
        Self {
            id,
            name: name.into(),
            variants: BTreeMap::new(),
        }
    }

    /// Returns the family id.
    pub const fn id(&self) -> &PartId {
        &self.id
    }

    /// Returns the display name.
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Inserts or replaces a variant.
    pub fn insert_variant(&mut self, variant: PartVariant) {
        self.variants.insert(variant.id.clone(), variant);
    }

    /// Returns a variant by id.
    pub fn variant(&self, id: &VariantId) -> Option<&PartVariant> {
        self.variants.get(id)
    }

    /// Returns all variants.
    pub fn variants(&self) -> impl Iterator<Item = &PartVariant> {
        self.variants.values()
    }
}

impl PartVariant {
    /// Creates a variant with optional revision.
    pub fn new(id: VariantId, revision: Option<RevisionId>) -> Self {
        Self {
            id,
            revision,
            assertions: Vec::new(),
            aspects: Vec::new(),
            terminals: BTreeMap::new(),
            interfaces: Vec::new(),
            subparts: Vec::new(),
            processes: Vec::new(),
        }
    }

    /// Returns the variant id.
    pub const fn id(&self) -> &VariantId {
        &self.id
    }

    /// Returns the optional revision id.
    pub const fn revision(&self) -> Option<&RevisionId> {
        self.revision.as_ref()
    }

    /// Adds a source-attributed assertion.
    pub fn add_assertion(&mut self, assertion: PartAssertion) {
        self.assertions.push(assertion);
    }

    /// Adds a part aspect.
    pub fn add_aspect(&mut self, aspect: PartAspect) {
        self.aspects.push(aspect);
    }

    /// Adds a terminal.
    pub fn add_terminal(&mut self, terminal: Terminal) {
        self.terminals.insert(terminal.id().clone(), terminal);
    }

    /// Adds an interface.
    pub fn add_interface(&mut self, interface: Interface) {
        self.interfaces.push(interface);
    }

    /// Adds a subpart family reference.
    pub fn add_subpart(&mut self, part: PartId) {
        self.subparts.push(part);
    }

    /// Adds a process capability.
    pub fn add_process(&mut self, process: ProcessCapability) {
        self.processes.push(process);
    }

    /// Returns a terminal by id.
    pub fn terminal(&self, id: &TerminalId) -> Option<&Terminal> {
        self.terminals.get(id)
    }

    /// Returns assertions.
    pub fn assertions(&self) -> &[PartAssertion] {
        &self.assertions
    }

    /// Returns aspects.
    pub fn aspects(&self) -> &[PartAspect] {
        &self.aspects
    }

    /// Returns interfaces.
    pub fn interfaces(&self) -> &[Interface] {
        &self.interfaces
    }

    /// Returns subpart ids.
    pub fn subparts(&self) -> &[PartId] {
        &self.subparts
    }
}

impl PartGraph {
    /// Inserts or replaces a family.
    pub fn insert_family(&mut self, family: PartFamily) {
        self.families.insert(family.id.clone(), family);
    }

    /// Returns a family by id.
    pub fn family(&self, id: &PartId) -> Option<&PartFamily> {
        self.families.get(id)
    }

    /// Records a compatibility relation.
    pub fn add_compatibility(&mut self, relation: CompatibilityRelation) {
        self.compatibility.push(relation);
    }

    /// Records a general relationship.
    pub fn add_relationship(&mut self, relationship: Relationship) {
        self.relationships.push(relationship);
    }

    /// Records a typed process capability.
    pub fn add_capability(&mut self, capability: Capability) {
        self.capabilities.push(capability);
    }

    /// Records a manufacturing route.
    pub fn add_manufacturing_route(&mut self, route: ManufacturingRoute) {
        self.routes.push(route);
    }

    /// Records a tool capability.
    pub fn add_tool(&mut self, tool: ToolCapability) {
        self.tools.push(tool);
    }

    /// Records an import report.
    pub fn add_import_report(&mut self, report: ImportReport) {
        self.imports.push(report);
    }

    /// Returns import reports.
    pub fn import_reports(&self) -> &[ImportReport] {
        &self.imports
    }

    /// Returns compatibility relations.
    pub fn compatibility(&self) -> &[CompatibilityRelation] {
        &self.compatibility
    }

    /// Returns general relationships.
    pub fn relationships(&self) -> &[Relationship] {
        &self.relationships
    }

    /// Returns typed process capabilities.
    pub fn capabilities(&self) -> &[Capability] {
        &self.capabilities
    }

    /// Returns manufacturing routes.
    pub fn manufacturing_routes(&self) -> &[ManufacturingRoute] {
        &self.routes
    }

    /// Finds typed capabilities by tool handle.
    pub fn capabilities_for_tool(&self, tool: &str) -> Vec<&Capability> {
        self.capabilities
            .iter()
            .filter(|capability| capability.tool.handle == tool)
            .collect()
    }

    /// Finds typed capabilities that declare the target as an input or output.
    pub fn capabilities_for_target(&self, target: &str) -> Vec<&Capability> {
        self.capabilities
            .iter()
            .filter(|capability| {
                capability.inputs.iter().any(|input| input.handle == target)
                    || capability
                        .outputs
                        .iter()
                        .any(|output| output.handle == target)
            })
            .collect()
    }

    /// Queries part families with ranked evidence and explicit unknowns.
    pub fn query_parts(&self, query: &PartQuery) -> QueryResult<PartId> {
        let mut unknowns = Vec::new();
        let mut candidates = Vec::new();
        for family in self.families.values() {
            let mut rank = 0;
            let mut notes = Vec::new();
            let mut matched = true;
            for constraint in &query.constraints {
                match constraint {
                    PartConstraint::FamilyNameContains(needle) => {
                        if family.name.contains(needle) {
                            rank += 10;
                            notes.push(format!("family name contains {needle}"));
                        } else {
                            matched = false;
                        }
                    }
                    PartConstraint::PartIdContains(needle) => {
                        if family.id.as_str().contains(needle) {
                            rank += 10;
                            notes.push(format!("part id contains {needle}"));
                        } else {
                            matched = false;
                        }
                    }
                    PartConstraint::HasInterface => {
                        if family
                            .variants()
                            .any(|variant| !variant.interfaces().is_empty())
                        {
                            rank += 5;
                            notes.push("variant exposes interface".into());
                        } else {
                            matched = false;
                        }
                    }
                    PartConstraint::HasGeometry => {
                        if family.variants().any(|variant| {
                            variant
                                .assertions()
                                .iter()
                                .any(|assertion| matches!(assertion, PartAssertion::ShapeHandle(_)))
                        }) {
                            rank += 5;
                            notes.push("variant has geometry assertion".into());
                        } else {
                            matched = false;
                        }
                    }
                    PartConstraint::HasCapabilityTarget(target) => {
                        if !self.capabilities_for_target(target).is_empty() {
                            rank += 3;
                            notes.push(format!("capability target {target} exists"));
                        } else {
                            matched = false;
                        }
                    }
                    PartConstraint::Custom(field) => {
                        unknowns.push(QueryUnknown {
                            field: field.clone(),
                            reason: "custom query constraint requires an adapter".into(),
                        });
                        matched = false;
                    }
                }
                if !matched {
                    break;
                }
            }
            if matched {
                candidates.push(QueryCandidate {
                    value: family.id.clone(),
                    rank,
                    evidence: QueryEvidence {
                        evidence: PartQueryEvidence::empty(),
                        status: QueryMatchStatus::ExactMatch,
                        notes,
                    },
                });
            }
        }
        candidates.sort_by(|left, right| {
            right
                .rank
                .cmp(&left.rank)
                .then_with(|| left.value.as_str().cmp(right.value.as_str()))
        });
        if candidates.is_empty() && unknowns.is_empty() {
            unknowns.push(QueryUnknown {
                field: "part-query".into(),
                reason: "no part family satisfied all constraints".into(),
            });
        }
        QueryResult {
            candidates,
            unknowns,
            conflicts: Vec::new(),
        }
    }

    /// Returns recorded tool capabilities.
    pub fn tools(&self) -> &[ToolCapability] {
        &self.tools
    }

    /// Queries whether two terminals can be safely connected.
    ///
    /// The decision is deliberately conservative. Explicit power-to-ground
    /// connections are unsafe, matching power terminals require overlapping
    /// exact voltage envelopes when both are known, and missing polarity or
    /// voltage facts return [`ConnectionDecision::Unknown`]. This keeps source
    /// gaps visible instead of fabricating compatibility from defaults, which
    /// is the data-graph analogue of Yap's "certified decision or explicit
    /// uncertainty" rule.
    pub fn safe_connection(
        &self,
        left: (&PartId, &VariantId, &TerminalId),
        right: (&PartId, &VariantId, &TerminalId),
    ) -> PartsResult<PartQueryResult<ConnectionDecision>> {
        let left_terminal = self.lookup_terminal(left)?;
        let right_terminal = self.lookup_terminal(right)?;
        let mut evidence = PartQueryEvidence::empty();
        evidence.facts.push(format!(
            "left polarity={:?}, right polarity={:?}",
            left_terminal.polarity(),
            right_terminal.polarity()
        ));

        let value = match (left_terminal.polarity(), right_terminal.polarity()) {
            (ElectricalPolarity::Ground, ElectricalPolarity::Ground) => ConnectionDecision::Safe,
            (ElectricalPolarity::Power, ElectricalPolarity::Ground)
            | (ElectricalPolarity::Ground, ElectricalPolarity::Power) => ConnectionDecision::Unsafe,
            (ElectricalPolarity::Power, ElectricalPolarity::Power) => {
                match (left_terminal.voltage(), right_terminal.voltage()) {
                    (Some(left_voltage), Some(right_voltage)) => {
                        match left_voltage.overlaps(right_voltage) {
                            Some(true) => ConnectionDecision::Safe,
                            Some(false) => ConnectionDecision::Unsafe,
                            None => ConnectionDecision::Unknown,
                        }
                    }
                    _ => ConnectionDecision::Unknown,
                }
            }
            (ElectricalPolarity::Signal, ElectricalPolarity::Signal)
            | (ElectricalPolarity::Passive, ElectricalPolarity::Passive)
            | (ElectricalPolarity::Signal, ElectricalPolarity::Passive)
            | (ElectricalPolarity::Passive, ElectricalPolarity::Signal) => ConnectionDecision::Safe,
            _ => ConnectionDecision::Unknown,
        };

        Ok(PartQueryResult { value, evidence })
    }

    fn lookup_terminal(&self, key: (&PartId, &VariantId, &TerminalId)) -> PartsResult<&Terminal> {
        let family = self.families.get(key.0).ok_or(PartsError::MissingPart)?;
        let variant = family.variant(key.1).ok_or(PartsError::MissingVariant)?;
        variant.terminal(key.2).ok_or(PartsError::MissingTerminal)
    }
}
