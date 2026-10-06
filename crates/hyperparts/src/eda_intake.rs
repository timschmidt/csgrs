//! EDA authoring and interchange intake reports.
//!
//! This module is the source boundary for tscircuit-like authoring bundles:
//! Circuit JSON records, footprint strings, generated model references, package
//! metadata, and autorouter/fabrication artifacts enter as attributed records
//! and leave as explicit handoff reports. Exact strings are parsed into
//! symbolic [`Real`] values, while missing, lossy, or unresolved fields remain
//! report entries instead of being guessed into primitive floats.

use hyperreal::Real;

use crate::{
    Assertion, AssertionConfidence, AssertionValue, ElectricalFactStatus, ElectricalPolarity,
    ElectronicPackage, GeneralPartAssertion, GeometryHandle, GeometryHandoffReport, GeometryStatus,
    ImportIssue, ImportIssueKind, ImportReport, ImportTargetKind, Interface, InterfaceKind,
    PartAspect, PartAssertion, PartFamily, PartGraph, PartId, PartQueryEvidence, PartVariant,
    PinFunction, ProcessKind, SourceRef, Terminal, TerminalId, VariantId, VoltageEnvelope,
};

/// Overall status for an EDA authoring intake.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EdaIntakeStatus {
    /// Every imported source fact was exact or certified.
    Accepted,
    /// Import succeeded with explicit unknowns that downstream crates must keep visible.
    AcceptedWithUnknowns,
    /// Import succeeded but rejected, lossy, conflicting, or review-only fields are present.
    NeedsReview,
    /// No authoritative part, circuit, package, route, model, or fabrication fact was importable.
    Rejected,
}

/// Status for a downstream handoff record.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EdaHandoffStatus {
    /// Handoff contains exact symbolic source facts.
    Exact,
    /// Handoff contains certified imported facts.
    Certified,
    /// Handoff exists, but humans or domain-specific validators must review it.
    NeedsReview,
    /// Source was inspected but did not provide enough information.
    Unknown,
    /// Source field was present but unusable for the target owner.
    Rejected,
}

/// Geometry certainty carried by generated model references.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EdaModelStatus {
    /// Native exact model or exact construction handle.
    Exact,
    /// Reviewed model artifact that can be treated as certified input.
    Certified,
    /// Mesh or preview artifact with known lossy conversion.
    LossyPreview,
    /// Browser/viewer-only artifact that must not feed exact kernels.
    DisplayOnly,
    /// Source referenced a model but did not provide a usable handle.
    Missing,
}

/// Route geometry certainty carried by autorouter output.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EdaRouteStatus {
    /// Route geometry is exact symbolic geometry.
    Exact,
    /// Route geometry has been reviewed or certified by the source boundary.
    Certified,
    /// Route geometry is a lossy preview or sampled polyline.
    Lossy,
    /// Route geometry was referenced but not available.
    Missing,
}

/// Fabrication/readiness status carried toward `hyperdrc`.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EdaFabricationReadiness {
    /// Fabrication evidence says the artifact is ready for rule checking or release.
    Ready,
    /// Fabrication evidence exists but requires review.
    NeedsReview,
    /// Fabrication evidence reports a failed or blocked output.
    Failed,
    /// Source was inspected but readiness is unknown.
    Unknown,
}

/// Exact source field represented as text, never as a primitive float.
///
/// Numeric strings are parsed into [`Real`] by the intake pass.
/// JavaScript/JSON decimal spelling is retained as source text and converted to
/// exact arithmetic only after validation.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct EdaExactField {
    /// Field path relative to the source record.
    pub field: String,
    /// Source numeric text. `None` means the source was inspected and omitted it.
    pub value: Option<String>,
    /// Unit label retained with the exact value.
    pub unit: Option<String>,
}

/// Circuit JSON-like source record.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CircuitJsonSourceRecord {
    /// Source record id.
    pub id: String,
    /// Source record kind, such as `resistor`, `capacitor`, `trace`, or `chip`.
    pub kind: String,
    /// Optional reference designator.
    pub reference: Option<String>,
    /// Source nets connected to this record.
    pub nets: Vec<String>,
    /// Exact numeric fields such as resistance, capacitance, width, or clearance.
    pub exact_fields: Vec<EdaExactField>,
}

/// Footprint expression generated or referenced by an EDA authoring source.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct EdaFootprintString {
    /// Stable footprint handle.
    pub handle: String,
    /// Source expression, for example `soic:pins=8,pitch=1.27mm`.
    pub expression: String,
}

/// Generated model or preview reference.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct GeneratedModelReference {
    /// Stable source model handle.
    pub handle: String,
    /// Owning generator, package, or artifact family.
    pub owner: String,
    /// Artifact format such as `step`, `wrl`, `stl`, or `preview`.
    pub format: String,
    /// Optional path/URI retained for provenance.
    pub uri: Option<String>,
    /// Units associated with geometry, when known.
    pub units: Option<String>,
    /// Declared model certainty.
    pub status: EdaModelStatus,
}

/// Package pin metadata.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct EdaPackagePin {
    /// Stable terminal id text.
    pub terminal: String,
    /// Display pin/pad name.
    pub name: String,
    /// Semantic pin function.
    pub function: PinFunction,
    /// Optional exact minimum voltage text.
    pub voltage_min: Option<String>,
    /// Optional exact maximum voltage text.
    pub voltage_max: Option<String>,
}

/// Package metadata imported from the authoring bundle.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct EdaPackageMetadata {
    /// Package display name.
    pub name: String,
    /// Package/footprint aspect handle.
    pub handle: String,
    /// Declared terminal count.
    pub terminal_count: Option<usize>,
    /// Source pin entries.
    pub pins: Vec<EdaPackagePin>,
}

/// Autorouter output record carried toward `hyperpath`.
///
/// The route handoff cites Lee, "An Algorithm for Path Connections and Its
/// Applications," *IRE Transactions on Electronic Computers*, 1961, because
/// EDA autorouter output must be retained as route facts for a routing/path
/// owner, not reinterpreted as certified part geometry by this catalog crate.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AutorouterOutputRecord {
    /// Stable route id.
    pub route_id: String,
    /// Net name routed by this output.
    pub net: String,
    /// Geometry handle supplied by the router or source.
    pub geometry_handle: Option<String>,
    /// Units associated with route geometry.
    pub units: Option<String>,
    /// Optional exact grid/step used by the router.
    pub exact_grid: Option<String>,
    /// Declared route geometry status.
    pub status: EdaRouteStatus,
}

/// Fabrication artifact or readiness output carried toward `hyperdrc`.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct FabricationOutputRecord {
    /// Stable fabrication artifact id.
    pub artifact_id: String,
    /// Artifact format such as `gerber`, `excellon`, `ipc2581`, or `bom`.
    pub format: String,
    /// Process family represented by the artifact.
    pub process: ProcessKind,
    /// Declared readiness status.
    pub readiness: EdaFabricationReadiness,
    /// Source notes retained as evidence.
    pub notes: Vec<String>,
}

/// Complete tscircuit-like authoring bundle accepted by this intake pass.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct EdaAuthoringBundle {
    /// Source evidence for all records in this bundle.
    pub source: SourceRef,
    /// Target part family.
    pub part: PartId,
    /// Target part variant.
    pub variant: VariantId,
    /// Human-readable family name.
    pub display_name: String,
    /// Circuit JSON-like records.
    pub circuit_records: Vec<CircuitJsonSourceRecord>,
    /// Optional generated or referenced footprint expression.
    pub footprint: Option<EdaFootprintString>,
    /// Generated model or preview references.
    pub model_references: Vec<GeneratedModelReference>,
    /// Package metadata.
    pub package: Option<EdaPackageMetadata>,
    /// Autorouter outputs.
    pub routes: Vec<AutorouterOutputRecord>,
    /// Fabrication/readiness outputs.
    pub fabrication: Vec<FabricationOutputRecord>,
}

/// Exact circuit parameter passed to `hypercircuit`.
#[derive(Clone, Debug, PartialEq)]
pub struct CircuitResidualParameter {
    /// Source field path.
    pub field: String,
    /// Exact value.
    pub value: Real,
    /// Unit label.
    pub unit: Option<String>,
}

/// Circuit residual-fact handoff to `hypercircuit`.
///
/// Topology/model records and exact parameter facts remain separate from
/// numerical solution. `hyperparts` records the source evidence and leaves
/// stamping and residual semantics to `hypercircuit`.
#[derive(Clone, Debug, PartialEq)]
pub struct CircuitResidualFactHandoff {
    /// Downstream owner.
    pub owner: String,
    /// Model/residual handle for the downstream owner.
    pub model_handle: String,
    /// Source record id.
    pub source_record_id: String,
    /// Source record kind.
    pub record_kind: String,
    /// Source nets.
    pub nets: Vec<String>,
    /// Exact numeric parameters accepted for residual construction.
    pub exact_parameters: Vec<CircuitResidualParameter>,
    /// Handoff status.
    pub status: EdaHandoffStatus,
    /// Evidence used to create the handoff.
    pub evidence: PartQueryEvidence,
    /// Explicit unknowns that block a certified residual handoff.
    pub unknowns: Vec<String>,
}

/// Route geometry handoff to `hyperpath`.
#[derive(Clone, Debug, PartialEq)]
pub struct RouteGeometryHandoff {
    /// Downstream owner.
    pub owner: String,
    /// Route id.
    pub route_id: String,
    /// Net name.
    pub net: String,
    /// Geometry handle owned by the route/path layer.
    pub geometry: Option<GeometryHandle>,
    /// Exact routing grid/step, when supplied.
    pub exact_grid: Option<Real>,
    /// Handoff status.
    pub status: EdaHandoffStatus,
    /// Evidence used to create the handoff.
    pub evidence: PartQueryEvidence,
    /// Explicit unknowns that block exact route consumption.
    pub unknowns: Vec<String>,
}

/// Fabrication/readiness handoff to `hyperdrc`.
#[derive(Clone, Debug, PartialEq)]
pub struct DrcFabricationHandoff {
    /// Downstream owner.
    pub owner: String,
    /// Fabrication artifact id.
    pub artifact_id: String,
    /// Artifact format.
    pub format: String,
    /// Process family represented by the artifact.
    pub process: ProcessKind,
    /// Declared readiness.
    pub readiness: EdaFabricationReadiness,
    /// Handoff status.
    pub status: EdaHandoffStatus,
    /// Evidence used to create the handoff.
    pub evidence: PartQueryEvidence,
    /// Explicit unknowns that block release/certification.
    pub unknowns: Vec<String>,
}

/// Full result of an EDA authoring intake.
#[derive(Clone, Debug, PartialEq)]
pub struct EdaAuthoringImportResult {
    /// Populated part graph containing exact part assertions and import report.
    pub graph: PartGraph,
    /// Structured import audit report.
    pub import_report: ImportReport,
    /// Overall intake status.
    pub status: EdaIntakeStatus,
    /// Geometry/model handoffs retained for downstream geometry owners.
    pub geometry_handoffs: Vec<GeometryHandoffReport>,
    /// Circuit residual facts destined for `hypercircuit`.
    pub circuit_handoffs: Vec<CircuitResidualFactHandoff>,
    /// Route geometry destined for `hyperpath`.
    pub route_handoffs: Vec<RouteGeometryHandoff>,
    /// Fabrication/readiness evidence destined for `hyperdrc`.
    pub drc_handoffs: Vec<DrcFabricationHandoff>,
}

impl EdaAuthoringImportResult {
    /// Returns true when no unknown, lossy, rejected, or review-only facts remain.
    pub fn is_exact_ready(&self) -> bool {
        self.status == EdaIntakeStatus::Accepted
            && self
                .circuit_handoffs
                .iter()
                .all(|handoff| handoff.status == EdaHandoffStatus::Exact)
            && self
                .route_handoffs
                .iter()
                .all(|handoff| handoff.status == EdaHandoffStatus::Exact)
            && self
                .drc_handoffs
                .iter()
                .all(|handoff| handoff.status == EdaHandoffStatus::Certified)
    }
}

/// Imports a tscircuit-like EDA authoring bundle into exact part assertions and
/// downstream handoff reports.
///
/// This function is intentionally report-bearing rather than fallible for most
/// source defects: a malformed footprint parameter, lossy preview model, or
/// missing voltage range is retained in [`ImportReport`] so callers can show the
/// exact blocker instead of losing provenance at the API boundary.
pub fn import_eda_authoring_bundle(bundle: EdaAuthoringBundle) -> EdaAuthoringImportResult {
    let expected_issue_count = bundle
        .circuit_records
        .iter()
        .map(|record| 1 + record.exact_fields.len())
        .sum::<usize>()
        + bundle.model_references.len()
        + bundle
            .package
            .as_ref()
            .map_or(0, |package| 2 + package.pins.len())
        + 2 * bundle.routes.len()
        + bundle.fabrication.len()
        + usize::from(bundle.footprint.is_some());
    let mut issues = ImportIssueBuilder::new(bundle.source.clone(), expected_issue_count);
    let mut variant = PartVariant::new(bundle.variant.clone(), None);
    let mut geometry_handoffs = Vec::with_capacity(bundle.model_references.len());
    let mut circuit_handoffs = Vec::with_capacity(bundle.circuit_records.len());
    let mut route_handoffs = Vec::with_capacity(bundle.routes.len());
    let mut drc_handoffs = Vec::with_capacity(bundle.fabrication.len());

    import_circuit_records(&bundle, &mut variant, &mut circuit_handoffs, &mut issues);
    import_footprint(&bundle, &mut variant, &mut issues);
    import_models(&bundle, &mut variant, &mut geometry_handoffs, &mut issues);
    import_package(&bundle, &mut variant, &mut issues);
    import_routes(&bundle, &mut route_handoffs, &mut issues);
    import_fabrication(&bundle, &mut drc_handoffs, &mut issues);

    let mut graph = PartGraph::default();
    let name = if bundle.display_name.trim().is_empty() {
        issues.review(
            "display_name",
            "bundle display name was empty; part id used as family name",
        );
        bundle.part.as_str().to_owned()
    } else {
        bundle.display_name.clone()
    };
    let mut family = PartFamily::new(bundle.part.clone(), name);
    family.insert_variant(variant);
    graph.insert_family(family);

    let imported_any = !circuit_handoffs.is_empty()
        || !geometry_handoffs.is_empty()
        || !route_handoffs.is_empty()
        || !drc_handoffs.is_empty()
        || bundle.package.is_some()
        || bundle.footprint.is_some();
    let status = issues.status(imported_any);
    let import_report = issues.finish(1, 1);
    graph.add_import_report(import_report.clone());

    EdaAuthoringImportResult {
        graph,
        import_report,
        status,
        geometry_handoffs,
        circuit_handoffs,
        route_handoffs,
        drc_handoffs,
    }
}

fn import_circuit_records(
    bundle: &EdaAuthoringBundle,
    variant: &mut PartVariant,
    handoffs: &mut Vec<CircuitResidualFactHandoff>,
    issues: &mut ImportIssueBuilder,
) {
    for record in &bundle.circuit_records {
        let record_path = format!("circuit/{}", record.id);
        if record.id.trim().is_empty() {
            issues.reject("circuit/<empty>/id", "circuit source record id is empty");
            continue;
        }
        if record.kind.trim().is_empty() {
            issues.reject(
                format!("{record_path}/kind"),
                "circuit source record kind is empty",
            );
            continue;
        }

        let model_handle = format!(
            "hypercircuit:{}:{}:{}",
            bundle.part.as_str(),
            bundle.variant.as_str(),
            record.id
        );
        variant.add_assertion(PartAssertion::CircuitModel(Assertion::known(
            model_handle.clone(),
            bundle.source.clone(),
        )));

        let mut exact_parameters = Vec::with_capacity(record.exact_fields.len());
        let mut unknowns = Vec::with_capacity(record.exact_fields.len() + 1);
        for field in &record.exact_fields {
            let field_path = format!("{record_path}/{}", field.field);
            match parse_exact_field(&field_path, field, issues) {
                ParsedExactField::Exact(value) => {
                    variant.add_assertion(general_assertion(
                        format!("eda.circuit.{}.{}", record.id, field.field),
                        AssertionValue::exact_scalar(value.clone()),
                        field.unit.clone(),
                        bundle.source.clone(),
                    ));
                    exact_parameters.push(CircuitResidualParameter {
                        field: field.field.clone(),
                        value,
                        unit: field.unit.clone(),
                    });
                }
                ParsedExactField::Unknown => {
                    let unknown = format!("{field_path} omitted");
                    unknowns.push(unknown.clone());
                    variant.add_assertion(general_assertion(
                        format!("eda.circuit.{}.{}", record.id, field.field),
                        AssertionValue::Unknown,
                        field.unit.clone(),
                        bundle.source.clone(),
                    ));
                }
                ParsedExactField::Rejected => {}
            }
        }

        if record.nets.is_empty() {
            let unknown = format!("{record_path}/nets omitted");
            issues.unknown(&unknown, "circuit record has no connected nets");
            unknowns.push(unknown);
        }

        issues.parsed(
            &record_path,
            "circuit record accepted as hypercircuit residual-fact handoff",
        );
        let status = if !unknowns.is_empty() {
            EdaHandoffStatus::Unknown
        } else if exact_parameters.is_empty() {
            EdaHandoffStatus::NeedsReview
        } else {
            EdaHandoffStatus::Exact
        };
        handoffs.push(CircuitResidualFactHandoff {
            owner: "hypercircuit".into(),
            model_handle,
            source_record_id: record.id.clone(),
            record_kind: record.kind.clone(),
            nets: record.nets.clone(),
            exact_parameters,
            status,
            evidence: PartQueryEvidence::from_fact(
                bundle.source.clone(),
                format!("circuit record {} imported", record.id),
            ),
            unknowns,
        });
    }
}

fn import_footprint(
    bundle: &EdaAuthoringBundle,
    variant: &mut PartVariant,
    issues: &mut ImportIssueBuilder,
) {
    let Some(footprint) = &bundle.footprint else {
        return;
    };
    if footprint.handle.trim().is_empty() {
        issues.reject("footprint/handle", "footprint handle is empty");
        return;
    }
    if footprint.expression.trim().is_empty() {
        issues.reject("footprint/expression", "footprint expression is empty");
        return;
    }

    let Some((family, parameters)) = parse_footprint_expression(&footprint.expression, issues)
    else {
        return;
    };

    variant.add_aspect(PartAspect::new(
        &footprint.handle,
        crate::AspectKind::Package,
    ));
    variant.add_assertion(PartAssertion::ShapeHandle(Assertion::known(
        footprint.handle.clone(),
        bundle.source.clone(),
    )));
    variant.add_assertion(general_assertion(
        format!("eda.footprint.{}.family", footprint.handle),
        AssertionValue::Text(family),
        None,
        bundle.source.clone(),
    ));

    for parameter in parameters {
        variant.add_assertion(general_assertion(
            format!("eda.footprint.{}.{}", footprint.handle, parameter.key),
            parameter.value,
            parameter.unit,
            bundle.source.clone(),
        ));
    }
    issues.parsed(
        "footprint/expression",
        "footprint string accepted as exact package aspect assertion",
    );
}

fn import_models(
    bundle: &EdaAuthoringBundle,
    variant: &mut PartVariant,
    handoffs: &mut Vec<GeometryHandoffReport>,
    issues: &mut ImportIssueBuilder,
) {
    for model in &bundle.model_references {
        let path = format!("models/{}", model.handle);
        if model.handle.trim().is_empty() {
            issues.reject("models/<empty>/handle", "model handle is empty");
            continue;
        }
        if model.format.trim().is_empty() {
            issues.reject(format!("{path}/format"), "model format is empty");
            continue;
        }
        let geometry_status = match model.status {
            EdaModelStatus::Exact => GeometryStatus::Exact,
            EdaModelStatus::Certified => GeometryStatus::Certified,
            EdaModelStatus::LossyPreview => {
                issues.lossy(&path, "generated model is a lossy preview artifact");
                GeometryStatus::LossyMesh
            }
            EdaModelStatus::DisplayOnly => {
                issues.lossy(&path, "generated model is display-only preview geometry");
                GeometryStatus::DisplayOnly
            }
            EdaModelStatus::Missing => {
                issues.unresolved(&path, "source referenced a model without a usable handle");
                GeometryStatus::Missing
            }
        };

        if matches!(
            model.status,
            EdaModelStatus::Exact | EdaModelStatus::Certified
        ) {
            variant.add_assertion(PartAssertion::ShapeHandle(Assertion::known(
                model.handle.clone(),
                bundle.source.clone(),
            )));
        }

        let geometry = if geometry_status == GeometryStatus::Missing {
            None
        } else {
            Some(GeometryHandle {
                owner: if model.owner.trim().is_empty() {
                    "generated-model".into()
                } else {
                    model.owner.clone()
                },
                handle: model.handle.clone(),
                units: model.units.clone(),
            })
        };
        handoffs.push(GeometryHandoffReport {
            part: bundle.part.clone(),
            source: crate::ShapeSource::ModelArtifact,
            geometry,
            status: geometry_status,
            evidence: PartQueryEvidence::from_fact(
                bundle.source.clone(),
                format!("model {} imported as {}", model.handle, model.format),
            ),
        });
        issues.parsed(
            &path,
            "generated model reference retained as geometry handoff",
        );
    }
}

fn import_package(
    bundle: &EdaAuthoringBundle,
    variant: &mut PartVariant,
    issues: &mut ImportIssueBuilder,
) {
    let Some(package) = &bundle.package else {
        return;
    };
    let path = "package";
    if package.name.trim().is_empty() {
        issues.reject("package/name", "package name is empty");
    } else {
        variant.add_assertion(general_assertion(
            "eda.package.name",
            AssertionValue::Text(package.name.clone()),
            None,
            bundle.source.clone(),
        ));
    }
    if package.handle.trim().is_empty() {
        issues.reject("package/handle", "package handle is empty");
    } else {
        variant.add_aspect(PartAspect::new(&package.handle, crate::AspectKind::Package));
    }
    if let Some(count) = package.terminal_count {
        variant.add_assertion(general_assertion(
            "eda.package.terminal_count",
            AssertionValue::exact_scalar(Real::from(count as i64)),
            None,
            bundle.source.clone(),
        ));
        if count != package.pins.len() {
            issues.review(
                "package/terminal_count",
                format!(
                    "declared terminal count {count} does not match {} pin records",
                    package.pins.len()
                ),
            );
        }
    } else {
        issues.unknown("package/terminal_count", "package omitted terminal count");
    }

    let mut terminals = Vec::with_capacity(package.pins.len());
    let mut imported_pin_count = 0;
    for pin in &package.pins {
        let pin_path = format!("{path}/pins/{}", pin.terminal);
        if pin.terminal.trim().is_empty() {
            issues.reject("package/pins/<empty>", "pin terminal id is empty");
            continue;
        }
        let Ok(terminal) = TerminalId::new(pin.terminal.clone()) else {
            issues.reject(&pin_path, "pin terminal id is invalid");
            continue;
        };
        let voltage = parse_pin_voltage(&pin_path, pin, issues);
        variant.add_terminal(Terminal::new(
            terminal.clone(),
            if pin.name.trim().is_empty() {
                pin.terminal.clone()
            } else {
                pin.name.clone()
            },
            polarity_from_pin_function(&pin.function),
            voltage.clone(),
        ));
        terminals.push(terminal.clone());
        imported_pin_count += 1;
        issues.parsed(&pin_path, "package pin retained as terminal metadata");
    }

    if !terminals.is_empty() {
        variant.add_interface(Interface::new(
            format!("{}:electrical", package.handle),
            InterfaceKind::Electrical,
            terminals,
        ));
    }

    let electrical_package = ElectronicPackage {
        name: package.name.clone(),
        handle: package.handle.clone(),
        terminal_count: package.terminal_count,
        status: if package.terminal_count == Some(imported_pin_count) {
            ElectricalFactStatus::Certified
        } else {
            ElectricalFactStatus::Unknown
        },
    };
    variant.add_assertion(general_assertion(
        "eda.package.electrical_report",
        AssertionValue::Text(format!(
            "{}:{}:{:?}",
            electrical_package.name, electrical_package.handle, electrical_package.status
        )),
        None,
        bundle.source.clone(),
    ));
    issues.parsed(
        "package",
        "package metadata retained as hyperparts terminals",
    );
}

fn import_routes(
    bundle: &EdaAuthoringBundle,
    handoffs: &mut Vec<RouteGeometryHandoff>,
    issues: &mut ImportIssueBuilder,
) {
    for route in &bundle.routes {
        let path = format!("routes/{}", route.route_id);
        if route.route_id.trim().is_empty() {
            issues.reject("routes/<empty>/route_id", "route id is empty");
            continue;
        }
        if route.net.trim().is_empty() {
            issues.reject(format!("{path}/net"), "route net is empty");
            continue;
        }
        let mut unknowns = Vec::new();
        let geometry = match &route.geometry_handle {
            Some(handle) if !handle.trim().is_empty() => Some(GeometryHandle {
                owner: "hyperpath".into(),
                handle: handle.clone(),
                units: route.units.clone(),
            }),
            _ => {
                issues.unresolved(&path, "autorouter output omitted route geometry handle");
                unknowns.push(format!("{path}/geometry_handle omitted"));
                None
            }
        };
        let exact_grid = route.exact_grid.as_ref().and_then(|grid| {
            let field = EdaExactField {
                field: "exact_grid".into(),
                value: Some(grid.clone()),
                unit: route.units.clone(),
            };
            match parse_exact_field(&format!("{path}/exact_grid"), &field, issues) {
                ParsedExactField::Exact(value) => Some(value),
                ParsedExactField::Unknown => {
                    unknowns.push(format!("{path}/exact_grid omitted"));
                    None
                }
                ParsedExactField::Rejected => None,
            }
        });
        if route.exact_grid.is_none() {
            issues.unknown(
                format!("{path}/exact_grid"),
                "autorouter output omitted exact routing grid",
            );
            unknowns.push(format!("{path}/exact_grid omitted"));
        }

        let status = match route.status {
            EdaRouteStatus::Exact if geometry.is_some() && exact_grid.is_some() => {
                EdaHandoffStatus::Exact
            }
            EdaRouteStatus::Certified if geometry.is_some() => EdaHandoffStatus::Certified,
            EdaRouteStatus::Lossy => {
                issues.lossy(&path, "autorouter output is a lossy sampled route");
                EdaHandoffStatus::NeedsReview
            }
            EdaRouteStatus::Missing => EdaHandoffStatus::Unknown,
            _ => EdaHandoffStatus::Unknown,
        };
        issues.parsed(
            &path,
            "autorouter output retained as hyperpath route handoff",
        );
        handoffs.push(RouteGeometryHandoff {
            owner: "hyperpath".into(),
            route_id: route.route_id.clone(),
            net: route.net.clone(),
            geometry,
            exact_grid,
            status,
            evidence: PartQueryEvidence::from_fact(
                bundle.source.clone(),
                format!("route {} imported", route.route_id),
            ),
            unknowns,
        });
    }
}

fn import_fabrication(
    bundle: &EdaAuthoringBundle,
    handoffs: &mut Vec<DrcFabricationHandoff>,
    issues: &mut ImportIssueBuilder,
) {
    for artifact in &bundle.fabrication {
        let path = format!("fabrication/{}", artifact.artifact_id);
        if artifact.artifact_id.trim().is_empty() {
            issues.reject(
                "fabrication/<empty>/artifact_id",
                "fabrication artifact id is empty",
            );
            continue;
        }
        if artifact.format.trim().is_empty() {
            issues.reject(
                format!("{path}/format"),
                "fabrication artifact format is empty",
            );
            continue;
        }
        let (status, unknowns) = match artifact.readiness {
            EdaFabricationReadiness::Ready => (EdaHandoffStatus::Certified, Vec::new()),
            EdaFabricationReadiness::NeedsReview => {
                issues.review(&path, "fabrication output requires review before release");
                (EdaHandoffStatus::NeedsReview, Vec::new())
            }
            EdaFabricationReadiness::Failed => {
                issues.reject(&path, "fabrication output reports failed readiness");
                (EdaHandoffStatus::Rejected, Vec::new())
            }
            EdaFabricationReadiness::Unknown => {
                issues.unknown(&path, "fabrication output readiness is unknown");
                (
                    EdaHandoffStatus::Unknown,
                    vec![format!("{path}/readiness unknown")],
                )
            }
        };
        issues.parsed(&path, "fabrication artifact retained as hyperdrc handoff");
        handoffs.push(DrcFabricationHandoff {
            owner: "hyperdrc".into(),
            artifact_id: artifact.artifact_id.clone(),
            format: artifact.format.clone(),
            process: artifact.process.clone(),
            readiness: artifact.readiness,
            status,
            evidence: PartQueryEvidence::from_fact(
                bundle.source.clone(),
                format!("fabrication artifact {} imported", artifact.artifact_id),
            ),
            unknowns,
        });
    }
}

#[derive(Clone, Debug, PartialEq)]
enum ParsedExactField {
    Exact(Real),
    Unknown,
    Rejected,
}

fn parse_exact_field(
    field_path: &str,
    field: &EdaExactField,
    issues: &mut ImportIssueBuilder,
) -> ParsedExactField {
    let Some(value) = field.value.as_ref() else {
        issues.unknown(field_path, "exact field omitted by source");
        return ParsedExactField::Unknown;
    };
    if value.trim().is_empty() {
        issues.unknown(field_path, "exact field was present but empty");
        return ParsedExactField::Unknown;
    }
    parse_exact_value(field_path, value, issues)
        .map(ParsedExactField::Exact)
        .unwrap_or(ParsedExactField::Rejected)
}

fn parse_exact_value(
    field_path: &str,
    value: &str,
    issues: &mut ImportIssueBuilder,
) -> Option<Real> {
    let trimmed = value.trim();
    debug_assert!(!trimmed.is_empty());
    if trimmed.eq_ignore_ascii_case("nan")
        || trimmed.eq_ignore_ascii_case("inf")
        || trimmed.eq_ignore_ascii_case("infinity")
        || trimmed.contains('_')
    {
        issues.reject(
            field_path,
            "numeric field is not an exact decimal/rational token",
        );
        return None;
    }
    match trimmed.parse::<Real>() {
        Ok(value) => {
            issues.parsed(field_path, "numeric field parsed as exact Real");
            Some(value)
        }
        Err(_) => {
            issues.reject(
                field_path,
                "numeric field could not be parsed as exact Real",
            );
            None
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
struct FootprintParameter {
    key: String,
    value: AssertionValue,
    unit: Option<String>,
}

fn parse_footprint_expression(
    expression: &str,
    issues: &mut ImportIssueBuilder,
) -> Option<(String, Vec<FootprintParameter>)> {
    let trimmed = expression.trim();
    let (family, rest) = trimmed.split_once(':').unwrap_or((trimmed, ""));
    let family = family.trim();
    if family.is_empty() {
        issues.reject("footprint/family", "footprint family is empty");
        return None;
    }

    let mut parameters = Vec::new();
    for token in rest.split(',').filter(|token| !token.trim().is_empty()) {
        let Some((key, value)) = token.split_once('=') else {
            issues.reject(
                format!("footprint/{token}"),
                "footprint parameter must be key=value",
            );
            return None;
        };
        let key = key.trim();
        let value = value.trim();
        if key.is_empty() || value.is_empty() {
            issues.reject(
                format!("footprint/{token}"),
                "footprint parameter key and value must be non-empty",
            );
            return None;
        }

        if starts_like_number(value) {
            let (number, unit) = split_number_and_unit(value);
            if number.is_empty() {
                issues.reject(
                    format!("footprint/{key}"),
                    "numeric footprint parameter has no numeric prefix",
                );
                return None;
            }
            let field = EdaExactField {
                field: key.into(),
                value: Some(number.to_owned()),
                unit: unit.clone(),
            };
            let field_path = format!("footprint/{key}");
            match parse_exact_field(&field_path, &field, issues) {
                ParsedExactField::Exact(value) => parameters.push(FootprintParameter {
                    key: key.into(),
                    value: AssertionValue::exact_scalar(value),
                    unit,
                }),
                ParsedExactField::Unknown | ParsedExactField::Rejected => return None,
            }
        } else {
            parameters.push(FootprintParameter {
                key: key.into(),
                value: AssertionValue::Text(value.into()),
                unit: None,
            });
            issues.parsed(
                format!("footprint/{key}"),
                "text footprint parameter retained verbatim",
            );
        }
    }
    Some((family.into(), parameters))
}

fn parse_pin_voltage(
    path: &str,
    pin: &EdaPackagePin,
    issues: &mut ImportIssueBuilder,
) -> Option<VoltageEnvelope> {
    match (&pin.voltage_min, &pin.voltage_max) {
        (Some(min), Some(max)) => {
            let min = parse_exact_value(&format!("{path}/voltage_min"), min, issues)?;
            let max = parse_exact_value(&format!("{path}/voltage_max"), max, issues)?;
            match VoltageEnvelope::new(min, max) {
                Ok(voltage) => Some(voltage),
                Err(_) => {
                    issues.reject(path, "pin voltage range has min greater than max");
                    None
                }
            }
        }
        (None, None) => None,
        _ => {
            issues.review(path, "pin supplied only one voltage bound");
            None
        }
    }
}

fn starts_like_number(value: &str) -> bool {
    value
        .chars()
        .next()
        .is_some_and(|ch| ch.is_ascii_digit() || matches!(ch, '+' | '-' | '.'))
}

fn split_number_and_unit(value: &str) -> (&str, Option<String>) {
    let numeric_end = value
        .char_indices()
        .take_while(|(_, ch)| ch.is_ascii_digit() || matches!(ch, '+' | '-' | '.' | '/'))
        .last()
        .map(|(index, ch)| index + ch.len_utf8())
        .unwrap_or(0);
    let (number, unit) = value.split_at(numeric_end);
    let unit = unit.trim();
    if unit.is_empty() {
        (number, None)
    } else {
        (number, Some(unit.into()))
    }
}

fn polarity_from_pin_function(function: &PinFunction) -> ElectricalPolarity {
    match function {
        PinFunction::Power => ElectricalPolarity::Power,
        PinFunction::Ground => ElectricalPolarity::Ground,
        PinFunction::NoConnect | PinFunction::Shield | PinFunction::Thermal => {
            ElectricalPolarity::Passive
        }
        PinFunction::Unknown => ElectricalPolarity::Unknown,
        PinFunction::Digital
        | PinFunction::Analog
        | PinFunction::Reset
        | PinFunction::Test
        | PinFunction::Differential(_)
        | PinFunction::Custom(_) => ElectricalPolarity::Signal,
    }
}

fn general_assertion(
    key: impl Into<String>,
    value: AssertionValue,
    unit: Option<String>,
    source: SourceRef,
) -> PartAssertion {
    PartAssertion::General(Box::new(GeneralPartAssertion {
        key: key.into(),
        value,
        unit,
        conditions: Vec::new(),
        confidence: AssertionConfidence::Imported,
        source,
        revision: None,
    }))
}

#[derive(Clone, Debug)]
struct ImportIssueBuilder {
    source: SourceRef,
    unknown_field_count: usize,
    parsed_assertions: Vec<ImportIssue>,
    rejected_fields: Vec<ImportIssue>,
    lossy_conversions: Vec<ImportIssue>,
    unresolved_references: Vec<ImportIssue>,
    review_requirements: Vec<ImportIssue>,
    warnings: Vec<String>,
}

impl ImportIssueBuilder {
    fn new(source: SourceRef, expected_issue_count: usize) -> Self {
        Self {
            source,
            unknown_field_count: 0,
            parsed_assertions: Vec::with_capacity(expected_issue_count),
            rejected_fields: Vec::new(),
            lossy_conversions: Vec::new(),
            unresolved_references: Vec::new(),
            review_requirements: Vec::new(),
            warnings: vec![
                "EDA authoring intake retained exact strings, unknowns, and handoff owners".into(),
            ],
        }
    }

    fn parsed(&mut self, field: impl Into<String>, detail: impl Into<String>) {
        self.parsed_assertions.push(ImportIssue {
            field: field.into(),
            kind: ImportIssueKind::Parsed,
            detail: detail.into(),
        });
    }

    fn reject(&mut self, field: impl Into<String>, detail: impl Into<String>) {
        self.rejected_fields.push(ImportIssue {
            field: field.into(),
            kind: ImportIssueKind::Rejected,
            detail: detail.into(),
        });
    }

    fn lossy(&mut self, field: impl Into<String>, detail: impl Into<String>) {
        self.lossy_conversions.push(ImportIssue {
            field: field.into(),
            kind: ImportIssueKind::LossyConversion,
            detail: detail.into(),
        });
    }

    fn unresolved(&mut self, field: impl Into<String>, detail: impl Into<String>) {
        self.unresolved_references.push(ImportIssue {
            field: field.into(),
            kind: ImportIssueKind::UnresolvedReference,
            detail: detail.into(),
        });
    }

    fn review(&mut self, field: impl Into<String>, detail: impl Into<String>) {
        self.review_requirements.push(ImportIssue {
            field: field.into(),
            kind: ImportIssueKind::ReviewRequired,
            detail: detail.into(),
        });
    }

    fn unknown(&mut self, field: impl Into<String>, detail: impl Into<String>) {
        self.unknown_field_count += 1;
        self.review(field, detail);
    }

    fn status(&self, imported_any: bool) -> EdaIntakeStatus {
        if !imported_any || self.parsed_assertions.is_empty() {
            return EdaIntakeStatus::Rejected;
        }
        if !self.rejected_fields.is_empty()
            || !self.lossy_conversions.is_empty()
            || !self.unresolved_references.is_empty()
            || !self.review_requirements.is_empty()
        {
            if self.unknown_field_count > 0
                && self.rejected_fields.is_empty()
                && self.lossy_conversions.is_empty()
                && self.unresolved_references.is_empty()
            {
                EdaIntakeStatus::AcceptedWithUnknowns
            } else {
                EdaIntakeStatus::NeedsReview
            }
        } else {
            EdaIntakeStatus::Accepted
        }
    }

    fn finish(self, imported_family_count: usize, imported_variant_count: usize) -> ImportReport {
        ImportReport {
            source: self.source,
            target: ImportTargetKind::Custom("tscircuit-authoring".into()),
            imported_family_count,
            imported_variant_count,
            unknown_field_count: self.unknown_field_count,
            parsed_assertions: self.parsed_assertions,
            rejected_fields: self.rejected_fields,
            lossy_conversions: self.lossy_conversions,
            stale_sources: Vec::new(),
            unresolved_references: self.unresolved_references,
            license_notes: Vec::new(),
            review_requirements: self.review_requirements,
            warnings: self.warnings,
        }
    }
}
