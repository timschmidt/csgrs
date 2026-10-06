//! Deterministic BOM and pick-and-place views over retained circuit/layout intent.

use std::collections::{BTreeMap, BTreeSet};
use std::str::FromStr;

use hyperlattice::Point2;

use crate::{
    AssemblyVariantId, BoardId, BoardSide, Circuit, CircuitInstanceId, CoordinateFrame2,
    DeviceModelId, LandPatternId, PanelDefinition, PanelIssue, PartRef, PcbLayout, Real,
    RigidTransform2,
};

/// Stable canonical assembly-data schema identity.
pub const ASSEMBLY_DATA_SCHEMA: &str = "hypercircuit.assembly-data";
/// Current canonical assembly-data schema revision.
pub const ASSEMBLY_DATA_VERSION: u32 = 2;

/// Explicit status for assembly facts unavailable from current retained intent.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub enum AssemblyEvidence<T> {
    Provided(T),
    NotProvided { reason: String },
    NotRequired { reason: String },
}

/// One machine-neutral canonical component record.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct AssemblyComponentV2 {
    pub stable_component_id: String,
    pub reference: CircuitInstanceId,
    pub chosen_part: Option<PartRef>,
    pub dnp: bool,
    pub panel_child: Option<String>,
    pub side: Option<BoardSide>,
    pub centroid: Option<[Real; 2]>,
    pub rotation_degrees: Option<Real>,
    pub coordinate_frame: CoordinateFrame2,
    pub polarity: AssemblyEvidence<String>,
    pub pin_one: AssemblyEvidence<String>,
    pub height: AssemblyEvidence<Real>,
    pub package: Option<LandPatternId>,
    pub provenance: Vec<String>,
}

/// Canonical assembly schema v2; CSV remains a rendered adapter.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, PartialEq)]
pub struct AssemblyDataV2 {
    pub schema: String,
    pub version: u32,
    pub release: String,
    pub variant: String,
    pub panel: Option<String>,
    pub coordinate_frame: CoordinateFrame2,
    pub components: Vec<AssemblyComponentV2>,
}

/// Machine-readable CSV dialect sidecar.
#[cfg_attr(feature = "interchange", derive(serde::Deserialize, serde::Serialize))]
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AssemblyCsvDialect {
    pub schema: String,
    pub schema_version: u32,
    pub document: String,
    pub delimiter: char,
    pub quote: char,
    pub line_ending: String,
    pub encoding: String,
    pub columns: Vec<String>,
    /// JSON Schema for the decoded row object after CSV parsing.
    pub row_schema: serde_json::Value,
}

/// Variant-specific fitted-part substitution for one logical instance.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AssemblyPartOverride {
    /// Logical instance receiving the alternate fitted part.
    pub instance: CircuitInstanceId,
    /// Stable alternate part-library handle; facts remain owned by `hyperparts`.
    pub part: PartRef,
}

/// Named assembly population variant.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AssemblyVariant {
    /// Stable variant identity, such as `prototype` or `no-radio`.
    pub id: AssemblyVariantId,
    /// Explicit do-not-populate logical instances.
    pub dnp_instances: Vec<CircuitInstanceId>,
    /// Alternate fitted part handles for selected instances.
    pub part_overrides: Vec<AssemblyPartOverride>,
}

/// Invalid variant reference or contradictory population declaration.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AssemblyVariantIssue {
    /// Circuit or layout structural validation failed before variant selection.
    InvalidDesign,
    /// DNP list repeats one instance.
    DuplicateDnp(CircuitInstanceId),
    /// DNP list references no circuit instance.
    UnknownDnp(CircuitInstanceId),
    /// More than one part override targets an instance.
    DuplicatePartOverride(CircuitInstanceId),
    /// Part override references no circuit instance.
    UnknownPartOverride(CircuitInstanceId),
    /// A DNP instance also declares a fitted-part override.
    OverrideOnDnp(CircuitInstanceId),
}

/// Deterministic variant validation report.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AssemblyVariantValidationReport {
    /// Every population/reference issue in authored order.
    pub issues: Vec<AssemblyVariantIssue>,
}

impl AssemblyVariantValidationReport {
    /// True when the variant can produce unambiguous assembly outputs.
    pub fn is_valid(&self) -> bool {
        self.issues.is_empty()
    }
}

/// One grouped bill-of-materials line.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BomLine {
    /// Stable part-library handle when assigned.
    pub part: Option<PartRef>,
    /// Electrical/model identity shared by the grouped instances.
    pub model: DeviceModelId,
    /// Physical land pattern shared by the grouped instances.
    pub land_pattern: LandPatternId,
    /// Number of fitted references in the group.
    pub quantity: usize,
    /// Deterministically sorted logical references.
    pub references: Vec<CircuitInstanceId>,
}

/// One exact pick-and-place row.
#[derive(Clone, Debug, PartialEq)]
pub struct PickAndPlaceRow {
    /// Logical reference designator.
    pub reference: CircuitInstanceId,
    /// Stable part-library handle when assigned.
    pub part: Option<PartRef>,
    /// Land pattern placed for the instance.
    pub land_pattern: LandPatternId,
    /// Exact board-space placement origin.
    pub position: Point2,
    /// Exact counter-clockwise rotation in degrees.
    pub rotation_degrees: Real,
    /// Physical assembly side.
    pub side: BoardSide,
}

/// One parsed do-not-populate documentation row.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct AssemblyDnpRow {
    /// Logical reference designator.
    pub reference: CircuitInstanceId,
    /// Selected variant, when the document declares one.
    pub variant: Option<AssemblyVariantId>,
}

/// Assembly CSV document participating in a round-trip audit.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AssemblyCsvDocument {
    /// Grouped bill of materials.
    Bom,
    /// Component placement list.
    PickAndPlace,
    /// Explicit do-not-populate list.
    Dnp,
}

/// Design-owned field compared during assembly CSV reconciliation.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum AssemblyCsvField {
    /// Fitted part-library handle.
    Part,
    /// Electrical device-model identity.
    Model,
    /// Physical land-pattern identity.
    LandPattern,
    /// Exact board-space X coordinate.
    PositionX,
    /// Exact board-space Y coordinate.
    PositionY,
    /// Exact counter-clockwise rotation.
    RotationDegrees,
    /// Physical board side.
    Side,
    /// Assembly variant identity.
    Variant,
    /// Deterministic BOM grouping and ordering.
    Grouping,
}

/// One syntax, schema, or semantic mismatch found during assembly re-import.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum AssemblyRoundTripIssue {
    /// A fitted logical instance has no physical placement and cannot appear
    /// in either emitted assembly document.
    UnplacedInstance(CircuitInstanceId),
    /// CSV quoting or record termination is malformed.
    CsvSyntax {
        /// Affected document.
        document: AssemblyCsvDocument,
        /// One-based source line.
        line: usize,
        /// Parser diagnostic.
        detail: String,
    },
    /// The exact expected column header was not recovered.
    Header {
        /// Affected document.
        document: AssemblyCsvDocument,
        /// Expected header fields.
        expected: Vec<String>,
        /// Parsed header fields.
        parsed: Vec<String>,
    },
    /// A data row has the wrong number of fields.
    RowWidth {
        /// Affected document.
        document: AssemblyCsvDocument,
        /// One-based source line.
        line: usize,
        /// Expected field count.
        expected: usize,
        /// Parsed field count.
        parsed: usize,
    },
    /// A typed identifier, quantity, coordinate, rotation, or side is invalid.
    InvalidField {
        /// Affected document.
        document: AssemblyCsvDocument,
        /// One-based source line.
        line: usize,
        /// Column name.
        field: String,
        /// Original field text.
        value: String,
    },
    /// A BOM quantity differs from its recovered reference count.
    BomQuantity {
        /// One-based source line.
        line: usize,
        /// Declared quantity.
        declared: usize,
        /// Number of recovered references.
        references: usize,
    },
    /// One logical reference occurs more than once in a document.
    DuplicateReference {
        /// Affected document.
        document: AssemblyCsvDocument,
        /// Repeated logical reference.
        reference: CircuitInstanceId,
    },
    /// An expected fitted or DNP reference is absent.
    MissingReference {
        /// Affected document.
        document: AssemblyCsvDocument,
        /// Missing logical reference.
        reference: CircuitInstanceId,
    },
    /// A re-imported reference was not present in the retained output.
    UnexpectedReference {
        /// Affected document.
        document: AssemblyCsvDocument,
        /// Unexpected logical reference.
        reference: CircuitInstanceId,
    },
    /// A design-owned field differs after re-import.
    FieldMismatch {
        /// Affected document.
        document: AssemblyCsvDocument,
        /// Logical reference, or `None` for whole-document grouping.
        reference: Option<CircuitInstanceId>,
        /// Compared semantic field.
        field: AssemblyCsvField,
        /// Canonical retained value.
        expected: String,
        /// Re-imported value.
        parsed: String,
    },
}

/// Independently parsed assembly documents and exact reconciliation evidence.
#[derive(Clone, Debug, PartialEq)]
pub struct AssemblyRoundTripReport {
    /// BOM rows recovered from CSV.
    pub bom: Vec<BomLine>,
    /// Placement rows recovered from CSV.
    pub pick_and_place: Vec<PickAndPlaceRow>,
    /// DNP rows recovered from CSV.
    pub dnp: Vec<AssemblyDnpRow>,
    /// Release-blocking syntax, schema, identity, and content mismatches.
    pub issues: Vec<AssemblyRoundTripIssue>,
}

impl AssemblyRoundTripReport {
    /// Whether every design-owned assembly field round-tripped exactly.
    pub fn is_release_clean(&self) -> bool {
        self.issues.is_empty()
    }
}

/// Assembly views and explicit logical instances that currently lack placement.
#[derive(Clone, Debug, PartialEq)]
pub struct AssemblyOutputs {
    /// Selected population variant; `None` means every circuit instance is fitted.
    pub variant: Option<AssemblyVariantId>,
    /// Grouped BOM lines.
    pub bom: Vec<BomLine>,
    /// One row per placed logical instance.
    pub pick_and_place: Vec<PickAndPlaceRow>,
    /// Logical instances absent from the PCB placement set.
    pub unplaced_instances: Vec<CircuitInstanceId>,
    /// Explicit DNP instances excluded from BOM and placement output.
    pub dnp_instances: Vec<CircuitInstanceId>,
}

impl AssemblyOutputs {
    /// Derives deterministic assembly views from one structurally valid design.
    pub fn from_design(circuit: &Circuit, layout: &PcbLayout) -> Option<Self> {
        if !circuit.validate().is_valid() || !layout.validate(circuit).is_valid() {
            return None;
        }
        Some(Self::build(circuit, layout, None))
    }

    /// Derives fitted BOM/PnP and explicit DNP output for one validated variant.
    pub fn from_variant(
        circuit: &Circuit,
        layout: &PcbLayout,
        variant: &AssemblyVariant,
    ) -> Result<Self, AssemblyVariantValidationReport> {
        if !circuit.validate().is_valid() || !layout.validate(circuit).is_valid() {
            return Err(AssemblyVariantValidationReport {
                issues: vec![AssemblyVariantIssue::InvalidDesign],
            });
        }
        let validation = variant.validate(circuit);
        if !validation.is_valid() {
            return Err(validation);
        }
        Ok(Self::build(circuit, layout, Some(variant)))
    }

    fn build(circuit: &Circuit, layout: &PcbLayout, variant: Option<&AssemblyVariant>) -> Self {
        let placements = layout
            .placements
            .iter()
            .map(|placement| (placement.instance.clone(), placement))
            .collect::<BTreeMap<_, _>>();
        let dnp_instances = variant
            .map(|variant| {
                variant
                    .dnp_instances
                    .iter()
                    .cloned()
                    .collect::<BTreeSet<_>>()
            })
            .unwrap_or_default();
        let overrides = variant
            .map(|variant| {
                variant
                    .part_overrides
                    .iter()
                    .map(|override_| (override_.instance.clone(), override_.part.clone()))
                    .collect::<BTreeMap<_, _>>()
            })
            .unwrap_or_default();
        let instances = circuit
            .instances
            .iter()
            .map(|instance| (instance.id.clone(), instance))
            .collect::<BTreeMap<_, _>>();

        let mut grouped = BTreeMap::<
            (Option<PartRef>, DeviceModelId, LandPatternId),
            Vec<CircuitInstanceId>,
        >::new();
        let mut pick_and_place = Vec::new();
        let mut unplaced_instances = Vec::new();
        for (id, instance) in &instances {
            if dnp_instances.contains(id) {
                continue;
            }
            let Some(placement) = placements.get(id) else {
                unplaced_instances.push(id.clone());
                continue;
            };
            let part = overrides.get(id).cloned().or_else(|| instance.part.clone());
            grouped
                .entry((
                    part.clone(),
                    instance.model.clone(),
                    placement.land_pattern.clone(),
                ))
                .or_default()
                .push(id.clone());
            pick_and_place.push(PickAndPlaceRow {
                reference: id.clone(),
                part,
                land_pattern: placement.land_pattern.clone(),
                position: placement.position.clone(),
                rotation_degrees: placement.rotation_degrees.clone(),
                side: placement.side,
            });
        }
        pick_and_place.sort_by(|left, right| left.reference.cmp(&right.reference));
        let bom = grouped
            .into_iter()
            .map(|((part, model, land_pattern), mut references)| {
                references.sort();
                BomLine {
                    part,
                    model,
                    land_pattern,
                    quantity: references.len(),
                    references,
                }
            })
            .collect();
        Self {
            variant: variant.map(|variant| variant.id.clone()),
            bom,
            pick_and_place,
            unplaced_instances,
            dnp_instances: dnp_instances.into_iter().collect(),
        }
    }

    /// Writes a deterministic reviewable BOM CSV.
    pub fn bom_csv(&self) -> String {
        let mut output = "quantity,references,part,model,land_pattern\n".to_owned();
        for line in &self.bom {
            let references = line
                .references
                .iter()
                .map(CircuitInstanceId::as_str)
                .collect::<Vec<_>>()
                .join(" ");
            output.push_str(&format!(
                "{},{},{},{},{}\n",
                line.quantity,
                csv(&references),
                csv(line.part.as_ref().map(PartRef::as_str).unwrap_or("")),
                csv(line.model.as_str()),
                csv(line.land_pattern.as_str())
            ));
        }
        output
    }

    /// Writes deterministic exact-coordinate pick-and-place CSV.
    pub fn pick_and_place_csv(&self) -> String {
        let mut output = "reference,part,land_pattern,x,y,rotation_degrees,side\n".to_owned();
        for row in &self.pick_and_place {
            output.push_str(&format!(
                "{},{},{},{},{},{},{}\n",
                csv(row.reference.as_str()),
                csv(row.part.as_ref().map(PartRef::as_str).unwrap_or("")),
                csv(row.land_pattern.as_str()),
                csv(&row.position.x.to_string()),
                csv(&row.position.y.to_string()),
                csv(&row.rotation_degrees.to_string()),
                match row.side {
                    BoardSide::Front => "front",
                    BoardSide::Back => "back",
                }
            ));
        }
        output
    }

    /// Writes an explicit DNP list for assembly documentation.
    pub fn dnp_csv(&self) -> String {
        let mut output = "reference,variant\n".to_owned();
        for instance in &self.dnp_instances {
            output.push_str(&format!(
                "{},{}\n",
                csv(instance.as_str()),
                csv(self
                    .variant
                    .as_ref()
                    .map(AssemblyVariantId::as_str)
                    .unwrap_or(""))
            ));
        }
        output
    }

    /// Builds canonical assembly schema v2 using exact retained coordinates.
    pub fn canonical_v2(
        &self,
        release: impl Into<String>,
        frame: CoordinateFrame2,
        panel: Option<String>,
        board_to_panel: Option<&RigidTransform2>,
    ) -> AssemblyDataV2 {
        let mut components = self
            .pick_and_place
            .iter()
            .map(|row| {
                let centroid = [row.position.x.clone(), row.position.y.clone()];
                let centroid =
                    board_to_panel.map_or(centroid.clone(), |transform| transform.apply(centroid));
                AssemblyComponentV2 {
                    stable_component_id: format!("component:{}", row.reference.as_str()),
                    reference: row.reference.clone(),
                    chosen_part: row.part.clone(),
                    dnp: false,
                    panel_child: None,
                    side: Some(row.side),
                    centroid: Some(centroid),
                    rotation_degrees: Some(row.rotation_degrees.clone()),
                    coordinate_frame: frame.clone(),
                    polarity: AssemblyEvidence::NotProvided {
                        reason: "no typed polarity marker retained for this component".into(),
                    },
                    pin_one: AssemblyEvidence::NotProvided {
                        reason: "no typed pin-one marker retained for this component".into(),
                    },
                    height: AssemblyEvidence::NotProvided {
                        reason: "no selected-package height evidence retained".into(),
                    },
                    package: Some(row.land_pattern.clone()),
                    provenance: row
                        .part
                        .as_ref()
                        .map(|part| vec![format!("part:{}", part.as_str())])
                        .unwrap_or_default(),
                }
            })
            .collect::<Vec<_>>();
        components.extend(
            self.dnp_instances
                .iter()
                .map(|reference| AssemblyComponentV2 {
                    stable_component_id: format!("component:{}", reference.as_str()),
                    reference: reference.clone(),
                    chosen_part: None,
                    dnp: true,
                    panel_child: None,
                    side: None,
                    centroid: None,
                    rotation_degrees: None,
                    coordinate_frame: frame.clone(),
                    polarity: AssemblyEvidence::NotRequired {
                        reason: "component is not populated in this variant".into(),
                    },
                    pin_one: AssemblyEvidence::NotRequired {
                        reason: "component is not populated in this variant".into(),
                    },
                    height: AssemblyEvidence::NotRequired {
                        reason: "component is not populated in this variant".into(),
                    },
                    package: None,
                    provenance: Vec::new(),
                }),
        );
        components.sort_by(|left, right| left.reference.cmp(&right.reference));
        AssemblyDataV2 {
            schema: ASSEMBLY_DATA_SCHEMA.into(),
            version: ASSEMBLY_DATA_VERSION,
            release: release.into(),
            variant: self
                .variant
                .as_ref()
                .map(AssemblyVariantId::as_str)
                .unwrap_or("default")
                .into(),
            panel,
            coordinate_frame: frame,
            components,
        }
    }

    /// Expands canonical assembly records for every matching child in a panel.
    pub fn canonical_v2_for_panel(
        &self,
        release: impl Into<String>,
        panel: &PanelDefinition,
        board: &BoardId,
    ) -> Result<AssemblyDataV2, PanelIssue> {
        let release = release.into();
        let children = panel.children_for_board(board).collect::<Vec<_>>();
        if children.is_empty() {
            return Err(PanelIssue::BoardNotPresent(board.as_str().into()));
        }
        let mut components = Vec::new();
        for child in children {
            let mut child_data = self.canonical_v2(
                release.clone(),
                panel.frame.clone(),
                Some(panel.id.clone()),
                Some(&child.transform),
            );
            for component in &mut child_data.components {
                component.stable_component_id = format!(
                    "panel:{}/child:{}/{}",
                    panel.id, child.id, component.stable_component_id
                );
                component.panel_child = Some(child.id.clone());
            }
            components.extend(child_data.components);
        }
        components.sort_by(|left, right| {
            left.panel_child
                .cmp(&right.panel_child)
                .then_with(|| left.reference.cmp(&right.reference))
        });
        Ok(AssemblyDataV2 {
            schema: ASSEMBLY_DATA_SCHEMA.into(),
            version: ASSEMBLY_DATA_VERSION,
            release,
            variant: self
                .variant
                .as_ref()
                .map(AssemblyVariantId::as_str)
                .unwrap_or("default")
                .into(),
            panel: Some(panel.id.clone()),
            coordinate_frame: panel.frame.clone(),
            components,
        })
    }

    /// Returns deterministic JSON for the canonical assembly schema.
    #[cfg(feature = "interchange")]
    pub fn canonical_v2_json(
        &self,
        release: impl Into<String>,
        frame: CoordinateFrame2,
        panel: Option<String>,
        board_to_panel: Option<&RigidTransform2>,
    ) -> String {
        serde_json::to_string_pretty(&self.canonical_v2(release, frame, panel, board_to_panel))
            .expect("canonical assembly data is infallibly serializable")
    }

    /// JSON Schema for the stable top-level assembly-data v2 contract.
    pub fn canonical_v2_json_schema() -> serde_json::Value {
        serde_json::json!({
            "$schema": "https://json-schema.org/draft/2020-12/schema",
            "$id": "urn:hypercircuit:assembly-data:2",
            "title": "HyperCircuit canonical assembly data v2",
            "type": "object",
            "additionalProperties": false,
            "required": [
                "schema",
                "version",
                "release",
                "variant",
                "panel",
                "coordinate_frame",
                "components"
            ],
            "properties": {
                "schema": { "const": ASSEMBLY_DATA_SCHEMA },
                "version": { "const": ASSEMBLY_DATA_VERSION },
                "release": { "type": "string", "minLength": 1 },
                "variant": { "type": "string", "minLength": 1 },
                "panel": { "type": ["string", "null"] },
                "coordinate_frame": { "type": "object" },
                "components": {
                    "type": "array",
                    "items": {
                        "type": "object",
                        "additionalProperties": false,
                        "required": [
                            "stable_component_id",
                            "reference",
                            "chosen_part",
                            "dnp",
                            "panel_child",
                            "side",
                            "centroid",
                            "rotation_degrees",
                            "coordinate_frame",
                            "polarity",
                            "pin_one",
                            "height",
                            "package",
                            "provenance"
                        ],
                        "properties": {
                            "stable_component_id": { "type": "string", "minLength": 1 },
                            "reference": { "type": "string", "minLength": 1 },
                            "chosen_part": { "type": ["string", "null"] },
                            "dnp": { "type": "boolean" },
                            "panel_child": { "type": ["string", "null"] },
                            "side": {},
                            "centroid": { "type": ["array", "null"], "minItems": 2, "maxItems": 2 },
                            "rotation_degrees": {},
                            "coordinate_frame": { "type": "object" },
                            "polarity": { "type": "object" },
                            "pin_one": { "type": "object" },
                            "height": { "type": "object" },
                            "package": { "type": ["string", "null"] },
                            "provenance": {
                                "type": "array",
                                "items": { "type": "string" }
                            }
                        }
                    }
                }
            }
        })
    }

    /// Describes each rendered CSV's exact parser dialect and columns.
    pub fn csv_dialect(document: AssemblyCsvDocument) -> AssemblyCsvDialect {
        let (name, columns) = match document {
            AssemblyCsvDocument::Bom => (
                "bom",
                vec!["quantity", "references", "part", "model", "land_pattern"],
            ),
            AssemblyCsvDocument::PickAndPlace => (
                "pick_and_place",
                vec![
                    "reference",
                    "part",
                    "land_pattern",
                    "x",
                    "y",
                    "rotation_degrees",
                    "side",
                ],
            ),
            AssemblyCsvDocument::Dnp => ("dnp", vec!["reference", "variant"]),
        };
        AssemblyCsvDialect {
            schema: "hypercircuit.assembly-csv-dialect".into(),
            schema_version: 1,
            document: name.into(),
            delimiter: ',',
            quote: '"',
            line_ending: "LF".into(),
            encoding: "UTF-8".into(),
            columns: columns.into_iter().map(str::to_owned).collect(),
            row_schema: Self::csv_row_json_schema(document),
        }
    }

    /// JSON Schema for one decoded row of a rendered assembly CSV.
    pub fn csv_row_json_schema(document: AssemblyCsvDocument) -> serde_json::Value {
        let dialect = match document {
            AssemblyCsvDocument::Bom => (
                "bom",
                vec!["quantity", "references", "part", "model", "land_pattern"],
            ),
            AssemblyCsvDocument::PickAndPlace => (
                "pick-and-place",
                vec![
                    "reference",
                    "part",
                    "land_pattern",
                    "x",
                    "y",
                    "rotation_degrees",
                    "side",
                ],
            ),
            AssemblyCsvDocument::Dnp => ("dnp", vec!["reference", "variant"]),
        };
        let properties = dialect
            .1
            .iter()
            .map(|column| {
                (
                    (*column).to_owned(),
                    serde_json::json!({ "type": "string" }),
                )
            })
            .collect::<serde_json::Map<_, _>>();
        serde_json::json!({
            "$schema": "https://json-schema.org/draft/2020-12/schema",
            "$id": format!("urn:hypercircuit:assembly-csv-row:{}:1", dialect.0),
            "type": "object",
            "additionalProperties": false,
            "required": dialect.1,
            "properties": properties
        })
    }

    /// Serializes and independently re-imports all assembly CSV documents.
    pub fn audit_csv_round_trip(&self) -> AssemblyRoundTripReport {
        self.audit_csv_documents(&self.bom_csv(), &self.pick_and_place_csv(), &self.dnp_csv())
    }

    /// Re-imports caller-supplied assembly CSV and reconciles every
    /// design-owned field with this retained output.
    pub fn audit_csv_documents(
        &self,
        bom_csv: &str,
        pick_and_place_csv: &str,
        dnp_csv: &str,
    ) -> AssemblyRoundTripReport {
        let mut issues = self
            .unplaced_instances
            .iter()
            .cloned()
            .map(AssemblyRoundTripIssue::UnplacedInstance)
            .collect::<Vec<_>>();
        let bom = parse_bom_csv(bom_csv, &mut issues);
        let pick_and_place = parse_pick_and_place_csv(pick_and_place_csv, &mut issues);
        let dnp = parse_dnp_csv(dnp_csv, &mut issues);
        reconcile_bom(&self.bom, &bom, &mut issues);
        reconcile_pick_and_place(&self.pick_and_place, &pick_and_place, &mut issues);
        reconcile_dnp(self, &dnp, &mut issues);
        AssemblyRoundTripReport {
            bom,
            pick_and_place,
            dnp,
            issues,
        }
    }
}

impl AssemblyVariant {
    /// Validates population references and contradictory DNP/substitution intent.
    pub fn validate(&self, circuit: &Circuit) -> AssemblyVariantValidationReport {
        let instances = circuit
            .instances
            .iter()
            .map(|instance| instance.id.clone())
            .collect::<BTreeSet<_>>();
        let mut issues = Vec::new();
        let mut dnp = BTreeSet::new();
        for instance in &self.dnp_instances {
            if !dnp.insert(instance.clone()) {
                issues.push(AssemblyVariantIssue::DuplicateDnp(instance.clone()));
            }
            if !instances.contains(instance) {
                issues.push(AssemblyVariantIssue::UnknownDnp(instance.clone()));
            }
        }
        let mut overrides = BTreeSet::new();
        for override_ in &self.part_overrides {
            if !overrides.insert(override_.instance.clone()) {
                issues.push(AssemblyVariantIssue::DuplicatePartOverride(
                    override_.instance.clone(),
                ));
            }
            if !instances.contains(&override_.instance) {
                issues.push(AssemblyVariantIssue::UnknownPartOverride(
                    override_.instance.clone(),
                ));
            }
            if dnp.contains(&override_.instance) {
                issues.push(AssemblyVariantIssue::OverrideOnDnp(
                    override_.instance.clone(),
                ));
            }
        }
        AssemblyVariantValidationReport { issues }
    }
}

fn csv(value: &str) -> String {
    if value.contains([',', '"', '\n', '\r']) {
        format!("\"{}\"", value.replace('"', "\"\""))
    } else {
        value.to_owned()
    }
}

#[derive(Clone, Debug)]
struct CsvRecord {
    line: usize,
    fields: Vec<String>,
}

fn parse_csv(
    input: &str,
    document: AssemblyCsvDocument,
) -> Result<Vec<CsvRecord>, AssemblyRoundTripIssue> {
    let mut records = Vec::new();
    let mut fields = Vec::new();
    let mut field = String::new();
    let mut chars = input.chars().peekable();
    let mut line = 1;
    let mut record_line = 1;
    let mut quoted = false;
    let mut quote_closed = false;
    let mut field_started = false;

    while let Some(ch) = chars.next() {
        if quoted {
            match ch {
                '"' if chars.peek() == Some(&'"') => {
                    chars.next();
                    field.push('"');
                }
                '"' => {
                    quoted = false;
                    quote_closed = true;
                }
                '\n' => {
                    field.push('\n');
                    line += 1;
                }
                _ => field.push(ch),
            }
            continue;
        }
        if quote_closed && !matches!(ch, ',' | '\n' | '\r') {
            return Err(AssemblyRoundTripIssue::CsvSyntax {
                document,
                line,
                detail: "characters follow a closing quote".into(),
            });
        }
        match ch {
            '"' if !field_started => {
                quoted = true;
                field_started = true;
            }
            '"' => {
                return Err(AssemblyRoundTripIssue::CsvSyntax {
                    document,
                    line,
                    detail: "quote appears inside an unquoted field".into(),
                });
            }
            ',' => {
                fields.push(std::mem::take(&mut field));
                field_started = false;
                quote_closed = false;
            }
            '\n' => {
                fields.push(std::mem::take(&mut field));
                records.push(CsvRecord {
                    line: record_line,
                    fields: std::mem::take(&mut fields),
                });
                line += 1;
                record_line = line;
                field_started = false;
                quote_closed = false;
            }
            '\r' if chars.peek() == Some(&'\n') => {}
            '\r' => {
                fields.push(std::mem::take(&mut field));
                records.push(CsvRecord {
                    line: record_line,
                    fields: std::mem::take(&mut fields),
                });
                line += 1;
                record_line = line;
                field_started = false;
                quote_closed = false;
            }
            _ => {
                field.push(ch);
                field_started = true;
            }
        }
    }
    if quoted {
        return Err(AssemblyRoundTripIssue::CsvSyntax {
            document,
            line,
            detail: "unterminated quoted field".into(),
        });
    }
    if field_started || quote_closed || !fields.is_empty() {
        fields.push(field);
        records.push(CsvRecord {
            line: record_line,
            fields,
        });
    }
    Ok(records)
}

fn records_with_header(
    input: &str,
    document: AssemblyCsvDocument,
    expected: &[&str],
    issues: &mut Vec<AssemblyRoundTripIssue>,
) -> Vec<CsvRecord> {
    let records = match parse_csv(input, document) {
        Ok(records) => records,
        Err(issue) => {
            issues.push(issue);
            return Vec::new();
        }
    };
    let Some((header, rows)) = records.split_first() else {
        issues.push(AssemblyRoundTripIssue::Header {
            document,
            expected: expected.iter().map(|field| (*field).to_owned()).collect(),
            parsed: Vec::new(),
        });
        return Vec::new();
    };
    let expected = expected
        .iter()
        .map(|field| (*field).to_owned())
        .collect::<Vec<_>>();
    if header.fields != expected {
        issues.push(AssemblyRoundTripIssue::Header {
            document,
            expected,
            parsed: header.fields.clone(),
        });
        return Vec::new();
    }
    rows.to_vec()
}

fn row_width(
    record: &CsvRecord,
    document: AssemblyCsvDocument,
    expected: usize,
    issues: &mut Vec<AssemblyRoundTripIssue>,
) -> bool {
    if record.fields.len() == expected {
        true
    } else {
        issues.push(AssemblyRoundTripIssue::RowWidth {
            document,
            line: record.line,
            expected,
            parsed: record.fields.len(),
        });
        false
    }
}

fn invalid_field(
    document: AssemblyCsvDocument,
    record: &CsvRecord,
    field: &str,
    index: usize,
) -> AssemblyRoundTripIssue {
    AssemblyRoundTripIssue::InvalidField {
        document,
        line: record.line,
        field: field.into(),
        value: record.fields[index].clone(),
    }
}

fn parse_bom_csv(input: &str, issues: &mut Vec<AssemblyRoundTripIssue>) -> Vec<BomLine> {
    let document = AssemblyCsvDocument::Bom;
    let records = records_with_header(
        input,
        document,
        &["quantity", "references", "part", "model", "land_pattern"],
        issues,
    );
    let mut output = Vec::new();
    let mut seen = BTreeSet::new();
    for record in records {
        if !row_width(&record, document, 5, issues) {
            continue;
        }
        let Ok(quantity) = record.fields[0].parse::<usize>() else {
            issues.push(invalid_field(document, &record, "quantity", 0));
            continue;
        };
        let mut references = Vec::new();
        let mut valid = true;
        for value in record.fields[1].split_whitespace() {
            match CircuitInstanceId::new(value) {
                Ok(reference) => {
                    if !seen.insert(reference.clone()) {
                        issues.push(AssemblyRoundTripIssue::DuplicateReference {
                            document,
                            reference: reference.clone(),
                        });
                    }
                    references.push(reference);
                }
                Err(_) => {
                    issues.push(invalid_field(document, &record, "references", 1));
                    valid = false;
                }
            }
        }
        if quantity != references.len() {
            issues.push(AssemblyRoundTripIssue::BomQuantity {
                line: record.line,
                declared: quantity,
                references: references.len(),
            });
        }
        let part = if record.fields[2].is_empty() {
            None
        } else {
            match PartRef::new(record.fields[2].clone()) {
                Ok(part) => Some(part),
                Err(_) => {
                    issues.push(invalid_field(document, &record, "part", 2));
                    valid = false;
                    None
                }
            }
        };
        let model = match DeviceModelId::new(record.fields[3].clone()) {
            Ok(model) => model,
            Err(_) => {
                issues.push(invalid_field(document, &record, "model", 3));
                continue;
            }
        };
        let land_pattern = match LandPatternId::new(record.fields[4].clone()) {
            Ok(pattern) => pattern,
            Err(_) => {
                issues.push(invalid_field(document, &record, "land_pattern", 4));
                continue;
            }
        };
        if valid {
            output.push(BomLine {
                part,
                model,
                land_pattern,
                quantity,
                references,
            });
        }
    }
    output
}

fn parse_pick_and_place_csv(
    input: &str,
    issues: &mut Vec<AssemblyRoundTripIssue>,
) -> Vec<PickAndPlaceRow> {
    let document = AssemblyCsvDocument::PickAndPlace;
    let records = records_with_header(
        input,
        document,
        &[
            "reference",
            "part",
            "land_pattern",
            "x",
            "y",
            "rotation_degrees",
            "side",
        ],
        issues,
    );
    let mut output = Vec::new();
    let mut seen = BTreeSet::new();
    for record in records {
        if !row_width(&record, document, 7, issues) {
            continue;
        }
        let reference = match CircuitInstanceId::new(record.fields[0].clone()) {
            Ok(reference) => reference,
            Err(_) => {
                issues.push(invalid_field(document, &record, "reference", 0));
                continue;
            }
        };
        if !seen.insert(reference.clone()) {
            issues.push(AssemblyRoundTripIssue::DuplicateReference {
                document,
                reference: reference.clone(),
            });
        }
        let part = if record.fields[1].is_empty() {
            None
        } else {
            match PartRef::new(record.fields[1].clone()) {
                Ok(part) => Some(part),
                Err(_) => {
                    issues.push(invalid_field(document, &record, "part", 1));
                    continue;
                }
            }
        };
        let land_pattern = match LandPatternId::new(record.fields[2].clone()) {
            Ok(pattern) => pattern,
            Err(_) => {
                issues.push(invalid_field(document, &record, "land_pattern", 2));
                continue;
            }
        };
        let x = match Real::from_str(&record.fields[3]) {
            Ok(value) => value,
            Err(_) => {
                issues.push(invalid_field(document, &record, "x", 3));
                continue;
            }
        };
        let y = match Real::from_str(&record.fields[4]) {
            Ok(value) => value,
            Err(_) => {
                issues.push(invalid_field(document, &record, "y", 4));
                continue;
            }
        };
        let rotation_degrees = match Real::from_str(&record.fields[5]) {
            Ok(value) => value,
            Err(_) => {
                issues.push(invalid_field(document, &record, "rotation_degrees", 5));
                continue;
            }
        };
        let side = match record.fields[6].as_str() {
            "front" => BoardSide::Front,
            "back" => BoardSide::Back,
            _ => {
                issues.push(invalid_field(document, &record, "side", 6));
                continue;
            }
        };
        output.push(PickAndPlaceRow {
            reference,
            part,
            land_pattern,
            position: Point2::new(x, y),
            rotation_degrees,
            side,
        });
    }
    output
}

fn parse_dnp_csv(input: &str, issues: &mut Vec<AssemblyRoundTripIssue>) -> Vec<AssemblyDnpRow> {
    let document = AssemblyCsvDocument::Dnp;
    let records = records_with_header(input, document, &["reference", "variant"], issues);
    let mut output = Vec::new();
    let mut seen = BTreeSet::new();
    for record in records {
        if !row_width(&record, document, 2, issues) {
            continue;
        }
        let reference = match CircuitInstanceId::new(record.fields[0].clone()) {
            Ok(reference) => reference,
            Err(_) => {
                issues.push(invalid_field(document, &record, "reference", 0));
                continue;
            }
        };
        if !seen.insert(reference.clone()) {
            issues.push(AssemblyRoundTripIssue::DuplicateReference {
                document,
                reference: reference.clone(),
            });
        }
        let variant = if record.fields[1].is_empty() {
            None
        } else {
            match AssemblyVariantId::new(record.fields[1].clone()) {
                Ok(variant) => Some(variant),
                Err(_) => {
                    issues.push(invalid_field(document, &record, "variant", 1));
                    continue;
                }
            }
        };
        output.push(AssemblyDnpRow { reference, variant });
    }
    output
}

fn reconcile_reference_sets<T, U>(
    document: AssemblyCsvDocument,
    expected: &BTreeMap<CircuitInstanceId, T>,
    parsed: &BTreeMap<CircuitInstanceId, U>,
    issues: &mut Vec<AssemblyRoundTripIssue>,
) {
    for reference in expected.keys() {
        if !parsed.contains_key(reference) {
            issues.push(AssemblyRoundTripIssue::MissingReference {
                document,
                reference: reference.clone(),
            });
        }
    }
    for reference in parsed.keys() {
        if !expected.contains_key(reference) {
            issues.push(AssemblyRoundTripIssue::UnexpectedReference {
                document,
                reference: reference.clone(),
            });
        }
    }
}

fn mismatch(
    document: AssemblyCsvDocument,
    reference: Option<&CircuitInstanceId>,
    field: AssemblyCsvField,
    expected: impl Into<String>,
    parsed: impl Into<String>,
    issues: &mut Vec<AssemblyRoundTripIssue>,
) {
    issues.push(AssemblyRoundTripIssue::FieldMismatch {
        document,
        reference: reference.cloned(),
        field,
        expected: expected.into(),
        parsed: parsed.into(),
    });
}

fn optional_part(part: Option<&PartRef>) -> &str {
    part.map(PartRef::as_str).unwrap_or("")
}

fn reconcile_bom(
    expected: &[BomLine],
    parsed: &[BomLine],
    issues: &mut Vec<AssemblyRoundTripIssue>,
) {
    type BomFacts = (Option<PartRef>, DeviceModelId, LandPatternId);
    let flatten = |lines: &[BomLine]| {
        lines
            .iter()
            .flat_map(|line| {
                line.references.iter().cloned().map(|reference| {
                    (
                        reference,
                        (
                            line.part.clone(),
                            line.model.clone(),
                            line.land_pattern.clone(),
                        ),
                    )
                })
            })
            .collect::<BTreeMap<CircuitInstanceId, BomFacts>>()
    };
    let expected_map = flatten(expected);
    let parsed_map = flatten(parsed);
    reconcile_reference_sets(AssemblyCsvDocument::Bom, &expected_map, &parsed_map, issues);
    for (reference, (expected_part, expected_model, expected_pattern)) in &expected_map {
        let Some((parsed_part, parsed_model, parsed_pattern)) = parsed_map.get(reference) else {
            continue;
        };
        if expected_part != parsed_part {
            mismatch(
                AssemblyCsvDocument::Bom,
                Some(reference),
                AssemblyCsvField::Part,
                optional_part(expected_part.as_ref()),
                optional_part(parsed_part.as_ref()),
                issues,
            );
        }
        if expected_model != parsed_model {
            mismatch(
                AssemblyCsvDocument::Bom,
                Some(reference),
                AssemblyCsvField::Model,
                expected_model.as_str(),
                parsed_model.as_str(),
                issues,
            );
        }
        if expected_pattern != parsed_pattern {
            mismatch(
                AssemblyCsvDocument::Bom,
                Some(reference),
                AssemblyCsvField::LandPattern,
                expected_pattern.as_str(),
                parsed_pattern.as_str(),
                issues,
            );
        }
    }
    if expected_map == parsed_map && expected != parsed {
        mismatch(
            AssemblyCsvDocument::Bom,
            None,
            AssemblyCsvField::Grouping,
            "canonical grouped BOM",
            "noncanonical grouping or ordering",
            issues,
        );
    }
}

fn reconcile_pick_and_place(
    expected: &[PickAndPlaceRow],
    parsed: &[PickAndPlaceRow],
    issues: &mut Vec<AssemblyRoundTripIssue>,
) {
    let expected = expected
        .iter()
        .map(|row| (row.reference.clone(), row))
        .collect::<BTreeMap<_, _>>();
    let parsed = parsed
        .iter()
        .map(|row| (row.reference.clone(), row))
        .collect::<BTreeMap<_, _>>();
    reconcile_reference_sets(
        AssemblyCsvDocument::PickAndPlace,
        &expected,
        &parsed,
        issues,
    );
    for (reference, expected) in &expected {
        let Some(parsed) = parsed.get(reference) else {
            continue;
        };
        let comparisons = [
            (
                AssemblyCsvField::Part,
                optional_part(expected.part.as_ref()).to_owned(),
                optional_part(parsed.part.as_ref()).to_owned(),
            ),
            (
                AssemblyCsvField::LandPattern,
                expected.land_pattern.as_str().to_owned(),
                parsed.land_pattern.as_str().to_owned(),
            ),
            (
                AssemblyCsvField::PositionX,
                expected.position.x.to_string(),
                parsed.position.x.to_string(),
            ),
            (
                AssemblyCsvField::PositionY,
                expected.position.y.to_string(),
                parsed.position.y.to_string(),
            ),
            (
                AssemblyCsvField::RotationDegrees,
                expected.rotation_degrees.to_string(),
                parsed.rotation_degrees.to_string(),
            ),
            (
                AssemblyCsvField::Side,
                format!("{:?}", expected.side),
                format!("{:?}", parsed.side),
            ),
        ];
        for (field, expected_value, parsed_value) in comparisons {
            if expected_value != parsed_value {
                mismatch(
                    AssemblyCsvDocument::PickAndPlace,
                    Some(reference),
                    field,
                    expected_value,
                    parsed_value,
                    issues,
                );
            }
        }
    }
}

fn reconcile_dnp(
    expected: &AssemblyOutputs,
    parsed: &[AssemblyDnpRow],
    issues: &mut Vec<AssemblyRoundTripIssue>,
) {
    let expected = expected
        .dnp_instances
        .iter()
        .map(|reference| (reference.clone(), expected.variant.as_ref()))
        .collect::<BTreeMap<_, _>>();
    let parsed = parsed
        .iter()
        .map(|row| (row.reference.clone(), row.variant.as_ref()))
        .collect::<BTreeMap<_, _>>();
    reconcile_reference_sets(AssemblyCsvDocument::Dnp, &expected, &parsed, issues);
    for (reference, expected_variant) in &expected {
        let Some(parsed_variant) = parsed.get(reference) else {
            continue;
        };
        if expected_variant != parsed_variant {
            mismatch(
                AssemblyCsvDocument::Dnp,
                Some(reference),
                AssemblyCsvField::Variant,
                expected_variant
                    .map(AssemblyVariantId::as_str)
                    .unwrap_or(""),
                parsed_variant.map(AssemblyVariantId::as_str).unwrap_or(""),
                issues,
            );
        }
    }
}
