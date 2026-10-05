//! Bills of materials derived from an assembly document.
//!
//! Quantities multiply instance counts from the root. A step marked
//! `merge_into_parent` contributes its parts to the nearest ancestor column
//! and does not receive a column of its own.

use super::assembly::{AssemblyDocument, AssemblyNode, NodeKind, instance_count};
use super::metadata::NodeId;

/// BOM section a line belongs to.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd)]
pub enum BomCategory {
    /// Catalog part.
    Vitamin,
    /// Printed solid.
    Printed,
    /// Routed sheet.
    Routed,
    /// Named build step.
    Subassembly,
}

/// One counted part or subassembly.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BomLine {
    /// Section.
    pub category: BomCategory,
    /// Stable key. Vitamins use the call text before the colon. Fabricated
    /// parts use the node name.
    pub key: String,
    /// Text shown in the table.
    pub description: String,
    /// Project-wide quantity, after ancestor instance counts.
    pub quantity: u64,
    /// Node that contributed the line.
    pub node: NodeId,
}

/// A named step and the parts directly inside it.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BomSubassembly {
    /// Child step.
    pub node: NodeId,
    /// Child name.
    pub name: String,
    /// How many of that step this parent uses.
    pub quantity: u64,
}

/// One step in the hierarchical BOM.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BomAssembly {
    /// Step node.
    pub node: NodeId,
    /// Step name.
    pub name: String,
    /// How many of this step the project builds.
    pub quantity: u64,
    /// Parts of this step are folded into the parent column.
    pub merge_into_parent: bool,
    /// Printed, routed, and vitamin lines directly inside the step.
    pub lines: Vec<BomLine>,
    /// Child steps.
    pub subassemblies: Vec<BomSubassembly>,
}

/// Flat and hierarchical bills, plus the columns of the global table.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct BomReport {
    /// Every step in tree order.
    pub hierarchical: Vec<BomAssembly>,
    /// Parts summed across the project, vitamins sorted by description.
    pub flat: Vec<BomLine>,
    /// Steps that own a column in the global table.
    pub columns: Vec<BomAssembly>,
}

/// Builds the bill of materials for `document`.
pub fn bill_of_materials(document: &AssemblyDocument) -> BomReport {
    let mut hierarchical = Vec::new();
    collect_steps(document, document.root(), &mut hierarchical);
    let flat = flatten(&hierarchical);
    let columns = hierarchical
        .iter()
        .filter(|assembly| !assembly.merge_into_parent)
        .map(|assembly| column_assembly(document, &hierarchical, assembly.node))
        .collect();
    BomReport {
        hierarchical,
        flat,
        columns,
    }
}

fn collect_steps(document: &AssemblyDocument, id: NodeId, output: &mut Vec<BomAssembly>) {
    let Some(node) = document.node(id) else {
        return;
    };
    if node.kind != NodeKind::Step {
        return;
    }
    let quantity = instance_count(document, id).unwrap_or(0);
    let mut lines = Vec::new();
    let mut subassemblies = Vec::new();
    let mut child_steps = Vec::new();
    for child_id in &node.children {
        let Some(child) = document.node(*child_id) else {
            continue;
        };
        if child.kind == NodeKind::Step {
            let child_quantity = instance_count(document, *child_id).unwrap_or(0);
            subassemblies.push(BomSubassembly {
                node: *child_id,
                name: child.name.clone(),
                quantity: child_quantity,
            });
            child_steps.push(*child_id);
        } else if let Some(line) = part_line(document, child, *child_id) {
            lines.push(line);
        }
    }
    sort_lines(&mut lines);
    output.push(BomAssembly {
        node: id,
        name: node.name.clone(),
        quantity,
        merge_into_parent: node.bom.merge_into_parent,
        lines,
        subassemblies,
    });
    for child in child_steps {
        collect_steps(document, child, output);
    }
}

fn part_line(document: &AssemblyDocument, node: &AssemblyNode, id: NodeId) -> Option<BomLine> {
    if node.bom.hidden || node.kind == NodeKind::Reference {
        return None;
    }
    let category = match node.kind {
        NodeKind::Vitamin => BomCategory::Vitamin,
        NodeKind::Printed => BomCategory::Printed,
        NodeKind::Routed => BomCategory::Routed,
        NodeKind::Step | NodeKind::Reference => return None,
    };
    let quantity = instance_count(document, id)?;
    let description = if node.bom.description.is_empty() {
        node.name.clone()
    } else {
        node.bom.description.clone()
    };
    Some(BomLine {
        category,
        key: line_key(category, &node.name, &description),
        description,
        quantity,
        node: id,
    })
}

fn line_key(category: BomCategory, name: &str, description: &str) -> String {
    match category {
        BomCategory::Vitamin => description
            .split_once(':')
            .map(|(call, _)| call.trim().to_string())
            .filter(|call| !call.is_empty())
            .unwrap_or_else(|| name.to_string()),
        BomCategory::Printed | BomCategory::Routed | BomCategory::Subassembly => {
            name.to_string()
        },
    }
}

fn sort_lines(lines: &mut [BomLine]) {
    lines.sort_by(|left, right| {
        left.category
            .cmp(&right.category)
            .then_with(|| sort_text(left).cmp(&sort_text(right)))
            .then_with(|| left.key.cmp(&right.key))
    });
}

fn sort_text(line: &BomLine) -> String {
    if line.category == BomCategory::Vitamin {
        line.description
            .split_once(':')
            .map(|(_, description)| description.trim().to_string())
            .unwrap_or_else(|| line.description.clone())
    } else {
        line.description.clone()
    }
}

fn flatten(hierarchical: &[BomAssembly]) -> Vec<BomLine> {
    let mut lines = Vec::new();
    for assembly in hierarchical {
        for line in &assembly.lines {
            if let Some(existing) = lines.iter_mut().find(|existing: &&mut BomLine| {
                existing.category == line.category && existing.key == line.key
            }) {
                existing.quantity = existing.quantity.saturating_add(line.quantity);
            } else {
                lines.push(line.clone());
            }
        }
    }
    sort_lines(&mut lines);
    lines
}

fn column_assembly(
    document: &AssemblyDocument,
    hierarchical: &[BomAssembly],
    step: NodeId,
) -> BomAssembly {
    let assembly = hierarchical
        .iter()
        .find(|assembly| assembly.node == step)
        .expect("column step is in the hierarchical BOM");
    let mut lines = assembly.lines.clone();
    let mut subassemblies = Vec::new();
    if let Some(node) = document.node(step) {
        for child in &node.children {
            let Some(child_node) = document.node(*child) else {
                continue;
            };
            if child_node.kind != NodeKind::Step {
                continue;
            }
            if child_node.bom.merge_into_parent {
                let merged = column_assembly(document, hierarchical, *child);
                lines.extend(merged.lines);
            } else if let Some(sub) =
                assembly.subassemblies.iter().find(|sub| sub.node == *child)
            {
                subassemblies.push(sub.clone());
            }
        }
    }
    sort_lines(&mut lines);
    BomAssembly {
        node: assembly.node,
        name: assembly.name.clone(),
        quantity: assembly.quantity,
        merge_into_parent: false,
        lines,
        subassemblies,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::parts::assembly::{AssemblyDocument, AssemblyNode, ExplodeSpec};
    use crate::solid;
    use hyperlattice::{Real, Vector3};

    fn sample() -> AssemblyDocument {
        let mut document = AssemblyDocument::new("Handle", "main");
        let mut handle = AssemblyNode::step("handle");
        handle.bom.merge_into_parent = true;
        handle.count = 2;
        let handle_id = document.insert(document.root(), handle).expect("handle");
        let mut printed =
            AssemblyNode::printed("handle.stl", "Printed handle", solid::cube(Real::from(4)));
        printed.count = 1;
        document.insert(handle_id, printed).expect("printed");
        let mut insert = AssemblyNode::vitamin(
            "insert(M3)",
            "insert(M3): Heatfit insert M3",
            solid::cube(Real::from(1)),
        );
        insert.count = 2;
        document.insert(handle_id, insert).expect("insert");
        let mut screw = AssemblyNode::vitamin(
            "screw(M3)",
            "screw(M3): Screw M3 pan x 16mm",
            solid::cube(Real::from(1)),
        );
        screw.count = 4;
        screw.explode = ExplodeSpec::translation(Vector3::new([
            Real::zero(),
            Real::zero(),
            Real::from(8),
        ]));
        document.insert(document.root(), screw).expect("screw");
        let mut hidden =
            AssemblyNode::vitamin("nut(M3)", "nut(M3): Nut M3", solid::cube(Real::from(1)));
        hidden.bom.hidden = true;
        document.insert(document.root(), hidden).expect("nut");
        document
    }

    #[test]
    fn quantities_merge_and_sort() {
        let report = bill_of_materials(&sample());
        assert_eq!(report.columns.len(), 1);
        assert_eq!(report.columns[0].name, "main");
        let descriptions: Vec<_> = report.columns[0]
            .lines
            .iter()
            .map(|line| (line.description.as_str(), line.quantity))
            .collect();
        assert_eq!(
            descriptions,
            vec![
                ("insert(M3): Heatfit insert M3", 4),
                ("screw(M3): Screw M3 pan x 16mm", 4),
                ("Printed handle", 2),
            ]
        );
        assert!(report.flat.iter().all(|line| line.key != "nut(M3)"));
        assert!(
            report
                .hierarchical
                .iter()
                .any(|assembly| assembly.name == "handle" && assembly.merge_into_parent)
        );
    }
}
