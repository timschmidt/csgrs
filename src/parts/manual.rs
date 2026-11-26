//! Markdown assembly manual.
//!
//! The section order follows the NopSCADlib build document: description, a
//! global parts table, then one section per step with its parts, the exploded
//! drawing, the instructions, and the assembled drawing. Drawing paths are
//! supplied by the caller. This function does not render pixels.
//! Text and paths are inserted verbatim as Markdown, without escaping, file
//! validation or renderer-specific anchor resolution. BOM quantities retain the
//! counting limitations of [`bill_of_materials`].

use super::assembly::AssemblyDocument;
use super::bom::{BomCategory, BomLine, BomReport, bill_of_materials};
use super::metadata::NodeId;

/// Markdown manual and the steps that still need instruction text.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Manual {
    /// Generated Markdown, including placeholders for missing instructions.
    pub markdown: String,
    /// Messages for emitted, nonblank-named steps whose trimmed instructions are empty.
    /// Does not report drawing, geometry, count or Markdown validity.
    pub blockers: Vec<String>,
}

/// Builds Markdown in memory from the hierarchical BOM's step order.
///
/// Every step receives a table-of-contents entry, including blank names; only
/// nonblank-named steps receive sections. Merged steps still receive sections.
/// For each emitted section, calls `exploded_path` then `assembled_path` once,
/// even when instructions are missing. Paths are embedded without opening files.
/// Names, descriptions, instructions and paths are not escaped. Anchor collisions
/// are not resolved. No geometry, pose or instruction completeness is certified.
///
/// Requires the consistent acyclic hierarchy expected by [`bill_of_materials`].
/// Per-column duplicate quantities use ordinary `u64` addition (overflow can
/// panic when checks are enabled or wrap otherwise); cross-column totals saturate.
/// Allocation and callback failures have no typed diagnostic channel here.
pub fn assembly_manual(
    document: &AssemblyDocument,
    exploded_path: impl Fn(NodeId) -> String,
    assembled_path: impl Fn(NodeId) -> String,
) -> Manual {
    let report = bill_of_materials(document);
    let mut blockers = Vec::new();
    let mut markdown = String::new();
    markdown.push_str(&format!("# {}\n\n", document.title));
    for (index, section) in document.description.iter().enumerate() {
        if index > 0 {
            markdown.push_str("\n---\n\n");
        }
        markdown.push_str(section);
        markdown.push_str("\n\n");
    }
    markdown.push_str("## Table of contents\n\n");
    markdown.push_str("1. [Parts list](#parts-list)\n");
    for assembly in &report.hierarchical {
        markdown.push_str(&format!(
            "1. [{name}](#{anchor})\n",
            name = assembly.name,
            anchor = anchor(&assembly.name)
        ));
    }
    markdown.push_str("\n## Parts list\n\n");
    write_global_table(&mut markdown, &report);
    for assembly in &report.hierarchical {
        if assembly
            .name
            .chars()
            .all(|character| character.is_whitespace())
        {
            continue;
        }
        let node = document.node(assembly.node);
        let instructions = node.map(|node| node.instructions.trim()).unwrap_or("");
        if instructions.is_empty() {
            blockers.push(format!("{} is missing assembly instructions", assembly.name));
        }
        markdown.push_str(&format!("\n## {name}\n\n", name = assembly.name));
        write_category(
            &mut markdown,
            "Vitamins",
            &assembly.lines,
            BomCategory::Vitamin,
        );
        write_files(
            &mut markdown,
            "Printed parts",
            &assembly.lines,
            BomCategory::Printed,
        );
        write_files(
            &mut markdown,
            "Routed parts",
            &assembly.lines,
            BomCategory::Routed,
        );
        if !assembly.subassemblies.is_empty() {
            markdown.push_str("### Sub-assemblies\n\n");
            markdown.push_str("| Qty | Sub-assembly |\n| ---: | --- |\n");
            for sub in &assembly.subassemblies {
                markdown.push_str(&format!("| {} | {} |\n", sub.quantity, sub.name));
            }
            markdown.push('\n');
        }
        markdown.push_str("### Assembly instructions\n\n");
        markdown.push_str(&format!(
            "![{name} exploded]({path})\n\n",
            name = assembly.name,
            path = exploded_path(assembly.node)
        ));
        if instructions.is_empty() {
            markdown.push_str(&format!(
                "Instructions were not provided for {}.\n\n",
                assembly.name
            ));
        } else {
            markdown.push_str(instructions);
            markdown.push_str("\n\n");
        }
        markdown.push_str(&format!(
            "![{name} assembled]({path})\n",
            name = assembly.name,
            path = assembled_path(assembly.node)
        ));
    }
    Manual { markdown, blockers }
}

/// Appends one row per category/key first encountered in the retained BOM columns.
/// Uses the first line's description, sorted by category then that full description;
/// the report's flat lines are not used. Duplicate quantities within each column
/// use ordinary `u64` sums, while the total across columns saturates. Even an empty
/// column list emits the Description/Total header, with no data rows.
fn write_global_table(markdown: &mut String, report: &BomReport) {
    markdown.push_str("| Description |");
    for column in &report.columns {
        markdown.push_str(&format!(" {} |", column.name));
    }
    markdown.push_str(" Total |\n| --- |");
    for _ in &report.columns {
        markdown.push_str(" ---: |");
    }
    markdown.push_str(" ---: |\n");
    let mut keys = Vec::new();
    for column in &report.columns {
        for line in &column.lines {
            if !keys.iter().any(|existing: &BomLine| {
                existing.category == line.category && existing.key == line.key
            }) {
                keys.push(line.clone());
            }
        }
    }
    keys.sort_by(|left, right| {
        left.category
            .cmp(&right.category)
            .then_with(|| left.description.cmp(&right.description))
    });
    for key in &keys {
        markdown.push_str("| ");
        markdown.push_str(&display_description(key));
        markdown.push_str(" |");
        let mut total = 0u64;
        for column in &report.columns {
            let quantity: u64 = column
                .lines
                .iter()
                .filter(|line| line.category == key.category && line.key == key.key)
                .map(|line| line.quantity)
                .sum();
            total = total.saturating_add(quantity);
            markdown.push_str(&format!(" {quantity} |"));
        }
        markdown.push_str(&format!(" {total} |\n"));
    }
    markdown.push('\n');
}

/// Removes a vitamin's first colon-delimited prefix and trims its suffix;
/// returns every other description verbatim, including a colon-free vitamin.
fn display_description(line: &BomLine) -> String {
    if line.category == BomCategory::Vitamin
        && let Some((_, description)) = line.description.split_once(':')
    {
        return description.trim().to_string();
    }
    line.description.clone()
}

/// Appends a quantity/description table in input order for one category;
/// emits no heading when there are no matching rows and does not aggregate them.
fn write_category(
    markdown: &mut String,
    title: &str,
    lines: &[BomLine],
    category: BomCategory,
) {
    let rows: Vec<_> = lines
        .iter()
        .filter(|line| line.category == category)
        .collect();
    if rows.is_empty() {
        return;
    }
    markdown.push_str(&format!("### {title}\n\n"));
    markdown.push_str("| Qty | Description |\n| ---: | --- |\n");
    for line in rows {
        markdown.push_str(&format!(
            "| {} | {} |\n",
            line.quantity,
            display_description(line)
        ));
    }
    markdown.push('\n');
}

/// Appends quantity/key bullets in input order for one nonempty category.
/// Keys are inserted verbatim; no file link, existence check or export is created.
fn write_files(markdown: &mut String, title: &str, lines: &[BomLine], category: BomCategory) {
    let rows: Vec<_> = lines
        .iter()
        .filter(|line| line.category == category)
        .collect();
    if rows.is_empty() {
        return;
    }
    markdown.push_str(&format!("### {title}\n\n"));
    for line in rows {
        markdown.push_str(&format!("- {} x {}\n", line.quantity, line.key));
    }
    markdown.push('\n');
}

/// Lowercases ASCII alphanumerics and replaces each other character with one `-`.
/// Does not trim/collapse separators, preserve non-ASCII letters or uniquify names;
/// the resulting fragment need not match a particular Markdown renderer's anchor.
fn anchor(name: &str) -> String {
    name.chars()
        .map(|character| {
            if character.is_ascii_alphanumeric() {
                character.to_ascii_lowercase()
            } else {
                '-'
            }
        })
        .collect()
}

#[cfg(test)]
/// Source-level witnesses for instruction text, drawing paths and missing-step reports.
mod tests {
    use super::*;
    use crate::parts::assembly::{AssemblyDocument, AssemblyNode};
    use crate::solid;
    use hyperlattice::Real;

    #[test]
    /// Checks root instructions/paths and a merged child step's placeholder/blocker;
    /// permits either printed description or filename rather than fixing full output.
    fn manual_contains_instructions_and_reports_gaps() {
        let mut document = AssemblyDocument::new("Handle project", "main");
        document.description.push("A small handle.".into());
        document.node_mut(document.root()).expect("root").instructions =
            "Fit the handle.".into();
        let mut handle = AssemblyNode::step("handle");
        handle.bom.merge_into_parent = true;
        let handle_id = document.insert(document.root(), handle).expect("handle");
        document
            .insert(
                handle_id,
                AssemblyNode::printed(
                    "handle.stl",
                    "Printed handle",
                    solid::cube(Real::from(4)).expect("valid primitive fixture"),
                ),
            )
            .expect("printed");
        let manual = assembly_manual(
            &document,
            |id| format!("drawings/{id}-exploded.svg", id = id.0),
            |id| format!("drawings/{id}-assembled.svg", id = id.0),
        );
        assert!(manual.markdown.contains("Fit the handle."));
        assert!(manual.markdown.contains("drawings/0-exploded.svg"));
        assert!(manual.markdown.contains("drawings/0-assembled.svg"));
        assert!(
            manual.markdown.contains("Printed handle")
                || manual.markdown.contains("handle.stl")
        );
        assert!(
            manual
                .blockers
                .iter()
                .any(|blocker| blocker.contains("handle"))
        );
        assert!(
            manual
                .markdown
                .contains("Instructions were not provided for handle.")
        );
        assert!(!manual.blockers.iter().any(|blocker| blocker.contains("main")));
    }
}
