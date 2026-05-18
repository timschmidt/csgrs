//! Deterministic exact text and binary snapshots for packing fixtures.
//!
//! Fixture formats are part of the exact-computing boundary: if exact values
//! are lowered to primitive floats while writing tests, later replay may certify
//! a different combinatorial layout. Following Yap, "Towards Exact Geometric
//! Computation," *Computational Geometry* 7(1-2), 1997
//! (<https://doi.org/10.1016/0925-7721(95)00040-2>), these helpers serialize
//! scalars as text fields that preserve exact rational values or full
//! `hyperreal` structural JSON, never as display-rounded floats. The binary
//! helpers use length-prefixed UTF-8 fields for the same exact scalar strings;
//! binary framing improves fixture robustness without changing scalar meaning.

use hyperreal::Real;

use crate::{
    AxisBox3, Bin3, Item3, Placement3, Rect2, SheetBin2, SheetItem2, SheetPlacement2, StockBin1,
    StockItem1, StockPlacement1,
};

/// Deterministic text fixture for one 1D stock-packing instance.
pub fn snapshot_stock_1d_text(
    bin: &StockBin1,
    items: &[StockItem1],
    placements: &[StockPlacement1],
) -> String {
    let mut lines = vec![
        "hyperpack-snapshot-v1".to_string(),
        "kind\tstock-1d".to_string(),
        format!("bin\t{}", scalar_text(&bin.length)),
    ];
    for item in items {
        lines.push(format!(
            "item\t{}\t{}",
            escape_field(item.id.as_str()),
            scalar_text(&item.length)
        ));
    }
    for placement in placements {
        lines.push(format!(
            "placement\t{}\t{}",
            escape_field(placement.item.as_str()),
            scalar_text(&placement.start)
        ));
    }
    lines.join("\n")
}

/// Deterministic text fixture for one 2D sheet-packing instance.
pub fn snapshot_sheet_2d_text(
    bin: &SheetBin2,
    items: &[SheetItem2],
    placements: &[SheetPlacement2],
) -> String {
    let mut lines = vec![
        "hyperpack-snapshot-v1".to_string(),
        "kind\tsheet-2d".to_string(),
        format!("bin\t{}", rect_text(&bin.size)),
    ];
    for item in items {
        lines.push(format!(
            "item\t{}\t{}",
            escape_field(item.id.as_str()),
            rect_text(&item.size)
        ));
    }
    for placement in placements {
        lines.push(format!(
            "placement\t{}\t{}\t{}",
            escape_field(placement.item.as_str()),
            scalar_text(&placement.x),
            scalar_text(&placement.y)
        ));
    }
    lines.join("\n")
}

/// Deterministic text fixture for one 3D cuboid-packing instance.
pub fn snapshot_packing_3d_text(bin: &Bin3, items: &[Item3], placements: &[Placement3]) -> String {
    let mut lines = vec![
        "hyperpack-snapshot-v1".to_string(),
        "kind\tpacking-3d".to_string(),
        format!("bin\t{}", box_text(&bin.size)),
    ];
    for item in items {
        lines.push(format!(
            "item\t{}\t{}",
            escape_field(item.id.as_str()),
            box_text(&item.size)
        ));
    }
    for placement in placements {
        lines.push(format!(
            "placement\t{}\t{}\t{}\t{}",
            escape_field(placement.item.as_str()),
            scalar_text(&placement.x),
            scalar_text(&placement.y),
            scalar_text(&placement.z)
        ));
    }
    lines.join("\n")
}

/// Deterministic binary fixture for one 1D stock-packing instance.
///
/// Fields are length-prefixed UTF-8 strings. Exact scalar payloads are the same
/// rational or structural strings used by [`snapshot_stock_1d_text`], following
/// Yap's requirement that fixture serialization not introduce primitive-float
/// decisions into later geometric replay.
pub fn snapshot_stock_1d_binary(
    bin: &StockBin1,
    items: &[StockItem1],
    placements: &[StockPlacement1],
) -> Vec<u8> {
    let mut fields = vec![
        "hyperpack-snapshot-bin-v1".to_string(),
        "stock-1d".to_string(),
    ];
    fields.push(scalar_text(&bin.length));
    fields.push(items.len().to_string());
    for item in items {
        fields.push(item.id.as_str().to_string());
        fields.push(scalar_text(&item.length));
    }
    fields.push(placements.len().to_string());
    for placement in placements {
        fields.push(placement.item.as_str().to_string());
        fields.push(scalar_text(&placement.start));
    }
    encode_binary_fields(&fields)
}

/// Deterministic binary fixture for one 2D sheet-packing instance.
///
/// The binary frame preserves exact scalar strings and raw UTF-8 ids through
/// length prefixes, so ids containing tabs or newlines do not require escaping.
pub fn snapshot_sheet_2d_binary(
    bin: &SheetBin2,
    items: &[SheetItem2],
    placements: &[SheetPlacement2],
) -> Vec<u8> {
    let mut fields = vec![
        "hyperpack-snapshot-bin-v1".to_string(),
        "sheet-2d".to_string(),
    ];
    fields.extend([scalar_text(&bin.size.x), scalar_text(&bin.size.y)]);
    fields.push(items.len().to_string());
    for item in items {
        fields.push(item.id.as_str().to_string());
        fields.extend([scalar_text(&item.size.x), scalar_text(&item.size.y)]);
    }
    fields.push(placements.len().to_string());
    for placement in placements {
        fields.push(placement.item.as_str().to_string());
        fields.extend([scalar_text(&placement.x), scalar_text(&placement.y)]);
    }
    encode_binary_fields(&fields)
}

/// Deterministic binary fixture for one 3D cuboid-packing instance.
///
/// This is a binary frame, not a lossy binary scalar format: all coordinates
/// and dimensions remain exact scalar strings that must be parsed by the scalar
/// layer before replay.
pub fn snapshot_packing_3d_binary(
    bin: &Bin3,
    items: &[Item3],
    placements: &[Placement3],
) -> Vec<u8> {
    let mut fields = vec![
        "hyperpack-snapshot-bin-v1".to_string(),
        "packing-3d".to_string(),
    ];
    fields.extend([
        scalar_text(&bin.size.x),
        scalar_text(&bin.size.y),
        scalar_text(&bin.size.z),
    ]);
    fields.push(items.len().to_string());
    for item in items {
        fields.push(item.id.as_str().to_string());
        fields.extend([
            scalar_text(&item.size.x),
            scalar_text(&item.size.y),
            scalar_text(&item.size.z),
        ]);
    }
    fields.push(placements.len().to_string());
    for placement in placements {
        fields.push(placement.item.as_str().to_string());
        fields.extend([
            scalar_text(&placement.x),
            scalar_text(&placement.y),
            scalar_text(&placement.z),
        ]);
    }
    encode_binary_fields(&fields)
}

fn rect_text(size: &Rect2) -> String {
    format!("{}\t{}", scalar_text(&size.x), scalar_text(&size.y))
}

fn box_text(size: &AxisBox3) -> String {
    format!(
        "{}\t{}\t{}",
        scalar_text(&size.x),
        scalar_text(&size.y),
        scalar_text(&size.z)
    )
}

fn scalar_text(value: &Real) -> String {
    value
        .exact_rational()
        .map(|rational| rational.to_string())
        .unwrap_or_else(|| escape_field(&value.to_json()))
}

fn escape_field(value: &str) -> String {
    let mut escaped = String::new();
    for character in value.chars() {
        match character {
            '\\' => escaped.push_str("\\\\"),
            '\n' => escaped.push_str("\\n"),
            '\r' => escaped.push_str("\\r"),
            '\t' => escaped.push_str("\\t"),
            _ => escaped.push(character),
        }
    }
    escaped
}

fn encode_binary_fields(fields: &[String]) -> Vec<u8> {
    let mut bytes = Vec::new();
    bytes.extend_from_slice(b"HPB1");
    bytes.extend_from_slice(&(fields.len() as u32).to_le_bytes());
    for field in fields {
        let field_bytes = field.as_bytes();
        bytes.extend_from_slice(&(field_bytes.len() as u32).to_le_bytes());
        bytes.extend_from_slice(field_bytes);
    }
    bytes
}
