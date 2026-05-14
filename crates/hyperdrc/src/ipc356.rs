//! IPC-D-356 electrical-test netlist parsing.
//!
//! IPC-D-356B describes a bare-substrate electrical test data format rather
//! than an artwork format. hyperdrc therefore treats records as net/test access
//! evidence and records malformed recognized test records as parser diagnostics
//! instead of guessing geometry. See IPC-D-356B, *Bare Substrate Electrical Test
//! Data Format* (IPC, 2002).

use std::path::Path;

use anyhow::{Context, Result};

#[derive(Clone, Debug)]
/// Public data model for `Ipc356Point`.
pub struct Ipc356Point {
    /// Field `net`.
    pub net: String,
    /// Field `reference`.
    pub reference: Option<String>,
    /// Field `pin`.
    pub pin: Option<String>,
    /// Field `location`.
    pub location: [f64; 2],
    /// Field `diameter`.
    pub diameter: Option<f64>,
    /// Field `access_side`.
    pub access_side: Option<Ipc356AccessSide>,
    /// Field `feature_type`.
    pub feature_type: Option<Ipc356FeatureType>,
    /// Field `soldermask`.
    pub soldermask: Option<Ipc356Soldermask>,
}

/// Probe-side hints from common IPC-D-356 sidecar exports.
///
/// IPC-D-356B standardizes electrical-test records, but many EDA/export flows
/// add pragmatic side tokens beside the record. hyperdrc keeps those tokens as
/// readiness evidence instead of treating them as authoritative geometry.
#[derive(Copy, Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
/// Public enumeration for `Ipc356AccessSide`.
pub enum Ipc356AccessSide {
    /// Variant `Top`.
    Top,
    /// Variant `Bottom`.
    Bottom,
    /// Variant `Both`.
    Both,
}

/// Coarse test-feature class recovered from common sidecar annotations.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
/// Public enumeration for `Ipc356FeatureType`.
pub enum Ipc356FeatureType {
    /// Variant `ThroughHole`.
    ThroughHole,
    /// Variant `Smd`.
    Smd,
    /// Variant `Via`.
    Via,
    /// Variant `Tooling`.
    Tooling,
    /// Variant `Connector`.
    Connector,
    /// Variant `Other`.
    Other,
}

/// Soldermask access state recovered from common testpoint annotations.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
/// Public enumeration for `Ipc356Soldermask`.
pub enum Ipc356Soldermask {
    /// Variant `Open`.
    Open,
    /// Variant `Covered`.
    Covered,
    /// Variant `Unknown`.
    Unknown,
}

#[derive(Clone, Debug)]
/// Public data model for `Ipc356Issue`.
pub struct Ipc356Issue {
    /// Field `line`.
    pub line: usize,
    /// Field `kind`.
    pub kind: Ipc356IssueKind,
    /// Field `detail`.
    pub detail: String,
}

#[derive(Clone, Debug)]
/// Public enumeration for `Ipc356IssueKind`.
pub enum Ipc356IssueKind {
    /// Variant `MalformedTestRecord`.
    MalformedTestRecord,
}

#[derive(Clone, Debug)]
/// Public data model for `Ipc356Report`.
pub struct Ipc356Report {
    /// Field `source`.
    pub source: String,
    /// Field `points`.
    pub points: Vec<Ipc356Point>,
    /// Field `issues`.
    pub issues: Vec<Ipc356Issue>,
}

impl Ipc356Issue {
    /// Run or compute `message`.
    pub fn message(&self) -> String {
        match self.kind {
            Ipc356IssueKind::MalformedTestRecord => format!(
                "line {}: recognized IPC-D-356 test record could not be parsed",
                self.line
            ),
        }
    }
}

/// Run or compute `load_ipc356_report`.
pub fn load_ipc356_report(path: &Path) -> Result<Ipc356Report> {
    let text = std::fs::read_to_string(path)
        .with_context(|| format!("failed to read {}", path.display()))?;
    Ok(parse_ipc356_report(&text, path))
}

/// Run or compute `load_ipc356`.
pub fn load_ipc356(path: &Path) -> Result<Vec<Ipc356Point>> {
    Ok(load_ipc356_report(path)?.points)
}

/// Run or compute `parse_ipc356`.
pub fn parse_ipc356(input: &str) -> Vec<Ipc356Point> {
    parse_ipc356_report(input, Path::new("<inline-ipc356>")).points
}

/// Run or compute `parse_ipc356_report`.
pub fn parse_ipc356_report(input: &str, source: &Path) -> Ipc356Report {
    let mut points = Vec::new();
    let mut issues = Vec::new();
    for (index, raw_line) in input.lines().enumerate() {
        let line = raw_line.trim();
        if line.is_empty()
            || line.starts_with('C')
            || line.starts_with('P')
            || line.starts_with('9')
        {
            continue;
        }
        if !is_test_record(line) {
            continue;
        }
        match parse_record(line) {
            Some(point) => points.push(point),
            None => issues.push(Ipc356Issue {
                line: index + 1,
                kind: Ipc356IssueKind::MalformedTestRecord,
                detail: line.to_string(),
            }),
        }
    }

    Ipc356Report {
        source: source.display().to_string(),
        points,
        issues,
    }
}

fn parse_record(raw_line: &str) -> Option<Ipc356Point> {
    let line = raw_line.trim();
    if line.contains(' ') {
        parse_loose_record(line).or_else(|| parse_fixed_record(line))
    } else {
        parse_fixed_record(line)
    }
}

fn is_test_record(line: &str) -> bool {
    line.starts_with("327") || line.starts_with("317") || line.starts_with("367")
}

fn parse_fixed_record(line: &str) -> Option<Ipc356Point> {
    let net = slice(line, 3, 17)?
        .trim()
        .trim_start_matches('/')
        .to_string();
    let reference = nonempty(slice(line, 20, 26)?.trim());
    let pin = nonempty(slice(line, 27, 31)?.trim());
    let x_marker = line.find('X')?;
    let y_marker = line.find('Y')?;
    let x = parse_ipc_number(take_number(&line[x_marker + 1..])?)?;
    let y = parse_ipc_number(take_number(&line[y_marker + 1..])?)?;
    let diameter = line
        .find("D")
        .and_then(|index| take_number(&line[index + 1..]))
        .and_then(parse_ipc_number);
    let metadata = parse_metadata(
        line.split(|ch: char| ch.is_whitespace())
            .filter(|part| !part.is_empty()),
    );
    Some(Ipc356Point {
        net,
        reference,
        pin,
        location: [x, y],
        diameter,
        access_side: metadata.access_side,
        feature_type: metadata.feature_type,
        soldermask: metadata.soldermask,
    })
}

fn parse_loose_record(line: &str) -> Option<Ipc356Point> {
    let parts = line.split_whitespace().collect::<Vec<_>>();
    let net = parts.get(1)?.trim_start_matches('/').to_string();
    let coordinate_text = parts
        .iter()
        .find(|part| part.starts_with('X') && part.contains('Y'))?;
    let (x, y) = parse_xy_markers(coordinate_text)?;
    let diameter = parts
        .iter()
        .find(|part| part.starts_with('D'))
        .and_then(|part| parse_ipc_number(&part[1..]))
        .or_else(|| {
            coordinate_text
                .find('D')
                .and_then(|index| take_number(&coordinate_text[index + 1..]))
                .and_then(parse_ipc_number)
        });
    let metadata = parse_metadata(parts.iter().copied());
    Some(Ipc356Point {
        net,
        reference: parts.get(2).map(|value| (*value).to_string()),
        pin: parts.get(3).map(|value| (*value).to_string()),
        location: [x, y],
        diameter,
        access_side: metadata.access_side,
        feature_type: metadata.feature_type,
        soldermask: metadata.soldermask,
    })
}

#[derive(Copy, Clone, Debug, Default)]
struct Ipc356Metadata {
    access_side: Option<Ipc356AccessSide>,
    feature_type: Option<Ipc356FeatureType>,
    soldermask: Option<Ipc356Soldermask>,
}

fn parse_metadata<'a>(parts: impl IntoIterator<Item = &'a str>) -> Ipc356Metadata {
    let mut metadata = Ipc356Metadata::default();
    for part in parts {
        let normalized = part
            .trim_matches(|ch: char| ch == ',' || ch == ';')
            .to_ascii_uppercase();
        let value = normalized
            .strip_prefix("ACCESS=")
            .or_else(|| normalized.strip_prefix("SIDE="))
            .or_else(|| normalized.strip_prefix("A="));
        if let Some(value) = value {
            metadata.access_side = parse_access_side(value).or(metadata.access_side);
            continue;
        }

        let value = normalized
            .strip_prefix("FEATURE=")
            .or_else(|| normalized.strip_prefix("TYPE="))
            .or_else(|| normalized.strip_prefix("F="));
        if let Some(value) = value {
            metadata.feature_type = parse_feature_type(value).or(metadata.feature_type);
            continue;
        }

        let value = normalized
            .strip_prefix("MASK=")
            .or_else(|| normalized.strip_prefix("SOLDERMASK="))
            .or_else(|| normalized.strip_prefix("SM="));
        if let Some(value) = value {
            metadata.soldermask = parse_soldermask(value).or(metadata.soldermask);
            continue;
        }

        metadata.access_side = parse_access_side(&normalized).or(metadata.access_side);
        metadata.feature_type = parse_feature_type(&normalized).or(metadata.feature_type);
        metadata.soldermask = parse_soldermask(&normalized).or(metadata.soldermask);
    }
    metadata
}

fn parse_access_side(value: &str) -> Option<Ipc356AccessSide> {
    match value {
        "T" | "TOP" | "FRONT" | "COMPONENT" | "PRIMARY" => Some(Ipc356AccessSide::Top),
        "B" | "BOT" | "BOTTOM" | "BACK" | "SOLDER" | "SECONDARY" => Some(Ipc356AccessSide::Bottom),
        "BOTH" | "EITHER" | "ANY" => Some(Ipc356AccessSide::Both),
        _ => None,
    }
}

fn parse_feature_type(value: &str) -> Option<Ipc356FeatureType> {
    match value {
        "TH" | "THT" | "PTH" | "THROUGH" | "THROUGHHOLE" | "THROUGH-HOLE" => {
            Some(Ipc356FeatureType::ThroughHole)
        }
        "SMD" | "SMT" | "PAD" | "SURFACE" => Some(Ipc356FeatureType::Smd),
        "VIA" | "V" => Some(Ipc356FeatureType::Via),
        "TOOL" | "TOOLING" | "FID" | "FIDUCIAL" => Some(Ipc356FeatureType::Tooling),
        "CONN" | "CONNECTOR" | "EDGE" => Some(Ipc356FeatureType::Connector),
        "OTHER" => Some(Ipc356FeatureType::Other),
        _ => None,
    }
}

fn parse_soldermask(value: &str) -> Option<Ipc356Soldermask> {
    match value {
        "OPEN" | "UNMASKED" | "EXPOSED" | "CLEAR" | "WINDOW" => Some(Ipc356Soldermask::Open),
        "COVERED" | "MASKED" | "TENTED" | "CLOSED" => Some(Ipc356Soldermask::Covered),
        "UNKNOWN" | "NA" | "N/A" => Some(Ipc356Soldermask::Unknown),
        _ => None,
    }
}

fn parse_xy_markers(value: &str) -> Option<(f64, f64)> {
    let x_marker = value.find('X')?;
    let y_marker = value.find('Y')?;
    let x_end = if x_marker < y_marker {
        y_marker
    } else {
        value.len()
    };
    let y_end = value[y_marker + 1..]
        .find(|ch: char| ch == 'X' || ch == 'D' || ch.is_whitespace())
        .map(|offset| y_marker + 1 + offset)
        .unwrap_or(value.len());
    let x = parse_ipc_number(&value[x_marker + 1..x_end])?;
    let y = parse_ipc_number(&value[y_marker + 1..y_end])?;
    Some((x, y))
}

fn slice(value: &str, start: usize, end: usize) -> Option<&str> {
    value.get(start..end)
}

fn nonempty(value: &str) -> Option<String> {
    (!value.is_empty()).then(|| value.to_string())
}

fn take_number(value: &str) -> Option<&str> {
    let end = value
        .find(|ch: char| !(ch.is_ascii_digit() || ch == '+' || ch == '-' || ch == '.'))
        .unwrap_or(value.len());
    (end > 0).then_some(&value[..end])
}

fn parse_ipc_number(value: &str) -> Option<f64> {
    let value = value.trim();
    if value.is_empty() {
        return None;
    }
    if value.contains('.') {
        return value.parse().ok();
    }

    let sign = value.starts_with('-');
    let digits = value.trim_start_matches(['+', '-']);
    if !digits.chars().all(|ch| ch.is_ascii_digit()) {
        return None;
    }
    if digits.len() <= 4 {
        let parsed = digits.parse::<f64>().ok()? / 1000.0;
        return Some(if sign { -parsed } else { parsed });
    }

    let split = digits.len().saturating_sub(4);
    let mut normalized = String::new();
    if sign {
        normalized.push('-');
    }
    normalized.push_str(&digits[..split]);
    normalized.push('.');
    normalized.push_str(&digits[split..]);
    normalized.parse().ok()
}

#[cfg(test)]
mod tests {
    use proptest::prelude::*;

    use super::{
        Ipc356AccessSide, Ipc356FeatureType, Ipc356IssueKind, Ipc356Soldermask, parse_ipc356,
        parse_ipc356_report,
    };

    #[test]
    fn parses_loose_ipc356_test_record() {
        let points = parse_ipc356("327 /GND U1 1 X010000Y020000D000600\n");

        assert_eq!(points.len(), 1);
        assert_eq!(points[0].net, "GND");
        assert_eq!(points[0].location, [1.0, 2.0]);
        assert_eq!(points[0].diameter, Some(0.06));
    }

    #[test]
    fn parses_optional_access_feature_and_soldermask_metadata() {
        let points = parse_ipc356(
            "327 /USB_D+ J1 2 X010000Y020000D000600 ACCESS=TOP FEATURE=SMD MASK=OPEN\n",
        );

        assert_eq!(points.len(), 1);
        assert_eq!(points[0].access_side, Some(Ipc356AccessSide::Top));
        assert_eq!(points[0].feature_type, Some(Ipc356FeatureType::Smd));
        assert_eq!(points[0].soldermask, Some(Ipc356Soldermask::Open));
    }

    #[test]
    fn ignores_comments_and_unknown_records() {
        let points = parse_ipc356(
            r#"
            C comment
            P parameter
            999 /GND X010000Y020000
            327 missing-coordinates
            "#,
        );

        assert!(points.is_empty());
    }

    #[test]
    fn reports_malformed_recognized_records() {
        let report = parse_ipc356_report(
            r#"
            C comment
            999 /GND X010000Y020000
            327 missing-coordinates
            "#,
            std::path::Path::new("board.ipc"),
        );

        assert!(report.points.is_empty());
        assert_eq!(report.issues.len(), 1);
        assert!(matches!(
            report.issues[0].kind,
            Ipc356IssueKind::MalformedTestRecord
        ));
        assert!(report.issues[0].message().contains("line 4"));
    }

    #[test]
    fn parses_fixed_record_without_panicking() {
        let points = parse_ipc356("327/GND          X010000Y020000D000600\n");

        assert_eq!(points.len(), 1);
        assert!(!points[0].net.is_empty());
        assert!(points[0].location[0].is_finite());
        assert!(points[0].location[1].is_finite());
    }

    proptest! {
        #[test]
        fn arbitrary_ipc356_text_never_panics(input in "\\PC*") {
            let _ = parse_ipc356(&input);
        }

        #[test]
        fn generated_loose_records_have_finite_coordinates(
            net in "[A-Z0-9_.+-]{1,24}",
            x in 0u32..999_999,
            y in 0u32..999_999,
            diameter in 0u32..999_999,
        ) {
            let text = format!("327 /{net} U1 1 X{x:06}Y{y:06}D{diameter:06}\n");
            let points = parse_ipc356(&text);
            prop_assert_eq!(points.len(), 1);
            prop_assert_eq!(points[0].net.as_str(), net.as_str());
            prop_assert!(points[0].location[0].is_finite());
            prop_assert!(points[0].location[1].is_finite());
        }
    }
}
