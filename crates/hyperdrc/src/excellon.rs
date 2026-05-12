use std::collections::HashMap;
use std::path::Path;

use anyhow::{Context, Result};

use crate::kicad::DrillFeature;

pub fn load_excellon(path: &Path) -> Result<Vec<DrillFeature>> {
    let text = std::fs::read_to_string(path)
        .with_context(|| format!("failed to read {}", path.display()))?;
    Ok(parse_excellon(&text))
}

fn parse_excellon(input: &str) -> Vec<DrillFeature> {
    let mut tools: HashMap<String, f64> = HashMap::new();
    let mut current_tool: Option<String> = None;
    let mut drills = Vec::new();
    let mut units_scale = 1.0;

    for raw_line in input.lines() {
        let line = raw_line.trim();
        if line.is_empty() || line.starts_with(';') {
            continue;
        }

        if line.starts_with("METRIC") {
            units_scale = 1.0;
            continue;
        }
        if line.starts_with("INCH") {
            units_scale = 25.4;
            continue;
        }

        if let Some((tool, diameter)) = parse_tool_definition(line, units_scale) {
            tools.insert(tool, diameter);
            continue;
        }

        if line.starts_with('T') && !line.contains('C') {
            current_tool = Some(line.to_string());
            continue;
        }

        if let Some(location) = parse_coordinate(line, units_scale)
            && let Some(tool) = &current_tool
            && let Some(diameter) = tools.get(tool)
        {
            drills.push(DrillFeature {
                location,
                diameter: *diameter,
                net: None,
                plated: false,
            });
        }
    }

    drills
}

fn parse_tool_definition(line: &str, units_scale: f64) -> Option<(String, f64)> {
    let c_index = line.find('C')?;
    let tool = line[..c_index].to_string();
    let diameter = line[c_index + 1..].parse::<f64>().ok()? * units_scale;
    Some((tool, diameter))
}

fn parse_coordinate(line: &str, units_scale: f64) -> Option<[f64; 2]> {
    let x_index = line.find('X')?;
    let y_index = line.find('Y')?;
    let x_end = if x_index < y_index {
        y_index
    } else {
        line.len()
    };
    let y_end = if y_index < x_index {
        x_index
    } else {
        line.len()
    };
    let x = parse_number(&line[x_index + 1..x_end])? * units_scale;
    let y = parse_number(&line[y_index + 1..y_end])? * units_scale;
    Some([x, y])
}

fn parse_number(value: &str) -> Option<f64> {
    if !value
        .chars()
        .all(|ch| ch.is_ascii_digit() || ch == '+' || ch == '-' || ch == '.')
    {
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
    if digits.len() <= 3 {
        return value.parse().ok();
    }

    let split = digits.len().saturating_sub(3);
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

    use super::parse_excellon;

    #[test]
    fn parses_metric_tool_hits() {
        let drills = parse_excellon(
            r#"
            M48
            METRIC,TZ
            T01C0.600
            %
            T01
            X010000Y020000
            M30
            "#,
        );

        assert_eq!(drills.len(), 1);
        assert_eq!(drills[0].diameter, 0.6);
        assert_eq!(drills[0].location, [10.0, 20.0]);
    }

    #[test]
    fn ignores_hits_before_tool_selection_or_definition() {
        let drills = parse_excellon(
            r#"
            METRIC,TZ
            X010000Y020000
            T01
            X010000Y020000
            "#,
        );

        assert!(drills.is_empty());
    }

    #[test]
    fn parses_inches_as_millimeters() {
        let drills = parse_excellon(
            r#"
            INCH,TZ
            T01C0.010
            T01
            X001000Y002000
            "#,
        );

        assert_eq!(drills.len(), 1);
        assert!((drills[0].diameter - 0.254).abs() < 1.0e-9);
    }

    proptest! {
        #[test]
        fn arbitrary_excellon_text_never_panics(input in "\\PC*") {
            let _ = parse_excellon(&input);
        }

        #[test]
        fn generated_metric_hits_are_finite(x in 0u32..999_999, y in 0u32..999_999, diameter in 1u32..5000) {
            let text = format!(
                "METRIC,TZ\nT01C{}.{:03}\nT01\nX{x:06}Y{y:06}\n",
                diameter / 1000,
                diameter % 1000
            );
            let drills = parse_excellon(&text);
            prop_assert_eq!(drills.len(), 1);
            prop_assert!(drills[0].location[0].is_finite());
            prop_assert!(drills[0].location[1].is_finite());
            prop_assert!(drills[0].diameter.is_finite());
            prop_assert!(drills[0].diameter > 0.0);
        }
    }
}
