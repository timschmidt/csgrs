use std::path::Path;

use anyhow::{Context, Result};

#[derive(Clone, Debug)]
pub struct Ipc356Point {
    pub net: String,
    pub reference: Option<String>,
    pub pin: Option<String>,
    pub location: [f64; 2],
    pub diameter: Option<f64>,
}

pub fn load_ipc356(path: &Path) -> Result<Vec<Ipc356Point>> {
    let text = std::fs::read_to_string(path)
        .with_context(|| format!("failed to read {}", path.display()))?;
    Ok(parse_ipc356(&text))
}

fn parse_ipc356(input: &str) -> Vec<Ipc356Point> {
    input.lines().filter_map(parse_record).collect()
}

fn parse_record(raw_line: &str) -> Option<Ipc356Point> {
    let line = raw_line.trim();
    if line.is_empty() || line.starts_with('C') || line.starts_with('P') || line.starts_with('9') {
        return None;
    }

    if !line.starts_with("327") && !line.starts_with("317") && !line.starts_with("367") {
        return None;
    }

    if line.contains(' ') {
        parse_loose_record(line).or_else(|| parse_fixed_record(line))
    } else {
        parse_fixed_record(line)
    }
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

    Some(Ipc356Point {
        net,
        reference,
        pin,
        location: [x, y],
        diameter,
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

    Some(Ipc356Point {
        net,
        reference: parts.get(2).map(|value| (*value).to_string()),
        pin: parts.get(3).map(|value| (*value).to_string()),
        location: [x, y],
        diameter,
    })
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

    use super::parse_ipc356;

    #[test]
    fn parses_loose_ipc356_test_record() {
        let points = parse_ipc356("327 /GND U1 1 X010000Y020000D000600\n");

        assert_eq!(points.len(), 1);
        assert_eq!(points[0].net, "GND");
        assert_eq!(points[0].location, [1.0, 2.0]);
        assert_eq!(points[0].diameter, Some(0.06));
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
