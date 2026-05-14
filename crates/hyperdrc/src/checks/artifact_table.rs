//! Lightweight table parsing for pre-production sidecar artifacts.
//!
//! The production artifact checks need enough structure to validate common BOM,
//! centroid, and netlist exports without becoming a spreadsheet engine. This
//! parser keeps to RFC-4180-style quoted fields for comma/semicolon/tab data and
//! a whitespace fallback for simple text exports.

/// Parsed text table with normalized headers and raw cell values.
pub(super) struct Table {
    pub(super) headers: Vec<String>,
    pub(super) rows: Vec<Vec<String>>,
}

#[derive(Copy, Clone)]
enum TableDelimiter {
    Comma,
    Tab,
    Semicolon,
    Whitespace,
}

/// Parse a delimited text artifact into headers and data rows.
pub(super) fn parse_table(text: &str) -> Option<Table> {
    let lines = text
        .lines()
        .map(str::trim)
        .filter(|line| !line.is_empty() && !line.starts_with('#'))
        .collect::<Vec<_>>();
    let header = lines.first()?;
    let delimiter = detect_table_delimiter(header);
    let headers = split_row(header, delimiter)
        .into_iter()
        .map(|header| normalize_header(&header))
        .collect::<Vec<_>>();
    let rows = lines
        .iter()
        .skip(1)
        .map(|line| split_row(line, delimiter))
        .filter(|row| row.iter().any(|cell| !cell.trim().is_empty()))
        .collect::<Vec<_>>();

    Some(Table { headers, rows }).filter(|table| !table.headers.is_empty())
}

/// Find the first header whose normalized name equals or contains a candidate.
pub(super) fn find_column(headers: &[String], candidates: &[&str]) -> Option<usize> {
    let candidates = candidates
        .iter()
        .map(|candidate| normalize_header(candidate))
        .collect::<Vec<_>>();
    headers.iter().position(|header| {
        candidates
            .iter()
            .any(|candidate| header == candidate || header.contains(candidate))
    })
}

/// Return a cell or an empty string when the row is short.
pub(super) fn cell(row: &[String], column: usize) -> &str {
    row.get(column).map(String::as_str).unwrap_or("")
}

fn detect_table_delimiter(header: &str) -> TableDelimiter {
    if header.contains('\t') {
        TableDelimiter::Tab
    } else if header.contains(',') {
        TableDelimiter::Comma
    } else if header.contains(';') {
        TableDelimiter::Semicolon
    } else {
        TableDelimiter::Whitespace
    }
}

fn split_row(line: &str, delimiter: TableDelimiter) -> Vec<String> {
    let mut cells = Vec::new();
    let mut current = String::new();
    let mut in_quotes = false;
    let mut chars = line.chars().peekable();

    while let Some(ch) = chars.next() {
        if ch == '"' {
            if in_quotes && chars.peek() == Some(&'"') {
                current.push('"');
                chars.next();
            } else {
                in_quotes = !in_quotes;
            }
        } else if delimiter.matches(ch) && !in_quotes {
            cells.push(current.trim().to_string());
            current.clear();
            if matches!(delimiter, TableDelimiter::Whitespace) {
                while chars.peek().is_some_and(|next| next.is_ascii_whitespace()) {
                    chars.next();
                }
            }
        } else {
            current.push(ch);
        }
    }
    cells.push(current.trim().to_string());

    cells
}

impl TableDelimiter {
    fn matches(self, ch: char) -> bool {
        match self {
            TableDelimiter::Comma => ch == ',',
            TableDelimiter::Tab => ch == '\t',
            TableDelimiter::Semicolon => ch == ';',
            TableDelimiter::Whitespace => ch.is_ascii_whitespace(),
        }
    }
}

fn normalize_header(header: &str) -> String {
    header
        .trim_matches('"')
        .chars()
        .filter(|ch| ch.is_ascii_alphanumeric())
        .collect::<String>()
        .to_ascii_lowercase()
}
