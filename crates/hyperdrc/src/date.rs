//! Small Gregorian date helpers for review and manifest checks.
//!
//! `hyperdrc` only needs day-level comparisons for governance metadata and
//! generated-file tags, so a compact helper avoids adding a wall-clock date
//! dependency to the core DRC binary.

use std::time::{SystemTime, UNIX_EPOCH};

/// Run or compute `current_day_number`.
pub fn current_day_number() -> Option<i64> {
    let seconds = SystemTime::now().duration_since(UNIX_EPOCH).ok()?.as_secs();
    Some((seconds / 86_400) as i64)
}

/// Run or compute `parse_iso_day`.
pub fn parse_iso_day(value: &str) -> Option<i64> {
    if value.len() != 10 {
        return None;
    }
    let bytes = value.as_bytes();
    if bytes[4] != b'-' || bytes[7] != b'-' {
        return None;
    }

    let year = parse_fixed_digits(&value[0..4])?;
    let month = parse_fixed_digits(&value[5..7])?;
    let day = parse_fixed_digits(&value[8..10])?;
    calendar_day_number(year, month, day)
}

/// Run or compute `parse_compact_day`.
pub fn parse_compact_day(value: &str) -> Option<i64> {
    if value.len() != 8 || !value.bytes().all(|byte| byte.is_ascii_digit()) {
        return None;
    }
    let year = parse_fixed_digits(&value[0..4])?;
    let month = parse_fixed_digits(&value[4..6])?;
    let day = parse_fixed_digits(&value[6..8])?;
    calendar_day_number(year, month, day)
}

fn calendar_day_number(year: i64, month: i64, day: i64) -> Option<i64> {
    if !(1..=12).contains(&month) || day < 1 || day > days_in_month(year, month) {
        return None;
    }
    Some(days_from_civil(year, month, day))
}

fn parse_fixed_digits(value: &str) -> Option<i64> {
    value
        .bytes()
        .all(|byte| byte.is_ascii_digit())
        .then(|| value.parse::<i64>().ok())
        .flatten()
}

fn days_in_month(year: i64, month: i64) -> i64 {
    match month {
        1 | 3 | 5 | 7 | 8 | 10 | 12 => 31,
        4 | 6 | 9 | 11 => 30,
        2 if is_leap_year(year) => 29,
        2 => 28,
        _ => 0,
    }
}

fn is_leap_year(year: i64) -> bool {
    year % 4 == 0 && (year % 100 != 0 || year % 400 == 0)
}

fn days_from_civil(year: i64, month: i64, day: i64) -> i64 {
    // Howard Hinnant's civil-calendar algorithm converts Gregorian Y-M-D dates
    // to days since the Unix epoch without a date crate. See Hinnant,
    // "chrono-Compatible Low-Level Date Algorithms."
    let year = year - i64::from(month <= 2);
    let era = if year >= 0 { year } else { year - 399 } / 400;
    let year_of_era = year - era * 400;
    let month_prime = month + if month > 2 { -3 } else { 9 };
    let day_of_year = (153 * month_prime + 2) / 5 + day - 1;
    let day_of_era = year_of_era * 365 + year_of_era / 4 - year_of_era / 100 + day_of_year;
    era * 146_097 + day_of_era - 719_468
}

#[cfg(test)]
mod tests {
    use super::{parse_compact_day, parse_iso_day};

    #[test]
    fn parses_iso_and_compact_dates_consistently() {
        assert_eq!(parse_iso_day("2026-05-13"), parse_compact_day("20260513"));
        assert_eq!(parse_iso_day("2024-02-29"), parse_compact_day("20240229"));
    }

    #[test]
    fn rejects_malformed_or_impossible_dates() {
        assert!(parse_iso_day("2026/05/13").is_none());
        assert!(parse_iso_day("2026-02-30").is_none());
        assert!(parse_compact_day("20260230").is_none());
        assert!(parse_compact_day("20261301").is_none());
    }
}
