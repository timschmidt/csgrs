//! SQLite report sink for design-readiness findings.
//!
//! The schema keeps frequently queried fields as columns and stores nested
//! geometry/provenance as JSON so no report information is discarded.

use std::path::Path;

use anyhow::{Context, Result};
use rusqlite::{Connection, params};

use crate::report::{Diagnostic, Report, Severity, Violation};

/// Write a report to a SQLite database file.
pub fn write_report_sqlite(report: &Report, path: &Path) -> Result<()> {
    if let Some(parent) = path
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
    {
        std::fs::create_dir_all(parent)
            .with_context(|| format!("failed to create {}", parent.display()))?;
    }
    let mut connection = Connection::open(path)
        .with_context(|| format!("failed to open SQLite report {}", path.display()))?;
    create_schema(&connection)?;
    let transaction = connection.transaction()?;
    insert_report(&transaction, report)?;
    transaction.commit()?;
    Ok(())
}

fn create_schema(connection: &Connection) -> Result<()> {
    connection.execute_batch(
        r#"
PRAGMA foreign_keys = ON;
CREATE TABLE IF NOT EXISTS run_summary (
    id INTEGER PRIMARY KEY CHECK (id = 1),
    violation_count INTEGER NOT NULL,
    waived_count INTEGER NOT NULL,
    errors INTEGER NOT NULL,
    warnings INTEGER NOT NULL,
    summary_json TEXT NOT NULL
);
CREATE TABLE IF NOT EXISTS inputs (
    id INTEGER PRIMARY KEY,
    adapter TEXT NOT NULL,
    role TEXT NOT NULL,
    path TEXT NOT NULL,
    origin TEXT,
    source_units TEXT,
    normalized_units TEXT,
    source_json TEXT NOT NULL
);
CREATE TABLE IF NOT EXISTS diagnostics (
    id INTEGER PRIMARY KEY,
    source TEXT NOT NULL,
    line INTEGER,
    severity TEXT NOT NULL,
    code TEXT NOT NULL,
    message TEXT NOT NULL
);
CREATE TABLE IF NOT EXISTS findings (
    id TEXT PRIMARY KEY,
    geometry_hash TEXT NOT NULL,
    waived INTEGER NOT NULL,
    check_name TEXT NOT NULL,
    severity TEXT NOT NULL,
    layers_json TEXT NOT NULL,
    island_index INTEGER,
    total_area REAL NOT NULL,
    polygons_json TEXT NOT NULL,
    locations_json TEXT NOT NULL,
    message TEXT
);
CREATE INDEX IF NOT EXISTS idx_findings_check ON findings(check_name);
CREATE INDEX IF NOT EXISTS idx_findings_geometry_hash ON findings(geometry_hash);
CREATE INDEX IF NOT EXISTS idx_findings_waived ON findings(waived);
CREATE INDEX IF NOT EXISTS idx_diagnostics_code ON diagnostics(code);
"#,
    )?;
    ensure_findings_geometry_hash_column(connection)?;
    Ok(())
}

fn ensure_findings_geometry_hash_column(connection: &Connection) -> Result<()> {
    let has_column = connection
        .prepare("PRAGMA table_info(findings)")?
        .query_map([], |row| row.get::<_, String>(1))?
        .collect::<std::result::Result<Vec<_>, _>>()?
        .iter()
        .any(|column| column == "geometry_hash");
    if !has_column {
        connection.execute(
            "ALTER TABLE findings ADD COLUMN geometry_hash TEXT NOT NULL DEFAULT ''",
            [],
        )?;
        connection.execute(
            "UPDATE findings SET geometry_hash = id WHERE geometry_hash = ''",
            [],
        )?;
    }
    Ok(())
}

fn insert_report(connection: &Connection, report: &Report) -> Result<()> {
    connection.execute("DELETE FROM run_summary", [])?;
    connection.execute("DELETE FROM inputs", [])?;
    connection.execute("DELETE FROM diagnostics", [])?;
    connection.execute("DELETE FROM findings", [])?;

    connection.execute(
        "INSERT INTO run_summary (id, violation_count, waived_count, errors, warnings, summary_json)
         VALUES (1, ?1, ?2, ?3, ?4, ?5)",
        params![
            report.violation_count as i64,
            report.waived_count as i64,
            report.summary.errors as i64,
            report.summary.warnings as i64,
            serde_json::to_string(&report.summary)?,
        ],
    )?;

    for input in &report.inputs {
        connection.execute(
            "INSERT INTO inputs
             (adapter, role, path, origin, source_units, normalized_units, source_json)
             VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7)",
            params![
                format!("{:?}", input.adapter),
                format!("{:?}", input.role),
                input.path.as_str(),
                input.origin.as_deref(),
                input.source_units.as_deref(),
                input.normalized_units.as_deref(),
                serde_json::to_string(input)?,
            ],
        )?;
    }

    for diagnostic in &report.diagnostics {
        insert_diagnostic(connection, diagnostic)?;
    }
    for violation in &report.violations {
        insert_violation(connection, violation, false)?;
    }
    for violation in &report.waived_violations {
        insert_violation(connection, violation, true)?;
    }
    Ok(())
}

fn insert_diagnostic(connection: &Connection, diagnostic: &Diagnostic) -> Result<()> {
    connection.execute(
        "INSERT INTO diagnostics (source, line, severity, code, message)
         VALUES (?1, ?2, ?3, ?4, ?5)",
        params![
            diagnostic.source.as_str(),
            diagnostic.line.map(|line| line as i64),
            severity_label(diagnostic.severity),
            diagnostic.code.as_str(),
            diagnostic.message.as_str(),
        ],
    )?;
    Ok(())
}

fn insert_violation(connection: &Connection, violation: &Violation, waived: bool) -> Result<()> {
    connection.execute(
        "INSERT OR REPLACE INTO findings
         (id, geometry_hash, waived, check_name, severity, layers_json, island_index, total_area,
          polygons_json, locations_json, message)
         VALUES (?1, ?2, ?3, ?4, ?5, ?6, ?7, ?8, ?9, ?10, ?11)",
        params![
            violation.id.as_str(),
            violation.id.as_str(),
            i64::from(waived),
            violation.check.as_str(),
            severity_label(violation.severity),
            serde_json::to_string(&violation.layers)?,
            violation.island_index.map(|index| index as i64),
            violation.total_area,
            serde_json::to_string(&violation.polygons)?,
            serde_json::to_string(&violation.locations)?,
            violation.message.as_deref(),
        ],
    )?;
    Ok(())
}

fn severity_label(severity: Severity) -> &'static str {
    match severity {
        Severity::Error => "error",
        Severity::Warning => "warning",
    }
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use rusqlite::Connection;

    use crate::io::{IoAdapter, IoRole, SourceRecord};
    use crate::report::{
        Diagnostic, Report, Severity, Violation, ViolationPolygon, report_summary,
    };

    use super::write_report_sqlite;

    #[test]
    fn writes_queryable_sqlite_report() {
        let active = vec![Violation::new(
            "spacing",
            Severity::Error,
            vec!["F.Cu".to_string()],
            None,
            vec![ViolationPolygon {
                area: 1.0,
                exterior: vec![[0.0, 0.0], [1.0, 0.0], [0.0, 0.0]],
                holes: Vec::new(),
            }],
            vec![[0.5, 0.5]],
            Some("too close".to_string()),
        )];
        let waived = vec![Violation::new(
            "silk",
            Severity::Warning,
            vec!["B.SilkS".to_string()],
            None,
            Vec::new(),
            vec![[2.0, 2.0]],
            Some("waived".to_string()),
        )];
        let report = Report {
            files: vec!["board.gbr".to_string()],
            inputs: vec![SourceRecord::new(
                IoAdapter::DirectFile,
                IoRole::GerberLayer,
                "board.gbr",
                Option::<&std::path::Path>::None,
            )],
            diagnostics: vec![Diagnostic {
                source: "board.gbr".to_string(),
                line: Some(3),
                severity: Severity::Warning,
                code: "gerber::example".to_string(),
                message: "example".to_string(),
            }],
            violation_count: active.len(),
            waived_count: waived.len(),
            waived_violations: waived,
            summary: report_summary(&active, 1),
            violations: active,
        };
        let path = PathBuf::from(format!(
            "/tmp/hyperdrc-report-{}.sqlite",
            std::process::id()
        ));
        let _ = std::fs::remove_file(&path);

        write_report_sqlite(&report, &path).unwrap();

        let connection = Connection::open(&path).unwrap();
        let active_count: i64 = connection
            .query_row(
                "SELECT COUNT(*) FROM findings WHERE waived = 0",
                [],
                |row| row.get(0),
            )
            .unwrap();
        let waived_count: i64 = connection
            .query_row(
                "SELECT COUNT(*) FROM findings WHERE waived = 1",
                [],
                |row| row.get(0),
            )
            .unwrap();
        let input_count: i64 = connection
            .query_row("SELECT COUNT(*) FROM inputs", [], |row| row.get(0))
            .unwrap();
        let diagnostic_count: i64 = connection
            .query_row("SELECT COUNT(*) FROM diagnostics", [], |row| row.get(0))
            .unwrap();
        let geometry_hash_count: i64 = connection
            .query_row(
                "SELECT COUNT(*) FROM findings WHERE geometry_hash = id",
                [],
                |row| row.get(0),
            )
            .unwrap();

        assert_eq!(active_count, 1);
        assert_eq!(waived_count, 1);
        assert_eq!(input_count, 1);
        assert_eq!(diagnostic_count, 1);
        assert_eq!(geometry_hash_count, 2);
        let _ = std::fs::remove_file(path);
    }
}
