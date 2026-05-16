//! Arrow IPC report sink for design-readiness findings.
//!
//! The file uses one wide schema with a `record_type` discriminator so summary,
//! input, diagnostic, and finding rows can be read by standard Arrow tools
//! without coordinating multiple schemas.

use std::fs::File;
use std::path::Path;
use std::sync::Arc;

use anyhow::{Context, Result};
use arrow_array::builder::{BooleanBuilder, Float64Builder, StringBuilder, UInt64Builder};
use arrow_array::{ArrayRef, RecordBatch};
use arrow_ipc::writer::FileWriter;
use arrow_schema::{DataType, Field, Schema};

use crate::report::{Diagnostic, Report, Severity, Violation};

/// Write a report to an Arrow IPC file.
pub fn write_report_arrow(report: &Report, path: &Path) -> Result<()> {
    if let Some(parent) = path
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
    {
        std::fs::create_dir_all(parent)
            .with_context(|| format!("failed to create {}", parent.display()))?;
    }
    let schema = Arc::new(report_schema());
    let batch = report_batch(report, Arc::clone(&schema))?;
    let file = File::create(path)
        .with_context(|| format!("failed to create Arrow report {}", path.display()))?;
    let mut writer = FileWriter::try_new(file, &schema)
        .with_context(|| format!("failed to open Arrow writer for {}", path.display()))?;
    writer
        .write(&batch)
        .with_context(|| format!("failed to write Arrow report {}", path.display()))?;
    writer
        .finish()
        .with_context(|| format!("failed to finish Arrow report {}", path.display()))?;
    Ok(())
}

pub(crate) fn report_schema() -> Schema {
    Schema::new(vec![
        Field::new("record_type", DataType::Utf8, false),
        Field::new("id", DataType::Utf8, true),
        Field::new("geometry_hash", DataType::Utf8, true),
        Field::new("waived", DataType::Boolean, true),
        Field::new("adapter", DataType::Utf8, true),
        Field::new("role", DataType::Utf8, true),
        Field::new("source", DataType::Utf8, true),
        Field::new("line", DataType::UInt64, true),
        Field::new("severity", DataType::Utf8, true),
        Field::new("code", DataType::Utf8, true),
        Field::new("check_name", DataType::Utf8, true),
        Field::new("message", DataType::Utf8, true),
        Field::new("layers_json", DataType::Utf8, true),
        Field::new("total_area", DataType::Float64, true),
        Field::new("geometry_json", DataType::Utf8, true),
        Field::new("source_json", DataType::Utf8, true),
        Field::new("summary_json", DataType::Utf8, true),
    ])
}

pub(crate) fn report_batch(report: &Report, schema: Arc<Schema>) -> Result<RecordBatch> {
    let row_count = 1
        + report.inputs.len()
        + report.diagnostics.len()
        + report.violations.len()
        + report.waived_violations.len();
    let mut rows = Rows::with_capacity(row_count);

    rows.push_summary(report)?;
    for input in &report.inputs {
        rows.push_input(input)?;
    }
    for diagnostic in &report.diagnostics {
        rows.push_diagnostic(diagnostic);
    }
    for violation in &report.violations {
        rows.push_violation(violation, false)?;
    }
    for violation in &report.waived_violations {
        rows.push_violation(violation, true)?;
    }

    RecordBatch::try_new(schema, rows.finish()).context("failed to build Arrow report batch")
}

struct Rows {
    record_type: StringBuilder,
    id: StringBuilder,
    geometry_hash: StringBuilder,
    waived: BooleanBuilder,
    adapter: StringBuilder,
    role: StringBuilder,
    source: StringBuilder,
    line: UInt64Builder,
    severity: StringBuilder,
    code: StringBuilder,
    check_name: StringBuilder,
    message: StringBuilder,
    layers_json: StringBuilder,
    total_area: Float64Builder,
    geometry_json: StringBuilder,
    source_json: StringBuilder,
    summary_json: StringBuilder,
}

impl Rows {
    fn with_capacity(rows: usize) -> Self {
        Self {
            record_type: StringBuilder::with_capacity(rows, rows * 12),
            id: StringBuilder::with_capacity(rows, rows * 16),
            geometry_hash: StringBuilder::with_capacity(rows, rows * 16),
            waived: BooleanBuilder::with_capacity(rows),
            adapter: StringBuilder::with_capacity(rows, rows * 12),
            role: StringBuilder::with_capacity(rows, rows * 16),
            source: StringBuilder::with_capacity(rows, rows * 24),
            line: UInt64Builder::with_capacity(rows),
            severity: StringBuilder::with_capacity(rows, rows * 8),
            code: StringBuilder::with_capacity(rows, rows * 24),
            check_name: StringBuilder::with_capacity(rows, rows * 24),
            message: StringBuilder::with_capacity(rows, rows * 32),
            layers_json: StringBuilder::with_capacity(rows, rows * 24),
            total_area: Float64Builder::with_capacity(rows),
            geometry_json: StringBuilder::with_capacity(rows, rows * 64),
            source_json: StringBuilder::with_capacity(rows, rows * 64),
            summary_json: StringBuilder::with_capacity(rows, rows * 32),
        }
    }

    fn push_summary(&mut self, report: &Report) -> Result<()> {
        self.record_type.append_value("summary");
        self.id.append_value("summary");
        self.geometry_hash.append_null();
        self.waived.append_null();
        self.adapter.append_null();
        self.role.append_null();
        self.source.append_null();
        self.line.append_null();
        self.severity.append_null();
        self.code.append_null();
        self.check_name.append_null();
        self.message.append_null();
        self.layers_json.append_null();
        self.total_area.append_null();
        self.geometry_json.append_null();
        self.source_json.append_null();
        self.summary_json
            .append_value(serde_json::to_string(&report.summary)?);
        Ok(())
    }

    fn push_input(&mut self, input: &crate::io::SourceRecord) -> Result<()> {
        self.record_type.append_value("input");
        self.id.append_null();
        self.geometry_hash.append_null();
        self.waived.append_null();
        self.adapter.append_value(format!("{:?}", input.adapter));
        self.role.append_value(format!("{:?}", input.role));
        self.source.append_value(input.path.as_str());
        self.line.append_null();
        self.severity.append_null();
        self.code.append_null();
        self.check_name.append_null();
        self.message.append_null();
        self.layers_json.append_null();
        self.total_area.append_null();
        self.geometry_json.append_null();
        self.source_json.append_value(serde_json::to_string(input)?);
        self.summary_json.append_null();
        Ok(())
    }

    fn push_diagnostic(&mut self, diagnostic: &Diagnostic) {
        self.record_type.append_value("diagnostic");
        self.id.append_null();
        self.geometry_hash.append_null();
        self.waived.append_null();
        self.adapter.append_null();
        self.role.append_null();
        self.source.append_value(diagnostic.source.as_str());
        append_u64(&mut self.line, diagnostic.line);
        self.severity
            .append_value(severity_label(diagnostic.severity));
        self.code.append_value(diagnostic.code.as_str());
        self.check_name.append_null();
        self.message.append_value(diagnostic.message.as_str());
        self.layers_json.append_null();
        self.total_area.append_null();
        self.geometry_json.append_null();
        self.source_json.append_null();
        self.summary_json.append_null();
    }

    fn push_violation(&mut self, violation: &Violation, waived: bool) -> Result<()> {
        self.record_type.append_value("finding");
        self.id.append_value(violation.id.as_str());
        self.geometry_hash.append_value(violation.id.as_str());
        self.waived.append_value(waived);
        self.adapter.append_null();
        self.role.append_null();
        self.source.append_null();
        self.line.append_null();
        self.severity
            .append_value(severity_label(violation.severity));
        self.code.append_null();
        self.check_name.append_value(violation.check.as_str());
        append_string(&mut self.message, violation.message.as_deref());
        self.layers_json
            .append_value(serde_json::to_string(&violation.layers)?);
        self.total_area.append_value(violation.total_area);
        self.geometry_json
            .append_value(serde_json::to_string(&serde_json::json!({
                "polygons": violation.polygons,
                "locations": violation.locations,
            }))?);
        self.source_json.append_null();
        self.summary_json.append_null();
        Ok(())
    }

    fn finish(mut self) -> Vec<ArrayRef> {
        vec![
            Arc::new(self.record_type.finish()),
            Arc::new(self.id.finish()),
            Arc::new(self.geometry_hash.finish()),
            Arc::new(self.waived.finish()),
            Arc::new(self.adapter.finish()),
            Arc::new(self.role.finish()),
            Arc::new(self.source.finish()),
            Arc::new(self.line.finish()),
            Arc::new(self.severity.finish()),
            Arc::new(self.code.finish()),
            Arc::new(self.check_name.finish()),
            Arc::new(self.message.finish()),
            Arc::new(self.layers_json.finish()),
            Arc::new(self.total_area.finish()),
            Arc::new(self.geometry_json.finish()),
            Arc::new(self.source_json.finish()),
            Arc::new(self.summary_json.finish()),
        ]
    }
}

fn append_string(builder: &mut StringBuilder, value: Option<&str>) {
    match value {
        Some(value) => builder.append_value(value),
        None => builder.append_null(),
    }
}

fn append_u64(builder: &mut UInt64Builder, value: Option<usize>) {
    match value {
        Some(value) => builder.append_value(value as u64),
        None => builder.append_null(),
    }
}

fn severity_label(severity: Severity) -> &'static str {
    match severity {
        Severity::Error => "error",
        Severity::Warning => "warning",
    }
}

#[cfg(test)]
mod tests {
    use std::fs::File;
    use std::path::PathBuf;

    use arrow_array::{Array, StringArray};
    use arrow_ipc::reader::FileReader;

    use crate::io::{IoAdapter, IoRole, SourceRecord};
    use crate::report::{
        Diagnostic, Report, Severity, Violation, ViolationPolygon, report_summary,
    };

    use super::write_report_arrow;

    #[test]
    fn writes_readable_arrow_report() {
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
        let path = PathBuf::from(format!("/tmp/hyperdrc-report-{}.arrow", std::process::id()));
        let _ = std::fs::remove_file(&path);

        write_report_arrow(&report, &path).unwrap();

        let file = File::open(&path).unwrap();
        let reader = FileReader::try_new(file, None).unwrap();
        let schema = reader.schema();
        assert!(schema.field_with_name("record_type").is_ok());
        assert!(schema.field_with_name("geometry_hash").is_ok());
        assert!(schema.field_with_name("geometry_json").is_ok());
        let batches: Vec<_> = reader.map(Result::unwrap).collect();
        assert_eq!(batches.len(), 1);
        assert_eq!(batches[0].num_rows(), 5);
        let record_type = batches[0]
            .column_by_name("record_type")
            .unwrap()
            .as_any()
            .downcast_ref::<StringArray>()
            .unwrap();
        assert_eq!(record_type.value(0), "summary");
        assert_eq!(record_type.value(1), "input");
        assert_eq!(record_type.value(2), "diagnostic");
        assert_eq!(record_type.value(3), "finding");
        assert_eq!(record_type.value(4), "finding");
        let geometry_hash = batches[0]
            .column_by_name("geometry_hash")
            .unwrap()
            .as_any()
            .downcast_ref::<StringArray>()
            .unwrap();
        let id = batches[0]
            .column_by_name("id")
            .unwrap()
            .as_any()
            .downcast_ref::<StringArray>()
            .unwrap();
        assert!(geometry_hash.is_null(0));
        assert_eq!(geometry_hash.value(3), id.value(3));
        assert_eq!(geometry_hash.value(4), id.value(4));
        let _ = std::fs::remove_file(path);
    }
}
