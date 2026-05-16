//! Parquet report sink for design-readiness findings.
//!
//! This sink reuses the Arrow report schema and record batch, then writes a
//! single Parquet file for warehouse and columnar analytics workflows.

use std::fs::File;
use std::path::Path;
use std::sync::Arc;

use anyhow::{Context, Result};
use parquet::arrow::ArrowWriter;

use crate::report::Report;

/// Write a report to a Parquet file.
pub fn write_report_parquet(report: &Report, path: &Path) -> Result<()> {
    if let Some(parent) = path
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
    {
        std::fs::create_dir_all(parent)
            .with_context(|| format!("failed to create {}", parent.display()))?;
    }
    let schema = Arc::new(crate::arrow_report::report_schema());
    let batch = crate::arrow_report::report_batch(report, Arc::clone(&schema))?;
    let file = File::create(path)
        .with_context(|| format!("failed to create Parquet report {}", path.display()))?;
    let mut writer = ArrowWriter::try_new(file, schema, None)
        .with_context(|| format!("failed to open Parquet writer for {}", path.display()))?;
    writer
        .write(&batch)
        .with_context(|| format!("failed to write Parquet report {}", path.display()))?;
    writer
        .close()
        .with_context(|| format!("failed to finish Parquet report {}", path.display()))?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::fs::File;
    use std::path::PathBuf;

    use arrow_array::StringArray;
    use parquet::arrow::arrow_reader::ParquetRecordBatchReaderBuilder;

    use crate::io::{IoAdapter, IoRole, SourceRecord};
    use crate::report::{
        Diagnostic, Report, Severity, Violation, ViolationPolygon, report_summary,
    };

    use super::write_report_parquet;

    #[test]
    fn writes_readable_parquet_report() {
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
            "/tmp/hyperdrc-report-{}.parquet",
            std::process::id()
        ));
        let _ = std::fs::remove_file(&path);

        write_report_parquet(&report, &path).unwrap();

        let file = File::open(&path).unwrap();
        let builder = ParquetRecordBatchReaderBuilder::try_new(file).unwrap();
        assert!(builder.schema().field_with_name("record_type").is_ok());
        assert!(builder.schema().field_with_name("geometry_json").is_ok());
        let reader = builder.build().unwrap();
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
        assert_eq!(record_type.value(4), "finding");
        let _ = std::fs::remove_file(path);
    }
}
