//! Generic input conversion infrastructure.
//!
//! Converters are intentionally modeled as external tools. That keeps `hyperdrc`
//! focused on readiness checks while allowing format bridges such as TransJLC to
//! be added without entangling their file naming logic with the checker.

use std::path::{Path, PathBuf};
use std::process::Command;

use anyhow::{Context, Result, anyhow};

use crate::cli::{ConversionBackend, SourceEda};

#[derive(Clone, Debug)]
pub struct ConversionRequest {
    pub backend: ConversionBackend,
    pub input_dir: PathBuf,
    pub output_dir: PathBuf,
    pub source_eda: SourceEda,
    pub zip: bool,
    pub zip_name: String,
    pub top_color_image: Option<PathBuf>,
    pub bottom_color_image: Option<PathBuf>,
    pub transjlc_bin: PathBuf,
}

pub fn convert(request: &ConversionRequest) -> Result<ConversionOutput> {
    match request.backend {
        ConversionBackend::Transjlc => TransjlcConverter.convert(request),
    }
}

#[derive(Clone, Debug)]
pub struct ConversionOutput {
    pub source_dir: PathBuf,
    pub gerber_dir: PathBuf,
}

trait Converter {
    fn convert(&self, request: &ConversionRequest) -> Result<ConversionOutput>;
}

struct TransjlcConverter;

impl Converter for TransjlcConverter {
    fn convert(&self, request: &ConversionRequest) -> Result<ConversionOutput> {
        std::fs::create_dir_all(&request.output_dir)
            .with_context(|| format!("failed to create {}", request.output_dir.display()))?;

        let mut command = transjlc_command(request);
        let status = command.status().with_context(|| {
            format!(
                "failed to run TransJLC executable {}",
                request.transjlc_bin.display()
            )
        })?;

        if !status.success() {
            return Err(anyhow!(
                "TransJLC conversion failed for {} with status {status}",
                request.input_dir.display()
            ));
        }

        Ok(ConversionOutput {
            source_dir: request.input_dir.clone(),
            gerber_dir: request.output_dir.clone(),
        })
    }
}

fn transjlc_command(request: &ConversionRequest) -> Command {
    let mut command = Command::new(&request.transjlc_bin);
    command
        .arg("--path")
        .arg(&request.input_dir)
        .arg("--output_path")
        .arg(&request.output_dir)
        .arg("--eda")
        .arg(request.source_eda.as_transjlc_arg())
        .arg("--zip")
        .arg(if request.zip { "true" } else { "false" })
        .arg("--zip_name")
        .arg(&request.zip_name);

    if let Some(path) = &request.top_color_image {
        command.arg("--top_color_image").arg(path);
    }
    if let Some(path) = &request.bottom_color_image {
        command.arg("--bottom_color_image").arg(path);
    }

    command
}

pub fn default_conversion_output_dir(base: &Path, index: usize) -> PathBuf {
    base.join(format!("conversion-{index}"))
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use super::{ConversionOutput, ConversionRequest, default_conversion_output_dir};
    use crate::cli::{ConversionBackend, SourceEda};

    #[test]
    fn default_conversion_output_dir_is_stable_and_separate_per_input() {
        assert_eq!(
            default_conversion_output_dir(&PathBuf::from("out"), 2),
            PathBuf::from("out/conversion-2")
        );
    }

    #[test]
    fn conversion_output_points_at_generated_gerber_directory() {
        let output = ConversionOutput {
            source_dir: PathBuf::from("source"),
            gerber_dir: PathBuf::from("converted"),
        };

        assert_eq!(output.source_dir, PathBuf::from("source"));
        assert_eq!(output.gerber_dir, PathBuf::from("converted"));
    }

    #[test]
    fn transjlc_request_keeps_backend_specific_options_together() {
        let request = ConversionRequest {
            backend: ConversionBackend::Transjlc,
            input_dir: PathBuf::from("in"),
            output_dir: PathBuf::from("out"),
            source_eda: SourceEda::Kicad,
            zip: true,
            zip_name: "board".to_string(),
            top_color_image: Some(PathBuf::from("top.png")),
            bottom_color_image: None,
            transjlc_bin: PathBuf::from("TransJLC"),
        };

        assert_eq!(request.source_eda.as_transjlc_arg(), "kicad");
        assert!(request.zip);
    }
}
