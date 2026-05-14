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
/// Public data model for `ConversionRequest`.
pub struct ConversionRequest {
    /// Field `backend`.
    pub backend: ConversionBackend,
    /// Field `input_dir`.
    pub input_dir: PathBuf,
    /// Field `output_dir`.
    pub output_dir: PathBuf,
    /// Field `source_eda`.
    pub source_eda: SourceEda,
    /// Field `zip`.
    pub zip: bool,
    /// Field `zip_name`.
    pub zip_name: String,
    /// Field `top_color_image`.
    pub top_color_image: Option<PathBuf>,
    /// Field `bottom_color_image`.
    pub bottom_color_image: Option<PathBuf>,
    /// Field `transjlc_bin`.
    pub transjlc_bin: PathBuf,
    /// Field `extra_args`.
    pub extra_args: Vec<String>,
}

/// Run or compute `convert`.
pub fn convert(request: &ConversionRequest) -> Result<ConversionOutput> {
    match request.backend {
        ConversionBackend::Transjlc => TransjlcConverter.convert(request),
    }
}

#[derive(Clone, Debug)]
/// Public data model for `ConversionOutput`.
pub struct ConversionOutput {
    /// Field `source_dir`.
    pub source_dir: PathBuf,
    /// Field `gerber_dir`.
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
    for arg in &request.extra_args {
        command.arg(arg);
    }

    command
}

/// Run or compute `default_conversion_output_dir`.
pub fn default_conversion_output_dir(base: &Path, index: usize) -> PathBuf {
    base.join(format!("conversion-{index}"))
}

#[cfg(test)]
mod tests {
    use std::fs::remove_dir_all;
    use std::path::PathBuf;
    use std::process;

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
            extra_args: vec!["--foo".to_string(), "--bar=baz".to_string()],
        };

        assert_eq!(request.source_eda.as_transjlc_arg(), "kicad");
        assert!(request.zip);
    }

    #[test]
    fn transjlc_command_includes_extra_backend_args() {
        let request = ConversionRequest {
            backend: ConversionBackend::Transjlc,
            input_dir: PathBuf::from("in"),
            output_dir: PathBuf::from("out"),
            source_eda: SourceEda::Kicad,
            zip: false,
            zip_name: "board".to_string(),
            top_color_image: None,
            bottom_color_image: None,
            transjlc_bin: PathBuf::from("transjlc"),
            extra_args: vec!["--foo".to_string(), "--bar=baz".to_string()],
        };

        let command = super::transjlc_command(&request);

        let rendered = format!("{:?}", command);
        assert!(rendered.contains("--foo"));
        assert!(rendered.contains("--bar=baz"));
    }

    #[test]
    fn transjlc_command_includes_color_images_when_supplied() {
        let request = ConversionRequest {
            backend: ConversionBackend::Transjlc,
            input_dir: PathBuf::from("in"),
            output_dir: PathBuf::from("out"),
            source_eda: SourceEda::Kicad,
            zip: false,
            zip_name: "board".to_string(),
            top_color_image: Some(PathBuf::from("top-art.png")),
            bottom_color_image: Some(PathBuf::from("bottom-art.png")),
            transjlc_bin: PathBuf::from("transjlc"),
            extra_args: Vec::new(),
        };

        let command = super::transjlc_command(&request);
        let rendered = format!("{:?}", command);

        assert!(rendered.contains("--top_color_image"));
        assert!(rendered.contains("top-art.png"));
        assert!(rendered.contains("--bottom_color_image"));
        assert!(rendered.contains("bottom-art.png"));
    }

    #[test]
    fn convert_executes_successful_converter_and_returns_output_paths() {
        let process_id = process::id();
        let temp_input = PathBuf::from(format!("/tmp/hyperdrc-conv-input-{process_id}"));
        let temp_output = PathBuf::from(format!("/tmp/hyperdrc-conv-output-{process_id}"));
        let _ = remove_dir_all(&temp_input);
        let _ = remove_dir_all(&temp_output);
        std::fs::create_dir_all(&temp_input).unwrap();

        let request = ConversionRequest {
            backend: ConversionBackend::Transjlc,
            input_dir: temp_input.clone(),
            output_dir: temp_output.clone(),
            source_eda: SourceEda::Kicad,
            zip: true,
            zip_name: "bundle".to_string(),
            top_color_image: None,
            bottom_color_image: None,
            transjlc_bin: PathBuf::from("/bin/true"),
            extra_args: vec!["--noop".to_string()],
        };

        let converted = super::convert(&request).unwrap();

        assert_eq!(converted.source_dir, temp_input);
        assert_eq!(converted.gerber_dir, temp_output);
        assert!(temp_output.exists());
        let _ = remove_dir_all(&temp_input);
        let _ = remove_dir_all(&temp_output);
    }

    #[test]
    fn convert_reports_command_failure_as_contextual_error() {
        let request = ConversionRequest {
            backend: ConversionBackend::Transjlc,
            input_dir: PathBuf::from("input"),
            output_dir: PathBuf::from("out"),
            source_eda: SourceEda::Auto,
            zip: false,
            zip_name: "Gerber".to_string(),
            top_color_image: None,
            bottom_color_image: None,
            transjlc_bin: PathBuf::from("/bin/false"),
            extra_args: Vec::new(),
        };

        let error = super::convert(&request)
            .expect_err("conversion should fail when command returns non-zero");
        let message = format!("{}", error);

        assert!(message.contains("TransJLC conversion failed"));
        assert!(message.contains("input"));
    }

    #[test]
    fn convert_reports_missing_converter_binary() {
        let request = ConversionRequest {
            backend: ConversionBackend::Transjlc,
            input_dir: PathBuf::from("input"),
            output_dir: PathBuf::from("out"),
            source_eda: SourceEda::Auto,
            zip: false,
            zip_name: "Gerber".to_string(),
            top_color_image: None,
            bottom_color_image: None,
            transjlc_bin: PathBuf::from("/does-not-exist"),
            extra_args: Vec::new(),
        };

        let error = super::convert(&request)
            .expect_err("missing converter should return an execution error");
        let message = format!("{}", error);

        assert!(message.contains("failed to run TransJLC executable"));
        assert!(message.contains("/does-not-exist"));
    }

    #[test]
    fn convert_reports_directory_creation_failure() {
        let process_id = process::id();
        let temp_input = PathBuf::from(format!("/tmp/hyperdrc-conv-input-{process_id}"));
        let temp_output_file = PathBuf::from(format!("/tmp/hyperdrc-output-file-{process_id}.txt"));
        let _ = remove_dir_all(&temp_input);
        std::fs::create_dir_all(&temp_input).unwrap();
        std::fs::write(&temp_output_file, "file-path-conflict").unwrap();

        let request = ConversionRequest {
            backend: ConversionBackend::Transjlc,
            input_dir: temp_input.clone(),
            output_dir: temp_output_file,
            source_eda: SourceEda::Kicad,
            zip: true,
            zip_name: "bundle".to_string(),
            top_color_image: None,
            bottom_color_image: None,
            transjlc_bin: PathBuf::from("/bin/true"),
            extra_args: Vec::new(),
        };

        let error =
            super::convert(&request).expect_err("directory creation must fail for file path");
        let message = format!("{error}");

        assert!(message.contains("failed to create"));
        let _ = remove_dir_all(&temp_input);
        let _ = std::fs::remove_file(format!("/tmp/hyperdrc-output-file-{process_id}.txt"));
    }
}
