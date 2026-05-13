pub mod app;
pub mod baseline;
pub mod checks;
pub mod cli;
pub mod config;
pub mod conversion;
pub mod date;
pub mod excellon;
pub mod geometry;
pub mod github_annotations;
pub mod html_report;
pub mod io;
pub mod ipc356;
pub mod jsonl;
pub mod junit;
pub mod kicad;
pub mod report;
pub mod sarif;
pub mod sexp;
pub mod svg_overlay;
pub mod waiver;

use csgrs::sketch::Sketch;

pub type PcbSketch = Sketch<LayerMetadata>;

#[derive(Clone, Debug)]
pub struct LayerMetadata {
    pub name: String,
}
