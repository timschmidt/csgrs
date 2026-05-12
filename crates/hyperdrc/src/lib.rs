pub mod app;
pub mod checks;
pub mod cli;
pub mod config;
pub mod excellon;
pub mod geometry;
pub mod ipc356;
pub mod kicad;
pub mod report;
pub mod sexp;
pub mod svg_overlay;
pub mod waiver;

use csgrs::sketch::Sketch;

pub type PcbSketch = Sketch<LayerMetadata>;

#[derive(Clone, Debug)]
pub struct LayerMetadata {
    pub name: String,
}
