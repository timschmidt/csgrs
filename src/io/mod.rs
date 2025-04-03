#[cfg(feature = "svg-io")]
mod svg;

#[derive(Debug, thiserror::Error)]
pub enum IoError {
    #[error("std::io::Error: {}", .0)]
    StdIo(#[from] std::io::Error),
    #[error(transparent)]
    ParseFloat(#[from] std::num::ParseFloatError),

    #[error("Input is malformed: {}", .0)]
    MalformedInput(String),
    #[error("The path is malformed: {}", .0)]
    MalformedPath(String),
    #[error("Feature is not implemented: {}", .0)]
    Unimplemented(String),

    #[cfg(feature = "svg-io")]
    #[error("SVG Parsing error: {}", .0)]
    SvgParsing(#[from] ::svg::parser::Error),
}
