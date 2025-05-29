#[cfg(feature = "svg-io")]
mod svg;

#[cfg(feature = "stl-io")]
mod stl;

#[cfg(feature = "dxf-io")]
mod dxf;

/// Generic I/O and format‑conversion errors.
///
/// Many I/O features are behind cargo feature‑flags.  
/// When a feature is disabled the corresponding variant is *not*
/// constructed in user code.
#[derive(Debug)]
pub enum IoError {
    StdIo(std::io::Error),
    ParseFloat(std::num::ParseFloatError),

    MalformedInput(String),
    MalformedPath(String),
    Unimplemented(String),

    #[cfg(feature = "svg-io")]
    /// Error bubbled up from the `svg` crate during parsing.
    SvgParsing(::svg::parser::Error),
}

impl std::fmt::Display for IoError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        use IoError::*;

        match self {
            StdIo(error) => write!(f, "std::io::Error: {error}"),
            ParseFloat(error) => write!(f, "Could not parse float: {error}"),

            MalformedInput(msg) => write!(f, "Input is malformed: {msg}"),
            MalformedPath(msg) => write!(f, "The path is malformed: {msg}"),
            Unimplemented(msg) => write!(f, "Feature is not implemented: {msg}"),

            #[cfg(feature = "svg-io")]
            SvgParsing(error) => write!(f, "SVG Parsing error: {error}"),
        }
    }
}

impl std::error::Error for IoError {}

impl From<std::io::Error> for IoError {
    fn from(value: std::io::Error) -> Self {
        Self::StdIo(value)
    }
}

impl From<std::num::ParseFloatError> for IoError {
    fn from(value: std::num::ParseFloatError) -> Self {
        Self::ParseFloat(value)
    }
}

#[cfg(feature = "svg-io")]
impl From<::svg::parser::Error> for IoError {
    fn from(value: ::svg::parser::Error) -> Self {
        Self::SvgParsing(value)
    }
}
