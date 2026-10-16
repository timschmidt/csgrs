//! Typed JavaScript descriptions of native format failures.

use crate::io::IoError;
use js_sys::{Error, Reflect};
use wasm_bindgen::prelude::*;

#[wasm_bindgen(typescript_custom_section)]
const IO_ERROR_TYPES: &str = r#"
/** A native validation category carried by an ordinary JavaScript Error. */
export type GeometryValidationError = Error & {
    readonly name: "GeometryError";
    readonly code: "invalid_input" | "unsupported" | "undecided" |
        "resource_exhausted" | "unbounded" | "geometry_failure";
};

/**
 * Native interchange failure. These are host Error objects, not Wasm handles.
 * Inspect code before reading variant-specific fields. Construction causes
 * retain the native validation category; no geometry is returned on failure.
 */
export type GeometryIoError = Error & {
    readonly name: "GeometryIoError";
    readonly format: string;
} & (
    | { readonly code: "std_io"; readonly ioKind: string; readonly osCode?: number; readonly cause: Error }
    | { readonly code: "parse_float"; readonly cause: Error }
    | { readonly code: "malformed_input"; readonly detail: string }
    | { readonly code: "unrepresentable_coordinate"; readonly field: string; readonly target: string }
    | { readonly code: "invalid_metadata"; readonly field: string }
    | { readonly code: "size_overflow"; readonly limit: string }
    | { readonly code: "unsupported"; readonly detail: string }
    | { readonly code: "geometry"; readonly detail: string }
    | { readonly code: "construction"; readonly cause: GeometryValidationError }
    | { readonly code: "dxf"; readonly cause: Error }
    | { readonly code: "gerber_parsing"; readonly detail: string }
    | { readonly code: "gerber_codegen"; readonly cause: Error }
);
"#;

fn property(exception: &Error, name: &str, value: &JsValue) {
    Reflect::set(exception, &JsValue::from_str(name), value)
        .expect("new Error object accepts diagnostic properties");
}

fn text_property(exception: &Error, name: &str, value: &str) {
    property(exception, name, &JsValue::from_str(value));
}

fn native_cause(error: impl std::fmt::Display, name: &str) -> JsValue {
    let cause = Error::new(&error.to_string());
    cause.set_name(name);
    cause.into()
}

/// Preserve the native variant and its fields, supplying boundary context for
/// variants (such as parser IO) that do not store a format name themselves.
/// Error strings and causes are owned by JavaScript and outlive mesh handles.
pub(super) fn io_error(error: IoError, format: &'static str) -> JsValue {
    let exception = Error::new(&error.to_string());
    exception.set_name("GeometryIoError");
    text_property(&exception, "format", format);
    let code = match error {
        IoError::StdIo(source) => {
            text_property(&exception, "ioKind", &format!("{:?}", source.kind()));
            if let Some(code) = source.raw_os_error() {
                property(&exception, "osCode", &JsValue::from(code));
            }
            property(&exception, "cause", &native_cause(source, "IoError"));
            "std_io"
        },
        IoError::ParseFloat(source) => {
            property(&exception, "cause", &native_cause(source, "ParseFloatError"));
            "parse_float"
        },
        IoError::MalformedInput(detail) => {
            text_property(&exception, "detail", &detail);
            "malformed_input"
        },
        IoError::UnrepresentableCoordinate {
            format,
            field,
            target,
        } => {
            text_property(&exception, "format", format);
            text_property(&exception, "field", field);
            text_property(&exception, "target", target);
            "unrepresentable_coordinate"
        },
        IoError::InvalidMetadata { format, field } => {
            text_property(&exception, "format", format);
            text_property(&exception, "field", field);
            "invalid_metadata"
        },
        IoError::SizeOverflow { format, limit } => {
            text_property(&exception, "format", format);
            text_property(&exception, "limit", limit);
            "size_overflow"
        },
        IoError::Unsupported { format, detail } => {
            text_property(&exception, "format", format);
            text_property(&exception, "detail", &detail);
            "unsupported"
        },
        IoError::Geometry { format, detail } => {
            text_property(&exception, "format", format);
            text_property(&exception, "detail", &detail);
            "geometry"
        },
        IoError::Construction { format, source } => {
            text_property(&exception, "format", format);
            property(&exception, "cause", &super::mesh_js::geometry_error(source));
            "construction"
        },
        #[cfg(feature = "dxf-io")]
        IoError::Dxf(source) => {
            property(&exception, "cause", &native_cause(source, "DxfError"));
            "dxf"
        },
        #[cfg(feature = "gerber-io")]
        IoError::GerberParsing(detail) => {
            text_property(&exception, "detail", &detail);
            "gerber_parsing"
        },
        #[cfg(feature = "gerber-io")]
        IoError::GerberCodegen(source) => {
            property(&exception, "cause", &native_cause(source, "GerberError"));
            "gerber_codegen"
        },
    };
    text_property(&exception, "code", code);
    exception.into()
}
