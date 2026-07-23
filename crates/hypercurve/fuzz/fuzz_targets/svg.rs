//! Fuzz strict SVG document and path-data boundaries.

#![no_main]

use hypercurve::{SvgOptions, import_svg_document, parse_svg_path_data};
use libfuzzer_sys::fuzz_target;
use std::fmt::Write;

fuzz_target!(|bytes: &[u8]| {
    if bytes.len() > 16 * 1024 {
        return;
    }
    let input = String::from_utf8_lossy(bytes);
    let _ = parse_svg_path_data(&input);
    if let Ok(geometry) = import_svg_document(&input) {
        let _ = geometry.to_svg_with_options(SvgOptions {
            curve_tolerance: 0.1,
            max_curve_segments: 4096,
            max_extension_bytes: 64 * 1024,
        });
    }

    let mut extension = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        write!(&mut extension, "{byte:02x}").unwrap();
    }
    let document = format!(
        r#"<svg xmlns="http://www.w3.org/2000/svg"><path fill="none" stroke="black" data-hypercurve-path="1:{extension}" d="M0 0 L1 1"/></svg>"#
    );
    let _ = import_svg_document(&document);
});
