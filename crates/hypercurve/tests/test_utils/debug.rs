use cavalier_contours::polyline::{PlineSource, Polyline};

/// Helper function to create json string from polyline to be used for debugging.
pub fn to_debug_json_str(pline: &Polyline) -> String {
    format!(
        r#"
{{
    "isClosed": {},
    "vertexes": [
        {}
    ]
}}
"#,
        pline.is_closed(),
        pline
            .iter_vertexes()
            .map(|v| format!("[{}, {}, {}]", v.x, v.y, v.bulge))
            .collect::<Vec<_>>()
            .join(",\n        ")
    )
}
