//! Self-contained HTML report sink for design and manufacturing review packets.
//!
//! The HTML report intentionally embeds the existing SVG overlay instead of
//! creating a second drawing path. That keeps visual review consistent across
//! `--svg-overlay` artifacts and browser-based report bundles.

use crate::report::{Report, Severity, Violation};
use crate::svg_overlay;

/// Run the `report_to_html` design-readiness check or report helper.
pub fn report_to_html(report: &Report) -> String {
    let mut out = String::new();
    out.push_str("<!doctype html>\n<html lang=\"en\">\n<head>\n");
    out.push_str("<meta charset=\"utf-8\">\n");
    out.push_str("<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n");
    out.push_str("<title>hyperdrc report</title>\n");
    out.push_str("<style>\n");
    out.push_str(STYLE);
    out.push_str("</style>\n<script>\n");
    out.push_str(SCRIPT);
    out.push_str("</script>\n</head>\n<body>\n");
    out.push_str("<main>\n");
    out.push_str("<header>\n<h1>hyperdrc report</h1>\n");
    out.push_str(&format!(
        "<p>{} active finding(s), {} waived, {} error(s), {} warning(s).</p>\n",
        report.violation_count, report.waived_count, report.summary.errors, report.summary.warnings
    ));
    out.push_str("</header>\n");

    out.push_str("<section>\n<h2>Summary</h2>\n");
    out.push_str("<table><thead><tr><th>Check</th><th>Findings</th></tr></thead><tbody>\n");
    for check in &report.summary.checks {
        out.push_str(&format!(
            "<tr><td>{}</td><td>{}</td></tr>\n",
            escape_html(&check.check),
            check.count
        ));
    }
    if report.summary.checks.is_empty() {
        out.push_str("<tr><td colspan=\"2\">No active findings.</td></tr>\n");
    }
    out.push_str("</tbody></table>\n</section>\n");

    out.push_str("<section>\n<h2>Parser Diagnostics</h2>\n");
    if report.diagnostics.is_empty() {
        out.push_str("<p>No parser diagnostics were recorded.</p>\n");
    } else {
        out.push_str("<table><thead><tr><th>Severity</th><th>Source</th><th>Line</th><th>Code</th><th>Message</th></tr></thead><tbody>\n");
        for diagnostic in &report.diagnostics {
            out.push_str(&format!(
                "<tr><td>{}</td><td>{}</td><td>{}</td><td>{}</td><td>{}</td></tr>\n",
                escape_html(&format!("{:?}", diagnostic.severity)),
                escape_html(&diagnostic.source),
                diagnostic
                    .line
                    .map(|line| line.to_string())
                    .unwrap_or_default(),
                escape_html(&diagnostic.code),
                escape_html(&diagnostic.message)
            ));
        }
        out.push_str("</tbody></table>\n");
    }
    out.push_str("</section>\n");

    out.push_str("<section>\n<h2>Overlay</h2>\n<div class=\"overlay\">\n");
    out.push_str(&svg_overlay::report_to_svg(report));
    out.push_str("</div>\n</section>\n");

    let states = finding_states(report);
    if !states.is_empty() {
        out.push_str("<section>\n<h2>Finding State</h2>\n<div class=\"filter-controls\">\n");
        for state in &states {
            out.push_str(&format!(
                "<label><input type=\"checkbox\" data-state-toggle=\"{}\" checked> {}</label>\n",
                escape_html_attr(state),
                escape_html(state)
            ));
        }
        out.push_str("</div>\n</section>\n");
    }

    let layers = finding_layers(report);
    if !layers.is_empty() {
        out.push_str("<section>\n<h2>Layers</h2>\n<div class=\"filter-controls\">\n");
        for layer in &layers {
            out.push_str(&format!(
                "<label><input type=\"checkbox\" data-layer-toggle=\"{}\" checked> {}</label>\n",
                escape_html_attr(layer),
                escape_html(layer)
            ));
        }
        out.push_str("</div>\n</section>\n");
    }

    out.push_str("<section>\n<h2>Inputs</h2>\n");
    if report.inputs.is_empty() {
        out.push_str("<p>No structured input manifest was recorded.</p>\n");
    } else {
        out.push_str("<table><thead><tr><th>Role</th><th>Adapter</th><th>Path</th><th>Origin</th></tr></thead><tbody>\n");
        for input in &report.inputs {
            out.push_str(&format!(
                "<tr><td>{}</td><td>{}</td><td>{}</td><td>{}</td></tr>\n",
                escape_html(&format!("{:?}", input.role)),
                escape_html(&format!("{:?}", input.adapter)),
                escape_html(&input.path),
                escape_html(input.origin.as_deref().unwrap_or(""))
            ));
        }
        out.push_str("</tbody></table>\n");
    }
    out.push_str("</section>\n");

    out.push_str("<section>\n<h2>Findings</h2>\n");
    if report.violations.is_empty() && report.waived_violations.is_empty() {
        out.push_str("<p>No active findings.</p>\n");
    } else {
        out.push_str("<div class=\"findings\">\n");
        for violation in &report.violations {
            out.push_str(&finding_card(violation, "active"));
        }
        for violation in &report.waived_violations {
            out.push_str(&finding_card(violation, "waived"));
        }
        out.push_str("</div>\n");
    }
    out.push_str("</section>\n");
    out.push_str("</main>\n</body>\n</html>\n");
    out
}

fn finding_card(violation: &Violation, state: &str) -> String {
    let severity = match violation.severity {
        Severity::Error => "error",
        Severity::Warning => "warning",
    };
    let mut out = String::new();
    out.push_str(&format!(
        "<article class=\"finding {severity} {state}\" data-state=\"{}\" data-layers=\"{}\" data-geometry-hash=\"{}\">\n",
        escape_html_attr(state),
        escape_html_attr(&violation.layers.join("|")),
        escape_html_attr(&violation.id)
    ));
    out.push_str(&format!(
        "<h3>{}</h3>\n<p class=\"meta\">{} · {} · {}</p>\n",
        escape_html(&violation.check),
        escape_html(state),
        escape_html(&violation.id),
        escape_html(&violation.layers.join(", "))
    ));
    out.push_str(&format!(
        "<p class=\"meta\">Geometry hash: <code>{}</code></p>\n",
        escape_html(&violation.id)
    ));
    if let Some(message) = &violation.message {
        out.push_str(&format!("<p>{}</p>\n", escape_html(message)));
    }
    if !violation.locations.is_empty() {
        out.push_str(&format!(
            "<p class=\"meta\">Locations: {}</p>\n",
            escape_html(&coordinates(&violation.locations))
        ));
    }
    if !violation.polygons.is_empty() {
        out.push_str(&format!(
            "<p class=\"meta\">{} polygon(s), total area {:.6}</p>\n",
            violation.polygons.len(),
            violation.total_area
        ));
    }
    out.push_str("</article>\n");
    out
}

fn finding_states(report: &Report) -> Vec<String> {
    let mut states = Vec::new();
    if !report.violations.is_empty() {
        states.push("active".to_string());
    }
    if !report.waived_violations.is_empty() {
        states.push("waived".to_string());
    }
    states
}

fn finding_layers(report: &Report) -> Vec<String> {
    let mut layers = std::collections::BTreeSet::new();
    for violation in report
        .violations
        .iter()
        .chain(report.waived_violations.iter())
    {
        for layer in &violation.layers {
            if !layer.is_empty() {
                layers.insert(layer.clone());
            }
        }
    }
    layers.into_iter().collect()
}

fn coordinates(locations: &[[f64; 2]]) -> String {
    locations
        .iter()
        .take(6)
        .map(|point| format!("({:.6}, {:.6})", point[0], point[1]))
        .collect::<Vec<_>>()
        .join(", ")
}

fn escape_html(value: &str) -> String {
    value
        .replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
        .replace('\'', "&#39;")
}

fn escape_html_attr(value: &str) -> String {
    escape_html(value)
}

const STYLE: &str = r#"
:root {
  color-scheme: light;
  font-family: Inter, ui-sans-serif, system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
  color: #17202a;
  background: #f6f8fa;
}
body {
  margin: 0;
}
main {
  max-width: 1120px;
  margin: 0 auto;
  padding: 32px 20px 48px;
}
header, section {
  margin-bottom: 28px;
}
h1, h2, h3, p {
  margin-top: 0;
}
table {
  width: 100%;
  border-collapse: collapse;
  background: #ffffff;
  border: 1px solid #d8dee4;
}
th, td {
  padding: 8px 10px;
  border-bottom: 1px solid #d8dee4;
  text-align: left;
  vertical-align: top;
}
th {
  background: #eef2f5;
}
.overlay {
  overflow: auto;
  background: #ffffff;
  border: 1px solid #d8dee4;
}
.overlay svg {
  display: block;
  width: 100%;
  min-height: 280px;
}
.filter-controls {
  display: flex;
  flex-wrap: wrap;
  gap: 8px;
}
.filter-controls label {
  display: inline-flex;
  align-items: center;
  gap: 6px;
  background: #ffffff;
  border: 1px solid #d8dee4;
  padding: 6px 10px;
}
.findings {
  display: grid;
  grid-template-columns: repeat(auto-fit, minmax(280px, 1fr));
  gap: 12px;
}
.finding {
  background: #ffffff;
  border: 1px solid #d8dee4;
  border-left-width: 6px;
  padding: 12px;
}
.finding.error {
  border-left-color: #cf222e;
}
.finding.warning {
  border-left-color: #9a6700;
}
.finding.waived {
  opacity: 0.72;
  border-left-style: dashed;
}
.meta {
  color: #57606a;
  font-size: 0.92rem;
}
"#;

const SCRIPT: &str = r#"
document.addEventListener("DOMContentLoaded", () => {
  const toggles = Array.from(document.querySelectorAll("[data-layer-toggle]"));
  const stateToggles = Array.from(document.querySelectorAll("[data-state-toggle]"));
  const layerSeparator = "|";
  const activeLayers = () => new Set(
    toggles
      .filter((toggle) => toggle.checked)
      .map((toggle) => toggle.getAttribute("data-layer-toggle"))
  );
  const activeStates = () => new Set(
    stateToggles
      .filter((toggle) => toggle.checked)
      .map((toggle) => toggle.getAttribute("data-state-toggle"))
  );
  const itemLayers = (item) => (item.getAttribute("data-layers") || "")
    .split(layerSeparator)
    .filter(Boolean);
  const refresh = () => {
    const active = activeLayers();
    const states = activeStates();
    document.querySelectorAll("[data-layers]").forEach((item) => {
      const layers = itemLayers(item);
      const state = item.getAttribute("data-state");
      const layerVisible = layers.length === 0 || layers.some((layer) => active.has(layer));
      const stateVisible = !state || states.size === 0 || states.has(state);
      const visible = layerVisible && stateVisible;
      item.style.display = visible ? "" : "none";
    });
  };
  toggles.forEach((toggle) => toggle.addEventListener("change", refresh));
  stateToggles.forEach((toggle) => toggle.addEventListener("change", refresh));
  refresh();
});
"#;

#[cfg(test)]
mod tests {
    use crate::report::{Report, Severity, Violation, report_summary};

    use super::report_to_html;

    #[test]
    fn html_report_contains_summary_overlay_and_escaped_findings() {
        let violations = vec![Violation::new(
            "mask<sliver>",
            Severity::Error,
            vec!["F.Mask".to_string()],
            None,
            Vec::new(),
            vec![[1.0, 2.0]],
            Some("opening crosses R1 & C1".to_string()),
        )];
        let waived = vec![Violation::new(
            "waived-check",
            Severity::Warning,
            vec!["B.Mask".to_string()],
            None,
            Vec::new(),
            vec![[3.0, 4.0]],
            Some("accepted finding".to_string()),
        )];
        let report = Report {
            files: vec!["board.gbr".to_string()],
            inputs: Vec::new(),
            diagnostics: Vec::new(),
            violation_count: violations.len(),
            waived_count: waived.len(),
            waived_violations: waived,
            summary: report_summary(&violations, 2),
            violations,
        };

        let html = report_to_html(&report);

        assert!(html.contains("<!doctype html>"));
        assert!(html.contains("<svg"));
        assert!(html.contains("data-layer-toggle=\"F.Mask\""));
        assert!(html.contains("data-layer-toggle=\"B.Mask\""));
        assert!(html.contains("data-state-toggle=\"active\""));
        assert!(html.contains("data-state-toggle=\"waived\""));
        assert!(html.contains("data-layers=\"F.Mask\""));
        assert!(html.contains("data-state=\"waived\""));
        assert!(html.contains("data-geometry-hash=\""));
        assert!(html.contains("Geometry hash: <code>"));
        assert!(html.contains("mask&lt;sliver&gt;"));
        assert!(html.contains("opening crosses R1 &amp; C1"));
        assert!(html.contains("1 waived"));
    }
}
