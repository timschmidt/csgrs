use hyperdrc::checks::{
    TextArtifact, TextArtifactParserKind, production_artifact_readiness,
    text_artifact_parser_diagnostics,
};
use serde_json::{Value, json};
use std::io::{self, BufRead};

fn main() {
    for line in io::stdin().lock().lines() {
        let row: Value = serde_json::from_str(&line.unwrap()).unwrap();
        let artifact = |key: &str| TextArtifact {
            path: format!("{key}.csv"),
            text: row[key].as_str().unwrap().to_string(),
        };
        let bom = artifact("bom");
        let centroid = artifact("centroid");
        let diagnostics =
            text_artifact_parser_diagnostics(&centroid, TextArtifactParserKind::Centroid);
        let violations =
            production_artifact_readiness(&[bom], &[centroid], &[], &[], &[], &[], &[]);
        println!(
            "{}",
            json!({"id": row["id"], "violations": violations, "diagnostics": diagnostics})
        );
    }
}
