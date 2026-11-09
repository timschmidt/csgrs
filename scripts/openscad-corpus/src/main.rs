#![forbid(unsafe_code)]

use csgrs::{GeometryContext, SolidExpr, TriangleMesh, solid};
use hypercurve::{BooleanOp, CurveRegion2, FillRule, PredicatePolicy, Similarity2};
use serde_json::{Value, json};
use std::{error::Error, fs, io::Cursor, path::Path};
use synaps_cad::compiler::{
    Evaluator, PartEvidence, ShapeFailure, evaluator::Value as ScadValue, geometry::Shape,
};

fn failure(phase: &str, kind: &str, detail: impl std::fmt::Debug) -> Value {
    json!({"status":"failed", "phase":phase, "kind":kind, "detail":format!("{detail:?}")})
}

fn geometry_failure(phase: &str, error: impl Into<csgrs::Error>) -> Value {
    let error = error.into();
    failure(phase, error.kind().as_str(), error)
}

fn shape_failure(part: usize, error: &ShapeFailure) -> Value {
    let mut causes = Vec::new();
    let mut next = error.source();
    while let Some(cause) = next {
        causes.push(cause.to_string());
        next = cause.source();
    }
    json!({"part":part,"kind":error.kind().as_str(),"message":error.to_string(),
        "debug":format!("{error:?}"),"span":error.span().map(|s|json!({"start":s.start,"end":s.end})),"causes":causes})
}

fn stage(receipt: &Path, phase: &str) -> Result<(), Value> {
    fs::write(
        receipt.with_extension("phase.json"),
        json!({"phase":phase}).to_string(),
    )
    .map_err(|e| failure("phase_receipt", "io", e))
}

enum NativeScene {
    Empty,
    Mesh(TriangleMesh),
    Region(CurveRegion2),
}

// This is the pinned OpenSCAD boundary-export convention, not SVG paint.
// Closed paths describe model boundaries; viewport and stroke presentation do
// not describe material. The existing region arrangement owns winding/holes.
fn openscad_boundaries(text: &str) -> Result<CurveRegion2, Value> {
    const SVG: &str = "http://www.w3.org/2000/svg";
    // The pinned exporter emits this external SVG declaration. It has no
    // geometry semantics. Remove only this exact declaration; the XML parser
    // continues to reject other DTDs rather than adopting custom entities.
    let text = text.replacen(
        r#"<!DOCTYPE svg PUBLIC "-//W3C//DTD SVG 1.1//EN" "http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd">"#,
        "",
        1,
    );
    let document = roxmltree::Document::parse(&text)
        .map_err(|e| failure("reference_import", "svg_syntax", e))?;
    let root = document.root_element();
    if root.tag_name().name() != "svg"
        || root.tag_name().namespace() != Some(SVG)
        || root.attributes().any(|a| {
            a.namespace().is_some()
                || !["width", "height", "viewBox", "version"].contains(&a.name())
        })
    {
        return Err(failure(
            "reference_import",
            "unsupported_svg_boundary_convention",
            "root",
        ));
    }
    let mut paths = Vec::new();
    for node in root.children() {
        if node.is_comment()
            || (node.is_text() && node.text().unwrap_or_default().trim().is_empty())
        {
            continue;
        }
        if !node.is_element() || node.tag_name().namespace() != Some(SVG) {
            return Err(failure(
                "reference_import",
                "unsupported_svg_boundary_convention",
                "child",
            ));
        }
        match node.tag_name().name() {
            "title"
                if node.attributes().next().is_none()
                    && node.children().all(|n| n.is_text()) => {},
            "path" => {
                if node.attributes().any(|a| {
                    a.namespace().is_some()
                        || !["d", "fill", "stroke", "stroke-width"].contains(&a.name())
                }) || node.attribute("fill") != Some("none")
                    || node.children().any(|n| {
                        !n.is_text() || !n.text().unwrap_or_default().trim().is_empty()
                    })
                {
                    return Err(failure(
                        "reference_import",
                        "unsupported_svg_boundary_convention",
                        "path",
                    ));
                }
                let data = node
                    .attribute("d")
                    .ok_or_else(|| failure("reference_import", "missing_path_data", "d"))?;
                let subpaths = hypercurve::parse_svg_path_data(data)
                    .map_err(|e| failure("reference_import", "svg_path", e))?;
                for subpath in subpaths {
                    if !subpath.is_closed() {
                        return Err(failure(
                            "reference_import",
                            "open_reference_boundary",
                            data,
                        ));
                    }
                    paths.push(subpath.into_path());
                }
            },
            name => return Err(failure("reference_import", "unsupported_svg_element", name)),
        }
    }
    if paths.is_empty() {
        return Err(failure(
            "reference_import",
            "missing_reference_geometry",
            "Use an explicit empty-model receipt",
        ));
    }
    let region = CurveRegion2::try_from_boundary_paths(&paths, FillRule::NonZero)
        .map_err(|e| geometry_failure("reference_import", e))?;
    let reflection = Similarity2::try_from_real_affine(
        1.into(),
        0.into(),
        0.into(),
        (-1).into(),
        0.into(),
        0.into(),
    )
    .map_err(|e| geometry_failure("reference_import", e))?;
    region
        .transform_similarity(&reflection)
        .map_err(|e| geometry_failure("reference_import", e))
}

fn reference_obj(text: &str) -> Result<csgrs::GeometryOutcome<TriangleMesh>, Value> {
    let mut vertices = 0;
    let mut faces = 0;
    for line in text.lines() {
        let line = line.split('#').next().unwrap_or_default();
        let fields = line.split_whitespace().collect::<Vec<_>>();
        match fields.first().copied() {
            None => {},
            Some("v") if fields.len() == 4 => vertices += 1,
            Some("f") if fields.len() >= 4 => faces += 1,
            Some("o" | "g" | "s" | "vt" | "vn" | "usemtl" | "mtllib") => {},
            _ => return Err(failure("reference_import", "unsupported_obj_record", line)),
        }
    }
    if vertices == 0 || faces == 0 {
        return Err(failure(
            "reference_import",
            "missing_obj_geometry",
            (vertices, faces),
        ));
    }
    csgrs::io::obj::from_obj_with_context(Cursor::new(text), GeometryContext::STRICT)
        .map_err(|e| failure("reference_import", "obj_admission", e))
}

fn mesh_comparison(
    native: &TriangleMesh,
    reference: &TriangleMesh,
    receipt: &Path,
) -> Result<Value, Value> {
    stage(receipt, "mesh_symmetric_difference")?;
    let difference = solid::boolean_with_context(
        native,
        reference,
        hypermesh::BooleanOp::SymmetricDifference,
        GeometryContext::STRICT,
    )
    .map_err(|e| geometry_failure("comparison", e))?;
    let equal = difference.value.positions.is_empty() && difference.value.triangles.is_empty();
    stage(receipt, "mesh_continuation")?;
    let continued = solid::boolean_with_context(
        &difference.value,
        &solid::empty(),
        hypermesh::BooleanOp::Union,
        GeometryContext::STRICT,
    )
    .map_err(|e| geometry_failure("continuation", e))?;
    let identity = solid::boolean_with_context(
        &difference.value,
        &continued.value,
        hypermesh::BooleanOp::SymmetricDifference,
        GeometryContext::STRICT,
    )
    .map_err(|e| geometry_failure("continuation_identity", e))?;
    Ok(
        json!({"status":if equal {"represented_sets_equal"} else {"represented_sets_different"},
        "dimension":3,
        "principal_certainty":format!("{:?}",difference.certainty),
        "continuation_certainty":format!("{:?}",continued.certainty),
        "continuation_identity_certainty":format!("{:?}",identity.certainty),
        "continuation_identity_empty":identity.value.positions.is_empty() && identity.value.triangles.is_empty(),
        "native_construction_flags":PartEvidence::of_mesh(native).flags(),
        "difference_construction_flags":PartEvidence::of_mesh(&difference.value).flags()}),
    )
}

fn region_comparison(
    native: &CurveRegion2,
    reference: &CurveRegion2,
    receipt: &Path,
) -> Result<Value, Value> {
    stage(receipt, "region_symmetric_difference")?;
    let difference = native
        .boolean_region(reference, BooleanOp::Xor)
        .map_err(|e| geometry_failure("comparison", e))?;
    stage(receipt, "region_continuation")?;
    let continued = difference
        .boolean_region(&CurveRegion2::empty(), BooleanOp::Union)
        .map_err(|e| geometry_failure("continuation", e))?;
    let identity = difference
        .boolean_region(&continued, BooleanOp::Xor)
        .map_err(|e| geometry_failure("continuation_identity", e))?;
    Ok(
        json!({"status":if difference.is_empty() {"represented_sets_equal"} else {"represented_sets_different"},
        "dimension":2,
        "continuation_identity_empty":identity.is_empty()}),
    )
}

fn run(source: &Path, reference: Option<&Path>, receipt: &Path) -> Result<Value, Value> {
    stage(receipt, "source_read")?;
    let code = fs::read_to_string(source)
        .map_err(|e| failure("source_read", "source_encoding_or_io", e))?;
    stage(receipt, "parse")?;
    let parsed = openscad_rs::parse(&code).map_err(|e| failure("parse", "parser", e))?;
    let mut evaluator = Evaluator::new();
    evaluator
        .variables
        .insert("$preview".into(), ScadValue::Bool(false));
    stage(receipt, "evaluate")?;
    let shapes = evaluator.eval_source_file(&parsed);
    let mut meshes = Vec::new();
    let mut regions = Vec::new();
    let mut failures = Vec::new();
    let mut parts = Vec::new();
    for (index, (shape, _color)) in shapes.into_iter().enumerate() {
        stage(receipt, "resolve_native_part")?;
        match shape.resolve() {
            Shape::TriangleMesh3D(mesh) => {
                parts.push(json!({"part":index+1,"dimension":3,"vertices":mesh.positions.len(),"triangles":mesh.triangles.len(),"construction_flags":PartEvidence::of_mesh(&mesh).flags()}));
                meshes.push(mesh);
            },
            Shape::CurveRegion2D(region) => {
                parts.push(json!({"part":index+1,"dimension":2,"empty":region.is_empty()}));
                regions.push(region);
            },
            Shape::Failed(error) => failures.push(shape_failure(index + 1, &error)),
            Shape::Csg3D { .. } => {
                return Err(failure(
                    "resolve_native_part",
                    "unresolved_expression",
                    "Shape::resolve returned Csg3D",
                ));
            },
        }
    }
    let mut observations = json!({"parts":parts,"failures":failures,"diagnostics":evaluator.warnings,
        "evaluation_mode":"native Evaluator, preview=false, original fn/fa/fs defaults; no renderer buffers",
        "source_semantic_fidelity":"not certified by this harness; compiler coverage and all diagnostics remain separate"});
    if !failures.is_empty() {
        return Ok(json!({"status":"native_failed","native":observations,"comparison":null}));
    }
    if !meshes.is_empty() && !regions.is_empty() {
        return Ok(
            json!({"status":"native_mixed_dimensions","native":observations,"comparison":null}),
        );
    }
    let processed = (|| -> Result<Value, Value> {
        stage(receipt, "scene_union")?;
        let scene = if !meshes.is_empty() {
            let expr = meshes
                .into_iter()
                .fold(SolidExpr::from(solid::empty()), |expr, mesh| {
                    expr | SolidExpr::from(mesh)
                });
            let outcome = expr
                .evaluate_with_context(GeometryContext::STRICT)
                .map_err(|e| failure("scene_union", e.kind().as_str(), e))?;
            observations["scene_principal_certainty"] =
                json!(format!("{:?}", outcome.certainty));
            NativeScene::Mesh(outcome.value)
        } else if !regions.is_empty() {
            let mut region = CurveRegion2::empty();
            for next in regions {
                region = region
                    .boolean_region(&next, BooleanOp::Union)
                    .map_err(|e| failure("scene_union", "curve_boolean", e))?;
            }
            NativeScene::Region(region)
        } else {
            NativeScene::Empty
        };
        let Some(reference) = reference else {
            return Ok(
                json!({"status":"native_observed_without_reference","native":observations,"dimension":match scene {NativeScene::Mesh(_)=>Some(3),NativeScene::Region(_)=>Some(2),NativeScene::Empty=>None},"comparison":null}),
            );
        };
        stage(receipt, "reference_read")?;
        let text = fs::read_to_string(reference)
            .map_err(|e| failure("reference_read", "source_encoding_or_io", e))?;
        let comparison = match reference.extension().and_then(|s| s.to_str()) {
            Some("obj") => {
                let reference = reference_obj(&text)?;
                let certainty = format!("{:?}", reference.certainty);
                let result = match scene {
                    NativeScene::Mesh(mesh) => {
                        mesh_comparison(&mesh, &reference.value, receipt)
                    },
                    NativeScene::Empty => {
                        mesh_comparison(&solid::empty(), &reference.value, receipt)
                    },
                    NativeScene::Region(_) => Err(failure(
                        "comparison",
                        "dimension_mismatch",
                        "native2D reference3D",
                    )),
                };
                result.map(|mut value| {
                    value["reference_import_certainty"] = json!(certainty);
                    value
                })
            },
            Some("svg") => {
                let reference = openscad_boundaries(&text)?;
                match scene {
                    NativeScene::Region(region) => {
                        region_comparison(&region, &reference, receipt)
                    },
                    NativeScene::Empty => {
                        region_comparison(&CurveRegion2::empty(), &reference, receipt)
                    },
                    NativeScene::Mesh(_) => Err(failure(
                        "comparison",
                        "dimension_mismatch",
                        "native3D reference2D",
                    )),
                }
            },
            Some("empty") if text == "D03_EXPLICITLY_ADOPTED_EMPTY_REFERENCE_MODEL_V1\n" => {
                match scene {
                    NativeScene::Mesh(mesh) => {
                        mesh_comparison(&mesh, &solid::empty(), receipt)
                    },
                    NativeScene::Region(region) => {
                        region_comparison(&region, &CurveRegion2::empty(), receipt)
                    },
                    NativeScene::Empty => region_comparison(
                        &CurveRegion2::empty(),
                        &CurveRegion2::empty(),
                        receipt,
                    ),
                }
            },
            _ => Err(failure(
                "reference_read",
                "unsupported_reference_format",
                reference,
            )),
        };
        Ok(
            json!({"status":"native_and_reference_observed","native":observations,"comparison":comparison.unwrap_or_else(|error|error),
        "reference_fidelity":"exact emitted OBJ/SVG boundary model only; no established error bound or equivalence to original source",
        "reference_svg_convention":"OpenSCAD closed paths, NonZero winding, ignore viewport/paint, exact y reflection"}),
        )
    })();
    let mut result = processed.unwrap_or_else(|error| error);
    result["native"] = observations;
    Ok(result)
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = std::env::args_os().skip(1).collect::<Vec<_>>();
    if args.len() != 3 {
        return Err("usage: d03-native-worker SOURCE REFERENCE_OR_DASH RECEIPT".into());
    }
    let source = Path::new(&args[0]);
    let reference = (args[1] != "-").then(|| Path::new(&args[1]));
    let receipt = Path::new(&args[2]);
    let provisional = hypercurve::evaluate_under(PredicatePolicy::STRICT, || {
        run(source, reference, receipt)
    });
    let certainty = provisional.certainty();
    let result = provisional.into_unverified().unwrap_or_else(|error| error);
    let output = json!({"protocol_version":1,"policy":"STRICT","curve_principal_certainty":format!("{certainty:?}"),"source":source,"result":result});
    fs::write(receipt, serde_json::to_vec_pretty(&output)?)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn boundary(body: &str) -> String {
        format!(
            r#"<svg xmlns="http://www.w3.org/2000/svg" version="1.1"><title>OpenSCAD Model</title>{body}</svg>"#
        )
    }

    fn parsed(text: &str) -> Result<CurveRegion2, Value> {
        hypercurve::evaluate_under(PredicatePolicy::STRICT, || openscad_boundaries(text))
            .into_unverified()
    }

    #[test]
    fn pinned_exporter_external_declaration_is_admitted() {
        let text = r#"<?xml version="1.0" standalone="no"?>
<!DOCTYPE svg PUBLIC "-//W3C//DTD SVG 1.1//EN" "http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd">
<svg xmlns="http://www.w3.org/2000/svg"><path d="M 0,0 L 1,0 L 1,1 z" fill="none"/></svg>"#;
        assert!(!parsed(text).unwrap().is_empty());
    }

    #[test]
    fn custom_entity_declaration_is_rejected() {
        let text = r#"<!DOCTYPE svg [<!ENTITY edge "1">]><svg xmlns="http://www.w3.org/2000/svg"><path d="M 0,0 L &edge;,0 L 1,1 z" fill="none"/></svg>"#;
        assert!(parsed(text).is_err());
    }

    #[test]
    fn pathless_svg_is_not_an_empty_model_receipt() {
        assert!(parsed(&boundary("")).is_err());
    }

    #[test]
    fn truncated_xml_is_not_geometry() {
        let text = boundary(r#"<path d="M 0,0 L 1,0 L 1,1 z" fill="none"/>"#);
        assert!(parsed(text.strip_suffix("</svg>").unwrap()).is_err());
    }

    #[test]
    fn paint_document_is_not_the_declared_boundary_convention() {
        assert!(parsed(&boundary(r#"<path d="M 0,0 L 1,0 L 1,1 z" fill="red"/>"#)).is_err());
    }

    #[test]
    fn foreign_namespace_is_rejected() {
        let text = boundary(r#"<path d="M 0,0 L 1,0 L 1,1 z" fill="none"/>"#).replace(
            "http://www.w3.org/2000/svg",
            "https://example.invalid/foreign",
        );
        assert!(parsed(&text).is_err());
    }

    #[test]
    fn transform_is_not_silently_dropped() {
        assert!(
            parsed(&boundary(
                r#"<path d="M 0,0 L 1,0 L 1,1 z" fill="none" transform="translate(2,3)"/>"#
            ))
            .is_err()
        );
    }

    #[test]
    fn open_path_is_not_a_filled_boundary() {
        assert!(parsed(&boundary(r#"<path d="M 0,0 L 1,0 L 1,1" fill="none"/>"#)).is_err());
    }

    #[test]
    fn closed_boundary_is_admitted() {
        assert!(
            !parsed(&boundary(r#"<path d="M 0,0 L 1,0 L 1,1 z" fill="none"/>"#))
                .unwrap()
                .is_empty()
        );
    }
}
