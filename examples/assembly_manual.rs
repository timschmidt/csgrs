//! Printed block, heat-fit insert, and screw: poses, a bill of materials, and
//! an assembly manual with both orthographic views.
//!
//! Run from the `csgrs` directory:
//!
//! ```text
//! cargo run --example assembly_manual
//! ```
//!
//! Output is written under `target/assembly_manual`, which Cargo already ignores.

use std::fs;
use std::path::PathBuf;

use csgrs::parts::{
    AssemblyDocument, AssemblyNode, AssemblyView, AttachmentFrame, AttachmentRole,
    ExplodeSpec, annotate, assembly_manual, bill_of_materials, blueprint_from_posed_mesh,
    dimension_x, export_blueprint_svg, mate, posed_parts,
};
use csgrs::parts::{BlueprintProjection, NodeId, PosedAssembly};
use csgrs::solid::{self, SolidExt};
use hyperlattice::{Point3, Real, Vector3};

fn main() {
    let (document, insert_id, screw_id) = build_document();
    let bom = bill_of_materials(&document);
    assert_eq!(bom.columns.len(), 1, "the root step owns the only column");
    let rows: Vec<_> = bom.columns[0]
        .lines
        .iter()
        .map(|line| (line.description.as_str(), line.quantity))
        .collect();
    assert_eq!(
        rows,
        vec![
            ("insert(M3): Heatfit insert M3", 1),
            ("screw(M3): Screw M3 cap x 12mm", 1),
            ("Printed block", 1),
        ]
    );

    let assembled = posed_parts(&document, AssemblyView::Assembled);
    let exploded = posed_parts(
        &document,
        AssemblyView::Exploded {
            step: document.root(),
        },
    );
    assert!(assembled.blockers.is_empty(), "{:?}", assembled.blockers);
    assert!(exploded.blockers.is_empty(), "{:?}", exploded.blockers);
    assert!(same(&origin_z(&assembled, insert_id), &Real::from(10)));
    assert!(same(&origin_z(&assembled, screw_id), &Real::from(10)));
    assert!(same(&origin_z(&exploded, insert_id), &Real::from(26)));
    assert!(same(&origin_z(&exploded, screw_id), &Real::from(28)));
    assert!(exploded.guides.len() >= 2);

    let mut assembled_drawing =
        blueprint_from_posed_mesh(&document, &assembled, BlueprintProjection::Front, false);
    let exploded_drawing =
        blueprint_from_posed_mesh(&document, &exploded, BlueprintProjection::Front, false);
    let width = dimension_x(
        Point3::origin(),
        Real::from(30),
        Vector3::new([Real::zero(), Real::from(-6), Real::zero()]),
    )
    .expect("block width is a positive length");
    assert!(same(&width.length, &Real::from(30)));
    annotate(&mut assembled_drawing.assembled, &width, "block");

    let directory = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("target/assembly_manual");
    fs::create_dir_all(&directory).expect("create target/assembly_manual");
    let assembled_name = "0-assembled.svg";
    let exploded_name = "0-exploded.svg";
    let assembled_svg =
        export_blueprint_svg(&assembled_drawing.assembled).expect("assembled svg");
    let exploded_svg = export_blueprint_svg(&exploded_drawing.exploded).expect("exploded svg");
    assert!(assembled_svg.contains("<line "));
    assert!(exploded_svg.contains("<line "));
    fs::write(directory.join(assembled_name), &assembled_svg).expect("write assembled svg");
    fs::write(directory.join(exploded_name), &exploded_svg).expect("write exploded svg");

    let manual = assembly_manual(
        &document,
        |_| exploded_name.to_string(),
        |_| assembled_name.to_string(),
    );
    assert!(
        manual.blockers.is_empty(),
        "instructions are present: {:?}",
        manual.blockers
    );
    assert!(manual.markdown.contains("Press the insert into the block"));
    assert!(manual.markdown.contains(assembled_name));
    assert!(manual.markdown.contains(exploded_name));
    assert!(manual.markdown.contains("block.stl"));
    fs::write(directory.join("manual.md"), &manual.markdown).expect("write manual");
    println!("wrote {}", directory.display());
}

fn build_document() -> (AssemblyDocument, NodeId, NodeId) {
    let mut document = AssemblyDocument::new("Printed block", "main");
    document
        .description
        .push("A printed block with one heat-fit insert and one screw.".into());
    document.node_mut(document.root()).expect("root").instructions =
        "Press the insert into the block, then fit the screw.".into();

    let mut block = AssemblyNode::printed(
        "block.stl",
        "Printed block",
        solid::cuboid(Real::from(30), Real::from(12), Real::from(10)),
    );
    let hole = frame("insert-hole", 8, 6, 10);
    let screw_hole = frame("screw-hole", 22, 6, 10);
    block.attachments.push(hole.clone());
    block.attachments.push(screw_hole.clone());
    document.insert(document.root(), block).expect("block");

    let insert_seat = frame("insert-seat", 0, 0, 0);
    let mut insert = AssemblyNode::vitamin(
        "insert(M3)",
        "insert(M3): Heatfit insert M3",
        solid::cuboid(Real::from(4), Real::from(4), Real::from(6)).translated(
            Real::from(-2),
            Real::from(-2),
            Real::from(-6),
        ),
    );
    insert.placement = mate(&insert_seat, &hole).expect("insert mates onto the block");
    insert.attachments.push(insert_seat);
    insert.explode = guide_offset(8, 6, 10, 0, 0, 16);
    let insert_id = document.insert(document.root(), insert).expect("insert");

    let screw_seat = frame("screw-seat", 0, 0, 0);
    let mut screw = AssemblyNode::vitamin(
        "screw(M3)",
        "screw(M3): Screw M3 cap x 12mm",
        solid::cuboid(Real::from(3), Real::from(3), Real::from(12)).translated(
            Real::from(-1),
            Real::from(-1),
            Real::zero(),
        ),
    );
    screw.placement = mate(&screw_seat, &screw_hole).expect("screw mates onto the block");
    screw.attachments.push(screw_seat);
    screw.explode = guide_offset(22, 6, 10, 0, 0, 18);
    let screw_id = document.insert(document.root(), screw).expect("screw");
    (document, insert_id, screw_id)
}

fn frame(handle: &str, x: i64, y: i64, z: i64) -> AttachmentFrame {
    AttachmentFrame::try_new(
        handle,
        Point3::new(Real::from(x), Real::from(y), Real::from(z)),
        Vector3::new([Real::zero(), Real::zero(), Real::one()]),
        None,
        AttachmentRole::Mounting,
    )
    .expect("attachment axis is +Z")
}

fn guide_offset(x: i64, y: i64, z: i64, dx: i64, dy: i64, dz: i64) -> ExplodeSpec {
    let mut spec = ExplodeSpec::translation(Vector3::new([
        Real::from(dx),
        Real::from(dy),
        Real::from(dz),
    ]));
    spec.guide_origin = Point3::new(Real::from(x), Real::from(y), Real::from(z));
    spec
}

fn origin_z(posed: &PosedAssembly, id: NodeId) -> Real {
    let part = posed
        .parts
        .iter()
        .find(|part| part.node == id)
        .expect("part is posed");
    part.world
        .transform_point3(&Point3::origin())
        .expect("origin transforms")
        .z
}

fn same(left: &Real, right: &Real) -> bool {
    hyperlimit::compare_reals(left, right, hyperlimit::PredicatePolicy::STRICT).value()
        == Some(std::cmp::Ordering::Equal)
}
