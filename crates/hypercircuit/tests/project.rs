#![cfg(feature = "layout")]

use hypercircuit::{
    BoardOutline, BoardSide, Design, DesignModule, Footprint, LayoutTransform, ModuleBuildError,
    PcbStackup, PlacementRule, PortDirection, Real, parts,
};
use hyperlattice::Point2;
use hyperpath::TraceLayer;

fn point(x: i64, y: i64) -> Point2 {
    Point2::new(Real::from(x), Real::from(y))
}

#[test]
fn nested_layout_transforms_compose_mirroring_and_orientation_exactly() {
    let parent = LayoutTransform {
        position: point(10, 4),
        rotation_degrees: Real::zero(),
        side: BoardSide::Back,
    };
    let child = LayoutTransform {
        position: point(2, 3),
        rotation_degrees: Real::zero(),
        side: BoardSide::Back,
    };

    let composed = parent.compose(&child);
    assert_eq!(composed.position, point(8, 7));
    assert_eq!(composed.rotation_degrees, Real::zero());
    assert_eq!(composed.side, BoardSide::Front);
    assert_eq!(
        composed.transform_point(&point(5, 7)),
        parent.transform_point(&child.transform_point(&point(5, 7)))
    );

    let rotated = LayoutTransform {
        rotation_degrees: Real::from(30),
        ..parent
    }
    .compose(&LayoutTransform {
        rotation_degrees: Real::from(10),
        ..child
    });
    assert_eq!(rotated.rotation_degrees, Real::from(20));
    assert_eq!(rotated.side, BoardSide::Front);
}

#[test]
fn recursive_checked_modules_compile_through_circuit_and_layout_hierarchy() {
    let mut leaf = Design::new(
        "leaf-circuit",
        BoardOutline::rectangle(Real::from(30), Real::from(20)),
        PcbStackup::single_layer(Real::one(), None),
    )
    .unwrap();
    let leaf_input = leaf.signal("IN").unwrap();
    let leaf_ground = leaf.ground("GND").unwrap();
    let leaf_input_port = leaf
        .port("input", &leaf_input, PortDirection::Input, false)
        .unwrap();
    let leaf_ground_port = leaf
        .port("ground", &leaf_ground, PortDirection::Ground, false)
        .unwrap();
    let resistor = leaf
        .add(
            parts::resistor("R1", Real::from(1_000))
                .footprint(Footprint::two_pad_smd(
                    Real::one(),
                    Real::one(),
                    Real::from(2),
                    vec![TraceLayer(0)],
                ))
                .at(point(1, 1)),
        )
        .unwrap();
    leaf.connect(&leaf_input, [resistor.pin("1").unwrap()])
        .unwrap();
    leaf.connect(&leaf_ground, [resistor.pin("2").unwrap()])
        .unwrap();
    leaf.constrain(PlacementRule::fixed("leaf-origin", &resistor, point(3, 2)))
        .unwrap();
    let leaf = DesignModule::new("leaf-layout", leaf.finish().unwrap()).unwrap();

    let mut middle_design = Design::new(
        "middle-circuit",
        BoardOutline::rectangle(Real::from(30), Real::from(20)),
        PcbStackup::single_layer(Real::one(), None),
    )
    .unwrap();
    let middle_input = middle_design.signal("IN").unwrap();
    let middle_ground = middle_design.ground("GND").unwrap();
    let middle_input_port = middle_design
        .port("input", &middle_input, PortDirection::Input, false)
        .unwrap();
    let middle_ground_port = middle_design
        .port("ground", &middle_ground, PortDirection::Ground, false)
        .unwrap();
    let mut middle = DesignModule::new("middle-layout", middle_design.finish().unwrap()).unwrap();
    middle
        .instantiate(
            "leaf",
            leaf,
            [
                (&leaf_input_port, &middle_input),
                (&leaf_ground_port, &middle_ground),
            ],
            LayoutTransform {
                position: point(2, 0),
                ..LayoutTransform::default()
            },
        )
        .unwrap();

    let mut root_design = Design::new(
        "root-circuit",
        BoardOutline::rectangle(Real::from(30), Real::from(20)),
        PcbStackup::single_layer(Real::one(), None),
    )
    .unwrap();
    let root_input = root_design.signal("IN").unwrap();
    let root_ground = root_design.ground("GND").unwrap();
    let mut root = DesignModule::new("root-layout", root_design.finish().unwrap()).unwrap();
    root.instantiate(
        "middle",
        middle,
        [
            (&middle_input_port, &root_input),
            (&middle_ground_port, &root_ground),
        ],
        LayoutTransform {
            position: point(10, 0),
            ..LayoutTransform::default()
        },
    )
    .unwrap();

    let project = root.compile().unwrap();
    assert!(project.circuits.validate().is_valid());
    assert_eq!(project.layouts.modules.len(), 2);
    assert_eq!(project.layouts.instances.len(), 2);
    assert_eq!(project.composed.modules.len(), 2);
    assert_eq!(project.composed.modules[1].placement_constraints, 1);
    assert_eq!(project.composed.layout.placements.len(), 1);
    let placement = &project.composed.layout.placements[0];
    assert_eq!(placement.instance.as_str(), "middle/leaf/R1");
    assert_eq!(placement.position, point(15, 2));
    assert_eq!(project.layouts.modules[0].placement_constraints.len(), 1);
    assert!(
        project
            .composed
            .circuit
            .instances
            .iter()
            .any(|instance| instance.id.as_str() == "middle/leaf/R1")
    );
    assert_eq!(project.schematics.len(), 3);
    assert_eq!(project.sources.len(), 3);
}

#[test]
fn module_bindings_reject_missing_duplicate_and_foreign_interfaces_atomically() {
    let mut child_design = Design::new(
        "child",
        BoardOutline::rectangle(Real::from(10), Real::from(10)),
        PcbStackup::single_layer(Real::one(), None),
    )
    .unwrap();
    let child_net = child_design.signal("IN").unwrap();
    let child_port = child_design
        .port("input", &child_net, PortDirection::Input, false)
        .unwrap();
    let child = DesignModule::new("child-layout", child_design.finish().unwrap()).unwrap();

    let mut parent_design = Design::new(
        "parent",
        BoardOutline::rectangle(Real::from(10), Real::from(10)),
        PcbStackup::single_layer(Real::one(), None),
    )
    .unwrap();
    let parent_net = parent_design.signal("IN").unwrap();
    let mut parent = DesignModule::new("parent-layout", parent_design.finish().unwrap()).unwrap();

    assert!(matches!(
        parent.instantiate(
            "missing",
            child.clone(),
            std::iter::empty(),
            LayoutTransform::default()
        ),
        Err(ModuleBuildError::MissingRequiredPort(port)) if port == "input"
    ));
    assert!(parent.instances.is_empty());
    assert!(matches!(
        parent.instantiate(
            "duplicate",
            child.clone(),
            [(&child_port, &parent_net), (&child_port, &parent_net)],
            LayoutTransform::default()
        ),
        Err(ModuleBuildError::DuplicatePortBinding(port)) if port == "input"
    ));
    assert!(parent.instances.is_empty());

    let mut foreign_design = Design::new(
        "parent",
        BoardOutline::rectangle(Real::from(10), Real::from(10)),
        PcbStackup::single_layer(Real::one(), None),
    )
    .unwrap();
    let foreign_net = foreign_design.signal("IN").unwrap();
    assert_eq!(
        parent.instantiate(
            "foreign",
            child,
            [(&child_port, &foreign_net)],
            LayoutTransform::default()
        ),
        Err(ModuleBuildError::ForeignParentNet)
    );
    assert!(parent.instances.is_empty());
}
