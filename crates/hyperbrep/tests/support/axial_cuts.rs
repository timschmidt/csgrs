//! Axial intersection proofs and complete touching/disjoint-tool continuations.
//! Partial-depth results retain whole-shell admission through cold replay.
use hyperbrep::boolean::{BooleanOperation, FacePairRelation, FacePairTrim};
use hyperbrep::*;

#[allow(dead_code)]
#[path = "closed_cylinder.rs"]
mod closed_cylinder;

pub fn p(x: i32, y: i32, z: i32) -> Point3 {
    Point3::new(x.into(), y.into(), z.into())
}
pub fn equal(a: &Real, b: &Real) {
    assert_eq!(
        hyperlimit::compare_reals(a, b, STRICT_PREDICATES).value(),
        Some(std::cmp::Ordering::Equal)
    );
}
pub fn replay(model: &Model) -> Model {
    let text = model.to_json().unwrap();
    let result = RawModel::from_json(&text, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(result.to_json().unwrap(), text);
    result
}
fn incidence(model: &Model) {
    for edge in model.edges() {
        let uses = edge.uses().collect::<Vec<_>>();
        assert_eq!(uses.len(), 2);
        assert_ne!(uses[0].direction(), uses[1].direction());
    }
}

fn check_traces(graph: &boolean::SolidIntersectionGraph) -> usize {
    assert_eq!(graph.unsupported_pairs(), 0);
    assert_eq!(graph.unresolved_trim_pairs(), 0);
    let mut axial = 0;
    for pair in graph.intersections() {
        if !matches!(
            pair.relation(),
            FacePairRelation::Exact(
                SurfaceSurfaceIntersection::Line(_) | SurfaceSurfaceIntersection::Lines(_)
            )
        ) || ![
            pair.first_face().surface().kind(),
            pair.second_face().surface().kind(),
        ]
        .contains(&SurfaceKind::Cylinder)
        {
            continue;
        }
        let fragments = match pair.trim() {
            FacePairTrim::SurfaceCurveFragments(fragments) => fragments,
            FacePairTrim::NoCurveInterior => continue,
            other => panic!("axial trim is not certified: {other:?}"),
        };
        axial += fragments.len();
        for trace in fragments {
            assert_eq!(trace.curve().kind(), Curve3Kind::Line);
            let domain = trace.curve().domain();
            for parameter in [
                domain.represented_start().unwrap().clone(),
                ((domain.represented_start().unwrap() + domain.represented_end().unwrap())
                    / Real::from(2))
                .unwrap(),
                domain.represented_end().unwrap().clone(),
            ] {
                let point = trace.curve().point_at(&parameter.clone().into()).unwrap();
                for (face, pcurve) in [
                    (pair.first_face(), trace.first_pcurve()),
                    (pair.second_face(), trace.second_pcurve()),
                ] {
                    let uv = pcurve.point_at(&parameter.clone().into()).unwrap();
                    let uv = uv.coordinates().expect("represented axial pcurve fixture");
                    let uv = hyperbrep::Point2::new(uv.x().clone(), uv.y().clone());
                    assert_eq!(
                        hyperlimit::point3_equal(
                            point.try_coordinates().unwrap(),
                            &face
                                .surface()
                                .geometry()
                                .point_at(&{
                                    let uv = &uv;
                                    hypercurve::Point2::new(uv.x.clone(), uv.y.clone()).into()
                                })
                                .map(|point| point
                                    .try_coordinates()
                                    .expect("represented surface fixture")
                                    .clone())
                                .unwrap(),
                            STRICT_PREDICATES
                        )
                        .value(),
                        Some(true)
                    );
                }
            }
        }
    }
    axial
}

#[allow(dead_code)]
#[path = "axial_shells.rs"]
mod axial_shells;

pub fn partial_stages() {
    let delta: Real = "1e-12".parse().unwrap();
    for (kind, (lower, upper)) in [
        ((-20).into(), Real::from(6) - &delta),
        (delta.clone(), 20.into()),
        (2.into(), 4.into()),
    ]
    .into_iter()
    .enumerate()
    {
        let source = builder::cylinder(5.into(), 6.into()).unwrap();
        let tool = builder::cuboid(
            Point3::new(0.into(), (-20).into(), lower.clone()),
            Point3::new(20.into(), 20.into(), upper.clone()),
        )
        .unwrap();
        let graph = boolean::intersection_graph(
            source.bodies().next().unwrap(),
            tool.bodies().next().unwrap(),
        )
        .unwrap();
        assert!(check_traces(&graph) >= 2);
        let first = graph.partition_first_faces().unwrap();
        let second = graph.partition_second_faces().unwrap();
        for (partition, volume, area, is_tool) in [
            (
                &first,
                Real::from(150) * Real::pi(),
                Real::from(110) * Real::pi(),
                false,
            ),
            (
                &second,
                Real::from(800) * (&upper - &lower),
                Real::from(1600) + Real::from(120) * (&upper - &lower),
                true,
            ),
        ] {
            for model in [partition.model().clone(), replay(partition.model())] {
                equal(&model.solids().next().unwrap().volume().unwrap(), &volume);
                equal(&model.faces().map(|face| face.area().unwrap()).sum(), &area);
                incidence(&model);
                if is_tool {
                    use SolidPointLocation::{Boundary, Inside, Outside};
                    let middle = ((&lower + &upper) / Real::from(2)).unwrap();
                    for (point, expected) in [
                        (Point3::new(1.into(), 0.into(), middle.clone()), Inside),
                        (Point3::new(0.into(), 0.into(), middle.clone()), Boundary),
                        (Point3::new(-&delta, 0.into(), middle), Outside),
                        (Point3::new(1.into(), 0.into(), &upper + &delta), Outside),
                    ] {
                        assert_eq!(
                            model
                                .solids()
                                .next()
                                .unwrap()
                                .classify_point(&point)
                                .unwrap(),
                            expected
                        );
                    }
                    let body = BodySet::from_solid(model.solids().next().unwrap());
                    let continued = boolean::intersection(&body, &tool).unwrap();
                    equal(
                        &continued.bodies().next().unwrap().volume().unwrap(),
                        &volume,
                    );
                    incidence(&replay(continued.model()));
                }
            }
        }
        for selection in [
            graph
                .select_first_faces(BooleanOperation::Difference)
                .unwrap(),
            graph
                .select_second_faces(BooleanOperation::Difference)
                .unwrap(),
        ] {
            assert_eq!(
                selection.faces().len(),
                selection.partitioned().model().faces().count()
            );
        }
        let result = boolean::difference(&source, &tool).unwrap();
        axial_shells::verify(result.model(), kind, &Matrix4::identity());
        axial_shells::verify(&replay(result.model()), kind, &Matrix4::identity());
        assert_eq!(
            source.model().to_json().unwrap(),
            replay(source.model()).to_json().unwrap()
        );
    }
}

pub fn periodic_traces() {
    for (turns, reversed) in [(-100, false), (100, true)] {
        let model = closed_cylinder::build(Real::zero(), turns, reversed).unwrap();
        let source = model.solids().next().unwrap();
        for x in [Real::zero(), Real::one(), Real::from(2)] {
            let tool =
                builder::cuboid(Point3::new(x, (-20).into(), 1.into()), p(20, 20, 3)).unwrap();
            let tool = tool.bodies().next().unwrap();
            let forward = boolean::intersection_graph(source, tool).unwrap();
            let reverse = boolean::intersection_graph(tool, source).unwrap();
            assert!(check_traces(&forward) > 0);
            assert!(check_traces(&reverse) > 0);
        }
        let delta: Real = "1e-12".parse().unwrap();
        let tool = builder::cuboid(
            Point3::new(Real::from(2) + delta, (-20).into(), 1.into()),
            p(20, 20, 3),
        )
        .unwrap();
        assert_eq!(
            check_traces(
                &boolean::intersection_graph(source, tool.bodies().next().unwrap()).unwrap()
            ),
            0
        );
    }
}

pub fn verify_cylinder(model: &Model, map: &Matrix4) {
    assert_eq!(model.solids().count(), 1);
    equal(
        &model.solids().next().unwrap().volume().unwrap(),
        &(Real::from(150) * Real::pi()),
    );
    equal(
        &model.faces().map(|face| face.area().unwrap()).sum(),
        &(Real::from(110) * Real::pi()),
    );
    incidence(model);
    let delta: Real = "1e-12".parse().unwrap();
    use SolidPointLocation::{Boundary as B, Inside as I, Outside as O};
    for (point, expected) in [
        (p(1, 0, 3), I),
        (p(-1, 0, 3), I),
        (p(0, 0, 6), B),
        (p(5, 0, 3), B),
        (Point3::new(Real::from(5) - &delta, 0.into(), 3.into()), I),
        (Point3::new(Real::from(5) + &delta, 0.into(), 3.into()), O),
        (Point3::new(1.into(), 0.into(), Real::from(6) - &delta), I),
        (Point3::new(1.into(), 0.into(), Real::from(6) + delta), O),
    ] {
        assert_eq!(
            model
                .solids()
                .next()
                .unwrap()
                .classify_point(&map.transform_point3(&point).unwrap())
                .unwrap(),
            expected
        );
    }
}

pub fn unchanged_cut(disjoint: bool) -> BodySet {
    let source = builder::cylinder(5.into(), 6.into()).unwrap();
    let lower = if disjoint {
        Real::from(6) + "1e-12".parse::<Real>().unwrap()
    } else {
        6.into()
    };
    let tool = builder::cuboid(Point3::new(0.into(), (-20).into(), lower), p(20, 20, 20)).unwrap();
    let result = boolean::difference(&source, &tool).unwrap();
    verify_cylinder(result.model(), &Matrix4::identity());
    assert!(boolean::intersection(&source, &tool).unwrap().is_empty());
    let cold = replay(result.model());
    verify_cylinder(&cold, &Matrix4::identity());
    let body = BodySet::from_solid(cold.solids().next().unwrap());
    let cover = builder::cuboid(p(0, -20, -20), p(20, 20, 20)).unwrap();
    let half = boolean::difference(&body, &cover).unwrap();
    let other = boolean::intersection(&body, &cover).unwrap();
    for part in [&half, &other] {
        equal(
            &part.bodies().next().unwrap().volume().unwrap(),
            &(Real::from(75) * Real::pi()),
        );
        equal(
            &part.model().faces().map(|face| face.area().unwrap()).sum(),
            &(Real::from(55) * Real::pi() + Real::from(60)),
        );
        incidence(&replay(part.model()));
    }
    let rebuilt = boolean::union(&half, &other).unwrap();
    verify_cylinder(rebuilt.model(), &Matrix4::identity());
    let empty = boolean::difference(&rebuilt, &body).unwrap();
    assert!(empty.is_empty());
    verify_cylinder(
        boolean::union(&empty, &body).unwrap().model(),
        &Matrix4::identity(),
    );
    body
}
