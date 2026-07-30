#![no_main]

use hyperbrep::{
    Matrix4, RawModel, Real, Surface, SurfaceIntersectionOperand, SurfaceSurfaceIntersection,
    Vector3, boolean, builder,
};
use hypercurve::{Curve2, CurvePath2, LineSeg2, Point2 as CurvePoint2};
use libfuzzer_sys::fuzz_target;

fuzz_target!(|bytes: &[u8]| {
    if bytes.len() < 4 {
        return;
    }
    let positive = |index: usize| Real::from(i32::from(bytes[index]) + 1);
    let built = match bytes[0] % 14 {
        0 => builder::cylinder(positive(1), positive(2)).ok(),
        1 => {
            let top = positive(1);
            let base = &top + positive(2);
            builder::cone_frustum(base, top, positive(3)).ok()
        }
        2 => {
            let minor = positive(1);
            let major = &minor + positive(2);
            builder::torus(major, minor).ok()
        }
        3 => builder::sphere(positive(1)).ok(),
        4 => {
            let radius = positive(1);
            let Ok((first, solid)) = builder::sphere(radius.clone()) else {
                return;
            };
            let Ok(second) = first.transformed(&Matrix4::affine_translation([
                radius,
                Real::zero(),
                Real::zero(),
            ])) else {
                return;
            };
            let result = match bytes[2] % 3 {
                0 => boolean::union(&first, solid, &second, solid),
                1 => boolean::intersection(&first, solid, &second, solid),
                _ => boolean::difference(&first, solid, &second, solid),
            };
            match result {
                Ok(boolean::BooleanResult::Solid { model, solid }) => Some((model, solid)),
                _ => None,
            }
        }
        5 => {
            let radius = positive(1);
            let Ok((first, solid)) = builder::cylinder(radius.clone(), positive(2)) else {
                return;
            };
            let Ok((second, second_solid)) = builder::cylinder(radius, positive(3)) else {
                return;
            };
            let orient = |offset: i32| {
                Matrix4::affine_orthonormal(
                    [
                        [Real::zero(), Real::zero(), Real::one()],
                        [Real::one(), Real::zero(), Real::zero()],
                        [Real::zero(), Real::one(), Real::zero()],
                    ],
                    [Real::from(offset), Real::zero(), Real::zero()],
                )
            };
            let Ok(first) = first.transformed(&orient(0)) else {
                return;
            };
            let Ok(second) = second.transformed(&orient(i32::from(bytes[1] / 2))) else {
                return;
            };
            let result = match bytes[2] % 3 {
                0 => boolean::union(&first, solid, &second, second_solid),
                1 => boolean::intersection(&first, solid, &second, second_solid),
                _ => boolean::difference(&first, solid, &second, second_solid),
            };
            match result {
                Ok(boolean::BooleanResult::Solid { model, solid }) => Some((model, solid)),
                Ok(boolean::BooleanResult::Solids { model, solids }) => {
                    solids.first().copied().map(|solid| (model, solid))
                }
                _ => None,
            }
        }
        6 => {
            let top = positive(1);
            let middle = &top + positive(2);
            let base = &middle + positive(3);
            let Ok((outer, solid)) = builder::cone_frustum(base.clone(), top.clone(), &base - &top)
            else {
                return;
            };
            let Ok((inner, inner_solid)) =
                builder::cone_frustum(middle.clone(), top.clone(), &middle - &top)
            else {
                return;
            };
            let Ok(inner) = inner.transformed(&Matrix4::affine_translation([
                Real::zero(),
                Real::zero(),
                &base - &middle,
            ])) else {
                return;
            };
            let result = match bytes[1] % 3 {
                0 => boolean::union(&outer, solid, &inner, inner_solid),
                1 => boolean::intersection(&outer, solid, &inner, inner_solid),
                _ => boolean::difference(&outer, solid, &inner, inner_solid),
            };
            match result {
                Ok(boolean::BooleanResult::Solid { model, solid }) => Some((model, solid)),
                Ok(boolean::BooleanResult::Solids { model, solids }) => {
                    solids.first().copied().map(|solid| (model, solid))
                }
                _ => None,
            }
        }
        7 => {
            let inner = positive(1);
            let outer = &inner + positive(2);
            let height = positive(3);
            builder::revolve(&[
                hyperbrep::Point2::new(inner.clone(), Real::zero()),
                hyperbrep::Point2::new(outer.clone(), Real::zero()),
                hyperbrep::Point2::new(outer, height.clone()),
                hyperbrep::Point2::new(inner, height),
            ])
            .ok()
        }
        8 => {
            let radial_offset = positive(1);
            let overlap_width = positive(2);
            let overlap_height = positive(3);
            let first_outer = Real::one() + &radial_offset + &overlap_width;
            let first_top = Real::one() + &overlap_height;
            let Ok((first, solid)) = builder::revolve(&[
                hyperbrep::Point2::new(Real::one(), Real::zero()),
                hyperbrep::Point2::new(first_outer.clone(), Real::zero()),
                hyperbrep::Point2::new(first_outer.clone(), first_top.clone()),
                hyperbrep::Point2::new(Real::one(), first_top),
            ]) else {
                return;
            };
            let second_inner = Real::one() + radial_offset;
            let second_outer = &first_outer + Real::one();
            let second_top = &overlap_height + Real::from(2);
            let Ok((second, second_solid)) = builder::revolve(&[
                hyperbrep::Point2::new(second_inner.clone(), Real::one()),
                hyperbrep::Point2::new(second_outer.clone(), Real::one()),
                hyperbrep::Point2::new(second_outer, second_top.clone()),
                hyperbrep::Point2::new(second_inner, second_top),
            ]) else {
                return;
            };
            let result = match bytes[1] % 3 {
                0 => boolean::union(&first, solid, &second, second_solid),
                1 => boolean::intersection(&first, solid, &second, second_solid),
                _ => boolean::difference(&first, solid, &second, second_solid),
            };
            match result {
                Ok(boolean::BooleanResult::Solid { model, solid }) => Some((model, solid)),
                Ok(boolean::BooleanResult::Solids { model, solids }) => {
                    solids.first().copied().map(|solid| (model, solid))
                }
                _ => None,
            }
        }
        9 => {
            let width = positive(1);
            let depth = positive(2);
            let height = positive(3);
            builder::sweep(
                &[
                    hyperbrep::Point2::new(Real::zero(), Real::zero()),
                    hyperbrep::Point2::new(width.clone(), Real::zero()),
                    hyperbrep::Point2::new(width, depth.clone()),
                    hyperbrep::Point2::new(Real::zero(), depth),
                ],
                hyperbrep::Point3::origin(),
                hyperbrep::Vector3::from_xyz(Real::from(2), Real::zero(), Real::zero()),
                hyperbrep::Vector3::from_xyz(Real::zero(), Real::from(3), Real::zero()),
                hyperbrep::Vector3::from_xyz(Real::one(), Real::zero(), height),
            )
            .ok()
        }
        10 => {
            let width = positive(1);
            let depth = positive(2);
            let height = positive(3);
            let quarter_height = (&height / Real::from(4)).expect("four is a nonzero denominator");
            let Ok(path) = hyperbrep::Curve3::rational_bezier(
                vec![
                    hyperbrep::Point3::origin(),
                    hyperbrep::Point3::new(
                        Real::from(i32::from(bytes[1]) - 128),
                        Real::from(i32::from(bytes[2]) - 128),
                        quarter_height,
                    ),
                    hyperbrep::Point3::new(Real::zero(), Real::zero(), height),
                ],
                vec![Real::one(), Real::from(2), Real::from(3)],
            ) else {
                return;
            };
            builder::sweep_curve(
                &[
                    hyperbrep::Point2::new(Real::zero(), Real::zero()),
                    hyperbrep::Point2::new(width.clone(), Real::zero()),
                    hyperbrep::Point2::new(width, depth.clone()),
                    hyperbrep::Point2::new(Real::zero(), depth),
                ],
                Vector3::x(),
                Vector3::y(),
                path,
            )
            .ok()
        }
        11 => {
            let minor = positive(1);
            let major = &minor + positive(2);
            let center = hypercurve::Point2::new(major.clone(), Real::zero());
            let right = hypercurve::Point2::new(&major + &minor, Real::zero());
            let left = hypercurve::Point2::new(&major - &minor, Real::zero());
            let Ok(profile) = hypercurve::Contour2::try_new(vec![
                hypercurve::Segment2::Arc(
                    hypercurve::CircularArc2::try_from_center(
                        right.clone(),
                        left.clone(),
                        center.clone(),
                        false,
                    )
                    .expect("positive toroidal profile is a circle"),
                ),
                hypercurve::Segment2::Arc(
                    hypercurve::CircularArc2::try_from_center(left, right, center, false)
                        .expect("positive toroidal profile is a circle"),
                ),
            ]) else {
                return;
            };
            builder::revolve_contour(&profile).ok()
        }
        12 => {
            let width = positive(1);
            let depth = positive(2);
            let height = positive(3);
            let Ok(frame) = hyperbrep::RationalBezierSweepFrame::try_new(
                vec![
                    hyperbrep::Point3::origin(),
                    hyperbrep::Point3::new(Real::zero(), Real::zero(), height),
                ],
                vec![
                    Vector3::x(),
                    Vector3::from_xyz(positive(1), Real::zero(), Real::zero()),
                ],
                vec![
                    Vector3::y(),
                    Vector3::from_xyz(Real::zero(), positive(2), Real::zero()),
                ],
                vec![Real::one(), Real::one()],
            ) else {
                return;
            };
            builder::sweep_moving_frame(
                &[
                    hyperbrep::Point2::new(Real::zero(), Real::zero()),
                    hyperbrep::Point2::new(width.clone(), Real::zero()),
                    hyperbrep::Point2::new(width, depth.clone()),
                    hyperbrep::Point2::new(Real::zero(), depth),
                ],
                frame,
            )
            .ok()
        }
        _ => {
            let width = positive(1);
            let depth = positive(2);
            let scale = positive(3);
            let upper_width = &width * &scale;
            let upper_depth = &depth * &scale;
            let upper_top_right = if bytes[0].is_multiple_of(2) {
                &upper_width + Real::one()
            } else {
                (&upper_width / Real::from(2)).expect("two is nonzero") + Real::one()
            };
            let upper_profile = vec![
                hyperbrep::Point2::new(Real::one(), Real::one()),
                hyperbrep::Point2::new(&upper_width + Real::one(), Real::one()),
                hyperbrep::Point2::new(upper_top_right, &upper_depth + Real::one()),
                hyperbrep::Point2::new(Real::one(), upper_depth + Real::one()),
            ];
            let mut sections = vec![
                hyperbrep::LoftSection {
                    profile: vec![
                        hyperbrep::Point2::new(Real::zero(), Real::zero()),
                        hyperbrep::Point2::new(width.clone(), Real::zero()),
                        hyperbrep::Point2::new(width, depth.clone()),
                        hyperbrep::Point2::new(Real::zero(), depth),
                    ],
                    z: Real::zero(),
                },
                hyperbrep::LoftSection {
                    profile: upper_profile.clone(),
                    z: Real::from(2),
                },
            ];
            if bytes[1].is_multiple_of(2) {
                sections.push(hyperbrep::LoftSection {
                    profile: upper_profile
                        .iter()
                        .map(|point| {
                            hyperbrep::Point2::new(
                                Real::one() + Real::from(2) * &point.x,
                                Real::from(-1) + Real::from(2) * &point.y,
                            )
                        })
                        .collect(),
                    z: Real::from(5),
                });
            }
            builder::loft(&sections).ok()
        }
    };
    let Some((model, solid)) = built else {
        return;
    };
    let _ = model.solid_volume(solid);
    let _ = model.classify_point(solid, &hyperbrep::Point3::origin());
    let _ = boolean::intersection_graph(&model, solid, &model, solid);
    let chordal_policy = hyperbrep::tessellation::ChordalApproximationPolicy::uniform(
        std::num::NonZeroUsize::new(usize::from(bytes[1] % 3) + 1)
            .expect("fuzz boundary subdivision is positive"),
        bytes[2] % 3,
    );
    if let Some(face) = model
        .solid(solid)
        .and_then(|solid| model.shell(solid.outer()))
        .and_then(|shell| {
            shell
                .faces()
                .get(usize::from(bytes[3]) % shell.faces().len())
        })
        && let Ok(artifact) =
            hyperbrep::tessellation::approximate_face_chordally(&model, *face, chordal_policy)
    {
        assert_eq!(artifact.parameters().len(), artifact.points().len());
        for index in 0..artifact.parameters().len() {
            assert!(
                artifact
                    .triangles()
                    .iter()
                    .flatten()
                    .any(|retained| *retained == index)
            );
        }
    }
    let translation = Matrix4::affine_translation([
        Real::from(i32::from(bytes[1]) - 128),
        Real::from(i32::from(bytes[2]) - 128),
        Real::from(i32::from(bytes[3]) - 128),
    ]);
    let Ok(transformed) = model.transformed(&translation) else {
        return;
    };
    let Ok(json) = transformed.to_json() else {
        return;
    };
    let Ok(raw) = RawModel::from_json(&json) else {
        return;
    };
    let Ok(decoded) = raw.validate() else {
        panic!("validated analytic model failed exact persistence replay");
    };
    let _ = decoded.solid_volume(solid);
    let _ = boolean::intersection_graph(&model, solid, &decoded, solid);

    if bytes[0] % 14 == 1 {
        let Ok((frustum, frustum_solid)) =
            builder::cone_frustum(Real::from(4), Real::one(), Real::from(3))
        else {
            return;
        };
        let three_fifths = (Real::from(3) / Real::from(5)).unwrap();
        let four_fifths = (Real::from(4) / Real::from(5)).unwrap();
        let parameter_rotation = Matrix4::affine_orthonormal(
            [
                [three_fifths.clone(), -four_fifths.clone(), Real::zero()],
                [four_fifths, three_fifths, Real::zero()],
                [Real::zero(), Real::zero(), Real::one()],
            ],
            [Real::zero(), Real::zero(), Real::zero()],
        );
        let Ok(frustum) = frustum.transformed(&parameter_rotation) else {
            return;
        };
        let Ok((cutter, cutter_solid)) = builder::cuboid(
            hyperbrep::Point3::new(Real::zero(), Real::from(-5), Real::from(-1)),
            hyperbrep::Point3::new(Real::from(5), Real::from(5), Real::from(4)),
        ) else {
            return;
        };
        let result = if bytes[1].is_multiple_of(2) {
            boolean::intersection(&frustum, frustum_solid, &cutter, cutter_solid)
        } else {
            boolean::difference(&frustum, frustum_solid, &cutter, cutter_solid)
        };
        let Ok(boolean::BooleanResult::Solid { model, solid }) = result else {
            panic!("certified axial frustum Boolean did not retain one solid");
        };
        let expected = (Real::from(21) * Real::pi() / Real::from(2)).unwrap();
        assert_eq!(
            hyperlimit::compare_reals(&model.solid_volume(solid).unwrap(), &expected).value(),
            Some(std::cmp::Ordering::Equal)
        );
        let json = model.to_json().unwrap();
        assert!(
            json.len() < 100_000,
            "axial frustum result retained noncanonical pcurve expressions"
        );
        let replayed = RawModel::from_json(&json).unwrap().validate().unwrap();
        assert_eq!(replayed.to_json().unwrap(), json);
    }

    let radius = positive(1);
    let Ok((sphere, sphere_solid)) = builder::sphere(radius.clone()) else {
        return;
    };
    let Ok(half_radius) = &radius / Real::from(2) else {
        return;
    };
    let extent = &radius * Real::from(2);
    let Ok((trim_box, trim_box_solid)) = builder::cuboid(
        hyperbrep::Point3::new(half_radius.clone(), half_radius, -extent.clone()),
        hyperbrep::Point3::new(extent.clone(), extent.clone(), extent),
    ) else {
        return;
    };
    let _ = boolean::intersection_graph(&sphere, sphere_solid, &trim_box, trim_box_solid);

    let width = positive(1);
    let depth = positive(2);
    let height = positive(3);
    let Ok((first_box, first_box_solid)) = builder::cuboid(
        hyperbrep::Point3::origin(),
        hyperbrep::Point3::new(width.clone(), depth.clone(), height.clone()),
    ) else {
        return;
    };
    let gap_start = &width + Real::one();
    let Ok((second_box, second_box_solid)) = builder::cuboid(
        hyperbrep::Point3::new(gap_start.clone(), Real::zero(), Real::zero()),
        hyperbrep::Point3::new(gap_start + &width, depth.clone(), height.clone()),
    ) else {
        return;
    };
    let Ok(graph) =
        boolean::intersection_graph(&first_box, first_box_solid, &second_box, second_box_solid)
    else {
        return;
    };
    let operation = match bytes[0] % 3 {
        0 => boolean::BooleanOperation::Union,
        1 => boolean::BooleanOperation::Intersection,
        _ => boolean::BooleanOperation::Difference,
    };
    let _ = graph.select_first_faces(operation);
    let _ = graph.select_second_faces(operation);

    let half_width = (&width / Real::from(2)).unwrap();
    let half_depth = (&depth / Real::from(2)).unwrap();
    let Ok((overlap_box, overlap_box_solid)) = builder::cuboid(
        hyperbrep::Point3::new(half_width.clone(), half_depth.clone(), Real::zero()),
        hyperbrep::Point3::new(&width + half_width, &depth + half_depth, height.clone()),
    ) else {
        return;
    };
    let Ok(overlap_graph) =
        boolean::intersection_graph(&first_box, first_box_solid, &overlap_box, overlap_box_solid)
    else {
        return;
    };
    let _ = overlap_graph.stitch_selected_faces(operation);

    let three_fifths = (Real::from(3) / Real::from(5)).unwrap();
    let four_fifths = (Real::from(4) / Real::from(5)).unwrap();
    let quarter_width = (&width / Real::from(4)).unwrap();
    let half_depth = (&depth / Real::from(2)).unwrap();
    let quarter_height = (&height / Real::from(4)).unwrap();
    let skew = Matrix4::affine_orthonormal(
        [
            [Real::one(), Real::zero(), Real::zero()],
            [Real::zero(), three_fifths.clone(), -four_fifths.clone()],
            [Real::zero(), four_fifths, three_fifths],
        ],
        [quarter_width, half_depth, quarter_height],
    );
    let Ok(skew_box) = first_box.transformed(&skew) else {
        return;
    };
    let skew_result = match operation {
        boolean::BooleanOperation::Union => {
            boolean::union(&first_box, first_box_solid, &skew_box, first_box_solid)
        }
        boolean::BooleanOperation::Intersection => {
            boolean::intersection(&first_box, first_box_solid, &skew_box, first_box_solid)
        }
        boolean::BooleanOperation::Difference => {
            boolean::difference(&first_box, first_box_solid, &skew_box, first_box_solid)
        }
    };
    let skew_model = match skew_result {
        Ok(boolean::BooleanResult::Solid { model, .. })
        | Ok(boolean::BooleanResult::Solids { model, .. }) => model,
        Ok(boolean::BooleanResult::Empty) | Err(_) => return,
    };
    let Ok(json) = skew_model.to_json() else {
        return;
    };
    let Ok(raw) = RawModel::from_json(&json) else {
        return;
    };
    if raw.validate().is_err() {
        panic!("validated skew planar Boolean failed exact persistence replay");
    }

    let Ok((void_outer, void_outer_solid)) = builder::cuboid(
        hyperbrep::Point3::origin(),
        hyperbrep::Point3::new(Real::from(2), Real::from(2), Real::from(2)),
    ) else {
        return;
    };
    let Ok((void_inner, void_inner_solid)) = builder::cuboid(
        hyperbrep::Point3::origin(),
        hyperbrep::Point3::new(Real::one(), Real::one(), Real::one()),
    ) else {
        return;
    };
    let fraction = |numerator: i32| {
        (Real::from(numerator) / Real::from(25)).expect("nonzero rational denominator")
    };
    let void_transform = Matrix4::affine_orthonormal(
        [
            [fraction(9), fraction(-12), fraction(20)],
            [fraction(20), fraction(15), Real::zero()],
            [fraction(-12), fraction(16), fraction(15)],
        ],
        [
            fraction(16),
            fraction(5),
            fraction(if bytes[3].is_multiple_of(2) { 13 } else { 12 }),
        ],
    );
    let Ok(void_inner) = void_inner.transformed(&void_transform) else {
        return;
    };
    if let Ok(boolean::BooleanResult::Solid { model, .. }) =
        boolean::difference(&void_outer, void_outer_solid, &void_inner, void_inner_solid)
    {
        let Ok(json) = model.to_json() else {
            return;
        };
        let Ok(raw) = RawModel::from_json(&json) else {
            return;
        };
        if raw.validate().is_err() {
            panic!("validated skew planar void failed exact persistence replay");
        }
    }

    let Ok(tensor) = Surface::rational_bezier(
        vec![
            vec![
                hyperbrep::Point3::new(Real::zero(), Real::zero(), Real::zero()),
                hyperbrep::Point3::new(Real::one(), Real::from(2), Real::zero()),
                hyperbrep::Point3::new(Real::from(2), Real::zero(), Real::zero()),
            ],
            vec![
                hyperbrep::Point3::new(Real::zero(), Real::zero(), Real::from(2)),
                hyperbrep::Point3::new(Real::one(), Real::from(2), Real::from(2)),
                hyperbrep::Point3::new(Real::from(2), Real::zero(), Real::from(2)),
            ],
        ],
        vec![
            vec![Real::one(), Real::from(2), Real::one()],
            vec![Real::one(), Real::from(2), Real::one()],
        ],
    ) else {
        return;
    };
    let Ok(plane) = Surface::plane(
        hyperbrep::Point3::new(Real::zero(), Real::zero(), Real::from(bytes[0] % 4)),
        Vector3::x(),
        Vector3::y(),
    ) else {
        return;
    };
    if let Ok(SurfaceSurfaceIntersection::Curve(curve)) = tensor.intersect_surface(&plane) {
        let parameter = (Real::one() / Real::from(2)).expect("nonzero rational denominator");
        let _ = curve.curve().point_at(&parameter);
        let _ = curve.first_pcurve().point_at(&parameter);
        let _ = curve.second_pcurve().point_at(&parameter);
    }
    let Ok(curved_translation_tensor) = Surface::rational_bezier(
        vec![
            vec![
                hyperbrep::Point3::origin(),
                hyperbrep::Point3::new(Real::from(2), Real::zero(), Real::zero()),
            ],
            vec![
                hyperbrep::Point3::new(Real::zero(), Real::from(2), Real::one()),
                hyperbrep::Point3::new(Real::from(2), Real::from(2), Real::one()),
            ],
            vec![
                hyperbrep::Point3::new(Real::zero(), Real::from(2), Real::from(2)),
                hyperbrep::Point3::new(Real::from(2), Real::from(2), Real::from(2)),
            ],
        ],
        vec![
            vec![Real::one(), Real::one()],
            vec![Real::from(2), Real::from(2)],
            vec![Real::from(3), Real::from(3)],
        ],
    ) else {
        return;
    };
    let Ok(oblique_plane) = Surface::plane(
        hyperbrep::Point3::new(
            Real::from(i32::from(bytes[0] % 7) - 1),
            Real::zero(),
            Real::zero(),
        ),
        Vector3::y(),
        Vector3::from_xyz(Real::one(), Real::zero(), -Real::one()),
    ) else {
        return;
    };
    if let Ok(SurfaceSurfaceIntersection::Curve(curve)) =
        curved_translation_tensor.intersect_surface(&oblique_plane)
    {
        let parameter = (Real::one() / Real::from(2)).expect("nonzero rational denominator");
        let _ = curve.curve().point_at(&parameter);
        let _ = curve.first_pcurve().materialize();
        let _ = curve.second_pcurve().materialize();
    }
    let bilinear_controls = vec![
        vec![
            hyperbrep::Point3::origin(),
            hyperbrep::Point3::new(Real::from(2), Real::zero(), Real::zero()),
        ],
        vec![
            hyperbrep::Point3::new(Real::zero(), Real::from(2), Real::zero()),
            hyperbrep::Point3::new(Real::from(2), Real::from(2), Real::one()),
        ],
    ];
    let bilinear_weights = vec![
        vec![Real::one(), Real::from(2)],
        vec![Real::from(3), Real::from(4)],
    ];
    let Ok(rational_bilinear_tensor) =
        Surface::rational_bezier(bilinear_controls.clone(), bilinear_weights.clone())
    else {
        return;
    };
    let section_offset = i32::from(bytes[1] % 7);
    let Ok(bilinear_plane) = Surface::plane(
        hyperbrep::Point3::new(Real::from(section_offset), Real::zero(), Real::zero()),
        Vector3::y(),
        Vector3::from_xyz(Real::from(2), Real::zero(), -Real::one()),
    ) else {
        return;
    };
    if let Ok(SurfaceSurfaceIntersection::Curve(curve)) =
        rational_bilinear_tensor.intersect_surface(&bilinear_plane)
    {
        let parameter = (Real::one() / Real::from(2)).expect("nonzero rational denominator");
        let _ = curve.curve().point_at(&parameter);
        let _ = curve.first_pcurve().point_at(&parameter);
        let _ = curve.second_pcurve().point_at(&parameter);
        if matches!(section_offset, 1 | 3) {
            let Ok((patch, face)) =
                builder::rational_bezier_patch(bilinear_controls, bilinear_weights)
            else {
                return;
            };
            if let Ok((split, _)) =
                patch.split_face_by_surface_curve(face, curve.curve(), curve.first_pcurve())
            {
                let Ok(json) = split.to_json() else {
                    return;
                };
                let Ok(replayed) = RawModel::from_json(&json).and_then(RawModel::validate) else {
                    panic!("validated weighted bilinear split failed persistence replay");
                };
                if replayed.to_json().ok().as_deref() != Some(json.as_str()) {
                    panic!("weighted bilinear split replay changed exact persistence");
                }
            }
        }
    }
    let rational = |numerator: i32, denominator: i32| {
        (Real::from(numerator) / Real::from(denominator)).expect("fixed nonzero fuzz denominator")
    };
    let plane_values = match bytes[2] % 4 {
        0 => [
            [rational(3, 16), rational(-5, 16)],
            [rational(-5, 16), rational(3, 16)],
        ],
        1 => [
            [rational(1, 4), rational(-1, 4)],
            [rational(-1, 4), rational(1, 4)],
        ],
        2 => [[Real::one(), -Real::one()], [Real::one(), Real::one()]],
        _ => [[Real::zero(), Real::one()], [Real::one(), Real::zero()]],
    };
    let pole_weights = vec![
        vec![Real::one(), Real::from(2)],
        vec![Real::from(3), Real::from(4)],
    ];
    let pole_controls = vec![
        vec![
            hyperbrep::Point3::new(Real::zero(), Real::zero(), plane_values[0][0].clone()),
            hyperbrep::Point3::new(
                Real::from(2),
                Real::zero(),
                (&plane_values[0][1] / Real::from(2)).expect("fixed nonzero fuzz denominator"),
            ),
        ],
        vec![
            hyperbrep::Point3::new(
                Real::zero(),
                Real::from(2),
                (&plane_values[1][0] / Real::from(3)).expect("fixed nonzero fuzz denominator"),
            ),
            hyperbrep::Point3::new(
                Real::from(2),
                Real::from(2),
                (&plane_values[1][1] / Real::from(4)).expect("fixed nonzero fuzz denominator"),
            ),
        ],
    ];
    let Ok(pole_tensor) = Surface::rational_bezier(pole_controls.clone(), pole_weights.clone())
    else {
        return;
    };
    let Ok(pole_plane) = Surface::plane(hyperbrep::Point3::origin(), Vector3::x(), Vector3::y())
    else {
        return;
    };
    let retained = match pole_tensor.intersect_surface(&pole_plane) {
        Ok(SurfaceSurfaceIntersection::Curve(curve)) => vec![*curve],
        Ok(SurfaceSurfaceIntersection::Curves(curves)) => curves,
        Ok(SurfaceSurfaceIntersection::Point(point)) => {
            let _ = point;
            Vec::new()
        }
        Ok(SurfaceSurfaceIntersection::Points(points)) => {
            let _ = points;
            Vec::new()
        }
        _ => Vec::new(),
    };
    let parameter = (Real::one() / Real::from(2)).expect("nonzero rational denominator");
    for curve in &retained {
        let _ = curve.curve().point_at(&parameter);
        let _ = curve.first_pcurve().point_at(&parameter);
        let _ = curve.second_pcurve().point_at(&parameter);
    }
    if bytes[2] % 4 < 3 && !retained.is_empty() {
        let Ok((patch, face)) = builder::rational_bezier_patch(pole_controls, pole_weights) else {
            return;
        };
        if let Ok((split, _)) =
            patch.split_face_by_surface_curves(face, &retained, SurfaceIntersectionOperand::First)
        {
            let Ok(json) = split.to_json() else {
                return;
            };
            let Ok(replayed) = RawModel::from_json(&json).and_then(RawModel::validate) else {
                panic!("validated pole-branch bilinear split failed persistence replay");
            };
            if replayed.to_json().ok().as_deref() != Some(json.as_str()) {
                panic!("pole-branch bilinear split replay changed exact persistence");
            }
        }
    }
    if bytes[0] % 7 == 3 {
        let Ok((graph_patch, graph_face)) = builder::rational_bezier_patch(
            vec![
                vec![
                    hyperbrep::Point3::origin(),
                    hyperbrep::Point3::new(Real::from(2), Real::zero(), Real::zero()),
                ],
                vec![
                    hyperbrep::Point3::new(Real::zero(), Real::from(2), Real::one()),
                    hyperbrep::Point3::new(Real::from(2), Real::from(2), Real::one()),
                ],
                vec![
                    hyperbrep::Point3::new(Real::zero(), Real::from(2), Real::from(2)),
                    hyperbrep::Point3::new(Real::from(2), Real::from(2), Real::from(2)),
                ],
            ],
            vec![
                vec![Real::one(), Real::one()],
                vec![Real::from(2), Real::from(2)],
                vec![Real::from(3), Real::from(3)],
            ],
        ) else {
            return;
        };
        let graph_surface = graph_patch
            .face(graph_face)
            .and_then(|face| graph_patch.surface(face.surface()))
            .expect("validated graph patch face has a surface");
        if let Ok(SurfaceSurfaceIntersection::Curve(curve)) =
            graph_surface.intersect_surface(&oblique_plane)
            && let Ok((split, _)) = graph_patch.split_face_by_surface_curve(
                graph_face,
                curve.curve(),
                curve.first_pcurve(),
            )
        {
            let Ok(json) = split.to_json() else {
                return;
            };
            let Ok(raw) = RawModel::from_json(&json) else {
                return;
            };
            if raw.validate().is_err() {
                panic!("validated non-isoparametric tensor split failed persistence replay");
            }
        }
        let Ok((nurbs_graph_patch, nurbs_graph_face)) = builder::nurbs_patch(
            1,
            2,
            vec![
                vec![
                    hyperbrep::Point3::origin(),
                    hyperbrep::Point3::new(Real::from(2), Real::zero(), Real::zero()),
                ],
                vec![
                    hyperbrep::Point3::new(Real::zero(), Real::from(2), Real::one()),
                    hyperbrep::Point3::new(Real::from(2), Real::from(2), Real::one()),
                ],
                vec![
                    hyperbrep::Point3::new(Real::zero(), Real::from(2), Real::from(2)),
                    hyperbrep::Point3::new(Real::from(2), Real::from(2), Real::from(2)),
                ],
            ],
            vec![
                vec![Real::one(), Real::one()],
                vec![Real::from(2), Real::from(2)],
                vec![Real::from(3), Real::from(3)],
            ],
            vec![Real::zero(), Real::zero(), Real::one(), Real::one()],
            vec![
                Real::from(2),
                Real::from(2),
                Real::from(2),
                Real::from(5),
                Real::from(5),
                Real::from(5),
            ],
        ) else {
            return;
        };
        let nurbs_graph_surface = nurbs_graph_patch
            .face(nurbs_graph_face)
            .and_then(|face| nurbs_graph_patch.surface(face.surface()))
            .expect("validated NURBS graph patch face has a surface");
        if let Ok(SurfaceSurfaceIntersection::Curve(curve)) =
            nurbs_graph_surface.intersect_surface(&oblique_plane)
            && let Ok((split, _)) = nurbs_graph_patch.split_face_by_surface_curve(
                nurbs_graph_face,
                curve.curve(),
                curve.first_pcurve(),
            )
        {
            let Ok(json) = split.to_json() else {
                return;
            };
            let Ok(raw) = RawModel::from_json(&json) else {
                return;
            };
            if raw.validate().is_err() {
                panic!("validated NURBS tensor graph split failed persistence replay");
            }
        }
        let Ok((v_nurbs_graph_patch, v_nurbs_graph_face)) = builder::nurbs_patch(
            2,
            1,
            vec![
                vec![
                    hyperbrep::Point3::origin(),
                    hyperbrep::Point3::new(Real::one(), Real::from(2), Real::zero()),
                    hyperbrep::Point3::new(Real::one(), Real::from(3), Real::zero()),
                    hyperbrep::Point3::new(Real::from(2), Real::from(2), Real::zero()),
                ],
                vec![
                    hyperbrep::Point3::new(Real::zero(), Real::zero(), Real::from(2)),
                    hyperbrep::Point3::new(Real::one(), Real::from(2), Real::from(2)),
                    hyperbrep::Point3::new(Real::one(), Real::from(3), Real::from(2)),
                    hyperbrep::Point3::new(Real::from(2), Real::from(2), Real::from(2)),
                ],
            ],
            vec![
                vec![Real::one(), Real::from(2), Real::from(3), Real::one()],
                vec![Real::one(), Real::from(2), Real::from(3), Real::one()],
            ],
            vec![
                Real::from(2),
                Real::from(2),
                Real::from(2),
                Real::from(3),
                Real::from(5),
                Real::from(5),
                Real::from(5),
            ],
            vec![Real::from(7), Real::from(7), Real::from(11), Real::from(11)],
        ) else {
            return;
        };
        let v_nurbs_graph_surface = v_nurbs_graph_patch
            .face(v_nurbs_graph_face)
            .and_then(|face| v_nurbs_graph_patch.surface(face.surface()))
            .expect("validated v-linear NURBS graph patch face has a surface");
        if let Ok(SurfaceSurfaceIntersection::Curve(curve)) =
            v_nurbs_graph_surface.intersect_surface(&oblique_plane)
            && let Ok((split, _)) = v_nurbs_graph_patch.split_face_by_surface_curve(
                v_nurbs_graph_face,
                curve.curve(),
                curve.first_pcurve(),
            )
        {
            let Ok(json) = split.to_json() else {
                return;
            };
            let Ok(raw) = RawModel::from_json(&json) else {
                return;
            };
            if raw.validate().is_err() {
                panic!("validated v-linear NURBS tensor graph split failed persistence replay");
            }
        }
        let Ok((partial_graph_patch, partial_graph_face)) = builder::nurbs_patch(
            1,
            1,
            vec![
                vec![
                    hyperbrep::Point3::new(Real::zero(), Real::zero(), Real::from(3)),
                    hyperbrep::Point3::new(Real::from(2), Real::zero(), Real::from(3)),
                ],
                vec![
                    hyperbrep::Point3::new(Real::zero(), Real::one(), Real::one()),
                    hyperbrep::Point3::new(Real::from(2), Real::one(), Real::one()),
                ],
                vec![
                    hyperbrep::Point3::new(Real::zero(), Real::from(2), Real::from(3)),
                    hyperbrep::Point3::new(Real::from(2), Real::from(2), Real::from(3)),
                ],
                vec![
                    hyperbrep::Point3::new(Real::zero(), Real::from(3), Real::one()),
                    hyperbrep::Point3::new(Real::from(2), Real::from(3), Real::one()),
                ],
                vec![
                    hyperbrep::Point3::new(Real::zero(), Real::from(4), Real::from(3)),
                    hyperbrep::Point3::new(Real::from(2), Real::from(4), Real::from(3)),
                ],
            ],
            vec![vec![Real::one(), Real::one()]; 5],
            vec![Real::from(7), Real::from(7), Real::from(11), Real::from(11)],
            vec![
                Real::from(2),
                Real::from(2),
                Real::from(3),
                Real::from(4),
                Real::from(5),
                Real::from(6),
                Real::from(6),
            ],
        ) else {
            return;
        };
        let partial_graph_surface = partial_graph_patch
            .face(partial_graph_face)
            .and_then(|face| partial_graph_patch.surface(face.surface()))
            .expect("validated partial graph patch face has a surface");
        if let Ok(SurfaceSurfaceIntersection::Curves(curves)) =
            partial_graph_surface.intersect_surface(&oblique_plane)
            && let Ok((partitioned, partition)) = partial_graph_patch.split_face_by_surface_curves(
                partial_graph_face,
                &curves,
                SurfaceIntersectionOperand::First,
            )
        {
            assert_eq!(partition.faces.len(), curves.len() + 1);
            let Ok(json) = partitioned.to_json() else {
                return;
            };
            let Ok(raw) = RawModel::from_json(&json) else {
                return;
            };
            if raw.validate().is_err() {
                panic!("validated partial NURBS tensor graph partition failed persistence replay");
            }
        }
    }
    let crossing_width = positive(1);
    let crossing_depth = positive(2);
    let affine_weights = vec![
        vec![positive(1), &positive(1) * Real::from(2)],
        vec![
            &positive(1) * positive(2),
            &positive(1) * positive(2) * Real::from(2),
        ],
    ];
    let Ok((affine_patch, affine_face)) = builder::rational_bezier_patch(
        vec![
            vec![
                hyperbrep::Point3::origin(),
                hyperbrep::Point3::new(crossing_width.clone(), Real::zero(), Real::zero()),
            ],
            vec![
                hyperbrep::Point3::new(Real::zero(), crossing_depth.clone(), Real::zero()),
                hyperbrep::Point3::new(
                    crossing_width.clone(),
                    crossing_depth.clone(),
                    Real::zero(),
                ),
            ],
        ],
        affine_weights,
    ) else {
        return;
    };
    let expected_affine_area = &crossing_width * &crossing_depth;
    assert_eq!(
        hyperlimit::compare_reals(
            &affine_patch
                .face_area(affine_face)
                .expect("separable affine patch has exact area"),
            &expected_affine_area,
        )
        .value(),
        Some(std::cmp::Ordering::Equal)
    );
    let affine_json = affine_patch
        .to_json()
        .expect("separable affine patch serializes");
    let replayed_affine = RawModel::from_json(&affine_json)
        .expect("separable affine patch JSON parses")
        .validate()
        .expect("separable affine patch JSON revalidates");
    assert_eq!(
        hyperlimit::compare_reals(
            &replayed_affine
                .face_area(affine_face)
                .expect("replayed separable affine patch has exact area"),
            &expected_affine_area,
        )
        .value(),
        Some(std::cmp::Ordering::Equal)
    );

    let planar_width = positive(1);
    let planar_height = positive(2);
    let planar_half_width =
        (&planar_width / Real::from(2)).expect("two is a nonzero exact denominator");
    let planar_point = |x: Real, y: Real| CurvePoint2::new(x, y);
    let planar_00 = planar_point(Real::zero(), Real::zero());
    let planar_10 = planar_point(planar_width.clone(), Real::zero());
    let planar_11 = planar_point(planar_width.clone(), planar_height.clone());
    let planar_01 = planar_point(Real::zero(), planar_height.clone());
    let planar_line = |start: CurvePoint2, end: CurvePoint2| {
        Curve2::from(LineSeg2::try_new(start, end).expect("distinct fuzz rectangle vertices"))
    };
    let Ok(planar_bottom) = Curve2::try_nurbs(
        2,
        vec![
            planar_00.clone(),
            planar_point(planar_half_width, Real::zero()),
            planar_10.clone(),
        ],
        vec![Real::one(), positive(2), positive(3)],
        vec![
            Real::from(2),
            Real::from(2),
            Real::from(2),
            Real::from(5),
            Real::from(5),
            Real::from(5),
        ],
    ) else {
        return;
    };
    let Ok(planar_outer) = CurvePath2::try_new(vec![
        planar_bottom,
        planar_line(planar_10, planar_11.clone()),
        planar_line(planar_11, planar_01.clone()),
        planar_line(planar_01, planar_00),
    ]) else {
        return;
    };
    let planar_outer = if bytes[3] & 1 == 0 {
        planar_outer
    } else {
        let Ok(reversed) = planar_outer.reversed() else {
            return;
        };
        reversed
    };
    let planar_u_scale = positive(3);
    let planar_v_scale = positive(1);
    let Ok((planar_model, planar_face)) = builder::planar_face(
        &planar_outer,
        &[],
        hyperbrep::Point3::origin(),
        Vector3::from_xyz(planar_u_scale.clone(), Real::zero(), Real::zero()),
        Vector3::from_xyz(positive(2), planar_v_scale.clone(), Real::zero()),
    ) else {
        return;
    };
    let expected_planar_area = &planar_width * &planar_height * planar_u_scale * planar_v_scale;
    assert_eq!(
        hyperlimit::compare_reals(
            &planar_model
                .face_area(planar_face)
                .expect("authored-frame planar spline region has exact area"),
            &expected_planar_area,
        )
        .value(),
        Some(std::cmp::Ordering::Equal)
    );
    let planar_json = planar_model
        .to_json()
        .expect("planar spline region serializes");
    let replayed_planar = RawModel::from_json(&planar_json)
        .expect("planar spline region JSON parses")
        .validate()
        .expect("planar spline region JSON revalidates");
    assert_eq!(
        hyperlimit::compare_reals(
            &replayed_planar
                .face_area(planar_face)
                .expect("replayed planar spline region has exact area"),
            &expected_planar_area,
        )
        .value(),
        Some(std::cmp::Ordering::Equal)
    );

    let path_extrusion_height = positive(3);
    let Ok((path_extrusion, path_extrusion_solid)) =
        builder::extrude_path(&planar_outer, Real::zero(), path_extrusion_height.clone())
    else {
        return;
    };
    let expected_path_extrusion_volume = planar_width * planar_height * path_extrusion_height;
    assert_eq!(
        hyperlimit::compare_reals(
            &path_extrusion
                .solid_volume(path_extrusion_solid)
                .expect("path extrusion has exact volume"),
            &expected_path_extrusion_volume,
        )
        .value(),
        Some(std::cmp::Ordering::Equal)
    );
    let path_extrusion_json = path_extrusion.to_json().expect("path extrusion serializes");
    let replayed_path_extrusion = RawModel::from_json(&path_extrusion_json)
        .expect("path extrusion JSON parses")
        .validate()
        .expect("path extrusion JSON revalidates");
    assert_eq!(
        hyperlimit::compare_reals(
            &replayed_path_extrusion
                .solid_volume(path_extrusion_solid)
                .expect("replayed path extrusion has exact volume"),
            &expected_path_extrusion_volume,
        )
        .value(),
        Some(std::cmp::Ordering::Equal)
    );

    let transverse_middle = positive(2);
    let transverse_end = &transverse_middle + positive(3);
    let extrusion_controls = vec![
        hyperbrep::Point3::origin(),
        hyperbrep::Point3::new(positive(1), transverse_middle, Real::zero()),
        hyperbrep::Point3::new(Real::zero(), transverse_end.clone(), Real::zero()),
    ];
    let extrusion_weights = vec![Real::one(), positive(2), positive(3)];
    let spline_profile = if bytes[0] & 1 == 0 {
        hyperbrep::Curve3::rational_bezier(extrusion_controls, extrusion_weights)
    } else {
        hyperbrep::Curve3::nurbs(
            2,
            extrusion_controls,
            extrusion_weights,
            vec![
                Real::from(2),
                Real::from(2),
                Real::from(2),
                Real::from(5),
                Real::from(5),
                Real::from(5),
            ],
        )
    };
    let Ok(spline_profile) = spline_profile else {
        return;
    };
    let extrusion_end = positive(1);
    let Ok((extrusion_patch, extrusion_face)) = builder::extrusion_patch(
        spline_profile,
        Vector3::x(),
        -Real::one(),
        extrusion_end.clone(),
    ) else {
        return;
    };
    let expected_extrusion_area = transverse_end * (extrusion_end + Real::one());
    assert_eq!(
        hyperlimit::compare_reals(
            &extrusion_patch
                .face_area(extrusion_face)
                .expect("monotone planar spline extrusion has exact area"),
            &expected_extrusion_area,
        )
        .value(),
        Some(std::cmp::Ordering::Equal)
    );
    let extrusion_json = extrusion_patch
        .to_json()
        .expect("spline extrusion patch serializes");
    let replayed_extrusion = RawModel::from_json(&extrusion_json)
        .expect("spline extrusion patch JSON parses")
        .validate()
        .expect("spline extrusion patch JSON revalidates");
    assert_eq!(
        hyperlimit::compare_reals(
            &replayed_extrusion
                .face_area(extrusion_face)
                .expect("replayed spline extrusion has exact area"),
            &expected_extrusion_area,
        )
        .value(),
        Some(std::cmp::Ordering::Equal)
    );

    let revolution_start_radius = positive(1);
    let revolution_radial_step = positive(2);
    let revolution_height_step = positive(3);
    let revolution_middle_radius = &revolution_start_radius + &revolution_radial_step;
    let revolution_end_radius = &revolution_middle_radius + &revolution_radial_step;
    let revolution_controls = vec![
        hyperbrep::Point3::new(revolution_start_radius, Real::zero(), Real::zero()),
        hyperbrep::Point3::new(
            revolution_middle_radius.clone(),
            Real::zero(),
            revolution_height_step.clone(),
        ),
        hyperbrep::Point3::new(
            revolution_end_radius,
            Real::zero(),
            Real::from(2) * &revolution_height_step,
        ),
    ];
    let revolution_weights = vec![Real::one(), positive(2), positive(3)];
    let revolution_profile = if bytes[1] & 1 == 0 {
        hyperbrep::Curve3::rational_bezier(revolution_controls, revolution_weights)
    } else {
        hyperbrep::Curve3::nurbs(
            2,
            revolution_controls,
            revolution_weights,
            vec![
                Real::from(2),
                Real::from(2),
                Real::from(2),
                Real::from(5),
                Real::from(5),
                Real::from(5),
            ],
        )
    };
    let Ok(revolution_profile) = revolution_profile else {
        return;
    };
    let quarter = (Real::pi() / Real::from(2)).expect("two is nonzero");
    let Ok((revolution_patch, revolution_face)) = builder::revolution_patch(
        revolution_profile,
        hyperbrep::Point3::origin(),
        Vector3::z(),
        Real::zero(),
        quarter,
    ) else {
        return;
    };
    let meridian_step = (&revolution_radial_step * &revolution_radial_step
        + &revolution_height_step * &revolution_height_step)
        .sqrt()
        .expect("positive fuzz steps have an exact square root expression");
    let expected_revolution_area = Real::pi() * meridian_step * revolution_middle_radius;
    assert_eq!(
        hyperlimit::compare_reals(
            &revolution_patch
                .face_area(revolution_face)
                .expect("rational line-image revolution has exact area"),
            &expected_revolution_area,
        )
        .value(),
        Some(std::cmp::Ordering::Equal)
    );
    let revolution_json = revolution_patch
        .to_json()
        .expect("spline revolution patch serializes");
    let replayed_revolution = RawModel::from_json(&revolution_json)
        .expect("spline revolution patch JSON parses")
        .validate()
        .expect("spline revolution patch JSON revalidates");
    assert_eq!(
        hyperlimit::compare_reals(
            &replayed_revolution
                .face_area(revolution_face)
                .expect("replayed spline revolution has exact area"),
            &expected_revolution_area,
        )
        .value(),
        Some(std::cmp::Ordering::Equal)
    );

    let Ok((crossing_patch, crossing_face)) = builder::rational_bezier_patch(
        vec![
            vec![
                hyperbrep::Point3::origin(),
                hyperbrep::Point3::new(crossing_width.clone(), Real::zero(), Real::zero()),
            ],
            vec![
                hyperbrep::Point3::new(Real::zero(), crossing_depth.clone(), Real::zero()),
                hyperbrep::Point3::new(
                    crossing_width.clone(),
                    crossing_depth.clone(),
                    Real::zero(),
                ),
            ],
        ],
        vec![vec![Real::one(), Real::one()]; 2],
    ) else {
        return;
    };
    let crossing_surface = crossing_patch
        .face(crossing_face)
        .and_then(|face| crossing_patch.surface(face.surface()))
        .expect("validated crossing patch face has a surface");
    let half_width = (&crossing_width / Real::from(2)).expect("two is nonzero");
    let half_depth = (&crossing_depth / Real::from(2)).expect("two is nonzero");
    let Ok(x_plane) = Surface::plane(
        hyperbrep::Point3::new(half_width, Real::zero(), Real::zero()),
        Vector3::y(),
        Vector3::z(),
    ) else {
        return;
    };
    let Ok(y_plane) = Surface::plane(
        hyperbrep::Point3::new(Real::zero(), half_depth, Real::zero()),
        Vector3::x(),
        Vector3::z(),
    ) else {
        return;
    };
    let (
        Ok(SurfaceSurfaceIntersection::Curve(x_trace)),
        Ok(SurfaceSurfaceIntersection::Curve(y_trace)),
    ) = (
        crossing_surface.intersect_surface(&x_plane),
        crossing_surface.intersect_surface(&y_plane),
    )
    else {
        return;
    };
    let crossing_traces = [*x_trace, *y_trace];
    let Ok((crossing_partitioned, crossing_partition)) = crossing_patch
        .split_face_by_surface_curves(
            crossing_face,
            &crossing_traces,
            SurfaceIntersectionOperand::First,
        )
    else {
        panic!("represented transverse tensor traces must partition exactly");
    };
    assert_eq!(crossing_partition.faces.len(), 4);
    let Ok(crossing_json) = crossing_partitioned.to_json() else {
        return;
    };
    if RawModel::from_json(&crossing_json)
        .and_then(RawModel::validate)
        .is_err()
    {
        panic!("crossing tensor partition failed exact persistence replay");
    }

    let patch_controls = vec![
        vec![
            hyperbrep::Point3::new(Real::zero(), Real::zero(), Real::zero()),
            hyperbrep::Point3::new(Real::one(), Real::from(2), Real::zero()),
            hyperbrep::Point3::new(Real::from(2), Real::zero(), Real::zero()),
        ],
        vec![
            hyperbrep::Point3::new(Real::zero(), Real::zero(), Real::from(2)),
            hyperbrep::Point3::new(Real::one(), Real::from(2), Real::from(2)),
            hyperbrep::Point3::new(Real::from(2), Real::zero(), Real::from(2)),
        ],
    ];
    let patch_weights = vec![
        vec![Real::one(), Real::from(2), Real::one()],
        vec![Real::one(), Real::from(2), Real::one()],
    ];
    let Ok((patch, patch_face)) =
        builder::rational_bezier_patch(patch_controls.clone(), patch_weights.clone())
    else {
        return;
    };
    if let Ok(artifact) =
        hyperbrep::tessellation::approximate_face_chordally(&patch, patch_face, chordal_policy)
    {
        assert_eq!(artifact.parameters().len(), artifact.points().len());
        for triangle in artifact.triangles() {
            assert!(
                triangle
                    .iter()
                    .all(|index| *index < artifact.parameters().len())
            );
        }
    }
    let plane_z = Real::from(bytes[0] % 4);
    let Ok((plane_face_model, plane_solid)) = builder::cuboid(
        hyperbrep::Point3::new(Real::from(-1), Real::from(-1), plane_z.clone()),
        hyperbrep::Point3::new(Real::from(3), Real::from(3), plane_z + Real::one()),
    ) else {
        return;
    };
    let Some(plane_face) = plane_face_model
        .solid(plane_solid)
        .and_then(|solid| plane_face_model.shell(solid.outer()))
        .and_then(|shell| shell.faces().first())
        .copied()
    else {
        panic!("validated cuboid has no outer-shell face");
    };
    if let Ok(Some(pair)) =
        boolean::intersect_faces(&patch, patch_face, &plane_face_model, plane_face)
        && let boolean::FacePairTrim::SurfaceCurveFragments(fragments) = pair.trim()
    {
        for fragment in fragments {
            let domain = fragment.curve().domain();
            let Ok(parameter) = (domain.start() + domain.end()) / Real::from(2) else {
                continue;
            };
            let _ = fragment.curve().point_at(&parameter);
            let _ = fragment.first_pcurve().point_at(&parameter);
            let _ = fragment.second_pcurve().point_at(&parameter);
            let _ = patch.split_face_by_surface_curve(
                patch_face,
                fragment.curve(),
                fragment.first_pcurve(),
            );
        }
    }
    if let Ok((patch_shell, _)) = builder::tensor_patch_shell(vec![
        hyperbrep::TensorPatch::RationalBezier {
            control_points: patch_controls,
            weights: patch_weights,
        },
        hyperbrep::TensorPatch::RationalBezier {
            control_points: vec![
                vec![
                    hyperbrep::Point3::new(Real::from(2), Real::zero(), Real::zero()),
                    hyperbrep::Point3::new(Real::from(3), Real::from(2), Real::zero()),
                    hyperbrep::Point3::new(Real::from(4), Real::zero(), Real::zero()),
                ],
                vec![
                    hyperbrep::Point3::new(Real::from(2), Real::zero(), Real::from(2)),
                    hyperbrep::Point3::new(Real::from(3), Real::from(2), Real::from(2)),
                    hyperbrep::Point3::new(Real::from(4), Real::zero(), Real::from(2)),
                ],
            ],
            weights: vec![
                vec![Real::one(), Real::from(3), Real::one()],
                vec![Real::one(), Real::from(3), Real::one()],
            ],
        },
    ]) {
        let Ok(json) = patch_shell.to_json() else {
            return;
        };
        let Ok(raw) = RawModel::from_json(&json) else {
            return;
        };
        if raw.validate().is_err() {
            panic!("validated multi-patch shell failed exact persistence replay");
        }
    }

    let Ok(profile) = hyperbrep::Curve3::rational_bezier(
        vec![
            hyperbrep::Point3::origin(),
            hyperbrep::Point3::new(positive(1), positive(2), Real::zero()),
            hyperbrep::Point3::new(positive(3), Real::zero(), Real::zero()),
        ],
        vec![Real::one(), positive(2), Real::one()],
    ) else {
        return;
    };
    let Ok(extrusion) = Surface::extrusion(profile, Vector3::z()) else {
        return;
    };
    let Ok(oblique_plane) = Surface::plane(
        hyperbrep::Point3::origin(),
        Vector3::from_xyz(Real::one(), Real::zero(), Real::one()),
        Vector3::y(),
    ) else {
        return;
    };
    if let Ok(SurfaceSurfaceIntersection::Curve(curve)) =
        extrusion.intersect_surface(&oblique_plane)
    {
        let parameter = (Real::one() / Real::from(2)).expect("nonzero rational denominator");
        let _ = curve.curve().point_at(&parameter);
        let _ = curve.first_pcurve().point_at(&parameter);
        let _ = curve.second_pcurve().point_at(&parameter);
    }

    let Ok(ruled) = Surface::extrusion(
        hyperbrep::Curve3::line(
            hyperbrep::Point3::origin(),
            hyperbrep::Point3::new(positive(1), Real::zero(), Real::zero()),
        )
        .expect("positive fuzz extent makes a nondegenerate line"),
        Vector3::z(),
    ) else {
        return;
    };
    let Ok(parallel_plane) = Surface::plane(
        hyperbrep::Point3::new(
            (positive(1) / Real::from(2)).expect("nonzero rational denominator"),
            Real::zero(),
            Real::zero(),
        ),
        Vector3::y(),
        Vector3::z(),
    ) else {
        return;
    };
    let _ = ruled.intersect_surface(&parallel_plane);
});
