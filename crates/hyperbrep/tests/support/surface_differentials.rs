use hyperbrep::*;
use hypercurve::{CurvePoint2, RationalMap2};
use hypersolve::BivariatePolynomial;
use std::cmp::Ordering;

fn q(n: i32, d: i32) -> Real {
    (Real::from(n) / Real::from(d)).unwrap()
}
fn polynomial(rows: Vec<Vec<Real>>) -> BivariatePolynomial {
    BivariatePolynomial::new(rows)
}
fn one() -> BivariatePolynomial {
    polynomial(vec![vec![1.into()]])
}
fn x() -> BivariatePolynomial {
    polynomial(vec![vec![0.into()], vec![1.into()]])
}
fn y() -> BivariatePolynomial {
    polynomial(vec![vec![0.into(), 1.into()]])
}
fn circle() -> BivariatePolynomial {
    polynomial(vec![
        vec![q(7, 16), (-1).into(), 1.into()],
        vec![(-1).into()],
        vec![1.into()],
    ])
}
fn binomial(n: usize, k: usize) -> i32 {
    (0..k).fold(1, |value, i| value * (n - i) as i32 / (i + 1) as i32)
}
// Independent power-to-Bernstein identity supplies exact authored test nets.
fn tensor_net(
    coordinates: [BivariatePolynomial; 3],
    weight: BivariatePolynomial,
) -> (Vec<Vec<Point3>>, Vec<Vec<Real>>) {
    let polynomials = [
        coordinates[0].clone(),
        coordinates[1].clone(),
        coordinates[2].clone(),
        weight,
    ];
    let n = polynomials
        .iter()
        .map(|p| p.coefficients.len().saturating_sub(1))
        .max()
        .unwrap()
        .max(2);
    let m = polynomials
        .iter()
        .flat_map(|p| &p.coefficients)
        .map(|row| row.len().saturating_sub(1))
        .max()
        .unwrap()
        .max(2);
    assert!(n <= 8 && m <= 8);
    let bernstein = |p: &BivariatePolynomial, i, j| {
        let mut value = Real::zero();
        for (k, row) in p.coefficients.iter().enumerate().take(i + 1) {
            for (l, coefficient) in row.iter().enumerate().take(j + 1) {
                value += coefficient
                    * q(binomial(i, k), binomial(n, k))
                    * q(binomial(j, l), binomial(m, l));
            }
        }
        value
    };
    let mut points = Vec::new();
    let mut weights = Vec::new();
    for j in 0..=m {
        let mut row = Vec::new();
        let mut wrow = Vec::new();
        for i in 0..=n {
            let w = bernstein(&polynomials[3], i, j);
            let coords = (0..3)
                .map(|axis| (bernstein(&polynomials[axis], i, j) / &w).unwrap())
                .collect::<Vec<_>>();
            row.push(Point3::new(
                coords[0].clone(),
                coords[1].clone(),
                coords[2].clone(),
            ));
            wrow.push(w);
        }
        points.push(row);
        weights.push(wrow);
    }
    (points, weights)
}
fn tensor(coordinates: [BivariatePolynomial; 3], weight: BivariatePolynomial) -> Surface {
    let (points, weights) = tensor_net(coordinates, weight);
    Surface::rational_bezier(points, weights).unwrap()
}
fn plane() -> Surface {
    Surface::plane(Point3::origin(), Vector3::x(), Vector3::y()).unwrap()
}
fn branches(result: SurfaceSurfaceIntersection) -> Vec<SurfaceIntersectionCurve> {
    match result {
        SurfaceSurfaceIntersection::Curve(c) => vec![*c],
        SurfaceSurfaceIntersection::Curves(c) => c,
        other => panic!("expected complete curve branches, got {other:?}"),
    }
}

#[allow(dead_code)]
#[path = "../../../hypercurve/tests/support/rational_point_forms.rs"]
mod point_recipe;

fn selected_parameter() -> Parameter {
    let curve = Curve3::rational_bezier(
        [-1, -1, -1, 4]
            .map(|n| Point3::new(n.into(), 0.into(), 0.into()))
            .to_vec(),
        vec![Real::one(); 4],
    )
    .unwrap();
    let CurveParameterLocation::Parameters(mut parameters) =
        curve.parameters_of(&Point3::origin()).unwrap()
    else {
        panic!("one selected cubic root")
    };
    assert_eq!(parameters.len(), 1);
    let p = parameters.pop().unwrap();
    assert!(p.try_real().is_none());
    p
}

fn shear() -> Matrix4 {
    Matrix4::from_row_major([
        2.into(),
        3.into(),
        0.into(),
        7.into(),
        0.into(),
        (-1).into(),
        0.into(),
        (-4).into(),
        1.into(),
        0.into(),
        4.into(),
        2.into(),
        0.into(),
        0.into(),
        0.into(),
        1.into(),
    ])
}
fn represented(x: Real, y: Real) -> CurvePoint2 {
    hypercurve::Point2::new(x, y).into()
}
fn real_vector(x: i32, y: i32, z: i32) -> GeometricVector3 {
    GeometricVector3::from_coordinates(Vector3::new([x.into(), y.into(), z.into()]))
}
fn unit_and_orthogonal(partials: &SurfacePartials, normal: &GeometricVector3) {
    assert_eq!(
        normal.compare_squared_length(&Real::one()).unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        normal.compare_dot(partials.u(), &Real::zero()).unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        normal.compare_dot(partials.v(), &Real::zero()).unwrap(),
        Ordering::Equal
    );
    assert!(normal.equals(&partials.normal().unwrap()).unwrap());
}
fn replay(surface: &Surface, tag: &str) -> Surface {
    let mut builder = ModelBuilder::new();
    builder.surface(surface.clone()).unwrap();
    let model = builder.finish().unwrap();
    let json = model.to_json().unwrap();
    #[cfg(not(target_arch = "wasm32"))]
    if let Some(dir) = std::env::var_os("HYPERBREP_SURFACE_DIFFERENTIAL_ARTIFACT_DIR") {
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(
            std::path::Path::new(&dir).join(format!("{tag}.json")),
            &json,
        )
        .unwrap();
    }
    let _ = tag;
    let restored = RawModel::from_json(&json, &ArchiveLimits::default())
        .unwrap()
        .validate()
        .unwrap();
    assert_eq!(restored.to_json().unwrap(), json);
    restored.surfaces().next().unwrap().geometry().clone()
}
fn source_parameter(curve: &Curve3, base: &Parameter) -> Parameter {
    let frame = curve.planar_geometry().unwrap();
    let scale = (Real::one() / frame.source_scale).unwrap();
    base.affine_image(&scale, &(-frame.source_offset * &scale))
        .unwrap()
}
fn chain(section: &SurfaceIntersectionCurve, surfaces: [&Surface; 2], base: &Parameter) {
    let at = source_parameter(section.curve(), base);
    let spatial = section.curve().derivative_at(&at, 1).unwrap();
    for (surface, pcurve) in surfaces
        .into_iter()
        .zip([section.first_pcurve(), section.second_pcurve()])
    {
        let expected_uv = pcurve.point_at(&at).unwrap();
        let pcurve = pcurve.materialize().unwrap();
        let (scale, offset) = pcurve.correspondence().affine_coefficients().unwrap();
        // spatial = scale*pcurve + offset, including negative reversal scales.
        let inverse = (Real::one() / scale).unwrap();
        let local = at
            .affine_image(&inverse, &(-offset * &inverse))
            .unwrap()
            .curve_parameter();
        let uv = pcurve.curve().point_at(&local).unwrap();
        assert!(uv.coincides_with(&expected_uv).unwrap());
        let velocity = pcurve.curve().derivative_at(&local).unwrap();
        let partials = surface.partials_at(&uv).unwrap();
        let chart = Matrix4::from_row_major([
            inverse.clone(),
            0.into(),
            0.into(),
            0.into(),
            0.into(),
            inverse.clone(),
            0.into(),
            0.into(),
            0.into(),
            0.into(),
            inverse,
            0.into(),
            0.into(),
            0.into(),
            0.into(),
            1.into(),
        ]);
        let lifted = partials
            .apply_direction(&velocity)
            .unwrap()
            .affine_image(&chart)
            .unwrap();
        assert!(
            lifted.equals(&spatial).unwrap(),
            "both complete pcurves must lift to the spatial derivative"
        );
        let normal = surface.normal_at(&uv).unwrap();
        unit_and_orthogonal(&partials, &normal);
        assert_eq!(
            normal.compare_dot(&spatial, &Real::zero()).unwrap(),
            Ordering::Equal
        );
    }
}

pub fn complete_circle_extrusions_and_both_pcurve_consumers() {
    let patch = tensor([x(), y(), circle()], one());
    let plane = plane();
    let sections = branches(plane.intersect_surface(&patch).unwrap());
    assert_eq!(sections.len(), 2);
    for (i, section) in sections.iter().enumerate() {
        let at = source_parameter(section.curve(), &q(3, 5).into());
        let surface = Surface::extrusion(section.curve().clone(), Vector3::z()).unwrap();
        for surface in [surface.clone(), replay(&surface, &format!("circle-{i}"))] {
            let uv = represented(at.try_real().unwrap().clone(), q(1, 3));
            let partials = surface.partials_at(&uv).unwrap();
            assert!(partials.u().try_coordinates().is_none());
            let normal = surface.normal_at(&uv).unwrap();
            unit_and_orthogonal(&partials, &normal);
            // At x=3/5 the unit-circle-branch normal has |nx|=2/5,
            // ny=-sqrt(21)/5, nz=0 in this authored graph chart.
            assert_eq!(
                normal
                    .compare_coordinate(Axis::X, &if i == 0 { q(2, 5) } else { q(-2, 5) })
                    .unwrap(),
                Ordering::Equal
            );
            assert_eq!(
                normal.compare_coordinate(Axis::Z, &Real::zero()).unwrap(),
                Ordering::Equal
            );
            assert_eq!(
                normal.compare_coordinate(Axis::Y, &q(-9, 10)).unwrap(),
                Ordering::Less
            );
            assert_eq!(
                normal.compare_coordinate(Axis::Y, &q(-19, 20)).unwrap(),
                Ordering::Greater
            );
            let gap = (Real::one() / Real::from(1_000_000_000_000u64)).unwrap();
            let expected = if i == 0 { q(2, 5) } else { q(-2, 5) };
            assert_eq!(
                normal
                    .compare_coordinate(Axis::X, &(&expected + &gap))
                    .unwrap(),
                Ordering::Less
            );
            assert_eq!(
                normal
                    .compare_coordinate(Axis::X, &(&expected - gap))
                    .unwrap(),
                Ordering::Greater
            );
        }
        chain(section, [&plane, &patch], &q(3, 5).into());
        chain(section, [&plane, &patch], &selected_parameter());
        let restricted = section
            .restrict_source(ParameterRange::new(q(3, 5).into(), q(2, 3).into()).unwrap())
            .unwrap();
        chain(&restricted, [&plane, &patch], &q(3, 5).into());
        let reverse = section.reversed().unwrap();
        chain(&reverse, [&plane, &patch], &q(3, 5).into());
    }
}

pub fn cubic_and_projective_chain_rules() {
    let equation = polynomial(vec![
        vec![0.into(), 0.into(), 0.into(), 1.into()],
        vec![(-1).into()],
    ]);
    for weighted in [false, true] {
        let patch = tensor(
            [x(), y(), equation.clone()],
            if weighted {
                polynomial(vec![vec![1.into()], vec![1.into()]])
            } else {
                one()
            },
        );
        let plane = plane();
        let sections = branches(plane.intersect_surface(&patch).unwrap());
        assert_eq!(sections.len(), 1);
        chain(&sections[0], [&plane, &patch], &q(1, 2).into());
        chain(&sections[0], [&plane, &patch], &selected_parameter());
        let restored = replay(
            &patch,
            if weighted {
                "projective-patch"
            } else {
                "cubic-patch"
            },
        );
        chain(&sections[0], [&plane, &restored], &q(1, 2).into());
    }
}

pub fn selected_tensors_normals_and_native_knot_charts() {
    let uv = point_recipe::joint_uv();
    let mut previous = None;
    for weighted in [false, true] {
        // Exact map (u²,16v³,u²+16v³)/(1+uv), with positive weights.
        let a = polynomial(vec![vec![], vec![], vec![1.into()]]);
        let b = polynomial(vec![vec![0.into(), 0.into(), 0.into(), 16.into()]]);
        let sum = polynomial(vec![
            vec![0.into(), 0.into(), 0.into(), 16.into()],
            vec![],
            vec![1.into()],
        ]);
        let (control_points, weights) = tensor_net(
            [a, b, sum],
            if weighted {
                polynomial(vec![vec![1.into()], vec![0.into(), 1.into()]])
            } else {
                one()
            },
        );
        let surface = Surface::rational_bezier(control_points.clone(), weights.clone()).unwrap();
        let expected = real_vector(-1, -1, 1).normalized().unwrap();
        for surface in [
            surface.clone(),
            replay(&surface, &format!("joint-tensor-{weighted}")),
        ] {
            let partials = surface.partials_at(&uv).unwrap();
            assert!(
                partials.u().try_coordinates().is_none()
                    && partials.v().try_coordinates().is_none()
            );
            let normal = surface.normal_at(&uv).unwrap();
            unit_and_orthogonal(&partials, &normal);
            assert!(normal.equals(&expected).unwrap());
            if !weighted {
                assert_eq!(
                    partials.u().compare_squared_length(&Real::one()).unwrap(),
                    Ordering::Equal
                );
                assert_eq!(
                    partials
                        .u()
                        .compare_coordinate(Axis::Y, &Real::zero())
                        .unwrap(),
                    Ordering::Equal
                );
                assert_eq!(
                    partials
                        .v()
                        .compare_coordinate(Axis::X, &Real::zero())
                        .unwrap(),
                    Ordering::Equal
                );
            }
            assert_eq!(
                normal
                    .compare_dot(&real_vector(1, 1, -1), &(-Real::one()))
                    .unwrap(),
                Ordering::Less
            );
            assert_eq!(
                normal.compare_dot(&expected, &Real::one()).unwrap(),
                Ordering::Equal
            );
            let nurbs = Surface::nurbs(
                control_points[0].len() - 1,
                control_points.len() - 1,
                control_points.to_vec(),
                weights.to_vec(),
                [
                    vec![2.into(); control_points[0].len()],
                    vec![5.into(); control_points[0].len()],
                ]
                .concat(),
                [
                    vec![(-4).into(); control_points.len()],
                    vec![3.into(); control_points.len()],
                ]
                .concat(),
            )
            .unwrap();
            let native = uv
                .rational_image(&RationalMap2::new(
                    [
                        polynomial(vec![vec![2.into()], vec![3.into()]]),
                        polynomial(vec![vec![(-4).into(), 7.into()]]),
                    ],
                    one(),
                ))
                .unwrap();
            let np = nurbs.partials_at(&native).unwrap();
            let scale = Matrix4::from_row_major([
                3.into(),
                0.into(),
                0.into(),
                0.into(),
                0.into(),
                3.into(),
                0.into(),
                0.into(),
                0.into(),
                0.into(),
                3.into(),
                0.into(),
                0.into(),
                0.into(),
                0.into(),
                1.into(),
            ]);
            assert!(
                np.u()
                    .affine_image(&scale)
                    .unwrap()
                    .equals(partials.u())
                    .unwrap()
            );
            let scale = Matrix4::from_row_major([
                7.into(),
                0.into(),
                0.into(),
                0.into(),
                0.into(),
                7.into(),
                0.into(),
                0.into(),
                0.into(),
                0.into(),
                7.into(),
                0.into(),
                0.into(),
                0.into(),
                0.into(),
                1.into(),
            ]);
            assert!(
                np.v()
                    .affine_image(&scale)
                    .unwrap()
                    .equals(partials.v())
                    .unwrap()
            );
            assert!(nurbs.normal_at(&native).unwrap().equals(&normal).unwrap());
            assert!(
                replay(&nurbs, &format!("joint-nurbs-{weighted}"))
                    .normal_at(&native)
                    .unwrap()
                    .equals(&normal)
                    .unwrap()
            );
            previous = Some(normal);
        }
    }
    assert!(
        previous
            .unwrap()
            .equals(&real_vector(-1, -1, 1).normalized().unwrap())
            .unwrap()
    );
}

pub fn native_vector_algebra_affine_transport_and_replay() {
    let curve = Curve3::rational_bezier(
        vec![
            Point3::origin(),
            Point3::new(q(1, 3), 0.into(), 0.into()),
            Point3::new(q(2, 3), q(1, 3), 0.into()),
            Point3::new(1.into(), 1.into(), 1.into()),
        ],
        vec![1.into(); 4],
    )
    .unwrap();
    // C=(t,t²,t³): v=(1,2t,3t²), a=(0,2,6t), v·a=4t+18t³.
    let at = selected_parameter();
    let v = curve.derivative_at(&at, 1).unwrap();
    assert_eq!(
        curve
            .derivative_at(&at, 4)
            .unwrap()
            .normalized()
            .unwrap_err(),
        GeometryError::DegenerateVector
    );
    let a = curve.derivative_at(&at, 2).unwrap();
    let cross = v.cross(&a).unwrap();
    assert_eq!(
        cross.compare_coordinate(Axis::Z, &2.into()).unwrap(),
        Ordering::Equal
    );
    assert_eq!(cross.compare_dot(&v, &0.into()).unwrap(), Ordering::Equal);
    assert_eq!(cross.compare_dot(&a, &0.into()).unwrap(), Ordering::Equal);
    let normal = cross.normalized().unwrap();
    assert_eq!(
        normal.compare_squared_length(&1.into()).unwrap(),
        Ordering::Equal
    );
    assert!(normal.normalized().unwrap().equals(&normal).unwrap());
    let mapped = Curve3::affine_image(curve.clone(), shear()).unwrap();
    let transported = v.affine_image(&shear()).unwrap();
    assert!(
        transported
            .equals(&mapped.derivative_at(&at, 1).unwrap())
            .unwrap()
    );
    let mapped_cross = transported
        .cross(&a.affine_image(&shear()).unwrap())
        .unwrap();
    assert_eq!(
        mapped_cross.compare_dot(&transported, &0.into()).unwrap(),
        Ordering::Equal
    );
    assert_eq!(
        mapped_cross
            .normalized()
            .unwrap()
            .compare_squared_length(&1.into())
            .unwrap(),
        Ordering::Equal
    );
    let unit = v.normalized().unwrap();
    let affine_unit = unit.affine_image(&shear()).unwrap();
    assert_ne!(
        affine_unit.compare_squared_length(&1.into()).unwrap(),
        Ordering::Equal
    );
    assert!(
        affine_unit
            .normalized()
            .unwrap()
            .equals(&transported.normalized().unwrap())
            .unwrap()
    );
    let extrusion = Surface::extrusion(curve, Vector3::z()).unwrap();
    let uv = at
        .curve_parameter()
        .affine_point(1.into(), 0.into(), 0.into(), q(1, 3))
        .unwrap();
    for surface in [extrusion.clone(), replay(&extrusion, "native-extrusion")] {
        let partials = surface.partials_at(&uv).unwrap();
        assert!(partials.u().equals(&v).unwrap());
        unit_and_orthogonal(&partials, &surface.normal_at(&uv).unwrap());
    }
}

pub fn selected_parallel_oriented_unit_normals() {
    let at = selected_parameter()
        .affine_image(&q(4, 3), &Real::zero())
        .unwrap();
    for (i, distance) in [q(1, 4), 4.into()].into_iter().enumerate() {
        let parallel = hypercurve::QuadraticBezier2::new(
            hypercurve::Point2::from_values(0, 0),
            hypercurve::Point2::new(q(1, 2), 0.into()),
            hypercurve::Point2::from_values(1, 1),
        )
        .parallel_left(distance)
        .unwrap();
        let source = hypercurve::Curve2::try_analytic_parallel(
            parallel,
            hypercurve::BezierParameterRange2::try_new(
                hypercurve::BezierParameter2::Exact(0.into()),
                hypercurve::BezierParameter2::Exact(q(3, 4)),
            )
            .unwrap(),
        )
        .unwrap();
        let curve =
            Curve3::planar_image(source, Point3::origin(), Vector3::x(), Vector3::y()).unwrap();
        let surface = Surface::extrusion(curve, Vector3::z()).unwrap();
        let uv = at
            .curve_parameter()
            .affine_point(1.into(), 0.into(), 0.into(), 0.into())
            .unwrap();
        for surface in [surface.clone(), replay(&surface, &format!("parallel-{i}"))] {
            let normal = surface.normal_at(&uv).unwrap();
            unit_and_orthogonal(&surface.partials_at(&uv).unwrap(), &normal);
            assert_eq!(
                normal.compare_squared_length(&1.into()).unwrap(),
                Ordering::Equal
            );
            assert_eq!(
                normal.coordinate_sign(Axis::X).unwrap(),
                if i == 0 {
                    hyperreal::RealSign::Positive
                } else {
                    hyperreal::RealSign::Negative
                }
            );
            assert_eq!(
                normal.coordinate_sign(Axis::Y).unwrap(),
                if i == 0 {
                    hyperreal::RealSign::Negative
                } else {
                    hyperreal::RealSign::Positive
                }
            );
            assert_eq!(
                normal.compare_coordinate(Axis::Z, &0.into()).unwrap(),
                Ordering::Equal
            );
            // Selected parallel speed magnitudes remain explicit blockers.
            assert!(
                surface
                    .partials_at(&uv)
                    .unwrap()
                    .u()
                    .compare_squared_length(&1.into())
                    .is_err()
            );
        }
    }
}

fn parallel_cusp() {
    let source = hypercurve::QuadraticBezier2::new(
        hypercurve::Point2::from_values(0, 0),
        hypercurve::Point2::new(q(1, 2), 0.into()),
        hypercurve::Point2::from_values(1, 1),
    )
    .parallel_left(q(1, 2))
    .unwrap();
    let source = hypercurve::Curve2::try_analytic_parallel(
        source,
        hypercurve::BezierParameterRange2::try_new(
            hypercurve::BezierParameter2::Exact(0.into()),
            hypercurve::BezierParameter2::Exact(q(3, 4)),
        )
        .unwrap(),
    )
    .unwrap();
    let profile =
        Curve3::planar_image(source, Point3::origin(), Vector3::x(), Vector3::y()).unwrap();
    let extrusion = Surface::extrusion(profile, Vector3::z()).unwrap();
    assert_eq!(
        extrusion
            .normal_at(&represented(0.into(), 0.into()))
            .unwrap_err(),
        GeometryError::SingularSurfaceParameter
    );
}

pub fn singular_domains_policy_and_budget_recovery() {
    parallel_cusp();
    assert_eq!(
        real_vector(0, 0, 0).normalized().unwrap_err(),
        GeometryError::DegenerateVector
    );
    let stationary = Curve3::rational_bezier(
        vec![
            Point3::origin(),
            Point3::new(1.into(), 0.into(), 0.into()),
            Point3::origin(),
        ],
        vec![1.into(); 3],
    )
    .unwrap();
    let extrusion = Surface::extrusion(stationary, Vector3::z()).unwrap();
    assert_eq!(
        extrusion
            .normal_at(&represented(q(1, 2), 0.into()))
            .unwrap_err(),
        GeometryError::SingularSurfaceParameter
    );
    let patch = tensor([x(), y(), circle()], one());
    assert_eq!(
        patch
            .partials_at(&represented((-1).into(), 0.into()))
            .unwrap_err(),
        GeometryError::SurfaceParameterOutsideDomain
    );
    let selected = point_recipe::joint_uv();
    let outside = selected
        .rational_image(&RationalMap2::new(
            [polynomial(vec![vec![2.into()], vec![1.into()]]), y()],
            one(),
        ))
        .unwrap();
    assert_eq!(
        patch.partials_at(&outside).unwrap_err(),
        GeometryError::SurfaceParameterOutsideDomain
    );
    let cylinder = Surface::cylinder(
        Point3::origin(),
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        1.into(),
    )
    .unwrap();
    assert_eq!(
        cylinder.normal_at(&selected).unwrap_err(),
        GeometryError::UnsupportedSelectedEvaluation
    );
    // An already available exact proof is allowed to need no new refinement.
    let normal = patch.normal_at(&selected).unwrap();
    let warm = OperationContext::strict(WorkBudget::ZERO).evaluate(|| patch.normal_at(&selected));
    let Ok(Ok(warm_normal)) = warm.result else {
        panic!("warm exact normal must retain its proof")
    };
    assert!(warm_normal.equals(&normal).unwrap());
    // The full implicit-section extrusion requires fresh differential work.
    let section = branches(plane().intersect_surface(&patch).unwrap()).remove(0);
    let extrusion = Surface::extrusion(section.curve().clone(), Vector3::z()).unwrap();
    let uv = represented(q(3, 5), 0.into());
    let denied = OperationContext::strict(WorkBudget::ZERO).evaluate(|| extrusion.normal_at(&uv));
    let Err(exhausted) = denied.result else {
        panic!("cold implicit differential must report budget exhaustion")
    };
    assert_eq!(exhausted.limit, 0);
    unit_and_orthogonal(
        &extrusion.partials_at(&uv).unwrap(),
        &extrusion.normal_at(&uv).unwrap(),
    );
    for policy in [
        hypercurve::PredicatePolicy::STRICT,
        hypercurve::PredicatePolicy::APPROXIMATE_512,
        hypercurve::PredicatePolicy::STRICT,
    ] {
        let outcome = hypercurve::evaluate_under(policy, || {
            let p = patch.partials_at(&selected).unwrap();
            unit_and_orthogonal(&p, &patch.normal_at(&selected).unwrap());
        });
        assert_eq!(outcome.certainty(), hypercurve::CurveCertainty::Certified);
    }
    let zero = tensor([x(), polynomial(vec![]), polynomial(vec![])], one());
    assert_eq!(
        zero.normal_at(&selected).unwrap_err(),
        GeometryError::SingularSurfaceParameter
    );
}

pub fn nonsmooth_knot_ownership_and_reflected_surface_normals() {
    // A continuous piecewise-linear graph, with a slope discontinuity at u=3.
    // The authored parameter domain is [2,5] x [-4,3].
    let controls = [0, 7]
        .into_iter()
        .map(|y| {
            vec![
                Point3::new(0.into(), y.into(), 0.into()),
                Point3::new(1.into(), y.into(), 1.into()),
                Point3::new(2.into(), y.into(), 3.into()),
            ]
        })
        .collect::<Vec<_>>();
    let make = |net| {
        Surface::nurbs(
            1,
            1,
            net,
            vec![vec![1.into(); 3]; 2],
            vec![2.into(), 2.into(), 3.into(), 5.into(), 5.into()],
            vec![(-4).into(), (-4).into(), 3.into(), 3.into()],
        )
        .unwrap()
    };
    let surface = make(controls.clone());
    let transform = shear(); // det < 0; authored Ru x Rv includes the reflection.
    let reflected = make(
        controls
            .iter()
            .map(|row| {
                row.iter()
                    .map(|p| transform.transform_point3(p).unwrap())
                    .collect()
            })
            .collect(),
    );
    let parameter = selected_parameter().curve_parameter();
    let gap = (Real::one() / Real::from(1_000_000_000_000u64)).unwrap();
    for (u, slope) in [
        (2.into(), Real::one()),
        (Real::from(3) - &gap, Real::one()),
        (3.into(), q(1, 2)),
        (Real::from(3) + &gap, q(1, 2)),
        (5.into(), q(1, 2)),
    ] {
        let uv = parameter
            .affine_point(0.into(), u, 7.into(), (-4).into())
            .unwrap();
        let expected_u =
            GeometricVector3::from_coordinates(Vector3::new([slope, 0.into(), 1.into()]));
        let expected_v = real_vector(0, 1, 0);
        let partials = surface.partials_at(&uv).unwrap();
        assert!(partials.u().equals(&expected_u).unwrap());
        assert!(partials.v().equals(&expected_v).unwrap());
        let normal = surface.normal_at(&uv).unwrap();
        assert!(
            normal
                .equals(&expected_u.cross(&expected_v).unwrap().normalized().unwrap())
                .unwrap()
        );
        unit_and_orthogonal(&partials, &normal);
        let transformed = reflected.partials_at(&uv).unwrap();
        let ru = expected_u.affine_image(&transform).unwrap();
        let rv = expected_v.affine_image(&transform).unwrap();
        assert!(transformed.u().equals(&ru).unwrap());
        assert!(transformed.v().equals(&rv).unwrap());
        let reflected_normal = reflected.normal_at(&uv).unwrap();
        assert!(
            reflected_normal
                .equals(&ru.cross(&rv).unwrap().normalized().unwrap())
                .unwrap()
        );
        unit_and_orthogonal(&transformed, &reflected_normal);
        assert!(
            !reflected_normal
                .equals(
                    &normal
                        .affine_image(&transform)
                        .unwrap()
                        .normalized()
                        .unwrap()
                )
                .unwrap()
        );
    }
    let uv = parameter
        .affine_point(0.into(), 3.into(), 7.into(), (-4).into())
        .unwrap();
    assert!(
        replay(&surface, "nonsmooth-knot")
            .normal_at(&uv)
            .unwrap()
            .equals(&surface.normal_at(&uv).unwrap())
            .unwrap()
    );
    assert!(
        replay(&reflected, "reflected-nonsmooth-knot")
            .normal_at(&uv)
            .unwrap()
            .equals(&reflected.normal_at(&uv).unwrap())
            .unwrap()
    );
}
