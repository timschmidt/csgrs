//! Complete polynomial pullbacks and checked spatial images of plane sections.

use super::*;
use hypercurve::RationalMap2;
use hypersolve::BivariatePolynomial;
use hypersolve::bivariate_arithmetic::{bivariate_add, bivariate_scale, bivariate_subtract};

pub(super) fn intersect(
    plane: &PlaneSurface,
    surface: &RationalBezierSurface,
) -> GeometryResult<SurfaceSurfaceIntersection> {
    // A positive-weight tensor lies in its control-point convex hull. Strict
    // one-sided homogeneous plane values prove an empty section without
    // constructing a projection polynomial. Zeros mixed with one sign are
    // deliberately left to complete decomposition (boundary contacts matter).
    let normal = plane.u.cross(&plane.v);
    let mut signs = [false; 3];
    'controls: for row in surface.homogeneous_controls() {
        for point in row {
            let displacement = Vector3::new([
                &point.x - &plane.origin.x * &point.w,
                &point.y - &plane.origin.y * &point.w,
                &point.z - &plane.origin.z * &point.w,
            ]);
            let Ok(order) = decided_order(compare_reals(
                &normal.dot(&displacement),
                &Real::zero(),
                crate::STRICT_PREDICATES,
            )) else {
                signs = [true; 3];
                break 'controls;
            };
            let index = match order {
                Ordering::Less => 0,
                Ordering::Equal => 1,
                Ordering::Greater => 2,
            };
            signs[index] = true;
            if signs[0] && signs[2] {
                break 'controls;
            }
        }
    }
    if signs == [true, false, false] || signs == [false, false, true] {
        return Ok(SurfaceSurfaceIntersection::None);
    }
    if signs == [false, true, false] {
        return Ok(SurfaceSurfaceIntersection::ContainedSurface(
            SurfaceIntersectionOperand::Second,
        ));
    }
    let (equation, map) = pullback(plane, surface)?;
    let mut zero = true;
    for coefficient in equation.coefficients.iter().flatten() {
        if decided_order(compare_reals(
            coefficient,
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? != Ordering::Equal
        {
            zero = false;
            break;
        }
    }
    if zero {
        return Ok(SurfaceSurfaceIntersection::ContainedSurface(
            SurfaceIntersectionOperand::Second,
        ));
    }
    // Complete zeros on the unit square, including vertical components as
    // exact chords u = r (or swapped-chart branches).
    let set = super::implicit_tensor::unit_square_zeros(&equation)?;
    if set.curves.is_empty() && set.points.is_empty() {
        return Ok(SurfaceSurfaceIntersection::None);
    }
    let rectangle = hypercurve::Aabb2::from_points(&[
        CurvePoint2::new(Real::zero(), Real::zero()),
        CurvePoint2::new(Real::one(), Real::one()),
    ]);
    let rectangle = match rectangle {
        Classification::Decided(value) => value,
        Classification::Uncertain(_) => return Err(GeometryError::UnsupportedIntersection),
    };
    // UV topology alone does not rule out distinct parameter points with the
    // same spatial image. An injective plane projection certifies the whole
    // rectangle and gives planar images; otherwise an injective
    // coordinate-plane projection still certifies the tensor itself.
    let plane_injective = map.certify_injective_on(&rectangle).is_ok();
    if !plane_injective {
        let [x, y, z, weight] = surface_point::tensor_coefficients(surface);
        super::implicit_tensor::certify_injective_tensor([&x, &y, &z], &weight)?;
    }
    let tensor = Surface::rational_bezier(surface.control_points.clone(), surface.weights.clone())?;
    let mut curves = Vec::with_capacity(set.curves.len());
    let mut unmapped = Vec::new();
    for uv in &set.curves {
        let chord = matches!(uv.geometry(), Some(hypercurve::CurveGeometry2::Line(_)));
        if let Some((axis, constant)) = iso_line(uv)?
            && (chord || (!plane_injective && constant.exact_rational_ref().is_some()))
        {
            // Native chords and rational constant charts keep the cheap
            // exact iso-curve projection. Other selected constants use the
            // source-preserving map below, retaining the field relations
            // needed to re-establish their complete image after replay.
            let iso_curve = tensor.iso_curve(axis, &constant)?;
            let CurveGeometry3::RationalBezier(bezier) = &iso_curve.data.geometry else {
                return Err(GeometryError::UnsupportedIntersection);
            };
            let mut projected_points = Vec::new();
            let mut weights = Vec::new();
            for control in bezier.homogeneous_controls() {
                let w = control.w.clone();
                let point = Point3::new(
                    (&control.x / &w).map_err(GeometryError::Numeric)?,
                    (&control.y / &w).map_err(GeometryError::Numeric)?,
                    (&control.z / &w).map_err(GeometryError::Numeric)?,
                );
                let flat = project_point_to_plane_frame(&plane.origin, &plane.u, &plane.v, &point)?;
                projected_points.push(CurvePoint2::new(flat.x, flat.y));
                weights.push(w);
            }
            let projected = Curve2::from(hypercurve::RationalBezier2::try_new(
                projected_points,
                weights,
            )?);
            let section = if chord {
                // A chord has no polynomial branch: the native iso-curve and
                // its iso segment replay as an iso image.
                SurfaceIntersectionCurve::from_certified_exact_pcurves(
                    iso_curve,
                    projected,
                    uv.clone(),
                )?
            } else {
                // A branch keeps its selected graph and repeated-factor
                // multiplicity as the tensor pcurve; the spatial curve is its
                // tensor image, the same parameterization as the iso-curve.
                let curve = Curve3::from_tensor_image(TensorCurve3 {
                    surface: tensor.clone(),
                    source: uv.clone(),
                    source_scale: Real::one(),
                    source_offset: Real::zero(),
                })?;
                let mut section = SurfaceIntersectionCurve::from_certified_exact_pcurves(
                    curve,
                    projected,
                    uv.clone(),
                )?;
                section.polynomial_contact_exponent = uv
                    .polynomial_branch()
                    .map(|branch| branch.defining_exponent());
                section
            };
            curves.push(section);
            continue;
        }
        if !plane_injective && uv.polynomial_branch().is_none() {
            unmapped.push(uv.clone());
            continue;
        }
        let projected = uv.rational_image(&map)?;
        let curve = if plane_injective {
            Curve3::from_planar_image(PlanarCurve3 {
                source: projected.clone(),
                origin: plane.origin.clone(),
                u: plane.u.clone(),
                v: plane.v.clone(),
                source_scale: Real::one(),
                source_offset: Real::zero(),
            })?
        } else {
            // Injectivity of the tensor has already been certified. Its
            // restriction to the selected pullback zero set lies in this
            // plane, so projection and plane lift agree on the whole curve.
            // The plane map may fold elsewhere in the UV rectangle; retain
            // the tensor as spatial authority without claiming that this
            // rational map is an injective two-dimensional chart.
            Curve3::from_tensor_image(TensorCurve3 {
                surface: tensor.clone(),
                source: uv.clone(),
                source_scale: Real::one(),
                source_offset: Real::zero(),
            })?
        };
        let mut section =
            SurfaceIntersectionCurve::from_certified_exact_pcurves(curve, projected, uv.clone())?;
        section.polynomial_contact_exponent = uv
            .polynomial_branch()
            .map(|branch| branch.defining_exponent());
        curves.push(section);
    }
    if !unmapped.is_empty() {
        // Other source families whose rational plane images are not yet
        // admitted retain their tensor definitions and explicit unavailable
        // plane pcurves rather than fitted substitutes.
        let zeros = super::implicit_tensor::UnitSquareZeros {
            curves: unmapped,
            points: Vec::new(),
        };
        match super::implicit_tensor::tensor_image_sections(
            &tensor,
            &zeros,
            SurfaceIntersectionOperand::Second,
        )? {
            SurfaceSurfaceIntersection::Curve(curve) => curves.push(*curve),
            SurfaceSurfaceIntersection::Curves(more) => curves.extend(more),
            _ => return Err(GeometryError::UnsupportedIntersection),
        }
    }
    let plane = Surface::plane(plane.origin.clone(), plane.u.clone(), plane.v.clone())?;
    let mut points = set
        .points
        .iter()
        .map(|uv| plane.point_at(&uv.rational_image(&map)?))
        .collect::<GeometryResult<Vec<_>>>()?;
    Ok(match (points.len(), curves.len()) {
        (0, 0) => SurfaceSurfaceIntersection::None,
        (1, 0) => SurfaceSurfaceIntersection::Point(Box::new(points.pop().expect("one point"))),
        (_, 0) => SurfaceSurfaceIntersection::Points(points),
        (0, 1) => SurfaceSurfaceIntersection::Curve(Box::new(curves.pop().expect("one curve"))),
        (0, _) => SurfaceSurfaceIntersection::Curves(curves),
        _ => SurfaceSurfaceIntersection::Components(Box::new(SurfaceIntersectionComponents::new(
            points,
            Vec::new(),
            curves,
        ))),
    })
}

pub(super) fn pullback(
    plane: &PlaneSurface,
    surface: &RationalBezierSurface,
) -> GeometryResult<(BivariatePolynomial, RationalMap2)> {
    let [x, y, z, weight] = surface_point::tensor_coefficients(surface);
    let coordinates = [x, y, z];
    let normal = plane.u.cross(&plane.v);
    let displaced: [BivariatePolynomial; 3] = std::array::from_fn(|i| {
        bivariate_subtract(
            &coordinates[i],
            &bivariate_scale(
                weight.clone(),
                [&plane.origin.x, &plane.origin.y, &plane.origin.z][i],
            ),
        )
    });
    let linear = |direction: &Vector3| {
        (0..3).fold(BivariatePolynomial::new(Vec::new()), |sum, i| {
            bivariate_add(&sum, &bivariate_scale(displaced[i].clone(), &direction[i]))
        })
    };
    let equation = linear(&normal);
    // Reuse the plane-frame owner to obtain the exact dual basis. On the
    // pullback zero set, lifting these projected coordinates equals the
    // original homogeneous surface image, not an approximation of it.
    let origin = Point3::origin();
    let basis = [Vector3::x(), Vector3::y(), Vector3::z()]
        .map(|axis| project_point_to_plane_frame(&origin, &plane.u, &plane.v, &Point3::from(axis)));
    let [a, b, c] = basis;
    let [a, b, c] = [a?, b?, c?];
    let plane_x = linear(&Vector3::new([a.x, b.x, c.x]));
    let plane_y = linear(&Vector3::new([a.y, b.y, c.y]));
    let map = RationalMap2::new([plane_x, plane_y], weight);
    Ok((equation, map))
}

impl Surface {
    /// Re-establishes a retained tensor section's complete image on a plane.
    /// The selected UV branch must divide the plane pullback; projecting that
    /// same branch and lifting it then agrees with the tensor throughout it.
    pub(crate) fn certifies_tensor_section_image(
        &self,
        curve: &Curve3,
        pcurve: &Curve2,
        scale: &Real,
        offset: &Real,
    ) -> GeometryResult<bool> {
        let SurfaceGeometry::Plane(plane) = &self.data.geometry else {
            return Ok(false);
        };
        let CurveGeometry3::TensorImage(image) = &curve.data.geometry else {
            return Ok(false);
        };
        let SurfaceGeometry::RationalBezier(tensor) = &image.surface.data.geometry else {
            return Ok(false);
        };
        let Some(branch) = image.source.polynomial_branch() else {
            return Ok(false);
        };
        let (equation, map) = pullback(plane, tensor)?;
        if !branch.certifies_polynomial_identity(&equation) {
            return Ok(false);
        }
        let actual = PlanarCurve3 {
            source: image.source.rational_image(&map)?,
            origin: plane.origin.clone(),
            u: plane.u.clone(),
            v: plane.v.clone(),
            source_scale: image.source_scale.clone(),
            source_offset: image.source_offset.clone(),
        };
        let inverse = (Real::one() / scale).map_err(GeometryError::Numeric)?;
        let expected = PlanarCurve3 {
            source: pcurve.clone(),
            source_scale: inverse.clone(),
            source_offset: -offset * inverse,
            ..actual.clone()
        };
        actual.parameterization_equals(&expected)
    }

    /// Checks the entire retained polynomial branch, not a finite set of samples.
    pub(crate) fn certifies_polynomial_section_image(
        &self,
        curve: &Curve3,
        pcurve: &Curve2,
        scale: &Real,
        offset: &Real,
    ) -> GeometryResult<bool> {
        let SurfaceGeometry::RationalBezier(surface) = &self.data.geometry else {
            return Ok(false);
        };
        let CurveGeometry3::PlanarImage(image) = &curve.data.geometry else {
            return Ok(false);
        };
        let Some(branch) = pcurve.polynomial_branch() else {
            return Ok(false);
        };
        if !branch.rational_maps().is_empty() {
            return Ok(false);
        }
        if let Some(transform) = branch.image_transform() {
            // New mapped admission is the native unit-square chart exchange.
            // A polynomial identity alone does not prove that a general mapped
            // UV curve stays in this bounded patch, even with valid endpoints.
            let equals = |value: &Real, expected: i32| {
                value.exact_rational_normal_form()
                    == Real::from(expected).exact_rational_normal_form()
            };
            let (a, b, c, d, x, y) = transform.affine_components();
            if ![a, b, c, d, x, y]
                .into_iter()
                .zip([0, 1, 1, 0, 0, 0])
                .all(|(value, expected)| equals(value, expected))
                || !branch
                    .topology()
                    .domain()
                    .first
                    .iter()
                    .chain(&branch.topology().domain().second)
                    .zip([0, 1, 0, 1])
                    .all(|(value, expected)| equals(value, expected))
            {
                return Ok(false);
            }
        }
        let plane = PlaneSurface {
            origin: image.origin.clone(),
            u: image.u.clone(),
            v: image.v.clone(),
        };
        let (equation, map) = pullback(&plane, surface)?;
        if !branch.certifies_polynomial_identity(&equation) {
            return Ok(false);
        }
        let source = pcurve.rational_image(&map)?;
        let inverse = (Real::one() / scale).map_err(GeometryError::Numeric)?;
        let expected = PlanarCurve3 {
            source,
            origin: image.origin.clone(),
            u: image.u.clone(),
            v: image.v.clone(),
            source_scale: inverse.clone(),
            source_offset: -offset * inverse,
        };
        image.parameterization_equals(&expected)
    }
}

impl SurfaceIntersectionCurve {
    /// Defining-factor exponent of a checked plane/tensor pullback branch.
    ///
    /// Where both sides of a regular open arc belong to the parameter domain,
    /// even exponents keep the signed plane value on the same side and odd
    /// exponents change side. An exponent above one does not
    /// certify transversality. This is not a local order at a singular endpoint.
    /// `None` means this evidence was not established for the construction.
    /// Restrictions, reversal and operand exchange preserve it. Native pcurves
    /// retain the source polynomial and reconstruct its factor evidence on replay.
    pub const fn polynomial_contact_exponent(&self) -> Option<usize> {
        self.polynomial_contact_exponent
    }
}

#[cfg(test)]
mod chart_domain_tests {
    use super::*;
    use hypercurve::PolynomialCurveSet2;
    use hypersolve::bivariate_topology::{BivariateCurveDomain, decompose_bivariate_curve};

    #[test]
    fn mapped_polynomial_section_cannot_skip_surface_domain() {
        let surface = Surface::rational_bezier(
            vec![
                vec![
                    Point3::new(0.into(), 0.into(), 0.into()),
                    Point3::new(1.into(), 0.into(), 0.into()),
                ],
                vec![
                    Point3::new(0.into(), 1.into(), 0.into()),
                    Point3::new(1.into(), 1.into(), 0.into()),
                ],
            ],
            vec![vec![Real::one(); 2]; 2],
        )
        .unwrap();
        // v=4u(1-u) lies in the unit square. Translation keeps both endpoints
        // inside but moves its interior above v=1. The planar polynomial
        // identity alone cannot certify this bounded surface image.
        let equation = BivariatePolynomial::new(vec![
            vec![0.into(), 1.into()],
            vec![(-4).into()],
            vec![4.into()],
        ]);
        let topology = decompose_bivariate_curve(
            &equation,
            &BivariateCurveDomain {
                first: [0.into(), 1.into()],
                second: [0.into(), 2.into()],
            },
            Default::default(),
        )
        .unwrap();
        let set = PolynomialCurveSet2::from_topology(topology, Default::default()).unwrap();
        assert_eq!(set.curves().len(), 1);
        let half = (Real::one() / Real::from(2)).unwrap();
        let translation = hypercurve::Similarity2::try_from_real_affine(
            1.into(),
            0.into(),
            0.into(),
            1.into(),
            0.into(),
            half.clone(),
        )
        .unwrap();
        let pcurve = set.curves()[0].transform_similarity(&translation).unwrap();
        for endpoint in [pcurve.start(), pcurve.end()] {
            surface.point_at(&endpoint).unwrap();
        }
        let middle = pcurve.point_at(&half.into()).unwrap();
        assert!(matches!(
            surface.point_at(&middle),
            Err(GeometryError::SurfaceParameterOutsideDomain)
        ));
        let curve = Curve3::from_planar_image(PlanarCurve3 {
            source: pcurve.clone(),
            origin: Point3::origin(),
            u: Vector3::x(),
            v: Vector3::y(),
            source_scale: Real::one(),
            source_offset: Real::zero(),
        })
        .unwrap();
        assert!(
            !surface
                .certifies_polynomial_section_image(&curve, &pcurve, &Real::one(), &Real::zero())
                .unwrap()
        );
    }
}

/// Whether a unit-square zero component is a whole iso-line with a scalar
/// constant, traversed in increasing parameter over `[0, 1]`: a vertical
/// chord `u = r` (varying `v`), or a polynomial branch whose defining
/// equation depends on `v` alone (`v = c`, varying `u`), including an
/// axis exchange with the same native parameter law. Other charts return `None`.
pub(super) fn iso_line(uv: &Curve2) -> GeometryResult<Option<(SurfaceIsoAxis, Real)>> {
    let Some(chart) = iso_line_with_range(uv, true)? else {
        return Ok(None);
    };
    // This constructor uses the source parameter as the native iso parameter.
    // Other affine charts remain available to the retained-image consumer.
    if decided_order(compare_reals(
        &chart.scale,
        &Real::one(),
        crate::STRICT_PREDICATES,
    ))? != Ordering::Equal
        || decided_order(compare_reals(
            &chart.offset,
            &Real::zero(),
            crate::STRICT_PREDICATES,
        ))? != Ordering::Equal
    {
        return Ok(None);
    }
    Ok(Some((chart.axis, chart.constant)))
}

/// Exact law for an iso-line: the varying tensor coordinate is
/// `offset + scale * source_parameter` and the other is `constant`.
pub(super) struct IsoLineChart {
    pub(super) axis: SurfaceIsoAxis,
    pub(super) constant: Real,
    pub(super) scale: Real,
    pub(super) offset: Real,
}

/// Recognizes native unit vertical lines and constant-ordinate polynomial
/// branches, including axis-aligned similarities and axis exchange. Rational
/// maps and non-axis-aligned images retain their existing refusal.
pub(super) fn iso_line_with_range(
    uv: &Curve2,
    whole: bool,
) -> GeometryResult<Option<IsoLineChart>> {
    let unit = |value: &Real, expected: &Real| -> GeometryResult<bool> {
        Ok(
            decided_order(compare_reals(value, expected, crate::STRICT_PREDICATES))?
                == Ordering::Equal,
        )
    };
    if let Some(hypercurve::CurveGeometry2::Line(chord)) = uv.geometry() {
        let (start, end) = (chord.start(), chord.end());
        if unit(start.x(), end.x())?
            && unit(start.y(), &Real::zero())?
            && unit(end.y(), &Real::one())?
        {
            return Ok(Some(IsoLineChart {
                axis: SurfaceIsoAxis::V,
                constant: start.x().clone(),
                scale: Real::one(),
                offset: Real::zero(),
            }));
        }
        return Ok(None);
    }
    let Some(branch) = uv.polynomial_branch() else {
        return Ok(None);
    };
    if !branch.rational_maps().is_empty() {
        return Ok(None);
    }
    let equation = branch.topology().geometric_equation();
    for row in equation.coefficients.iter().skip(1) {
        for coefficient in row {
            if !unit(coefficient, &Real::zero())? {
                return Ok(None);
            }
        }
    }
    if whole {
        let (start, end) = uv.parameter_endpoints();
        let (Some(start), Some(end)) = (start.scalar(), end.scalar()) else {
            return Ok(None);
        };
        if !unit(start, &Real::zero())? || !unit(end, &Real::one())? {
            return Ok(None);
        }
    }
    let identity = hypercurve::Similarity2::try_from_real_affine(
        Real::one(),
        Real::zero(),
        Real::zero(),
        Real::one(),
        Real::zero(),
        Real::zero(),
    )?;
    let (a, b, c, d, x, y) = branch
        .image_transform()
        .unwrap_or(&identity)
        .affine_components();
    let (
        axis,
        varying_scale,
        varying_constant,
        varying_offset,
        constant_scale,
        constant_offset,
        selected_axis,
    ) = if unit(c, &Real::zero())? {
        (
            SurfaceIsoAxis::U,
            a,
            b,
            x,
            d,
            y,
            (Real::zero(), Real::one()),
        )
    } else if unit(a, &Real::zero())? {
        (
            SurfaceIsoAxis::V,
            c,
            d,
            y,
            b,
            x,
            (Real::one(), Real::zero()),
        )
    } else {
        return Ok(None);
    };
    // Similarities are nonsingular. The constant-coordinate coefficient is
    // nonzero, so equality here selects the original root uniquely as well.
    let selected = uv.point_at(uv.parameter_domain().start())?.affine_form(
        selected_axis.0,
        selected_axis.1,
        Real::zero(),
    );
    let Some(base_constant) = super::tensor_image::first_proved_equal(
        super::tensor_image::exact_root_candidates(equation.coefficients[0].clone())?,
        |candidate| Ok(selected.compare_to_real(&(constant_scale * candidate + constant_offset))?),
    )?
    else {
        return Ok(None);
    };
    Ok(Some(IsoLineChart {
        axis,
        constant: constant_scale * &base_constant + constant_offset,
        scale: varying_scale.clone(),
        offset: varying_constant * base_constant + varying_offset,
    }))
}

#[cfg(test)]
mod replay_proof_tests {
    use super::*;

    fn polynomial(rows: &[&[i32]]) -> BivariatePolynomial {
        BivariatePolynomial::new(
            rows.iter()
                .map(|row| row.iter().copied().map(Real::from).collect())
                .collect(),
        )
    }

    #[test]
    fn tensor_plane_agreement_checks_mapped_branches_and_complete_images() {
        use hypersolve::bivariate_topology::{
            BivariateCurveDomain, BivariateTopologyConfig, decompose_bivariate_curve,
        };
        let topology = decompose_bivariate_curve(
            &polynomial(&[&[0, 1], &[], &[-1]]),
            &BivariateCurveDomain {
                first: [0.into(), 1.into()],
                second: [0.into(), 1.into()],
            },
            BivariateTopologyConfig::default(),
        )
        .unwrap();
        let set = hypercurve::PolynomialCurveSet2::from_topology(
            topology,
            hypersolve::AlgebraicFiberRootIsolationConfig::default(),
        )
        .unwrap();
        // The retained swap sends (t,t^2) to UV=(t^2,t).
        let source = set.curves()[0]
            .rational_image(&RationalMap2::new(
                [polynomial(&[&[0, 1]]), polynomial(&[&[], &[1]])],
                polynomial(&[&[1]]),
            ))
            .unwrap();
        let half = (Real::one() / Real::from(2)).unwrap();
        // S(u,v)=(u,v,u-v^2): every mapped source point lies in z=0.
        // Tensor control rows are v, with u across each row.
        let tensor = Surface::rational_bezier(
            vec![
                vec![Point3::origin(), Point3::new(1.into(), 0.into(), 1.into())],
                vec![
                    Point3::new(0.into(), half.clone(), 0.into()),
                    Point3::new(1.into(), half.clone(), 1.into()),
                ],
                vec![
                    Point3::new(0.into(), 1.into(), (-1).into()),
                    Point3::new(1.into(), 1.into(), 0.into()),
                ],
            ],
            vec![vec![Real::one(); 2]; 3],
        )
        .unwrap();
        let curve = Curve3::from_tensor_image(TensorCurve3 {
            surface: tensor,
            source: source.clone(),
            source_scale: half.clone(),
            source_offset: half,
        })
        .unwrap();
        let plane = Surface::plane(Point3::origin(), Vector3::x(), Vector3::y()).unwrap();
        assert!(
            plane
                .certifies_tensor_section_image(&curve, &source, &2.into(), &(-1).into())
                .unwrap()
        );
        assert!(
            !plane
                .certifies_tensor_section_image(&curve, &source, &1.into(), &0.into())
                .unwrap()
        );
        let displaced = Surface::plane(
            Point3::new(0.into(), 0.into(), 1.into()),
            Vector3::x(),
            Vector3::y(),
        )
        .unwrap();
        assert!(
            !displaced
                .certifies_tensor_section_image(&curve, &source, &2.into(), &(-1).into())
                .unwrap()
        );
        // Endpoint agreement does not prove the interior: (X,Y+X-X^2).
        let wrong = source
            .rational_image(&RationalMap2::new(
                [
                    polynomial(&[&[], &[1]]),
                    polynomial(&[&[0, 1], &[1], &[-1]]),
                ],
                polynomial(&[&[1]]),
            ))
            .unwrap();
        assert!(source.start().coincides_with(&wrong.start()).unwrap());
        assert!(source.end().coincides_with(&wrong.end()).unwrap());
        assert!(
            !plane
                .certifies_tensor_section_image(&curve, &wrong, &2.into(), &(-1).into())
                .unwrap()
        );
    }
}
