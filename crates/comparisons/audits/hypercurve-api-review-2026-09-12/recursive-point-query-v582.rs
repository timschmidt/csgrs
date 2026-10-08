    /// The Cartesian incidence theorem also holds in a point's retained
    /// coefficient field. Homogeneous coordinates preserve their correlation;
    /// the common polynomial factor is the contact proof, and one oriented
    /// normal sign selects the authored offset without adjoining a speed root.
    fn visit_recursive_point_parameters(
        &self,
        point: &CurvePoint2,
        incident: Option<&BezierParallelIncidentDomain2>,
        domain: CurveParameterDomain2<'_>,
        policy: &CurveContext,
        visitor: &mut impl FnMut(Option<&CurveParameter2>) -> ControlFlow<()>,
    ) -> CurveResult<Classification<ControlFlow<()>>> {
        let strict = policy.strict_counterpart();
        let point = match recursive_projective_evidence_points(&[point], &strict)? {
            Classification::Decided(Some(mut points)) => points.remove(0),
            Classification::Decided(None) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
            }
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        let field = point.denominator.field();
        let source = self.parallel.source_power_basis()?;
        let unit = [Real::one()];
        let weight = source.weight.unwrap_or(&unit);
        let finite = SelectedThirdAxisDomain2::Finite(domain.finite);
        let extension = incident.map(|incident| SelectedThirdAxisDomain2::IncidentRay {
            anchor: incident.anchor(),
            direction: incident.direction(),
            barrier: incident.barrier(),
        });
        for axis in std::iter::once(finite).chain(extension) {
            match axis.polynomial_is_nonzero(weight, &strict)? {
                Classification::Decided(true) => {}
                Classification::Decided(false) => {
                    return Ok(Classification::Uncertain(UncertaintyReason::Boundary));
                }
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            }
        }
        let distance_sign = match real_sign(self.parallel.distance(), &strict) {
            Some(sign) => sign,
            None => return Ok(Classification::Uncertain(UncertaintyReason::RealSign)),
        };
        let tangent = if distance_sign == RealSign::Zero {
            None
        } else {
            let differential = self.parallel.differential()?;
            Some(self.frame.map(|frame| (&frame.x[..], &frame.y[..]))
                .unwrap_or((&differential.tangent_x, &differential.tangent_y)))
        };
        let Some((equations, normal)) = (|| {
            let real = |values: &[Real]| recursive_quadratic_real_polynomial(&field, values);
            let multiply = recursive_quadratic_polynomial_multiply;
            let scale = recursive_quadratic_polynomial_scale;
            let add = |a: &[_], b: &[_]| recursive_quadratic_polynomial_combine(a, b, false);
            let subtract = |a: &[_], b: &[_]| recursive_quadratic_polynomial_combine(a, b, true);
            let weight = real(weight)?;
            // delta = D*W*(point - source), for the retained point (X/D,Y/D).
            let delta_x = subtract(&scale(&weight, &point.x)?, &scale(&real(source.x_numerator)?, &point.denominator)?)?;
            let delta_y = subtract(&scale(&weight, &point.y)?, &scale(&real(source.y_numerator)?, &point.denominator)?)?;
            let Some((x, y)) = tangent else {
                return Some(([delta_x, delta_y], None));
            };
            let speed = polynomial_add(&polynomial_multiply(x, x), &polynomial_multiply(y, y));
            let x = real(x)?;
            let y = real(y)?;
            let orthogonality = add(&multiply(&delta_x, &x)?, &multiply(&delta_y, &y)?)?;
            let weighted_distance = recursive_quadratic_polynomial_scale_real(
                &scale(&weight, &point.denominator)?, self.parallel.distance(),
            )?;
            let distance = subtract(
                &add(&multiply(&delta_x, &delta_x)?, &multiply(&delta_y, &delta_y)?)?,
                &multiply(&weighted_distance, &weighted_distance)?,
            )?;
            let orientation = multiply(
                &subtract(&multiply(&delta_y, &x)?, &multiply(&delta_x, &y)?)?,
                &weighted_distance,
            )?;
            Some(([orthogonality, distance], Some((speed, orientation))))
        })() else {
            return Ok(Classification::Uncertain(UncertaintyReason::Unsupported));
        };
        let common = match hypersolve::ordered_field_polynomial_gcd(
            &equations[0], &equations[1],
            &mut BezierRecursiveOrderedFieldContext2 { field: field.clone(), policy: strict },
        ) {
            Ok(common) => common,
            Err(BezierRecursiveOrderedFieldError2::Curve(error)) => return Err(error),
            Err(BezierRecursiveOrderedFieldError2::Uncertain) => {
                return Ok(Classification::Uncertain(UncertaintyReason::Predicate));
            }
        };
        if common.is_empty() {
            if let Some((_, orientation)) = &normal {
                // Both equations are polynomial identities. On this connected
                // source domain, nonzero weight, distance and speed make the
                // branch continuous and nonzero; one strict sign chooses the
                // whole authored sheet, including its incident continuation.
                for axis in std::iter::once(finite).chain(extension) {
                    if let Classification::Uncertain(reason) = self.parallel
                        .certify_source_frame_in_domain(axis, self.frame, &strict)? {
                        return Ok(Classification::Uncertain(reason));
                    }
                }
                match recursive_projective_polynomial_sign_at_parameter(&field, orientation, domain.finite.start(), &strict)? {
                    Classification::Decided(RealSign::Positive) => {}
                    Classification::Decided(RealSign::Negative) => return Ok(Classification::Decided(ControlFlow::Continue(()))),
                    Classification::Decided(RealSign::Zero) => return Err(CurveError::Topology("a regular collapsed parallel lost its normal branch".into())),
                    Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
                }
            }
            return Ok(Classification::Decided(visitor(None)));
        }
        for axis in std::iter::once(finite).chain(extension) {
            let candidates = match recursive_projective_polynomial_parameters(&field, common.clone(), axis, &strict)? {
                Classification::Decided(candidates) => candidates,
                Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
            };
            for candidate in candidates {
                match CurveParameterDomain2::new(self.range, None).contains_finite_parameter(&candidate, &strict)? {
                    Classification::Decided(true) => {}
                    Classification::Decided(false) => {
                        let Some(incident) = incident else { continue; };
                        match incident.contains_extension_parameter(&candidate, &strict)? {
                            Classification::Decided(true) => {}
                            Classification::Decided(false) => continue,
                            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
                        }
                    }
                    Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
                }
                if let Some((speed, orientation)) = &normal {
                    match candidate.polynomial_sign(speed, &strict)? {
                        Classification::Decided(RealSign::Positive) => {}
                        Classification::Decided(RealSign::Zero) => return Ok(Classification::Uncertain(UncertaintyReason::Boundary)),
                        Classification::Decided(RealSign::Negative) => return Err(CurveError::Topology("a recursive point incidence had negative squared speed".into())),
                        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
                    }
                    match recursive_projective_polynomial_sign_at_parameter(&field, orientation, &candidate, &strict)? {
                        Classification::Decided(RealSign::Positive) => {}
                        Classification::Decided(RealSign::Negative) => continue,
                        Classification::Decided(RealSign::Zero) => return Err(CurveError::Topology("a regular point incidence lost its normal branch".into())),
                        Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
                    }
                }
                if let stop @ ControlFlow::Break(()) = visitor(Some(&candidate)) {
                    return Ok(Classification::Decided(stop));
                }
            }
        }
        Ok(Classification::Decided(ControlFlow::Continue(())))
    }
