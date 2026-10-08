    fn contains_parameter(
        self,
        parameter: &CurveParameter2,
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        match self {
            Self::Finite(range) => CurveParameterDomain2::new(range, None)
                .contains_finite_parameter(parameter, policy),
            Self::AffineLine => Ok(Classification::Decided(true)),
            Self::IncidentRay { anchor, direction, barrier } => {
                let wanted = match direction {
                    BezierParameterRayDirection2::Increasing => std::cmp::Ordering::Greater,
                    BezierParameterRayDirection2::Decreasing => std::cmp::Ordering::Less,
                };
                match parameter.cmp_by_refinement(&CurveParameter2::from(anchor.clone()), policy)? {
                    Classification::Decided(order) if order != wanted => return Ok(Classification::Decided(false)),
                    Classification::Decided(_) => {}
                    Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
                }
                if let Some(barrier) = barrier {
                    return Ok(parameter.cmp_by_refinement(&CurveParameter2::from(barrier.clone()), policy)?
                        .map(|order| order == wanted.reverse()));
                }
                Ok(Classification::Decided(true))
            }
        }
    }

    fn polynomial_is_nonzero(
        self,
        coefficients: &[Real],
        policy: &CurveContext,
    ) -> CurveResult<Classification<bool>> {
        if let Self::Finite(range) = self {
            return polynomial_is_nonzero_on_parameter_range(coefficients, range, policy);
        }
        let polynomial = match polynomial_from_coefficients(coefficients.to_vec(), policy)? {
            Classification::Decided(Some(polynomial)) => polynomial,
            Classification::Decided(None) => return Ok(Classification::Decided(false)),
            Classification::Uncertain(reason) => return Ok(Classification::Uncertain(reason)),
        };
        Ok(self.isolate(&polynomial, policy)?.map(|roots| roots.is_empty()))
    }
