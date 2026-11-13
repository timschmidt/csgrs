//! Field interpretation of Hypersolve's exact-coefficient Taylor models.
use super::*;
use hypersolve::expression_program::{TaylorError, TaylorModel, TaylorRequest};

/// Finite time/state domain and total polynomial degree.
///
/// The center is the exact midpoint of each nonempty input interval, in source
/// binding order. A midpoint that cannot be represented fails explicitly.
/// Replay this request and the original field definition to reconstruct evidence.
#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct FieldTaylorRequest<const N: usize> {
    /// Original physical time domain.
    pub time: DyadicInterval,
    /// Original state domain in field coordinate order.
    pub state: BoxN<DyadicInterval, N>,
    /// Total degree retained; the next order bounds the entire omitted remainder.
    pub order: usize,
}

/// Field interpretation, model construction or subdomain reuse failure.
#[derive(Clone, Debug)]
pub enum FieldTaylorError {
    /// Original binding, resource or native carrier diagnostic.
    Field(FieldError),
    /// Original coefficient, regularity, domain or resource diagnostic.
    Taylor(TaylorError),
    /// A nonempty domain cannot reuse a model of an empty product.
    EmptyDomain,
}
impl fmt::Display for FieldTaylorError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "field Taylor enclosure: {self:?}")
    }
}
impl std::error::Error for FieldTaylorError {
    fn source(&self) -> Option<&(dyn std::error::Error + 'static)> {
        match self {
            Self::Field(error) => Some(error),
            Self::Taylor(error) => Some(error),
            Self::EmptyDomain => None,
        }
    }
}
impl From<FieldError> for FieldTaylorError {
    fn from(error: FieldError) -> Self {
        Self::Field(error)
    }
}
impl From<TaylorError> for FieldTaylorError {
    fn from(error: TaylorError) -> Self {
        Self::Taylor(error)
    }
}

/// Immutable source-bound field image with exact coefficients and full remainder.
///
/// Hypersolve owns the coefficient and remainder proof. This adapter retains the
/// original time/state interpretation and Cartesian empty/unit semantics. It is
/// a field image, not an ODE existence or flow certificate. Reuse keeps the
/// model's original grid and global remainder. The budget's work/size limits still
/// apply; its precision does not rewrite a previously constructed model.
#[derive(Clone, Debug)]
pub struct FieldTaylorModel<const N: usize> {
    field: ExpressionField<N>,
    request: FieldTaylorRequest<N>,
    model: Option<TaylorModel>,
    image: IntervalVector<DyadicInterval, N>,
}
impl<const N: usize> FieldTaylorModel<N> {
    /// Original exact source and time/state interpretation.
    pub fn identity(&self) -> FieldIdentity {
        self.field.identity()
    }
    /// Original physical input domain and requested degree.
    pub fn request(&self) -> &FieldTaylorRequest<N> {
        &self.request
    }
    /// Complete coefficient/remainder evidence; absent only for an empty product.
    pub fn model(&self) -> Option<&TaylorModel> {
        self.model.as_ref()
    }
    /// Certified field image, preserving zero-dimensional empty/unit distinction.
    pub fn image(&self) -> &IntervalVector<DyadicInterval, N> {
        &self.image
    }
    /// Enclose a subdomain using the retained model, global remainder and grid.
    /// Empty products remain valid even when another coordinate lies outside.
    pub fn enclose_on(
        &self,
        time: &DyadicInterval,
        state: &BoxN<DyadicInterval, N>,
        budget: &mut EnclosureBudget,
    ) -> Result<IntervalVector<DyadicInterval, N>, FieldTaylorError> {
        let source = self.field.data.jacobian.source();
        let Some(domain) = self.field.arguments(source, time, state, budget)? else {
            let image = IntervalVector::empty().map_err(FieldError::from)?;
            budget.graph.charge(0).map_err(FieldError::from)?;
            return Ok(image);
        };
        let model = self.model.as_ref().ok_or(FieldTaylorError::EmptyDomain)?;
        let values = model.enclose_on(&domain, &mut budget.graph, &mut budget.arithmetic)?;
        vector(values, budget).map_err(Into::into)
    }
}
impl<const N: usize> ExpressionField<N> {
    /// Construct a centered exact polynomial plus certified remainder field image.
    ///
    /// The caller's one graph/native arithmetic budget covers preparation,
    /// coefficient construction, proof and materialization. Existing scalar
    /// payload/refinement, heap-byte and within-scalar cancellation exclusions
    /// remain those of `EnclosureBudget`. No default field or flow algorithm is
    /// replaced. Source guards and stronger Taylor regularity remain mandatory.
    pub fn taylor_model(
        &self,
        request: FieldTaylorRequest<N>,
        budget: &mut EnclosureBudget,
    ) -> Result<FieldTaylorModel<N>, FieldTaylorError> {
        let source = self.data.jacobian.source();
        let domain = self.arguments(source, &request.time, &request.state, budget)?;
        let (model, image) = if let Some(domain) = domain {
            budget
                .graph
                .check_entries(&[domain.len(), domain.len()])
                .map_err(FieldError::from)?;
            let mut center = Vec::new();
            for interval in &domain {
                budget.graph.charge(1).map_err(FieldError::from)?;
                center.push(
                    interval
                        .midpoint(&mut budget.arithmetic)
                        .map_err(FieldError::from)?,
                );
            }
            let model = source.taylor_model(
                TaylorRequest {
                    center,
                    domain,
                    order: request.order,
                    precision: budget.policy.precision,
                },
                &mut budget.graph,
                &mut budget.arithmetic,
            )?;
            budget.graph.charge(N).map_err(FieldError::from)?;
            let image = vector(model.image().to_vec(), budget)?;
            (Some(model), image)
        } else {
            (None, IntervalVector::empty().map_err(FieldError::from)?)
        };
        let result = FieldTaylorModel {
            field: self.clone(),
            request,
            model,
            image,
        };
        budget.graph.charge(0).map_err(FieldError::from)?;
        Ok(result)
    }
}
fn vector<const N: usize>(
    values: Vec<DyadicInterval>,
    budget: &mut EnclosureBudget,
) -> Result<IntervalVector<DyadicInterval, N>, FieldError> {
    budget.graph.charge(N)?;
    let lanes = values
        .try_into()
        .map_err(|values: Vec<_>| FieldError::OutputCount {
            expected: N,
            actual: values.len(),
        })?;
    let image = IntervalVector::new(lanes)?;
    budget.graph.charge(0)?;
    Ok(image)
}
