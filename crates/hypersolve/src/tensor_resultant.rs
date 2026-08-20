//! Exact constrained-axis resultants for dense polynomial tensors.
//!
//! The curve-specific resultant helpers cover two, three, and four variables
//! with convenient typed tensors.  Exact recursive constructions eventually
//! need one more selected source field, however, and duplicating another fixed
//! rank only postpones the same boundary.  This module supplies the rank-
//! independent primitive: eliminate any tensor axis against one exact
//! univariate constraint, retaining every other axis and every shared-root
//! correlation.
//!
//! Resultants are evaluated on an exact integer tensor grid and reconstructed
//! by exact univariate interpolation along each retained axis.  There is no
//! mathematical degree cap.  Checked dimension arithmetic and fallible
//! reservations turn unrepresentable host sizes into explicit statuses;
//! coefficients are never sampled through primitive floating point.

use hyperreal::{CertifiedRealSign, Real, RealSign, ZeroKnowledge};

use crate::resultant::quotient_ring_fiber_resultant_polynomial;
use crate::resultant::{UnivariateResultantError, resultant_univariate_polynomials};
use crate::root_isolation::polynomial_div_rem;

/// Dense ascending-power polynomial tensor with row-major coefficients.
///
/// `dimensions[axis]` is one more than the represented power bound on that
/// axis.  The last axis is contiguous.  A rank-zero tensor is one scalar and
/// therefore has exactly one coefficient.
#[derive(Clone, Debug, PartialEq)]
pub struct DenseTensorPolynomial {
    dimensions: Vec<usize>,
    coefficients: Vec<Real>,
}

impl DenseTensorPolynomial {
    /// Constructs a dense tensor after checked shape validation.
    pub fn try_new(dimensions: Vec<usize>, coefficients: Vec<Real>) -> Option<Self> {
        if dimensions.contains(&0) {
            return None;
        }
        let count = dimensions
            .iter()
            .try_fold(1_usize, |count, dimension| count.checked_mul(*dimension))?;
        (count == coefficients.len()).then_some(Self {
            dimensions,
            coefficients,
        })
    }

    /// Returns the tensor rank and per-axis coefficient counts.
    pub fn dimensions(&self) -> &[usize] {
        &self.dimensions
    }

    /// Returns the flat row-major coefficient storage.
    pub fn coefficients(&self) -> &[Real] {
        &self.coefficients
    }

    /// Returns one coefficient by ascending-power exponent tuple.
    pub fn coefficient(&self, exponents: &[usize]) -> Option<&Real> {
        (exponents.len() == self.dimensions.len()
            && exponents
                .iter()
                .zip(&self.dimensions)
                .all(|(exponent, dimension)| exponent < dimension))
        .then(|| &self.coefficients[flat_index(&self.dimensions, exponents)])
    }

    /// Constructs the zero polynomial with one explicit dense shape.
    pub fn zero(dimensions: Vec<usize>) -> Option<Self> {
        let count = checked_coefficient_count(&dimensions)?;
        let mut coefficients = Vec::new();
        coefficients.try_reserve_exact(count).ok()?;
        coefficients.resize(count, Real::zero());
        Self::try_new(dimensions, coefficients)
    }

    /// Embeds one univariate power-basis polynomial on a selected tensor axis.
    pub fn from_axis_polynomial(rank: usize, axis: usize, coefficients: &[Real]) -> Option<Self> {
        if rank == 0 || axis >= rank || coefficients.is_empty() {
            return None;
        }
        let mut dimensions = vec![1; rank];
        dimensions[axis] = coefficients.len();
        let mut polynomial = Self::zero(dimensions.clone())?;
        for (power, coefficient) in coefficients.iter().enumerate() {
            let mut exponent = vec![0; rank];
            exponent[axis] = power;
            polynomial.coefficients[flat_index(&dimensions, &exponent)] = coefficient.clone();
        }
        Some(polynomial)
    }

    /// Adds another tensor polynomial of the same rank.
    pub fn add(&self, other: &Self) -> Option<Self> {
        self.combine(other, false)
    }

    /// Subtracts another tensor polynomial of the same rank.
    pub fn subtract(&self, other: &Self) -> Option<Self> {
        self.combine(other, true)
    }

    /// Scales every coefficient by one exact scalar.
    pub fn scale(&self, scale: &Real) -> Option<Self> {
        let mut coefficients = Vec::new();
        coefficients
            .try_reserve_exact(self.coefficients.len())
            .ok()?;
        coefficients.extend(
            self.coefficients
                .iter()
                .map(|coefficient| coefficient * scale),
        );
        Self::try_new(self.dimensions.clone(), coefficients)
    }

    /// Multiplies two tensors by exact multidimensional convolution.
    pub fn multiply(&self, other: &Self) -> Option<Self> {
        if self.dimensions.len() != other.dimensions.len() {
            return None;
        }
        let dimensions = self
            .dimensions
            .iter()
            .zip(&other.dimensions)
            .map(|(left, right)| left.checked_add(*right)?.checked_sub(1))
            .collect::<Option<Vec<_>>>()?;
        let mut result = Self::zero(dimensions.clone())?;
        for (left_index, left) in self.coefficients.iter().enumerate() {
            if left.zero_status() == ZeroKnowledge::Zero {
                continue;
            }
            let left_exponents = exponents(&self.dimensions, left_index);
            for (right_index, right) in other.coefficients.iter().enumerate() {
                if right.zero_status() == ZeroKnowledge::Zero {
                    continue;
                }
                let right_exponents = exponents(&other.dimensions, right_index);
                let product_exponents = left_exponents
                    .iter()
                    .zip(right_exponents)
                    .map(|(left, right)| left + right)
                    .collect::<Vec<_>>();
                result.coefficients[flat_index(&dimensions, &product_exponents)] += left * right;
            }
        }
        Some(result)
    }

    /// Multiplies by one power of a selected tensor variable.
    pub fn shift_axis(&self, axis: usize, power: usize) -> Option<Self> {
        if axis >= self.dimensions.len() {
            return None;
        }
        if power == 0 {
            return Some(self.clone());
        }
        let mut dimensions = self.dimensions.clone();
        dimensions[axis] = dimensions[axis].checked_add(power)?;
        let mut result = Self::zero(dimensions.clone())?;
        for (index, coefficient) in self.coefficients.iter().enumerate() {
            let mut target = exponents(&self.dimensions, index);
            target[axis] += power;
            result.coefficients[flat_index(&dimensions, &target)] = coefficient.clone();
        }
        Some(result)
    }

    /// Substitutes one tensor variable for another equal selected variable.
    ///
    /// Powers on `removed_axis` are added to powers on `retained_axis`, then
    /// the removed axis disappears. This is exact diagonal substitution and
    /// lets represented-image callers avoid eliminating duplicate evidence
    /// for the same uniquely selected algebraic number.
    pub fn substitute_equal_axes(&self, retained_axis: usize, removed_axis: usize) -> Option<Self> {
        if retained_axis >= self.dimensions.len()
            || removed_axis >= self.dimensions.len()
            || retained_axis == removed_axis
        {
            return None;
        }
        let merged_dimension = self.dimensions[retained_axis]
            .checked_add(self.dimensions[removed_axis])?
            .checked_sub(1)?;
        let target_axis = if removed_axis < retained_axis {
            retained_axis - 1
        } else {
            retained_axis
        };
        let mut dimensions = self.dimensions.clone();
        dimensions.remove(removed_axis);
        dimensions[target_axis] = merged_dimension;
        let mut result = Self::zero(dimensions.clone())?;
        for (index, coefficient) in self.coefficients.iter().enumerate() {
            let mut target = exponents(&self.dimensions, index);
            let merged_power = target[retained_axis].checked_add(target[removed_axis])?;
            target.remove(removed_axis);
            target[target_axis] = merged_power;
            result.coefficients[flat_index(&dimensions, &target)] += coefficient;
        }
        Some(result)
    }

    /// Substitutes `removed = scale * retained + offset` and removes that
    /// affine-related tensor axis.
    ///
    /// This is the general exact counterpart of diagonal substitution. It is
    /// used only after an external algebraic-root certificate has proved the
    /// affine relation; the tensor operation itself performs the complete
    /// binomial expansion without sampling or coefficient approximation.
    pub fn substitute_affine_axis(
        &self,
        retained_axis: usize,
        removed_axis: usize,
        scale: &Real,
        offset: &Real,
    ) -> Option<Self> {
        if retained_axis >= self.dimensions.len()
            || removed_axis >= self.dimensions.len()
            || retained_axis == removed_axis
        {
            return None;
        }
        let merged_dimension = self.dimensions[retained_axis]
            .checked_add(self.dimensions[removed_axis])?
            .checked_sub(1)?;
        let target_axis = if removed_axis < retained_axis {
            retained_axis - 1
        } else {
            retained_axis
        };
        let mut dimensions = self.dimensions.clone();
        let removed_dimension = dimensions.remove(removed_axis);
        dimensions[target_axis] = merged_dimension;
        let mut result = Self::zero(dimensions.clone())?;

        let mut scale_powers = Vec::new();
        let mut offset_powers = Vec::new();
        scale_powers.try_reserve_exact(removed_dimension).ok()?;
        offset_powers.try_reserve_exact(removed_dimension).ok()?;
        scale_powers.push(Real::one());
        offset_powers.push(Real::one());
        for power in 1..removed_dimension {
            scale_powers.push(&scale_powers[power - 1] * scale);
            offset_powers.push(&offset_powers[power - 1] * offset);
        }
        let mut binomial_rows = Vec::new();
        binomial_rows.try_reserve_exact(removed_dimension).ok()?;
        binomial_rows.push(vec![Real::one()]);
        for power in 1..removed_dimension {
            let previous = &binomial_rows[power - 1];
            let mut row = Vec::new();
            row.try_reserve_exact(power + 1).ok()?;
            row.push(Real::one());
            for index in 1..power {
                row.push(&previous[index - 1] + &previous[index]);
            }
            row.push(Real::one());
            binomial_rows.push(row);
        }

        for (index, coefficient) in self.coefficients.iter().enumerate() {
            let source = exponents(&self.dimensions, index);
            let removed_power = source[removed_axis];
            for retained_power in 0..=removed_power {
                let mut target = source.clone();
                let merged_power = target[retained_axis].checked_add(retained_power)?;
                target.remove(removed_axis);
                target[target_axis] = merged_power;
                result.coefficients[flat_index(&dimensions, &target)] += coefficient
                    * &binomial_rows[removed_power][retained_power]
                    * &scale_powers[retained_power]
                    * &offset_powers[removed_power - retained_power];
            }
        }
        Some(result)
    }

    /// Removes an axis whose every positive-power coefficient is certified
    /// zero.
    ///
    /// `None` means either that the axis is invalid or that exact coefficient
    /// certification did not prove independence. No uncertain coefficient is
    /// discarded. This is useful when quotient or affine substitution has
    /// made a selected source irrelevant to the remaining relation.
    pub fn remove_certified_independent_axis(
        &self,
        axis: usize,
        min_precision: i32,
    ) -> Option<Self> {
        if axis >= self.dimensions.len() {
            return None;
        }
        for (index, coefficient) in self.coefficients.iter().enumerate() {
            if exponents(&self.dimensions, index)[axis] == 0 {
                continue;
            }
            if !matches!(
                coefficient.certified_sign_until(min_precision),
                CertifiedRealSign::Known {
                    sign: RealSign::Zero,
                    ..
                }
            ) {
                return None;
            }
        }
        let mut dimensions = self.dimensions.clone();
        dimensions.remove(axis);
        let mut result = Self::zero(dimensions.clone())?;
        for (index, coefficient) in self.coefficients.iter().enumerate() {
            let mut source = exponents(&self.dimensions, index);
            if source[axis] != 0 {
                continue;
            }
            source.remove(axis);
            result.coefficients[flat_index(&dimensions, &source)] = coefficient.clone();
        }
        Some(result)
    }

    /// Replaces every selected-axis fiber by its exact polynomial remainder.
    ///
    /// At any root of `modulus`, the returned tensor has exactly the same
    /// value as `self`. Reducing before a constrained resultant is therefore
    /// quotient-ring canonicalization, not approximation, and prevents powers
    /// already implied by selected-root evidence from inflating every later
    /// retained-axis degree.
    pub fn reduce_axis_modulo(
        &self,
        axis: usize,
        modulus: &[Real],
        policy: hyperlimit::PredicatePolicy,
    ) -> Option<Self> {
        if axis >= self.dimensions.len() || modulus.len() <= 1 {
            return None;
        }
        let target_axis_dimension = modulus.len() - 1;
        let mut dimensions = self.dimensions.clone();
        dimensions[axis] = target_axis_dimension;
        let mut reduced = Self::zero(dimensions.clone())?;
        let retained_dimensions = self
            .dimensions
            .iter()
            .enumerate()
            .filter_map(|(source_axis, dimension)| (source_axis != axis).then_some(*dimension))
            .collect::<Vec<_>>();
        let fiber_count = checked_coefficient_count(&retained_dimensions)?;
        for fiber_index in 0..fiber_count {
            let retained_exponents = exponents(&retained_dimensions, fiber_index);
            let mut source_exponents = vec![0; self.dimensions.len()];
            let mut retained = 0;
            for (source_axis, exponent) in source_exponents.iter_mut().enumerate() {
                if source_axis == axis {
                    continue;
                }
                *exponent = retained_exponents[retained];
                retained += 1;
            }
            let mut fiber = Vec::new();
            fiber.try_reserve_exact(self.dimensions[axis]).ok()?;
            for power in 0..self.dimensions[axis] {
                source_exponents[axis] = power;
                fiber.push(
                    self.coefficient(&source_exponents)
                        .cloned()
                        .unwrap_or_else(Real::zero),
                );
            }
            let (_, remainder) = polynomial_div_rem(fiber, modulus, policy)?;
            for (power, coefficient) in remainder.into_iter().enumerate() {
                if power >= target_axis_dimension {
                    return None;
                }
                source_exponents[axis] = power;
                let target = flat_index(&dimensions, &source_exponents);
                reduced.coefficients[target] = coefficient;
            }
        }
        Some(reduced)
    }

    fn combine(&self, other: &Self, subtract: bool) -> Option<Self> {
        if self.dimensions.len() != other.dimensions.len() {
            return None;
        }
        let dimensions = self
            .dimensions
            .iter()
            .zip(&other.dimensions)
            .map(|(left, right)| (*left).max(*right))
            .collect::<Vec<_>>();
        let mut result = Self::zero(dimensions.clone())?;
        for (source, subtract_source) in [(self, false), (other, subtract)] {
            for (index, coefficient) in source.coefficients.iter().enumerate() {
                let target = flat_index(&dimensions, &exponents(&source.dimensions, index));
                if subtract_source {
                    result.coefficients[target] -= coefficient;
                } else {
                    result.coefficients[target] += coefficient;
                }
            }
        }
        Some(result)
    }
}

/// Status for one exact constrained tensor-axis resultant.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum TensorConstraintResultantStatus {
    /// The exact retained-axis tensor was reconstructed.
    Constructed,
    /// The input tensor shape was invalid for axis elimination.
    InvalidAxis,
    /// A coefficient sign needed for exact degree certification was undecided.
    UndecidedCoefficient,
    /// The supplied univariate constraint was empty or constant.
    InvalidConstraint,
    /// Retained tensor dimensions overflowed the host index type.
    DimensionOverflow,
    /// A required dense tensor allocation could not be reserved.
    AllocationFailed,
    /// One sampled exact univariate resultant failed.
    ResultantError,
    /// Exact retained-axis interpolation failed.
    InterpolationDivisionFailed,
}

/// Exact report for eliminating one dense tensor axis.
#[derive(Clone, Debug, PartialEq)]
pub struct TensorConstraintResultantReport {
    /// Final construction status.
    pub status: TensorConstraintResultantStatus,
    /// Removed source axis.
    pub eliminated_axis: usize,
    /// Conservative power degree bounds on the retained axes.
    pub degree_bounds: Vec<usize>,
    /// Exact retained-axis resultant when construction succeeded.
    pub resultant: Option<DenseTensorPolynomial>,
    /// Sampled univariate resultant error, when applicable.
    pub resultant_error: Option<UnivariateResultantError>,
    /// Compact diagnostic reason.
    pub message: Option<String>,
}

/// Eliminates one tensor axis constrained to a univariate algebraic root.
///
/// The constraint is normalized to monic form before sampling so a fiber that
/// drops degree at an interpolation point retains the same resultant scale.
/// The output axis order is the input order with `eliminated_axis` removed.
/// Degree growth is not artificially capped; impossible host dimensions and
/// failed reservations are reported explicitly.
pub fn resultant_tensor_polynomial_univariate_constraint(
    polynomial: &DenseTensorPolynomial,
    constraint: &[Real],
    eliminated_axis: usize,
    min_precision: i32,
) -> TensorConstraintResultantReport {
    let report = |status, degree_bounds, resultant, resultant_error, message| {
        TensorConstraintResultantReport {
            status,
            eliminated_axis,
            degree_bounds,
            resultant,
            resultant_error,
            message,
        }
    };
    if polynomial.dimensions.is_empty() || eliminated_axis >= polynomial.dimensions.len() {
        return report(
            TensorConstraintResultantStatus::InvalidAxis,
            Vec::new(),
            None,
            None,
            Some("tensor resultant requires one valid eliminated axis".to_owned()),
        );
    }
    let constraint = match normalized_constraint(constraint, min_precision) {
        Ok(Some(constraint)) => constraint,
        Ok(None) => {
            return report(
                TensorConstraintResultantStatus::InvalidConstraint,
                Vec::new(),
                None,
                None,
                Some("tensor resultant requires a nonconstant constraint".to_owned()),
            );
        }
        Err(()) => {
            return report(
                TensorConstraintResultantStatus::UndecidedCoefficient,
                Vec::new(),
                None,
                None,
                Some("constraint degree or monic normalization was undecided".to_owned()),
            );
        }
    };
    let degrees = match certified_tensor_degrees(polynomial, min_precision) {
        Ok(Some(degrees)) => degrees,
        Ok(None) => {
            let retained_rank = polynomial.dimensions.len() - 1;
            let dimensions = vec![1; retained_rank];
            let zero = DenseTensorPolynomial::try_new(dimensions, vec![Real::zero()])
                .expect("a scalar zero tensor has a valid shape");
            return report(
                TensorConstraintResultantStatus::Constructed,
                vec![0; retained_rank],
                Some(zero),
                None,
                None,
            );
        }
        Err(()) => {
            return report(
                TensorConstraintResultantStatus::UndecidedCoefficient,
                Vec::new(),
                None,
                None,
                Some("tensor polynomial degree was not certified exactly".to_owned()),
            );
        }
    };
    let constraint_degree = constraint.len() - 1;
    let retained_axes = (0..polynomial.dimensions.len())
        .filter(|axis| *axis != eliminated_axis)
        .collect::<Vec<_>>();

    // A tensor linear in the eliminated variable has a closed-form norm for
    // a quadratic source constraint.  For
    //
    //     g(x) = c0 + c1*x + c2*x^2,   f(x) = A + B*x,
    //
    // `Res(g,f) = c2*A^2 - c1*A*B + c0*B^2`.  Constructing those retained-
    // axis tensor products directly avoids evaluating and interpolating a
    // multidimensional integer grid.  This case is especially common after
    // quotient-ring reduction of represented geometry: selected quadratic
    // source fields remain linear in the incidence tensor even when several
    // other fields and a high-degree output axis are present.
    if degrees[eliminated_axis] == 1 && constraint_degree == 2 {
        let Some(a) = tensor_axis_coefficient(polynomial, eliminated_axis, 0) else {
            return report(
                TensorConstraintResultantStatus::DimensionOverflow,
                Vec::new(),
                None,
                None,
                Some("linear tensor-resultant constant fiber exceeded its shape budget".to_owned()),
            );
        };
        let Some(b) = tensor_axis_coefficient(polynomial, eliminated_axis, 1) else {
            return report(
                TensorConstraintResultantStatus::DimensionOverflow,
                Vec::new(),
                None,
                None,
                Some(
                    "linear tensor-resultant coefficient fiber exceeded its shape budget"
                        .to_owned(),
                ),
            );
        };
        let Some(resultant) = a
            .multiply(&a)
            .and_then(|a_squared| a_squared.scale(&constraint[2]))
            .and_then(|a_squared| {
                a.multiply(&b)
                    .and_then(|product| product.scale(&constraint[1]))
                    .and_then(|product| a_squared.subtract(&product))
            })
            .and_then(|retained| {
                b.multiply(&b)
                    .and_then(|b_squared| b_squared.scale(&constraint[0]))
                    .and_then(|b_squared| retained.add(&b_squared))
            })
        else {
            return report(
                TensorConstraintResultantStatus::DimensionOverflow,
                Vec::new(),
                None,
                None,
                Some("closed-form linear tensor resultant exceeded its shape budget".to_owned()),
            );
        };
        let degree_bounds = retained_axes
            .iter()
            .map(|axis| degrees[*axis].saturating_mul(2))
            .collect::<Vec<_>>();
        // Convolution already has exactly the conservative resultant shape.
        // Do not certify and trim every retained coefficient here: callers
        // immediately reduce still-selected axes in their quotient rings,
        // and forcing zero decisions on the unreduced arithmetic DAG can cost
        // more than the elimination itself.
        return report(
            TensorConstraintResultantStatus::Constructed,
            degree_bounds,
            Some(resultant),
            None,
            None,
        );
    }

    // When only one retained axis remains, construct its exact quotient-ring
    // norm directly. This avoids reconstructing a high-degree univariate
    // resultant from an exact integer sample grid and keeps every coefficient
    // in compact primitive-integer form. The bounded determinant dimension is
    // the selected source degree, not the retained fiber degree.
    if retained_axes.len() == 1 {
        let retained_axis = retained_axes[0];
        let retained_count = polynomial.dimensions[retained_axis];
        let mut fiber_coefficients = Vec::new();
        if fiber_coefficients
            .try_reserve_exact(retained_count)
            .is_err()
        {
            return report(
                TensorConstraintResultantStatus::AllocationFailed,
                vec![
                    polynomial.dimensions[retained_axis]
                        .saturating_sub(1)
                        .saturating_mul(constraint_degree),
                ],
                None,
                None,
                Some("direct quotient-ring fiber storage could not be reserved".to_owned()),
            );
        }
        for retained_power in 0..retained_count {
            let mut fiber = Vec::new();
            if fiber
                .try_reserve_exact(polynomial.dimensions[eliminated_axis])
                .is_err()
            {
                return report(
                    TensorConstraintResultantStatus::AllocationFailed,
                    vec![
                        polynomial.dimensions[retained_axis]
                            .saturating_sub(1)
                            .saturating_mul(constraint_degree),
                    ],
                    None,
                    None,
                    Some("direct quotient-ring source fiber could not be reserved".to_owned()),
                );
            }
            for eliminated_power in 0..polynomial.dimensions[eliminated_axis] {
                let mut exponents = vec![0; polynomial.dimensions.len()];
                exponents[eliminated_axis] = eliminated_power;
                exponents[retained_axis] = retained_power;
                fiber.push(
                    polynomial
                        .coefficient(&exponents)
                        .cloned()
                        .unwrap_or_else(Real::zero),
                );
            }
            fiber_coefficients.push(fiber);
        }
        if let Some(coefficients) =
            quotient_ring_fiber_resultant_polynomial(&constraint, &fiber_coefficients, 8)
        {
            let degree = coefficients.len().saturating_sub(1);
            let resultant = DenseTensorPolynomial::try_new(vec![coefficients.len()], coefficients)
                .expect("a direct univariate quotient-ring norm has a valid dense shape");
            return report(
                TensorConstraintResultantStatus::Constructed,
                vec![degree],
                Some(resultant),
                None,
                None,
            );
        }
    }
    let Some(degree_bounds) = retained_axes
        .iter()
        .map(|axis| degrees[*axis].checked_mul(constraint_degree))
        .collect::<Option<Vec<_>>>()
    else {
        return report(
            TensorConstraintResultantStatus::DimensionOverflow,
            Vec::new(),
            None,
            None,
            Some("tensor resultant degree multiplication overflowed".to_owned()),
        );
    };
    let Some(sample_dimensions) = degree_bounds
        .iter()
        .map(|degree| degree.checked_add(1))
        .collect::<Option<Vec<_>>>()
    else {
        return report(
            TensorConstraintResultantStatus::DimensionOverflow,
            degree_bounds,
            None,
            None,
            Some("tensor resultant sample dimension overflowed".to_owned()),
        );
    };
    let Some(sample_count) = checked_coefficient_count(&sample_dimensions) else {
        return report(
            TensorConstraintResultantStatus::DimensionOverflow,
            degree_bounds,
            None,
            None,
            Some("tensor resultant sample grid overflowed".to_owned()),
        );
    };
    let mut samples = Vec::new();
    if samples.try_reserve_exact(sample_count).is_err() {
        return report(
            TensorConstraintResultantStatus::AllocationFailed,
            degree_bounds,
            None,
            None,
            Some("tensor resultant sample grid could not be reserved".to_owned()),
        );
    }
    for sample_index in 0..sample_count {
        let sample_exponents = exponents(&sample_dimensions, sample_index);
        let retained_values = sample_exponents
            .iter()
            .map(|value| Real::from(*value as u64))
            .collect::<Vec<_>>();
        let fiber = evaluate_retained_tensor_fiber(
            polynomial,
            eliminated_axis,
            &retained_axes,
            &retained_values,
        );
        let resultant = match resultant_univariate_polynomials(&constraint, &fiber, min_precision) {
            Ok(resultant) => resultant.resultant,
            Err(error) => {
                return report(
                    TensorConstraintResultantStatus::ResultantError,
                    degree_bounds,
                    None,
                    Some(error),
                    Some("one exact tensor-grid resultant failed".to_owned()),
                );
            }
        };
        samples.push(resultant);
    }
    if interpolate_tensor_grid(&mut samples, &sample_dimensions, min_precision).is_none() {
        return report(
            TensorConstraintResultantStatus::InterpolationDivisionFailed,
            degree_bounds,
            None,
            None,
            Some("exact tensor-grid interpolation failed".to_owned()),
        );
    }
    let resultant = match canonical_tensor(sample_dimensions, samples, min_precision) {
        Ok(resultant) => resultant,
        Err(CanonicalTensorError::UndecidedCoefficient) => {
            return report(
                TensorConstraintResultantStatus::UndecidedCoefficient,
                degree_bounds,
                None,
                None,
                Some("interpolated tensor degree was not certified exactly".to_owned()),
            );
        }
        Err(CanonicalTensorError::DimensionOverflow) => {
            return report(
                TensorConstraintResultantStatus::DimensionOverflow,
                degree_bounds,
                None,
                None,
                Some("canonical tensor dimensions overflowed".to_owned()),
            );
        }
        Err(CanonicalTensorError::AllocationFailed) => {
            return report(
                TensorConstraintResultantStatus::AllocationFailed,
                degree_bounds,
                None,
                None,
                Some("canonical tensor could not be reserved".to_owned()),
            );
        }
    };
    report(
        TensorConstraintResultantStatus::Constructed,
        degree_bounds,
        Some(resultant),
        None,
        None,
    )
}

fn normalized_constraint(constraint: &[Real], min_precision: i32) -> Result<Option<Vec<Real>>, ()> {
    let constraint = trim_trailing_zeroes(constraint.to_vec(), min_precision)?;
    if constraint.len() <= 1 {
        return Ok(None);
    }
    let leading = constraint
        .last()
        .expect("a nonconstant constraint retains its leading coefficient");
    constraint
        .iter()
        .map(|coefficient| (coefficient / leading).ok())
        .collect::<Option<Vec<_>>>()
        .map(Some)
        .ok_or(())
}

fn checked_coefficient_count(dimensions: &[usize]) -> Option<usize> {
    dimensions
        .iter()
        .try_fold(1_usize, |count, dimension| count.checked_mul(*dimension))
}

fn tensor_axis_coefficient(
    polynomial: &DenseTensorPolynomial,
    axis: usize,
    power: usize,
) -> Option<DenseTensorPolynomial> {
    if axis >= polynomial.dimensions.len() || power >= polynomial.dimensions[axis] {
        return None;
    }
    let mut dimensions = polynomial.dimensions.clone();
    dimensions.remove(axis);
    let count = checked_coefficient_count(&dimensions)?;
    let mut coefficients = Vec::new();
    coefficients.try_reserve_exact(count).ok()?;
    for index in 0..count {
        let mut source = exponents(&dimensions, index);
        source.insert(axis, power);
        coefficients.push(polynomial.coefficient(&source)?.clone());
    }
    DenseTensorPolynomial::try_new(dimensions, coefficients)
}

fn flat_index(dimensions: &[usize], exponents: &[usize]) -> usize {
    dimensions
        .iter()
        .zip(exponents)
        .fold(0_usize, |index, (dimension, exponent)| {
            index * dimension + exponent
        })
}

fn exponents(dimensions: &[usize], mut index: usize) -> Vec<usize> {
    let mut exponents = vec![0; dimensions.len()];
    for axis in (0..dimensions.len()).rev() {
        exponents[axis] = index % dimensions[axis];
        index /= dimensions[axis];
    }
    exponents
}

fn certified_tensor_degrees(
    polynomial: &DenseTensorPolynomial,
    min_precision: i32,
) -> Result<Option<Vec<usize>>, ()> {
    let mut degrees = None;
    for (index, coefficient) in polynomial.coefficients.iter().enumerate() {
        match coefficient.certified_sign_until(min_precision) {
            CertifiedRealSign::Known {
                sign: RealSign::Zero,
                ..
            } => {}
            CertifiedRealSign::Known { .. } => {
                let source = exponents(&polynomial.dimensions, index);
                let degrees = degrees.get_or_insert_with(|| vec![0; polynomial.dimensions.len()]);
                for (degree, exponent) in degrees.iter_mut().zip(source) {
                    *degree = (*degree).max(exponent);
                }
            }
            CertifiedRealSign::Unknown { .. } => return Err(()),
        }
    }
    Ok(degrees)
}

fn evaluate_retained_tensor_fiber(
    polynomial: &DenseTensorPolynomial,
    eliminated_axis: usize,
    retained_axes: &[usize],
    retained_values: &[Real],
) -> Vec<Real> {
    let powers = retained_axes
        .iter()
        .zip(retained_values)
        .map(|(axis, value)| {
            let mut powers = Vec::with_capacity(polynomial.dimensions[*axis]);
            let mut power = Real::one();
            for _ in 0..polynomial.dimensions[*axis] {
                powers.push(power.clone());
                power *= value;
            }
            powers
        })
        .collect::<Vec<_>>();
    let mut fiber = vec![Real::zero(); polynomial.dimensions[eliminated_axis]];
    for (index, coefficient) in polynomial.coefficients.iter().enumerate() {
        let source = exponents(&polynomial.dimensions, index);
        let mut value = coefficient.clone();
        for ((axis, powers), _) in retained_axes.iter().zip(&powers).zip(retained_values) {
            value *= &powers[source[*axis]];
        }
        fiber[source[eliminated_axis]] += value;
    }
    fiber
}

fn interpolate_tensor_grid(
    values: &mut [Real],
    dimensions: &[usize],
    min_precision: i32,
) -> Option<()> {
    let total = checked_coefficient_count(dimensions)?;
    if values.len() != total {
        return None;
    }
    for axis in 0..dimensions.len() {
        let length = dimensions[axis];
        let stride = checked_coefficient_count(&dimensions[axis + 1..])?;
        let outer = total.checked_div(length.checked_mul(stride)?)?;
        for outer_index in 0..outer {
            for inner_index in 0..stride {
                let line = (0..length)
                    .map(|power| {
                        values[outer_index * length * stride + power * stride + inner_index].clone()
                    })
                    .collect::<Vec<_>>();
                let coefficients = interpolate_integer_values(&line, min_precision)?;
                for (power, coefficient) in coefficients.into_iter().enumerate() {
                    values[outer_index * length * stride + power * stride + inner_index] =
                        coefficient;
                }
            }
        }
    }
    Some(())
}

fn interpolate_integer_values(values: &[Real], min_precision: i32) -> Option<Vec<Real>> {
    if values.is_empty() {
        return None;
    }
    let mut result = vec![Real::zero(); values.len()];
    let mut differences = values.to_vec();
    let mut falling_factorial = vec![Real::one()];
    let mut factorial = Real::one();
    for order in 0..values.len() {
        let scale = (differences[0].clone() / &factorial).ok()?;
        for (coefficient, basis) in result.iter_mut().zip(&falling_factorial) {
            *coefficient += basis * &scale;
        }
        for index in 0..differences.len() - 1 {
            differences[index] = &differences[index + 1] - &differences[index];
        }
        differences.pop();
        if order + 1 < values.len() {
            falling_factorial =
                multiply_by_linear_factor(falling_factorial, -Real::from(order as u64));
            factorial *= Real::from((order + 1) as u64);
        }
    }
    let trimmed = trim_trailing_zeroes(result, min_precision).ok()?;
    let mut padded = trimmed;
    padded.resize(values.len(), Real::zero());
    Some(padded)
}

fn multiply_by_linear_factor(polynomial: Vec<Real>, constant: Real) -> Vec<Real> {
    let mut result = vec![Real::zero(); polynomial.len() + 1];
    for (index, coefficient) in polynomial.into_iter().enumerate() {
        result[index] += &coefficient * &constant;
        result[index + 1] += coefficient;
    }
    result
}

fn trim_trailing_zeroes(coefficients: Vec<Real>, min_precision: i32) -> Result<Vec<Real>, ()> {
    for (index, coefficient) in coefficients.iter().enumerate().rev() {
        match coefficient.certified_sign_until(min_precision) {
            CertifiedRealSign::Known {
                sign: RealSign::Zero,
                ..
            } => {}
            CertifiedRealSign::Known { .. } => return Ok(coefficients[..=index].to_vec()),
            CertifiedRealSign::Unknown { .. } => return Err(()),
        }
    }
    Ok(vec![Real::zero()])
}

enum CanonicalTensorError {
    UndecidedCoefficient,
    DimensionOverflow,
    AllocationFailed,
}

fn canonical_tensor(
    dimensions: Vec<usize>,
    coefficients: Vec<Real>,
    min_precision: i32,
) -> Result<DenseTensorPolynomial, CanonicalTensorError> {
    let source = DenseTensorPolynomial::try_new(dimensions, coefficients)
        .ok_or(CanonicalTensorError::DimensionOverflow)?;
    let Some(degrees) = certified_tensor_degrees(&source, min_precision)
        .map_err(|()| CanonicalTensorError::UndecidedCoefficient)?
    else {
        return DenseTensorPolynomial::try_new(
            vec![1; source.dimensions.len()],
            vec![Real::zero()],
        )
        .ok_or(CanonicalTensorError::DimensionOverflow);
    };
    let dimensions = degrees
        .iter()
        .map(|degree| degree.checked_add(1))
        .collect::<Option<Vec<_>>>()
        .ok_or(CanonicalTensorError::DimensionOverflow)?;
    let count =
        checked_coefficient_count(&dimensions).ok_or(CanonicalTensorError::DimensionOverflow)?;
    let mut coefficients = Vec::new();
    coefficients
        .try_reserve_exact(count)
        .map_err(|_| CanonicalTensorError::AllocationFailed)?;
    coefficients.resize(count, Real::zero());
    for (source_index, coefficient) in source.coefficients.into_iter().enumerate() {
        let source_exponents = exponents(&source.dimensions, source_index);
        if source_exponents
            .iter()
            .zip(&dimensions)
            .all(|(exponent, dimension)| exponent < dimension)
        {
            let target = flat_index(&dimensions, &source_exponents);
            coefficients[target] = coefficient;
        }
    }
    DenseTensorPolynomial::try_new(dimensions, coefficients)
        .ok_or(CanonicalTensorError::DimensionOverflow)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::root_isolation::{
        IsolatedRootInterval, RootIsolationConfig, refine_isolated_univariate_polynomial_interval,
        square_free_part,
    };
    use hyperlimit::PredicatePolicy;

    fn real(value: i64) -> Real {
        Real::from(value)
    }

    fn selected_sum_polynomial(radical_count: usize) -> DenseTensorPolynomial {
        let mut dimensions = vec![2; radical_count + 1];
        let count = checked_coefficient_count(&dimensions).unwrap();
        let mut coefficients = vec![Real::zero(); count];
        for axis in 0..radical_count {
            let mut exponent = vec![0; dimensions.len()];
            exponent[axis] = 1;
            coefficients[flat_index(&dimensions, &exponent)] = -Real::one();
        }
        let output_axis = radical_count;
        let mut exponent = vec![0; dimensions.len()];
        exponent[output_axis] = 1;
        coefficients[flat_index(&dimensions, &exponent)] = Real::one();
        DenseTensorPolynomial::try_new(std::mem::take(&mut dimensions), coefficients).unwrap()
    }

    #[test]
    fn rank_five_eliminates_four_selected_square_roots_without_a_degree_cap() {
        let mut polynomial = selected_sum_polynomial(4);
        for square in [2_i64, 3, 5, 7] {
            let report = resultant_tensor_polynomial_univariate_constraint(
                &polynomial,
                &[-real(square), Real::zero(), Real::one()],
                0,
                PredicatePolicy::MAX_REFINEMENT_PRECISION,
            );
            assert_eq!(report.status, TensorConstraintResultantStatus::Constructed);
            polynomial = report.resultant.unwrap();
        }
        assert_eq!(polynomial.dimensions(), &[17]);
        let refinement = refine_isolated_univariate_polynomial_interval(
            polynomial.coefficients(),
            &IsolatedRootInterval {
                lower: real(8),
                upper: real(9),
                exact_root: None,
                distinct_root_count: 1,
            },
            RootIsolationConfig {
                policy: PredicatePolicy::STRICT,
                max_interval_width: None,
                max_refinement_steps: 4,
            },
        );
        assert!(refinement.refined_interval.is_some(), "{refinement:?}");
    }

    #[test]
    fn tensor_resultant_preserves_a_correlated_product_image() {
        let dimensions = vec![2, 2, 2];
        let mut coefficients = vec![Real::zero(); 8];
        coefficients[flat_index(&dimensions, &[0, 0, 1])] = Real::one();
        coefficients[flat_index(&dimensions, &[1, 1, 0])] = -Real::one();
        let mut polynomial = DenseTensorPolynomial::try_new(dimensions, coefficients).unwrap();
        for square in [2_i64, 3] {
            polynomial = resultant_tensor_polynomial_univariate_constraint(
                &polynomial,
                &[-real(square), Real::zero(), Real::one()],
                0,
                PredicatePolicy::MAX_REFINEMENT_PRECISION,
            )
            .resultant
            .unwrap();
        }
        // Both conjugates of the second selected root have the same squared
        // image, so the raw resultant correctly retains multiplicity. The
        // represented-value layer square-frees only after every source axis
        // has been eliminated.
        assert_eq!(polynomial.dimensions(), &[5]);
        assert_eq!(
            square_free_part(polynomial.coefficients().to_vec(), PredicatePolicy::STRICT).unwrap(),
            vec![real(-6), Real::zero(), Real::one()]
        );
    }

    #[test]
    fn final_axis_quotient_norm_preserves_a_quadratic_source_fiber() {
        // P(x,t)=t²+x*t+1 over x²-2 has norm
        // (t²+1)²-2t²=t⁴+1. The final-axis shortcut must retain the same
        // exact univariate eliminant without grid interpolation.
        let polynomial = DenseTensorPolynomial::try_new(
            vec![2, 3],
            vec![real(1), real(0), real(1), real(0), real(1), real(0)],
        )
        .unwrap();
        let report = resultant_tensor_polynomial_univariate_constraint(
            &polynomial,
            &[real(-2), real(0), real(1)],
            0,
            PredicatePolicy::MAX_REFINEMENT_PRECISION,
        );
        assert_eq!(report.status, TensorConstraintResultantStatus::Constructed);
        assert_eq!(
            report.resultant.unwrap().coefficients(),
            &[real(1), real(0), real(0), real(0), real(1)]
        );
    }

    #[test]
    fn tensor_resultant_reports_invalid_axes_and_constraints() {
        let polynomial = selected_sum_polynomial(1);
        assert_eq!(
            resultant_tensor_polynomial_univariate_constraint(
                &polynomial,
                &[Real::one()],
                0,
                PredicatePolicy::MAX_REFINEMENT_PRECISION,
            )
            .status,
            TensorConstraintResultantStatus::InvalidConstraint
        );
        assert_eq!(
            resultant_tensor_polynomial_univariate_constraint(
                &polynomial,
                &[Real::zero(), Real::one()],
                2,
                PredicatePolicy::MAX_REFINEMENT_PRECISION,
            )
            .status,
            TensorConstraintResultantStatus::InvalidAxis
        );
    }

    #[test]
    fn dense_tensor_arithmetic_preserves_rank_and_power_axes() {
        let first = DenseTensorPolynomial::from_axis_polynomial(5, 1, &[real(2), real(3)]).unwrap();
        let second =
            DenseTensorPolynomial::from_axis_polynomial(5, 3, &[real(-1), Real::one()]).unwrap();
        let product = first.multiply(&second).unwrap().shift_axis(4, 2).unwrap();
        assert_eq!(product.dimensions(), &[1, 2, 1, 2, 3]);
        assert_eq!(product.coefficient(&[0, 0, 0, 0, 2]), Some(&real(-2)));
        assert_eq!(product.coefficient(&[0, 1, 0, 1, 2]), Some(&real(3)));
        let replay = product.add(&product.scale(&real(-1)).unwrap()).unwrap();
        assert!(replay.coefficients().iter().all(Real::definitely_zero));
    }

    #[test]
    fn dense_tensor_axis_reduction_uses_the_exact_selected_quotient_ring() {
        let cubic = DenseTensorPolynomial::from_axis_polynomial(
            3,
            0,
            &[Real::zero(), Real::zero(), Real::zero(), Real::one()],
        )
        .unwrap();
        let other = DenseTensorPolynomial::from_axis_polynomial(3, 1, &[real(5), real(7)]).unwrap();
        let reduced = cubic
            .add(&other)
            .unwrap()
            .reduce_axis_modulo(
                0,
                &[real(-2), Real::zero(), Real::one()],
                PredicatePolicy::STRICT,
            )
            .unwrap();
        assert_eq!(reduced.dimensions(), &[2, 2, 1]);
        assert_eq!(reduced.coefficient(&[1, 0, 0]), Some(&real(2)));
        assert_eq!(reduced.coefficient(&[0, 0, 0]), Some(&real(5)));
        assert_eq!(reduced.coefficient(&[0, 1, 0]), Some(&real(7)));
    }

    #[test]
    fn dense_tensor_equal_axis_substitution_adds_source_powers() {
        let first = DenseTensorPolynomial::from_axis_polynomial(3, 0, &[Real::zero(), Real::one()])
            .unwrap();
        let second =
            DenseTensorPolynomial::from_axis_polynomial(3, 1, &[Real::zero(), Real::one()])
                .unwrap();
        let diagonal = first
            .add(&second)
            .unwrap()
            .add(&first.multiply(&second).unwrap())
            .unwrap()
            .substitute_equal_axes(0, 1)
            .unwrap();
        assert_eq!(diagonal.dimensions(), &[3, 1]);
        assert_eq!(diagonal.coefficient(&[1, 0]), Some(&real(2)));
        assert_eq!(diagonal.coefficient(&[2, 0]), Some(&Real::one()));

        let reverse = first
            .multiply(&second)
            .unwrap()
            .substitute_equal_axes(1, 0)
            .unwrap();
        assert_eq!(reverse.dimensions(), &[3, 1]);
        assert_eq!(reverse.coefficient(&[2, 0]), Some(&Real::one()));
    }

    #[test]
    fn dense_tensor_affine_axis_substitution_expands_exactly() {
        let first = DenseTensorPolynomial::from_axis_polynomial(3, 0, &[Real::zero(), Real::one()])
            .unwrap();
        let second =
            DenseTensorPolynomial::from_axis_polynomial(3, 1, &[Real::zero(), Real::one()])
                .unwrap();
        let affine = first
            .add(&second)
            .unwrap()
            .add(&first.multiply(&second).unwrap())
            .unwrap()
            .substitute_affine_axis(0, 1, &real(2), &real(3))
            .unwrap();
        assert_eq!(affine.dimensions(), &[3, 1]);
        assert_eq!(affine.coefficient(&[0, 0]), Some(&real(3)));
        assert_eq!(affine.coefficient(&[1, 0]), Some(&real(6)));
        assert_eq!(affine.coefficient(&[2, 0]), Some(&real(2)));
    }

    #[test]
    fn dense_tensor_removes_only_certified_independent_axes() {
        let independent = DenseTensorPolynomial::try_new(
            vec![2, 3],
            vec![
                real(1),
                Real::zero(),
                Real::zero(),
                real(2),
                Real::zero(),
                Real::zero(),
            ],
        )
        .unwrap();
        let removed = independent
            .remove_certified_independent_axis(1, PredicatePolicy::MAX_REFINEMENT_PRECISION)
            .unwrap();
        assert_eq!(removed.dimensions(), &[2]);
        assert_eq!(removed.coefficients(), &[real(1), real(2)]);

        let dependent = DenseTensorPolynomial::try_new(
            vec![2, 2],
            vec![real(1), real(3), real(2), Real::zero()],
        )
        .unwrap();
        assert!(
            dependent
                .remove_certified_independent_axis(1, PredicatePolicy::MAX_REFINEMENT_PRECISION)
                .is_none()
        );
    }
}
