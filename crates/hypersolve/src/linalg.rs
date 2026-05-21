//! Linear-solver adapter boundary for `hypersolve`.
//!
//! The problem model, residual expressions, and structural sparsity facts live
//! in exact `hyperreal::Real` space. This module is the named dense `f64`
//! adapter used by the current iterative solver. Keeping that boundary explicit
//! follows Yap's exact-geometric-computation separation: approximate numerical
//! methods may be selected as adapters, but exact structure and combinatorial
//! facts must not be silently replaced by primitive-float predicates. See Yap,
//! "Towards Exact Geometric Computation," *Computational Geometry* 7.1-2
//! (1997).

#[derive(Clone, Debug, PartialEq)]
pub enum LinearSolveError {
    Singular,
    DimensionMismatch,
}

/// Numerical adapter that produced a linear solve step.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LinearAdapterKind {
    /// Dense primitive-float normal-equation adapter.
    ///
    /// This is a lossy numerical edge. It is suitable for iteration steps and
    /// diagnostics, not for exact topology or symbolic rank decisions.
    DenseF64NormalEquations,
    /// Dense primitive-float BFGS direction adapter.
    ///
    /// This is a lossy quasi-Newton proposal edge, not an exact Hessian or
    /// convexity certificate.
    DenseF64BfgsDirection,
}

/// Precision boundary crossed by a linear-solver adapter.
///
/// Solver iterations may use approximate numerical adapters, but that adapter
/// status must remain visible to callers rather than becoming an implicit
/// backend mode. This mirrors Yap's exact-geometric-computation separation
/// between exact symbolic/geometric structure and approximate numerical stages;
/// see Yap, "Towards Exact Geometric Computation," *Computational Geometry*
/// 7.1-2 (1997).
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LinearAdapterPrecision {
    /// The adapter lowered exact residual/Jacobian data to primitive `f64`.
    LossyF64,
}

impl LinearAdapterKind {
    /// Return the precision boundary implied by this adapter.
    pub const fn precision(self) -> LinearAdapterPrecision {
        match self {
            Self::DenseF64NormalEquations | Self::DenseF64BfgsDirection => {
                LinearAdapterPrecision::LossyF64
            }
        }
    }

    /// Return whether this adapter crosses a lossy primitive-float boundary.
    pub const fn is_lossy(self) -> bool {
        matches!(self.precision(), LinearAdapterPrecision::LossyF64)
    }
}

/// Diagnostics returned by a linear-solver adapter.
///
/// These values describe the approximate adapter route, not a proof of exact
/// rank or feasibility. The pivot metadata follows the standard Gaussian
/// elimination diagnostic practice described by Higham, *Accuracy and
/// Stability of Numerical Algorithms*, 2nd ed., SIAM, 2002. Future exact or
/// mixed backends should add their own adapter variants rather than hiding
/// exact/approximate status behind the same report.
#[derive(Clone, Debug)]
pub struct LinearSolveReport {
    /// Adapter route used to compute the step.
    pub adapter: LinearAdapterKind,
    /// Whether the result crossed a lossy primitive-float boundary.
    pub lossy: bool,
    /// Structural rank hint observed by the adapter.
    ///
    /// For the dense f64 adapter this is only a pivot count after damping and
    /// must not be treated as an exact symbolic rank.
    pub rank_hint: Option<usize>,
    /// Damping added to the normal-equation diagonal.
    pub damping: f64,
    /// Number of successful elimination pivots.
    pub pivot_count: usize,
    /// Number of row swaps performed during elimination.
    pub row_swaps: usize,
    /// Smallest absolute pivot accepted by the dense adapter.
    pub min_abs_pivot: Option<f64>,
    /// Largest absolute pivot accepted by the dense adapter.
    pub max_abs_pivot: Option<f64>,
}

impl LinearSolveReport {
    /// Return the precision boundary crossed by the adapter that produced this
    /// report.
    pub const fn precision(&self) -> LinearAdapterPrecision {
        self.adapter.precision()
    }

    /// Return whether this report came from a lossy primitive-float adapter.
    pub const fn is_lossy(&self) -> bool {
        self.adapter.is_lossy() || self.lossy
    }
}

pub trait LinearBackend {
    /// Solve a damped normal system through an explicit f64 adapter.
    ///
    /// Structural-dispatch note: a future hyperreal/hyperlattice backend should
    /// carry row sparsity, rank hints, scaling, and exact-rational denominator
    /// facts into the solve so dense primitive Gaussian elimination is only one
    /// selectable edge strategy.
    fn solve_damped_normal(
        &self,
        jacobian: &[Vec<f64>],
        residuals: &[f64],
        damping: f64,
    ) -> Result<(Vec<f64>, LinearSolveReport), LinearSolveError>;

    /// Solve one dense trust-region dogleg proposal step through `f64`.
    ///
    /// This is a proposal adapter following Powell's dogleg trust-region
    /// construction; see M. J. D. Powell, "A Hybrid Method for Nonlinear
    /// Equations" (1970), and Nocedal and Wright, *Numerical Optimization*,
    /// 2nd ed. (2006). The returned step is not a proof: exact candidate
    /// replay must still certify any accepted coordinates.
    fn solve_dogleg(
        &self,
        jacobian: &[Vec<f64>],
        residuals: &[f64],
        trust_radius: f64,
    ) -> Result<(Vec<f64>, LinearSolveReport), LinearSolveError>;

    /// Compute one dense BFGS proposal direction through `f64`.
    ///
    /// The inverse-Hessian approximation is supplied by the nonlinear solver
    /// loop. This follows the quasi-Newton BFGS update family of Broyden,
    /// Fletcher, Goldfarb, and Shanno (1970), but remains only a lossy
    /// candidate-generation route; exact replay still decides acceptance.
    fn solve_bfgs_direction(
        &self,
        inverse_hessian: &[Vec<f64>],
        gradient: &[f64],
        step_limit: f64,
    ) -> Result<(Vec<f64>, LinearSolveReport), LinearSolveError>;
}

#[derive(Clone, Debug, Default)]
pub struct DenseLinearBackend;

impl LinearBackend for DenseLinearBackend {
    fn solve_damped_normal(
        &self,
        jacobian: &[Vec<f64>],
        residuals: &[f64],
        damping: f64,
    ) -> Result<(Vec<f64>, LinearSolveReport), LinearSolveError> {
        if jacobian.len() != residuals.len() {
            return Err(LinearSolveError::DimensionMismatch);
        }
        let Some(width) = jacobian.first().map(Vec::len) else {
            return Ok((
                Vec::new(),
                LinearSolveReport {
                    adapter: LinearAdapterKind::DenseF64NormalEquations,
                    lossy: true,
                    rank_hint: Some(0),
                    damping,
                    pivot_count: 0,
                    row_swaps: 0,
                    min_abs_pivot: None,
                    max_abs_pivot: None,
                },
            ));
        };
        if jacobian.iter().any(|row| row.len() != width) {
            return Err(LinearSolveError::DimensionMismatch);
        }

        let mut normal = vec![vec![0.0; width]; width];
        let mut rhs = vec![0.0; width];
        for (row, residual) in jacobian.iter().zip(residuals) {
            for i in 0..width {
                rhs[i] -= row[i] * residual;
                for j in 0..width {
                    normal[i][j] += row[i] * row[j];
                }
            }
        }
        for (i, row) in normal.iter_mut().enumerate() {
            row[i] += damping;
        }

        let (step, diagnostics) = solve_dense(normal, rhs)?;
        Ok((
            step,
            LinearSolveReport {
                adapter: LinearAdapterKind::DenseF64NormalEquations,
                lossy: true,
                rank_hint: Some(width),
                damping,
                pivot_count: diagnostics.pivot_count,
                row_swaps: diagnostics.row_swaps,
                min_abs_pivot: diagnostics.min_abs_pivot,
                max_abs_pivot: diagnostics.max_abs_pivot,
            },
        ))
    }

    fn solve_dogleg(
        &self,
        jacobian: &[Vec<f64>],
        residuals: &[f64],
        trust_radius: f64,
    ) -> Result<(Vec<f64>, LinearSolveReport), LinearSolveError> {
        if jacobian.len() != residuals.len() {
            return Err(LinearSolveError::DimensionMismatch);
        }
        let Some(width) = jacobian.first().map(Vec::len) else {
            return Ok((
                Vec::new(),
                LinearSolveReport {
                    adapter: LinearAdapterKind::DenseF64NormalEquations,
                    lossy: true,
                    rank_hint: Some(0),
                    damping: trust_radius,
                    pivot_count: 0,
                    row_swaps: 0,
                    min_abs_pivot: None,
                    max_abs_pivot: None,
                },
            ));
        };
        if jacobian.iter().any(|row| row.len() != width) {
            return Err(LinearSolveError::DimensionMismatch);
        }

        let (normal, gradient) = normal_matrix_and_gradient(jacobian, residuals);
        let rhs = gradient.iter().map(|value| -*value).collect::<Vec<_>>();
        let (gauss_newton, diagnostics) = solve_dense(normal.clone(), rhs)?;
        let radius = trust_radius.max(f64::EPSILON);
        let step = dogleg_step(&normal, &gradient, &gauss_newton, radius)?;
        Ok((
            step,
            LinearSolveReport {
                adapter: LinearAdapterKind::DenseF64NormalEquations,
                lossy: true,
                rank_hint: Some(width),
                damping: trust_radius,
                pivot_count: diagnostics.pivot_count,
                row_swaps: diagnostics.row_swaps,
                min_abs_pivot: diagnostics.min_abs_pivot,
                max_abs_pivot: diagnostics.max_abs_pivot,
            },
        ))
    }

    fn solve_bfgs_direction(
        &self,
        inverse_hessian: &[Vec<f64>],
        gradient: &[f64],
        step_limit: f64,
    ) -> Result<(Vec<f64>, LinearSolveReport), LinearSolveError> {
        if inverse_hessian.len() != gradient.len()
            || inverse_hessian
                .iter()
                .any(|row| row.len() != gradient.len())
        {
            return Err(LinearSolveError::DimensionMismatch);
        }
        let mut step = mat_vec(inverse_hessian, gradient)
            .into_iter()
            .map(|value| -value)
            .collect::<Vec<_>>();
        let step_norm = norm2(&step);
        let limit = step_limit.max(f64::EPSILON);
        if step_norm > limit {
            step = scaled(&step, limit / step_norm);
        }
        Ok((
            step,
            LinearSolveReport {
                adapter: LinearAdapterKind::DenseF64BfgsDirection,
                lossy: true,
                rank_hint: Some(gradient.len()),
                damping: step_limit,
                pivot_count: 0,
                row_swaps: 0,
                min_abs_pivot: None,
                max_abs_pivot: None,
            },
        ))
    }
}

fn normal_matrix_and_gradient(
    jacobian: &[Vec<f64>],
    residuals: &[f64],
) -> (Vec<Vec<f64>>, Vec<f64>) {
    let width = jacobian.first().map(Vec::len).unwrap_or(0);
    let mut normal = vec![vec![0.0; width]; width];
    let mut gradient = vec![0.0; width];
    for (row, residual) in jacobian.iter().zip(residuals) {
        for i in 0..width {
            gradient[i] += row[i] * residual;
            for j in 0..width {
                normal[i][j] += row[i] * row[j];
            }
        }
    }
    (normal, gradient)
}

fn dogleg_step(
    normal: &[Vec<f64>],
    gradient: &[f64],
    gauss_newton: &[f64],
    trust_radius: f64,
) -> Result<Vec<f64>, LinearSolveError> {
    if norm2(gauss_newton) <= trust_radius {
        return Ok(gauss_newton.to_vec());
    }
    let gradient_norm_sq = dot(gradient, gradient);
    if gradient_norm_sq <= f64::EPSILON {
        return Ok(vec![0.0; gradient.len()]);
    }
    let normal_gradient = mat_vec(normal, gradient);
    let curvature = dot(gradient, &normal_gradient);
    if curvature <= f64::EPSILON {
        return Ok(scaled(gradient, -trust_radius / gradient_norm_sq.sqrt()));
    }
    let alpha = gradient_norm_sq / curvature;
    let cauchy = scaled(gradient, -alpha);
    let cauchy_norm = norm2(&cauchy);
    if cauchy_norm >= trust_radius {
        return Ok(scaled(&cauchy, trust_radius / cauchy_norm));
    }
    let segment = gauss_newton
        .iter()
        .zip(&cauchy)
        .map(|(gn, sd)| gn - sd)
        .collect::<Vec<_>>();
    let a = dot(&segment, &segment);
    let b = 2.0 * dot(&cauchy, &segment);
    let c = dot(&cauchy, &cauchy) - trust_radius * trust_radius;
    let discriminant = b * b - 4.0 * a * c;
    if a <= f64::EPSILON || discriminant < 0.0 {
        return Err(LinearSolveError::Singular);
    }
    let tau = (-b + discriminant.sqrt()) / (2.0 * a);
    Ok(cauchy
        .iter()
        .zip(segment)
        .map(|(sd, direction)| sd + tau * direction)
        .collect())
}

fn mat_vec(matrix: &[Vec<f64>], vector: &[f64]) -> Vec<f64> {
    matrix
        .iter()
        .map(|row| row.iter().zip(vector).map(|(a, x)| a * x).sum())
        .collect()
}

fn dot(left: &[f64], right: &[f64]) -> f64 {
    left.iter()
        .zip(right)
        .map(|(left, right)| left * right)
        .sum()
}

fn norm2(vector: &[f64]) -> f64 {
    dot(vector, vector).sqrt()
}

fn scaled(vector: &[f64], scale: f64) -> Vec<f64> {
    vector.iter().map(|value| value * scale).collect()
}

#[derive(Clone, Debug, Default, PartialEq)]
struct DensePivotDiagnostics {
    pivot_count: usize,
    row_swaps: usize,
    min_abs_pivot: Option<f64>,
    max_abs_pivot: Option<f64>,
}

impl DensePivotDiagnostics {
    fn record_pivot(&mut self, abs_pivot: f64, swapped: bool) {
        self.pivot_count += 1;
        if swapped {
            self.row_swaps += 1;
        }
        self.min_abs_pivot = Some(
            self.min_abs_pivot
                .map_or(abs_pivot, |current| current.min(abs_pivot)),
        );
        self.max_abs_pivot = Some(
            self.max_abs_pivot
                .map_or(abs_pivot, |current| current.max(abs_pivot)),
        );
    }
}

fn solve_dense(
    mut matrix: Vec<Vec<f64>>,
    mut rhs: Vec<f64>,
) -> Result<(Vec<f64>, DensePivotDiagnostics), LinearSolveError> {
    let n = rhs.len();
    let mut diagnostics = DensePivotDiagnostics::default();
    for pivot in 0..n {
        let mut best = pivot;
        let mut best_abs = matrix[pivot][pivot].abs();
        for (row, values) in matrix.iter().enumerate().skip(pivot + 1) {
            let candidate = values[pivot].abs();
            if candidate > best_abs {
                best = row;
                best_abs = candidate;
            }
        }
        if best_abs <= f64::EPSILON {
            return Err(LinearSolveError::Singular);
        }
        diagnostics.record_pivot(best_abs, best != pivot);
        if best != pivot {
            matrix.swap(best, pivot);
            rhs.swap(best, pivot);
        }
        let pivot_value = matrix[pivot][pivot];
        for value in matrix[pivot].iter_mut().skip(pivot) {
            *value /= pivot_value;
        }
        let pivot_tail = matrix[pivot][pivot..].to_vec();
        rhs[pivot] /= pivot_value;
        for row in 0..n {
            if row == pivot {
                continue;
            }
            let factor = matrix[row][pivot];
            if factor == 0.0 {
                continue;
            }
            for (value, pivot_value) in matrix[row].iter_mut().skip(pivot).zip(&pivot_tail) {
                *value -= factor * pivot_value;
            }
            rhs[row] -= factor * rhs[pivot];
        }
    }
    Ok((rhs, diagnostics))
}
