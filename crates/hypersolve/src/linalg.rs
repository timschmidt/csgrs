#[derive(Clone, Debug, PartialEq)]
pub enum LinearSolveError {
    Singular,
    DimensionMismatch,
}

#[derive(Clone, Debug)]
pub struct LinearSolveReport {
    pub rank_hint: Option<usize>,
    pub damping: f64,
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
                    rank_hint: Some(0),
                    damping,
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

        let step = solve_dense(normal, rhs)?;
        Ok((
            step,
            LinearSolveReport {
                rank_hint: Some(width),
                damping,
            },
        ))
    }
}

fn solve_dense(mut matrix: Vec<Vec<f64>>, mut rhs: Vec<f64>) -> Result<Vec<f64>, LinearSolveError> {
    let n = rhs.len();
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
        if best != pivot {
            matrix.swap(best, pivot);
            rhs.swap(best, pivot);
        }
        let pivot_value = matrix[pivot][pivot];
        for col in pivot..n {
            matrix[pivot][col] /= pivot_value;
        }
        rhs[pivot] /= pivot_value;
        for row in 0..n {
            if row == pivot {
                continue;
            }
            let factor = matrix[row][pivot];
            if factor == 0.0 {
                continue;
            }
            for col in pivot..n {
                matrix[row][col] -= factor * matrix[pivot][col];
            }
            rhs[row] -= factor * rhs[pivot];
        }
    }
    Ok(rhs)
}
