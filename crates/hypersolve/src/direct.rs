//! Direct exact reductions for simple solver rows.
//!
//! SolveSpace performs substitution and "soluble alone" passes before Newton
//! iteration. This module adds the exact-stack version for prepared affine
//! equality rows with one active variable. The result is a candidate assignment
//! produced by exact `hyperreal::Real` arithmetic; callers still replay and
//! certify the full problem before trusting it. This keeps the optimization in
//! Yap's expression/object layer instead of making a lossy nonlinear backend
//! responsible for obvious exact algebra.

use hyperreal::{Real, RealSign};
use std::collections::{BTreeMap, BTreeSet, VecDeque};

use crate::eval::EvaluationContext;
use crate::model::ConstraintKind;
use crate::prepared::PreparedProblem;
use crate::symbolic::SymbolId;

/// Exact direct solution for one affine equality row.
#[derive(Clone, Debug, PartialEq)]
pub struct DirectAffineSolution {
    /// Source constraint index that produced the solution.
    pub constraint_index: usize,
    /// Solver symbol solved by this row.
    pub symbol: SymbolId,
    /// Exact candidate value.
    pub value: Real,
}

/// Exact root candidates for one prepared univariate quadratic equality row.
#[derive(Clone, Debug, PartialEq)]
pub struct DirectQuadraticSolution {
    /// Source constraint index that produced these roots.
    pub constraint_index: usize,
    /// Solver symbol solved by this row.
    pub symbol: SymbolId,
    /// Exact root candidates. Empty means the discriminant is certified negative.
    pub roots: Vec<Real>,
}

/// Exact equality substitution `left = right + offset`.
///
/// This is the first union-find-friendly payload for SolveSpace-style
/// substitution passes. It deliberately records the source row and exact offset
/// instead of mutating the problem immediately; callers can choose whether to
/// use it for simplification, diagnostics, or candidate initialization before
/// replaying the full problem.
#[derive(Clone, Debug, PartialEq)]
pub struct EqualitySubstitution {
    /// Source constraint index.
    pub constraint_index: usize,
    /// Symbol being substituted.
    pub left: SymbolId,
    /// Reference symbol.
    pub right: SymbolId,
    /// Exact offset such that `left = right + offset`.
    pub offset: Real,
}

/// Cycle found in equality-substitution candidates.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct EqualitySubstitutionCycle {
    /// Symbols participating in the cycle in discovery order.
    pub symbols: Vec<SymbolId>,
}

/// Exact consistency diagnostics for equality substitution candidates.
#[derive(Clone, Debug, PartialEq)]
pub struct EqualitySubstitutionValidation {
    /// Problems found while validating the directed substitution graph.
    pub problems: Vec<EqualitySubstitutionProblem>,
}

impl EqualitySubstitutionValidation {
    /// Return whether the substitution graph can be used as an acyclic rewrite.
    pub fn is_acyclic_rewrite_graph(&self) -> bool {
        self.problems.is_empty()
    }

    /// Return whether any reported problem proves algebraic inconsistency.
    pub fn has_inconsistency(&self) -> bool {
        self.problems
            .iter()
            .any(EqualitySubstitutionProblem::is_inconsistent)
    }
}

/// One member of an exact equality-substitution class.
#[derive(Clone, Debug, PartialEq)]
pub struct EqualitySubstitutionClassMember {
    /// Symbol in this connected equality component.
    pub symbol: SymbolId,
    /// Exact offset from the representative: `symbol = representative + offset`.
    pub offset_from_representative: Real,
}

/// Exact connected component induced by equality substitutions.
///
/// This is the union-find payload without committing to an in-place rewrite of
/// the model. Components preserve exact offsets to a stable representative, so
/// later affine elimination can rewrite symbols without re-solving the same
/// small graph. The construction follows the same pre-Newton symbolic
/// reduction boundary as SolveSpace, while keeping Yap's exact replay boundary
/// intact.
#[derive(Clone, Debug, PartialEq)]
pub struct EqualitySubstitutionClass {
    /// Stable class representative, chosen as the smallest symbol id.
    pub representative: SymbolId,
    /// Members sorted by symbol id.
    pub members: Vec<EqualitySubstitutionClassMember>,
}

/// One exact issue found in a substitution candidate set.
#[derive(Clone, Debug, PartialEq)]
pub enum EqualitySubstitutionProblem {
    /// A row rewrites a symbol to itself.
    ///
    /// This is a harmless tautology only when `offset == 0`; otherwise it
    /// proves `0 == offset` and the candidate set is inconsistent.
    SelfSubstitution {
        /// Source constraint index.
        constraint_index: usize,
        /// Rewritten symbol.
        symbol: SymbolId,
        /// Exact offset in `symbol = symbol + offset`.
        offset: Real,
        /// Whether the self-substitution is exactly consistent.
        consistent: bool,
    },
    /// Two rows try to rewrite the same left-hand symbol.
    DuplicateLeft {
        /// Rewritten symbol.
        left: SymbolId,
        /// First source constraint index.
        first_constraint_index: usize,
        /// Second source constraint index.
        second_constraint_index: usize,
        /// Whether both rows have exactly the same right-hand symbol and offset.
        same_rewrite: bool,
    },
    /// A directed cycle appears in the substitution graph.
    ///
    /// For a cycle `x0 = x1 + a0`, `x1 = x2 + a1`, ..., `xn = x0 + an`,
    /// consistency requires `sum(ai) == 0`. Even a consistent cycle is not an
    /// acyclic rewrite graph, so callers should choose a representative with a
    /// union-find pass before applying substitutions.
    DirectedCycle {
        /// Symbols participating in the cycle in discovery order.
        symbols: Vec<SymbolId>,
        /// Exact sum of offsets around the cycle.
        net_offset: Real,
        /// Whether the cycle is exactly consistent.
        consistent: bool,
    },
}

impl EqualitySubstitutionProblem {
    /// Return whether this problem is an exact algebraic contradiction.
    pub fn is_inconsistent(&self) -> bool {
        match self {
            Self::SelfSubstitution { consistent, .. } | Self::DirectedCycle { consistent, .. } => {
                !consistent
            }
            Self::DuplicateLeft { same_rewrite, .. } => !same_rewrite,
        }
    }
}

/// Failure modes for direct affine reduction.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum DirectSolveError {
    /// The row coefficient was structurally unknown, so selecting the variable
    /// would require an unproved branch.
    UnknownCoefficientSign {
        /// Source constraint index.
        constraint_index: usize,
        /// Variable column.
        variable_column: usize,
    },
    /// The exact division needed to isolate a variable failed.
    UnsupportedDivision {
        /// Source constraint index.
        constraint_index: usize,
    },
    /// The exact square root needed for a quadratic root was unsupported.
    UnsupportedSquareRoot {
        /// Source constraint index.
        constraint_index: usize,
    },
}

/// Solve every active one-variable affine equality row exactly.
///
/// This is intentionally conservative. Rows with multiple structurally
/// nonzero coefficients, non-equality kinds, unknown coefficient signs, or
/// unsupported exact division are not silently approximated.
pub fn solve_direct_affine_equalities(
    prepared: &PreparedProblem<'_>,
) -> Result<Vec<DirectAffineSolution>, DirectSolveError> {
    let mut solutions = Vec::new();
    for (constraint_index, constraint) in prepared.problem().constraints.iter().enumerate() {
        if !constraint.active || constraint.kind != ConstraintKind::Equality {
            continue;
        }
        let Some(affine) = &prepared.affine_residuals()[constraint_index] else {
            continue;
        };
        let mut nonzero = None;
        for (column, coefficient) in affine.coefficients().iter().enumerate() {
            match coefficient.structural_facts().sign {
                Some(RealSign::Zero) => {}
                Some(RealSign::Negative | RealSign::Positive) => {
                    if nonzero.replace((column, coefficient)).is_some() {
                        nonzero = None;
                        break;
                    }
                }
                None => {
                    return Err(DirectSolveError::UnknownCoefficientSign {
                        constraint_index,
                        variable_column: column,
                    });
                }
            }
        }
        let Some((column, coefficient)) = nonzero else {
            continue;
        };
        let value = (-affine.constant().clone() / coefficient.clone())
            .map_err(|_| DirectSolveError::UnsupportedDivision { constraint_index })?;
        solutions.push(DirectAffineSolution {
            constraint_index,
            symbol: prepared.problem().variables[column].symbol,
            value,
        });
    }
    Ok(solutions)
}

/// Solve prepared univariate quadratic equality rows exactly when possible.
///
/// This is a proposal-stage analogue of SolveSpace's soluble-alone pass for a
/// bounded nonlinear row. Roots from `a*x^2 + b*x + c = 0` are returned only
/// after exact discriminant construction, exact square-root construction, and
/// exact division. Callers must still replay the full problem with ordinary
/// certification before trusting a root candidate.
pub fn solve_direct_univariate_quadratic_equalities(
    prepared: &PreparedProblem<'_>,
) -> Result<Vec<DirectQuadraticSolution>, DirectSolveError> {
    let mut solutions = Vec::new();
    for (constraint_index, constraint) in prepared.problem().constraints.iter().enumerate() {
        if !constraint.active || constraint.kind != ConstraintKind::Equality {
            continue;
        }
        let Some(quadratic) = &prepared.univariate_quadratic_residuals()[constraint_index] else {
            continue;
        };
        match quadratic.quadratic().structural_facts().sign {
            Some(RealSign::Zero) => continue,
            Some(RealSign::Negative | RealSign::Positive) => {}
            None => {
                return Err(DirectSolveError::UnknownCoefficientSign {
                    constraint_index,
                    variable_column: prepared
                        .problem()
                        .variables
                        .iter()
                        .position(|variable| variable.symbol == quadratic.symbol())
                        .unwrap_or(0),
                });
            }
        }

        let four = Real::from(4);
        let two = Real::from(2);
        let discriminant = quadratic.linear().clone() * quadratic.linear().clone()
            - four * quadratic.quadratic().clone() * quadratic.constant().clone();
        let roots = match discriminant.structural_facts().sign {
            Some(RealSign::Negative) => Vec::new(),
            Some(RealSign::Zero) => {
                let denominator = two * quadratic.quadratic().clone();
                vec![
                    (-quadratic.linear().clone() / denominator)
                        .map_err(|_| DirectSolveError::UnsupportedDivision { constraint_index })?,
                ]
            }
            Some(RealSign::Positive) | None => {
                let sqrt = discriminant
                    .sqrt()
                    .map_err(|_| DirectSolveError::UnsupportedSquareRoot { constraint_index })?;
                let denominator = two * quadratic.quadratic().clone();
                let first = (((-quadratic.linear().clone()) + sqrt.clone()) / denominator.clone())
                    .map_err(|_| DirectSolveError::UnsupportedDivision { constraint_index })?;
                let second = (((-quadratic.linear().clone()) - sqrt) / denominator)
                    .map_err(|_| DirectSolveError::UnsupportedDivision { constraint_index })?;
                vec![first, second]
            }
        };
        solutions.push(DirectQuadraticSolution {
            constraint_index,
            symbol: quadratic.symbol(),
            roots,
        });
    }
    Ok(solutions)
}

/// Find exact two-variable equality substitutions from prepared affine rows.
///
/// Rows of the form `x - y + c = 0` become `x = y - c`. The implementation is
/// intentionally narrow: it accepts only structurally signed unit coefficients
/// and exact equality rows, so this pass cannot silently turn approximate
/// algebra into solver topology.
pub fn find_equality_substitutions(
    prepared: &PreparedProblem<'_>,
) -> Result<Vec<EqualitySubstitution>, DirectSolveError> {
    let mut substitutions = Vec::new();
    for (constraint_index, constraint) in prepared.problem().constraints.iter().enumerate() {
        if !constraint.active || constraint.kind != ConstraintKind::Equality {
            continue;
        }
        let Some(affine) = &prepared.affine_residuals()[constraint_index] else {
            continue;
        };
        let mut positive = None;
        let mut negative = None;
        let mut unsupported = false;
        for (column, coefficient) in affine.coefficients().iter().enumerate() {
            match coefficient.structural_facts().sign {
                Some(RealSign::Zero) => {}
                Some(RealSign::Positive) if *coefficient == Real::one() => {
                    unsupported |= positive.replace(column).is_some();
                }
                Some(RealSign::Negative) if *coefficient == -Real::one() => {
                    unsupported |= negative.replace(column).is_some();
                }
                Some(RealSign::Positive | RealSign::Negative) => {
                    unsupported = true;
                }
                None => {
                    return Err(DirectSolveError::UnknownCoefficientSign {
                        constraint_index,
                        variable_column: column,
                    });
                }
            }
        }
        if unsupported {
            continue;
        }
        let (Some(left_column), Some(right_column)) = (positive, negative) else {
            continue;
        };
        substitutions.push(EqualitySubstitution {
            constraint_index,
            left: prepared.problem().variables[left_column].symbol,
            right: prepared.problem().variables[right_column].symbol,
            offset: -affine.constant().clone(),
        });
    }
    Ok(substitutions)
}

/// Apply exact equality substitutions to a candidate context.
///
/// This is a candidate-update helper, not a proof. It is useful after
/// `find_equality_substitutions` has identified `x = y + c` rows and the
/// caller wants a candidate that satisfies those rows before exact replay. The
/// full residual certification step remains mandatory, preserving Yap's
/// distinction between construction/proposal and certified decision.
pub fn apply_equality_substitutions(
    context: &mut EvaluationContext,
    substitutions: &[EqualitySubstitution],
) -> Result<usize, DirectSolveError> {
    let mut applied = 0;
    for substitution in substitutions {
        let Some(right_value) = context.bindings().get(&substitution.right).cloned() else {
            continue;
        };
        context.bind(substitution.left, right_value + substitution.offset.clone());
        applied += 1;
    }
    Ok(applied)
}

/// Validate exact equality substitutions before using them as rewrites.
///
/// SolveSpace-style substitution relies on orienting simple equations before
/// nonlinear iteration. Yap's exact-computation discipline makes the proof
/// obligation explicit: symbolic reduction may propose cheaper topology, but
/// contradictions and cycles must be represented exactly instead of hidden in
/// floating-point iteration. This function checks self-rewrites, duplicate
/// left-hand sides, and directed cycles with exact offset accumulation.
pub fn validate_equality_substitutions(
    substitutions: &[EqualitySubstitution],
) -> EqualitySubstitutionValidation {
    let mut problems = Vec::new();
    let mut edges: BTreeMap<SymbolId, (SymbolId, Real, usize)> = BTreeMap::new();

    for substitution in substitutions {
        if substitution.left == substitution.right {
            problems.push(EqualitySubstitutionProblem::SelfSubstitution {
                constraint_index: substitution.constraint_index,
                symbol: substitution.left,
                offset: substitution.offset.clone(),
                consistent: substitution.offset == Real::zero(),
            });
        }

        if let Some((right, offset, constraint_index)) = edges.get(&substitution.left) {
            problems.push(EqualitySubstitutionProblem::DuplicateLeft {
                left: substitution.left,
                first_constraint_index: *constraint_index,
                second_constraint_index: substitution.constraint_index,
                same_rewrite: *right == substitution.right && *offset == substitution.offset,
            });
        } else {
            edges.insert(
                substitution.left,
                (
                    substitution.right,
                    substitution.offset.clone(),
                    substitution.constraint_index,
                ),
            );
        }
    }

    let mut globally_seen = BTreeSet::new();
    for &start in edges.keys() {
        if globally_seen.contains(&start) {
            continue;
        }
        let mut path = Vec::new();
        let mut offsets = Vec::new();
        let mut local_index = BTreeMap::new();
        let mut current = start;
        while let Some((next, offset, _)) = edges.get(&current) {
            if let Some(&cycle_start) = local_index.get(&current) {
                let symbols = path[cycle_start..].to_vec();
                let net_offset = offsets[cycle_start..]
                    .iter()
                    .cloned()
                    .fold(Real::zero(), |sum, offset| sum + offset);
                problems.push(EqualitySubstitutionProblem::DirectedCycle {
                    symbols,
                    consistent: net_offset == Real::zero(),
                    net_offset,
                });
                break;
            }
            if globally_seen.contains(&current) {
                break;
            }
            local_index.insert(current, path.len());
            path.push(current);
            offsets.push(offset.clone());
            current = *next;
        }
        globally_seen.extend(path);
    }

    EqualitySubstitutionValidation { problems }
}

/// Build exact equality-substitution classes when the graph is consistent.
///
/// Edges have the form `left = right + offset`. The returned class members use
/// `symbol = representative + offset_from_representative`, which is the shape
/// needed by future union-find and affine-row elimination passes. Consistent
/// cycles are allowed because they still define one equality class; inconsistent
/// cycles and conflicting duplicate rewrites return the validation report.
pub fn build_equality_substitution_classes(
    substitutions: &[EqualitySubstitution],
) -> Result<Vec<EqualitySubstitutionClass>, EqualitySubstitutionValidation> {
    let validation = validate_equality_substitutions(substitutions);
    if validation.has_inconsistency() {
        return Err(validation);
    }

    let mut adjacency: BTreeMap<SymbolId, Vec<(SymbolId, Real)>> = BTreeMap::new();
    for substitution in substitutions {
        adjacency
            .entry(substitution.left)
            .or_default()
            .push((substitution.right, -substitution.offset.clone()));
        adjacency
            .entry(substitution.right)
            .or_default()
            .push((substitution.left, substitution.offset.clone()));
    }

    let mut visited = BTreeSet::new();
    let mut classes = Vec::new();
    for &start in adjacency.keys() {
        if visited.contains(&start) {
            continue;
        }

        let mut offsets_from_start = BTreeMap::new();
        let mut queue = VecDeque::new();
        offsets_from_start.insert(start, Real::zero());
        queue.push_back(start);
        visited.insert(start);

        while let Some(symbol) = queue.pop_front() {
            let Some(neighbors) = adjacency.get(&symbol) else {
                continue;
            };
            let symbol_offset = offsets_from_start
                .get(&symbol)
                .expect("queued symbols must have offsets")
                .clone();
            for (neighbor, delta) in neighbors {
                if visited.insert(*neighbor) {
                    offsets_from_start.insert(*neighbor, symbol_offset.clone() + delta.clone());
                    queue.push_back(*neighbor);
                }
            }
        }

        let representative = offsets_from_start
            .keys()
            .copied()
            .min()
            .expect("component must contain at least the start symbol");
        let representative_offset = offsets_from_start
            .get(&representative)
            .expect("representative must have an offset")
            .clone();
        let members = offsets_from_start
            .into_iter()
            .map(
                |(symbol, offset_from_start)| EqualitySubstitutionClassMember {
                    symbol,
                    offset_from_representative: offset_from_start - representative_offset.clone(),
                },
            )
            .collect();
        classes.push(EqualitySubstitutionClass {
            representative,
            members,
        });
    }

    Ok(classes)
}

/// Find cycles in exact equality-substitution candidates.
///
/// Substitution cycles are not inherently inconsistent, but blindly applying
/// them as a directed rewrite system can loop. SolveSpace-style substitution
/// passes therefore need this diagnostic before choosing a representative or
/// union-find strategy. This function is structural only; it does not decide
/// feasibility.
pub fn find_equality_substitution_cycles(
    substitutions: &[EqualitySubstitution],
) -> Vec<EqualitySubstitutionCycle> {
    let edges = substitutions
        .iter()
        .map(|substitution| (substitution.left, substitution.right))
        .collect::<BTreeMap<_, _>>();
    let mut cycles = Vec::new();
    let mut globally_seen = BTreeSet::new();

    for &start in edges.keys() {
        if globally_seen.contains(&start) {
            continue;
        }
        let mut path = Vec::new();
        let mut local_index = BTreeMap::new();
        let mut current = start;
        while let Some(&next) = edges.get(&current) {
            if let Some(&cycle_start) = local_index.get(&current) {
                cycles.push(EqualitySubstitutionCycle {
                    symbols: path[cycle_start..].to_vec(),
                });
                break;
            }
            if globally_seen.contains(&current) {
                break;
            }
            local_index.insert(current, path.len());
            path.push(current);
            current = next;
        }
        globally_seen.extend(path);
    }
    cycles
}
