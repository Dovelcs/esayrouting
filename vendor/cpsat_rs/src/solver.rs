//! Solver entry point and response type.

use crate::error::SolveError;
use crate::ffi;
use crate::model::CpModel;
use crate::proto::{CpSolverResponse, CpSolverStatus, SatParameters};
use crate::vars::{BoolVar, IntVar};
use prost::Message;

/// The CP-SAT solver.
pub struct CpSolver;

impl CpSolver {
    /// Solve a model with default parameters.
    ///
    /// # Errors
    ///
    /// Returns `SolveError` if the FFI call fails or the response cannot be decoded.
    pub fn solve(model: &CpModel) -> Result<SolveResponse, SolveError> {
        Self::solve_with_params(model, &SatParameters::default())
    }

    /// Solve a model with custom parameters.
    ///
    /// # Errors
    ///
    /// Returns `SolveError` if the FFI call fails or the response cannot be decoded.
    pub fn solve_with_params(
        model: &CpModel,
        params: &SatParameters,
    ) -> Result<SolveResponse, SolveError> {
        let model_bytes = model.to_bytes();
        let params_bytes = params.to_bytes();
        let response_bytes = ffi::solve_raw(&model_bytes, Some(&params_bytes))?;
        let proto = CpSolverResponse::decode(response_bytes.as_slice())?;
        Ok(SolveResponse { proto })
    }
}

/// Response from the solver, containing status, solution values, and statistics.
#[must_use]
pub struct SolveResponse {
    proto: CpSolverResponse,
}

impl SolveResponse {
    /// Solver status (Optimal, Feasible, Infeasible, Unknown, ModelInvalid).
    pub fn status(&self) -> CpSolverStatus {
        self.proto.status()
    }

    /// Whether the solver found and proved an optimal solution.
    pub fn is_optimal(&self) -> bool {
        self.proto.status() == CpSolverStatus::Optimal
    }

    /// Whether the solver found a feasible solution (optimal or not).
    pub fn is_feasible(&self) -> bool {
        matches!(
            self.proto.status(),
            CpSolverStatus::Optimal | CpSolverStatus::Feasible
        )
    }

    /// Objective value of the best solution found.
    pub fn objective_value(&self) -> f64 {
        self.proto.objective_value
    }

    /// Best objective bound proved by the solver.
    pub fn best_bound(&self) -> f64 {
        self.proto.best_objective_bound
    }

    /// Wall clock time spent solving, in seconds.
    pub fn wall_time(&self) -> f64 {
        self.proto.wall_time
    }

    /// Get the value of an integer variable in the solution.
    ///
    /// # Panics
    ///
    /// Panics if the status is not `Optimal` or `Feasible`,
    /// or if the variable index is out of bounds.
    pub fn value(&self, var: IntVar) -> i64 {
        assert!(
            self.is_feasible(),
            "Cannot read solution values: status is {:?}",
            self.status()
        );
        assert!(
            (var.index() as usize) < self.proto.solution.len(),
            "Variable index {} out of bounds (solution has {} values)",
            var.index(),
            self.proto.solution.len()
        );
        self.proto.solution[var.index() as usize]
    }

    /// Get the value of a Boolean variable in the solution.
    ///
    /// # Panics
    ///
    /// Panics if the status is not `Optimal` or `Feasible`,
    /// if the variable is negated, or if the index is out of bounds.
    pub fn bool_value(&self, var: BoolVar) -> bool {
        assert!(
            self.is_feasible(),
            "Cannot read solution values: status is {:?}",
            self.status()
        );
        assert!(
            !var.is_negated(),
            "Cannot read negated BoolVar value directly. \
             Use the non-negated variable."
        );
        self.proto.solution[var.index() as usize] != 0
    }

    /// Assumptions sufficient for infeasibility (when status is Infeasible
    /// and assumptions were provided).
    pub fn sufficient_assumptions_for_infeasibility(&self) -> &[i32] {
        &self.proto.sufficient_assumptions_for_infeasibility
    }

    /// Access the raw response proto.
    pub fn raw_proto(&self) -> &CpSolverResponse {
        &self.proto
    }
}

impl std::fmt::Display for SolveResponse {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(
            f,
            "SolveResponse(status={:?}, objective={:.1}, bound={:.1}, time={:.3}s)",
            self.status(),
            self.objective_value(),
            self.best_bound(),
            self.wall_time(),
        )
    }
}
