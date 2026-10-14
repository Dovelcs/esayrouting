//! OR-Tools CP-SAT 9.15 native CPU backend; no GPU execution is claimed.
use crate::planner::*;
use cpsat_rs::prelude::*;
use std::collections::BTreeMap;
pub fn solve_joint(
    problem: &PlannerProblem,
    workers: usize,
    budget_ms: u64,
    seed: u64,
) -> Result<PlanOutcome, String> {
    problem.validate()?;
    if workers == 0 || workers > i32::MAX as usize {
        return Err("invalid workers".into());
    }
    let mut model = CpModel::new();
    let vars: BTreeMap<_, _> = problem
        .candidates
        .iter()
        .map(|c| (c.id.clone(), model.new_bool_var(&c.id)))
        .collect();
    for s in &problem.selection_sets {
        if s.candidate_ids.is_empty() {
            return Ok(PlanOutcome {
                solver_scope: "candidate_model".into(),
                status: PlanStatus::Infeasible,
                selected_ids: vec![],
                objective: None,
                best_bound: None,
                wall_time_seconds: 0.0,
            });
        }
        model.add_exactly_one(
            &s.candidate_ids
                .iter()
                .map(|id| vars[id])
                .collect::<Vec<_>>(),
        );
    }
    for (a, b) in &problem.conflicts {
        model.add_bool_or(&[!vars[a], !vars[b]]);
    }
    for cap in &problem.capacities {
        let mut expr = LinearExpr::constant(0);
        for id in &cap.candidate_ids {
            expr = expr + LinearExpr::from(vars[id]);
        }
        model.add(expr.le(i64::try_from(cap.capacity).map_err(|_| "capacity overflow")?));
    }
    for c in &problem.candidates {
        for alternatives in &c.requires_any {
            let mut clause = vec![!vars[&c.id]];
            clause.extend(alternatives.iter().map(|id| vars[id]));
            model.add_bool_or(&clause);
        }
    }
    let mut objective = LinearExpr::constant(0);
    for c in &problem.candidates {
        objective = objective + LinearExpr::from(vars[&c.id]) * c.cost;
    }
    model.minimize(objective);
    let parameters = SatParameters::default()
        .with_num_workers(workers as i32)
        .with_max_time(budget_ms as f64 / 1000.0)
        .with_random_seed((seed % i32::MAX as u64) as i32);
    let response = CpSolver::solve_with_params(&model, &parameters).map_err(|e| e.to_string())?;
    let status = match response.status() {
        CpSolverStatus::Optimal => PlanStatus::Optimal,
        CpSolverStatus::Feasible => PlanStatus::Feasible,
        CpSolverStatus::Infeasible => PlanStatus::Infeasible,
        CpSolverStatus::Unknown => PlanStatus::Unknown,
        CpSolverStatus::ModelInvalid => PlanStatus::ModelInvalid,
    };
    let selected_ids = if response.is_feasible() {
        problem
            .candidates
            .iter()
            .filter(|c| response.bool_value(vars[&c.id]))
            .map(|c| c.id.clone())
            .collect()
    } else {
        vec![]
    };
    let cost = if response.is_feasible() {
        Some(problem.validate_selection(&selected_ids)?)
    } else {
        None
    };
    Ok(PlanOutcome {
        solver_scope: "candidate_model".into(),
        status,
        selected_ids,
        objective: cost,
        best_bound: response.is_feasible().then(|| response.best_bound()),
        wall_time_seconds: response.wall_time(),
    })
}
