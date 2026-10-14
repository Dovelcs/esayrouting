//! Discrete candidate joint planning, following AWX joint_escape.py at
//! caad45cd252f9a50f1fb4b3b15461bdf1b9457a1 (MIT). Geometry generation is
//! separate: this is not a complete AWX port. CP-SAT-LP: CP 2023 paper 3.
use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};
pub type Point = [f64; 2];
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RouteLeg {
    pub net: String,
    pub layer: String,
    pub points: Vec<Point>,
    pub width_mm: f64,
    pub clearance_mm: f64,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlanVia {
    pub padstack: String,
    pub net: String,
    pub point: Point,
    pub from_layer: String,
    pub to_layer: String,
    pub diameter_mm: f64,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ReservedRegion {
    pub owner_net: String,
    pub layer: String,
    pub min: Point,
    pub max: Point,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlanCandidate {
    pub id: String,
    pub selection_set_id: String,
    pub group_id: String,
    pub cost: i64,
    pub legs: Vec<RouteLeg>,
    pub vias: Vec<PlanVia>,
    pub reservations: Vec<ReservedRegion>,
    pub requires_any: Vec<Vec<String>>,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SelectionSet {
    pub id: String,
    pub group_id: String,
    pub candidate_ids: Vec<String>,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CapacityConstraint {
    pub candidate_ids: Vec<String>,
    pub capacity: usize,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlannerProblem {
    pub selection_sets: Vec<SelectionSet>,
    pub candidates: Vec<PlanCandidate>,
    pub conflicts: Vec<(String, String)>,
    pub capacities: Vec<CapacityConstraint>,
}
#[derive(Debug, Clone, Copy, Serialize, Deserialize, PartialEq, Eq)]
#[serde(rename_all = "snake_case")]
pub enum PlanStatus {
    Optimal,
    Feasible,
    Infeasible,
    Unknown,
    ModelInvalid,
}
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct PlanOutcome {
    pub solver_scope: String,
    pub status: PlanStatus,
    pub selected_ids: Vec<String>,
    pub objective: Option<i64>,
    pub best_bound: Option<f64>,
    pub wall_time_seconds: f64,
}
impl PlannerProblem {
    /// Forbid one failed checked-insertion candidate without changing decision membership.
    pub fn ban_candidate(&mut self, id: &str) -> Result<(), String> {
        if !self.candidates.iter().any(|c| c.id == id) {
            return Err("unknown banned candidate".into());
        }
        self.conflicts.push((id.to_owned(), id.to_owned()));
        Ok(())
    }
    /// Exclude a failed combination while retaining all other joint assignments.
    pub fn add_no_good(&mut self, ids: Vec<String>) -> Result<(), String> {
        if ids.is_empty() {
            return Err("empty no-good".into());
        }
        let unique: BTreeSet<_> = ids.iter().collect();
        if unique.len() != ids.len()
            || ids
                .iter()
                .any(|id| !self.candidates.iter().any(|c| c.id == *id))
        {
            return Err("invalid no-good members".into());
        }
        let capacity = ids.len() - 1;
        self.capacities.push(CapacityConstraint {
            candidate_ids: ids,
            capacity,
        });
        Ok(())
    }
    pub fn validate(&self) -> Result<(), String> {
        let mut ids = BTreeSet::new();
        let mut decisions = BTreeMap::new();
        for s in &self.selection_sets {
            if s.id.trim().is_empty()
                || s.group_id.trim().is_empty()
                || decisions.insert(&s.id, s).is_some()
            {
                return Err("duplicate/empty selection set".into());
            }
            let unique: BTreeSet<_> = s.candidate_ids.iter().collect();
            if unique.len() != s.candidate_ids.len() {
                return Err("duplicate decision member".into());
            }
        }
        for c in &self.candidates {
            if c.id.trim().is_empty() || c.group_id.trim().is_empty() || !ids.insert(&c.id) {
                return Err("duplicate/empty candidate".into());
            }
            if c.cost < 0 {
                return Err("negative candidate cost".into());
            }
            let s = decisions
                .get(&c.selection_set_id)
                .ok_or("unknown selection set")?;
            if s.group_id != c.group_id || !s.candidate_ids.contains(&c.id) {
                return Err("candidate membership mismatch".into());
            }
            for l in &c.legs {
                if l.points.len() < 2
                    || l.width_mm <= 0.0
                    || l.clearance_mm < 0.0
                    || !l.width_mm.is_finite()
                    || !l.clearance_mm.is_finite()
                    || l.points.iter().flatten().any(|x| !x.is_finite())
                {
                    return Err("invalid route geometry".into());
                }
            }
            for v in &c.vias {
                if v.diameter_mm <= 0.0
                    || !v.diameter_mm.is_finite()
                    || v.point.iter().any(|x| !x.is_finite())
                    || v.from_layer == v.to_layer
                {
                    return Err("invalid via".into());
                }
            }
            for r in &c.reservations {
                if r.min.iter().chain(r.max.iter()).any(|x| !x.is_finite())
                    || (0..2).any(|i| r.min[i] >= r.max[i])
                {
                    return Err("invalid reserved region".into());
                }
            }
        }
        let candidate_by_id: BTreeMap<_, _> = self.candidates.iter().map(|c| (&c.id, c)).collect();
        let known = |id: &String| ids.contains(id);
        for s in &self.selection_sets {
            if s.candidate_ids.iter().any(|x| !known(x)) {
                return Err("unknown decision member".into());
            }
            for id in &s.candidate_ids {
                let c = candidate_by_id[id];
                if c.selection_set_id != s.id || c.group_id != s.group_id {
                    return Err("reverse candidate membership mismatch".into());
                }
            }
        }
        for c in &self.candidates {
            if c.requires_any.iter().flatten().any(|x| !known(x)) {
                return Err("unknown prerequisite".into());
            }
        }
        for (a, b) in &self.conflicts {
            if !known(a) || !known(b) {
                return Err("unknown conflict member".into());
            }
        }
        for cap in &self.capacities {
            let unique: BTreeSet<_> = cap.candidate_ids.iter().collect();
            if unique.len() != cap.candidate_ids.len()
                || cap.candidate_ids.iter().any(|x| !known(x))
            {
                return Err("invalid capacity members".into());
            }
        }
        let total = self.candidates.iter().try_fold(0i64, |sum, c| {
            sum.checked_add(c.cost).ok_or("objective overflow")
        })?;
        if total > i64::MAX / 2 {
            return Err("objective exceeds CP-SAT safe integer range".into());
        }
        Ok(())
    }
    pub fn validate_selection(&self, selected: &[String]) -> Result<i64, String> {
        self.validate()?;
        let chosen: BTreeSet<_> = selected.iter().collect();
        if chosen.len() != selected.len() {
            return Err("duplicate selected id".into());
        }
        let all: BTreeSet<_> = self.candidates.iter().map(|c| &c.id).collect();
        if !chosen.is_subset(&all) {
            return Err("unknown selected id".into());
        }
        for s in &self.selection_sets {
            if s.candidate_ids
                .iter()
                .filter(|id| chosen.contains(id))
                .count()
                != 1
            {
                return Err(format!("decision {} not satisfied", s.id));
            }
        }
        for (a, b) in &self.conflicts {
            if chosen.contains(a) && chosen.contains(b) {
                return Err("conflicting selection".into());
            }
        }
        for cap in &self.capacities {
            if cap
                .candidate_ids
                .iter()
                .filter(|id| chosen.contains(id))
                .count()
                > cap.capacity
            {
                return Err("capacity exceeded".into());
            }
        }
        let mut cost = 0;
        for c in &self.candidates {
            if chosen.contains(&c.id) {
                for alternatives in &c.requires_any {
                    if !alternatives.iter().any(|id| chosen.contains(id)) {
                        return Err("prerequisite not satisfied".into());
                    }
                }
                cost += c.cost
            }
        }
        Ok(cost)
    }
}
pub use crate::solver::solve_joint;
