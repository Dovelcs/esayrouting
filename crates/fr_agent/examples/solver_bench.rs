use fr_agent::planner::*;
use std::time::Instant;
fn main() {
    let start = Instant::now();
    let mut p = PlannerProblem {
        selection_sets: vec![],
        candidates: vec![],
        conflicts: vec![],
        capacities: vec![],
    };
    for n in 0..200 {
        let mut ids = vec![];
        for k in 0..6 {
            let id = format!("{n}_{k}");
            ids.push(id.clone());
            p.candidates.push(PlanCandidate {
                id,
                selection_set_id: n.to_string(),
                group_id: "bench".into(),
                cost: (k * 7 + (n * k) % 13) as i64,
                legs: vec![],
                vias: vec![],
                reservations: vec![],
                requires_any: vec![],
            });
        }
        p.selection_sets.push(SelectionSet {
            id: n.to_string(),
            group_id: "bench".into(),
            candidate_ids: ids,
        });
    }
    let generation = start.elapsed();
    let start = Instant::now();
    for n in 0..199 {
        for k in 0..6 {
            p.conflicts
                .push((format!("{n}_{k}"), format!("{}_{k}", n + 1)));
        }
    }
    let conflicts = start.elapsed();
    for w in [1, 8] {
        let wall = Instant::now();
        let r = solve_joint(&p, w, 5000, 42).unwrap();
        println!("workers={w} decisions={} candidates={} conflicts={} generation_us={} conflict_us={} solve_ms={} status={:?} objective={:?} bound={:?}",p.selection_sets.len(),p.candidates.len(),p.conflicts.len(),generation.as_micros(),conflicts.as_micros(),wall.elapsed().as_millis(),r.status,r.objective,r.best_bound);
    }
}
