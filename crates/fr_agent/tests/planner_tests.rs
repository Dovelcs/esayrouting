use fr_agent::planner::*;
fn candidate(id: &str, set: &str, cost: i64) -> PlanCandidate {
    PlanCandidate {
        id: id.into(),
        selection_set_id: set.into(),
        group_id: "g".into(),
        cost,
        legs: vec![],
        vias: vec![],
        reservations: vec![],
        requires_any: vec![],
    }
}
fn problem() -> PlannerProblem {
    PlannerProblem {
        selection_sets: vec![
            SelectionSet {
                id: "a".into(),
                group_id: "g".into(),
                candidate_ids: vec!["a0".into(), "a1".into()],
            },
            SelectionSet {
                id: "b".into(),
                group_id: "g".into(),
                candidate_ids: vec!["b0".into(), "b1".into()],
            },
        ],
        candidates: vec![
            candidate("a0", "a", 0),
            candidate("a1", "a", 3),
            candidate("b0", "b", 0),
            candidate("b1", "b", 8),
        ],
        conflicts: vec![("a0".into(), "b0".into())],
        capacities: vec![],
    }
}
#[test]
fn global_choice_beats_greedy() {
    let r = solve_joint(&problem(), 2, 1000, 1).unwrap();
    assert_eq!(r.status, PlanStatus::Optimal);
    assert_eq!(r.objective, Some(3));
    assert!(r.selected_ids.contains(&"a1".into()));
}
#[test]
fn incompatible_escape_channel_is_infeasible() {
    let mut p = problem();
    p.candidates[0].requires_any = vec![vec!["b0".into()]];
    p.candidates[1].requires_any = vec![vec!["b0".into()]];
    p.capacities.push(CapacityConstraint {
        candidate_ids: vec!["a1".into(), "b0".into()],
        capacity: 1,
    });
    assert_eq!(
        solve_joint(&p, 1, 1000, 1).unwrap().status,
        PlanStatus::Infeasible
    );
}
#[test]
fn zero_budget_is_unknown() {
    let r = solve_joint(&problem(), 1, 0, 1).unwrap();
    assert_eq!(r.status, PlanStatus::Unknown);
    assert!(r.selected_ids.is_empty());
    assert_eq!(r.objective, None);
}
#[test]
fn selection_checks_independent() {
    let p = problem();
    assert!(p.validate_selection(&["a0".into(), "b0".into()]).is_err());
    assert!(p.validate_selection(&["a1".into()]).is_err());
    assert_eq!(
        p.validate_selection(&["a1".into(), "b0".into()]).unwrap(),
        3
    );
}
#[test]
fn unknown_reference_is_error() {
    let mut p = problem();
    p.conflicts.push(("missing".into(), "a0".into()));
    assert!(solve_joint(&p, 1, 1000, 1).is_err());
}
#[test]
fn empty_decision_is_infeasible() {
    let mut p = problem();
    p.selection_sets.push(SelectionSet {
        id: "empty".into(),
        group_id: "g".into(),
        candidate_ids: vec![],
    });
    assert_eq!(
        solve_joint(&p, 1, 1000, 1).unwrap().status,
        PlanStatus::Infeasible
    );
}
#[test]
fn cross_decision_alias_is_rejected() {
    let mut p = problem();
    p.selection_sets[0].candidate_ids.push("b0".into());
    assert!(p.validate().is_err());
    assert!(solve_joint(&p, 1, 1000, 1).is_err());
}
#[test]
fn integer_objective_overflow_is_rejected() {
    let mut p = problem();
    p.candidates[0].cost = i64::MAX;
    assert!(p.validate().is_err());
}
#[test]
fn invalid_geometry_is_rejected() {
    let mut p = problem();
    p.candidates[0].legs.push(RouteLeg {
        net: "n".into(),
        layer: "F.Cu".into(),
        points: vec![[0.0, 0.0], [f64::NAN, 1.0]],
        width_mm: 0.2,
        clearance_mm: 0.2,
    });
    assert!(p.validate().is_err());
}
#[test]
fn no_good_replans_joint_assignment() {
    let mut p = problem();
    p.add_no_good(vec!["a1".into(), "b0".into()]).unwrap();
    let r = solve_joint(&p, 1, 1000, 1).unwrap();
    assert_eq!(r.status, PlanStatus::Optimal);
    assert_eq!(r.objective, Some(8));
    p.ban_candidate("b1").unwrap();
    assert_eq!(
        solve_joint(&p, 1, 1000, 1).unwrap().status,
        PlanStatus::Infeasible
    );
}
#[test]
fn candidate_model_matches_exhaustive_oracle() {
    for mask in 0u32..16 {
        let mut p = problem();
        p.conflicts.clear();
        for i in 0..2 {
            for j in 0..2 {
                if mask & (1 << (2 * i + j)) != 0 {
                    p.conflicts.push((format!("a{i}"), format!("b{j}")));
                }
            }
        }
        let oracle = (0..2)
            .flat_map(|i| (0..2).map(move |j| vec![format!("a{i}"), format!("b{j}")]))
            .filter_map(|ids| p.validate_selection(&ids).ok())
            .min();
        let outcome = solve_joint(&p, 1, 1000, 2).unwrap();
        assert_eq!(outcome.objective, oracle, "conflict mask {mask}");
        assert_eq!(
            outcome.status,
            if oracle.is_some() {
                PlanStatus::Optimal
            } else {
                PlanStatus::Infeasible
            }
        );
    }
}
