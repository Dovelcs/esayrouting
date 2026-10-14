use fr_agent::{
    board_adapter::BoardAdapter, conflict::build_conflicts, execution::execute_selected,
    planner::*, spec::RouteSpec,
};
use std::time::{Duration, Instant};
#[test]
fn native_neighbor_exit_survives_global_escape_choice() {
    let source = include_bytes!("fixtures/two_chip.dsn");
    let board = BoardAdapter::load(source).unwrap();
    let unchanged = board.ses();
    let spec: RouteSpec = toml::from_str(
        r#"version=1
 route_remaining=false
 input="board.dsn"
 output="board.ses"
 [[nets]]
 name="BUS"
 width_mm=0.15
 clearance_mm=0.1
 allowed_layers=["Top"]
 via_budget=0
 max_length_mm=40.0
 [[groups]]
 id="bus"
 nets=["BUS"]
 joint_escape=true
 [execution]
 threads=8
 budget_ms=5000
 seed=1
 "#,
    )
    .unwrap();
    let route = |id: &str, cost: i64, points: Vec<Point>| PlanCandidate {
        id: id.into(),
        selection_set_id: "bus_connection".into(),
        group_id: "bus".into(),
        cost,
        legs: vec![RouteLeg {
            net: "BUS".into(),
            layer: "Top".into(),
            points,
            width_mm: 0.15,
            clearance_mm: 0.1,
        }],
        vias: vec![],
        reservations: vec![],
        requires_any: vec![],
    };
    let greedy = route("greedy", 16000, vec![[2.0, 4.0], [18.0, 4.0]]);
    let alternative = route(
        "alternative",
        20000,
        vec![[2.0, 4.0], [2.0, 2.0], [18.0, 2.0], [18.0, 4.0]],
    );
    // U1's adjacent non-bus pad at (2,5) retains its single reserved southeast exit corridor.
    let neighbor = PlanCandidate {
        id: "neighbor_exit".into(),
        selection_set_id: "neighbor_pad".into(),
        group_id: "bus".into(),
        cost: 0,
        legs: vec![],
        vias: vec![],
        reservations: vec![ReservedRegion {
            owner_net: "OTHER".into(),
            layer: "Top".into(),
            min: [2.2, 3.8],
            max: [4.2, 5.2],
        }],
        requires_any: vec![],
    };
    board
        .check_candidate(&greedy, &spec)
        .expect("cheap route is individually native-feasible");
    let length = |c: &PlanCandidate| {
        c.legs
            .iter()
            .flat_map(|l| l.points.windows(2))
            .map(|s| (s[1][0] - s[0][0]).hypot(s[1][1] - s[0][1]))
            .sum::<f64>()
    };
    assert!(length(&greedy) < length(&alternative));
    assert_eq!(greedy.cost, (length(&greedy) * 1000.0).round() as i64);
    assert_eq!(
        alternative.cost,
        (length(&alternative) * 1000.0).round() as i64
    );
    let candidates = vec![greedy, alternative, neighbor];
    let deadline = Instant::now() + Duration::from_secs(5);
    let conflicts = build_conflicts(&board, &spec, &candidates, deadline).unwrap();
    assert!(conflicts.contains(&("greedy".into(), "neighbor_exit".into())));
    let problem = PlannerProblem {
        selection_sets: vec![
            SelectionSet {
                id: "bus_connection".into(),
                group_id: "bus".into(),
                candidate_ids: vec!["greedy".into(), "alternative".into()],
            },
            SelectionSet {
                id: "neighbor_pad".into(),
                group_id: "bus".into(),
                candidate_ids: vec!["neighbor_exit".into()],
            },
        ],
        candidates,
        conflicts,
        capacities: vec![],
    };
    let result = solve_joint(&problem, 8, 1000, 1).unwrap();
    assert_eq!(result.status, PlanStatus::Optimal);
    assert!(result.selected_ids.contains(&"alternative".into()));
    assert!(!result.selected_ids.contains(&"greedy".into()));
    let routed = execute_selected(&board, &spec, &problem, &result.selected_ids, deadline).unwrap();
    assert_eq!(routed.violations(), board.violations());
    let bus = routed.net_number("BUS").unwrap();
    assert_eq!(
        fr_engine::drc::NetIncompletes::new(
            &routed.board,
            bus,
            &routed.board.get_connectable_items(bus)
        )
        .count(),
        0
    );
    let reloaded = BoardAdapter::load_session(source, &routed.ses()).unwrap();
    let bus = reloaded.net_number("BUS").unwrap();
    assert_eq!(
        fr_engine::drc::NetIncompletes::new(
            &reloaded.board,
            bus,
            &reloaded.board.get_connectable_items(bus)
        )
        .count(),
        0
    );
    assert_eq!(reloaded.violations(), board.violations());
    assert_eq!(board.ses(), unchanged);
}
#[test]
fn generated_joint_candidates_couple_escape_route_and_commit() {
    let source = include_bytes!("fixtures/two_chip.dsn");
    let board = BoardAdapter::load(source).unwrap();
    let original = board.ses();
    let spec: RouteSpec = toml::from_str(
        r#"version=1
route_remaining=false
input="board.dsn"
output="board.ses"
[[nets]]
name="BUS"
width_mm=0.15
clearance_mm=0.1
allowed_layers=["Top"]
via_budget=0
max_length_mm=25.0
[[groups]]
id="bus"
nets=["BUS"]
joint_escape=true
[execution]
threads=8
budget_ms=5000
seed=42
"#,
    )
    .unwrap();
    let deadline = Instant::now() + Duration::from_secs(5);
    let problem = fr_agent::execution::generate_problem(&board, &spec, deadline).unwrap();
    assert!(
        problem.selection_sets.len() >= 3,
        "source and destination escape plus complete connection decisions required"
    );
    assert!(
        problem
            .candidates
            .iter()
            .any(|c| c.selection_set_id.starts_with("connect:") && !c.requires_any.is_empty()),
        "generated route must depend on actual escape choices"
    );
    assert!(
        problem
            .candidates
            .iter()
            .any(|c| c.reservations.iter().any(|r| r.owner_net == "OTHER")),
        "adjacent non-bus pads must receive protected physical exits"
    );
    let outcome = solve_joint(&problem, 8, 1000, 42).unwrap();
    assert!(matches!(
        outcome.status,
        PlanStatus::Optimal | PlanStatus::Feasible
    ));
    let routed =
        execute_selected(&board, &spec, &problem, &outcome.selected_ids, deadline).unwrap();
    assert!(routed.violations().is_subset(&board.violations()));
    let n = routed.net_number("BUS").unwrap();
    assert_eq!(
        fr_engine::drc::NetIncompletes::new(
            &routed.board,
            n,
            &routed.board.get_connectable_items(n)
        )
        .count(),
        0
    );
    let reloaded = BoardAdapter::load_session(source, &routed.ses()).unwrap();
    let n = reloaded.net_number("BUS").unwrap();
    assert_eq!(
        fr_engine::drc::NetIncompletes::new(
            &reloaded.board,
            n,
            &reloaded.board.get_connectable_items(n)
        )
        .count(),
        0
    );
    assert_eq!(board.ses(), original);
}
