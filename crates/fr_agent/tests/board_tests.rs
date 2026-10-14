use fr_agent::{
    board_adapter::{segment_in_channel, BoardAdapter},
    execution::{execute_selected, generate_problem},
    planner::*,
    spec::*,
};
use std::time::{Duration, Instant};
fn board() -> BoardAdapter {
    BoardAdapter::load(include_bytes!("fixtures/two_chip.dsn")).unwrap()
}
fn spec() -> RouteSpec {
    RouteSpec {
        version: 1,
        input: "board.dsn".into(),
        output: "board.ses".into(),
        nets: vec![NetRule {
            name: "BUS".into(),
            width_mm: 0.15,
            clearance_mm: 0.1,
            allowed_layers: vec!["Top".into()],
            via_budget: Some(0),
            max_length_mm: Some(25.0),
            protected: false,
        }],
        endpoint_groups: vec![],
        channels: vec![],
        groups: vec![RouteGroup {
            id: "bus".into(),
            nets: vec!["BUS".into()],
            endpoint_groups: vec![],
            channels: vec![],
            priority: 0,
            depends_on: vec![],
            joint_escape: true,
            awx: None,
        }],
        execution: Execution::default(),
        advanced: None,
        route_remaining: false,
        allow_partial_output: false,
        protected_nets: vec![],
        awx: None,
    }
}
fn c(id: &str, net: &str, points: Vec<[f64; 2]>) -> PlanCandidate {
    PlanCandidate {
        id: id.into(),
        selection_set_id: id.into(),
        group_id: "bus".into(),
        cost: 0,
        legs: vec![RouteLeg {
            net: net.into(),
            layer: "Top".into(),
            points,
            width_mm: 0.15,
            clearance_mm: 0.1,
        }],
        vias: vec![],
        reservations: vec![],
        requires_any: vec![],
    }
}
#[test]
fn unit_mm_mil_inch_transform() {
    for (unit, scale) in [("mm", 1.0), ("mil", 1.0 / 0.0254), ("inch", 1.0 / 25.4)] {
        let src=format!("(pcb units.dsn (resolution {unit} 10000) (unit {unit}) (structure (layer Top (type signal)) (boundary (rect pcb 0 0 {} {})) (rule (width {}) (clearance {}))) (library (image CHIP (pin PAD 1 0 0)) (padstack PAD (shape (circle Top {})))) (placement (component CHIP (place U1 {} {} front 0))) (network (net N (pins U1-1))))",20.0*scale,10.0*scale,0.15*scale,0.1*scale,0.4*scale,2.0*scale,4.0*scale);
        let b = BoardAdapter::load(src.as_bytes()).unwrap();
        let p = &b.pads()[0];
        assert!(
            (p.point[0] - 2.0).abs() <= 1.0 / b.units_per_mm,
            "{unit}: {:?}",
            p.point
        );
        assert!((p.point[1] - 4.0).abs() <= 1.0 / b.units_per_mm);
    }
}
#[test]
fn channel_envelope_and_nonconvex_chord() {
    let rect = ChannelRegion::Rectangle {
        min: [0.0, 0.0],
        max: [10.0, 2.0],
    };
    assert!(!segment_in_channel([1.0, 0.05], [9.0, 0.05], 0.1, &rect));
    assert!(segment_in_channel([1.0, 1.0], [9.0, 1.0], 0.1, &rect));
    let elbow = ChannelRegion::Polyline {
        points: vec![[0.0, 0.0], [5.0, 0.0], [5.0, 5.0]],
        width_mm: 1.0,
    };
    assert!(!segment_in_channel([0.0, 0.0], [5.0, 5.0], 0.1, &elbow));
}
#[test]
fn group_failure_rolls_back_original_and_protected_net() {
    let b = board();
    let original = b.ses();
    let mut s = spec();
    s.nets.push(NetRule {
        name: "OTHER".into(),
        width_mm: 0.15,
        clearance_mm: 0.1,
        allowed_layers: vec!["Top".into()],
        via_budget: Some(0),
        max_length_mm: None,
        protected: true,
    });
    let a = c("good", "BUS", vec![[2.0, 4.0], [18.0, 4.0]]);
    let bad = c("bad", "OTHER", vec![[2.0, 5.0], [18.0, 5.0]]);
    let problem = PlannerProblem {
        selection_sets: vec![
            SelectionSet {
                id: "good".into(),
                group_id: "bus".into(),
                candidate_ids: vec!["good".into()],
            },
            SelectionSet {
                id: "bad".into(),
                group_id: "bus".into(),
                candidate_ids: vec!["bad".into()],
            },
        ],
        candidates: vec![a, bad],
        conflicts: vec![],
        capacities: vec![],
    };
    assert!(execute_selected(
        &b,
        &s,
        &problem,
        &["good".into(), "bad".into()],
        Instant::now() + Duration::from_secs(5)
    )
    .is_err());
    assert_eq!(b.ses(), original);
}
#[test]
fn route_outside_channel_rejected() {
    let b = board();
    let mut s = spec();
    s.channels.push(Channel {
        id: "narrow".into(),
        region: ChannelRegion::Rectangle {
            min: [1.0, 3.0],
            max: [19.0, 4.0],
        },
    });
    s.groups[0].channels.push("narrow".into());
    assert!(b
        .check_candidate(&c("connect:x", "BUS", vec![[2.0, 4.0], [18.0, 4.0]]), &s)
        .is_err());
}
#[test]
fn neighboring_pad_escape_is_jointly_reserved() {
    let b = board();
    let s = spec();
    let problem = generate_problem(&b, &s, Instant::now() + Duration::from_secs(30)).unwrap();
    assert!(problem.selection_sets.iter().any(|x| x.id.contains("U1.2")));
    assert!(problem.selection_sets.iter().any(|x| x.id.contains("U2.2")));
    assert!(problem
        .candidates
        .iter()
        .any(|c| c.reservations.iter().any(|r| r.owner_net == "OTHER")));
    let outcome = solve_joint(&problem, 1, 10000, 0).unwrap();
    assert!(matches!(
        outcome.status,
        PlanStatus::Optimal | PlanStatus::Feasible
    ));
    let result = execute_selected(
        &b,
        &s,
        &problem,
        &outcome.selected_ids,
        Instant::now() + Duration::from_secs(10),
    )
    .unwrap();
    result.assert_preserved(&b).unwrap();
    assert!(result.connectivity()["BUS"].len() == 1);
    assert_eq!(result.connectivity()["OTHER"], b.connectivity()["OTHER"]);
}

#[test]
fn checked_native_via_transition_and_budget() {
    let b = board();
    let original = b.ses();
    let mut s = spec();
    s.nets[0].allowed_layers.push("Bottom".into());
    s.nets[0].via_budget = Some(2);
    let mut plan = c("via_route", "BUS", vec![[2.0, 4.0], [4.0, 4.0]]);
    plan.legs.push(RouteLeg {
        net: "BUS".into(),
        layer: "Bottom".into(),
        points: vec![[4.0, 4.0], [16.0, 4.0]],
        width_mm: 0.15,
        clearance_mm: 0.1,
    });
    plan.legs.push(RouteLeg {
        net: "BUS".into(),
        layer: "Top".into(),
        points: vec![[16.0, 4.0], [18.0, 4.0]],
        width_mm: 0.15,
        clearance_mm: 0.1,
    });
    for point in [[4.0, 4.0], [16.0, 4.0]] {
        plan.vias.push(PlanVia {
            net: "BUS".into(),
            point,
            from_layer: "Top".into(),
            to_layer: "Bottom".into(),
            diameter_mm: 0.5,
            padstack: "VIA".into(),
        });
    }
    let problem = PlannerProblem {
        selection_sets: vec![SelectionSet {
            id: "via_route".into(),
            group_id: "bus".into(),
            candidate_ids: vec!["via_route".into()],
        }],
        candidates: vec![plan],
        conflicts: vec![],
        capacities: vec![],
    };
    let trial = execute_selected(
        &b,
        &s,
        &problem,
        &["via_route".into()],
        Instant::now() + Duration::from_secs(5),
    )
    .unwrap();
    assert_eq!(trial.connectivity()["BUS"].len(), 1);
    assert_eq!(
        trial
            .board
            .item_repository()
            .iter()
            .filter(|&k| trial.board.item(k).is_via())
            .count(),
        2
    );
    s.nets[0].via_budget = Some(1);
    assert!(execute_selected(
        &b,
        &s,
        &problem,
        &["via_route".into()],
        Instant::now() + Duration::from_secs(5)
    )
    .is_err());
    assert_eq!(b.ses(), original);
}
#[test]
fn public_escape_cannot_bypass_channel_with_long_path() {
    let b = board();
    let s = spec();
    assert!(b
        .check_candidate(
            &c("escape:forged", "BUS", vec![[2.0, 4.0], [18.0, 4.0]]),
            &s
        )
        .is_err());
}
#[test]
fn excessive_width_never_saturates_native_integer() {
    let b = board();
    let s = spec();
    let mut plan = c("width", "BUS", vec![[2.0, 4.0], [18.0, 4.0]]);
    plan.legs[0].width_mm = 1e100;
    assert!(b.check_candidate(&plan, &s).is_err());
}

#[test]
fn library_via_without_net_rule_authorization_is_rejected() {
    let mut b = board();
    let n = b.net_number("BUS").unwrap();
    let class = b.board.rules.nets.get(n).unwrap().get_net_class();
    let empty = b
        .board
        .rules_mut()
        .via_rules
        .add(fr_engine::rules::ViaRule::empty());
    b.board.rules_mut().net_classes[class].set_via_rule(Some(empty));
    let mut s = spec();
    s.nets[0].allowed_layers.push("Bottom".into());
    s.nets[0].via_budget = Some(1);
    let mut plan = c("unauthorized", "BUS", vec![[2.0, 4.0], [4.0, 4.0]]);
    plan.vias.push(PlanVia {
        net: "BUS".into(),
        point: [4.0, 4.0],
        from_layer: "Top".into(),
        to_layer: "Bottom".into(),
        diameter_mm: 0.5,
        padstack: "VIA".into(),
    });
    assert!(b
        .check_candidate(&plan, &s)
        .unwrap_err()
        .contains("not authorized"));
}
#[test]
fn two_terminal_max_length_rejects_branch_without_touching_original() {
    let b = board();
    let original = b.ses();
    let s = spec();
    let mut plan = c("branch", "BUS", vec![[2.0, 4.0], [18.0, 4.0]]);
    plan.legs.push(RouteLeg {
        net: "BUS".into(),
        layer: "Top".into(),
        points: vec![[10.0, 4.0], [10.0, 7.0]],
        width_mm: 0.15,
        clearance_mm: 0.1,
    });
    let problem = PlannerProblem {
        selection_sets: vec![SelectionSet {
            id: "branch".into(),
            group_id: "bus".into(),
            candidate_ids: vec!["branch".into()],
        }],
        candidates: vec![plan],
        conflicts: vec![],
        capacities: vec![],
    };
    let error = execute_selected(
        &b,
        &s,
        &problem,
        &["branch".into()],
        Instant::now() + Duration::from_secs(5),
    )
    .err()
    .unwrap();
    assert!(error.contains("unsupported"), "{error}");
    assert_eq!(b.ses(), original);
}
#[test]
fn full_cli_route_commits_native_session_without_overwrite() {
    let dir = tempfile::tempdir().unwrap();
    let input = dir.path().join("board.dsn");
    let output = dir.path().join("board.ses");
    std::fs::write(&input, include_bytes!("fixtures/two_chip.dsn")).unwrap();
    let mut s = spec();
    s.input = input.clone();
    s.output = output.clone();
    let config = dir.path().join("route.toml");
    std::fs::write(&config, toml::to_string(&s).unwrap()).unwrap();
    let result = std::process::Command::new(env!("CARGO_BIN_EXE_fastroute_agent"))
        .args(["route", config.to_str().unwrap()])
        .output()
        .unwrap();
    assert!(
        result.status.success(),
        "stdout={} stderr={}",
        String::from_utf8_lossy(&result.stdout),
        String::from_utf8_lossy(&result.stderr)
    );
    let report: serde_json::Value = serde_json::from_slice(&result.stdout).unwrap();
    assert_eq!(report["success"], true);
    assert_eq!(report["output_committed"], true);
    let session = std::fs::read(&output).unwrap();
    let imported = BoardAdapter::load_session(&std::fs::read(&input).unwrap(), &session).unwrap();
    assert_eq!(imported.connectivity()["BUS"].len(), 1);
    let repeated = std::process::Command::new(env!("CARGO_BIN_EXE_fastroute_agent"))
        .args(["route", config.to_str().unwrap()])
        .output()
        .unwrap();
    assert!(!repeated.status.success());
    assert_eq!(session, std::fs::read(output).unwrap());
}

#[test]
fn path_metric_known_length_and_intermediate_via_bridges() {
    let mut b = board();
    let s = spec();
    b.insert_candidate(&c("line", "BUS", vec![[2.0, 4.0], [18.0, 4.0]]), &s)
        .unwrap();
    assert!(
        (fr_agent::execution::two_terminal_path_length(&b, "BUS").unwrap() - 16.0).abs() < 1e-9
    );
}
#[test]
fn path_metric_rejects_same_layer_interior_crossings() {
    let mut b = board();
    let s = spec();
    let mut plan = c("cross", "BUS", vec![[2.0, 2.0], [8.0, 8.0]]);
    plan.legs.push(RouteLeg {
        net: "BUS".into(),
        layer: "Top".into(),
        points: vec![[2.0, 8.0], [8.0, 2.0]],
        width_mm: 0.15,
        clearance_mm: 0.1,
    });
    b.insert_candidate(&plan, &s).unwrap();
    assert!(fr_agent::execution::two_terminal_path_length(&b, "BUS")
        .unwrap_err()
        .contains("interior trace crossings"));
}
#[test]
fn frozen_unfixed_keepout_cannot_be_deleted() {
    let mut b = board();
    let key = b.board.insert_obstacle(
        fr_geom::Area::Shape(fr_geom::Shape::Tile(fr_geom::TileShape::IntBox(
            fr_geom::IntBox::new(8000, 6000, 9000, 7000),
        ))),
        0,
        1,
        fr_engine::ids::FixedState::Unfixed,
    );
    let before = b.clone();
    b.board.remove_item(key);
    assert!(b
        .assert_invariants(&before, &std::collections::BTreeSet::new(), false)
        .is_err());
}

#[test]
fn active_single_pad_net_cannot_be_reported_already_connected() {
    let source = std::str::from_utf8(include_bytes!("fixtures/two_chip.dsn"))
        .unwrap()
        .replace("(net BUS (pins U1-1 U2-1))", "(net BUS (pins U1-1))");
    let b = BoardAdapter::load(source.as_bytes()).unwrap();
    let error = generate_problem(&b, &spec(), Instant::now() + Duration::from_secs(5))
        .err()
        .unwrap();
    assert!(error.contains("requires at least two"), "{error}");
}
#[test]
fn group_dependencies_override_priority_with_stable_ties() {
    let mut s = spec();
    let mut dependent = s.groups[0].clone();
    dependent.id = "later".into();
    dependent.priority = 100;
    dependent.depends_on = vec!["bus".into()];
    s.groups.push(dependent);
    let ordered = fr_agent::execution::ordered_groups(&s).unwrap();
    assert_eq!(
        ordered.iter().map(|g| g.id.as_str()).collect::<Vec<_>>(),
        vec!["bus", "later"]
    );
}

#[test]
fn full_session_replaces_source_routing_instead_of_adding_it() {
    let source = std::str::from_utf8(include_bytes!("fixtures/two_chip.dsn")).unwrap();
    let source=source.trim_end().strip_suffix(')').unwrap().to_owned()+" (wiring (wire (path Top 0.15 2 4 18 4) (net BUS)) (via VIA 10 8 (net OTHER) (type fix))))";
    let b = BoardAdapter::load(source.as_bytes()).unwrap();
    let session = b.ses();
    let imported = BoardAdapter::load_session(source.as_bytes(), &session).unwrap();
    assert_eq!(
        b.board.get_traces().len(),
        imported.board.get_traces().len()
    );
    assert_eq!(
        b.board
            .item_repository()
            .iter()
            .filter(|&k| b.board.item(k).is_via())
            .count(),
        imported
            .board
            .item_repository()
            .iter()
            .filter(|&k| imported.board.item(k).is_via())
            .count()
    );
    assert!(
        (b.board.cumulative_trace_length() - imported.board.cumulative_trace_length()).abs() < 1e-9
    );
    assert_eq!(
        b.connectivity()
            .values()
            .map(|s| s.len())
            .collect::<Vec<_>>(),
        imported
            .connectivity()
            .values()
            .map(|s| s.len())
            .collect::<Vec<_>>()
    );
    assert!(BoardAdapter::load_session(source.as_bytes(), b"(session invalid)").is_err());
    assert!(BoardAdapter::load_session(
        source.as_bytes(),
        b"(session empty (routes (network_out)))"
    )
    .is_err());
}
