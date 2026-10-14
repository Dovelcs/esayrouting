use std::collections::{BTreeMap, BTreeSet};

use awx_grid::obstacle_map::GridObstacleMap;
use fr_agent::{
    awx::{
        braid_connect::{rung_specs, LadderMode, PairRungRequest, RungEndpoint, RungRequest},
        braid_corridor::{BraidCorridorOptions, CorridorBandContext, CorridorPlan},
        braid_schedule::BraidSchedule,
        corridor::{Point, Spine},
        fab_rules::{EscalationPolicy, FabTier},
        grid_path::{path_to_geometry, GridPathRequest},
        native_pair_router::NativePairRouter,
        pair_connection::{EnvelopeRequest, PairConnectionBackend},
        whole_context::{CopperVia, RoutingRules},
    },
    board_adapter::BoardAdapter,
};
use fr_engine::{
    autoroute::point_router::{route_with_frontier, SourceGridConfig, TrackMarginArg},
    ids::NetNo,
};

fn via_obstacle_detour() -> GridObstacleMap {
    let mut obstacles = GridObstacleMap::new(2);
    // This wall leaves one distant opening. The layer-0 detour costs more
    // than a single-ended via pair but less than the source pair's doubled
    // via penalty.
    for y in -90..=89 {
        obstacles.add_blocked_cell(20, y, 0);
    }
    obstacles
}

fn transition_count(path: &[(i32, i32, u8)]) -> usize {
    path.windows(2)
        .filter(|pair| pair[0].2 != pair[1].2)
        .count()
}

#[test]
fn source_grid_via_units_and_pair_truncation_select_route() {
    let at_tenth = SourceGridConfig::from_source(0.1, 75, 2, &[0, 1], None, None).unwrap();
    let at_twentieth = SourceGridConfig::from_source(0.05, 75, 2, &[0, 1], None, None).unwrap();
    let at_seventh = SourceGridConfig::from_source(0.07, 75, 2, &[0, 1], None, None).unwrap();
    assert_eq!(at_tenth.via_cost, 75_000);
    assert_eq!(at_twentieth.via_cost, 150_000);
    assert_eq!(at_seventh.via_cost, 107_142);

    // The source diff-pair router doubles the already-truncated single-via
    // cost. At 0.07 mm this is 214284; scaling a 150-step input first would
    // produce 214285 and changes the cost contract by one unit.
    let pair_via_cost = at_seventh.via_cost * 2;
    let pair_via_cost_tenth = at_tenth.via_cost * 2;
    let truncate_after_doubling = SourceGridConfig::from_source(0.07, 150, 2, &[0, 1], None, None)
        .unwrap()
        .via_cost;
    assert_eq!(pair_via_cost, 214_284);
    assert_eq!(truncate_after_doubling, 214_285);
    assert_ne!(pair_via_cost, truncate_after_doubling);

    let obstacles = via_obstacle_detour();
    let route = |config: &SourceGridConfig| {
        route_with_frontier(
            &config.router(),
            &obstacles,
            vec![(0, 0, 0)],
            vec![(40, 0, 0)],
            1_000_000,
            TrackMarginArg::Scalar(0.0),
            None,
            None,
        )
        .0
        .expect("source GridRouter should find the open-layer route")
    };
    let single = route(&at_tenth);
    let mut pair_cost_config = at_tenth.clone();
    pair_cost_config.via_cost = pair_via_cost_tenth;
    let pair = route(&pair_cost_config);
    assert!(
        transition_count(&single) > 0,
        "single-ended cost should choose two vias"
    );
    assert_eq!(
        transition_count(&pair),
        0,
        "doubled pair cost should take the detour"
    );
}

const THRU_BOARD: &str = r#"(pcb through_pad.dsn
 (resolution mm 1000) (unit mm)
 (structure
  (layer Top (type signal)) (layer Bottom (type signal)) (via VIA)
  (boundary (rect pcb 0 0 20 10))
  (rule (width 0.15) (clearance 0.1)))
 (library
  (image CHIP (pin PTH 1 0 0))
  (padstack PTH (shape (rect Top -0.3 -0.3 0.3 0.3))
               (shape (rect Bottom -0.3 -0.3 0.3 0.3)))
  (padstack VIA (shape (circle Top 0.5)) (shape (circle Bottom 0.5))))
 (placement (component CHIP (place U1 2 4 front 0) (place U2 18 4 front 0)))
 (network (net BUS (pins U1-1 U2-1))))"#;

#[test]
fn path_adapter_maps_subset_layer_indices_to_bottom_and_top() {
    let board = BoardAdapter::load(THRU_BOARD.as_bytes()).unwrap();
    let net = board.net_number("BUS").unwrap();
    let route_layers = vec!["Bottom".to_owned(), "Top".to_owned()];
    let via_candidates: Vec<_> = board.board.rules.via_infos.iter().collect();
    let stack = via_candidates
        .iter()
        .map(|id| &board.board.rules.via_infos[*id])
        .find_map(|info| {
            let stack = board.board.library.padstacks.get(info.get_padstack())?;
            (stack.from_layer() == 0 && stack.to_layer() == 1).then_some(stack)
        })
        .expect("fixture must expose one full-board via");
    let via_size_mm = (0..board.board.layer_count())
        .filter_map(|layer| stack.get_shape(layer))
        .map(|shape| shape.bounding_box().max_width() as f64 / board.units_per_mm)
        .fold(0.0, f64::max);
    // Fixture-derived dimensions select a native via candidate; this does
    // not claim that the DSN supplies measured physical drill evidence.
    let via_drill_mm = 2.0 * stack.get_drill_radius() / board.units_per_mm;
    let path = [(20, 40, 0), (21, 40, 0), (22, 40, 1), (23, 40, 1)];
    let geometry = path_to_geometry(GridPathRequest {
        path: &path,
        grid_step_board: 100,
        route_layers: &route_layers,
        start: &RungEndpoint {
            point: [2.0, 4.0],
            layer: "Bottom".into(),
        },
        target: &RungEndpoint {
            point: [2.3, 4.0],
            layer: "Top".into(),
        },
        board: &board,
        net,
        via_candidates: &via_candidates,
        via_size_mm,
        via_drill_mm,
        hole_to_hole_clearance_mm: 0.0,
        existing_vias: &[],
        via_refit_rungs: &[],
        registered_via_sizes: &BTreeMap::new(),
        through_hole_positions: &BTreeSet::new(),
    })
    .unwrap();
    assert_eq!(geometry.vias.len(), 1);
    assert_eq!(geometry.vias[0].from_layer, "Bottom");
    assert_eq!(geometry.vias[0].to_layer, "Top");
    assert!(geometry
        .segments
        .iter()
        .any(|segment| segment.layer == "Bottom"));
    assert!(geometry
        .segments
        .iter()
        .any(|segment| segment.layer == "Top"));
}

fn dummy_corridor() -> CorridorPlan {
    let spine = Spine::new(&[[0.0, 5.0], [20.0, 5.0]]).unwrap();
    let options = BraidCorridorOptions {
        track_width: 0.15,
        clearance: 0.1,
        half_sep: 0.15,
        via_size: 1.0,
        pair_gap: 0.1,
        lane_pitch: 0.5,
        exit_pitch: 0.5,
        grid: 0.1,
        pairs: false,
        block_push: 0,
        pages_first: false,
        pages_siders: 0,
        slope_pitch: false,
        branch: false,
        pair_dive_extra: 0.0,
        jog_via: 0.0,
        approach_reserve: 0.0,
        end_keep: 0.0,
    };
    let schedule = BraidSchedule {
        launch: Vec::new(),
        target: Vec::new(),
        trank: BTreeMap::new(),
        lidx: BTreeMap::new(),
        tooth_layer: BTreeMap::new(),
        page: BTreeMap::new(),
        b_page: Vec::new(),
        swimmers: Vec::new(),
        divers: BTreeSet::new(),
        birth_b: BTreeSet::new(),
        priority: Vec::new(),
        serial_order: Vec::new(),
        exact_pages_used: false,
    };
    let band = CorridorBandContext {
        index: 0,
        members: Vec::new(),
        spine: spine.clone(),
        options,
        schedule: None,
        teeth: BTreeMap::new(),
        stubs: BTreeMap::new(),
        st: BTreeMap::new(),
        se: BTreeMap::new(),
        launch: Vec::new(),
        target: Vec::new(),
        launch_o: BTreeMap::new(),
        target_o: BTreeMap::new(),
        s0: 0.0,
        s1: 20.0,
        l_free: 0.0,
        heads_l: Vec::new(),
        joiners: Vec::new(),
        heads_e: Vec::new(),
        siders: Vec::new(),
        far_exit: BTreeSet::new(),
        is_branch: false,
        wrap: false,
        keep_order: false,
        pin_target: BTreeSet::new(),
        join_side: BTreeMap::new(),
        exit_side: BTreeMap::new(),
        join_block: BTreeMap::new(),
        join_leg_s: BTreeMap::new(),
        exit_block: BTreeMap::new(),
        exit_leg_s: BTreeMap::new(),
        req: BTreeMap::new(),
        back_windows: BTreeMap::new(),
        mid: BTreeMap::new(),
        legs: BTreeMap::new(),
        jogs: BTreeMap::new(),
        mid_xy: BTreeMap::new(),
        lanes: BTreeMap::new(),
        req_xy: BTreeMap::new(),
        leg_layer: BTreeMap::new(),
        leg_split: BTreeMap::new(),
        layer_profiles: BTreeMap::new(),
        swim_changes: BTreeMap::new(),
        islands_by_layer: BTreeMap::new(),
        islands: BTreeMap::new(),
        hops: BTreeMap::new(),
        dive_s_plan: BTreeMap::new(),
        pair_names: BTreeSet::new(),
        pair_members: BTreeMap::new(),
        pair_ends: BTreeMap::new(),
        pair_layers: BTreeMap::new(),
        source_paths: BTreeMap::new(),
        all_ends: BTreeMap::new(),
        tooth_dir: BTreeMap::new(),
        stub_dir: BTreeMap::new(),
        slope: BTreeMap::new(),
        tooth_layer: BTreeMap::new(),
        dest_layer: BTreeMap::new(),
        byname: BTreeMap::new(),
        copper_layers: vec!["Top".into(), "Bottom".into()],
        via_drill: 0.9,
    };
    CorridorPlan {
        index: 0,
        members: Vec::new(),
        spine,
        launch: Vec::new(),
        target: Vec::new(),
        schedule,
        lanes: BTreeMap::new(),
        mid: BTreeMap::new(),
        required: BTreeMap::new(),
        back_windows: BTreeMap::new(),
        exit_leg_layer: BTreeMap::new(),
        layer_profiles: BTreeMap::new(),
        swim_changes: BTreeMap::new(),
        islands: BTreeMap::new(),
        joiners: Vec::new(),
        siders: Vec::new(),
        s0: 0.0,
        s1: 20.0,
        band,
    }
}

fn pair_test_request(
    board: BoardAdapter,
    virtual_tracks: Vec<fr_agent::awx::whole_context::ReservationTube>,
    virtual_vias: Vec<CopperVia>,
) -> RungRequest {
    let p = board.net_number("BUS").unwrap();
    let n = board.net_number("OTHER").unwrap();
    let mut net_via_sizes = BTreeMap::new();
    net_via_sizes.insert(p, [1.0, 0.9]);
    net_via_sizes.insert(n, [1.0, 0.9]);
    let pair = PairRungRequest {
        positive_net: "BUS".into(),
        positive_net_id: p,
        negative_net: "OTHER".into(),
        negative_net_id: n,
        start_points: [[9.5, 8.0], [9.5, 8.2]],
        end_points: [[10.5, 8.0], [10.5, 8.2]],
        start_layer: "Top".into(),
        end_layer: "Top".into(),
        negative_start_layer: "Top".into(),
        negative_end_layer: "Top".into(),
        gap: 0.1,
        lead_mm: 0.15,
        allow_cross: false,
        start_direction: None,
        end_direction: None,
        start_connectors: None,
        end_connectors: None,
        a_given: None,
        b_given: None,
        x_given: None,
    };
    let rules = RoutingRules {
        grid_step: 0.1,
        track_width: 0.15,
        clearance: 0.1,
        via_size: 1.0,
        via_drill: 0.9,
        pair_gap: 0.1,
        lane_pitch: 0.5,
        lane_min: 0.2,
        min_turning_radius: 0.0,
        max_turn_angle: 180.0,
        max_setback_angle: 45.0,
        routing_clearance_margin: 1.0,
        board_edge_clearance: 0.0,
        hole_to_hole_clearance: 0.0,
        fan_clearance: 0.0,
        geometry_pair_room: false,
        geometry_pair_turn_room_steps: None,
        same_net_pad_clearance: 0.0,
        net_via_sizes,
    };
    RungRequest {
        board,
        net: "BUS".into(),
        net_id: p,
        stage: "adapter-test".into(),
        start: RungEndpoint {
            point: [9.5, 8.0],
            layer: "Top".into(),
        },
        end: RungEndpoint {
            point: [10.5, 8.0],
            layer: "Top".into(),
        },
        b_alternatives: Vec::new(),
        pair: Some(pair),
        spec: rung_specs(LadderMode::Open)[0],
        band: None,
        band_slack: 0.0,
        window_points: None,
        virtual_tracks,
        virtual_vias,
        free_via_pads: Vec::new(),
        same_net_pads: Vec::new(),
        existing_vias: Vec::new(),
        paste_pads: BTreeMap::new(),
        rules,
        fabrication: FabTier::Standard,
        escalation: EscalationPolicy::Off,
        fabrication_overrides: BTreeMap::new(),
        fab_clearance_floor_mm: 0.0,
        fab_track_floor_mm: 0.0,
        may_narrow: false,
        fab_via_rungs: Vec::new(),
        max_iterations: 20_000,
        route_layers: vec!["Top".into()],
        deadline: None,
    }
}

#[test]
fn pair_envelope_honors_virtual_via_hard_block() {
    let board = BoardAdapter::load(include_bytes!("fixtures/two_chip.dsn")).unwrap();
    let clear = pair_test_request(board.clone(), Vec::new(), Vec::new());
    let envelope = |virtual_size| EnvelopeRequest {
        start: [9.5, 8.0],
        end: [10.5, 8.0],
        start_layer: "Top".into(),
        end_layer: "Top".into(),
        positive_net_id: board.net_number("BUS").unwrap(),
        negative_net_id: board.net_number("OTHER").unwrap(),
        envelope_track_width: 0.3,
        envelope_via_size: 1.0,
        envelope_via_drill: 0.9,
        virtual_size,
        // Keep the route centerline and its 0.25 mm envelope clear of the
        // adapter's synthetic window fence in the unblocked control case.
        margin: 1.0,
        band_slack: 0.0,
        window_points: Vec::new(),
        band_restrictions: Vec::new(),
    };
    let e = envelope((0.15, 1.0));
    let mut router = NativePairRouter::new(&clear).unwrap();
    assert!(router.route_envelope(&clear, &e).unwrap().is_some());

    let virtual_via = CopperVia {
        net: "reservation".into(),
        net_id: -99,
        point: [10.0, 8.0],
        diameter: 5.0,
        drill: 1.0,
        layers: vec!["Top".into(), "Bottom".into()],
    };
    let blocked = pair_test_request(board.clone(), Vec::new(), vec![virtual_via]);
    let mut router = NativePairRouter::new(&blocked).unwrap();
    assert!(router
        .route_envelope(&blocked, &envelope((0.15, 5.0)))
        .unwrap()
        .is_none());
}
