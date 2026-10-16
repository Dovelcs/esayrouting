use super::*;

fn restore_fixture() -> (BoardAdapter, BoardAdapter, BoardAdapter, NetNo) {
    let mut baseline = BoardAdapter::load(include_bytes!("../../tests/fixtures/two_chip.dsn"))
        .expect("two_chip board loads");
    crate::awx::board_snapshot::BoardSnapshot::apply_configured_via_drill(&mut baseline, 0.3)
        .expect("the job config supplies the fixture via drill");
    let bus = baseline.net_number("BUS").unwrap();
    let other = baseline.net_number("OTHER").unwrap();

    let mut saved = baseline.clone();
    for (a, b) in [
        ([2.0, 4.0], [3.0, 4.0]),
        ([3.0, 4.0], [6.0, 4.0]),
        ([6.0, 4.0], [7.0, 4.0]),
        ([18.0, 4.0], [17.0, 4.0]),
    ] {
        native_execution::trace(&mut saved, "BUS", "Top", a, b, 0.15)
            .expect("saved BUS trace inserts");
    }
    let via_padstack = saved
        .board
        .library
        .padstacks
        .get_by_name("VIA")
        .expect("fixture via padstack")
        .id;
    let clearance_class = saved
        .board
        .rules
        .net_class_of(bus)
        .get_trace_clearance_class();
    let via_center = saved.point([3.0, 4.0]).unwrap();
    saved.board.insert_via(
        via_padstack,
        via_center,
        &[bus],
        clearance_class,
        FixedState::Unfixed,
        false,
    );

    let mut current = baseline.clone();
    let other_clearance_class = current
        .board
        .rules
        .net_class_of(other)
        .get_trace_clearance_class();
    let foreign = Polyline::from_two_points(
        &current.point([4.5, 3.5]).unwrap(),
        &current.point([4.5, 4.5]).unwrap(),
    );
    let top_layer = current.board.layer_structure.get_no("Top");
    current
        .board
        .insert_trace_without_cleaning(
            foreign,
            top_layer,
            (0.15 * current.units_per_mm / 2.0).round() as i32,
            &[other],
            other_clearance_class,
            FixedState::Unfixed,
        )
        .expect("foreign OTHER trace inserts");
    let foreign_stub_blocker = Polyline::from_two_points(
        &current.point([17.5, 3.5]).unwrap(),
        &current.point([17.5, 4.5]).unwrap(),
    );
    current
        .board
        .insert_trace_without_cleaning(
            foreign_stub_blocker,
            top_layer,
            (0.15 * current.units_per_mm / 2.0).round() as i32,
            &[other],
            other_clearance_class,
            FixedState::Unfixed,
        )
        .expect("foreign copper crosses the right-side pad escape stub");
    (saved, current, baseline, bus)
}

fn trace_geometry(board: &BoardAdapter, key: fr_engine::board::ItemKey) -> ([f64; 2], [f64; 2]) {
    let trace = board.board.item(key).trace();
    let start = trace.polyline().first_corner().to_float();
    let end = trace.polyline().last_corner().to_float();
    let scale = board.units_per_mm;
    (
        [start.x / scale, start.y / scale],
        [end.x / scale, end.y / scale],
    )
}

fn has_segment(board: &BoardAdapter, net: NetNo, a: [f64; 2], b: [f64; 2]) -> bool {
    board
        .board
        .get_traces()
        .into_iter()
        .any(|key| is_segment(board, key, net, a, b, "Top"))
}

fn is_segment(
    board: &BoardAdapter,
    key: fr_engine::board::ItemKey,
    net: NetNo,
    a: [f64; 2],
    b: [f64; 2],
    layer: &str,
) -> bool {
    let item = board.board.item(key);
    if !item.contains_net(net) || item.trace().layer() != board.board.layer_structure.get_no(layer)
    {
        return false;
    }
    let (start, end) = trace_geometry(board, key);
    let near =
        |x: [f64; 2], y: [f64; 2]| (x[0] - y[0]).abs() < 0.001 && (x[1] - y[1]).abs() < 0.001;
    (near(start, a) && near(end, b)) || (near(start, b) && near(end, a))
}

fn copper_counts(board: &BoardAdapter, net: NetNo) -> (usize, usize) {
    let traces = board
        .board
        .get_traces()
        .into_iter()
        .filter(|&key| board.board.item(key).contains_net(net))
        .count();
    let vias = board
        .board
        .get_vias()
        .into_iter()
        .filter(|&key| board.board.item(key).contains_net(net))
        .count();
    (traces, vias)
}

#[test]
fn terminal_restore_keeps_only_safe_pad_escape_stub() {
    let (saved, current, baseline, bus) = restore_fixture();
    let protected = BTreeSet::new();

    let restored = restore_failed_victim(&saved, &current, &baseline, &protected, bus);

    let board = restored
        .board
        .expect("safe terminal stub should be retained");
    assert_eq!(restored.status, "stub");
    assert!(restored.full_restore_error.is_some() || restored.full_invariant_error.is_some());
    assert_eq!(restored.restored_segments, 1);
    assert_eq!(restored.restored_vias, 1);
    assert!(has_segment(&board, bus, [2.0, 4.0], [3.0, 4.0]));
    assert!(!has_segment(&board, bus, [3.0, 4.0], [6.0, 4.0]));
    assert!(!has_segment(&board, bus, [6.0, 4.0], [7.0, 4.0]));
    assert!(!has_segment(&board, bus, [18.0, 4.0], [17.0, 4.0]));

    let stub = board
        .board
        .get_traces()
        .into_iter()
        .find(|&key| is_segment(&board, key, bus, [2.0, 4.0], [3.0, 4.0], "Top"))
        .expect("restored stub trace exists");
    let source_stub = saved
        .board
        .get_traces()
        .into_iter()
        .find(|&key| is_segment(&saved, key, bus, [2.0, 4.0], [3.0, 4.0], "Top"))
        .expect("saved stub trace exists");
    let actual = board.board.item(stub);
    let expected = saved.board.item(source_stub);
    assert_eq!(actual.trace().half_width(), expected.trace().half_width());
    assert_eq!(actual.clearance_class(), expected.clearance_class());
    assert_eq!(actual.trace().layer(), expected.trace().layer());

    let restored_via = board
        .board
        .get_vias()
        .into_iter()
        .map(|key| board.board.item(key))
        .find(|item| item.contains_net(bus))
        .expect("far-end via is retained");
    let expected_via = saved
        .board
        .get_vias()
        .into_iter()
        .map(|key| saved.board.item(key))
        .find(|item| item.contains_net(bus))
        .expect("saved via exists");
    assert_eq!(restored_via.as_via().unwrap().attach_allowed, false);
    assert_eq!(
        restored_via
            .padstack(&board.board)
            .unwrap()
            .get_explicit_drill_radius(),
        expected_via
            .padstack(&saved.board)
            .unwrap()
            .get_explicit_drill_radius()
    );
    assert!(
        fr_engine::drc::NetIncompletes::new(
            &board.board,
            bus,
            &board.board.get_connectable_items(bus),
        )
        .count()
            > 0,
        "restored net remains incomplete"
    );
}

#[test]
fn terminal_restore_is_idempotent_and_preserves_strict_baseline_invariants() {
    let (saved, current, baseline, bus) = restore_fixture();
    let protected = BTreeSet::new();
    let first = restore_failed_victim(&saved, &current, &baseline, &protected, bus);
    let first_board = first.board.expect("first safe stub restore succeeds");
    let first_counts = copper_counts(&first_board, bus);

    let second = restore_failed_victim(&saved, &first_board, &baseline, &protected, bus);
    let second_board = second.board.expect("repeat restore keeps the board");

    assert!(matches!(second.status, "stub" | "full_open"));
    assert_eq!(copper_counts(&second_board, bus), first_counts);
    assert_eq!((first.restored_segments, first.restored_vias), (1, 1));
    assert_eq!((second.restored_segments, second.restored_vias), (0, 0));
    second_board
        .assert_invariants(&baseline, &protected, true)
        .expect("restored geometry satisfies strict baseline invariants");
}
