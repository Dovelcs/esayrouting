use super::*;
use crate::awx::{board_snapshot::BoardSnapshot, native_execution};
use std::sync::Arc;

fn source_board() -> BoardAdapter {
    let mut board = BoardAdapter::load(include_bytes!("../../tests/fixtures/two_chip.dsn"))
        .expect("two_chip fixture loads");
    BoardSnapshot::apply_configured_via_drill(&mut board, 0.3).expect("configured drill");
    board
}

fn source_board_with_b_copper() -> BoardAdapter {
    let mut dsn = String::from_utf8(
        include_bytes!("../../tests/fixtures/two_chip.dsn").to_vec(),
    ).unwrap();
    let network_tail = "(net OTHER (pins U1-2 U2-2))";
    dsn = dsn.replace(
        network_tail,
        "(net OTHER (pins U1-2 U2-2)) (net A) (net B) (net C) (net D) (net E)",
    );
    let root_close = dsn.rfind(')').expect("DSN root closes");
    dsn.insert_str(
        root_close,
        "\n (wiring (wire (path Top 0.15 4 8 5 8) (net B) (type protect)))\n",
    );
    let mut board = BoardAdapter::load(dsn.as_bytes()).expect("fixture with source B trace loads");
    BoardSnapshot::apply_configured_via_drill(&mut board, 0.3).expect("configured drill");
    board
}

fn with_five_nets(mut board: BoardAdapter) -> (BoardAdapter, [NetNo; 5]) {
    let a = board.board.rules_mut().add_net("A", 0, false);
    let b = board.board.rules_mut().add_net("B", 0, false);
    let c = board.board.rules_mut().add_net("C", 0, false);
    let d = board.board.rules_mut().add_net("D", 0, false);
    let e = board.board.rules_mut().add_net("E", 0, false);
    // Real native copper on the fixture board gives the custody snapshots valid
    // item ids and exercises the FIFO's overwritten and stale entries.
    native_execution::trace(&mut board, "A", "Top", [2.0, 7.0], [3.0, 7.0], 0.15).unwrap();
    native_execution::trace(&mut board, "B", "Top", [4.0, 7.0], [5.0, 7.0], 0.15).unwrap();
    native_execution::trace(&mut board, "C", "Top", [6.0, 7.0], [7.0, 7.0], 0.15).unwrap();
    native_execution::trace(&mut board, "D", "Top", [8.0, 7.0], [9.0, 7.0], 0.15).unwrap();
    native_execution::trace(&mut board, "E", "Top", [11.0, 7.0], [12.0, 7.0], 0.15).unwrap();
    (board, [a, b, c, d, e])
}

#[test]
fn stale_fifo_duplicate_is_skipped_after_overwritten_custody() {
    let (board, [a, b, c, d, e]) = with_five_nets(source_board());
    let mut state = RecoveryState::default();
    state.record_commit(a, vec![b], &board);
    state.record_commit(b, vec![c], &board);
    state.record_commit(d, vec![b], &board);
    state.record_commit(a, vec![e], &board);

    let first = state.pop_queued_victim().expect("B remains at the first FIFO slot");
    assert_eq!(first.0, b);
    assert_eq!(first.1.board.get_traces().len(), board.board.get_traces().len());
    let second = state.pop_queued_victim().expect("C follows B");
    assert_eq!(second.0, c);
    let third = state.pop_queued_victim().expect("stale B is skipped and E follows");
    assert_eq!(third.0, e);
    assert!(state.pop_queued_victim().is_none());
    for saved in [first.1, second.1, third.1] {
        saved.assert_session_invariants(&board, &BTreeSet::new(), true)
            .expect("saved custody board remains valid");
    }
}

#[test]
fn nested_epoch_eligibility_requires_authority_or_new_copper_and_excludes_protected() {
    let source = source_board_with_b_copper();
    let a = source.net_number("A").unwrap();
    let b = source.net_number("B").unwrap();
    let c = source.net_number("C").unwrap();
    let source_b = source.board.get_traces().into_iter()
        .find(|&key| source.board.item(key).contains_net(b))
        .expect("source B copper exists");
    assert!(source.is_source_protected_copper(source_b));

    let mut epoch = source.clone();
    native_execution::trace(&mut epoch, "A", "Top", [2.0, 6.0], [3.0, 6.0], 0.15).unwrap();
    native_execution::trace(&mut epoch, "B", "Top", [4.0, 6.0], [5.0, 6.0], 0.15).unwrap();
    let mut current = epoch.clone();
    native_execution::trace(&mut current, "C", "Top", [6.0, 6.0], [7.0, 6.0], 0.15).unwrap();
    let protected = BTreeSet::new();
    let authorized = BTreeSet::from([b]);
    let eligible = this_run_eligible(
        &current, &epoch, a, &protected, &BTreeSet::from([a]), true, &authorized,
    ).unwrap();
    assert_eq!(eligible, BTreeSet::from(["B".to_owned(), "C".to_owned()]));
    let source_only_eligible = this_run_eligible(
        &source, &source, a, &protected, &BTreeSet::from([a]), true, &authorized,
    ).unwrap();
    assert!(source_only_eligible.is_empty(), "authorization cannot make source copper detachable");
    let mut stripped = epoch.clone();
    native_execution::strip_nets(&mut stripped, &BTreeSet::from(["B".to_owned()])).unwrap();
    assert!(stripped.board.get_traces().into_iter().any(|key|
        stripped.board.item(key).id().0 == source.board.item(source_b).id().0
    ), "stripping authorized B preserves its source-protected trace");

    let mut nested = RecoveryState::default();
    nested.begin_nested(&epoch, &authorized);
    assert_eq!(nested.routed_result_nets(), &BTreeSet::from([b]));
    nested.note_routed_result(c);
    assert_eq!(nested.routed_result_nets(), &BTreeSet::from([b, c]));
    nested.record_terminal_restore(b, Arc::new(epoch.clone()), "stub");
    assert_eq!(nested.take_priority_candidates().len(), 1);
    assert!(nested.take_priority_candidates().is_empty(), "nested priority sweep is one-shot");
}
