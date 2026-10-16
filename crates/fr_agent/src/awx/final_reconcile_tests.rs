use super::*;
use crate::awx::{board_snapshot::BoardSnapshot, native_execution};

fn boards() -> (BoardAdapter, BoardAdapter, NetNo, NetNo) {
    let mut source = BoardAdapter::load(include_bytes!("../../tests/fixtures/two_chip.dsn"))
        .expect("two_chip fixture loads");
    BoardSnapshot::apply_configured_via_drill(&mut source, 0.3).expect("configured drill");
    let bus = source.net_number("BUS").unwrap();
    let other = source.net_number("OTHER").unwrap();
    let mut routed = source.clone();
    native_execution::trace(&mut routed, "BUS", "Top", [2.0, 4.0], [18.0, 4.0], 0.15)
        .unwrap();
    native_execution::trace(&mut routed, "OTHER", "Top", [2.0, 5.0], [18.0, 5.0], 0.15)
        .unwrap();
    (source, routed, bus, other)
}

fn score_tuple(board: &BoardAdapter) -> (usize, u32, usize) {
    let score = quality_score(board);
    (score.broken_nets, score.incomplete_connections, score.disconnected_pins)
}

#[test]
fn lap_accepts_mutable_net_reroute_without_quality_regression() {
    let (source, before, bus, _) = boards();
    assert_eq!(score_tuple(&before), (0, 0, 0));
    let mut after = before.clone();
    native_execution::strip_nets(&mut after, &BTreeSet::from(["BUS".to_owned()])).unwrap();
    for (a, b) in [
        ([2.0, 4.0], [6.0, 4.0]),
        ([6.0, 4.0], [6.0, 3.0]),
        ([6.0, 3.0], [14.0, 3.0]),
        ([14.0, 3.0], [14.0, 4.0]),
        ([14.0, 4.0], [18.0, 4.0]),
    ] {
        native_execution::trace(&mut after, "BUS", "Top", a, b, 0.15).unwrap();
    }
    assert_eq!(score_tuple(&after), (0, 0, 0));
    assert_lap_candidate(
        &before,
        &after,
        &source,
        &BTreeSet::new(),
        &BTreeSet::from([bus]),
    )
    .expect("valid mutable BUS reroute is accepted");
}

#[test]
fn lap_rejects_nonmutable_net_copper_change_without_quality_change() {
    let (source, before, bus, _) = boards();
    let mut after = before.clone();
    native_execution::strip_nets(&mut after, &BTreeSet::from(["OTHER".to_owned()])).unwrap();
    native_execution::trace(&mut after, "OTHER", "Top", [2.0, 5.0], [18.0, 5.0], 0.20).unwrap();
    assert_eq!(score_tuple(&before), (0, 0, 0));
    assert_eq!(score_tuple(&after), (0, 0, 0));
    assert!(after.new_violations_since(&before).is_empty());
    assert!(assert_lap_candidate(
        &before,
        &after,
        &source,
        &BTreeSet::new(),
        &BTreeSet::from([bus]),
    )
    .is_err(), "fixed nonmutable OTHER copper must remain invariant");
}

#[test]
fn priority_restore_candidate_rips_only_the_routed_owner_and_keeps_current_unchanged() {
    let (source, _, bus, other) = boards();
    let mut saved = source.clone();
    native_execution::trace(&mut saved, "BUS", "Top", [2.0, 4.0], [18.0, 4.0], 0.15).unwrap();

    let mut current = source.clone();
    for (a, b) in [
        ([2.0, 5.0], [10.0, 5.0]),
        ([10.0, 5.0], [10.0, 3.0]),
        ([10.0, 3.0], [17.0, 3.0]),
        ([17.0, 3.0], [17.0, 5.0]),
        ([17.0, 5.0], [18.0, 5.0]),
    ] {
        native_execution::trace(&mut current, "OTHER", "Top", a, b, 0.15).unwrap();
    }
    let current_ids: BTreeSet<_> = current.board.get_items().into_iter()
        .map(|key| current.board.item(key).id().0).collect();
    assert!(current.new_violations_since(&source).is_empty(), "fixture current board is DRC-clean");
    assert!(fr_engine::drc::NetIncompletes::new(
        &current.board, bus, &current.board.get_connectable_items(bus)
    ).count() > 0);
    let result = prepare_priority_restore(
        &current, &source, &BTreeSet::new(), bus, &saved,
        &BTreeSet::from([other]), &BTreeSet::new(),
    ).unwrap().expect("restore has one routed collision owner");
    assert_eq!(result.victim, bus);
    assert_eq!(result.owners, vec![other]);
    assert!(fr_engine::drc::NetIncompletes::new(
        &result.board.board, bus, &result.board.board.get_connectable_items(bus)
    ).count() == 0, "restored BUS is complete even though OTHER awaits rerouting");
    assert!(result.board.new_violations_since(&current).is_empty());
    assert_eq!(current.board.get_items().into_iter().map(|key| current.board.item(key).id().0).collect::<BTreeSet<_>>(), current_ids);
    assert!(prepare_priority_restore(
        &current, &source, &BTreeSet::new(), bus, &saved,
        &BTreeSet::from([other]), &BTreeSet::from([other]),
    ).unwrap().is_none());
    assert!(prepare_priority_restore(
        &current, &source, &BTreeSet::from(["OTHER".to_owned()]), bus, &saved,
        &BTreeSet::from([other]), &BTreeSet::new(),
    ).unwrap().is_none());
}

#[test]
fn authority_cap_preserves_hint_order_for_thirteen_routed_owners() {
    let (source, mut routed, target, _) = boards();
    let mut owners = Vec::new();
    for index in 0..13 {
        let name = format!("OWNER_{index:02}");
        let owner = routed.board.rules_mut().add_net(&name, 0, false);
        let x = 1.0 + index as f64;
        native_execution::trace(
            &mut routed,
            &name,
            "Bottom",
            [x, 8.0],
            [x + 0.2, 8.0],
            0.15,
        )
        .expect("generated owner copper inserts");
        assert!(source.board.get_traces().into_iter().all(|key|
            !source.board.item(key).contains_net(owner)
        ));
        owners.push(owner);
    }
    let hint_order: Vec<_> = owners.iter().rev().copied().collect();
    let hints = BTreeMap::from([(target, hint_order.clone())]);
    let mut authorized = BTreeSet::new();
    let added = grant_authority(
        &routed,
        &hints,
        &BTreeSet::from([target]),
        &BTreeSet::new(),
        &mut authorized,
    );

    assert_eq!(added, hint_order[..12].to_vec());
    assert_eq!(authorized, hint_order[..12].iter().copied().collect());
    assert!(!authorized.contains(&owners[0]));
}
