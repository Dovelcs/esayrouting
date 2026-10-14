use fr_agent::{awx::native_execution, board_adapter::BoardAdapter};

#[test]
fn native_seed_via_metadata_authorizes_padstack_before_session_replay() {
    let source = include_bytes!("fixtures/two_chip.dsn");
    let mut routed = BoardAdapter::load(source).expect("fixture board loads");
    native_execution::via(&mut routed, "BUS", [8.0, 6.0], 0.8, 0.3)
        .expect("native via materializes");
    let session = routed.ses();

    assert!(
        BoardAdapter::load_session(source, &session).is_err(),
        "an SES-created native padstack must not be trusted without manifest metadata"
    );
    let replayed = BoardAdapter::load_session_with_via_candidates(
        source,
        &session,
        &[("BUS".into(), 0.8, 0.3)],
    )
    .expect("explicit net, copper diameter, and drill authorize replay");
    assert_eq!(
        replayed.board.get_vias().len(),
        routed.board.get_vias().len()
    );
    let stack = replayed
        .board
        .get_vias()
        .into_iter()
        .find_map(|key| {
            let item = replayed.board.item(key);
            item.padstack(&replayed.board)
                .filter(|stack| stack.name.starts_with("awx_via_"))
        })
        .expect("replayed native padstack");
    assert!((2.0 * stack.get_drill_radius() / replayed.units_per_mm - 0.3).abs() < 0.001);
}
