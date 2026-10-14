use fr_agent::board_adapter::BoardAdapter;
use std::collections::BTreeSet;

fn replace_via_padstack(session: &str, old_name: &str, new_name: &str) -> (String, usize) {
    fn visit(expr: &mut fr_dsn::sexpr::Sexpr, old_name: &str, new_name: &str) -> usize {
        let fr_dsn::sexpr::Sexpr::List(list) = expr else {
            return 0;
        };
        let mut changed = 0;
        if list.is("via") {
            if let Some(fr_dsn::sexpr::Sexpr::Atom(name)) = list.items.get_mut(1) {
                if name.text == old_name {
                    name.text = new_name.to_string();
                    changed += 1;
                }
            }
        }
        for child in &mut list.items {
            changed += visit(child, old_name, new_name);
        }
        changed
    }

    let mut expressions = fr_dsn::sexpr::parse(session.as_bytes()).unwrap();
    let changed = expressions
        .iter_mut()
        .map(|expr| visit(expr, old_name, new_name))
        .sum();
    let serialized = expressions
        .iter()
        .map(|expr| format!("{expr:?}"))
        .collect::<Vec<_>>()
        .join("\n");
    (serialized, changed)
}

fn source_with_original_copper() -> String {
    let source = String::from_utf8(include_bytes!("fixtures/two_chip.dsn").to_vec()).unwrap();
    let source = source.trim_end();
    let source = source.strip_suffix(')').unwrap();
    format!(
        "{source}\n (wiring\n  (wire (path Top 0.15 2 4 18 4) (net BUS) (type fix))\n  (wire (path Top 0.15 2 3 18 3) (net BUS) (type fix))\n  (wire (path Top 0.15 10 1 10 9) (net OTHER) (type fix))\n )\n)\n"
    )
}

fn source_with_native_via() -> String {
    let source = String::from_utf8(include_bytes!("fixtures/two_chip.dsn").to_vec()).unwrap();
    let source = source
        .replace(
            "(via VIA)",
            "(via awx_via_r1000_d1000 awx_via_r1000_d1200)",
        )
        .replace(
            "(padstack VIA (shape (circle Top 0.5)) (shape (circle Bottom 0.5)))",
            "(padstack awx_via_r1000_d1000 (shape (circle Top 2.0)) (shape (circle Bottom 2.0))) (padstack awx_via_r1000_d1200 (shape (circle Top 2.0)) (shape (circle Bottom 2.0)))",
        );
    let source = source.trim_end().strip_suffix(')').unwrap();
    format!(
        "{source}\n (wiring (via awx_via_r1000_d1000 2 4 (net BUS) (type protect)) (via awx_via_r1000_d1200 18 4 (net BUS) (type protect)))\n)\n"
    )
}

#[test]
fn ses_replay_preserves_fixed_copper_multiplicity_and_inherited_violations() {
    let source = source_with_original_copper();
    let before = BoardAdapter::load(source.as_bytes()).unwrap();
    let protected = BTreeSet::from(["BUS".to_string()]);
    let original_protected_count = before
        .board
        .item_repository()
        .iter()
        .filter(|&key| before.is_source_protected_copper(key))
        .filter(|&key| {
            let item = before.board.item(key);
            item.net_numbers().iter().any(|&net| {
                before
                    .board
                    .rules
                    .nets
                    .get(net)
                    .is_some_and(|record| record.name == "BUS")
            })
        })
        .count();
    assert_eq!(original_protected_count, 2);
    assert!(!before.session_violations().is_empty());

    let replayed = BoardAdapter::load_session(source.as_bytes(), &before.ses()).unwrap();
    replayed
        .assert_session_invariants(&before, &protected, true)
        .unwrap();
    let replayed_protected_count = replayed
        .board
        .item_repository()
        .iter()
        .filter(|&key| replayed.is_source_protected_copper(key))
        .filter(|&key| {
            let item = replayed.board.item(key);
            item.net_numbers().iter().any(|&net| {
                replayed
                    .board
                    .rules
                    .nets
                    .get(net)
                    .is_some_and(|record| record.name == "BUS")
            })
        })
        .count();
    assert_eq!(replayed_protected_count, 2);
}

#[test]
fn replay_can_enrich_unknown_source_via_drill_with_exact_candidate_metadata() {
    let source = source_with_native_via();
    let before = BoardAdapter::load(source.as_bytes()).unwrap();
    assert_eq!(before.board.get_vias().len(), 2);
    let serialized = String::from_utf8(before.ses()).unwrap();
    assert_eq!(
        replace_via_padstack(&serialized, "awx_via_r1000_d1000", "awx_via_r1000_d1000").1,
        1
    );
    assert_eq!(
        replace_via_padstack(&serialized, "awx_via_r1000_d1200", "awx_via_r1000_d1200").1,
        1
    );
    let candidates = vec![("BUS".to_string(), 2.0, 1.0), ("BUS".to_string(), 2.0, 1.2)];
    let replayed = BoardAdapter::load_session_with_via_candidates(
        source.as_bytes(),
        &before.ses(),
        &candidates,
    )
    .unwrap();
    replayed
        .assert_session_invariants(&before, &BTreeSet::new(), true)
        .unwrap();

    let (altered, changed_count) =
        replace_via_padstack(&serialized, "awx_via_r1000_d1000", "awx_via_r1000_d1200");
    assert_eq!(changed_count, 1);
    assert_ne!(altered, serialized);
    assert!(BoardAdapter::load_session_with_via_candidates(
        source.as_bytes(),
        altered.as_bytes(),
        &candidates,
    )
    .is_err());
}
