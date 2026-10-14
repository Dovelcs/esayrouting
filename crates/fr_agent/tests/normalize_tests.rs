use fr_agent::dsn_normalize::normalize;
fn board(network: &str, wiring: &str) -> String {
    format!("(PCB test (resolution mil 1000)(structure(layer Top(type signal))(rule(width 10)(clear 6)))(network {network})(wiring {wiring}))")
}
#[test]
fn quoted_names_and_numeric_subnets_preserved() {
    let input = board(
        "(net \"USB D+\" 2(pins))",
        "(wire(path Top 10 0 0 10 10)(net \"USB D+\" 2))",
    );
    let n = normalize(input.as_bytes()).unwrap();
    let d = fr_dsn::Dsn::parse(&n.bytes).unwrap();
    assert_eq!(d.network.nets[0].name, "USB D+");
    assert_eq!(d.network.nets[0].subnet, 2);
    assert_eq!(d.wiring.wires[0].net.as_ref().unwrap().subnet, 2);
    assert!(n.evidence.geometry_and_rule_atoms_unchanged);
}
#[test]
fn exporter_names_classes_and_orphan_copper_preserved() {
    let input = board(
        "(net USB D+(pins))(class USB D+ 'USB D+'(rule(width 10)(clearance 4.02)))",
        "(wire(path Top 10 0 0 10 10)(net copper_only))",
    );
    let n = normalize(input.as_bytes()).unwrap();
    let d = fr_dsn::Dsn::parse(&n.bytes).unwrap();
    assert_eq!(d.network.nets.len(), 2);
    assert_eq!(d.network.classes[0].name, "USB D+");
    assert_eq!(d.network.classes[0].nets, vec!["USB D+"]);
    assert_eq!(n.evidence.implicit_nets_from_copper, vec!["copper_only"]);
    assert!(n.evidence.copper_net_assignments_preserved);
    assert_eq!(d.wiring.wires.len(), 1);
    assert!(n.evidence.geometry_and_rule_atoms_unchanged);
}
#[test]
fn ambiguous_and_empty_classes_rejected() {
    for class in [
        "(class 'USB D+')",
        "(class x 'USB D+)",
        "(class x 'USB'D+')",
        "(class \"\" \"USB D+\")",
    ] {
        assert!(
            normalize(board(&format!("(net USB D+(pins)){class}"), "").as_bytes()).is_err(),
            "{class}"
        );
    }
}
#[test]
fn source_literal_apostrophe_identifier_is_not_empty_name() {
    let n = normalize(board("(net n(pins))(class ''(rule(width 10)))", "").as_bytes()).unwrap();
    assert_eq!(
        fr_dsn::Dsn::parse(&n.bytes).unwrap().network.classes[0].name,
        "''"
    );
}
#[test]
fn case_insensitive_identity_uses_declared_spelling() {
    let input = board(
        "(net CLK(pins))(class timing 'clk'(rule(width 10)))",
        "(wire(path Top 10 0 0 10 10)(net clk))",
    );
    let n = normalize(input.as_bytes()).unwrap();
    let d = fr_dsn::Dsn::parse(&n.bytes).unwrap();
    assert_eq!(d.network.nets.len(), 1);
    assert_eq!(d.wiring.wires[0].net.as_ref().unwrap().name, "CLK");
    assert_eq!(d.network.classes[0].nets, vec!["CLK"]);
    assert!(n.evidence.implicit_nets_from_copper.is_empty());
    assert!(!n.evidence.identifier_case_sensitive);
}
#[test]
fn case_sensitive_board_is_rejected_by_current_engine_boundary() {
    let input = board("(net CLK(pins))(net clk(pins))", "")
        .replace("(PCB test", "(PCB test(parser(case_sensitive on))");
    assert!(normalize(input.as_bytes())
        .unwrap_err()
        .contains("unsupported parser case_sensitive ON"));
}
#[test]
fn second_existing_output_leaves_no_dsn_and_preserves_user_file() {
    let n = normalize(board("(net n(pins))", "").as_bytes()).unwrap();
    let dir = tempfile::tempdir().unwrap();
    let dsn = dir.path().join("normalized.dsn");
    let mapping = dir.path().join("mapping.json");
    std::fs::write(&mapping, b"user data").unwrap();
    assert!(fr_agent::dsn_normalize::write_normalized(&n, &dsn, &mapping).is_err());
    assert!(!dsn.exists());
    assert_eq!(std::fs::read(&mapping).unwrap(), b"user data");
    assert_eq!(std::fs::read_dir(dir.path()).unwrap().count(), 1);
}
#[test]
fn two_outputs_published_once_without_overwrite() {
    let n = normalize(board("(net n(pins))", "").as_bytes()).unwrap();
    let dir = tempfile::tempdir().unwrap();
    let dsn = dir.path().join("normalized.dsn");
    let mapping = dir.path().join("mapping.json");
    fr_agent::dsn_normalize::write_normalized(&n, &dsn, &mapping).unwrap();
    assert_eq!(std::fs::read(&dsn).unwrap(), n.bytes);
    assert!(fr_agent::dsn_normalize::write_normalized(&n, &dsn, &mapping).is_err());
    assert_eq!(std::fs::read_dir(dir.path()).unwrap().count(), 2);
}
