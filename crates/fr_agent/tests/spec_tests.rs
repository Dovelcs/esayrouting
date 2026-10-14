use fr_agent::spec::LoadedSpec;
use std::path::Path;
const BASE: &str = include_str!("../../../examples/agent_route/basic.toml");
fn parse(s: &str) -> Result<LoadedSpec, fr_agent::spec::SpecError> {
    LoadedSpec::parse(s, Path::new("/unlikely_to_exist/fr_agent/job.toml"))
}
#[test]
fn valid_and_relative() {
    let s = parse(BASE).unwrap();
    assert_eq!(
        s.spec.input,
        Path::new("/unlikely_to_exist/fr_agent/board.dsn")
    );
}
#[test]
fn unknown_fields_rejected() {
    assert!(parse(&BASE.replace("width_mm", "widht_mm")).is_err());
}
#[test]
fn invalid_numbers() {
    for v in ["nan", "inf", "0.0", "-0.2"] {
        assert!(parse(&BASE.replace("width_mm = 0.2", &format!("width_mm = {v}"))).is_err());
    }
    for v in ["0", "86400001"] {
        assert!(parse(&BASE.replace("budget_ms = 30000", &format!("budget_ms = {v}"))).is_err());
    }
}
#[test]
fn duplicate_and_bad_references() {
    assert!(
        parse(&BASE.replace("nets = [\"SIGNAL\"]", "nets = [\"SIGNAL\", \"SIGNAL\"]")).is_err()
    );
    assert!(parse(&BASE.replace("nets = [\"SIGNAL\"]", "nets = [\"MISSING\"]")).is_err());
}
#[test]
fn dependency_cycle_and_empty_groups() {
    assert!(
        parse(&BASE.replace("priority = 10", "priority = 10\ndepends_on = [\"signal\"]")).is_err()
    );
    assert!(parse(&BASE.replace("nets = [\"SIGNAL\"]", "nets = []")).is_err());
}
#[test]
fn unsupported_hard_constraints() {
    for field in [
        "differential_coupling = true",
        "impedance_ohms = 50.0",
        "cross_net_paths = true",
    ] {
        assert!(parse(&format!("{BASE}\n[advanced]\n{field}\n"))
            .unwrap_err()
            .to_string()
            .contains("unsupported"));
    }
}
#[test]
fn input_output_collision() {
    assert!(parse(&BASE.replace("board_routed.ses", "board.dsn")).is_err());
}
#[test]
fn protected_net_cannot_route() {
    assert!(parse(&BASE.replace(
        "max_length_mm = 50.0",
        "max_length_mm = 50.0\nprotected = true"
    ))
    .is_err());
}
#[test]
fn malformed_channels() {
    for region in [
        "kind = \"rectangle\"\nmin = [1.0, 0.0]\nmax = [0.0, 2.0]",
        "kind = \"polyline\"\npoints = [[0.0, 0.0]]\nwidth_mm = 0.2",
    ] {
        assert!(parse(&format!(
            "{BASE}\n[[channels]]\nid = \"c\"\n[channels.region]\n{region}\n"
        ))
        .is_err());
    }
}
#[test]
fn existing_output_and_aliases_rejected() {
    let dir = std::env::temp_dir().join(format!("fr_agent_spec_{}", std::process::id()));
    std::fs::create_dir_all(&dir).unwrap();
    std::fs::write(dir.join("board.dsn"), "input").unwrap();
    std::fs::write(dir.join("board_routed.ses"), "existing").unwrap();
    assert!(LoadedSpec::parse(BASE, &dir.join("job.toml")).is_err());
    std::fs::remove_file(dir.join("board_routed.ses")).unwrap();
    std::fs::hard_link(dir.join("board.dsn"), dir.join("board_routed.ses")).unwrap();
    assert!(LoadedSpec::parse(BASE, &dir.join("job.toml")).is_err());
    std::fs::remove_file(dir.join("board_routed.ses")).unwrap();
    #[cfg(unix)]
    {
        std::os::unix::fs::symlink("board.dsn", dir.join("board_routed.ses")).unwrap();
        assert!(LoadedSpec::parse(BASE, &dir.join("job.toml")).is_err());
        std::fs::remove_file(dir.join("board_routed.ses")).unwrap();
        std::os::unix::fs::symlink("missing.ses", dir.join("board_routed.ses")).unwrap();
        assert!(LoadedSpec::parse(BASE, &dir.join("job.toml")).is_err());
    }
    std::fs::remove_dir_all(dir).unwrap();
}
#[test]
fn duplicate_ids_and_endpoint_references_rejected() {
    let duplicate = "\n[[nets]]\nname='SIGNAL'\nwidth_mm=0.2\nclearance_mm=0.15\nallowed_layers=['F.Cu']\nvia_budget=0\n";
    assert!(parse(&format!("{BASE}{duplicate}")).is_err());
    let bad = "\n[[endpoint_groups]]\nid='pins'\nendpoints=[{net='MISSING',chip='U1',pad='1'}]\n";
    assert!(parse(&format!("{BASE}{bad}")).is_err());
}
#[test]
fn unused_definitions_and_cross_group_duplicate_pins() {
    let unused = "\n[[channels]]\nid='unused'\n[channels.region]\nkind='rectangle'\nmin=[0.0,0.0]\nmax=[10.0,10.0]\n";
    assert!(parse(&format!("{BASE}{unused}")).is_err());
    let endpoints = "\n[[endpoint_groups]]\nid='a'\nendpoints=[{net='SIGNAL',chip='U1',pad='1'}]\n[[endpoint_groups]]\nid='b'\nendpoints=[{net='SIGNAL',chip='U1',pad='1'}]\n";
    let text = BASE.replace("priority = 10", "priority = 10\nendpoint_groups=['a','b']");
    assert!(parse(&format!("{text}{endpoints}"))
        .unwrap_err()
        .to_string()
        .contains("duplicate or conflicting pad"));
}
#[test]
fn remaining_only_configuration_and_inherited_via_budget() {
    let remaining = "version=1\ninput='board.dsn'\noutput='routed.ses'\nroute_remaining=true\nnets=[]\ngroups=[]\n";
    assert!(parse(remaining).is_ok());
    assert!(parse(&remaining.replace("route_remaining=true", "route_remaining=false")).is_err());
    let inherited = parse(&BASE.replace("via_budget = 0", "")).unwrap();
    assert_eq!(inherited.spec.nets[0].via_budget, None);
    assert_eq!(parse(BASE).unwrap().spec.nets[0].via_budget, Some(0));
}
#[test]
fn remaining_only_can_declare_protected_copper_nets() {
    let protected = "version=1\ninput='board.dsn'\noutput='routed.ses'\nroute_remaining=true\nprotected_nets=['orphan']\ngroups=[]\nnets=[]\n";
    assert!(parse(protected).is_ok());
    assert!(parse(&protected.replace("route_remaining=true", "route_remaining=false")).is_err());
}
