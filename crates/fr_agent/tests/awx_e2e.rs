use std::{collections::BTreeSet, fs, path::Path};

use fr_agent::{
    awx::{evolution_native::NativeEvolutionOperators, route_evolution::EvolutionOperators},
    board_adapter::BoardAdapter,
    execution,
    spec::LoadedSpec,
};

const AWX_JOB: &str = r#"
version = 1
route_remaining = false
input = "board.dsn"
output = "routed.ses"

[[nets]]
name = "BUS"
width_mm = 0.15
clearance_mm = 0.1
allowed_layers = ["Top"]
via_budget = 0
max_length_mm = 30.0

[[nets]]
name = "OTHER"
width_mm = 0.15
clearance_mm = 0.1
allowed_layers = ["Top"]
via_budget = 0
max_length_mm = 30.0

[[groups]]
id = "two_chip_bus"
nets = ["BUS", "OTHER"]

[groups.awx]
source = "U1"
destination = "U2"
routing_layers = ["Top", "Bottom"]
pairs = []
fanout_rounds = 1
geometry_rounds = 1

[execution]
threads = 1
budget_ms = 60000
seed = 17

[awx]
via_drills = { VIA = 0.9 }
fabrication = "standard"
escalation = "off"
seed_export = { directory = "native-seeds" }

[[awx.components]]
reference = "U1"
kind = "bga"
[[awx.components.pads]]
native_ref = "U1.1"
number = "1"
shape = "rect"
rect_rotation = 0.0
[[awx.components.pads]]
native_ref = "U1.2"
number = "2"
shape = "rect"
rect_rotation = 0.0

[[awx.components]]
reference = "U2"
kind = "bga"
[[awx.components.pads]]
native_ref = "U2.1"
number = "1"
shape = "rect"
rect_rotation = 0.0
[[awx.components.pads]]
native_ref = "U2.2"
number = "2"
shape = "rect"
rect_rotation = 0.0

[awx.rules]
grid_step = 0.1
track_width = 0.15
clearance = 0.1
via_size = 1.0
via_drill = 0.9
pair_gap = 0.1
lane_pitch = 0.6
lane_min = 0.2
min_turning_radius = 0.0
max_turn_angle = 180.0
max_setback_angle = 45.0
routing_clearance_margin = 1.0
board_edge_clearance = 0.1
hole_to_hole_clearance = 0.2
fan_clearance = 0.1
"#;

#[test]
fn awx_execution_exports_connected_ses_and_importable_seed_bundle() {
    let temp = tempfile::tempdir().unwrap();
    let board_path = temp.path().join("board.dsn");
    let config_path = temp.path().join("job.toml");
    let source = include_bytes!("fixtures/two_chip.dsn");
    fs::write(&board_path, source).unwrap();
    fs::write(&config_path, AWX_JOB).unwrap();
    let before = fs::read(&board_path).unwrap();
    let loaded = LoadedSpec::load(&config_path).unwrap();

    let report = execution::run(&loaded, false).unwrap();
    assert!(report.success, "AWX run failed: {report:?}");
    assert!(report.output_committed);
    assert_eq!(
        fs::read(&board_path).unwrap(),
        before,
        "input DSN must remain unchanged"
    );

    let original = BoardAdapter::load(&before).unwrap();
    let session = fs::read(&loaded.spec.output).unwrap();
    let routed = BoardAdapter::load_session(&before, &session).unwrap();
    assert!(routed
        .violations()
        .difference(&original.violations())
        .next()
        .is_none());
    for name in ["BUS", "OTHER"] {
        let net = routed.net_number(name).unwrap();
        assert_eq!(
            fr_engine::drc::NetIncompletes::new(
                &routed.board,
                net,
                &routed.board.get_connectable_items(net),
            )
            .count(),
            0,
            "{name} must be connected in the exported native session"
        );
    }

    let manifests = report
        .awx_seed_manifests
        .as_ref()
        .expect("seed export enabled");
    assert_eq!(manifests.len(), 1);
    let manifest_path = &manifests[0];
    let manifest_bytes = fs::read(manifest_path).unwrap();
    let manifest_text = std::str::from_utf8(&manifest_bytes).unwrap();
    let manifest: toml::Value = toml::from_str(manifest_text).unwrap();
    assert_eq!(manifest["format"].as_str(), Some("fastroute-native-seed-v2"));
    let active: BTreeSet<_> = manifest["active_nets"]
        .as_array()
        .unwrap()
        .iter()
        .map(|name| name.as_str().unwrap().to_owned())
        .collect();
    assert_eq!(
        active,
        BTreeSet::from(["BUS".to_owned(), "OTHER".to_owned()])
    );
    assert!(manifest["runtime_scope"]["groups"]
        .as_array()
        .unwrap()
        .iter()
        .any(|g| g["id"].as_str() == Some("two_chip_bus")));

    let seed_base = manifest_path.parent().unwrap_or(Path::new("."));
    let operators = NativeEvolutionOperators::native(loaded.spec.clone(), seed_base.to_owned());
    let world = operators
        .import_seed(manifest_path, 2, &loaded.spec.input, "U2")
        .unwrap()
        .expect("exported seed should import as a native evolution world");
    assert_eq!(world.data.requested_k, 2);
    assert_eq!(world.data.destination, "U2");
    let imported_names: BTreeSet<_> = world
        .data
        .spec
        .nets
        .iter()
        .map(|net| net.name.clone())
        .collect();
    assert_eq!(imported_names, active);
    let imported_groups: BTreeSet<_> = world
        .data
        .spec
        .groups
        .iter()
        .map(|group| group.id.clone())
        .collect();
    assert!(imported_groups.contains("two_chip_bus"));
    assert_eq!(
        world
            .data
            .plan_ends
            .keys()
            .cloned()
            .collect::<BTreeSet<_>>(),
        active
    );
    assert_eq!(
        world.data.pack_sidecar["lanes"]
            .as_object()
            .unwrap()
            .keys()
            .cloned()
            .collect::<BTreeSet<_>>(),
        active,
    );
}
