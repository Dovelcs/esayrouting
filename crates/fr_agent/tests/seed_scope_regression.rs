use std::{
    collections::BTreeSet,
    fs,
    time::{Duration, Instant},
};

use fr_agent::{
    awx::{
        evolution_export::{export_native_seed_bundle, NativeSeedExportInput},
        evolution_native::NativeEvolutionOperators,
        fanout_driver,
        native_execution,
        route_evolution::EvolutionOperators,
    },
    board_adapter::BoardAdapter,
    spec::LoadedSpec,
};

// Reuse the established AWX end-to-end job verbatim so the capture and seed
// exporter exercise the same route configuration as the known-good fixture.
fn e2e_job() -> &'static str {
    let source = include_str!("awx_e2e.rs");
    let start = source.find("const AWX_JOB: &str = r#\"").unwrap()
        + "const AWX_JOB: &str = r#\"".len();
    let end = source[start..].find("\"#;").unwrap() + start;
    &source[start..end]
}

#[test]
fn exported_seed_replays_prior_inactive_net_via_and_rejects_wrong_net_authorization() {
    let temp = tempfile::tempdir().unwrap();
    let board_path = temp.path().join("board.dsn");
    let config_path = temp.path().join("job.toml");
    fs::write(&board_path, include_bytes!("fixtures/two_chip.dsn")).unwrap();
    fs::write(&config_path, e2e_job()).unwrap();
    let mut loaded = LoadedSpec::load(&config_path).unwrap();

    // Capture a real BUS-only pass from the same two-net board. OTHER remains
    // present in the source DSN and is the prior non-active bus in this seed.
    loaded.spec.groups[0].nets.retain(|name| name == "BUS");
    let source = fs::read(&board_path).unwrap();
    let board = BoardAdapter::load(&source).unwrap();
    let deadline = Instant::now() + Duration::from_millis(loaded.spec.execution.budget_ms);
    let execution = fanout_driver::execute(&board, &loaded.spec, deadline).unwrap();
    let capture = execution.seed_captures.first().expect("successful BUS seed capture");
    assert_eq!(capture.names, ["BUS"]);

    // Add two previously routed OTHER vias to the full routed board snapshot;
    // the active BUS plan/context comes untouched from the actual route capture.
    let mut routed = capture.routed.clone();
    native_execution::via(&mut routed, "OTHER", [10.0, 9.0], 0.8, 0.3).unwrap();
    native_execution::via(&mut routed, "OTHER", [12.0, 9.0], 0.8, 0.4).unwrap();
    let output_dir = temp.path().join("native-seed");
    let verdicts = std::collections::BTreeMap::new();
    let exported_manifest = export_native_seed_bundle(NativeSeedExportInput {
        board_dsn: &capture.board_dsn,
        output_dir: &output_dir,
        routed: &routed,
        fanout: &capture.fanout,
        spec: &capture.spec,
        requested_k: capture.requested_k,
        destination: &capture.destination,
        names: &capture.names,
        context: &capture.context,
        flow: capture.flow,
        plan_seed: &capture.plan_seed,
        census: None,
        verdicts: &verdicts,
    })
    .unwrap();

    let manifest: toml::Value = toml::from_str(&fs::read_to_string(&exported_manifest).unwrap()).unwrap();
    assert_eq!(manifest["active_nets"].as_array().unwrap()[0].as_str(), Some("BUS"));
    let prior_other_via = manifest["native_vias"].as_array().unwrap().iter()
        .find(|via| via["padstack"].as_str().unwrap().contains("r400_d300"))
        .expect("export captures the prior generated via");
    let authorized: BTreeSet<_> = prior_other_via["nets"].as_array().unwrap().iter()
        .map(|net| net.as_str().unwrap()).collect();
    assert_eq!(authorized, BTreeSet::from(["OTHER"]));

    let operators = NativeEvolutionOperators::native(loaded.spec.clone(), output_dir.clone());
    let imported = operators
        .import_seed(&exported_manifest, capture.requested_k, &loaded.spec.input, "U2")
        .unwrap()
        .expect("full serialized copper scope imports with prior inactive-net via");
    assert_eq!(
        imported.data.spec.nets.iter().map(|net| net.name.as_str()).collect::<Vec<_>>(),
        ["BUS"]
    );

    let mut swapped = manifest.clone();
    let metadata = swapped["native_vias"].as_array_mut().unwrap();
    let first = metadata.iter().position(|via| via["padstack"].as_str().unwrap().contains("r400_d300")).unwrap();
    let second = metadata.iter().position(|via| via["padstack"].as_str().unwrap().contains("r400_d400")).unwrap();
    let first_name = metadata[first]["padstack"].as_str().unwrap().to_owned();
    let second_name = metadata[second]["padstack"].as_str().unwrap().to_owned();
    metadata[first]["padstack"] = toml::Value::String(second_name);
    metadata[second]["padstack"] = toml::Value::String(first_name);
    fs::write(&exported_manifest, toml::to_string(&swapped).unwrap()).unwrap();
    let error = operators
        .import_seed(&exported_manifest, capture.requested_k, &loaded.spec.input, "U2")
        .err()
        .expect("swapped same-copper via names must fail before SES replay");
    assert!(error.contains("padstack") || error.contains("dimensions"), "{error}");

    let mut wrong_net = manifest;
    let via = wrong_net["native_vias"].as_array_mut().unwrap().iter_mut()
        .find(|via| via["padstack"].as_str().unwrap().contains("r400_d300"))
        .unwrap();
    via["nets"] = toml::Value::Array(vec![toml::Value::String("BUS".into())]);
    fs::write(&exported_manifest, toml::to_string(&wrong_net).unwrap()).unwrap();
    let error = operators
        .import_seed(&exported_manifest, capture.requested_k, &loaded.spec.input, "U2")
        .err()
        .expect("an unrelated net must not authorize the OTHER via");
    assert!(error.contains("SES replay") || error.contains("via"), "{error}");
}
